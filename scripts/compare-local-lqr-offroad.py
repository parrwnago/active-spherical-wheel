"""Original lab local LQR versus the current off-road controller, without editing either.

python scripts/compare-local-lqr-offroad.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from types import FunctionType, SimpleNamespace
import time

import mujoco
import numpy as np

from spherical_wheel import offroad_experiments as experiments
from spherical_wheel.contact_sensing import measure_contact
from spherical_wheel.experiments import rolling_feedforward
from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.mobility import MobilityPath, terrain_for, evaluate, BOUNDS, CASES, TARGETS, write_json
from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad import TerrainPlant
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import Workspace, save_run
from spherical_wheel.offroad_analysis import plot_run
from spherical_wheel.offroad_view import render_replay


class LocalLQRAdapter(OffroadController):
    """Original lab command law and gain selector, adapted only to the experiment API.

    Flat equilibrium designs, raw full-state error, original acceleration lean,
    rolling feedforward, no heading transform, no slope compensation, no
    synchronization. Contact sensing inherited here is used only for logs.
    """
    path = None

    def reset(self, phase=None):
        super().reset(phase)
        self.lab = None  # Build the path schedule after place_on_ground finishes.

    def _prepare(self):
        def local_design(phase):
            point = self.design.local(phase)
            contacts = point['equilibrium']['contacts']
            load = {kind: sum(c['normal_force_N'] for c in contacts
                             if any(n.startswith(kind+'_') for n in c['geoms']))
                    for kind in ('shell', 'belt')}
            return dict(point, regime=max(load, key=load.get))

        initial = local_design(self.ordinary['phase'])
        lab = SimpleNamespace(plant=self.plant, config=self.plant.config, selected=initial,
                              schedule=[initial], failed_gain_points=[], _design=local_design)
        # The lab plans a phase grid from x travel. Extend it to both extrema
        # for a closed path, using the original planner and its rejection rules.
        x = np.array([self.path(t)[0][0] for t in np.linspace(
            self.path.start_s, self.path.start_s+self.path.duration_s, 4001)])
        points = {initial['phase']: initial}
        for distance in (x.min()-self.path.origin[0], x.max()-self.path.origin[0]):
            SimulationLab._prepare_path_schedule(lab, 'line', float(distance))
            points.update({p['phase']: p for p in lab.schedule})
        lab.schedule = sorted(points.values(), key=lambda p: p['phase'])
        self.lab = lab

    def command(self, xy, velocity, acceleration):
        if self.lab is None:
            self._prepare()
        SimulationLab._select_gain(self.lab)
        point = self.lab.selected
        eq, design = point['equilibrium'], point['design']
        qref = eq['qpos'].copy()
        qref[:2] = xy
        qref[7:10] = self.nominal_joints
        delta = np.zeros(self.plant.model.nv)
        gravity = abs(self.plant.config['simulation']['gravity_m_s2'][2])
        delta[3:5] = [-np.arctan2(acceleration[1], gravity), np.arctan2(acceleration[0], gravity)]
        mujoco.mj_integratePos(self.plant.model, qref, delta, 1.)
        vref, feedforward = rolling_feedforward(self.plant, np.asarray(velocity))
        error = self.plant.error(qref, vref)
        command = eq['u']+feedforward-design['K']@error
        applied = np.clip(command, -self.plant.bounds, self.plant.bounds)
        measurement = measure_contact(self.plant)
        estimate = self.estimator.update(measurement, self.plant.dt)
        quat = self.plant.data.qpos[3:7]
        yaw = float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]), 1-2*(quat[2]**2+quat[3]**2)))
        self._vref = vref
        return applied, dict(measurement=measurement, estimate=estimate, requested=command,
                             reference_quat=qref[3:7], support_torque=np.zeros(5),
                             effective_phase_rad=point['phase'], yaw_rad=yaw, drive_fade=1.,
                             reference_qpos=qref, reference_qvel=vref, error=error,
                             sync_weights=np.zeros(5))

    def after_step(self):
        self.nominal_joints += self.plant.dt*self._vref[6:9]


def local_run(workspace, terrain, path, phase=.3):
    # Bind an independent copy of the existing runner to this controller
    # factory. No module global, source file or other process is mutated.
    class BoundLocalLQR(LocalLQRAdapter):
        pass
    BoundLocalLQR.path = path
    created = []

    def factory(*args, **kwargs):
        controller = BoundLocalLQR(*args, **kwargs)
        created.append(controller)
        return controller

    original = experiments.run_offroad
    runner = FunctionType(original.__code__, dict(original.__globals__, OffroadController=factory),
                          original.__name__, original.__defaults__, original.__closure__)
    log, summary = runner(workspace, terrain, path, OffroadParams(mode='flat_baseline', synchronize=False), phase=phase)
    summary['controller'] = 'local_lqr'
    summary['parameters']['mode'] = 'local_lqr'
    lab = created[0].lab
    summary['local_lqr_schedule'] = {
        'phase_rad': [p['phase'] for p in lab.schedule],
        'regimes': [p['regime'] for p in lab.schedule],
        'rejected_points': lab.failed_gain_points,
        'rule': 'Unmodified SimulationLab._select_gain; original planner extended to both path x extrema.'}
    return log, summary


def update_report(output, results):
    lines = ['# Original local LQR versus off-road controller', '',
             'Matched seed 7, start phase 0.30 rad, nominal speed, 1 ms physics step. Controllers are not retuned.', '',
             'Local LQR uses the original lab gain selector and command law. Its original x-travel gain planner is extended to both x extrema of each closed path. No slope lean, holding torque, heading-frame correction or incoming-drive preparation is added.', '',
             'The terrain-aware controller is rerun from the same state. RMS values for early failures cover only the observed prefix and must not be compared as equal-duration successes.', '',
             '| Case | Controller | Completed | Accepted | Time s | Moving RMS mm | Max mm | Final mm | Failure |',
             '|---|---|---|---|---|---|---|---|---|']
    for row in results:
        for s in row['summaries']:
            t = s['tracking']
            lines.append(f"| {row['case']} | {s['controller']} | {s['completed']} | {s['mobility']['passed']} | {s['completed_duration_s']:.2f} | {1000*t['rms_error_while_moving_m']:.1f} | {1000*t['max_error_m']:.1f} | {1000*t['final_error_m']:.1f} | {s['failure'] or '-'} |")
    lines.extend(['', '## Replays', ''])
    for row in results:
        lines.append(f"- [{row['case']}]({row['case']}/replay.mp4)")
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    write_json(output/'results.json', results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', nargs='+', choices=CASES,
                        default=['hills_circle_ccw', 'hills_circle_cw', 'rough_figure8', 'bank_circle'])
    parser.add_argument('--output', type=Path, default=REPO_ROOT/'results'/'offroad'/'local_lqr_comparison')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = load_config()
    manifest = dict(cases=args.cases, configuration=config, thresholds=TARGETS, seed=7, phase=.3,
                    source_sha256={str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in [Path(__file__).resolve(), *sorted((REPO_ROOT/'src'/'spherical_wheel').glob('*.py'))]})
    if (output/'manifest.json').exists() and json.loads((output/'manifest.json').read_text()) != manifest:
        raise ValueError('Existing experiment differs. Choose a new output directory.')
    write_json(output/'manifest.json', manifest)
    cache = REPO_ROOT/'generated'/'offroad'/'mobility'/'design_0.001'/'flat'/'control_schedule.npz'
    workspace = Workspace(config, REPO_ROOT/'generated'/'offroad'/'local_lqr_comparison', cache, plant_limit=1)
    results = []
    for name in args.cases:
        directory = output/name
        directory.mkdir(exist_ok=True)
        if (directory/'pair.json').exists():
            results.append(json.loads((directory/'pair.json').read_text()))
            update_report(output, results)
            continue
        started = time.time()
        print(f'Starting {name}', flush=True)
        kind, shape, duration, clockwise = CASES[name]
        terrain = terrain_for(kind, 7)
        audit = terrain.audit(BOUNDS, .12, samples=101)
        if not audit['single_contact_valid']:
            raise ValueError(f'Invalid terrain: {audit}')
        plant = TerrainPlant(config, terrain, workspace.directory/name, visual_bounds=BOUNDS)
        workspace._plants.clear()
        workspace._plants[json.dumps(terrain.spec(), sort_keys=True)] = plant
        path = MobilityPath(shape, duration, clockwise=clockwise)
        runs, summaries = [], []
        for mode in ('local_lqr', 'terrain_aware'):
            run_dir = directory/mode
            if (run_dir/'summary.json').exists():
                summary = json.loads((run_dir/'summary.json').read_text())
                with np.load(run_dir/'rollout.npz') as saved:
                    log = dict(saved)
            else:
                if mode == 'local_lqr':
                    log, summary = local_run(workspace, terrain, path)
                else:
                    log, summary = experiments.run_offroad(workspace, terrain, path, OffroadParams(), phase=.3)
                evaluate(log, summary, path)
                summary.update(seed=7, start_phase_rad=.3, physics_timestep_s=.001, terrain_audit=audit)
                save_run(run_dir, log, summary, terrain)
            plot_run(log, summary, terrain, path, run_dir/'plots.png', name+' / '+mode)
            log['planned_xy'] = np.array([path(t)[0] for t in np.linspace(path.start_s, path.start_s+path.duration_s, 180)])
            runs.append((mode, log))
            summaries.append(summary)
            print(f"  {mode}: completed={summary['completed']}, accepted={summary['mobility']['passed']}, "
                  f"time={summary['completed_duration_s']:.2f}s, failure={summary['failure']}", flush=True)
        np.testing.assert_allclose(runs[0][1]['qpos'][0], runs[1][1]['qpos'][0], atol=1e-12, rtol=0)
        replay = render_replay(plant, runs, directory/'replay.mp4', fps=20, width=640, height=480,
                               elevation=-55., distance=2.5, title=name,
                               fixed_lookat=[0., 0., terrain.height(0., 0.)])
        result = dict(case=name, summaries=summaries, replay=str(replay))
        write_json(directory/'pair.json', result)
        results.append(result)
        update_report(output, results)
        print(f'Finished {name} in {time.time()-started:.0f}s: {replay}', flush=True)
    write_json(output/'complete.json', dict(completed_pairs=len(results), cases=args.cases))


if __name__ == '__main__':
    main()
