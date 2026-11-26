"""Run the existing hardware controller on Gazebo's Mars yard and render comparisons.

No controller algorithm is changed. The comparison reuses the saved earlier
momentum_observer runs, whose rolling feedforward reads simulator contacts.
"""
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import ExitStack
import json
import logging
from pathlib import Path
import time
from unittest.mock import patch

import numpy as np

from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import save_run
from spherical_wheel.gazebo.mars import (
    MarsYard, _workspace, build_route, evaluate_route, plot_route, route_closed,
    route_item, route_samples,
)
from spherical_wheel.gazebo.experiments import run_gazebo

OUTPUT = Path(__file__).resolve().parent
PRIOR = REPO_ROOT/'results/gazebo/mars_regenerated_20261124/replays'
ROUTES = ('plain_circle', 'crater_rim_climb')


def run_case(name):
    logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
    workspace, yard = _workspace(OUTPUT/'_workspaces', list(ROUTES))
    path, yaw = build_route(name)
    original = OffroadController.command
    calls = 0
    samples = []

    def forbidden(*args, **kwargs):
        raise AssertionError('Controller attempted to read simulator contacts')

    def guarded(self, *args, **kwargs):
        nonlocal calls
        assert self.params.mode == 'hardware' and self.locator is not None
        assert self.plant.terrain is None and self.plant.ground is None
        assert self.plant.data.ncon == 0
        with ExitStack() as stack:
            for target in (
                'spherical_wheel.offroad_control.measure_contact',
                'spherical_wheel.offroad_control.rolling_feedforward',
                'spherical_wheel.synchronization.ContactSynchronizer._normal_loads',
                'mujoco.mj_contactForce',
            ):
                stack.enter_context(patch(target, forbidden))
            applied, diagnostics = original(self, *args, **kwargs)
        calls += 1
        measurement = diagnostics['measurement']
        samples.append({
            'time_s': float(self.plant.data.time),
            'estimated_point_m': np.array(measurement['point_m']),
            'estimated_loads_N': np.array([measurement['loads_N'][key]
                for key in ('shell_A', 'shell_B', 'belt_A', 'belt_B')]),
            'reference_qvel': np.array(diagnostics['reference_qvel']),
        })
        return applied, diagnostics

    started = time.monotonic()
    with patch.object(OffroadController, 'command', guarded):
        log, summary = run_gazebo(workspace, yard, path, OffroadParams(mode='hardware'), yaw_deg=yaw)
    summary = evaluate_route(log, summary, path, yard, route_closed(name))
    summary['route'] = {'name': name, 'about': route_item(name)['about'],
                        'initial_yaw_deg': yaw, 'path': path.spec(), 'start_phase_rad': .30}
    summary['wall_time_s'] = time.monotonic()-started
    summary['contact_input_audit'] = {
        'checked_controller_calls': calls,
        'forbidden_contact_api_accesses': 0,
        'controller_has_terrain_or_ground': False,
        'state_inputs': 'Exact simulated body pose/twist and joint encoders; no sensor noise/delay.',
        'initialization': 'Terrain-informed static balance, same as existing Mars comparison.',
    }
    directory = save_run(OUTPUT/name/'hardware', log, summary, yard)
    np.savez_compressed(directory/'contact_estimates.npz', **{
        key: np.asarray([row[key] for row in samples]) for key in samples[0]})
    plot_route(log, summary, path, yard, directory/'plots.png', f'{name}: estimated-contact feedforward')
    return {'route': name, 'completed': summary['completed'], 'failure': summary['failure'],
            'duration_s': summary['completed_duration_s'],
            'final_error_mm': 1000*summary['tracking']['final_error_m'],
            'max_error_mm': 1000*summary['tracking']['max_error_m'],
            'checked_calls': calls, 'wall_s': summary['wall_time_s']}


def load_log(directory):
    with np.load(Path(directory)/'rollout.npz') as data:
        return {key: np.asarray(value, dtype=float) if value.dtype.kind == 'f' else np.asarray(value)
                for key, value in data.items()}


def render_case(name):
    from spherical_wheel.offroad_view import render_replay
    from spherical_wheel.gazebo.mars_compare import replay_plant
    path, _ = build_route(name)
    old = load_log(PRIOR/name/'momentum_observer')
    new = load_log(OUTPUT/name/'hardware')
    same_start = bool(np.allclose(old['qpos'][0], new['qpos'][0], atol=1e-9, rtol=0))
    if not same_start:
        raise AssertionError(f'{name}: comparison starting states differ')
    points = np.vstack([route_samples(path, 400), old['qpos'][:, :2], new['qpos'][:, :2]])
    bounds = (float(points[:, 0].min()-1.6), float(points[:, 0].max()+1.6),
              float(points[:, 1].min()-1.6), float(points[:, 1].max()+1.6))
    for log in (old, new):
        log['planned_xy'] = route_samples(path, 400)
    runs = [('Prior: feedforward uses simulator contacts', old),
            ('Hardware mode: feedforward uses estimated contacts', new)]
    plant = replay_plant(load_config(), MarsYard(), bounds, OUTPUT/'_replay_scenes'/name)
    duration = max(float(log['time_s'][-1]) for _, log in runs)
    speed = 1. if duration <= 45. else 2.
    video = render_replay(plant, runs, OUTPUT/name/'comparison.mp4', fps=20, speed=speed,
                          width=720, height=540, azimuth=112., elevation=-24., distance=1.8,
                          title=name.replace('_', ' '),
                          annotate=lambda log, i: 'Both use exact state inputs; sensor noise/delay absent.')
    # Also provide an unambiguous standalone replay of the requested controller.
    standalone = render_replay(plant, [runs[1]], OUTPUT/name/'hardware.mp4', fps=20, speed=speed,
                               width=960, height=720, azimuth=112., elevation=-24., distance=1.8,
                               title='Observer contact feedforward',
                               annotate=lambda log, i: 'All controller contacts estimated; exact state inputs.')
    return {'route': name, 'comparison': str(video), 'hardware': str(standalone),
            'speed': speed, 'same_start_as_prior': same_start}


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    results = {}
    with ProcessPoolExecutor(max_workers=2) as pool:
        futures = {pool.submit(run_case, name): name for name in ROUTES}
        for future in as_completed(futures):
            result = future.result()
            results[result['route']] = result
            print(json.dumps(result), flush=True)
    (OUTPUT/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=1) as pool:
        for result in pool.map(render_case, ROUTES):
            print(json.dumps(result), flush=True)
    print('Existing hardware-mode Gazebo replays completed.', flush=True)


if __name__ == '__main__':
    main()
