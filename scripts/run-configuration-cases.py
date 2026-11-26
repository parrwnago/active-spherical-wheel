"""Short Gazebo mechanism tests; no new controller or hardware validation.

Compare existing LS and geometry-adaptive bounded allocation on flat and
10-degree side-slope ground, two carrier phases, nominal fork-pitch offsets,
and hold/forward/lateral references. Pitch offsets are compensated by shaft
angle so carrier and wheel contact geometry initially stay unchanged. Cold
observer initialization is kept consistent and initial reference error logged.
"""
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import ExitStack
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from unittest.mock import patch

import mujoco
import numpy as np

from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import ReferencePath, measure_for, save_run
from spherical_wheel.gazebo.experiments import GazeboWorkspace, run_gazebo
from spherical_wheel.terrain import Terrain
import spherical_wheel.offroad_experiments as experiments


_WORKSPACE = None
SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
SCALE = np.array([5., 5., 5., .3, .3])


def geometry(plant, points, loads):
    total = float(np.sum(loads))
    matrix = np.zeros((2, 5))
    if total <= 0:
        return matrix, np.nan, np.full(2, np.nan)
    for side, point, load in zip(SUPPORTS, points, loads):
        if load <= 0 or not np.isfinite(point).all():
            continue
        geom = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, side+'_smooth')
        jac = np.zeros((3, plant.model.nv))
        mujoco.mj_jac(plant.model, plant.data, jac, None, point, int(plant.model.geom_bodyid[geom]))
        row = np.zeros((2, 5))
        row[:, :3] = jac[:2, 6:9]
        if side.startswith('belt'):
            row[:, 3+int(side.endswith('B'))] = plant.data.geom_xmat[geom].reshape(3, 3)[:2, 1]
        matrix += load/total*row
    left, singular, _ = np.linalg.svd(matrix*SCALE, full_matrices=False)
    return matrix, float(singular[-1]), left[:, -1]


def run_case(job):
    global _WORKSPACE
    output = Path(job['output'])
    if _WORKSPACE is None:
        _WORKSPACE = GazeboWorkspace(
            load_config(), output/'_workspaces'/f'worker_{os.getpid()}',
            seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz',
            heightmap_cell_m=.04, plant_options={'visual_bounds': (-.7, .7, -.7, .7)})
    slope = np.tan(np.deg2rad(10.)) if job['terrain'] == 'side_slope_10' else 0.
    terrain = Terrain(name=job['terrain'], features=[{'type': 'plane', 'slope': [0., slope]}])
    # 0.12 m / (2.5 - 0.5 s) = 0.06 m/s peak, with smooth acceleration.
    moving = job['motion'] != 'hold'
    path = ReferencePath(.12 if moving else 0., 90. if job['motion'] == 'world_y' else 0.,
                         2.5, start_s=.5, ramp_s=.5)
    parameters = OffroadParams(mode='hardware', rolling_allocation=(
        'bounded' if job['variant'] == 'adaptive' else 'least_squares'),
        rolling_adaptive_effort=job['variant'] == 'adaptive')
    placement = {}
    original_place = experiments.place_on_ground

    def place(plant, controller, *args, **kwargs):
        qpos = original_place(plant, controller, *args, **kwargs)
        before_carrier = plant.data.xmat[controller.carrier_id].reshape(3, 3).copy()
        shaft_axis = plant.data.xmat[plant.fork_id].reshape(3, 3)[:, 1].copy()
        delta = np.deg2rad(job['fork_pitch_offset_deg'])
        target = qpos.copy()
        mujoco.mju_mulQuat(target[3:7], qpos[3:7].copy(),
                          np.array([np.cos(delta/2), 0., np.sin(delta/2), 0.]))
        target[7] -= delta
        plant.reset(qpos=target, qvel=np.zeros(plant.model.nv))
        after_carrier = plant.data.xmat[controller.carrier_id].reshape(3, 3)
        after_axis = plant.data.xmat[plant.fork_id].reshape(3, 3)[:, 1]
        rotation_error = float(np.max(abs(after_carrier-before_carrier)))
        shaft_error = float(np.max(abs(after_axis-shaft_axis)))
        assert rotation_error < 1e-8 and shaft_error < 1e-8
        measure_for(controller, plant)
        controller.reset(job['phase_rad'])
        controller.nominal_joints = plant.data.qpos[7:10].copy()
        placement.update(carrier_rotation_max_error=rotation_error,
                         shaft_axis_max_error=shaft_error,
                         terrain_normal=terrain.normal_at(0., 0.).tolist(),
                         initial_qpos=plant.data.qpos.tolist(),
                         initialization='Existing terrain-informed balance, then compensated fork pitch; static observer reset in every case.')
        return plant.data.qpos.copy()

    records = []
    original_command = OffroadController.command

    def forbidden(*args, **kwargs):
        raise AssertionError('Controller attempted to read simulator contacts')

    def command(controller, *args, **kwargs):
        plant = controller.plant
        assert plant.ground is None and plant.terrain is None and plant.data.ncon == 0
        with ExitStack() as stack:
            for name in ('spherical_wheel.offroad_control.measure_contact',
                         'spherical_wheel.offroad_control.records_from_mujoco',
                         'spherical_wheel.offroad_control.rolling_feedforward',
                         'spherical_wheel.synchronization.ContactSynchronizer._normal_loads',
                         'mujoco.mj_contactForce'):
                stack.enter_context(patch(name, forbidden))
            applied, info = original_command(controller, *args, **kwargs)
            loads = info['estimated_support_loads_N']
            matrix, authority, weak = geometry(plant, info['estimated_contact_points_m'], loads)
            records.append({'configuration_matrix': matrix,
                            'configuration_authority_m_s': authority,
                            'configuration_weak_requested_speed_m_s': abs(float(weak@np.asarray(args[1]))),
                            'configuration_patch': int(np.argmax(loads)) if np.sum(loads) > 0 else -1})
        return applied, info

    with patch.object(experiments, 'place_on_ground', place), patch.object(OffroadController, 'command', command):
        log, summary = run_gazebo(_WORKSPACE, terrain, path, parameters,
                                  phase=job['phase_rad'], duration_s=4., record_diagnostics=True)
    assert len(records) == len(log['time_s'])
    for key in records[0]:
        log[key] = np.asarray([record[key] for record in records])
    summary['configuration_case'] = {**job, 'placement': placement,
        'reference': path.spec(), 'scope': 'Four-second mechanism comparison, not Mars-route qualification.',
        'cold_observer_window_s': [0., .3], 'motion_starts_s': .5,
        'contact_truth_api_guarded_calls': len(records)}
    directory = output/'runs'/job['case_id']/job['variant']
    save_run(directory, log, summary, terrain)
    patch_series = log['configuration_patch']
    error = np.linalg.norm(log['qpos'][:, :2]-log['reference_xy'], axis=1)
    attitude = np.degrees(log['attitude_error_rad'])
    cold = log['time_s'] <= .3
    warm = log['time_s'] > .3
    authority = log['configuration_authority_m_s']
    row = {key: job[key] for key in ('case_id', 'terrain', 'phase_rad', 'fork_pitch_offset_deg', 'motion', 'variant')}
    row.update(completed=summary['completed'], failure=summary['failure'],
               duration_s=summary['completed_duration_s'], initial_reference_lean_error_deg=float(attitude[0]),
               cold_max_reference_lean_error_deg=float(attitude[cold].max()),
               warm_max_reference_lean_error_deg=float(attitude[warm].max()) if warm.any() else None,
               max_tracking_error_mm=float(1000*error.max()), final_tracking_error_mm=float(1000*error[-1]),
               rms_tracking_error_mm=float(1000*np.sqrt(np.mean(error**2))),
               min_authority_m_s=float(np.nanmin(authority)), initial_authority_m_s=float(authority[0]),
               initial_estimated_support=SUPPORTS[patch_series[0]] if patch_series[0] >= 0 else 'none',
               initial_true_support=SUPPORTS[int(np.argmax(log['support_loads_N'][0]))],
               estimated_patch_switches=int(np.count_nonzero(np.diff(patch_series))),
               max_joint_reference_rad_s=float(np.max(abs(log['reference_qvel'][:, 6:9]))),
               max_actual_joint_speed_rad_s=float(np.max(abs(log['qvel'][:, 6:9]))),
               saturated_fraction=summary['actuation']['saturated_time_fraction'],
               guarded_calls=len(records), directory=str(directory))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=3)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    jobs = []
    for terrain in ('flat', 'side_slope_10'):
        for phase in (.60, np.pi/2):
            for offset in (0., 3.):
                for motion in ('hold', 'world_x', 'world_y'):
                    # With zero desired motion and zero allocator history the
                    # two existing allocation modes emit the same zero refs.
                    for variant in (('baseline',) if motion == 'hold' else ('baseline', 'adaptive')):
                        case = f'{terrain}/phase_{phase:.4f}/pitch_{offset:.0f}/{motion}'
                        jobs.append(dict(output=str(output), case_id=case, terrain=terrain, phase_rad=phase,
                                         fork_pitch_offset_deg=offset, motion=motion, variant=variant))
    sources = [Path(__file__), *(REPO_ROOT/'src/spherical_wheel'/name for name in
        ('rolling_allocation.py', 'offroad_control.py', 'offroad_experiments.py',
         'experiments.py', 'contact_sensing.py', 'synchronization.py'))]
    provenance = {'jobs': jobs, 'reference_duration_s': 4., 'peak_speed_m_s': .06,
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'sources': {str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        'scope': 'Existing controller modes; no new weight law, particle filter, or real-robot data.'}
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    rows = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run_case, job) for job in jobs]):
            row = future.result()
            rows.append(row)
            (output/'summary.json').write_text(json.dumps(rows, indent=2)+'\n')
            print(json.dumps(row), flush=True)
    print(f'Completed {len(rows)} controlled cases.', flush=True)


if __name__ == '__main__':
    main()
