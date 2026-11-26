"""Fixed-input drift followed by unchanged feedback recovery in Gazebo.

The first .4s reproduces the nearby-pose I31 hold fixture. The native observer
runs as a shadow throughout; feedback starts without resetting state or the
observer. Constant goal XY is retained from the beginning of the run.
"""
from contextlib import ExitStack
from dataclasses import asdict
from datetime import datetime, timezone
import argparse
import hashlib
import importlib.util
from importlib.metadata import PackageNotFoundError, version
import json
from pathlib import Path
import subprocess
import traceback
from unittest.mock import patch

import mujoco
import numpy as np

from spherical_wheel.gazebo.experiments import GazeboWorkspace, run_gazebo
from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import ReferencePath, save_run
from spherical_wheel.terrain import Terrain


SOURCE_CASE = REPO_ROOT/'results/mpc_equilibrium_20261125/independent_search/report.json'
MPC_SCRIPT = REPO_ROOT/'scripts/run-mpc-comparison.py'
DESIGN_PHASE = np.pi/2


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_mpc_harness():
    spec = importlib.util.spec_from_file_location('feedback_recovery_mpc', MPC_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def snapshot(directory, args, source_case):
    paths = [Path(__file__).resolve(), MPC_SCRIPT, REPO_ROOT/'pyproject.toml',
             *sorted((REPO_ROOT/'src/spherical_wheel').rglob('*.py')),
             *sorted((REPO_ROOT/'config').glob('*.yaml'))]
    hashes = {}
    for source in paths:
        relative = source.relative_to(REPO_ROOT)
        target = directory/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
        hashes[str(relative)] = digest(source)
    versions = {}
    for name in ('numpy', 'scipy', 'mujoco', 'osqp'):
        try:
            versions[name] = version(name)
        except PackageNotFoundError:
            versions[name] = None
    report = {'captured_before_run_utc': datetime.now(timezone.utc).isoformat(),
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'arguments': vars(args) | {'output': str(args.output)}, 'source_sha256': hashes,
        'versions': versions, 'source_case_path': str(SOURCE_CASE.relative_to(REPO_ROOT)),
        'source_case_sha256': digest(SOURCE_CASE), 'source_case': source_case,
        'measurement_boundary': 'Exact simulated qpos/qvel and actual previous inputs; observer/locator support. No live contacts or terrain supplied to controller.',
        'initialization': 'Separate fixture reset to saved nearby pose, zero twist requested; Gazebo reset advances1us. Controller reset only at startup after measured mirror follows the actual returned state; nominal joints use actual initial angles. Design anchor remains I30 pi/2.',
        'switch': 'Only actual command selection changes at drift_s; no robot, observer, estimator, synchronizer, reference-goal or heading reset.',
        'native_components': 'LQR/feedforward/synchronization columns are native shadow fields while fixed inputs or MPC replace the final command. recovery_command_delta reconstructs actual requested input.',
        'terrain': 'Uniform10deg side slope mesh .04m; not Mars or rough offroad.' if args.terrain == 'slope' else 'Level plane sanity fixture; zero fixed input before feedback.',
        'scope': 'Fixed-input perturbation then feedback recovery, unchanged controller settings. MPC remains offline and no global/hardware guarantee is asserted.'}
    (directory/'provenance.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def recovery_metrics(log, drift_s):
    time = log['time_s']
    active = time >= drift_s-1e-9
    tail = active & (time >= time[-1]-1.-1e-9)
    goal_error = np.linalg.norm(log['qpos'][:, :2]-log['reference_xy'], axis=1)
    speed = np.linalg.norm(log['qvel'][:, :2], axis=1)
    lean_error = np.degrees(log['attitude_error_rad'])
    absolute_tilt = np.degrees(log['tilt_rad'])
    reference_tilt = np.degrees(np.arccos(np.clip(1.-2.*np.sum(log['reference_quat'][:, 1:3]**2, axis=1), -1., 1.)))
    in_goal = active & (goal_error <= .005)
    settled = in_goal & (speed <= .005) & (lean_error <= 2.)
    sample = float(np.median(np.diff(time))) if len(time) > 1 else .005
    hold_count = int(round(1./sample))+1
    stay_start = None
    for i in np.flatnonzero(active):
        if i+hold_count <= len(time) and np.all(settled[i:i+hold_count]):
            stay_start = float(time[i])
            break
    index = int(np.flatnonzero(active)[0]) if active.any() else len(time)-1
    actuated = active.copy()
    actuated[-1] = False
    return {'goal_xy_m': log['reference_xy'][0].tolist(),
        'feedback_started': bool(active.any()), 'feedback_start_time_s': float(time[index]) if active.any() else None,
        'goal_error_at_feedback_start_mm': float(1000.*goal_error[index]),
        'goal_error_final_mm': float(1000.*goal_error[-1]),
        'goal_error_peak_feedback_mm': float(1000.*goal_error[active].max()) if active.any() else None,
        'goal_error_min_feedback_mm': float(1000.*goal_error[active].min()) if active.any() else None,
        'first_entry_within_5mm_time_s': float(time[np.flatnonzero(in_goal)[0]]) if in_goal.any() else None,
        'first_1s_settled_window_start_s': stay_start,
        'final_1s_within_goal_and_speed_and_reference_lean': bool(tail.any() and time[-1]-time[0] >= drift_s+1.-1e-9 and np.all(settled[tail])),
        'return_gate': 'Final1s: goalXYerror<=5mm AND XYspeed<=5mm/s AND referenceleanerror<=2deg. First entry alone is not recovery.',
        'xy_speed_final_mm_s': float(1000.*speed[-1]),
        'xy_speed_peak_feedback_mm_s': float(1000.*speed[active].max()) if active.any() else None,
        'absolute_platform_tilt_initial_deg': float(absolute_tilt[0]),
        'absolute_platform_tilt_at_feedback_start_deg': float(absolute_tilt[index]),
        'absolute_platform_tilt_final_deg': float(absolute_tilt[-1]),
        'absolute_platform_tilt_peak_feedback_deg': float(absolute_tilt[active].max()) if active.any() else None,
        'reference_platform_tilt_final_deg': float(reference_tilt[-1]),
        'reference_lean_error_final_deg': float(lean_error[-1]),
        'reference_lean_error_peak_feedback_deg': float(lean_error[active].max()) if active.any() else None,
        'fallback_actuated_feedback_samples': int(log['mpc_fallback'][actuated].sum()),
        'actuated_feedback_samples': int(actuated.sum()),
        'input_limit_occupancy_feedback_fraction': float(np.any(log['recovery_actual_input_at_limit'][actuated], axis=1).mean()) if actuated.any() else None,
        'normal_load_min_feedback_N': float(log['normal_force_N'][active].min()) if active.any() else None,
        'mpc_update_wall_median_s': float(np.median(log['mpc_total_wall_s'][log['mpc_updated']])) if np.any(log['mpc_updated']) else None}


def run_case(args):
    directory = args.output/'runs'/args.terrain/args.variant
    if directory.exists():
        raise FileExistsError(f'Preserve earlier runs; choose a fresh output: {directory}')
    directory.mkdir(parents=True)
    source = json.loads(SOURCE_CASE.read_text())['result']
    provenance = snapshot(directory, args, source)
    workspace = GazeboWorkspace(load_config(), args.output/'_workspaces'/args.terrain/args.variant,
        seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz', heightmap_cell_m=.04,
        plant_options={'visual_bounds': (-.6, .6, -.6, .6)})
    terrain = (Terrain(name='side_slope_10', features=[{'type': 'plane', 'slope': [0., np.tan(np.deg2rad(10.))]}])
               if args.terrain == 'slope' else Terrain(name='flat'))
    requested_qpos = np.asarray(source['qpos'], float).copy()
    fixed_input = np.asarray(source['input'], float).copy() if args.terrain == 'slope' else np.zeros(5)
    path = ReferencePath(0., 0., 1., profile='quintic', origin=requested_qpos[:2].copy())
    params = OffroadParams(mode='hardware', rolling_allocation='least_squares', track_contact_point=False)
    mpc = load_mpc_harness()
    original = OffroadController.command
    rows, activation, initialization = [], {}, {}
    experiment = None
    component_ids = None
    held_native = None
    next_native_time = -np.inf

    def initialize(plant, controller, origin, phase, balanced_lean=True, yaw_deg=0.):
        qpos = requested_qpos.copy()
        if args.terrain == 'flat':
            qpos[2] = controller.ordinary['equilibrium']['qpos'][2]
            qpos[3:7] = controller.ordinary['equilibrium']['qpos'][3:7]
        plant.reset(qpos=qpos, qvel=np.zeros(9))
        controller.plant.follow(plant)
        controller.reset(DESIGN_PHASE)
        controller.nominal_joints = controller.plant.data.qpos[7:10].copy()
        initialization.update(requested_qpos=qpos.tolist(), returned_qpos=plant.data.qpos.tolist(),
            returned_qvel=plant.data.qvel.tolist(), fixed_input=fixed_input.tolist(),
            goal_xy=path.origin.tolist(), design_phase_rad=DESIGN_PHASE,
            nominal_joints=controller.nominal_joints.tolist(), reset_microstep_s=1e-6)
        return qpos

    def forbidden(*unused, **ignored):
        raise AssertionError('Controller attempted to read live simulator contact/terrain information')

    def command(controller, xy, velocity, acceleration):
        nonlocal experiment, component_ids, held_native, next_native_time
        mirror = controller.plant
        assert mirror.ground is None and mirror.terrain is None and mirror.data.ncon == 0
        ids = (id(controller.observer), id(controller.estimator), id(controller.synchronizer))
        if component_ids is None:
            component_ids = ids
        assert ids == component_ids
        now = float(mirror.data.time)
        active = now >= args.drift_s-1e-9
        before_time = float(controller.observer._previous[0])
        with ExitStack() as stack:
            for target in ('spherical_wheel.offroad_control.measure_contact',
                           'spherical_wheel.offroad_control.records_from_mujoco',
                           'spherical_wheel.offroad_control.rolling_feedforward',
                           'spherical_wheel.synchronization.ContactSynchronizer._normal_loads',
                           'mujoco.mj_contactForce'):
                stack.enter_context(patch(target, forbidden))
            native, info = original(controller, xy, velocity, acceleration)
            native_requested = info['requested'].copy()
            row = dict(mpc_active=False, mpc_updated=False, mpc_success=False, mpc_fallback=False,
                mpc_total_wall_s=0., mpc_qp_wall_s=0., mpc_linearization_wall_s=0.,
                mpc_plan_nonlinear_xy_max_error_m=np.nan, mpc_plan_nonlinear_rotation_max_error_rad=np.nan,
                mpc_plane_offset_m=np.nan, mpc_load_residual_N=np.nan,
                mpc_nonlinear_cost=np.nan, mpc_plan_source_code=0, mpc_qp_solves_this_update=0)
            if active and not activation:
                activation.update(time_s=now, qpos=mirror.data.qpos.tolist(), qvel=mirror.data.qvel.tolist(),
                    previous_applied_input=mirror.last_input.tolist(), goal_xy=path.origin.tolist(),
                    observer_wrench=np.r_[controller.observer.wrench()].tolist(), physical_state_not_reset=True,
                    live_observer_estimator_synchronizer_preserved=True)
                if args.variant == 'mpc':
                    from spherical_wheel.predictive_control import AffineMPCSettings
                    settings = AffineMPCSettings()
                    experiment = mpc.TrajectoryPredictiveExperiment(mirror, path, settings,
                        load_residual_fraction=.10, jacobian_stride=5, seed_shaft_torque=.03,
                        refinement_steps=args.refinement_steps,
                        native_rollout_seed=args.native_rollout_seed,
                        native_command_function=original)
                    provenance['mpc_settings'] = asdict(settings)
            applied = native.copy() if active else fixed_input.copy()
            if active and args.variant == 'native':
                if held_native is None or now >= next_native_time-1e-9:
                    held_native = native.copy()
                    next_native_time = now+args.native_hold_s
                applied = held_native.copy()
            if experiment is not None:
                applied, update = experiment.command(controller, now, native, info)
                row.update(update)
            if not active or experiment is not None or args.native_hold_s > .005+1e-12:
                info['requested'] = applied.copy()
                info['actuator_saturated'] = np.abs(applied) > mirror.bounds+1e-12
                info['clipping_command_delta'] = np.zeros(5)
            assert float(controller.observer._previous[0]) == now
            assert now == 0. or before_time < now
            assert mirror.data.ncon == 0
            assert np.all(abs(applied) <= mirror.bounds+1e-10)
        quat = mirror.data.qpos[3:7]
        reference = np.asarray(info['reference_quat'])
        row.update(recovery_feedback_active=active, recovery_native_requested=native_requested,
            recovery_native_applied=native.copy(), recovery_command_delta=applied-native_requested,
            recovery_actual_input_at_limit=np.abs(applied) >= .99*mirror.bounds,
            recovery_goal_xy=path.origin.copy(), exact_qpos=mirror.data.qpos.copy(),
            exact_qvel=mirror.data.qvel.copy(), exact_observer_wrench=np.r_[controller.observer.wrench()],
            exact_estimated_normal=np.asarray(info['estimate']['normal']).copy(),
            exact_applied=applied.copy(), time_s=now,
            recovery_absolute_platform_tilt_deg=float(np.degrees(np.arccos(np.clip(1.-2.*np.sum(quat[1:3]**2), -1., 1.)))),
            recovery_reference_platform_tilt_deg=float(np.degrees(np.arccos(np.clip(1.-2.*np.sum(reference[1:3]**2), -1., 1.)))))
        rows.append(row)
        if len(rows) % 100 == 0:
            print(json.dumps({'variant': args.variant, 'terrain': args.terrain, 'time_s': now,
                'stage': 'feedback' if active else 'fixed_input_drift',
                'status': experiment.last_status if experiment else ('native' if active else 'fixed_input')}), flush=True)
        return applied, info

    try:
        with patch('spherical_wheel.offroad_experiments.place_on_ground', initialize), patch.object(OffroadController, 'command', command):
            log, summary = run_gazebo(workspace, terrain, path, params, phase=DESIGN_PHASE,
                duration_s=args.drift_s+args.recovery_s, record_diagnostics=True)
        extra = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
        log.update({key: value for key, value in extra.items() if key.startswith(('mpc_', 'recovery_'))})
        summary['feedback_recovery'] = recovery_metrics(log, args.drift_s) | {
            'variant': args.variant, 'terrain_case': args.terrain, 'initialization': initialization,
            'native_hold_s': args.native_hold_s, 'refinement_steps': args.refinement_steps,
            'native_rollout_seed_enabled': args.native_rollout_seed,
            'activation': activation, 'guarded_controller_calls': len(rows),
            'plans_generated_including_terminal': len(experiment.plan_rows) if experiment else 0,
            'accepted_plans_including_terminal': sum(plan.get('success', False) for plan in experiment.plan_rows) if experiment else 0,
            'native_shadow_fields': 'Native controller components plus recovery_command_delta reconstruct actual command.'}
        save_run(directory, log, summary, terrain)
        np.savez_compressed(directory/'precision.npz', **extra)
        (directory/'plans.json').write_text(json.dumps(mpc.diagnostic_json(experiment.plan_rows if experiment else []), indent=2, allow_nan=False)+'\n')
        provenance.update(initialization=initialization, activation=activation,
            generated_sha256={name: digest(directory/name) for name in ('rollout.npz', 'precision.npz', 'summary.json', 'plans.json')},
            gain_cache_sha256=digest(workspace.cache_path), engine=summary['engine'])
        (directory/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
        print(json.dumps(summary['feedback_recovery']), flush=True)
    except Exception:
        (directory/'execution_failure.json').write_text(json.dumps({'traceback': traceback.format_exc(),
            'initialization': initialization, 'activation': activation, 'guarded_controller_rows': len(rows)}, indent=2)+'\n')
        if rows:
            np.savez_compressed(directory/'partial_precision.npz', **{
                key: np.asarray([row[key] for row in rows]) for key in rows[0]})
        raise
    finally:
        if workspace._current is not None:
            workspace._current[1].close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', choices=('native', 'mpc'), required=True)
    parser.add_argument('--terrain', choices=('slope', 'flat'), default='slope')
    parser.add_argument('--drift-s', type=float, default=.4)
    parser.add_argument('--recovery-s', type=float, default=8.)
    parser.add_argument('--native-hold-s', type=float, default=.005)
    parser.add_argument('--refinement-steps', type=int, default=1)
    parser.add_argument('--native-rollout-seed', action='store_true')
    args = parser.parse_args()
    args.output = args.output.resolve()
    if (not np.isfinite(args.native_hold_s) or args.native_hold_s < .005 or
            not np.isclose(args.native_hold_s/.005, round(args.native_hold_s/.005), atol=1e-9, rtol=0)):
        parser.error('Native hold must be a positive integer multiple of .005s')
    if args.refinement_steps < 1:
        parser.error('Refinement steps must be positive')
    if args.variant != 'mpc' and (args.native_rollout_seed or args.refinement_steps != 1):
        parser.error('Rollout seeds and refinements apply to MPC variants')
    for name in ('drift_s', 'recovery_s'):
        value = getattr(args, name)
        if not np.isfinite(value) or value < 0 or (name == 'recovery_s' and value == 0):
            parser.error('Drift must be finite/nonnegative and recovery finite/positive')
        if not np.isclose(value/.005, round(value/.005), atol=1e-9, rtol=0):
            parser.error('Durations must be integer multiples of the .005s control interval')
    if args.variant == 'mpc' and importlib.util.find_spec('osqp') is None:
        parser.error('MPC requires the optional osqp solver')
    run_case(args)


if __name__ == '__main__':
    main()
