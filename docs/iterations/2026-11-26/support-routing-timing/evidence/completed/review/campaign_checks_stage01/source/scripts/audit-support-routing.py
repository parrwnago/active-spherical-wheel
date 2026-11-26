"""I-46 native-history support-routing audit and registered native data collection.

Every command sent to the robot simulator comes from the unchanged native
controller.  Alternative interval-mean support shares are shadow arithmetic
only.  Fresh process/dir boundaries preserve failures and simulator history.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
from time import perf_counter
import traceback
import types
from unittest.mock import patch

import numpy as np

from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.support_routing_audit import (
    FALLBACK_CODES, LABELS, IntervalTruthSampler, bridge_contact_truth,
    feedback_routing_shadow, history_fingerprint,
)


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def safe(value):
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def save_json(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def load_helper():
    path = REPO_ROOT/'scripts/run-lqr-contact-forecast.py'
    module = types.ModuleType('_I46_unchanged_I40_helper')
    module.__file__ = str(path)
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def source_snapshot(stage, output, mode, case_id=None):
    sources = [Path(__file__).resolve(), REPO_ROOT/'scripts/run-lqr-contact-forecast.py',
               REPO_ROOT/'pyproject.toml', output/'protocol.json']
    if (output/'campaign/jobs.json').is_file():
        sources.append(output/'campaign/jobs.json')
    sources += sorted((REPO_ROOT/'src/spherical_wheel').rglob('*.py'))
    sources += sorted((REPO_ROOT/'config').glob('*.yaml'))
    sources += sorted((REPO_ROOT/'gazebo/plugin').glob('*.cc'))
    sources += sorted((REPO_ROOT/'gazebo/plugin').glob('CMakeLists.txt'))
    sources += sorted((REPO_ROOT/'tests').glob('test_support_routing*.py'))
    records = []
    for source in sorted(set(sources)):
        rel = source.relative_to(REPO_ROOT)
        target = stage/'source_snapshot'/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        records.append(dict(origin=str(rel), snapshot=str(target.relative_to(stage)), sha256=sha(source)))
    provenance = dict(iteration='I-46', mode=mode, case_id=case_id,
        captured_before_execution_utc=datetime.now(timezone.utc).isoformat(), sources=records,
        process_id=os.getpid(), versions=load_helper().versions(),
        scope='Native controller plus non-actuated same-state gain/holding support-share sensitivity.',
        measurement_boundary='Controller receives exact simulated qpos/qvel and known previous input; no solved contacts or terrain. Truth is evaluator-only.')
    if mode != 'saved_native_history':
        from spherical_wheel.gazebo.bridge import plugin_path
        from spherical_wheel.gazebo.experiments import GAZEBO_VERSIONS
        binary = plugin_path()
        target = stage/'source_snapshot'/binary.relative_to(REPO_ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(binary, target)
        provenance['bridge_binary'] = dict(path=str(binary.relative_to(REPO_ROOT)), sha256=sha(binary), snapshot=str(target.relative_to(stage)))
        provenance['engine_versions'] = GAZEBO_VERSIONS
    save_json(stage/'provenance.json', provenance)
    return provenance


def verify_registered_inputs(protocol):
    result = {}
    if set(protocol.get('inputs', {})) != set(protocol.get('input_sha256', {})):
        raise RuntimeError('Every registered input requires an exact hash')
    for name, path in protocol.get('inputs', {}).items():
        if not isinstance(path, str):
            continue
        actual = sha(REPO_ROOT/path)
        expected = protocol.get('input_sha256', {}).get(name)
        if expected is not None and actual != expected:
            raise RuntimeError('Registered input changed: '+name)
        result[name] = actual
    for path, expected in protocol.get('source_sha256', {}).items():
        if sha(REPO_ROOT/path) != expected:
            raise RuntimeError('Registered source changed: '+path)
    return result


def native_row(controller, applied, info, reference, nominal_before, previous, hook):
    shadow = hook['shadow']
    filtered, interval = shadow['filtered'], shadow['interval']
    row = dict(time_s=float(controller.plant.data.time), exact_qpos=controller.plant.data.qpos.copy(),
        exact_qvel=controller.plant.data.qvel.copy(), exact_applied=np.asarray(applied).copy(),
        exact_previous_applied=np.asarray(previous).copy(), exact_native_requested=np.asarray(info['requested']).copy(),
        exact_nominal_joints_before=np.asarray(nominal_before).copy(),
        exact_reference_xy=np.asarray(reference[0]).copy(),
        exact_reference_velocity_m_s=np.asarray(reference[1]).copy(),
        exact_reference_acceleration_m_s2=np.asarray(reference[2]).copy(),
        exact_estimated_normal=np.asarray(info['estimate']['normal']).copy(),
        exact_observer_wrench=np.r_[controller.observer.wrench()],
        exact_observer_generalized=controller.observer.generalized.copy(),
        exact_observer_interval_mean=controller.observer.interval_mean.copy(),
        exact_feedback_gain=np.asarray(shadow['native_feedback_gain']).copy(),
        exact_feedback_error=np.asarray(shadow['native_error']).copy(),
        exact_support_torque=np.asarray(shadow['native_support_torque']).copy(),
        exact_rolling_feedforward=np.asarray(info['rolling_feedforward']).copy(),
        exact_feedback_reference_qvel=np.asarray(info['feedback_reference_qvel']).copy(),
        exact_reference_qpos=np.asarray(info['reference_qpos']).copy(),
        exact_sync_weights=np.asarray(info['sync_weights']).copy(),
        exact_transition_command_delta=np.asarray(info['transition_command_delta']).copy(),
        exact_heading_command=np.asarray(info['heading_command']).copy(),
        exact_fade_command_delta=np.asarray(info['fade_command_delta']).copy(),
        input_bounds=controller.plant.bounds.copy(),
        native_touching=shadow['native_touching'], shadow_fallback_code=shadow['fallback_code'],
        shadow_used_loads_N=shadow['used_loads_N'],
        native_feedback_effective_loads_N=shadow['native_feedback_effective_loads_N'],
        shadow_used_feedback_effective_loads_N=shadow['used_feedback_effective_loads_N'],
        shadow_feedback_gain=shadow['interval_share_feedback_gain'],
        shadow_support_torque=shadow['interval_share_support_torque'],
        shadow_gain_command_delta=shadow['delta_gain_command'],
        shadow_support_command_delta=shadow['delta_support_command'],
        shadow_pre_sync_command_delta=shadow['delta_pre_sync_command'],
        native_pre_sync_command=shadow['native_pre_sync_command'],
        native_pre_sync_clipped_command=np.clip(shadow['native_pre_sync_command'], -controller.plant.bounds, controller.plant.bounds),
        shadow_pre_sync_clipped_command=np.clip(shadow['interval_share_pre_sync_command'], -controller.plant.bounds, controller.plant.bounds),
        shadow_pre_sync_command=shadow['interval_share_pre_sync_command'],
        native_pre_sync_reconstruction_max_gap=shadow['native_reconstruction_max_abs_gap'],
        shadow_delta_reconstruction_max_gap=shadow['delta_reconstruction_max_abs_gap'],
        shadow_history_unchanged=all(shadow['invariants'].values()),
        shadow_feedback_call_count=hook['feedback_calls'])
    for name, summary in (('filtered', filtered), ('interval', interval)):
        for key, value in summary.items():
            if key != 'qualification':
                row[f'{name}_{key}'] = value
    return row


class NativeFeedbackProbe:
    """One native feedback call, then pure read-only alternative routing."""
    def __init__(self):
        self.current = None

    def begin(self):
        if self.current is not None:
            raise AssertionError('Overlapping native command probes')
        self.current = dict(feedback_calls=0)

    def feedback(self, original, controller, qref, vref, feedforward, normal, measurement, touching):
        if self.current is None:
            raise AssertionError('Feedback probe outside native command')
        before_inputs = [np.asarray(x).copy() for x in (qref, vref, feedforward, normal)]
        command, feedback = original(controller, qref, vref, feedforward, normal, measurement, touching)
        self.current['feedback_calls'] += 1
        shadow = feedback_routing_shadow(controller, normal, measurement, touching, feedback, command, feedforward)
        for current, before in zip((qref, vref, feedforward, normal), before_inputs):
            np.testing.assert_array_equal(current, before)
        self.current['shadow'] = shadow
        return command, feedback

    def finish(self):
        result = self.current
        if result is None or result['feedback_calls'] != 1:
            raise AssertionError('Exactly one original feedback invocation is required')
        self.current = None
        return result


def save_precision(stage, rows, name='precision.npz'):
    if not rows:
        raise AssertionError('No saved samples')
    np.savez_compressed(stage/name, **{key: np.asarray([row[key] for row in rows]) for key in rows[0]})


def finalize(stage, provenance, report):
    changed = [r['origin'] for r in provenance['sources'] if sha(REPO_ROOT/r['origin']) != r['sha256']]
    binary = provenance.get('bridge_binary')
    if binary is not None and sha(REPO_ROOT/binary['path']) != binary['sha256']:
        changed.append(binary['path'])
    report['source_hashes_unchanged'] = not changed
    report['changed_source_paths'] = changed
    save_json(stage/'report.json', report)
    records = [dict(path=str(p.relative_to(stage)), bytes=p.stat().st_size, sha256=sha(p))
        for p in sorted(stage.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json'
        and '_workspace' not in p.relative_to(stage).parts]
    save_json(stage/'artifact_manifest.json', dict(iteration='I-46', files=records))
    if changed:
        raise RuntimeError('Source changed during execution')


def replay_history(output, tag, protocol):
    from spherical_wheel.offroad_control import OffroadController, OffroadParams
    from spherical_wheel.offroad_experiments import Workspace
    stage = output/'campaign'/f'historical_{tag}'
    stage.mkdir(parents=True, exist_ok=False)
    provenance = source_snapshot(stage, output, 'saved_native_history')
    helper = load_helper()
    rows, metrics = [], {}
    started = perf_counter()
    try:
        inputs = verify_registered_inputs(protocol)
        with np.load(REPO_ROOT/protocol['inputs']['historical_precision']) as z:
            exact = {key: z[key].copy() for key in z.files}
        actual = json.loads((REPO_ROOT/protocol['inputs']['historical_provenance']).read_text())
        if helper.versions() != actual['versions']:
            raise AssertionError('Use the historical native numerical environment')
        workspace = Workspace(load_config(), stage/'_workspace',
                              seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz')
        mirror = workspace.mirror()
        mirror.set_state(exact['exact_qpos'][0], exact['exact_qvel'][0], exact['time_s'][0], exact['exact_previous_applied'][0])
        params = OffroadParams(**actual['parameters'])
        assert params.mode == 'hardware' and params.tilt_reference is None
        controller = OffroadController(mirror, workspace.design, params,
            phase=actual['case']['phase_rad'], record_diagnostics=True)
        controller.nominal_joints = np.asarray(actual['initial_nominal_joints'], float).copy()
        path = helper.make_path(actual['case'])
        probe = NativeFeedbackProbe()
        original_feedback = OffroadController._terrain_feedback
        def feedback(instance, *args, **kwargs):
            return probe.feedback(original_feedback, instance, *args, **kwargs)
        with helper.guards(), patch.object(OffroadController, '_terrain_feedback', feedback):
            for index, now in enumerate(exact['time_s']):
                mirror.set_state(exact['exact_qpos'][index], exact['exact_qvel'][index], float(now), exact['exact_previous_applied'][index])
                nominal, previous = controller.nominal_joints.copy(), mirror.last_input.copy()
                reference = path(float(now))
                probe.begin()
                applied, info = controller.command(*reference)
                hook = probe.finish()
                row = native_row(controller, applied, info, reference, nominal, previous, hook)
                pairs = dict(exact_applied=row['exact_applied'], exact_native_requested=row['exact_native_requested'],
                    exact_observer_wrench=row['exact_observer_wrench'], exact_estimated_normal=row['exact_estimated_normal'],
                    exact_nominal_joints_before=nominal, exact_reference_xy=row['exact_reference_xy'],
                    exact_reference_velocity_m_s=row['exact_reference_velocity_m_s'],
                    exact_reference_acceleration_m_s2=row['exact_reference_acceleration_m_s2'])
                for key, value in pairs.items():
                    target = exact[key][index]
                    metric = metrics.setdefault(key, dict(maximum_absolute_gap=0., all_rows_bit_identical=True))
                    metric['maximum_absolute_gap'] = max(metric['maximum_absolute_gap'], float(np.max(abs(value-target))))
                    metric['all_rows_bit_identical'] &= bool(value.shape == target.shape and value.dtype == target.dtype and value.tobytes() == target.tobytes())
                if not all(m['all_rows_bit_identical'] for m in metrics.values()):
                    raise AssertionError((index, float(now), metrics))
                for key in ('truth_time_s', 'truth_patch_loads_N', 'truth_patch_points_m',
                            'truth_patch_contact_counts', 'truth_equivalent_point_m'):
                    row[key] = exact[key][index].copy() if hasattr(exact[key][index], 'copy') else exact[key][index]
                row['truth_interval_valid'] = False
                rows.append(row)
                if index+1 < len(exact['time_s']):
                    controller.after_step()
                if index % 1000 == 0:
                    print(json.dumps(dict(mode='historical', samples=index+1, time_s=float(now))), flush=True)
        save_precision(stage, rows)
        report = dict(iteration='I-46', mode='saved_native_history', passed=True,
            samples=len(rows), history_parity=metrics, inputs_before_sha256=inputs,
            inputs_after_sha256=verify_registered_inputs(protocol),
            all_shadow_histories_unchanged=all(r['shadow_history_unchanged'] for r in rows),
            max_native_pre_sync_reconstruction_gap=max(r['native_pre_sync_reconstruction_max_gap'] for r in rows),
            elapsed_s=perf_counter()-started, new_physics_runs=0,
            truth_timing='Historical truth is last completed1ms solve only. No preceding5ms truth is reconstructed from it.',
            fallback_codes=FALLBACK_CODES, commands='Every requested/applied native command reproduced bit-identically; shadow routing never actuated.')
        finalize(stage, provenance, report)
    except Exception:
        save_json(stage/'failure.json', dict(traceback=traceback.format_exc(), samples=len(rows)))
        if rows:
            save_precision(stage, rows, 'partial_precision.npz')
        raise


def build_path(job, helper):
    """Declared out-and-back program, no measured-state reanchoring."""
    if 'path' in job and 'kind' in job['path']:
        path = helper.make_path(job)
    else:
        spec = job.get('path', job)
        distance = float(spec['leg_distance_m'])
        angle = np.radians(float(spec['heading_deg']))
        delta = distance*np.array([np.cos(angle), np.sin(angle)])
        path = helper.WaypointPath([np.zeros(2), delta, np.zeros(2)],
                                   spec['leg_duration_s'], spec['start_s'])
        path.direction = delta/np.linalg.norm(delta)
    if 'origin_xy_m' in job:
        path.origin = np.asarray(job['origin_xy_m'], float).copy()
    return path


def build_native_workspace(output, case_id, job):
    from spherical_wheel.gazebo.experiments import GazeboWorkspace
    from spherical_wheel.gazebo.mars import MarsYard
    from spherical_wheel.terrain import Terrain
    origin = np.asarray(job['origin_xy_m'], float)
    bounds = tuple(job.get('visual_bounds', [origin[0]-.7, origin[0]+.7, origin[1]-.7, origin[1]+.7]))
    terrain_kind = job['terrain']
    if terrain_kind in ('marsyard2021_dem', 'marsyard2021'):
        terrain = MarsYard()
        options = dict(ground='marsyard2021_dem', visual_bounds=bounds,
                       ground_check_points=job.get('ground_check_points', [origin.tolist()]))
        workspace = GazeboWorkspace(load_config(), output/'campaign/_workspaces'/case_id,
            seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz', plant_options=options)
    elif terrain_kind == 'flat':
        height = float(job.get('flat_height_m', 0.))
        if height != 0.:
            raise ValueError('Registered I46 flat control uses standard plane z0')
        terrain = Terrain(name='I46_flat_control', features=[dict(type='plane', offset=height, slope=[0., 0.])])
        workspace = GazeboWorkspace(load_config(), output/'campaign/_workspaces'/case_id,
            seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz', ground='plane',
            plant_options=dict(visual_bounds=bounds,
                ground_check_points=job.get('ground_check_points', [origin.tolist()])))
    else:
        raise ValueError('Only registered Mars DEM and flat control terrains supported')
    return workspace, terrain


def interval_truth_row(completed):
    valid = completed is not None
    return dict(truth_interval_valid=valid,
        truth_interval_start_s=completed['start_time_s'] if valid else np.nan,
        truth_interval_end_s=completed['end_time_s'] if valid else np.nan,
        truth_interval_mean_wrench_world=completed['mean_wrench_world'] if valid else np.full(6, np.nan),
        truth_interval_mean_all_external_wrench_world=completed['mean_all_external_wrench_world'] if valid else np.full(6, np.nan),
        truth_interval_mean_body_wrench_world=completed['mean_body_wrench_world'] if valid else np.full(6, np.nan),
        truth_interval_mean_body_load_N=completed['mean_body_load_N'] if valid else np.nan,
        truth_interval_mean_loads_N=completed['mean_loads_N'] if valid else np.full(4, np.nan),
        truth_interval_mean_patch_points_m=completed['mean_patch_points_m'] if valid else np.full((4, 3), np.nan),
        truth_interval_mean_equivalent_point_m=completed['mean_equivalent_point_m'] if valid else np.full(3, np.nan),
        truth_interval_sample_count=len(completed['substep_time_s']) if valid else 0)


def save_substeps(stage, intervals):
    from spherical_wheel.gazebo.bridge import STATE_FIELDS
    times, states, contacts, offsets = [], [], [], [0]
    for interval in intervals:
        for k, raw in enumerate(interval['substep_contact_rows']):
            times.append(interval['substep_time_s'][k])
            states.append(interval['substep_bridge_state'][k])
            contacts.append(raw)
            offsets.append(offsets[-1]+len(raw))
    if times:
        np.savez_compressed(stage/'truth_substeps.npz', time_s=np.array(times),
            bridge_state=np.array(states), bridge_state_fields=np.array(STATE_FIELDS), contacts=np.concatenate(contacts, axis=0),
            contact_offsets=np.array(offsets, dtype=np.int64))


def collect_native(output, case_id, protocol):
    from spherical_wheel.offroad_control import OffroadController, OffroadParams
    from spherical_wheel.gazebo.experiments import run_gazebo
    import spherical_wheel.offroad_experiments as experiments
    jobs_path = output/'campaign/jobs.json'
    jobs = json.loads(jobs_path.read_text())['cases'] if jobs_path.is_file() else protocol['cases']
    job = next(j for j in jobs if j['case_id'] == case_id)
    stage = output/'campaign/runs'/case_id
    stage.mkdir(parents=True, exist_ok=False)
    provenance = source_snapshot(stage, output, 'native_collection', case_id)
    helper, probe = load_helper(), NativeFeedbackProbe()
    rows, truth_rows, intervals = [], [], []
    workspace = None
    started = perf_counter()
    try:
        inputs = verify_registered_inputs(protocol)
        workspace, terrain = build_native_workspace(output, case_id, job)
        physical = workspace.plant(terrain)
        sampler = IntervalTruthSampler(physical.sim, physical_step_s=.001, control_dt_s=physical.dt)
        path = build_path(job, helper)
        parameters = protocol.get('execution', {}).get('parameters', helper.PARAMETERS)
        params = OffroadParams(**parameters)
        assert params.mode == 'hardware' and params.synchronize and params.tilt_reference is None
        original_command, original_feedback = OffroadController.command, OffroadController._terrain_feedback
        original_step = physical.step
        original_measure = experiments.measure_contact
        registered_loop_started = False

        def measured(plant):
            measured_truth = original_measure(plant)
            truth = helper.truth_contact_row(plant, measured_truth)
            truth['truth_last_wrench_world'] = np.r_[measured_truth['force_N'], measured_truth['moment_Nm']]
            truth['truth_last_body_load_N'] = float(measured_truth['fork_load_N'])
            truth_rows.append(truth)
            return measured_truth

        def command(controller, xy, velocity, acceleration):
            nonlocal registered_loop_started
            mirror = controller.plant
            assert mirror.ground is None and mirror.terrain is None and mirror.data.ncon == 0
            nominal, previous = controller.nominal_joints.copy(), mirror.last_input.copy()
            before = history_fingerprint(controller)
            probe.begin()
            with helper.guards(), patch.object(physical, 'contact_records', helper.forbidden):
                applied, info = original_command(controller, xy, velocity, acceleration)
            hook = probe.finish()
            row = native_row(controller, applied, info, (xy, velocity, acceleration), nominal, previous, hook)
            completed = sampler.completed_interval(float(mirror.data.time))
            row.update(interval_truth_row(completed))
            row.update(native_history_fingerprint_before=before,
                       native_history_fingerprint_after=history_fingerprint(controller))
            rows.append(row)
            if completed is not None:
                intervals.append(completed)
            registered_loop_started = True
            if len(rows) % 400 == 0:
                print(json.dumps(dict(mode='native_collection', case_id=case_id, time_s=row['time_s'])), flush=True)
            return applied, info

        def feedback(controller, *args, **kwargs):
            return probe.feedback(original_feedback, controller, *args, **kwargs)

        def step(*args, **kwargs):
            if not registered_loop_started:
                return original_step(*args, **kwargs)
            sampler.begin(float(physical.data.time))
            result = original_step(*args, **kwargs)
            sampler.finish()
            return result

        with patch.object(OffroadController, 'command', command), \
             patch.object(OffroadController, '_terrain_feedback', feedback), \
             patch.object(experiments, 'measure_contact', measured), \
             patch.object(physical, 'step', step), patch.object(physical.sim, 'run', sampler.run):
            log, summary = run_gazebo(workspace, terrain, path, params, phase=job['phase_rad'],
                yaw_deg=job['yaw_deg'], duration_s=job['duration_s'], record_diagnostics=True)
        if not len(rows) == len(truth_rows) == len(log['time_s']):
            raise AssertionError('Native commands, evaluator records and rollout must align')
        for row, truth in zip(rows, truth_rows):
            if abs(row['time_s']-truth['truth_time_s']) > 1e-12:
                raise AssertionError('Evaluator truth clock mismatch')
            row.update(truth)
        summary['support_routing_audit'] = dict(iteration='I-46', case=job,
            native_commands_unchanged=True, shadow_actuated=False, observer_updates_per_command=1,
            interval_truth='Five1ms completed solver records; moments averaged about each matching moving centre.',
            stepping='One held command; five run(1) calls verified separately against one run(5).')
        summary['contact_forecast_case'] = dict(case=job, actual_path=path.spec(), activation={},
            guarded_controller_calls=len(rows), forecast_affects_control=False)
        experiments.save_run(stage, log, summary, terrain)
        save_precision(stage, rows)
        save_substeps(stage, intervals)
        provenance.update(case=job, actual_path=path.spec(), parameters=params.as_dict(),
            initial_nominal_joints=rows[0]['exact_nominal_joints_before'],
            model_mass_kg=physical.total_mass, weight_N=physical.total_mass*9.81,
            input_bounds=physical.bounds.copy(), controller_dt_s=physical.dt, physics_step_s=.001,
            gain_cache_sha256=sha(workspace.cache_path), engine=summary['engine'])
        save_json(stage/'provenance.json', provenance)
        report = dict(iteration='I-46', mode='native_collection', case_id=case_id,
            completed=summary['completed'], failure=summary['failure'], samples=len(rows),
            duration_s=summary['completed_duration_s'], preceding_intervals=len(intervals),
            all_shadow_histories_unchanged=all(r['shadow_history_unchanged'] for r in rows),
            every_native_feedback_called_once=all(r['shadow_feedback_call_count'] == 1 for r in rows),
            all_interval_counts_five=all(r['truth_interval_sample_count'] == 5 for r in rows[1:]),
            inputs_before_sha256=inputs, inputs_after_sha256=verify_registered_inputs(protocol),
            fallback_codes=FALLBACK_CODES, elapsed_s=perf_counter()-started,
            truth_timing='Last1ms solver endpoint and preceding5ms solver mean are separate evaluator targets.',
            shadow_scope='Only gain/holding routing sensitivity; native references/FF/synchronization/heading/fade/clipping execute unchanged.')
        finalize(stage, provenance, report)
    except Exception:
        save_json(stage/'failure.json', dict(traceback=traceback.format_exc(), samples=len(rows)))
        if rows:
            save_precision(stage, rows, 'partial_precision.npz')
        if intervals:
            save_substeps(stage, intervals)
        raise
    finally:
        if workspace is not None and workspace._current is not None:
            workspace._current[1].close()


def run_bridge_fixture(output, branch, tag, protocol):
    """One isolated fresh-process branch; root compares the two preserved results."""
    from spherical_wheel.gazebo.experiments import GazeboWorkspace
    from spherical_wheel.gazebo.bridge import STATE_FIELDS
    from spherical_wheel.terrain import Terrain
    stage = output/'review'/f'bridge_fixture_{tag}_{branch}'
    stage.mkdir(parents=True, exist_ok=False)
    provenance = source_snapshot(stage, output, 'bridge_stepping_fixture', branch)
    fixture = protocol['bridge_fixture']
    workspace = None
    try:
        verify_registered_inputs(protocol)
        workspace = GazeboWorkspace(load_config(), stage/'_workspace',
            seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz', ground='plane',
            plant_options=dict(visual_bounds=(-.4, .4, -.4, .4), ground_check_points=[[0., 0.]]))
        physical = workspace.plant(Terrain(name='I46_bridge_fixture_flat'))
        qpos = np.asarray(fixture['qpos'], float)
        qvel = np.asarray(fixture['qvel'], float)
        physical.reset(qpos=qpos, qvel=qvel)
        states = [physical.sim.state().copy()]
        contact_rows = [physical.sim.contacts().copy()]
        commands = np.asarray(fixture['commands'], float)
        for command in commands:
            physical.apply_input(command)
            physical.sim.command(physical.last_input[:3], physical.last_input[3:],
                                 np.zeros(6), physical.nominal_friction)
            if branch == 'batch':
                physical.sim.run(5)
            else:
                for _ in range(5):
                    physical.sim.run(1)
                    # Reading state/contact snapshots must not advance a server.
                    physical.sim.state()
                    physical.sim.contacts()
            states.append(physical.sim.state().copy())
            contact_rows.append(physical.sim.contacts().copy())
        offsets = np.r_[0, np.cumsum([len(c) for c in contact_rows])]
        np.savez_compressed(stage/'bridge_fixture.npz', state=np.array(states), commands=commands,
                            contacts=np.concatenate(contact_rows, axis=0), contact_offsets=offsets, bridge_state_fields=np.array(STATE_FIELDS))
        finalize(stage, provenance, dict(iteration='I-46', mode='bridge_stepping_fixture',
            branch=branch, completed=True, control_holds=len(commands), physics_steps=len(commands)*5,
            duration_s=len(commands)*.005, controller_calls=0, source='Two separately isolated fresh processes; no restored warmstart parity assumption.'))
    except Exception:
        save_json(stage/'failure.json', dict(traceback=traceback.format_exc()))
        raise
    finally:
        if workspace is not None and workspace._current is not None:
            workspace._current[1].close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tag', default='stage01')
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--historical', action='store_true')
    choice.add_argument('--collect', metavar='CASE_ID')
    choice.add_argument('--bridge-fixture', choices=('batch', 'single'))
    choice.add_argument('--preflight', action='store_true')
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        parser.error('Fresh stage tags must be alphanumeric with optional underscores')
    output = args.output.resolve()
    protocol = json.loads((output/'protocol.json').read_text())
    if protocol['iteration'] != 'I-46':
        raise ValueError('Registered I-46 protocol required')
    if args.preflight:
        print(json.dumps(dict(passed=True, inputs_sha256=verify_registered_inputs(protocol))), flush=True)
    elif args.historical:
        replay_history(output, args.tag, protocol)
    elif args.collect:
        collect_native(output, args.collect, protocol)
    else:
        run_bridge_fixture(output, args.bridge_fixture, args.tag, protocol)


if __name__ == '__main__':
    main()
