"""I52 whole-run gain-phase x joint-reference experiment, one fresh process/case.

Adapted from the preserved I49 precise runner. The original design, path and
observer remain fixed. Both factors are set before the first feedback call.
--preflight reads registrations; --run explicitly executes Gazebo/DART.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import copy
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import time
import traceback
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
CAPS = (.30,)
PHASES = {'m100':-1.,'p030':.3,'p090':.9}
VARIANTS = {'B':(False,False),'R':(False,True),'N':(True,False),'NR':(True,True)}
CASE_CAPS = {code+'_'+variant:.3 for code in PHASES for variant in VARIANTS}
BASE_BOUNDS = np.array([1.5, .6, .6, .3, .3])
EXACT_INFO_FIELDS = ('feedback_gain', 'error', 'reference_qpos', 'reference_qvel',
    'feedback_reference_qvel', 'equilibrium_command', 'rolling_feedforward',
    'support_command', 'lqr_command', 'transition_command_delta', 'heading_command',
    'fade_command_delta', 'clipping_command_delta', 'sync_weights', 'input_bounds',
    'estimated_support_loads_N', 'estimated_contact_point_m', 'allocation_drive_reference')


def load_component(relative, name):
    source = HERE / relative
    spec = importlib.util.spec_from_file_location(name, source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def digest(value):
    return hashlib.sha256(json.dumps(safe(value), sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def load_helper():
    source = ROOT/'scripts/run-lqr-contact-forecast.py'
    helper = types.ModuleType('i49_unchanged_i40_capture_helper')
    helper.__file__ = str(source)
    exec(compile(source.read_text(), str(source), 'exec'), helper.__dict__)
    return helper


def validate_case(case_id, cap, jobs):
    """Require one registered case and exact predeclared capacity."""
    cap = float(cap)
    if not np.isfinite(cap) or case_id not in CASE_CAPS or cap != CASE_CAPS[case_id]:
        raise ValueError('Case ID and cap must match the registered intervention pair')
    matches = [j for j in jobs if j['case_id'] == case_id]
    if len(matches) != 1:
        raise ValueError('Exactly one registered job is required')
    job = copy.deepcopy(matches[0])
    if float(job['cap_m_s']) != cap:
        raise ValueError('CLI cap disagrees with registered case')
    code, variant = case_id.split('_')
    normal_phase, reference_guard = VARIANTS[variant]
    if (job['terrain'] != 'marsyard2021' or job['phase_rad'] != PHASES[code] or
            job['yaw_deg'] != 45. or job['duration_s'] != 44. or
            job['path'] != {'kind': 'mars_route', 'route': 'plain_figure8'} or
            job['reanchor_at_motion_start'] or job['variant'] != variant or
            job['normal_phase_enabled'] != normal_phase or
            job['reference_guard_enabled'] != reference_guard):
        raise ValueError('Native historical Mars figure-eight conditions must remain fixed')
    return job


def apply_operating_cap(workspace, physical, mirror, cap):
    """Bound-only intervention, after default design and before live controller.

    Never mutate config or design bounds: those participate in model generation,
    cache validation and LQR normalization. Both measured and physical plants
    need the operating cap because StateMirror.follow re-applies prior commands.
    """
    cap = float(cap)
    if cap not in CAPS:
        raise ValueError('Undeclared operating belt cap')
    designer = workspace.design.designer
    objects = (designer, physical, mirror)
    if len({id(p) for p in objects}) != 3:
        raise AssertionError('Design, physical and mirror plants must be separate objects')
    for p in objects:
        np.testing.assert_array_equal(p.bounds, BASE_BOUNDS)
        if p.config['actuation']['belt_speed_limit_m_s'] != .3:
            raise AssertionError('Configuration must retain the original .30 design capacity')
    if any(np.shares_memory(a.bounds, b.bounds) for i, a in enumerate(objects) for b in objects[i+1:]):
        raise AssertionError('Plant bounds must not alias')
    original_config = digest(workspace.config)
    if any(digest(p.config) != original_config for p in objects):
        raise AssertionError('All generated models must use the same original configuration')
    physical.bounds[3:] = cap
    mirror.bounds[3:] = cap
    verify_operating_cap(workspace, physical, mirror, cap, original_config)
    return original_config


def verify_operating_cap(workspace, physical, mirror, cap, original_config=None):
    expected = BASE_BOUNDS.copy()
    expected[3:] = float(cap)
    np.testing.assert_array_equal(workspace.design.designer.bounds, BASE_BOUNDS)
    np.testing.assert_array_equal(physical.bounds, expected)
    np.testing.assert_array_equal(mirror.bounds, expected)
    if original_config is not None:
        assert digest(workspace.config) == original_config
        assert all(digest(p.config) == original_config for p in (workspace.design.designer, physical, mirror))
    return expected


def design_identity(design):
    """Hash actual numeric gain bank/equilibria, independent of cache filename."""
    points = [{key: point[key] for key in ('phase', 'K', 'eq', 'regime', 'side')} for point in design.points]
    return dict(gain_bank_sha256=digest(points), design_bounds=design.designer.bounds.copy(),
        design_config_sha256=digest(design.designer.config), weights=design.weights)


def ordinary_identity(ordinary):
    # The complete design also contains complex eigenvalues; hash exactly the
    # equilibrium/reference and real matrices used by live command generation.
    return digest(dict(equilibrium=ordinary['equilibrium'],
        design={key: ordinary['design'][key] for key in ('K', 'A', 'B', 'state_scales', 'keep')}))


def preflight(output):
    """Read-only registrations/hashes/config/cache, without model construction."""
    protocol = json.loads((output/'protocol.json').read_text())
    jobs_path = output/'campaign/jobs.json'
    jobs = json.loads(jobs_path.read_text())['cases'] if jobs_path.is_file() else protocol['cases']
    assert protocol['iteration'] == 'I-52'
    assert [j['case_id'] for j in jobs] == list(CASE_CAPS)
    for job in jobs:
        validate_case(job['case_id'], job['cap_m_s'], jobs)
    assert jobs == protocol['cases']
    for relative, expected in protocol['source_sha256'].items():
        assert sha(ROOT/relative) == expected, f'Protected source changed: {relative}'
    for key, expected in protocol['input_sha256'].items():
        assert sha(ROOT/protocol['inputs'][key]) == expected, f'Baseline input changed: {key}'
    execution = protocol['execution']
    seed = ROOT/execution['gain_cache_path']
    assert sha(seed) == execution['gain_cache_sha256']
    from spherical_wheel.model import load_config
    config = load_config()
    assert config['actuation']['belt_speed_limit_m_s'] == .3
    np.testing.assert_array_equal([*config['actuation']['torque_limits_Nm'], .3, .3], BASE_BOUNDS)
    helper = load_helper()
    from spherical_wheel.offroad_control import OffroadParams
    parameters = safe(OffroadParams(**helper.PARAMETERS).as_dict())
    assert parameters == safe(OffroadParams(**execution['parameters']).as_dict())
    assert parameters == json.loads((ROOT/protocol['inputs']['historical_provenance']).read_text())['parameters']
    assert helper.PARAMETERS == dict(mode='hardware', rolling_allocation='least_squares', track_contact_point=False)
    from spherical_wheel.gazebo.bridge import plugin_path
    if 'bridge_binary_sha256' in execution:
        assert sha(plugin_path()) == execution['bridge_binary_sha256']
    cache = np.load(seed, allow_pickle=False)
    assert cache['K'].shape[1:] == (5, 18) and np.isfinite(cache['K']).all()
    cache.close()
    report = dict(passed=True, scope='Read-only registrations, hashes and cache arrays. No model construction, equilibrium calculation or integration.',
        protocol_sha256=sha(output/'protocol.json'), gain_cache_sha256=sha(seed), parameters=parameters,
        verified_source_count=len(protocol['source_sha256']), verified_baseline_inputs=len(protocol['input_sha256']))
    return protocol, jobs, helper, report


def snapshot(directory, output, job, protocol, helper):
    paths = {ROOT/p for p in protocol['source_sha256']}
    paths.update((ROOT/'src/spherical_wheel').rglob('*.py'))
    paths.update((ROOT/'config').glob('*.yaml'))
    paths.update((ROOT/'gazebo/plugin').rglob('*.cc'))
    paths.update([Path(__file__).resolve(), ROOT/'scripts/run-lqr-contact-forecast.py',
        ROOT/protocol['execution']['gain_cache_path'], output/'protocol.json'])
    paths.update(p for p in (output/'campaign/jobs.json', output/'verify_capacity_runner.py') if p.is_file())
    sources = []
    for source in sorted(paths):
        relative = source.relative_to(ROOT)
        target = directory/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
        sources.append(dict(origin=str(relative), snapshot=str(target.relative_to(directory)), sha256=sha(source)))
    from spherical_wheel.gazebo.bridge import plugin_path
    record = dict(iteration='I-52', captured_before_execution_utc=datetime.now(timezone.utc).isoformat(),
        case=job, argv=sys.argv, process=dict(pid=os.getpid(), one_case_per_fresh_process=True),
        sources=sources, protocol_sha256=sha(output/'protocol.json'), versions=helper.versions(),
        input_sha256=protocol['input_sha256'], input_paths=protocol['inputs'],
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        parameters=parameters_for_job(helper, job), bridge_binary=dict(path=str(plugin_path().relative_to(ROOT)), sha256=sha(plugin_path())),
        measurement_boundary='Native hardware mode uses exact simulated qpos/qvel and known previous applied commands; observer-only contacts. Solved contacts are evaluator-only after guarded command.',
        scope='Whole-run factorial normal-phase approximation and reference guard; original flat gain bank, no runtime/config/default edits.',
        variant=job['variant'])
    save_json(directory/'provenance.json', record)
    return record


def audit_parameters(protocol, helper):
    from spherical_wheel.offroad_control import OffroadParams
    expected = safe(OffroadParams(**protocol['execution']['parameters']).as_dict())
    assert expected == safe(OffroadParams(**helper.PARAMETERS).as_dict())
    return expected


def parameters_for_job(helper, job):
    from spherical_wheel.offroad_control import OffroadParams
    values = dict(helper.PARAMETERS)
    values['schedule_on_ground_normal'] = job['normal_phase_enabled']
    return safe(OffroadParams(**values).as_dict())


def run_case(output, case_id, cap):
    protocol, jobs, helper, audit = preflight(output)
    job = validate_case(case_id, cap, jobs)
    import spherical_wheel.offroad_experiments as experiments
    import spherical_wheel.gazebo.mars as mars
    from spherical_wheel.gazebo.experiments import run_gazebo
    from spherical_wheel.offroad_control import OffroadController, OffroadParams
    assert mars._WORKSPACE is None, 'One case requires one fresh process'
    directory = output/'campaign/runs'/case_id
    if directory.exists():
        raise FileExistsError(f'Preserve previous case: {directory}')
    directory.mkdir(parents=True)
    provenance = snapshot(directory, output, job, protocol, helper)
    rows, truth, candidate_diagnostics = [], [], []
    workspace = None
    clock_start = time.monotonic()
    try:
        workspace, terrain = mars._workspace(output/'campaign/_workspaces'/case_id, [job['path']['route']])
        seed_hash = protocol['execution']['gain_cache_sha256']
        assert sha(workspace.cache_path) == seed_hash, 'Standard gain seed unexpectedly rebuilt'
        fixed_design = design_identity(workspace.design)
        assert workspace.design.metadata['cache_loaded'] is True
        physical, mirror = workspace.plant(terrain), workspace.mirror()
        original_config = apply_operating_cap(workspace, physical, mirror, cap)
        assert design_identity(workspace.design)['gain_bank_sha256'] == fixed_design['gain_bank_sha256']
        path = helper.make_path(job)
        original_command = OffroadController.command
        original_feedback = OffroadController._terrain_feedback
        component = load_component('candidate/scheduling_reference.py', '_i52_scheduling_reference')
        reference_capture = {}

        def feedback(controller, qref, vref, feedforward, normal, measurement, touching):
            reference_capture['native'] = np.array(vref[6:9], copy=True)
            diagnostic = component.prepare(controller, vref, normal,
                reference_guard=job['reference_guard_enabled'], normal_phase=job['normal_phase_enabled'], touching=touching)
            reference_capture['diagnostics'] = diagnostic
            reference_capture['guarded'] = np.array(vref[6:9], copy=True)
            result = original_feedback(controller, qref, vref, feedforward, normal, measurement, touching)
            component.validate_feedback(diagnostic, result[1], controller)
            return result
        evaluate = experiments.measure_contact
        fixed_ordinary_identity = None

        def measured(plant):
            measurement = evaluate(plant)
            truth.append(helper.truth_contact_row(plant, measurement))
            return measurement

        def command(controller, xy, velocity, acceleration):
            nonlocal fixed_ordinary_identity
            assert controller.plant is mirror and controller.design is workspace.design
            assert mirror.ground is None and mirror.terrain is None and mirror.data.ncon == 0
            assert controller._tilt_reference is None and controller.rolling_allocator is None
            assert controller.params.synchronize and controller.params.heading_gain == 0.
            assert controller.params.schedule_on_ground_normal == job['normal_phase_enabled']
            np.testing.assert_array_equal(mirror.last_input, physical.last_input)
            np.testing.assert_array_equal(mirror.data.qpos, physical.data.qpos)
            np.testing.assert_array_equal(mirror.data.qvel, physical.data.qvel)
            assert float(mirror.data.time) == float(physical.data.time)
            verify_operating_cap(workspace, physical, mirror, cap)
            current_ordinary = ordinary_identity(controller.ordinary)
            if fixed_ordinary_identity is None:
                fixed_ordinary_identity = current_ordinary
            assert current_ordinary == fixed_ordinary_identity
            before_nominal, previous = controller.nominal_joints.copy(), mirror.last_input.copy()
            with ExitStack() as stack:
                stack.enter_context(helper.guards())
                stack.enter_context(patch.object(physical, 'contact_records', helper.forbidden))
                stack.enter_context(patch.object(OffroadController, '_terrain_feedback', feedback))
                applied, info = original_command(controller, xy, velocity, acceleration)
            np.testing.assert_array_equal(applied, np.clip(info['requested'], -mirror.bounds, mirror.bounds))
            np.testing.assert_array_equal(info['input_bounds'], physical.bounds)
            native_applied = applied.copy()
            native_clipping = np.asarray(info['clipping_command_delta']).copy()
            native_joint = reference_capture['native']
            guarded_joint = np.asarray(info['reference_qvel'])[6:9]
            candidate_diagnostics.append(dict(time_s=float(mirror.data.time),
                factors=reference_capture['diagnostics']))
            rotation = mirror.data.xmat[controller.carrier_id].reshape(3,3)
            vertical = rotation.T @ np.array([0.,0.,1.])
            located = rotation.T @ np.asarray(info['estimate']['normal'])
            row = dict(time_s=float(mirror.data.time), exact_qpos=mirror.data.qpos.copy(),
                exact_qvel=mirror.data.qvel.copy(), exact_applied=applied.copy(), exact_previous_applied=previous,
                exact_native_requested=np.asarray(info['requested']).copy(),
                exact_observer_wrench=np.r_[controller.observer.wrench()],
                exact_observer_generalized=controller.observer.generalized.copy(),
                exact_observer_interval_mean=controller.observer.interval_mean.copy(),
                exact_estimated_normal=np.asarray(info['estimate']['normal']).copy(),
                exact_nominal_joints_before=before_nominal, exact_reference_xy=np.asarray(xy).copy(),
                exact_reference_velocity_m_s=np.asarray(velocity).copy(),
                exact_reference_acceleration_m_s2=np.asarray(acceleration).copy(),
                exact_reference_quat=np.asarray(info['reference_quat']).copy(),
                exact_effective_phase_rad=float(info['effective_phase_rad']),
                exact_phase_world_vertical_rad=float(np.arctan2(-vertical[0],vertical[2])),
                exact_phase_estimated_normal_rad=float(np.arctan2(-located[0],located[2])),
                exact_estimated_normal_carrier=located.copy(),
                exact_world_vertical_carrier=vertical.copy(),
                exact_phase_normal_xz_norm=float(np.linalg.norm(located[[0,2]])),
                exact_phase_world_xz_norm=float(np.linalg.norm(vertical[[0,2]])),
                exact_schedule_on_ground_normal=bool(job['normal_phase_enabled']),
                exact_reference_guard_enabled=bool(job['reference_guard_enabled']),
                cap_m_s=float(cap), mirror_previous_applied_matches_physical=True,
                exact_native_applied=native_applied,
                exact_native_clipping_command_delta=native_clipping,
                exact_handoff_command_delta=applied-native_applied,
                exact_native_joint_reference=np.array(native_joint, copy=True),
                exact_guarded_joint_reference=np.array(guarded_joint, copy=True),
                exact_reference_guard_delta=np.array(guarded_joint-native_joint, copy=True))
            row.update({'exact_'+key: np.asarray(info[key]).copy() for key in EXACT_INFO_FIELDS})
            # Explicit error alias matches earlier diagnostic campaign schemas.
            row['exact_feedback_error'] = row['exact_error'].copy()
            rows.append(row)
            if len(rows) % 400 == 0:
                print(json.dumps(dict(case_id=case_id, time_s=row['time_s'], elapsed_s=time.monotonic()-clock_start)), flush=True)
            return applied, info

        with patch.object(OffroadController, 'command', command), patch.object(experiments, 'measure_contact', measured):
            log, summary = run_gazebo(workspace, terrain, path, OffroadParams(**parameters_for_job(helper, job)),
                phase=job['phase_rad'], yaw_deg=job['yaw_deg'], duration_s=job['duration_s'], record_diagnostics=True)
        assert len(rows) == len(truth) == len(log['time_s'])
        extra = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
        extra.update({key: np.asarray([row[key] for row in truth]) for key in truth[0]})
        np.testing.assert_array_equal(extra['time_s'], extra['truth_time_s'])
        np.testing.assert_array_equal(extra['exact_previous_applied'][1:], extra['exact_applied'][:-1])
        summary = mars.evaluate_route(log, summary, path, terrain, mars.route_closed(job['path']['route']))
        summary['contact_forecast_case'] = dict(case=job, actual_path=path.spec(), activation={},
            guarded_controller_calls=len(rows), forecast_affects_control=False)
        summary['contact_scheduling_joint_tracking'] = dict(iteration='I-52', variant=job['variant'],
            normal_phase_enabled=job['normal_phase_enabled'], reference_guard_enabled=job['reference_guard_enabled'],
            operating_belt_cap_m_s=cap,
            design_belt_cap_m_s=.3, fixed_design=fixed_design, ordinary_design_sha256=fixed_ordinary_identity,
            gain_cache_sha256=seed_hash, configuration_unchanged=True,
            synchronization_uses_operating_bounds=True, scope='Experimental mechanism isolation; not default adoption.')
        experiments.save_run(directory, log, summary, terrain)
        np.savez_compressed(directory/'precision.npz', **extra)
        save_json(directory/'candidate_diagnostics.json', candidate_diagnostics)
        verify_operating_cap(workspace, physical, mirror, cap, original_config)
        assert design_identity(workspace.design)['gain_bank_sha256'] == fixed_design['gain_bank_sha256']
        assert sha(workspace.cache_path) == seed_hash
        for source in provenance['sources']:
            assert sha(ROOT/source['origin']) == source['sha256'], f'Execution source changed: {source["origin"]}'
        provenance.update(actual_path=path.spec(), model_mass_kg=physical.total_mass, weight_N=physical.total_mass*9.81,
            input_bounds=physical.bounds.copy(), mirror_input_bounds=mirror.bounds.copy(), design=fixed_design,
            ordinary_design_sha256=fixed_ordinary_identity, initial_nominal_joints=extra['exact_nominal_joints_before'][0],
            controller_dt_s=physical.dt, physics_step_s=.001, engine=summary['engine'],
            gain_cache_sha256=seed_hash, preflight=audit, source_hashes_unchanged_after=True,
            elapsed_s=time.monotonic()-clock_start,
            generated_sha256={name: sha(directory/name) for name in ('rollout.npz', 'precision.npz', 'summary.json', 'terrain.json')})
        save_json(directory/'provenance.json', provenance)
        result = dict(iteration='I-52', case_id=case_id, cap_m_s=cap, completed=summary['completed'],
            failure=summary['failure'], duration_s=summary['completed_duration_s'],
            guarded_calls=len(rows), directory=str(directory.relative_to(output)), elapsed_s=time.monotonic()-clock_start)
        save_json(directory/'execution_result.json', result)
        print(json.dumps(result), flush=True)
    except Exception:
        save_json(directory/'execution_failure.json', dict(traceback=traceback.format_exc(), rows=len(rows),
            case_id=case_id, cap_m_s=cap, elapsed_s=time.monotonic()-clock_start))
        if rows:
            np.savez_compressed(directory/'partial_precision.npz', **{key: np.asarray([row[key] for row in rows]) for key in rows[0]})
        raise
    finally:
        if workspace is not None and workspace._current is not None:
            workspace._current[1].close()
        save_json(directory/'artifact_manifest.json', dict(iteration='I-52', case_id=case_id,
            files=[dict(path=str(p.relative_to(directory)), bytes=p.stat().st_size, sha256=sha(p))
                for p in sorted(directory.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE)
    parser.add_argument('--cap', type=float)
    parser.add_argument('--case-id', choices=tuple(CASE_CAPS))
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--preflight', action='store_true')
    mode.add_argument('--run', action='store_true')
    args = parser.parse_args()
    if args.preflight:
        print(json.dumps(preflight(args.output.resolve())[3], indent=2))
    elif args.cap is None or args.case_id is None:
        parser.error('--run requires --cap and --case-id')
    else:
        run_case(args.output.resolve(), args.case_id, args.cap)


if __name__ == '__main__':
    main()
