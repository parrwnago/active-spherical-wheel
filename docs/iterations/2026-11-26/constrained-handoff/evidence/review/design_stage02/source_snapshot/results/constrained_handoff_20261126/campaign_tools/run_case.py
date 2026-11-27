"""I56 one-case native/handoff comparison with a frozen flat gain bank.

This is a NEW experiment runner. It does not rewrite or bypass an I52 source
preflight. The registered current sources are checked before and after each
run; historical baseline data is evaluation-only. One fresh process per case.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
DEFAULT_OUTPUT = HERE.parent
SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
BOUNDS = np.array([1.5, .6, .6, .3, .3])
FROZEN_BANK_SHA256 = '45ab9990aa3961d1202533acf51d5d8a9294eefe5a4c7e6cfde6aa0b3ac462c1'
EXACT_INFO_FIELDS = ('feedback_gain', 'error', 'reference_qpos', 'reference_qvel',
    'feedback_reference_qvel', 'equilibrium_command', 'rolling_feedforward',
    'support_command', 'lqr_command', 'transition_command_delta', 'heading_command',
    'fade_command_delta', 'clipping_command_delta', 'sync_weights', 'input_bounds',
    'estimated_support_loads_N', 'estimated_contact_point_m', 'allocation_drive_reference')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def save(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def fingerprint(value):
    return hashlib.sha256(json.dumps(safe(value), sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def control_boundary(controller, info):
    """Protect native targets/observer history from a command-only candidate."""
    plant = controller.plant
    return fingerprint(dict(info=info, qpos=plant.data.qpos, qvel=plant.data.qvel,
        time_s=float(plant.data.time), previous_applied=plant.last_input,
        nominal_joints=controller.nominal_joints, held=controller._held,
        vref=controller._vref, sync=controller._sync, plane=controller._plane,
        parameters=controller.params.as_dict(), estimator=vars(controller.estimator),
        observer_generalized=controller.observer.generalized,
        observer_interval_mean=controller.observer.interval_mean,
        observer_previous=controller.observer._previous,
        model_surface_velocity=plant.model.geom_surfacevel))


def preflight(output, case_id=None):
    protocol = json.loads((output/'protocol.json').read_text())
    if protocol['iteration'] != 'I-56':
        raise ValueError('This runner accepts an I-56 registration only')
    cases = protocol['cases']
    if len({case['case_id'] for case in cases}) != len(cases):
        raise ValueError('Case IDs must be unique')
    for job in cases:
        if (job['variant'] not in ('native', 'handoff') or
                job['terrain'] != 'marsyard2021' or
                job['path'] != {'kind': 'mars_route', 'route': 'plain_figure8'} or
                float(job['duration_s']) != 44. or float(job['yaw_deg']) != 45. or
                float(job['phase_rad']) not in (-1., .3, .9)):
            raise ValueError('Unregistered main-comparison condition')
    for relative, expected in protocol['source_sha256'].items():
        if sha(ROOT/relative) != expected:
            raise ValueError(f'Registered current source changed: {relative}')
    for relative, expected in protocol.get('input_sha256', {}).items():
        if sha(ROOT/relative) != expected:
            raise ValueError(f'Registered input changed: {relative}')
    seed = ROOT/protocol['gain_cache']['path']
    if (protocol['gain_cache']['sha256'] != FROZEN_BANK_SHA256 or
            sha(seed) != FROZEN_BANK_SHA256):
        raise ValueError('The original flat gain-bank bytes changed')
    from spherical_wheel.gazebo.bridge import plugin_path
    plugin = plugin_path()
    if sha(plugin) != protocol['bridge_binary_sha256']:
        raise ValueError('Registered bridge binary changed')
    adapter = ROOT/protocol['candidate_adapter']['path']
    if sha(adapter) != protocol['candidate_adapter']['sha256']:
        raise ValueError('Candidate adapter source changed')
    from spherical_wheel.offroad_control import OffroadParams
    params = OffroadParams(**protocol['parameters'])
    if params.mode != 'hardware' or params.rolling_allocation != 'least_squares':
        raise ValueError('The measurement boundary and native allocation must remain fixed')
    if params.schedule_on_ground_normal or params.lqr_weights != 'flat':
        raise ValueError('I56 main comparison keeps the original flat bank/phase')
    if params.tilt_reference is not None:
        raise ValueError('No simultaneous attitude-target change is registered')
    if case_id is None:
        job = None
    else:
        matches = [job for job in cases if job['case_id'] == case_id]
        if len(matches) != 1:
            raise ValueError('Choose exactly one registered case')
        job = matches[0]
    audit = dict(passed=True, scope='Read-only protocol, sources, plugin and frozen-bank checks; no physics or gain design.',
        protocol_sha256=sha(output/'protocol.json'), frozen_bank_sha256=sha(seed),
        verified_source_count=len(protocol['source_sha256']), verified_input_count=len(protocol.get('input_sha256', {})),
        resolved_parameters=params.as_dict())
    return protocol, job, audit


def design_identity(design):
    return fingerprint([{key: point[key] for key in ('phase', 'K', 'eq', 'regime', 'side')}
                        for point in design.points])


def snapshot(directory, output, protocol, job, audit):
    paths = {ROOT/p for p in protocol['source_sha256']}
    paths.update((ROOT/'src/spherical_wheel').rglob('*.py'))
    paths.update((ROOT/'config').glob('*.yaml'))
    paths.update((ROOT/'gazebo/plugin').rglob('*.cc'))
    paths.update([Path(__file__).resolve(), output/'protocol.json',
                  ROOT/protocol['candidate_adapter']['path'], ROOT/protocol['gain_cache']['path']])
    sources = []
    for path in sorted(paths):
        relative = path.relative_to(ROOT)
        target = directory/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        sources.append(dict(origin=str(relative), snapshot=str(target.relative_to(directory)), sha256=sha(path)))
    return dict(iteration='I-56', captured_before_execution_utc=datetime.now(timezone.utc).isoformat(),
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        case=job, process=dict(pid=os.getpid(), one_case_per_fresh_process=True),
        sources=sources, preflight=audit, candidate_settings=protocol['candidate_settings'],
        measurement_boundary='Exact simulated body/joint state and known prior actual applied input. Control contact records come only from momentum observer and support geometry; solver contacts are evaluation-only.',
        scope='New total-command handoff experiment. Same raw target, native LQR bank/phase, observer, preview, mechanics and input caps; no R reference guard or controller-default adoption.')


def run_case(output, case_id):
    protocol, job, audit = preflight(output, case_id)
    directory = output/'campaign/runs'/case_id
    directory.mkdir(parents=True, exist_ok=False)
    helper = load_module(ROOT/'scripts/run-lqr-contact-forecast.py', '_i56_capture_helper')
    import spherical_wheel.offroad_experiments as experiments
    import spherical_wheel.gazebo.mars as mars
    from spherical_wheel.gazebo.experiments import run_gazebo
    from spherical_wheel.offroad_control import OffroadController, OffroadParams
    assert mars._WORKSPACE is None, 'Use one fresh Python process per case'
    provenance = snapshot(directory, output, protocol, job, audit)
    save(directory/'provenance_before.json', provenance)
    rows, truth, governed = [], [], []
    workspace = None
    started = time.monotonic()
    try:
        workspace, terrain = mars._workspace(output/'campaign/_workspaces'/case_id, [job['path']['route']])
        assert sha(workspace.cache_path) == protocol['gain_cache']['sha256'], 'Seed bank unexpectedly rebuilt'
        assert workspace.design.metadata['cache_loaded'] is True
        design_hash = design_identity(workspace.design)
        physical, mirror = workspace.plant(terrain), workspace.mirror()
        np.testing.assert_array_equal(physical.bounds, BOUNDS)
        np.testing.assert_array_equal(mirror.bounds, BOUNDS)
        path = helper.make_path(job)
        native_command = OffroadController.command
        evaluate = experiments.measure_contact
        adapter = (load_module(ROOT/protocol['candidate_adapter']['path'], '_i56_registered_adapter')
                   if job['variant'] == 'handoff' else None)
        instance = None
        ordinary_hash = None

        def measured(plant):
            value = evaluate(plant)
            truth.append(helper.truth_contact_row(plant, value))
            return value

        def command(controller, xy, velocity, acceleration):
            nonlocal instance, ordinary_hash
            assert controller.plant is mirror and controller.design is workspace.design
            assert mirror.ground is None and mirror.terrain is None and mirror.data.ncon == 0
            np.testing.assert_array_equal(mirror.last_input, physical.last_input)
            np.testing.assert_array_equal(mirror.data.qpos, physical.data.qpos)
            np.testing.assert_array_equal(mirror.data.qvel, physical.data.qvel)
            assert mirror.data.time == physical.data.time
            current_ordinary = fingerprint(dict(equilibrium=controller.ordinary['equilibrium'],
                design={k: controller.ordinary['design'][k] for k in ('K', 'A', 'B', 'state_scales', 'keep')}))
            if ordinary_hash is None:
                ordinary_hash = current_ordinary
            assert current_ordinary == ordinary_hash
            nominal, previous = controller.nominal_joints.copy(), mirror.last_input.copy()
            with ExitStack() as stack:
                stack.enter_context(helper.guards())
                stack.enter_context(patch.object(physical, 'contact_records', helper.forbidden))
                native_applied, native_info = native_command(controller, xy, velocity, acceleration)
                native_requested = np.asarray(native_info['requested']).copy()
                native_applied = np.asarray(native_applied).copy()
                np.testing.assert_array_equal(native_applied, np.clip(native_requested, -mirror.bounds, mirror.bounds))
                if adapter is not None:
                    boundary_before = control_boundary(controller, native_info)
                    if instance is None:
                        instance = adapter.create(controller, protocol['candidate_settings'])
                    final_requested, final_applied, extra = adapter.apply(instance, controller,
                        native_requested.copy(), native_applied.copy(), native_info)
                    final_requested, final_applied = np.asarray(final_requested, float), np.asarray(final_applied, float)
                    if control_boundary(controller, native_info) != boundary_before:
                        raise AssertionError('Command-only candidate mutated native targets, observer or controller/plant history')
                    # Fail before integration if a diagnostic payload is not serializable.
                    json.dumps(safe(extra), allow_nan=False)
                else:
                    final_requested, final_applied, extra = native_requested.copy(), native_applied.copy(), {}
            if (final_requested.shape != (5,) or final_applied.shape != (5,) or
                    not np.isfinite(np.r_[final_requested, final_applied]).all()):
                raise ValueError('Candidate must return finite requested/applied command[5]')
            np.testing.assert_array_equal(final_applied, np.clip(final_requested, -mirror.bounds, mirror.bounds))
            info = dict(native_info)
            info['requested'] = final_requested.copy()
            info['clipping_command_delta'] = final_applied-final_requested
            info['actuator_saturated'] = abs(final_requested) > mirror.bounds+1e-12
            rotation = mirror.data.xmat[controller.carrier_id].reshape(3, 3)
            vertical = rotation.T@np.array([0., 0., 1.])
            normal_local = rotation.T@np.asarray(native_info['estimate']['normal'])
            joint_ref = np.asarray(native_info['reference_qvel'])[6:9].copy()
            row = dict(time_s=float(mirror.data.time), exact_qpos=mirror.data.qpos.copy(), exact_qvel=mirror.data.qvel.copy(),
                exact_applied=final_applied.copy(), exact_previous_applied=previous,
                exact_native_requested=native_requested, exact_native_applied=native_applied,
                exact_requested=final_requested.copy(), exact_handoff_request_delta=final_requested-native_requested,
                exact_handoff_command_delta=final_applied-native_applied,
                exact_native_clipping_command_delta=native_applied-native_requested,
                exact_native_saturation_mask=abs(native_requested)>mirror.bounds+1e-12,
                exact_observer_wrench=np.r_[controller.observer.wrench()],
                exact_observer_generalized=controller.observer.generalized.copy(),
                exact_observer_interval_mean=controller.observer.interval_mean.copy(),
                exact_estimated_normal=np.asarray(native_info['estimate']['normal']).copy(),
                exact_nominal_joints_before=nominal, exact_reference_xy=np.asarray(xy).copy(),
                exact_reference_velocity_m_s=np.asarray(velocity).copy(), exact_reference_acceleration_m_s2=np.asarray(acceleration).copy(),
                exact_reference_quat=np.asarray(native_info['reference_quat']).copy(),
                exact_effective_phase_rad=float(native_info['effective_phase_rad']),
                exact_phase_world_vertical_rad=float(np.arctan2(-vertical[0], vertical[2])),
                exact_phase_estimated_normal_rad=float(np.arctan2(-normal_local[0], normal_local[2])),
                exact_estimated_normal_carrier=normal_local.copy(), exact_world_vertical_carrier=vertical.copy(),
                exact_phase_normal_xz_norm=float(np.linalg.norm(normal_local[[0, 2]])),
                exact_phase_world_xz_norm=float(np.linalg.norm(vertical[[0, 2]])),
                exact_schedule_on_ground_normal=bool(controller.params.schedule_on_ground_normal),
                exact_reference_guard_enabled=False, cap_m_s=.3, mirror_previous_applied_matches_physical=True,
                exact_native_joint_reference=joint_ref.copy(), exact_guarded_joint_reference=joint_ref.copy(),
                exact_reference_guard_delta=np.zeros(3))
            row.update({'exact_'+key: np.asarray(info[key]).copy() for key in EXACT_INFO_FIELDS})
            row['exact_feedback_error'] = row['exact_error'].copy()
            rows.append(row)
            governed.append(dict(time_s=float(mirror.data.time), native_requested=native_requested,
                native_applied=native_applied, final_requested=final_requested.copy(), final_applied=final_applied.copy(),
                native_saturation=abs(native_requested)>mirror.bounds+1e-12,
                diagnostics=safe(extra)))
            if len(rows) % 400 == 0:
                print(json.dumps(dict(case_id=case_id, time_s=float(mirror.data.time), elapsed_s=time.monotonic()-started)), flush=True)
            return final_applied, info

        with patch.object(OffroadController, 'command', command), patch.object(experiments, 'measure_contact', measured):
            log, summary = run_gazebo(workspace, terrain, path, OffroadParams(**protocol['parameters']),
                phase=job['phase_rad'], yaw_deg=job['yaw_deg'], duration_s=job['duration_s'], record_diagnostics=True)
        assert len(rows) == len(truth) == len(log['time_s'])
        exact = {k: np.asarray([row[k] for row in rows]) for k in rows[0]}
        exact.update({k: np.asarray([row[k] for row in truth]) for k in truth[0]})
        np.testing.assert_array_equal(exact['time_s'], exact['truth_time_s'])
        np.testing.assert_array_equal(exact['exact_previous_applied'][1:], exact['exact_applied'][:-1])
        np.testing.assert_allclose(exact['exact_native_requested']+exact['exact_handoff_request_delta'], exact['exact_requested'], atol=1e-12, rtol=0.)
        summary = mars.evaluate_route(log, summary, path, terrain, mars.route_closed(job['path']['route']))
        summary['constrained_handoff_experiment'] = dict(iteration='I-56', case=job,
            candidate_settings=protocol['candidate_settings'] if adapter is not None else None,
            source_boundary='observer-only contacts; body state exact simulation', default_adoption=False,
            frozen_design_bank_hash=design_hash, frozen_cache_sha256=protocol['gain_cache']['sha256'])
        experiments.save_run(directory, log, summary, terrain)
        np.savez_compressed(directory/'precision.npz', **exact)
        save(directory/'governor_diagnostics.json', governed)
        historical = protocol.get('historical_baseline_paths', {}).get(case_id)
        if job['variant'] == 'native' and historical is not None:
            historical = ROOT/historical
            with np.load(historical, allow_pickle=False) as old:
                fields = sorted(set(exact) & set(old.files))
                identical = {name: bool(np.array_equal(exact[name], old[name], equal_nan=True))
                             for name in fields}
            comparison = dict(scope='New native run versus the registered historical B dataset; no candidate outcome substitution.',
                historical_path=str(historical.relative_to(ROOT)), historical_sha256=sha(historical),
                shared_precision_fields=len(fields), exact_fields=sum(identical.values()),
                all_shared_fields_exact=all(identical.values()), fields=identical)
            save(directory/'historical_precision_comparison.json', comparison)
            if protocol.get('require_historical_baseline_exact', True) and not comparison['all_shared_fields_exact']:
                raise AssertionError('New baseline does not reproduce the registered historical trace; preserve the evidence before interpretation')
        assert sha(workspace.cache_path) == protocol['gain_cache']['sha256']
        assert design_identity(workspace.design) == design_hash
        for source in provenance['sources']:
            assert sha(ROOT/source['origin']) == source['sha256'], source['origin']
        provenance.update(completed=bool(summary['completed']), failure=summary['failure'],
            actual_duration_s=float(summary['completed_duration_s']), elapsed_s=time.monotonic()-started,
            guarded_calls=len(rows), parameters=OffroadParams(**protocol['parameters']).as_dict(),
            ordinary_design_sha256=ordinary_hash, fixed_bank_numeric_sha256=design_hash,
            fixed_bank_bytes_sha256=sha(workspace.cache_path), frozen_bank_loaded=True,
            models=dict(mirror_xml=str(workspace.directory/'mirror/model.xml'),
                        physical_sdf=str(physical.world_path.parent/'models/active_spherical_wheel/model.sdf')),
            versions=helper.versions(), source_hashes_unchanged_after=True)
        save(directory/'provenance.json', provenance)
        result = dict(iteration='I-56', case_id=case_id, completed=bool(summary['completed']),
            failure=summary['failure'], actual_duration_s=float(summary['completed_duration_s']),
            guarded_calls=len(rows), elapsed_s=time.monotonic()-started)
        save(directory/'execution_result.json', result)
        print(json.dumps(result), flush=True)
    except BaseException:
        save(directory/'failure.json', dict(case_id=case_id, rows=len(rows), traceback=traceback.format_exc(),
            elapsed_s=time.monotonic()-started))
        if rows:
            np.savez_compressed(directory/'partial_precision.npz', **{k: np.asarray([r[k] for r in rows]) for k in rows[0]})
        if governed:
            save(directory/'partial_governor_diagnostics.json', governed)
        raise
    finally:
        if workspace is not None and workspace._current is not None:
            workspace._current[1].close()
        save(directory/'artifact_manifest.json', dict(iteration='I-56', case_id=case_id,
            files=[dict(path=str(p.relative_to(directory)), bytes=p.stat().st_size, sha256=sha(p))
                   for p in sorted(directory.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--preflight', action='store_true')
    mode.add_argument('--run', action='store_true')
    parser.add_argument('--case-id')
    args = parser.parse_args()
    output = args.output.resolve()
    if args.preflight:
        print(json.dumps(safe(preflight(output, args.case_id)[2]), indent=2))
    elif args.case_id is None:
        parser.error('--run requires --case-id')
    else:
        run_case(output, args.case_id)


if __name__ == '__main__':
    main()
