"""Independent recorded-field and error-algebra audit of I77's final hold arm.

No integration, observer update, controller invocation or new physical trial.
The primary 96.900 s state precedes the released command on that row.  Its
fixed held target is evaluated separately from the row's emitted target.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse, hashlib, importlib.util, json, shutil
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]
OLD = ROOT / 'results/tangent_heading_routes_20261127'
CASE = 'eight_m100_N'
START, STOP, SOURCE, LAST = 19280, 19380, 19279, 20600
ENDPOINTS = (STOP, 19500, 20427, LAST)
STATE_FIELDS = (
    'time_s', 'exact_qpos', 'exact_qvel', 'exact_previous_applied',
    'exact_nominal_joints_before', 'exact_observer_wrench',
    'exact_observer_generalized', 'exact_observer_interval_mean',
    'exact_estimated_normal', 'exact_estimated_contact_point_m',
    'exact_estimated_support_loads_N', 'truth_time_s', 'truth_patch_loads_N',
    'truth_patch_points_m', 'truth_patch_contact_counts',
    'truth_equivalent_point_m', 'exact_reference_qpos',
    'exact_reference_quat', 'exact_reference_qvel',
    'exact_feedback_reference_qvel', 'exact_feedback_gain',
    'exact_support_command', 'exact_rolling_feedforward',
    'exact_effective_phase_rad',
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
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
    with path.open('x') as stream:
        json.dump(safe(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def exact(a, b):
    return (a.dtype == b.dtype and a.shape == b.shape
            and np.ascontiguousarray(a).tobytes() == np.ascontiguousarray(b).tobytes())


def error_from_records(model, q, v, raw_qref, raw_vref, normal, radius):
    """Direct tangent-coordinate algebra from saved arrays; no Plant instance."""
    # Retain scalar exponent operations from the native source: replacing
    # **2 by multiplication changes a few last bits of yaw and tangent error.
    yaw = float(np.arctan2(2*(q[3]*q[6]+q[4]*q[5]),
                          1-2*(q[5]**2+q[6]**2)))
    c, s = np.cos(yaw), np.sin(yaw)
    world_to_heading = np.array([[c, s], [-s, c]])
    target = raw_qref.copy()
    mujoco.mju_mulQuat(target[3:7], raw_qref[3:7].copy(),
                       np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)]))
    target[2] = q[2]
    velocity = raw_vref.copy()
    velocity[2] = -float(normal[:2] @ velocity[:2])/max(normal[2], .2)
    tangent = np.zeros(model.nv)
    mujoco.mj_differentiatePos(model, tangent, 1., target, q)
    error = np.r_[tangent, v-velocity]
    error[:2] = world_to_heading @ error[:2]
    error[9:11] = world_to_heading @ error[9:11]
    error[6:9] = (error[6:9]+np.pi) % (2*np.pi)-np.pi
    error[7:9] = 0.
    error[6] = error[0]/radius-error[4]
    return error, velocity


def angle(a, b):
    return np.degrees(np.arccos(np.clip(np.sum(a*b, axis=-1), -1., 1.)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    folder = BASE / 'component_audit/eight_attitude_hold103_stage01'
    precision = folder / 'child/campaign/runs' / CASE / 'precision.npz'
    baseline = OLD / 'campaign/runs' / CASE / 'precision.npz'
    model_path = OLD / 'campaign/_workspaces' / CASE / 'mirror/model.xml'
    config_path = model_path.with_name('model_config.json')
    metric_path = OLD / 'evaluation/metrics.py'
    manifest_path = folder / 'artifact_manifest.json'
    manifest = json.loads(manifest_path.read_text())['files']
    for relative, binding in manifest.items():
        path = folder / relative
        if path.stat().st_size != binding['bytes'] or sha(path) != binding['sha256']:
            raise ValueError('Final branch manifest differs: '+relative)
    seal_path = OLD / 'campaign/outputs_sealed.json'
    binding = json.loads(seal_path.read_text())['files'][str(baseline.relative_to(OLD))]
    if baseline.stat().st_size != binding['bytes'] or sha(baseline) != binding['sha256']:
        raise ValueError('Original full native precision changed')
    branch_source = BASE / 'component_audit/attitude_hold_branch.py'
    if sha(branch_source) != '4f88f48a8223e58f2a7836498aab5495aff96d7bd809ae3b36163578907012b5':
        raise ValueError('Reviewed physical branch source changed')
    inputs = [Path(__file__).resolve(), precision, baseline, seal_path,
              manifest_path, folder/'sidecar.npz', folder/'diagnostic_metadata.json',
              folder/'registered_before_arrays_and_physics.json',
              folder/'full_precision_prefix_receipt.json', model_path, config_path,
              branch_source, BASE/'component_audit/attitude_hold_protocol.json',
              BASE/'component_audit/attitude_hold_prephysics_horizon_amendment.json',
              ROOT/'src/spherical_wheel/sim.py',
              ROOT/'src/spherical_wheel/offroad_control.py', metric_path]
    bindings = {str(p): dict(sha256=sha(p), bytes=p.stat().st_size) for p in inputs}
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    for path in (Path(__file__).resolve(), branch_source, metric_path,
                 ROOT/'src/spherical_wheel/sim.py', ROOT/'src/spherical_wheel/offroad_control.py'):
        destination = out/'source'/path.relative_to(ROOT)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, destination)
    save(out/'registered_before_array_reads.json', dict(
        iteration='I-77', utc=datetime.now(timezone.utc).isoformat(), inputs=bindings,
        no_new_physics=True, no_controller_calls=True, no_observer_updates=True,
        hold_indices=[START, STOP], saved_reference_index=SOURCE,
        complete_prefix_stop_exclusive=START, first_active_state_inclusive=START,
        endpoint_indices=ENDPOINTS, expected_rows=LAST+1,
        target_convention='At96.900 saved state precedes released command; fixed held target evaluated separately from released emitted target.',
        metric='Frozen I76 nearest4097uniform-u polyline, separate timed error.',
        scope='Recorded-field and tangent-error algebra verification of one diagnostic branch; no general controller acceptance or full133s reproduction.'))
    with np.load(baseline, allow_pickle=False) as z:
        old = {key: z[key].copy() for key in z.files}
    with np.load(precision, allow_pickle=False) as z:
        new = {key: z[key].copy() for key in z.files}
    with np.load(folder/'sidecar.npz', allow_pickle=False) as z:
        side = {key: z[key].copy() for key in z.files}
    if set(new) != set(old):
        raise ValueError('Native/branch shared precision schema differs')
    n = len(new['time_s'])
    if n != LAST+1 or abs(float(new['time_s'][-1])-103.) > 1e-8:
        raise ValueError('Branch did not produce declared103s horizon')
    if not exact(new['time_s'], old['time_s'][:n]):
        raise ValueError('Branch clock differs from native clock')
    prefix_checks = [dict(field=k, rows=START, exact=exact(new[k][:START], old[k][:START]))
                     for k in sorted(new)]
    state_checks = [dict(field=k, rows=START+1,
                         exact=exact(new[k][:START+1], old[k][:START+1])) for k in STATE_FIELDS]
    state_checks.append(dict(field='untreated_error_at_first_active', rows=1,
                             exact=exact(side['raw_native_error'][START], old['exact_error'][START])))
    indices = np.arange(n)
    active = (indices >= START) & (indices < STOP)
    expected_quat = new['exact_reference_qpos'][:, 3:7].copy()
    held_quat = old['exact_reference_qpos'][SOURCE, 3:7]
    expected_quat[active] = held_quat
    changed = np.any(expected_quat != new['exact_reference_qpos'][:, 3:7], axis=1)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if (model.nq, model.nv) != (10, 9):
        raise ValueError('Unexpected tangent coordinate layout')
    radius = float(json.loads(config_path.read_text())['geometry']['sphere_radius_m'])
    untreated_error = np.empty((n, 18))
    used_error = np.empty((n, 18))
    feedback_vref = np.empty((n, 9))
    for i in range(n):
        qref = new['exact_reference_qpos'][i].copy()
        untreated_error[i], feedback_vref[i] = error_from_records(
            model, new['exact_qpos'][i], new['exact_qvel'][i], qref,
            new['exact_reference_qvel'][i], new['exact_estimated_normal'][i], radius)
        qref[3:7] = expected_quat[i]
        used_error[i], _ = error_from_records(
            model, new['exact_qpos'][i], new['exact_qvel'][i], qref,
            new['exact_reference_qvel'][i], new['exact_estimated_normal'][i], radius)
    unchanged_columns = [0, 1, 2, 7, 8]+list(range(9, 18))
    lqr = np.array([-K@e for K, e in zip(new['exact_feedback_gain'], used_error)])
    core = (new['exact_equilibrium_command']+new['exact_rolling_feedforward']
            +new['exact_support_command']+lqr)
    requested = (core+new['exact_transition_command_delta']
                 +new['exact_heading_command']+new['exact_fade_command_delta'])
    isolation = dict(
        active_mask_fixed_100rows=exact(side['hold_window_active'], active),
        used_quaternion_matches_frozen_rule=exact(side['used_reference_quat'], expected_quat),
        changed_quaternion_mask_exact=exact(side['reference_changed'], changed),
        raw_quaternion_still_diagnostic_reference=exact(side['raw_reference_quat'], new['exact_reference_quat']),
        raw_qpos_quaternion_still_diagnostic_reference=exact(new['exact_reference_qpos'][:, 3:7], new['exact_reference_quat']),
        raw_velocity_reference_still_diagnostic_reference=exact(side['raw_reference_qvel'], new['exact_reference_qvel']),
        original_native_quaternion_sidecar_exact=exact(side['original_native_reference_quat'], old['exact_reference_quat'][:n]),
        untreated_error_independently_exact=exact(untreated_error, side['raw_native_error']),
        used_error_independently_exact=exact(used_error, side['used_error']),
        emitted_error_matches_used_error=exact(used_error, new['exact_error']),
        unchanged_error_columns_exact=exact(used_error[:, unchanged_columns], untreated_error[:, unchanged_columns]),
        inactive_error_exact=exact(used_error[~active], untreated_error[~active]),
        feedback_velocity_independently_exact=exact(feedback_vref, new['exact_feedback_reference_qvel']),
        used_gain_still_diagnostic_gain=exact(side['feedback_gain'], new['exact_feedback_gain']),
        lqr_command_independently_exact=exact(lqr, new['exact_lqr_command']),
        exactly_one_observer_update=bool(np.all(new['native_observer_update_count'] == 1)),
        no_truth_or_terrain_command_read=bool(np.all(new['no_truth_or_terrain_read_during_command'])),
        zero_heading_command=bool(np.all(new['exact_heading_command'] == 0.)),
        terminal_emitted_command_unheld=not bool(new['command_held_after_row'][-1]),
        original_terminal_row103_command_held=bool(old['command_held_after_row'][LAST]),
    )
    maximum_identity_residual = float(np.max(np.abs(requested-new['exact_requested'])))
    isolation['requested_stage_identity_within_1e_12'] = maximum_identity_residual < 1e-12
    metadata = json.loads((folder/'diagnostic_metadata.json').read_text())
    isolation['metadata_actual_prefix_checks_match'] = (
        metadata['complete_command_prefix_rows_verified'] == START
        and metadata['state_prefix_rows_verified'] == START+1
        and metadata['first_active_stage_checks'] == 1
        and metadata['inactive_native_error_reconstruction_checks'] == n-(STOP-START))
    spec = importlib.util.spec_from_file_location('_i77_independent_attitude_metrics', metric_path)
    metrics = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(metrics)
    original_up = metrics.rotation(old['exact_reference_quat'][:n])[:, :, 2]
    held_up = metrics.rotation(held_quat[None, :])[0, :, 2]
    used_up = metrics.rotation(expected_quat)[:, :, 2]
    path = metrics.geometric_polyline('eight', 4097)
    comparison = {}
    for name, record in [('native', old), ('hold', new)]:
        xy = record['exact_qpos'][:n, :2]
        actual_up = metrics.rotation(record['exact_qpos'][:n, 3:7])[:, :, 2]
        current_up = metrics.rotation(record['exact_reference_quat'][:n])[:, :, 2]
        endpoint_geo = metrics.polyline_distance(xy[list(ENDPOINTS)], path)
        endpoints = {}
        for j, i in enumerate(ENDPOINTS):
            endpoints[str(i)] = dict(
                time_s=record['time_s'][i], geometric_error_m=endpoint_geo[j],
                timed_error_m=np.linalg.norm(xy[i]-record['exact_reference_xy'][i]),
                up_error_to_original_native_target_deg=angle(actual_up[i], original_up[i]),
                up_error_to_fixed_held_target_deg=angle(actual_up[i], held_up),
                up_error_to_current_untreated_target_deg=angle(actual_up[i], current_up[i]),
                up_error_to_emitted_target_deg=angle(actual_up[i], used_up[i] if name == 'hold' else current_up[i]),
                xy_m=xy[i], actual_up=actual_up[i], qvel=record['exact_qvel'][i],
                body_xy_speed_m_s=np.linalg.norm(record['exact_qvel'][i, :2]),
                state_before_current_command=True, full_route_completion_claim=False)
        comparison[name] = endpoints
    def first_changed(key):
        a, b = new[key], old[key][:n]
        different = np.any(a != b, axis=tuple(range(1, a.ndim))) if a.ndim > 1 else a != b
        where = np.flatnonzero(different)
        return None if not len(where) else int(where[0])
    changed_rows = np.flatnonzero(changed)
    result = dict(
        iteration='I-77', passed=all(v['exact'] for v in prefix_checks+state_checks) and all(isolation.values()),
        rows=n, shared_precision_fields=len(new), complete_prefix_checks=prefix_checks,
        first_active_inclusive_state_checks=state_checks, isolation=isolation,
        independent_error_maximum_abs_residual=float(np.max(abs(used_error-new['exact_error']))),
        requested_stage_identity_maximum_abs_residual=maximum_identity_residual,
        reference_changed_rows=len(changed_rows), reference_changed_indices=changed_rows,
        first_different_indices={k: first_changed(k) for k in
            ('exact_qpos', 'exact_qvel', 'exact_reference_quat', 'exact_error',
             'exact_requested', 'exact_applied', 'exact_estimated_normal')},
        first_applied_delta_vs_same_state_native=new['exact_applied'][START]-old['exact_applied'][START],
        first_requested_delta_vs_same_state_native=new['exact_requested'][START]-old['exact_requested'][START],
        primary_endpoint_index=STOP, endpoint_comparison=comparison,
        fixed_held_target_up=held_up,
        released_emitted_target_up=used_up[STOP],
        release_target_step_deg=angle(used_up[STOP], used_up[STOP-1]),
        no_new_physics=True, no_control_source_changes=True,
        limits=[
            'One fixed half-second diagnostic on one start, no general candidate acceptance.',
            'All shared fields reproduce only before intervention; post-branch native is a separate physical trajectory.',
            'The96.900 state precedes release. Later endpoints include the held intervention and the return to native references.',
            'Fixedheld target error is separate from the same saveduntreated native target used for both physical states.',
            'No full133s route completion or long-horizon stability claim.',
            'Pressure samples are last1ms positive pressure, not full5ms contact force/observer labels.',
        ])
    for name, bound in bindings.items():
        if sha(name) != bound['sha256']:
            raise ValueError('Independent verification source/input changed: '+name)
    save(out/'report.json', result)
    save(out/'artifact_manifest.json', dict(files={str(p.relative_to(out)): dict(bytes=p.stat().st_size, sha256=sha(p))
        for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json'}))
    print(json.dumps(safe({k: result[k] for k in ('passed', 'rows', 'shared_precision_fields',
        'isolation', 'first_different_indices', 'endpoint_comparison', 'release_target_step_deg')}), indent=2))
    if not result['passed']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
