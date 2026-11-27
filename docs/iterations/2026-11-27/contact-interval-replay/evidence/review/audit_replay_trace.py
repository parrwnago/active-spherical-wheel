"""Independent I68 full-history byte parity and 1ms trace-integrity audit.

Read-only with respect to campaign data. No physics, models, fitting, inferred
contact scores, or generalized-force reconstruction are evaluated here.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {key: z[key].copy() for key in z.files}


def rotations(quaternions):
    q = quaternions / np.linalg.norm(quaternions, axis=1)[:, None]
    w, x, y, z = q.T
    return np.stack((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                     2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                     2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=1).reshape(-1, 3, 3)


def maximum_gap(left, right):
    if left.shape != right.shape:
        return None
    finite = np.isfinite(left) & np.isfinite(right)
    return float(np.max(np.abs(left[finite].astype(float)-right[finite].astype(float)))) if finite.any() else None


def run(campaign, historical, output):
    output.mkdir(parents=True, exist_ok=False)
    paths = {'precision': campaign/'campaign/runs/m100_B/precision.npz',
             'trace': campaign/'substeps.npz', 'historical_precision': historical}
    started = dict(iteration='I-68', utc=datetime.now(timezone.utc).isoformat(),
        audit_source_sha256=sha(__file__), numpy_version=np.__version__,
        scope='Independent byte parity, hold/clock/contact-packet integrity and state joins only; no force-model or contact-ranking arithmetic.',
        inputs={key: dict(path=str(path), sha256=sha(path)) for key, path in paths.items()})
    (output/'registered_before_audit.json').write_text(json.dumps(started, indent=2)+'\n')
    new, old, trace = load(paths['precision']), load(historical), load(paths['trace'])
    checks = {}; parity = {}
    for key in sorted(set(new) & set(old)):
        left, right = new[key], old[key]
        exact = left.dtype == right.dtype and left.shape == right.shape and left.tobytes() == right.tobytes()
        row = dict(byte_exact=exact, new_shape=list(left.shape), old_shape=list(right.shape),
                   new_dtype=left.dtype.str, old_dtype=right.dtype.str)
        if not exact and left.shape == right.shape:
            unequal = np.asarray(left != right)
            if np.issubdtype(left.dtype, np.inexact):
                unequal &= ~(np.isnan(left) & np.isnan(right))
            indices = np.argwhere(unequal)
            row['first_numeric_difference_index'] = None if not len(indices) else indices[0].tolist()
            if np.issubdtype(left.dtype, np.number):
                row['maximum_finite_absolute_gap'] = maximum_gap(left, right)
        parity[key] = row
    checks['precision_field_sets_identical'] = set(new) == set(old)
    checks['all_precision_fields_byte_exact'] = bool(parity) and all(row['byte_exact'] for row in parity.values())
    checks['precision_rows_8801'] = len(new['time_s']) == len(old['time_s']) == 8801
    n = len(trace['time_s']); holds = n//5
    checks['trace_rows_44000'] = n == 44000
    checks['five_rows_per_hold'] = n % 5 == 0
    checks['hold_index_exact'] = np.array_equal(trace['hold_index'], np.repeat(np.arange(8800), 5))
    checks['substep_index_exact'] = np.array_equal(trace['substep_index'], np.tile(np.arange(1, 6), 8800))
    checks['all_holds_completed'] = bool(trace['hold_completed'].all())
    metadata = json.loads(str(trace['metadata_json']))
    checks['metadata_completed_8800_failed_zero'] = metadata['completed_holds'] == 8800 and metadata['failed_holds'] == 0
    before = trace['bridge_state_before']; after = trace['bridge_state']
    held = trace['held_input']; disturbance = trace['disturbance_world']
    checks['state_shape24'] = before.shape == after.shape == (44000, 24)
    checks['finite_bridge_state'] = bool(np.isfinite(before).all() and np.isfinite(after).all())
    checks['held_input_matches_every_native_command'] = np.array_equal(held, np.repeat(new['exact_applied'][:-1], 5, axis=0))
    checks['held_input_constant_in_hold'] = bool(np.all(held.reshape(holds, 5, 5) == held.reshape(holds, 5, 5)[:, :1]))
    checks['disturbance_constant_in_hold'] = bool(np.all(disturbance.reshape(holds, 5, 6) == disturbance.reshape(holds, 5, 6)[:, :1]))
    checks['native_disturbance_zero'] = bool(np.all(disturbance == 0))
    checks['bridge_iterations_increment_one'] = bool(np.all(after[:, 1]-before[:, 1] == 1))
    checks['bridge_iteration_chain_continuous'] = np.array_equal(before[1:, 1], after[:-1, 1])
    checks['bridge_state_chain_byte_exact'] = before[1:].tobytes() == after[:-1].tobytes()
    checks['no_reset_during_hold'] = bool(np.all(after[:, 22] == before[:, 22]) and np.all(after[:, 23] == 0))
    dt_gap = np.max(np.abs(after[:, 0]-before[:, 0]-.001))
    step_gap = np.max(np.abs(after[:, 21]-.001))
    checks['bridge_step_duration_tolerance_1e8'] = bool(dt_gap <= 1e-8)
    checks['reported_step_size_tolerance_1e12'] = bool(step_gap <= 1e-12)
    expected_start = np.repeat(new['time_s'][:-1], 5)
    expected_end = expected_start+.005
    checks['hold_start_clock_exact'] = np.array_equal(trace['control_start_time_s'], expected_start)
    checks['hold_end_clock_exact'] = np.array_equal(trace['control_end_time_s'], expected_end)
    expected_samples = expected_start + np.tile(np.arange(1, 6), 8800)*.001
    checks['relative_sample_clock_exact'] = np.array_equal(trace['time_s'], expected_samples)
    offsets = trace['contact_offsets']; contacts = trace['contact_rows']
    checks['contact_offsets_shape'] = offsets.shape == (44001,)
    checks['contact_offsets_monotone_complete'] = bool(offsets[0] == 0 and np.all(np.diff(offsets) >= 0) and offsets[-1] == len(contacts))
    checks['contact_stride16_finite'] = contacts.ndim == 2 and contacts.shape[1] == 16 and bool(np.isfinite(contacts).all())
    checks['contact_tags_known_integer'] = bool(np.all(contacts[:, 0] == np.round(contacts[:, 0])) and np.all((contacts[:, 0] >= 0) & (contacts[:, 0] <= 6)))
    checks['saved_tag_column_exact'] = np.array_equal(trace['contact_tags'], contacts[:, 0].astype(np.int64))
    checks['saved_admitted_column_exact'] = np.array_equal(trace['contact_admitted'], contacts[:, 12] >= .5)
    joins = {}
    for label, raw, expected_q, expected_v in (
            ('before_hold', before[::5], new['exact_qpos'][:-1], new['exact_qvel'][:-1]),
            ('after_hold', after[4::5], new['exact_qpos'][1:], new['exact_qvel'][1:])):
        q = np.c_[raw[:, 2:5], raw[:, 5:9]/np.linalg.norm(raw[:, 5:9], axis=1)[:, None], raw[:, 15:18]]
        angular_world = np.einsum('nij,nj->ni', rotations(expected_q[:, 3:7]), expected_v[:, 3:6])
        direct_v = np.c_[raw[:, 9:12], raw[:, 18:21]]
        expected_direct_v = np.c_[expected_v[:, :3], expected_v[:, 6:9]]
        joins[label] = dict(qpos_maximum_absolute_gap=maximum_gap(q, expected_q),
            linear_joint_velocity_byte_exact=direct_v.tobytes() == expected_direct_v.tobytes(),
            angular_world_maximum_absolute_gap=maximum_gap(raw[:, 12:15], angular_world))
        checks[label+'_qpos_tolerance_1e12'] = joins[label]['qpos_maximum_absolute_gap'] <= 1e-12
        checks[label+'_direct_velocity_exact'] = joins[label]['linear_joint_velocity_byte_exact']
        checks[label+'_angular_tolerance_1e12'] = joins[label]['angular_world_maximum_absolute_gap'] <= 1e-12
    # Independently re-create only endpoint packet aggregates already logged by
    # I58. This checks copied contact rows against native control-rate records;
    # it is not a new generalized-force or contact-ranking model.
    family_index = (0, 1, 2, 3, 0, 1, -1)
    endpoint_loads = np.zeros((8800, 4)); endpoint_sums = np.zeros((8800, 4, 3))
    endpoint_counts = np.zeros((8800, 4), np.int64)
    for hold in range(8800):
        sample = 5*hold+4
        for contact in contacts[offsets[sample]:offsets[sample+1]]:
            if contact[12] < .5:
                continue
            pressure = float(contact[7:10] @ contact[4:7])
            if contact[10] <= 0 and pressure <= 0:
                continue
            label = family_index[int(contact[0])]
            if label < 0:
                continue
            pressure = max(0., pressure)
            endpoint_loads[hold, label] += pressure
            endpoint_sums[hold, label] += pressure*contact[1:4]
            endpoint_counts[hold, label] += int(pressure > .05)
    endpoint_points = np.full((8800, 4, 3), np.nan)
    positive = endpoint_loads > 1e-12
    endpoint_points[positive] = endpoint_sums[positive]/endpoint_loads[positive, None]
    contact_joins = dict(load_maximum_absolute_gap_N=maximum_gap(endpoint_loads, new['truth_patch_loads_N'][1:]),
        point_maximum_absolute_gap_m=maximum_gap(endpoint_points, new['truth_patch_points_m'][1:]),
        point_finite_masks_exact=bool(np.array_equal(np.isfinite(endpoint_points), np.isfinite(new['truth_patch_points_m'][1:]))),
        resolved_counts_exact=bool(np.array_equal(endpoint_counts, new['truth_patch_contact_counts'][1:])),
        count_definition='Compressive normal force >0.05N only for counts; every nonnegative normal load enters centroids. Original admission/depth handling preserved.')
    checks['endpoint_contact_load_join_tolerance_1e10N'] = contact_joins['load_maximum_absolute_gap_N'] <= 1e-10
    checks['endpoint_contact_point_join_tolerance_1e12m'] = contact_joins['point_maximum_absolute_gap_m'] <= 1e-12
    checks['endpoint_contact_point_finite_masks_exact'] = contact_joins['point_finite_masks_exact']
    checks['endpoint_resolved_contact_counts_exact'] = contact_joins['resolved_counts_exact']
    rejected = contacts[:, 12] < .5
    first_offset = before[0, 0]-trace['control_start_time_s'][0]
    clock_offsets = after[:, 0]-trace['time_s']
    summary = dict(trace_rows=n, completed_holds=holds, contact_rows=len(contacts),
        admitted_rows=int((~rejected).sum()), rejected_rows=int(rejected.sum()),
        rejected_force_maximum_N=float(np.max(np.linalg.norm(contacts[rejected, 7:10], axis=1))) if rejected.any() else None,
        maximum_bridge_duration_gap_s=float(dt_gap), maximum_reported_step_size_gap_s=float(step_gap),
        initial_bridge_to_relative_clock_offset_s=float(first_offset),
        maximum_clock_offset_drift_s=float(np.max(np.abs(clock_offsets-first_offset))),
        joins=joins, endpoint_contact_joins=contact_joins)
    report = dict(iteration='I-68', audit_utc=datetime.now(timezone.utc).isoformat(),
        checks=checks, all_checks_pass=all(checks.values()), precision_field_count=len(parity),
        old_only_fields=sorted(set(old)-set(new)), new_only_fields=sorted(set(new)-set(old)),
        precision_fields=parity, summary=summary,
        limitation='Packet and measured evolution validation only. No claim that the bridge exports every generalized constraint force or contact couple.')
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(all_checks_pass=report['all_checks_pass'], checks=len(checks), failed=[key for key,value in checks.items() if not value], precision_fields=len(parity), trace_rows=n)))
    if not report['all_checks_pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('campaign', 'historical', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    run(args.campaign.resolve(), args.historical.resolve(), args.output.resolve())
