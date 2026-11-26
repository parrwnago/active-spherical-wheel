"""Independent saved-array QA of I46 clocks, raw contacts and command algebra.

No controller, contact locator, physics, terrain or scientific event/noise
reducer is invoked.  Raw bridge admission/aggregation is reconstructed with
independent vectorized arithmetic rather than the producer's helper.
"""
from pathlib import Path
import datetime
import hashlib
import json
import shutil

import numpy as np


HERE = Path(__file__).resolve().parent
FAMILY = HERE.parents[1]
ROOT = FAMILY.parents[1]
FRESH_FOLDERS = {'mars_compact_outback':'mars_compact_outback',
                 'flat_compact_outback':'flat_compact_outback_setup_corrected'}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def load(path):
    with np.load(path) as saved:
        return {name:saved[name].copy() for name in saved.files}


def compare(actual, expected, atol=1e-10):
    actual, expected = np.asarray(actual), np.asarray(expected)
    ok = actual.shape == expected.shape and np.allclose(actual, expected, atol=atol, rtol=0., equal_nan=True)
    finite = np.isfinite(actual) & np.isfinite(expected) if actual.shape == expected.shape else np.array([])
    gap = float(np.max(abs(actual[finite]-expected[finite]))) if np.any(finite) else None
    return dict(passed=bool(ok), maximum_absolute_gap=gap, tolerance=atol)


def verified_case_manifest(folder):
    entries = read(folder/'artifact_manifest.json')['files']
    failures = [row['path'] for row in entries
                if sha(folder/row['path']) != row['sha256'] or (folder/row['path']).stat().st_size != row['bytes']]
    return dict(passed=not failures, files=len(entries), failures=failures)


def command_checks(data):
    error = data['exact_feedback_error']
    delta_gain = -np.einsum('nij,nj->ni', data['shadow_feedback_gain']-data['exact_feedback_gain'], error)
    delta_support = data['shadow_support_torque']-data['exact_support_torque']
    expected_delta = delta_gain+delta_support
    bounds = data['input_bounds']
    requested = data['native_pre_sync_command']+data['exact_transition_command_delta']+data['exact_heading_command']+data['exact_fade_command_delta']
    return dict(
        separate_gain_delta=compare(data['shadow_gain_command_delta'], delta_gain),
        separate_holding_delta=compare(data['shadow_support_command_delta'], delta_support),
        combined_delta=compare(data['shadow_pre_sync_command_delta'], expected_delta),
        shadow_reconstruction=compare(data['shadow_pre_sync_command']-data['native_pre_sync_command'], expected_delta),
        native_component_sum=compare(data['exact_native_requested'], requested),
        native_actual_clip=compare(data['exact_applied'], np.clip(data['exact_native_requested'], -bounds, bounds)),
        native_shaft_gain_row=compare(data['shadow_feedback_gain'][:, 0], data['exact_feedback_gain'][:, 0], atol=0.),
        one_native_feedback_each=dict(passed=bool(np.all(data['shadow_feedback_call_count'] == 1))),
        all_shadow_history_guards=dict(passed=bool(np.all(data['shadow_history_unchanged']))),
        exact_fallback_delta=dict(passed=bool(np.array_equal(data['shadow_pre_sync_command_delta'][data['shadow_fallback_code'] != 0],
                                                           np.zeros_like(data['shadow_pre_sync_command_delta'][data['shadow_fallback_code'] != 0])))))


def independently_aggregate_bridge(raw):
    times, states, rows, offsets = raw['time_s'], raw['bridge_state'], raw['contacts'], raw['contact_offsets']
    if offsets.shape != (len(times)+1,) or offsets[0] != 0 or offsets[-1] != len(rows) or np.any(np.diff(offsets) < 0):
        raise ValueError('Raw bridge contact offsets are malformed')
    sample = np.repeat(np.arange(len(times)), np.diff(offsets))
    pressure = np.sum(rows[:, 7:10]*rows[:, 4:7], axis=1)
    admitted = (rows[:, 12] >= .5) & ((rows[:, 10] > 0) | (pressure > 0))
    tag = rows[:, 0].astype(int)
    if np.any(admitted & ((tag < 0) | (tag > 6))):
        raise ValueError('Unknown admitted native contact tag')
    body = admitted & (tag == 6)
    wheel = admitted & (tag != 6)
    loads, sums = np.zeros((len(times), 4)), np.zeros((len(times), 4, 3))
    counts = np.zeros((len(times), 4), int)
    patch = np.array([0, 1, 2, 3, 0, 1])[tag[wheel]]
    p = np.maximum(0., pressure[wheel])
    np.add.at(loads, (sample[wheel], patch), p)
    np.add.at(sums, (sample[wheel], patch), p[:, None]*rows[wheel, 1:4])
    np.add.at(counts, (sample[wheel], patch), p > .05)
    wheel_wrench, body_wrench, body_load = np.zeros((len(times), 6)), np.zeros((len(times), 6)), np.zeros(len(times))
    for mask, output in ((wheel, wheel_wrench), (body, body_wrench)):
        values = np.c_[rows[mask, 7:10], np.cross(rows[mask, 1:4]-states[sample[mask], 2:5], rows[mask, 7:10])]
        np.add.at(output, sample[mask], values)
    np.add.at(body_load, sample[body], np.maximum(0., pressure[body]))
    points = np.full_like(sums, np.nan)
    mask = loads > 1e-12
    points[mask] = sums[mask]/loads[mask, None]
    return dict(loads=loads, sums=sums, points=points, counts=counts,
                wheel_wrench=wheel_wrench, body_wrench=body_wrench, body_load=body_load)


def raw_truth_checks(data, raw):
    times, sub = data['time_s'], raw['time_s']
    rows = independently_aggregate_bridge(raw)
    n = len(times)-1
    if len(sub) != 5*n:
        raise ValueError('Fresh native case must have five physics samples per executed held command')
    loads = rows['loads'].reshape(n, 5, 4)
    sums = rows['sums'].reshape(n, 5, 4, 3).sum(axis=1)
    total = loads.sum(axis=1)
    points = np.full_like(sums, np.nan)
    mask = total > 1e-12
    points[mask] = sums[mask]/total[mask, None]
    equivalent = np.full((n, 3), np.nan)
    nonempty = total.sum(axis=1) > 1e-12
    equivalent[nonempty] = sums[nonempty].sum(axis=1)/total[nonempty].sum(axis=1)[:, None]
    checks = dict(
        relative_substep_clock=compare(sub.reshape(n, 5), times[:-1, None]+np.arange(1, 6)*.001, atol=1e-9),
        interval_endpoint_clock=compare(sub[4::5], times[1:], atol=1e-9),
        plugin_step_spacing=compare(np.diff(raw['bridge_state'][:, 0]), np.full(len(sub)-1, .001), atol=1e-8),
        t0_truth_invalid=dict(passed=bool(not data['truth_interval_valid'][0] and data['truth_interval_sample_count'][0] == 0
            and np.isnan(data['truth_interval_mean_loads_N'][0]).all())),
        all_following_intervals_valid=dict(passed=bool(np.all(data['truth_interval_valid'][1:]) and np.all(data['truth_interval_sample_count'][1:] == 5))),
        start_clock=compare(data['truth_interval_start_s'][1:], times[:-1], atol=1e-9),
        end_clock=compare(data['truth_interval_end_s'][1:], times[1:], atol=1e-9),
        mean_patch_loads=compare(data['truth_interval_mean_loads_N'][1:], loads.mean(axis=1)),
        mean_patch_points=compare(data['truth_interval_mean_patch_points_m'][1:], points),
        mean_equivalent_point=compare(data['truth_interval_mean_equivalent_point_m'][1:], equivalent),
        mean_wheel_wrench=compare(data['truth_interval_mean_wrench_world'][1:], rows['wheel_wrench'].reshape(n, 5, 6).mean(axis=1)),
        mean_body_wrench=compare(data['truth_interval_mean_body_wrench_world'][1:], rows['body_wrench'].reshape(n, 5, 6).mean(axis=1)),
        mean_all_external_wrench=compare(data['truth_interval_mean_all_external_wrench_world'][1:],
            (rows['wheel_wrench']+rows['body_wrench']).reshape(n, 5, 6).mean(axis=1)),
        mean_body_load=compare(data['truth_interval_mean_body_load_N'][1:], rows['body_load'].reshape(n, 5).mean(axis=1)),
        last_tick_patch_loads=compare(data['truth_patch_loads_N'][1:], rows['loads'][4::5]),
        last_tick_patch_points=compare(data['truth_patch_points_m'][1:], rows['points'][4::5]),
        last_tick_resolved_counts=compare(data['truth_patch_contact_counts'][1:], rows['counts'][4::5], atol=0.),
        last_tick_wheel_wrench=compare(data['truth_last_wrench_world'][1:], rows['wheel_wrench'][4::5]))
    return dict(passed=all(row['passed'] for row in checks.values()), checks=checks,
                physics_samples=len(sub), contacts=len(raw['contacts']), executed_intervals=n,
                published_terminal_command_is_unactuated=True,
                timing='Last completed1ms solver records vs integrated endpoint pose; independent convention retained without shifts.')


def main():
    if (HERE/'report.json').exists():
        raise FileExistsError('Preserve prior secondary raw QA before rerunning')
    protocol = read(FAMILY/'protocol.json')
    source = Path(__file__).resolve()
    snapshot = HERE/'source_snapshot'; snapshot.mkdir(exist_ok=False)
    shutil.copy2(source, snapshot/source.name)
    inputs = [FAMILY/'protocol.json', FAMILY/'campaign/jobs.json', FAMILY/'protocol_clarifications.json',
              FAMILY/'review/bridge_equivalence_stage01/report.json', FAMILY/'setup_correction_stage02/report.json',
              FAMILY/'campaign/runs/flat_compact_outback/failure.json']
    cases = {}
    for case in protocol['cases']:
        folder = FAMILY/'campaign/runs'/FRESH_FOLDERS[case['case_id']]
        required = ['artifact_manifest.json', 'report.json', 'precision.npz', 'truth_substeps.npz', 'rollout.npz', 'summary.json', 'provenance.json']
        if not all((folder/name).is_file() for name in required):
            raise FileNotFoundError('Wait for both sealed raw native cases')
        inputs += [folder/name for name in required]
    history = FAMILY/'campaign/historical_stage01'
    for name in ('artifact_manifest.json', 'report.json', 'precision.npz'):
        if not (history/name).is_file():
            raise FileNotFoundError('Wait for sealed historical replay')
        inputs.append(history/name)
    before = {str(path.relative_to(ROOT)):sha(path) for path in inputs}
    for case in protocol['cases']:
        folder = FAMILY/'campaign/runs'/FRESH_FOLDERS[case['case_id']]
        data, raw, case_report = load(folder/'precision.npz'), load(folder/'truth_substeps.npz'), read(folder/'report.json')
        commands = command_checks(data)
        truth = raw_truth_checks(data, raw)
        start = np.array(protocol['map_selection']['selected']['start_xy_m'])
        end = np.array(protocol['map_selection']['selected']['end_xy_m'])
        fraction = np.clip((data['exact_qpos'][:, :2]-start)@(end-start)/np.sum((end-start)**2), 0., 1.)
        distances = np.linalg.norm(data['exact_qpos'][:, :2]-(start+fraction[:, None]*(end-start)), axis=1)
        manifest = verified_case_manifest(folder)
        provenance = read(folder/'provenance.json')
        source_failures = [row['origin'] for row in provenance['sources']
            if sha(ROOT/row['origin']) != row['sha256'] or sha(folder/row['snapshot']) != row['sha256']]
        gate = provenance['stepping_equivalence']
        source_failures += [] if sha(ROOT/gate['path']) == gate['sha256'] else ['stepping_equivalence_report']
        cases[case['case_id']] = dict(raw_integrity_passed=manifest['passed'] and truth['passed'] and not source_failures and all(x['passed'] for x in commands.values()),
            manifest=manifest, commands=commands, truth=truth,
            folder=str(folder.relative_to(ROOT)), source_failures=source_failures,
            collection_complete=case_report['completed'], failure=case_report['failure'],
            actual_exposure_guard=dict(passed=bool(distances.max() <= .1), maximum_segment_distance_m=float(distances.max()),
                samples_outside_100mm=int(np.sum(distances > .1)),
                interpretation='Acceptance condition separate from raw integrity; Mars/flat are not an isolated slope experiment.'))
    old = read(history/'report.json')
    historical = dict(manifest=verified_case_manifest(history), samples=old['samples'],
        native_parity=all(row['all_rows_bit_identical'] and row['maximum_absolute_gap'] == 0. for row in old['history_parity'].values()),
        all_shadow_histories_unchanged=old['all_shadow_histories_unchanged'],
        commands=command_checks(load(history/'precision.npz')),
        truth_limit='Historical last1ms truth only; no preceding5ms truth reconstructed.')
    after = {str(path.relative_to(ROOT)):sha(path) for path in inputs}
    passed = before == after and all(x['raw_integrity_passed'] for x in cases.values()) and historical['manifest']['passed'] and historical['native_parity'] and all(x['passed'] for x in historical['commands'].values())
    report = dict(iteration='I-46', passed=passed, created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope=__doc__, render_or_simulation_calls=0, cases=cases, historical=historical,
        inputs_sha256=before, inputs_unchanged=before == after, source_sha256=sha(source),
        retained_setup_failure=dict(path='campaign/runs/flat_compact_outback/failure.json', native_command_samples=0,
            correction='Explicitly reviewed flat-only level-ground setup correction; original failed attempt retained, not a successful route.'),
        scientific_gate_ownership='Timing/noise/event/causal decision belongs canonical analysis reducer; no duplicate scientific reduction here.')
    (HERE/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(passed=passed, cases={k:v['raw_integrity_passed'] for k,v in cases.items()}), indent=2))
    if not passed:
        raise RuntimeError('Independent raw QA failed; preserve all outputs before correction')


if __name__ == '__main__':
    main()
