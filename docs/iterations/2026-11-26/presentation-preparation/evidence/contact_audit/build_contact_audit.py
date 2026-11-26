"""Presentation-only saved-data contact/timing audit; no controller or physics."""
from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
I46 = REPO / 'results/support_routing_timing_20261126'
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
INPUTS = {
    'historical_I46': I46 / 'campaign/historical_stage01/precision.npz',
    'routing_report': I46 / 'analysis/stage01/report.json',
    'I41_signals': REPO / 'results/figure8_event_audit_20261125/events/full_audit/mars/signals.npz',
    'I41_report': REPO / 'results/figure8_event_audit_20261125/final_report.json',
    'I42_report': REPO / 'results/handoff_factorial_20261125/analysis/stage01/report.json',
    'original_I40': REPO / 'results/lqr_contact_forecast_20261125/campaign/runs/mars_figure8_p090/precision.npz',
    'native_metadata': REPO / 'results/lqr_contact_forecast_20261125/campaign/runs/mars_figure8_p090/provenance.json',
    'robot_XML': I46 / 'campaign/historical_stage01/_workspace/flat/model.xml',
}
for arm in ('baseline', 'feedback_shares', 'sync_off', 'both'):
    INPUTS['I42_' + arm] = REPO / 'results/handoff_factorial_20261125/campaign/runs' / arm / 'precision.npz'
for arm, folder in (('mars', 'mars_compact_outback'), ('flat', 'flat_compact_outback_setup_corrected')):
    INPUTS['I46_' + arm] = I46 / 'campaign/runs' / folder / 'precision.npz'
    INPUTS['I46_' + arm + '_summary'] = I46 / 'campaign/runs' / folder / 'summary.json'
SOURCES = ['src/spherical_wheel/contact_sensing.py', 'src/spherical_wheel/offroad_control.py',
           'src/spherical_wheel/recovery_control.py', 'src/spherical_wheel/experiments.py',
           'src/spherical_wheel/synchronization.py', 'src/spherical_wheel/offroad_experiments.py',
           'src/spherical_wheel/gazebo/bridge.py', 'src/spherical_wheel/gazebo/plant.py',
           'src/spherical_wheel/gazebo/experiments.py', 'scripts/audit-support-routing.py']


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for piece in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(piece)
    return h.hexdigest()


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
    path.write_text(json.dumps(safe(value), indent=2, allow_nan=False) + '\n')


def load(path):
    with np.load(path) as z:
        return {k: z[k].copy() for k in z.files}


def label(loads):
    if not np.isfinite(loads).all() or loads.sum() <= .001:
        return 'none'
    fractions = loads / loads.sum()
    if np.count_nonzero(abs(fractions - fractions.max()) <= 1e-12) != 1:
        return 'tie'
    return LABELS[int(np.argmax(loads))]


def main():
    source_dir = HERE / 'source_before_audit'
    source_dir.mkdir(exist_ok=False)
    copied = []
    for path in [Path(__file__)] + [REPO / p for p in SOURCES]:
        target = source_dir / path.name
        shutil.copyfile(path, target)
        copied.append(dict(origin=str(path.relative_to(REPO)), snapshot=str(target.relative_to(HERE)), sha256=sha(path)))
    before = {key: dict(path=str(path.relative_to(REPO)), sha256=sha(path)) for key, path in INPUTS.items()}
    save(HERE / 'declaration_before_audit.json', dict(captured_utc=datetime.now(timezone.utc).isoformat(),
        scope='Presentation preparation only; saved arrays and source math, no new observer/controller/model/scene/physics execution.',
        sources=copied, inputs=before,
        sample_rule='All41 existing5ms samples in[6.700,6.900]inclusive; display8 fixed times6.700/6.725/6.775/6.825/6.855/6.860/6.865/6.900.',
        fresh_rule='Use the SAME7 common primary events from registered I46 report; include last1ms and next-held native-history conventions separately.'))

    schemas = {}
    for key, path in INPUTS.items():
        if path.suffix == '.npz':
            with np.load(path) as z:
                schemas[key] = dict(path=str(path.relative_to(REPO)), arrays={k: dict(shape=z[k].shape, dtype=str(z[k].dtype)) for k in z.files},
                    potential_wall_time_keys=[k for k in z.files if any(word in k.lower() for word in ('wall', 'latency', 'deadline', 'receipt', 'compute_time', 'execution_time'))])
    command = ['rg', '--files', '-uu', '-g', '!**/.git/**', '-g', '!**/.venv/**', '-g', '!**/.cache/**',
               '-g', '*.bag', '-g', '*.db3', '-g', '*.mcap', '-g', '*rosbag*', '-g', 'metadata.yaml']
    found = subprocess.run(command, cwd=REPO, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert found.returncode in (0, 1)
    inventory = dict(recording_search_command=command, recording_matches=found.stdout.splitlines(),
        workspace_scope=str(REPO), no_matches_means='No matching ROS-bag/MCAP/DB3 artifacts found in this workspace; not a claim about other machines.',
        NPZ_schemas=schemas, timing_conclusion='No per-command wall-computation/communication/deadline fields in these saved arrays. Aggregate elapsed/wall times and simulation clocks are separate.')
    save(HERE / 'artifact_inventory.json', inventory)

    raw = load(INPUTS['historical_I46']); original = load(INPUTS['original_I40']); signals = load(INPUTS['I41_signals'])
    baseline = load(INPUTS['I42_baseline'])
    parity = {key: dict(I46_vs_I40_exact=bool(np.array_equal(raw[key], original[key])),
                        I42_baseline_vs_I40_exact=bool(np.array_equal(baseline[key], original[key])))
              for key in ('time_s', 'exact_qpos', 'exact_qvel', 'exact_applied', 'exact_native_requested', 'exact_observer_wrench')}
    assert all(all(item.values()) for item in parity.values())
    tree = ET.parse(INPUTS['robot_XML'])
    metadata = {}
    for name in LABELS:
        geom = tree.getroot().find(".//geom[@name='" + name + "_smooth']")
        sign, lower, upper, kind, radius = map(float, geom.get('user').split()[:5])
        metadata[name] = dict(sign=sign, lower_m=lower, upper_m=upper, radius_m=radius, kind=kind)
    radius = metadata['shell_A']['radius_m']
    indices = [int(np.argmin(abs(raw['time_s'] - t))) for t in np.arange(1340, 1381) * .005]
    assert len(set(indices)) == 41
    rows = []
    for index in indices:
        now, q = raw['time_s'][index], raw['exact_qpos'][index]
        rotation = Rotation.from_quat(q[[4, 5, 6, 3]]).as_matrix()
        axis = rotation @ np.array([np.cos(q[7]), 0., -np.sin(q[7])])
        true_loads = raw['truth_patch_loads_N'][index]
        actual_label = label(true_loads)
        actual_point = raw['truth_equivalent_point_m'][index]
        item = dict(time_s=float(now), true_last1ms_dominant_patch=actual_label,
            native_effective_patch=label(raw['native_feedback_effective_loads_N'][index]),
            fast_effective_patch=label(raw['shadow_used_feedback_effective_loads_N'][index]),
            raw_filtered_patch=label(raw['filtered_loads_N'][index]), raw_interval_patch=label(raw['interval_loads_N'][index]),
            true_patch_loads_N=true_loads, native_effective_loads_N=raw['native_feedback_effective_loads_N'][index],
            fast_effective_loads_N=raw['shadow_used_feedback_effective_loads_N'][index],
            filtered_point_gap_mm=float(np.linalg.norm(raw['filtered_point_m'][index] - actual_point) * 1000),
            interval_point_gap_mm=float(np.linalg.norm(raw['interval_point_m'][index] - actual_point) * 1000),
            filtered_nearest_band_boundary_mm=float(raw['filtered_nearest_band_boundary_abs_m'][index] * 1000),
            interval_nearest_band_boundary_mm=float(raw['interval_nearest_band_boundary_abs_m'][index] * 1000),
            gain_relative_step=float(signals['gain_relative_frobenius_step'][index]),
            belt_A_requested_m_s=float(raw['exact_native_requested'][index, 3]), belt_A_applied_m_s=float(raw['exact_applied'][index, 3]),
            shell_A_requested_Nm=float(raw['exact_native_requested'][index, 1]),
            sync_weights=raw['exact_sync_weights'][index], sync_delta=raw['exact_transition_command_delta'][index],
            filtered_force_to_interval_gap_N=float(np.linalg.norm(raw['exact_observer_wrench'][index, :3] - raw['exact_observer_interval_mean'][index, :3])),
            filtered_moment_to_interval_gap_Nm=float(np.linalg.norm(raw['exact_observer_wrench'][index, 3:] - raw['exact_observer_interval_mean'][index, 3:6])))
        for prefix in ('filtered', 'interval'):
            chosen = label(raw[prefix + '_loads_N'][index])
            if chosen in LABELS:
                k = LABELS.index(chosen)
                item[prefix + '_selected_lower_margin_mm'] = float(raw[prefix + '_lower_band_margin_m'][index, k] * 1000)
                item[prefix + '_selected_upper_margin_mm'] = float(raw[prefix + '_upper_band_margin_m'][index, k] * 1000)
        if actual_label in LABELS:
            m = metadata[actual_label]
            signed = m['sign'] * float(axis @ (actual_point - q[:3]))
            item['true_equivalent_point_in_its_band_current_pose_margin_mm'] = float(min(signed - m['lower_m'], m['upper_m'] - signed) * 1000)
        rows.append(item)
    save(HERE / 'historical_samples_6p7_to_6p9.json', rows)
    display = [.700 + 6, .725 + 6, .775 + 6, .825 + 6, .855 + 6, .860 + 6, .865 + 6, .900 + 6]
    shown = [min(rows, key=lambda r: abs(r['time_s'] - t)) for t in display]
    columns = ['time_s', 'true_last1ms_dominant_patch', 'native_effective_patch', 'fast_effective_patch',
        'filtered_point_gap_mm', 'filtered_nearest_band_boundary_mm', 'filtered_selected_upper_margin_mm',
        'gain_relative_step', 'belt_A_requested_m_s', 'belt_A_applied_m_s', 'shell_A_requested_Nm']
    with (HERE / 'historical_display_samples.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore'); writer.writeheader(); writer.writerows(shown)

    canonical = json.loads(INPUTS['routing_report'].read_text())
    transitions, clocks = [], {}
    for case in ('mars_compact_outback', 'flat_compact_outback'):
        tables = canonical['cases'][case]['truth_tables']
        primary = tables['preceding_mean']['common_paired_events']
        next_by_id = {r['truth_index']: r for r in tables['next_hold_aux']['common_paired_events']}
        last_by_id = {r['truth_index']: r for r in tables['last1ms']['common_paired_events']}
        for row in primary:
            aux, last = next_by_id.get(row['truth_index']), last_by_id.get(row['truth_index'])
            assert aux is not None and aux['direction'] == row['direction']
            assert abs(aux['truth_time_s'] - row['truth_time_s'] + .005) < 1e-8
            transitions.append(dict(case_id=case, direction=row['direction'], preceding_mean_truth_time_s=row['truth_time_s'],
                native_offset_preceding_ms=1000 * row['native_signed_offset_s'], fast_offset_preceding_ms=1000 * row['fast_signed_offset_s'],
                native_offset_next_hold_ms=1000 * aux['native_signed_offset_s'], fast_offset_next_hold_ms=1000 * aux['fast_signed_offset_s'],
                native_offset_last1ms_ms=None if last is None else 1000 * last['native_signed_offset_s'],
                fast_offset_last1ms_ms=None if last is None else 1000 * last['fast_signed_offset_s']))
        clocks[case] = {clock: {source: table['sources'][source]['routing_mismatch_equivalent_s'] for source in ('native', 'fast')}
                        for clock, table in tables.items()}
    assert len(transitions) == 7
    save(HERE / 'seven_fresh_transitions.json', transitions)
    with (HERE / 'seven_fresh_transitions.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(transitions[0])); writer.writeheader(); writer.writerows(transitions)

    oracle_rows = []
    for arm in ('baseline', 'feedback_shares', 'sync_off', 'both'):
        data = load(INPUTS['I42_' + arm])
        for target in (6.725, 6.825, 6.86):
            i = int(np.argmin(abs(data['time_s'] - target)))
            oracle_rows.append(dict(arm=arm, time_s=float(data['time_s'][i]), true_patch=label(data['truth_patch_loads_N'][i]),
                native_inferred_patch=label(data['native_feedback_loads_N'][i]), used_feedback_patch=label(data['used_feedback_loads_N'][i]),
                requested=data['exact_native_requested'][i], applied=data['exact_applied'][i],
                comparison_limit='Same arm data; physical histories differ after intervention onset, not a same-state cross-arm command calculation.'))
    save(HERE / 'I42_context_samples.json', oracle_rows)
    result = dict(scope='Presentation-only interpretation of existing I41/I42/I46 data, no new physics or controller execution.',
        inventory=inventory, native_history_identity=parity, patch_metadata=metadata,
        display_samples=shown, fresh_seven_transitions=transitions, clock_mismatch_equivalent_seconds=clocks,
        registered_I46_decision=canonical['registered_gate'], historical_causality=canonical['historical_causality'],
        confirmed=['Synchronous finite blocking Gazebo steps separate wall-clock computation from simulated motion.',
            'Momentum residual uses preceding held input, an interval estimate and an explicit100rad/s low-pass.',
            'Current pose classifies the filtered-wrench support point; point closeness alone does not establish patch identity.',
            'Fresh event offsets can be early relative to preceding5ms mean; no universal fixed label delay is demonstrated.',
            'I46 does not justify replacing the wrench filter; I42 truth feedback shares did not prevent the6.860s departure.'],
        unestablished=['Per-command computation/transport latency and deadline misses were not logged in these arrays.',
            'The lockstep data do not demonstrate CPU calculation latency causing physical stale actuation.',
            'A net wrench does not uniquely determine arbitrary simultaneous contact locations/load distributions.',
            'Shadow command sensitivities do not establish an alternative closed-loop outcome.'])
    save(HERE / 'contact_audit.json', result)
    for key, path in INPUTS.items():
        assert sha(path) == before[key]['sha256']
    for item in copied:
        assert sha(REPO / item['origin']) == item['sha256']
    save(HERE / 'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(HERE)), bytes=p.stat().st_size, sha256=sha(p))
        for p in sorted(HERE.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json']))


if __name__ == '__main__':
    main()
