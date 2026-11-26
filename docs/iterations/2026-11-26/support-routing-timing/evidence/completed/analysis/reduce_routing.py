"""I46 registered routing reduction; native data and non-actuated sensitivities."""
from __future__ import annotations
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from spherical_wheel.contact_sensing import SupportLocator
from spherical_wheel.support_routing_audit import support_from_wrench

HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
REPO = FAMILY.parents[1]
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
SOURCES = ('native', 'fast')
DT = .005


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for b in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(b)
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


def save(path, obj):
    path.write_text(json.dumps(safe(obj), indent=2, allow_nan=False) + '\n')


def load_npz(path):
    with np.load(path) as z:
        return {k: z[k].copy() for k in z.files}


def statistics(values):
    a = np.asarray([v for v in values if v is not None and np.isfinite(v)], float)
    if not len(a):
        return dict(n=0, mean=None, rms=None, median=None, p95=None, maximum=None)
    return dict(n=len(a), mean=float(np.mean(a)), rms=float(np.linalg.norm(a) / np.sqrt(len(a))),
                median=float(np.median(a)), p95=float(np.quantile(a, .95)), maximum=float(np.max(a)))


def shares(loads, minimum_total=.001):
    loads = np.asarray(loads, float)
    total = loads.sum(axis=1)
    valid = np.isfinite(loads).all(axis=1) & (loads >= 0).all(axis=1) & (total > minimum_total)
    value = np.divide(loads, total[:, None], out=np.zeros_like(loads), where=valid[:, None])
    return value, valid, total


def dominant_labels(value, valid, tie_tolerance=1e-12):
    maximum = value.max(axis=1)
    tied = (np.abs(value - maximum[:, None]) <= tie_tolerance).sum(axis=1) > 1
    label = np.argmax(value, axis=1).astype(int)
    label[~valid] = -1
    label[valid & tied] = -2
    return label


def family_labels(value, valid, strong=.8):
    family = np.column_stack([value[:, :2].sum(axis=1), value[:, 2:].sum(axis=1)])
    result = np.full(len(value), -1, int)
    result[valid & (family[:, 0] >= strong)] = 0
    result[valid & (family[:, 1] >= strong)] = 1
    return result


def stable_family_events(times, labels, dwell=.015, max_gap=.05):
    """Retrospectively confirm run onset; 3x5ms samples span10ms, not15ms."""
    times, labels = np.asarray(times, float), np.asarray(labels, int)
    if not len(times):
        return dict(events=[], stable_runs=[], short_blips=[], censored_starts=[])
    boundaries = np.r_[0, np.flatnonzero((labels[1:] != labels[:-1]) | (np.diff(times) > DT + 1e-9)) + 1, len(times)]
    stable, blips = [], []
    for a, b in zip(boundaries[:-1], boundaries[1:]):
        if labels[a] < 0:
            continue
        run = dict(family='shell' if labels[a] == 0 else 'belt', start_s=float(times[a]), end_s=float(times[b - 1]),
                   first_index=int(a), last_index=int(b - 1), span_s=float(times[b - 1] - times[a]), samples=int(b - a))
        (stable if run['span_s'] >= dwell - 1e-10 else blips).append(run)
    events, censored = [], []
    previous = None
    for run in stable:
        if previous is None or run['start_s'] - previous['end_s'] > max_gap + 1e-10:
            censored.append(run)
        elif run['family'] != previous['family']:
            events.append(dict(time_s=run['start_s'], direction=previous['family'] + '_to_' + run['family'],
                               previous_stable_end_s=previous['end_s'], confirmation_s=run['start_s'] + dwell))
        previous = run
    return dict(events=events, stable_runs=stable, short_blips=blips, censored_starts=censored)


def match_events(truth, estimated, pair_window=(-.1, .15)):
    """Chronological truth greedy, nearest absolute signed offset, earlier tie."""
    used, matches = set(), []
    for ti, t in enumerate(truth):
        possible = []
        for ei, e in enumerate(estimated):
            delta = e['time_s'] - t['time_s']
            if ei not in used and e['direction'] == t['direction'] and pair_window[0] - 1e-10 <= delta <= pair_window[1] + 1e-10:
                possible.append((abs(delta), e['time_s'], ei, delta))
        if possible:
            smallest_offset = min(p[0] for p in possible)
            tied = [p for p in possible if p[0] <= smallest_offset + 1e-10]
            _, _, ei, delta = min(tied, key=lambda p: (p[1], p[2]))
            used.add(ei)
            matches.append(dict(truth_index=ti, estimate_index=ei, truth_time_s=t['time_s'], estimated_time_s=estimated[ei]['time_s'],
                                direction=t['direction'], signed_offset_s=float(delta)))
    return dict(matches=matches, unmatched_truth_indices=[i for i in range(len(truth)) if i not in {m['truth_index'] for m in matches}],
                extra_estimate_indices=[i for i in range(len(estimated)) if i not in used])


def within(times, window):
    return (times >= window[0] - 1e-9) & (times < window[1] - 1e-9)


def truth_metrics(times, truth_loads, truth_valid, body_load, sources, window, weight, events_spec):
    true_share, positive, total = shares(truth_loads)
    loaded = truth_valid & positive & (total >= max(.05, .05 * weight))
    body_ok = np.isfinite(body_load) & (body_load <= .05 * weight)
    eligible = loaded & body_ok
    truth_dom = dominant_labels(true_share, eligible)
    unique = eligible & (true_share.max(axis=1) >= .8)
    true_family = stable_family_events(times, family_labels(true_share, eligible), events_spec['stable_dwell_s'], events_spec['max_gap_between_stable_runs_s'])
    selected = within(times, window) & (times + DT <= times[-1] + 1e-9)
    result = dict(truth_loaded_samples=int(sum(loaded & selected)), primary_loaded_samples=int(sum(eligible & selected)),
        body_excluded_samples=int(sum(loaded & ~body_ok & selected)), true_unique_patch_samples=int(sum(unique & selected)),
        loaded_not_unique_samples=int(sum(eligible & ~unique & selected)), scored_indicator_duration_s=float(sum(selected) * DT),
        true_family=true_family, sources={})
    for name, loads in sources.items():
        estimate, valid, _ = shares(loads)
        labels = dominant_labels(estimate, valid)
        score = .5 * np.abs(estimate - true_share).sum(axis=1)
        score[~valid] = 1.
        sample_mask = selected & eligible
        wrong = selected & unique & (labels != truth_dom)
        adjacent = (np.diff(times) <= DT + 1e-9) & sample_mask[1:] & sample_mask[:-1] & valid[1:] & valid[:-1]
        variation = .5 * np.abs(np.diff(estimate, axis=0)).sum(axis=1)
        family = stable_family_events(times, family_labels(estimate, valid & body_ok & truth_valid), events_spec['stable_dwell_s'], events_spec['max_gap_between_stable_runs_s'])
        matched = match_events(true_family['events'], family['events'], events_spec['pair_window_s'])
        scored_matches = [m for m in matched['matches'] if window[0] - 1e-9 <= m['truth_time_s'] < window[1] - 1e-9]
        missed = [true_family['events'][i] for i in matched['unmatched_truth_indices'] if window[0] - 1e-9 <= true_family['events'][i]['time_s'] < window[1] - 1e-9]
        extra = [family['events'][i] for i in matched['extra_estimate_indices'] if window[0] - 1e-9 <= family['events'][i]['time_s'] < window[1] - 1e-9]
        result['sources'][name] = dict(share_error_with_missing_penalty=statistics(score[sample_mask]),
            missing_estimate_samples=int(sum(sample_mask & ~valid)), missing_estimate_fraction=float(sum(sample_mask & ~valid) / sum(sample_mask)) if sum(sample_mask) else None,
            wrong_patch_samples=int(sum(wrong)), routing_mismatch_equivalent_s=float(sum(wrong) * DT),
            tie_estimate_samples=int(sum(sample_mask & (labels == -2))),
            share_total_variation_adjacent_valid=float(variation[adjacent].sum()), variation_edges=int(sum(adjacent)),
            invalid_status_changes=int(sum((valid[1:] != valid[:-1]) & selected[1:] & selected[:-1])),
            stable_family=family, event_matches=scored_matches, unmatched_truth_events=missed, extra_estimated_events=extra,
            signed_event_offset_s=statistics([m['signed_offset_s'] for m in scored_matches]),
            absolute_event_offset_s=statistics([abs(m['signed_offset_s']) for m in scored_matches]))
    native = {m['truth_index']: m for m in result['sources']['native']['event_matches']}
    fast = {m['truth_index']: m for m in result['sources']['fast']['event_matches']}
    common = []
    for key in sorted(set(native) & set(fast)):
        a, b = native[key], fast[key]
        common.append(dict(truth_index=key, truth_time_s=a['truth_time_s'], direction=a['direction'],
            native_signed_offset_s=a['signed_offset_s'], fast_signed_offset_s=b['signed_offset_s'],
            absolute_timing_improvement_s=abs(a['signed_offset_s']) - abs(b['signed_offset_s'])))
    result['common_paired_events'] = common
    result['common_paired_timing_improvement_s'] = statistics([m['absolute_timing_improvement_s'] for m in common])
    result['scored_truth_event_direction_counts'] = dict(Counter(e['direction'] for e in true_family['events'] if window[0] - 1e-9 <= e['time_s'] < window[1] - 1e-9))
    result['common_event_direction_counts'] = dict(Counter(e['direction'] for e in common))
    return result


def command_metrics(times, data, window):
    mask = within(times, window) & (times + DT <= times[-1] + 1e-9)
    edges = mask[1:] & mask[:-1] & (np.diff(times) <= DT + 1e-9)
    bounds = np.asarray(data['input_bounds'])
    if bounds.ndim == 1:
        bounds = np.broadcast_to(bounds, (len(times), 5))
    assert np.isfinite(bounds).all() and (bounds > 0).all() and np.array_equal(bounds, np.broadcast_to(bounds[0], bounds.shape))
    result = {}
    for source, prefix in (('native', 'native'), ('fast', 'shadow')):
        result[source] = {}
        for view, suffix in (('requested', 'command'), ('clipped', 'clipped_command')):
            values = data[prefix + '_pre_sync_' + suffix]
            steps = np.max(np.abs(np.diff(values, axis=0)) / bounds[1:], axis=1)
            indices = np.flatnonzero(edges)
            peak = int(indices[np.argmax(steps[indices])]) + 1 if len(indices) else None
            result[source][view] = dict(normalized_adjacent_step=statistics(steps[edges]),
                maximum_step_time_s=None if peak is None else float(times[peak]))
        result[source]['requested_saturation_fraction'] = float(np.mean(np.any(np.abs(data[prefix + '_pre_sync_command'][mask]) >= bounds[mask] - 1e-12, axis=1))) if sum(mask) else None
    result['same_state_delta_normalized_inf'] = statistics(np.max(np.abs(data['shadow_pre_sync_command_delta'][mask]) / bounds[mask], axis=1))
    return result


def exposure_metrics(times, xy, job):
    start = np.asarray(job['origin_xy_m'])
    direction = np.array([np.cos(np.radians(job['heading_deg'])), np.sin(np.radians(job['heading_deg']))])
    delta = job['leg_distance_m'] * direction
    ratio = np.clip((xy - start) @ delta / (delta @ delta), 0., 1.)
    distance = np.linalg.norm(xy - start - ratio[:, None] * delta, axis=1)
    return dict(maximum_center_distance_from_segment_m=float(distance.max()),
                guard_passed=bool((distance <= .1 + 1e-12).all()),
                outside_samples=int(sum(distance > .1 + 1e-12)))


def paired_noise(case_order, all_data, weights, windows, model_path, protocol, stage):
    """Static local sensitivity at saved poses, using exact rounded XML metadata."""
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    fork = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'fork')
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'shell_A_smooth')
    radius = float(model.geom_user[geom, 4])
    rng = np.random.default_rng(protocol['noise']['seed'])
    draws = protocol['noise']['draws_per_sample']
    summaries, rows = {}, []
    model_inputs = [model_path]
    tree = ET.parse(model_path)
    compiler = tree.getroot().find('compiler')
    meshdir = Path(compiler.get('meshdir', '.')) if compiler is not None else Path('.')
    for mesh in tree.getroot().findall('asset/mesh'):
        if mesh.get('file'):
            path = Path(mesh.get('file'))
            model_inputs.append(path if path.is_absolute() else model_path.parent / meshdir / path)
    save(stage / 'noise_model_inputs_before.json', dict(files=[dict(path=str(p), sha256=sha(p)) for p in sorted(set(model_inputs))]))
    for case_number, case_id in enumerate(case_order):
        raw, window, weight = all_data[case_id], windows[case_id], weights[case_id]
        times = raw['time_s']
        targets = np.arange(window[0], min(window[1], times[-1]), protocol['noise']['sample_stride_s'])
        indices = [int(np.argmin(abs(times - t))) for t in targets]
        assert all(abs(times[i] - target) < 1e-8 for i, target in zip(indices, targets))
        locator = SupportLocator(model, fork, radius, .7, .05 * weight)
        sigma = np.r_[np.full(3, .002 * weight), np.full(3, .002 * weight * radius)]
        labels_all, invalid_all, base_all, margins_all = [], [], [], []
        parity_gap = 0.
        for index in indices:
            data.qpos[:] = raw['exact_qpos'][index]
            mujoco.mj_kinematics(model, data)
            perturbation = rng.normal(size=(draws, 6)) * sigma
            source_labels, source_invalid, baseline, margins = [], [], [], []
            for prefix, wrench in (('filtered', raw['exact_observer_wrench'][index]), ('interval', raw['exact_observer_interval_mean'][index, :6])):
                unperturbed = support_from_wrench(locator, data, wrench)
                logged = raw[prefix + '_loads_N'][index]
                parity_gap = max(parity_gap, float(np.max(abs(unperturbed['loads_N'] - logged))))
                np.testing.assert_allclose(unperturbed['loads_N'], logged, atol=1e-8, rtol=1e-10)
                base_shares, base_valid, _ = shares(unperturbed['loads_N'][None, :])
                baseline.append(int(dominant_labels(base_shares, base_valid)[0]))
                margins.append(unperturbed['nearest_band_boundary_abs_m'])
                labels, invalid = [], []
                for delta in perturbation:
                    support = support_from_wrench(locator, data, wrench + delta)
                    values, valid, _ = shares(support['loads_N'][None, :])
                    labels.append(int(dominant_labels(values, valid)[0]))
                    invalid.append(not bool(valid[0]))
                source_labels.append(labels)
                source_invalid.append(invalid)
            labels_all.append(source_labels); invalid_all.append(source_invalid)
            base_all.append(baseline); margins_all.append(margins)
            rows.append(dict(case_number=case_number, sample_index=index, time_s=times[index], perturbation=perturbation,
                             base_labels=baseline, labels=source_labels, invalid=source_invalid, boundary_margin_m=margins))
        labels_all = np.asarray(labels_all, int).reshape(-1, 2, draws)
        invalid_all = np.asarray(invalid_all, bool).reshape(-1, 2, draws)
        base_all = np.asarray(base_all, int).reshape(-1, 2)
        margins_all = np.asarray(margins_all, float).reshape(-1, 2)
        summaries[case_id] = dict(samples=len(indices), draws_per_sample=draws,
            support_parity_max_abs_gap_N=parity_gap, sources={name: dict(
                label_flip_fraction=float(np.mean(labels_all[:, k] != base_all[:, k, None])) if len(indices) else None,
                invalid_fraction=float(np.mean(invalid_all[:, k])) if len(indices) else None,
                baseline_invalid_fraction=float(np.mean(base_all[:, k] == -1)) if len(indices) else None,
                boundary_margin_m=statistics(margins_all[:, k])) for k, name in enumerate(SOURCES)},
            interpretation='Same static perturbation per source/pose; not physical sensor-noise propagation through either filter.')
    np.savez_compressed(stage / 'noise_samples.npz',
        case_ids=np.array(case_order), case_number=np.array([r['case_number'] for r in rows]),
        sample_index=np.array([r['sample_index'] for r in rows]), time_s=np.array([r['time_s'] for r in rows]),
        perturbation=np.array([r['perturbation'] for r in rows]), base_labels=np.array([r['base_labels'] for r in rows]),
        labels=np.array([r['labels'] for r in rows]), invalid=np.array([r['invalid'] for r in rows]),
        boundary_margin_m=np.array([r['boundary_margin_m'] for r in rows]))
    for item in json.loads((stage / 'noise_model_inputs_before.json').read_text())['files']:
        assert sha(Path(item['path'])) == item['sha256']
    return summaries


def registered_gate(case_reports, noise, fresh_ids, spec):
    coverage, benefit, robust, timing = {}, {}, {}, []
    extra_native = extra_fast = 0
    for case_id in fresh_ids:
        case = case_reports[case_id]
        primary = case['truth_tables']['preceding_mean']
        directions = ('shell_to_belt', 'belt_to_shell')
        coverage[case_id] = dict(completed=case['completed'], full_scoring_window=case['window_complete'],
            actual_exposure_guard=case['exposure']['guard_passed'],
            true_each_direction=all(primary['scored_truth_event_direction_counts'].get(d, 0) >= spec['minimum_each_family_direction_per_fresh_case'] for d in directions),
            common_matched_each_direction=all(primary['common_event_direction_counts'].get(d, 0) >= spec['minimum_each_family_direction_per_fresh_case'] for d in directions))
        a, b = (primary['sources'][k]['routing_mismatch_equivalent_s'] for k in SOURCES)
        relative = (a - b) / a if a > 0 else None
        benefit[case_id] = dict(native_mismatch_s=a, fast_mismatch_s=b,
            absolute_reduction_s=a - b, relative_reduction=relative,
            absolute_pass=a - b >= spec['wrong_patch_duration_absolute_reduction_s_min'] - 1e-12,
            relative_pass=relative is not None and relative >= spec['wrong_patch_duration_relative_reduction_min'] - 1e-12)
        commands = case['commands']
        native_step = commands['native']['requested']['normalized_adjacent_step']['maximum']
        fast_step = commands['fast']['requested']['normalized_adjacent_step']['maximum']
        native_noise, fast_noise = (noise[case_id]['sources'][k]['label_flip_fraction'] for k in SOURCES)
        robust[case_id] = dict(max_normalized_requested_step_increase=None if native_step is None or fast_step is None else fast_step - native_step,
            requested_step_pass=native_step is not None and fast_step is not None and fast_step - native_step <= spec['max_normalized_command_step_increase'] + 1e-12,
            noise_flip_fraction_increase=None if native_noise is None or fast_noise is None else fast_noise - native_noise,
            noise_pass=native_noise is not None and fast_noise is not None and fast_noise - native_noise <= spec['max_noise_flip_fraction_increase'] + 1e-12)
        timing.extend(dict(case_id=case_id, **row) for row in primary['common_paired_events'])
        extra_native += len(primary['sources']['native']['extra_estimated_events'])
        extra_fast += len(primary['sources']['fast']['extra_estimated_events'])
    improvement = statistics([row['absolute_timing_improvement_s'] for row in timing])
    checks = dict(all_case_coverage=all(all(v.values()) for v in coverage.values()),
        enough_common_paired_events=len(timing) >= spec['minimum_paired_family_events_total'],
        paired_median_absolute_timing_improvement=improvement['median'] is not None and improvement['median'] >= spec['minimum_median_abs_timing_improvement_s'] - 1e-12,
        each_case_wrong_duration_benefit=all(v['absolute_pass'] and v['relative_pass'] for v in benefit.values()),
        extra_false_family_transition_nonregression=extra_fast - extra_native <= spec['max_extra_false_family_transitions_total'],
        each_case_command_and_noise_nonregression=all(v['requested_step_pass'] and v['noise_pass'] for v in robust.values()))
    coverage_ok = checks['all_case_coverage'] and checks['enough_common_paired_events']
    passed = all(checks.values())
    return dict(passes_all_conditions=passed, checks=checks, case_coverage=coverage, case_benefit=benefit,
        case_nonregression=robust, paired_timing_improvement_s=improvement, common_paired_events=timing,
        total_native_extra_family_events=extra_native, total_fast_extra_family_events=extra_fast,
        decision='only_justify_separate_bounded_live_candidate' if passed else 'inconclusive_no_integration' if not coverage_ok else 'retain_filter_no_integration',
        interpretation='Registered diagnostic screen, not adoption; all candidate commands remained shadow-only.')


def historical_causality(times, data, case, window, weight, body_load):
    a, valid, _ = shares(data['truth_patch_loads_N'])
    unique = valid & (data['truth_patch_loads_N'].sum(axis=1) >= max(.05, .05 * weight)) & (a.max(axis=1) >= .8) & (body_load <= .05 * weight)
    truth = dominant_labels(a, unique)
    native, nv, _ = shares(data['native_feedback_effective_loads_N'])
    fast, fv, _ = shares(data['shadow_used_feedback_effective_loads_N'])
    corrected = unique & (dominant_labels(native, nv) != truth) & (dominant_labels(fast, fv) == truth)
    interval = (times >= window[0] - 1e-9) & (times <= 6.86 + 1e-9)
    saturation = interval & (np.abs(data['exact_native_requested'][:, 3]) >= data['input_bounds'][:, 3] - 1e-12)
    normalized_delta = np.max(abs(data['shadow_pre_sync_command_delta']) / data['input_bounds'], axis=1)
    def first(mask):
        indices = np.flatnonzero(mask)
        return float(times[indices[0]]) if len(indices) else None
    sat_time, correction = first(saturation), first(interval & corrected)
    na = case['commands']['native']['requested']['normalized_adjacent_step']['maximum']
    fa = case['commands']['fast']['requested']['normalized_adjacent_step']['maximum']
    step_reduction = (na - fa) / na if na is not None and na > 0 and fa is not None else None
    return dict(first_native_belt_A_request_saturation_s=sat_time,
        first_truth_correcting_effective_route_s=correction,
        first_any_shadow_command_difference_s=first(interval & (normalized_delta > 1e-6)),
        maximum_requested_step_relative_reduction=step_reduction,
        cue_not_after_saturation=correction is not None and sat_time is not None and correction <= sat_time + 1e-9,
        ten_percent_requested_jump_reduction=step_reduction is not None and step_reduction >= .1 - 1e-12,
        interpretation='Necessary diagnostic timing/jump checks only; even an early cue does not establish the cause of departure or recovery under fast routing.')


def verify_artifacts(folder):
    manifest = json.loads((folder / 'artifact_manifest.json').read_text())
    for item in manifest['files']:
        path = folder / item['path']
        assert path.stat().st_size == item['bytes'] and sha(path) == item['sha256'], str(path)


def reduce_main(stage, protocol, case_dirs_path=None):
    cases = ['historical_mars_figure8'] + [job['case_id'] for job in protocol['cases']]
    folders = dict(historical_mars_figure8=FAMILY / 'campaign/historical_stage01',
                   **{job['case_id']: FAMILY / 'campaign/runs' / job['case_id'] for job in protocol['cases']})
    if case_dirs_path is not None:
        overrides = json.loads(case_dirs_path.read_text())
        if set(overrides) - set(cases):
            raise ValueError('Only predeclared case IDs may be redirected after retained setup correction')
        folders.update({case: Path(path).resolve() for case, path in overrides.items()})
        shutil.copyfile(case_dirs_path, stage / 'case_directory_overrides.json')
        save(stage / 'case_directory_override_before.json', dict(origin=str(case_dirs_path), sha256=sha(case_dirs_path)))
    inputs = [folder / name for folder in folders.values() for name in ('precision.npz', 'report.json', 'provenance.json', 'artifact_manifest.json')]
    for key, path in protocol['inputs'].items():
        assert sha(REPO / path) == protocol['input_sha256'][key]
    for folder in folders.values():
        verify_artifacts(folder)
    save(stage / 'input_manifest_before_reduction.json', dict(files=[dict(path=str(p.relative_to(REPO)), sha256=sha(p)) for p in inputs]))
    all_data = {case: load_npz(folder / 'precision.npz') for case, folder in folders.items()}
    original_provenance = json.loads((REPO / protocol['inputs']['historical_provenance']).read_text())
    weights = {'historical_mars_figure8': original_provenance['weight_N']}
    windows = {'historical_mars_figure8': protocol['evaluation']['historical_window_s']}
    case_reports, historical_body = {}, None
    for case in cases:
        raw, folder = all_data[case], folders[case]
        times = raw['time_s']
        assert len(times) and np.allclose(np.diff(times), DT, rtol=0, atol=1e-9)
        assert np.all(raw['shadow_history_unchanged']) and np.all(raw['shadow_feedback_call_count'] == 1)
        report = json.loads((folder / 'report.json').read_text())
        assert report['source_hashes_unchanged']
        sources = dict(native=raw['native_feedback_effective_loads_N'], fast=raw['shadow_used_feedback_effective_loads_N'])
        raw_sources = dict(native=raw['filtered_loads_N'], fast=raw['interval_loads_N'])
        if case == 'historical_mars_figure8':
            assert report['passed'] and all(v['all_rows_bit_identical'] for v in report['history_parity'].values())
            with np.load(REPO / protocol['inputs']['historical_rollout']) as z:
                assert len(z['time_s']) == len(times)
                np.testing.assert_array_equal(times.astype(z['time_s'].dtype), z['time_s'])
                body = z['fork_load_N'].astype(float)
                historical_body = body.copy()
            truth_tables = dict(last1ms=(raw['truth_patch_loads_N'], np.ones(len(times), bool), body))
            completed, exposure = True, None
        else:
            provenance = json.loads((folder / 'provenance.json').read_text())
            weights[case] = provenance['weight_N']
            windows[case] = protocol['evaluation']['fresh_window_s']
            valid = raw['truth_interval_valid'].astype(bool)
            assert not valid[0] and np.all(raw['truth_interval_sample_count'][valid] == 5)
            shifted_loads = np.r_[raw['truth_interval_mean_loads_N'][1:], np.full((1, 4), np.nan)]
            shifted_body = np.r_[raw['truth_interval_mean_body_load_N'][1:], np.nan]
            shifted_valid = np.r_[valid[1:], False]
            truth_tables = dict(preceding_mean=(raw['truth_interval_mean_loads_N'], valid, raw['truth_interval_mean_body_load_N']),
                last1ms=(raw['truth_patch_loads_N'], np.ones(len(times), bool), raw['truth_last_body_load_N']),
                next_hold_aux=(shifted_loads, shifted_valid, shifted_body))
            completed = bool(report['completed'])
            job = next(j for j in protocol['cases'] if j['case_id'] == case)
            exposure = exposure_metrics(times, raw['exact_qpos'][:, :2], job)
        tables = {name: truth_metrics(times, loads, valid, body, sources, windows[case], weights[case], protocol['evaluation']['family_events'])
                  for name, (loads, valid, body) in truth_tables.items()}
        primary = 'last1ms' if case == 'historical_mars_figure8' else 'preceding_mean'
        raw_tables = truth_metrics(times, *truth_tables[primary], raw_sources, windows[case], weights[case], protocol['evaluation']['family_events'])
        case_reports[case] = dict(completed=completed, physical_failure=report.get('failure'), samples=len(times), final_time_s=float(times[-1]),
            scoring_window_s=windows[case], window_complete=bool(times[0] <= windows[case][0] + 1e-9 and times[-1] >= windows[case][1] - 1e-9),
            truth_tables=tables, raw_locator_primary=raw_tables, commands=command_metrics(times, raw, windows[case]), exposure=exposure,
            shadow_fallback_codes=dict(Counter(map(int, raw['shadow_fallback_code'][within(times, windows[case])]))))
    model_path = folders['historical_mars_figure8'] / '_workspace/flat/model.xml'
    noise = paired_noise(cases, all_data, weights, windows, model_path, protocol, stage)
    for case in cases:
        case_reports[case]['noise'] = noise[case]
    gate = registered_gate(case_reports, noise, cases[1:], protocol['registered_gate'])
    causal = historical_causality(all_data[cases[0]]['time_s'], all_data[cases[0]], case_reports[cases[0]], windows[cases[0]], weights[cases[0]], historical_body)
    for path in inputs:
        expected = next(x['sha256'] for x in json.loads((stage / 'input_manifest_before_reduction.json').read_text())['files'] if REPO / x['path'] == path)
        assert sha(path) == expected
    result = dict(iteration='I-46', scope=protocol['scope'], cases=case_reports, registered_gate=gate,
        historical_causality=causal,
        input_directories={case: str(folder.relative_to(REPO)) for case, folder in folders.items()},
        conventions='Effective feedback shares primary; raw locator separate. Mean/last1ms/next-hold native-history auxiliary truth kept separate; windows half-open and terminal excluded.',
        limitations=['Mismatch-equivalent seconds are sampled routing discrepancies, not measured wrong actuation over the following hold.',
            'Same-state command sensitivities do not establish a closed-loop improvement.',
            'Paired local wrench perturbations do not model physical sensor noise or filter attenuation.',
            'Mars and flat initial states/geometry differ; cross-case differences do not isolate slope as sole cause.',
            'No fast signal was actuated.'])
    save(stage / 'report.json', result)
    with (stage / 'case_metrics.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['case_id', 'truth_convention', 'source', 'completed', 'loaded_samples', 'unique_patch_samples',
            'routing_mismatch_equivalent_s', 'mean_share_error', 'adjacent_valid_share_TV', 'matched_events', 'extra_family_events',
            'max_normalized_requested_step', 'noise_label_flip_fraction'])
        writer.writeheader()
        for case in cases:
            report = case_reports[case]
            for target, table in report['truth_tables'].items():
                for source in SOURCES:
                    values = table['sources'][source]
                    writer.writerow(dict(case_id=case, truth_convention=target, source=source, completed=report['completed'],
                        loaded_samples=table['primary_loaded_samples'], unique_patch_samples=table['true_unique_patch_samples'],
                        routing_mismatch_equivalent_s=values['routing_mismatch_equivalent_s'],
                        mean_share_error=values['share_error_with_missing_penalty']['mean'], adjacent_valid_share_TV=values['share_total_variation_adjacent_valid'],
                        matched_events=len(values['event_matches']), extra_family_events=len(values['extra_estimated_events']),
                        max_normalized_requested_step=report['commands'][source]['requested']['normalized_adjacent_step']['maximum'],
                        noise_label_flip_fraction=noise[case]['sources'][source]['label_flip_fraction']))
    with (stage / 'paired_events.csv').open('w', newline='') as stream:
        fields = ['case_id', 'truth_convention', 'truth_index', 'truth_time_s', 'direction', 'native_signed_offset_s', 'fast_signed_offset_s', 'absolute_timing_improvement_s']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case in cases:
            for target, table in case_reports[case]['truth_tables'].items():
                writer.writerows(dict(case_id=case, truth_convention=target, **row) for row in table['common_paired_events'])
    paths = [p for p in stage.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    save(stage / 'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(stage)), bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(paths)]))


def pure_checks():
    t = np.arange(20) * DT
    labels = np.r_[np.zeros(4, int), np.ones(3, int), np.zeros(4, int), np.ones(9, int)]
    e = stable_family_events(t, labels)
    assert len(e['events']) == 1 and abs(e['events'][0]['time_s'] - .055) < 1e-12
    assert len(e['short_blips']) == 1
    truth = [dict(time_s=1., direction='shell_to_belt'), dict(time_s=1.1, direction='shell_to_belt')]
    estimate = [dict(time_s=.95, direction='shell_to_belt'), dict(time_s=1.05, direction='shell_to_belt')]
    matches = match_events(truth, estimate)
    assert [m['estimate_index'] for m in matches['matches']] == [0, 1]
    assert matches['matches'][0]['signed_offset_s'] < 0
    floating_tie = match_events([dict(time_s=1.1, direction='shell_to_belt')],
        [dict(time_s=1.05, direction='shell_to_belt'), dict(time_s=1.15, direction='shell_to_belt')])
    assert floating_tie['matches'][0]['estimate_index'] == 0
    assert not match_events([], [])['matches']
    value, valid, _ = shares(np.array([[1., 0, 0, 0], [0., 0, 0, 0], [.5, .5, 0, 0]]))
    assert dominant_labels(value, valid).tolist() == [0, -1, -2]
    assert within(np.array([.5, 1., 1.5]), (.5, 1.5)).tolist() == [True, True, False]
    assert stable_family_events(np.arange(8) * DT, np.r_[np.zeros(4, int), np.ones(4, int)])['events'][0]['time_s'] == .02
    t6 = np.arange(6) * DT
    loads = np.tile([1., 0, 0, 0], (6, 1))
    estimate = np.array([[1., 0, 0, 0], [0., 0, 0, 0], [0., 1, 0, 0], [0., 1, 0, 0], [0., 1, 0, 0], [0., 1, 0, 0]])
    spec = dict(stable_dwell_s=.015, max_gap_between_stable_runs_s=.05, pair_window_s=[-.1, .15])
    metrics = truth_metrics(t6, loads, np.ones(6, bool), np.zeros(6), dict(native=estimate, fast=estimate), [0, .03], 1., spec)
    assert metrics['sources']['native']['share_total_variation_adjacent_valid'] == 0
    assert metrics['sources']['native']['missing_estimate_samples'] == 1
    assert metrics['sources']['native']['share_error_with_missing_penalty']['maximum'] == 1
    native = np.zeros((6, 5)); native[1:, 0] = 2.
    data = dict(input_bounds=np.tile([2., 1., 1., .3, .3], (6, 1)), native_pre_sync_command=native,
        native_pre_sync_clipped_command=native, shadow_pre_sync_command=native,
        shadow_pre_sync_clipped_command=native, shadow_pre_sync_command_delta=np.zeros_like(native))
    assert command_metrics(t6, data, [0, .03])['native']['requested']['normalized_adjacent_step']['maximum'] == 1
    return dict(passed=True, checks=['three_samples_span10ms_not15ms', 'four_samples_confirm15ms', 'short_blip_does_not_become_event',
        'one_to_one_earlier_tie_signed_matching', 'empty_events_undefined_samples', 'missing_and_tie_labels', 'half_open_windows',
        'TV_does_not_bridge_missing_gap', 'missing_support_error_penalty', 'command_steps_normalized_per_actuator_bound',
        'floating_physical_grid_tie_chooses_earlier'])


def freeze(stage, mode):
    stage.mkdir(parents=True, exist_ok=False)
    source = stage / 'source'
    source.mkdir()
    sources = [Path(__file__), FAMILY / 'protocol.json', FAMILY / 'protocol_clarifications.json',
               REPO / 'src/spherical_wheel/contact_sensing.py', REPO / 'src/spherical_wheel/support_routing_audit.py']
    records = []
    for path in sources:
        if not path.is_file():
            raise FileNotFoundError('Required pre-outcome metric clarification/source: ' + str(path))
        shutil.copyfile(path, source / path.name)
        records.append(dict(origin=str(path.relative_to(REPO)), snapshot=str((source / path.name).relative_to(stage)), sha256=sha(path)))
    save(stage / 'declaration_before_execution.json', dict(mode=mode, captured_utc=datetime.now(timezone.utc).isoformat(),
        sources=records, boundary='Registered saved-array/noise-local-sensitivity reduction only; no control, scene or dynamics.'))
    return records


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checks-only', action='store_true')
    parser.add_argument('--tag', default='stage01')
    parser.add_argument('--case-dirs', type=Path, help='Explicit retained-setup-correction case-directory mapping, copied/hashed before reduction')
    args = parser.parse_args()
    if args.checks_only:
        output = HERE / 'development' / args.tag
        freeze(output, 'pure_metric_checks')
        save(output / 'report.json', pure_checks())
    else:
        output = HERE / args.tag
        freeze(output, 'registered_saved_data_reduction')
        save(output / 'pure_checks.json', pure_checks())
        reduce_main(output, json.loads((FAMILY / 'protocol.json').read_text()), args.case_dirs)
