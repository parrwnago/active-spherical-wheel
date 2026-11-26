"""I46 registered routing reduction; native data and non-actuated sensitivities."""
from __future__ import annotations
import argparse
from collections import Counter
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
            _, _, ei, delta = min(possible)
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
        family = stable_family_events(times, family_labels(estimate, valid), events_spec['stable_dwell_s'], events_spec['max_gap_between_stable_runs_s'])
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
        'TV_does_not_bridge_missing_gap', 'missing_support_error_penalty', 'command_steps_normalized_per_actuator_bound'])


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
    args = parser.parse_args()
    if args.checks_only:
        output = HERE / 'development' / args.tag
        freeze(output, 'pure_metric_checks')
        save(output / 'report.json', pure_checks())
    else:
        raise RuntimeError('Main reduction adapter not yet frozen')
