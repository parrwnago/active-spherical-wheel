"""I-70 retrospective evaluation of frozen causal contact-mode beliefs.

Only this evaluator reads pressure-label evaluation data.  It does not call the
inference implementation, fit its parameters, or step any simulator.  Contact
support probabilities sum rolling/slipping submodes before MAP and calibration.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import math
import shutil
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
METHODS = ('N', 'F', 'A', 'emission', 'hmm')
# Family order is shell_A, shell_B, belt_A, belt_B.  Pair order matches itertools.
SUPPORT_BITS = np.array([1, 2, 4, 8, 3, 5, 9, 6, 10, 12, 0], np.int8)
STATE_BITS = np.repeat(SUPPORT_BITS[:10], 2).tolist() + [0]


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
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False) + '\n')


def load(path):
    with np.load(path, allow_pickle=False) as packet:
        return {k: packet[k].copy() for k in packet.files}


def fraction(n, d):
    return float(n / d) if d else None


def summary(values):
    values = np.asarray(values, float)
    values = values[np.isfinite(values)]
    if not len(values):
        return dict(count=0, mean=None, rms=None, median=None, p95=None, maximum=None)
    return dict(count=len(values), mean=float(np.mean(values)),
                rms=float(np.sqrt(np.mean(values * values))), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), maximum=float(np.max(values)))


def primary_window(time_s):
    return (time_s >= 1. - 1e-9) & (time_s < 41. - 1e-9)


def family_bits(family):
    family = np.asarray(family, int)
    result = np.zeros(len(family), np.int8)
    valid = (family >= 0) & (family < 4)
    result[valid] = (1 << family[valid]).astype(np.int8)
    return result


def singleton_family(bits):
    result = np.full(len(bits), -1, np.int8)
    for family in range(4):
        result[np.asarray(bits) == 1 << family] = family
    return result


def truth_labels(loads, minimum_total=1., dominant_share=.9, family_share=.1):
    loads = np.asarray(loads, float)
    if loads.ndim != 2 or loads.shape[1] != 4:
        raise ValueError('Four family pressure loads required')
    finite = np.isfinite(loads).all(axis=1) & (loads >= 0).all(axis=1)
    clean = np.where(np.isfinite(loads), loads, 0.)
    total = clean.sum(axis=1)
    shares = np.divide(clean, total[:, None], out=np.zeros_like(clean), where=total[:, None] > 0)
    loaded = finite & (total >= minimum_total)
    maximum = clean.max(axis=1)
    unique = (clean == maximum[:, None]).sum(axis=1) == 1
    dominant = loaded & unique & (shares.max(axis=1) >= dominant_share)
    family = np.where(dominant, np.argmax(clean, axis=1), -1).astype(np.int8)
    bits = np.sum((shares >= family_share) * (1 << np.arange(4)), axis=1).astype(np.int8)
    bits[~loaded] = 0
    category = np.full(len(loads), -1, np.int8)
    for index, support in enumerate(SUPPORT_BITS[:-1]):
        category[loaded & (bits == support)] = index
    return dict(truth_family=family, truth_dominant=dominant, truth_loaded=loaded,
                truth_support_bits=bits, truth_support_category=category,
                truth_support_representable=loaded & (category >= 0), truth_shares=shares)


def collapse(probability21):
    probability21 = np.asarray(probability21, float)
    if probability21.ndim != 2 or probability21.shape[1] != 21:
        raise ValueError('Expected N by 21 mode probabilities')
    if (not np.isfinite(probability21).all() or (probability21 < 0).any()
            or not np.allclose(probability21.sum(axis=1), 1., rtol=0., atol=1e-10)):
        raise ValueError('Invalid probabilities; evaluator never repairs or renormalizes them')
    return np.column_stack([probability21[:, 2*i:2*i+2].sum(axis=1) for i in range(10)]
                           + [probability21[:, 20]])


def onehot(bits):
    result = np.zeros((len(bits), 11))
    for j, support in enumerate(SUPPORT_BITS):
        result[np.asarray(bits) == support, j] = 1.
    if not np.all(result.sum(axis=1) == 1.):
        raise ValueError('Prediction lies outside representable support sets')
    return result


def decode(probability11):
    index = np.argmax(probability11, axis=1)
    return SUPPORT_BITS[index], probability11[np.arange(len(index)), index]


def reliability(confidence, correct, mask, bins=10):
    rows = []
    denominator = int(np.sum(mask))
    weighted_error = 0.
    for i in range(bins):
        lo, hi = i / bins, (i + 1) / bins
        selected = mask & (confidence >= lo) & ((confidence < hi) if i < bins-1 else (confidence <= hi))
        count = int(selected.sum())
        mean_conf = float(np.mean(confidence[selected])) if count else None
        accuracy = float(np.mean(correct[selected])) if count else None
        if count:
            weighted_error += count * abs(mean_conf - accuracy)
        rows.append(dict(lower=lo, upper=hi, upper_inclusive=i == bins-1, count=count,
                         mean_confidence=mean_conf, accuracy=accuracy))
    return dict(denominator=denominator, ece=fraction(weighted_error, denominator), bins=rows)


def method_metrics(bits, probability, truth, window):
    dominant = window & truth['truth_dominant']
    loaded = window & truth['truth_loaded']
    representable = window & truth['truth_support_representable']
    family = singleton_family(bits)
    correct_family = family == truth['truth_family']
    correct_set = (bits == truth['truth_support_bits']) & (bits != 0)
    confidence = np.max(probability, axis=1)
    confident = (confidence >= .8) & (bits != 0)
    representable_index = np.maximum(truth['truth_support_category'], 0)
    teacher = np.eye(11)[representable_index]
    brier = np.sum((probability - teacher)**2, axis=1)
    family_inclusion = np.column_stack([np.sum(probability[:, (SUPPORT_BITS & (1 << f)) != 0], axis=1)
                                       for f in range(4)])
    teacher_inclusion = np.column_stack([(truth['truth_support_bits'] & (1 << f)) != 0 for f in range(4)])
    family_brier = np.mean((family_inclusion - teacher_inclusion)**2, axis=1)
    d, l = int(dominant.sum()), int(loaded.sum())
    errors = int(np.sum(dominant & ~correct_family))
    result = dict(singleton=dict(denominator=d, errors=errors, correct=d-errors,
                    error_fraction=fraction(errors, d), unknown=int(np.sum(dominant & (bits == 0))),
                    pair_output=int(np.sum(dominant & (family < 0) & (bits != 0)))),
                multiset=dict(denominator=l, correct=int(np.sum(loaded & correct_set)),
                    errors=int(np.sum(loaded & ~correct_set)),
                    accuracy=fraction(int(np.sum(loaded & correct_set)), l),
                    unsupported_truth_count=int(np.sum(loaded & ~truth['truth_support_representable'])),
                    unknown=int(np.sum(loaded & (bits == 0)))),
                confidence=dict(threshold=.8, denominator=d, covered=int(np.sum(dominant & confident)),
                    coverage=fraction(int(np.sum(dominant & confident)), d),
                    wrong=int(np.sum(dominant & confident & ~correct_family)),
                    correct=int(np.sum(dominant & confident & correct_family))),
                calibration=dict(representable_count=int(representable.sum()),
                    set_brier_mean=float(np.mean(brier[representable])) if representable.any() else None,
                    family_inclusion_brier_mean=float(np.mean(family_brier[loaded])) if loaded.any() else None,
                    support_reliability=reliability(confidence, correct_set, representable)),
                risk_coverage=[])
    for threshold in np.linspace(0., 1., 11):
        selected = dominant & (confidence >= threshold) & (bits != 0)
        count = int(selected.sum())
        wrong = int(np.sum(selected & ~correct_family))
        result['risk_coverage'].append(dict(threshold=float(threshold), covered=count,
                    coverage=fraction(count, d), errors=wrong, error_fraction=fraction(wrong, count)))
    return result, dict(bits=bits, family=family, probability=probability, confidence=confidence,
                        correct_family=correct_family, correct_set=correct_set,
                        set_brier=brier, family_inclusion_brier=family_brier)


def runs(time, values):
    """Contiguous half-open runs; a time gap always splits a run."""
    rows = []
    start = 0
    for end in range(1, len(values)+1):
        split = end == len(values) or values[end] != values[start] or abs(time[end]-time[end-1]-.005) > 1e-9
        if split:
            rows.append((start, end, int(values[start])))
            start = end
    return rows


def timing(time, truth_family, dominant, predictions, config=None):
    config = config or {}
    stable_s = config.get('truth_stable_s', .050)
    predicted_s = config.get('prediction_stable_s', .025)
    bracket_s = config.get('match_bracket_s', .100)
    gap_s = config.get('maximum_truth_gap_s', .100)
    labels = np.where(dominant, truth_family, -1)
    stable = [r for r in runs(time, labels) if r[2] >= 0 and time[r[1]-1]-time[r[0]]+.005 >= stable_s-1e-9]
    stable_domain = np.zeros(len(time), bool)
    for start, end, family in stable:
        stable_domain[start:end] = True
    events = []
    for old, new in zip(stable, stable[1:]):
        gap = float(time[new[0]] - time[old[1]-1] - .005)
        reference = float(time[new[0]])
        if old[2] == new[2] or gap > gap_s+1e-9 or not 1.-1e-9 <= reference < 41.-1e-9:
            continue
        events.append(dict(old_family=old[2], new_family=new[2], old_last_s=float(time[old[1]-1]),
                           reference_s=reference, gap_s=max(0., gap), methods={}))
    reports = {}
    for name, bits in predictions.items():
        predicted = singleton_family(bits)
        pruns = runs(time, predicted)
        used = set()
        offsets, missed, reverted = [], 0, 0
        for event in events:
            candidates = [(j, r) for j, r in enumerate(pruns) if r[2] == event['new_family']
                          and abs(float(time[r[0]])-event['reference_s']) <= bracket_s+1e-9]
            sustained = [(j, r) for j, r in candidates if j not in used and
                         time[r[1]-1]-time[r[0]]+.005 >= predicted_s-1e-9]
            short = [r for j, r in candidates if time[r[1]-1]-time[r[0]]+.005 < predicted_s-1e-9]
            reverted += len(short)
            row = dict(reverted_attempts=len(short), candidate_onsets_s=[float(time[r[0]]) for j, r in candidates])
            if sustained:
                j, chosen = min(sustained, key=lambda item:(abs(float(time[item[1][0]])-event['reference_s']), item[1][0]))
                used.add(j)
                offset = float(time[chosen[0]]) - event['reference_s']
                offsets.append(offset)
                row.update(onset_s=float(time[chosen[0]]), signed_latency_s=offset, missed=False)
            else:
                missed += 1
                row.update(onset_s=None, signed_latency_s=None, missed=True)
            event['methods'][name] = row
        window = primary_window(time)
        # A false switch is a changed emitted support (including unknown/pair)
        # between adjacent samples with the same dominant truth family.
        adjacent = abs(np.diff(time)-.005) <= 1e-9
        stable_pair = window[:-1] & window[1:] & stable_domain[:-1] & stable_domain[1:] & (truth_family[:-1] == truth_family[1:]) & adjacent
        false = int(np.sum(stable_pair & (bits[:-1] != bits[1:])))
        changed = stable_pair & (bits[:-1] != bits[1:])
        singleton_toggle = changed & (predicted[:-1] >= 0) & (predicted[1:] >= 0)
        unknown_toggle = changed & ((bits[:-1] == 0) | (bits[1:] == 0))
        pair_toggle = changed & ~singleton_toggle & ~unknown_toggle
        reports[name] = dict(events=len(events), matched=len(offsets), missed=missed,
                reverted_attempts=reverted, false_switches=false,
                false_singleton_toggles=int(singleton_toggle.sum()),
                false_unknown_transitions=int(unknown_toggle.sum()), false_pair_transitions=int(pair_toggle.sum()),
                signed_latency_s=summary(offsets), absolute_latency_s=summary(np.abs(offsets)))
    return dict(events=events, methods=reports,
                reference='First sample of >=50ms stable dominant-positive-pressure family, not physical first touch.',
                observation_clock='Last1ms solver sample at5ms snapshots; not interval force truth.')


def point_metrics(points, methods, truth, truth_points, window):
    """Per-emitted-singleton point against sampled dominant physical centroid.

    Wrong-family singletons remain in the all-singleton result.  Pairs/unknowns
    have no scalar point and are counted as missing coverage, never zero error.
    Correct-family matched masks compare methods on identical rows separately.
    """
    dominant = window & truth['truth_dominant']
    row = np.arange(len(dominant))
    target = truth_points[row, np.maximum(truth['truth_family'], 0)]
    target_valid = np.isfinite(target).all(axis=1)
    per_method = {}
    distances, validity, singletons = {}, {}, {}
    for name, arrays in methods.items():
        family = arrays['family']
        p = points[name][row, np.maximum(family, 0)]
        valid = (family >= 0) & np.isfinite(p).all(axis=1) & target_valid
        distance = np.linalg.norm(p-target, axis=1)*1000.
        distance[~valid] = np.nan
        singletons[name] = family >= 0
        distances[name], validity[name] = distance, valid
        selected = dominant & valid
        correct = selected & arrays['correct_family']
        per_method[name] = dict(denominator=int(dominant.sum()), scorable=int(selected.sum()),
            missing=int(np.sum(dominant & ~valid)), singleton_count=int(np.sum(dominant & (family >= 0))),
            all_scorable_singletons_mm=summary(distance[selected]), correct_family_only_mm=summary(distance[correct]))
    matched = dominant & target_valid
    for name in METHODS:
        matched &= validity[name] & methods[name]['correct_family']
    all_singletons = dominant & target_valid
    for name in METHODS:
        all_singletons &= validity[name]
    for name in METHODS:
        per_method[name]['all_methods_correct_family_matched_mm'] = summary(distances[name][matched])
        per_method[name]['all_methods_singleton_matched_mm'] = summary(distances[name][all_singletons])
    return dict(methods=per_method, all_methods_correct_family_matched_count=int(matched.sum()),
                all_methods_singleton_matched_count=int(all_singletons.sum())), distances, validity


def evaluate_case(packet, predictions, config=None):
    config = config or {}
    time = np.asarray(packet['time_s'])
    if len(time) < 2 or not np.isfinite(time).all() or (np.diff(time) <= 0).any():
        raise ValueError('Strictly increasing finite clock required')
    if not np.array_equal(time, predictions['time_s']):
        raise ValueError('Inference and evaluation clocks differ')
    truth = truth_labels(packet['truth_patch_loads_N'])
    window = primary_window(time)
    probability = {name: onehot(family_bits(packet[name+'_family'])) for name in ('N', 'F', 'A')}
    probability.update(emission=collapse(predictions['emission_probability21']), hmm=collapse(predictions['posterior21']))
    report, methods = {}, {}
    for name in METHODS:
        bits, confidence = decode(probability[name])
        report[name], methods[name] = method_metrics(bits, probability[name], truth, window)
    point_report, distances, validity = point_metrics(
        {name: packet[name+'_points_m'] for name in METHODS}, methods, truth,
        packet['truth_patch_points_m'], window)
    timing_report = timing(time, truth['truth_family'], truth['truth_dominant'],
                           {name: value['bits'] for name, value in methods.items()}, config.get('timing'))
    frames = dict(time_s=time, primary_mask=window, **truth)
    for name, value in methods.items():
        for key, array in value.items():
            frames[name+'__'+key] = array
        frames[name+'__point_distance_mm'] = distances[name]
        frames[name+'__point_valid'] = validity[name]
    return frames, dict(counts=dict(primary=int(window.sum()),
                dominant=int(np.sum(window & truth['truth_dominant'])), loaded=int(np.sum(window & truth['truth_loaded'])),
                representable=int(np.sum(window & truth['truth_support_representable']))),
                methods=report, timing=timing_report, points=point_report)


def aggregate(cases, frames):
    joined = {key: np.concatenate([frames[c][key] for c in CASES]) for key in frames[CASES[0]]}
    truth = {key: value for key, value in joined.items() if key.startswith('truth_')}
    window = joined['primary_mask']
    methods = {}
    for name in METHODS:
        methods[name], _ = method_metrics(joined[name+'__bits'], joined[name+'__probability'], truth, window)
    timing_report = {}
    for name in METHODS:
        offsets = [event['methods'][name]['signed_latency_s'] for c in CASES
                   for event in cases[c]['timing']['events'] if not event['methods'][name]['missed']]
        keys = ('events', 'matched', 'missed', 'reverted_attempts', 'false_switches',
                'false_singleton_toggles', 'false_unknown_transitions', 'false_pair_transitions')
        timing_report[name] = {key: sum(cases[c]['timing']['methods'][name][key] for c in CASES) for key in keys}
        timing_report[name].update(signed_latency_s=summary(offsets), absolute_latency_s=summary(np.abs(offsets)))
    points = {}
    all_correct = window & joined['truth_dominant']
    all_singleton = all_correct.copy()
    for name in METHODS:
        all_correct &= joined[name+'__point_valid'] & joined[name+'__correct_family']
        all_singleton &= joined[name+'__point_valid']
    for name in METHODS:
        domain = window & joined['truth_dominant']
        valid = joined[name+'__point_valid']
        distance = joined[name+'__point_distance_mm']
        mask = domain & valid
        points[name] = dict(denominator=int(domain.sum()), scorable=int(mask.sum()),
                missing=int(np.sum(domain & ~valid)),
                all_scorable_singletons_mm=summary(distance[mask]),
                correct_family_only_mm=summary(distance[mask & joined[name+'__correct_family']]),
                all_methods_correct_family_matched_mm=summary(distance[all_correct]),
                all_methods_singleton_matched_mm=summary(distance[all_singleton]))
    return dict(counts=dict(primary=int(window.sum()), dominant=int(np.sum(window & joined['truth_dominant'])),
                loaded=int(np.sum(window & joined['truth_loaded'])), representable=int(np.sum(window & joined['truth_support_representable']))),
                methods=methods, timing=timing_report, points=points,
                all_methods_correct_family_matched_count=int(all_correct.sum()),
                all_methods_singleton_matched_count=int(all_singleton.sum()))


def gates(pooled, cases):
    """Root-approved prespecified scientific gates; no parameter fitting."""
    rows = []
    def add(name, actual, limit, passed, **context):
        rows.append(dict(name=name, actual=actual, limit=limit, passed=bool(passed), **context))
    H, A = pooled['methods']['hmm'], pooled['methods']['A']
    add('pooled_primary_error_reduction_10pct', H['singleton']['errors'], .9*A['singleton']['errors'],
        H['singleton']['errors'] <= .9*A['singleton']['errors'])
    for case in CASES:
        h, a = cases[case]['methods']['hmm'], cases[case]['methods']['A']
        hr, ar = h['singleton']['error_fraction'], a['singleton']['error_fraction']
        add(case+'_primary_error_nonregression_0p1pp', hr, None if ar is None else ar+.001,
            hr is not None and ar is not None and hr <= ar+.001+1e-15)
    coverage = H['confidence']['coverage']
    add('high_confidence_coverage_80pct', coverage, .8, coverage is not None and coverage >= .8)
    add('high_confidence_errors_no_more_than_A_total', H['confidence']['wrong'], A['singleton']['errors'],
        H['confidence']['wrong'] <= A['singleton']['errors'])
    hacc, aacc = H['multiset']['accuracy'], A['multiset']['accuracy']
    add('all_loaded_multiset_nonregression', hacc, aacc,
        hacc is not None and aacc is not None and hacc >= aacc-1e-15)
    hb, ab = H['calibration']['set_brier_mean'], A['calibration']['set_brier_mean']
    add('representable_set_brier_nonregression', hb, ab,
        hb is not None and ab is not None and hb <= ab+1e-12)
    t, e = pooled['timing']['hmm'], pooled['timing']['emission']
    for key, bound in (('median', .020), ('p95', .050)):
        value = t['absolute_latency_s'][key]
        add('transition_absolute_latency_'+key, value, bound,
            value is not None and value <= bound+1e-9,
            scorable_events=t['matched'], total_events=t['events'])
    for key in ('missed', 'reverted_attempts', 'false_switches'):
        add('transition_'+key+'_no_more_than_emission', t[key], e[key], t[key] <= e[key])
    return rows


def fixed_event_report(frames, anchors):
    """The anchor list is protocol-owned; no event selection based on new scores."""
    result = []
    for anchor in anchors:
        case, time = anchor['case'], float(anchor['time_s'])
        f = frames[case]
        possible = np.flatnonzero(abs(f['time_s']-time) <= 1e-9)
        if len(possible) != 1:
            raise ValueError('Fixed anchor does not match exactly one archived sample')
        i = int(possible[0])
        methods = {}
        for name in METHODS:
            methods[name] = {key: safe(f[name+'__'+key][i]) for key in
                ('bits', 'family', 'confidence', 'correct_family', 'correct_set', 'point_distance_mm', 'point_valid')}
        result.append(dict(**anchor, row=i, truth_family=int(f['truth_family'][i]),
                      truth_support_bits=int(f['truth_support_bits'][i]), methods=methods))
    return result


def outliers(frames, count=10):
    result = {}
    for name in METHODS:
        candidates = []
        for case in CASES:
            f = frames[case]
            mask = f['primary_mask'] & f['truth_dominant'] & f[name+'__point_valid']
            for i in np.flatnonzero(mask):
                candidates.append((float(f[name+'__point_distance_mm'][i]), case, int(i)))
        chosen = sorted(candidates, key=lambda value:(-value[0], value[1], value[2]))[:count]
        result[name] = [dict(case=case, row=i, time_s=float(frames[case]['time_s'][i]),
                            distance_mm=distance, correct_family=bool(frames[case][name+'__correct_family'][i]))
                        for distance, case, i in chosen]
    return result


def run(protocol_path, packet_root, prediction_root, output):
    """Read compact evaluation packets only after inference artifacts are sealed.

    Packet API is documented in schema.json.  All paths and file hashes are
    recorded before opening arrays, and checked again after arithmetic.
    """
    protocol = json.loads(Path(protocol_path).read_text())
    output.mkdir(parents=True, exist_ok=False)
    bindings = []
    for case in CASES:
        for role, path in (('evaluation_packet', packet_root/case/'evaluation_packet.npz'),
                           ('prediction', prediction_root/case/'predictions.npz')):
            bindings.append(dict(case=case, role=role, path=str(path), sha256=sha(path)))
    write(output/'registered_before_truth_read.json', dict(iteration='I-70',
        utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)),
        protocol_sha256=sha(protocol_path), files=bindings))
    shutil.copyfile(Path(__file__), output/'evaluate_temporal.py')
    shutil.copyfile(protocol_path, output/'protocol.json')
    cases, frames = {}, {}
    for case in CASES:
        packet = load(packet_root/case/'evaluation_packet.npz')
        predictions = load(prediction_root/case/'predictions.npz')
        if len(packet['time_s']) != 8801 or int(primary_window(packet['time_s']).sum()) != 8000:
            raise ValueError('Registered complete44s archive/half-open primary window required')
        frames[case], cases[case] = evaluate_case(packet, predictions, protocol)
        np.savez_compressed(output/(case+'_frames.npz'), **frames[case])
    pooled = aggregate(cases, frames)
    checks = gates(pooled, cases)
    anchors = protocol.get('fixed_events', [])
    science = dict(iteration='I-70', new_physics_steps=0, live_controller_changed=False,
        interpretation='Retrospective causal shadow comparison; beliefs are model-conditional and calibration is evaluated, not guaranteed. Positive-pressure support labels do not identify all force-bearing bodies.',
        evaluation_clock='Last1ms positive-normal-pressure samples at5ms snapshots. No interval contact truth available for all3 original episodes.',
        cases=cases, pooled=pooled, gates=checks,
        all_gates_passed=all(row['passed'] for row in checks),
        failed_gates=[row['name'] for row in checks if not row['passed']],
        fixed_events=fixed_event_report(frames, anchors), point_outliers_top10=outliers(frames))
    for binding in bindings:
        if sha(binding['path']) != binding['sha256']:
            raise ValueError('Evaluation input changed during arithmetic')
    write(output/'science.json', science)
    write(output/'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(output)), sha256=sha(p), bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json']))
    print(json.dumps(safe(dict(all_gates_passed=science['all_gates_passed'],
        failed_gates=science['failed_gates'], pooled=pooled)), allow_nan=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--packet-root', type=Path, required=True)
    parser.add_argument('--prediction-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.protocol.resolve(), args.packet_root.resolve(), args.prediction_root.resolve(), args.output.resolve())

