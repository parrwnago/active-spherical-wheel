"""Independent I72 geometry selection, fixed ranking and fallback audit.

No production selector, frozen ranker, input decoder or evaluation imports.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

MODES = ('geometry', 'wrench', 'velocity', 'fused')
FEATURES = ('excitation_m_s', 'family_index', 'frame_valid', 'moment_residual_norm',
            'normal_angle_rad', 'valid', 'velocity_residual_m_s', 'velocity_residual_norm_m_s', 'kind')
NORMAL_SCALE = math.radians(10.)


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_case(folder):
    with np.load(folder/'rank_packet.npz', allow_pickle=False) as p:
        packed = {key: p[key] for key in p.files}
    with np.load(folder/'native_fallback.npz', allow_pickle=False) as p:
        fallback = {key: p[key] for key in p.files}
    f = {key: packed['projected__'+key] for key in FEATURES}
    n, c = f['valid'].shape
    index = packed['point_flat_index']
    if not np.array_equal(index, np.flatnonzero(f['valid'].reshape(-1))):
        raise ValueError('sparse point rows differ from original validity')
    if not np.array_equal(packed['point_shape'], [n, c, 3]): raise ValueError('point shape differs')
    points = np.full((n, c, 3), np.nan)
    points.reshape(-1, 3)[index] = packed['point_values_m']
    return dict(time_s=packed['time_s'], projected_features=f, world_points=points,
                native_anchor_point_m=packed['native_anchor_point_m'], candidate_ids=packed['candidate_ids'],
                native_fallback=fallback)


def frame(features, points, anchor, ids):
    """Reference loops: nearest geometry first, then four fixed scalar scores."""
    family = np.asarray(features['family_index'], int)
    angle = features['normal_angle_rad']; speed = features['velocity_residual_norm_m_s']
    moment = features['moment_residual_norm']; vectors = features['velocity_residual_m_s']
    excitation = float(features['excitation_m_s'])
    finite = np.isfinite(angle) & np.isfinite(speed) & np.isfinite(moment) & np.isfinite(vectors).all(axis=1)
    finite &= (angle >= 0) & (speed >= 0) & (moment >= 0)
    active = features['valid'].astype(bool) & finite & bool(features['frame_valid']) & np.isfinite(excitation)
    if not np.isfinite(points[active]).all(): raise ValueError('missing active physical point')
    available = bool(np.isfinite(anchor).all())
    chosen_rows = np.full(4, -1, np.int64); chosen_ids = chosen_rows.copy()
    distances = np.full(4, np.inf); selected = np.zeros(len(ids), bool)
    fsum_differences = []
    if available:
        squared = np.sum((points-anchor)**2, axis=1)
        for label in range(4):
            options = np.flatnonzero(active & (family == label))
            if not len(options): continue
            minimum = min(float(squared[row]) for row in options)
            tied = [int(row) for row in options if float(squared[row]) == minimum]
            row = min(tied, key=lambda j: int(ids[j]))
            chosen_rows[label], chosen_ids[label], distances[label] = row, ids[row], minimum
            selected[row] = True
            # Secondary arithmetic observation only; it never changes a choice.
            accurate = {int(j): math.fsum(float(x)*float(x) for x in points[j]-anchor) for j in options}
            alternate = min((accurate[j], int(ids[j]), j) for j in accurate)[2]
            if alternate != row:
                fsum_differences.append(dict(family=label, numpy_candidate_id=int(ids[row]),
                    fsum_candidate_id=int(ids[alternate]), numpy_squared_distance=minimum,
                    fsum_squared_distance=accurate[alternate],
                    numpy_distance_gap=float(squared[alternate]-squared[row])))
    result = dict(selected_candidate_id=chosen_ids, selected_row_index=chosen_rows,
                  selected_mask=selected, family_distance_squared_m2=distances,
                  anchor_available=available, legacy_active_mask=active)
    for mode in MODES:
        scores = np.full(4, np.inf); candidates = np.full(4, -1, np.int64)
        for label, row in enumerate(chosen_rows):
            if row < 0: continue
            a = float(angle[row])/NORMAL_SCALE
            v = float(speed[row])/.02; m = float(moment[row])
            g, vc, wc = a*a, v*v, m*m
            score = {'geometry': g, 'wrench': g+wc, 'velocity': g+vc, 'fused': (g+wc)+vc}[mode]
            if np.isfinite(score): scores[label], candidates[label] = score, row
        order = sorted(range(4), key=lambda label: (float(scores[label]), label))
        known = [label for label in order if np.isfinite(scores[label])]
        count = len(known); first = known[0] if count else -1
        second = known[1] if count >= 2 else -1
        margin = float(scores[second]-scores[first]) if count >= 2 else np.nan
        tied = bool(count >= 2 and scores[first] == scores[second])
        best_speed = float(np.linalg.norm(vectors[candidates[first]])) if count else np.nan
        separation = float(np.linalg.norm(vectors[candidates[first]]-vectors[candidates[second]])) if count >= 2 else np.nan
        confident = bool(features['frame_valid'] and count >= 2 and excitation >= .02
            and margin >= .25 and best_speed <= .02 and separation >= .01 and not tied)
        rank = dict(family_scores=scores, candidate_index=candidates, chosen_family=first,
            accepted_family=first if confident else -1, score_margin=margin, valid_family_count=count,
            confident=confident, score_tied=tied, geometry_forced=count == 1,
            best_velocity_residual_m_s=best_speed, residual_separation_m_s=separation,
            excitation_m_s=excitation)
        point = np.full(3, np.nan); ident = row_index = -1
        if first >= 0:
            row_index = int(candidates[first]); ident = int(ids[row_index]); point = points[row_index].copy()
        for key, value in rank.items(): result[mode+'__'+key] = value
        result[mode+'__point_m'] = point
        result[mode+'__chosen_candidate_id'] = ident
        result[mode+'__chosen_row_index'] = row_index
        result[mode+'__accepted'] = confident
    return result, fsum_differences


def fallback_family(loads):
    labels = []
    for row in loads:
        values = [float(x) for x in row]
        if any(not math.isfinite(x) or x < 0 for x in values): labels.append(-1); continue
        maximum = max(values)
        labels.append(values.index(maximum) if maximum > 0 and sum(values) >= np.finfo(float).tiny
                      and values.count(maximum) == 1 else -1)
    return np.array(labels, np.int64)


def derive(data):
    n = len(data['time_s']); f = data['projected_features']; rows = []; arithmetic = []
    for i in range(n):
        local = {key: value if key in ('family_index', 'kind') else value[i] for key, value in f.items()}
        out, alternate = frame(local, data['world_points'][i], data['native_anchor_point_m'][i], data['candidate_ids'])
        out['time_s'] = data['time_s'][i]; rows.append(out)
        arithmetic.extend(dict(frame=i, **entry) for entry in alternate)
    arrays = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
    native = data['native_fallback']; labels = fallback_family(native['native_support_loads_N'])
    if not np.array_equal(labels, native['native_family']): raise ValueError('native fallback rule differs')
    arrays['native_family'] = labels; arrays['native_point_m'] = native['native_point_m'].copy()
    for prefix, mode in (('C', 'fused'), ('C_VG', 'velocity')):
        use = arrays[mode+'__accepted']
        arrays[prefix+'_family'] = np.where(use, arrays[mode+'__accepted_family'], labels)
        arrays[prefix+'_point_m'] = np.where(use[:, None], arrays[mode+'__point_m'], native['native_point_m'])
        arrays[prefix+'_used_proposal'] = use.copy()
    return arrays, arithmetic


def subset(data, start, stop):
    f = {key: value if key in ('family_index', 'kind') else value[start:stop]
         for key, value in data['projected_features'].items()}
    fallback = {key: value[start:stop] for key, value in data['native_fallback'].items()}
    return dict(time_s=data['time_s'][start:stop], projected_features=f,
                world_points=data['world_points'][start:stop],
                native_anchor_point_m=data['native_anchor_point_m'][start:stop],
                candidate_ids=data['candidate_ids'], native_fallback=fallback)


def compare(reference, actual):
    if set(reference) != set(actual): raise ValueError('output schema mismatch')
    numeric_suffixes = ('__family_scores', '__score_margin', '__best_velocity_residual_m_s', '__residual_separation_m_s')
    result = {}; passed = True
    for key, expected in reference.items():
        value = actual[key]
        if expected.shape != value.shape: raise ValueError('shape mismatch: '+key)
        exact = bool(np.array_equal(expected, value, equal_nan=True))
        numerical = key == 'family_distance_squared_m2' or key.endswith(numeric_suffixes)
        if numerical:
            okay = bool(np.all(np.isclose(expected, value, atol=1e-12, rtol=1e-12, equal_nan=True)))
            finite = np.isfinite(expected) & np.isfinite(value)
            max_diff = float(np.max(abs(expected[finite]-value[finite]), initial=0))
        else:
            okay, max_diff = exact, 0. if exact else None
        result[key] = dict(pass_check=okay, exact=exact, max_absolute_difference=max_diff,
                           atol=1e-12 if numerical else 0., rtol=1e-12 if numerical else 0.)
        passed &= okay
    return dict(all_pass=bool(passed), arrays=result)


def fingerprint(data):
    h = hashlib.sha256()
    for group in (data['projected_features'], data['native_fallback']):
        for key in sorted(group): h.update(key.encode()); h.update(np.ascontiguousarray(group[key]).tobytes())
    for key in ('time_s', 'world_points', 'native_anchor_point_m', 'candidate_ids'):
        h.update(key.encode()); h.update(np.ascontiguousarray(data[key]).tobytes())
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(); p.add_argument('--base', type=Path, required=True)
    p.add_argument('--seal', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    sealed = json.loads(args.seal.read_text())
    for path, expected in sealed['sha256'].items():
        if sha(path) != expected: raise ValueError('sealed source/input/output changed')
    reports = {}
    for case in ('m100_B', 'p030_B', 'p090_B'):
        data = load_case(args.base/'inputs/stage01'/case); before = fingerprint(data)
        expected, arithmetic = derive(data)
        with np.load(args.base/'inference/stage01'/case/'predictions.npz', allow_pickle=False) as p:
            actual = {key: p[key] for key in p.files}
        result = compare(expected, actual)
        prefix, _ = derive(subset(data, 0, 101))
        chunks = [derive(subset(data, a, b))[0] for a, b in ((0, 101), (101, len(data['time_s'])))]
        joined = {key: np.concatenate([chunk[key] for chunk in chunks], axis=0) for key in expected}
        result.update(frames=len(data['time_s']), input_arrays_unchanged=before == fingerprint(data),
            independent_prefix101_exact=all(np.array_equal(prefix[key], expected[key][:101], equal_nan=True) for key in expected),
            independent_batch_partition_exact=all(np.array_equal(joined[key], expected[key], equal_nan=True) for key in expected),
            production_prefix_matches_reference=compare(prefix, {key: value[:101] for key, value in actual.items()})['all_pass'],
            alternate_fsum_choice_difference_count=len(arithmetic), alternate_fsum_examples=arithmetic[:20])
        reports[case] = result
    for path, expected in sealed['sha256'].items():
        if sha(path) != expected: raise ValueError('bound source/input/output changed during audit')
    report = dict(status='Independent current-frame selection and fixed scoring; no production selector/rank imports',
        cases=reports, all_pass=all(r['all_pass'] and r['input_arrays_unchanged'] and r['independent_prefix101_exact']
             and r['independent_batch_partition_exact'] and r['production_prefix_matches_reference'] for r in reports.values()),
        source_input_output_seal_sha256=sha(args.seal), truth_read=False, nnls_run=False,
        parameters_changed=False,
        alternate_arithmetic_scope='math.fsum is a secondary near-tie observation only; it never changes the audited NumPy squared-sum choice.',
        prefix_scope='Independent full/prefix/batch computations agree and saved production prefix matches reference; production selector was not imported or rerun.',
        immutability_scope='Independent decoded arrays and bound source/input/output files unchanged; production in-memory immutability is covered by its separate contracts.')
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n')
    print(json.dumps(dict(all_pass=report['all_pass'], cases={case: dict(frames=r['frames'], all_pass=r['all_pass'],
        max_numeric_difference=max(x['max_absolute_difference'] or 0. for x in r['arrays'].values()),
        fsum_choice_differences=r['alternate_fsum_choice_difference_count']) for case,r in reports.items()})))
    if not report['all_pass']: raise RuntimeError('independent selector audit failed; preserve this stage')


if __name__ == '__main__': main()
