"""Independent probability-domain I71 recurrence and physical-point audit.

No imports from the production filter, frozen I70 helper or inference runner.
No contact truth, baseline decisions or references are accepted.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np

MODES = np.array([0, 1, 2, 3, 4, 5, 6, 7, 20], np.int64)
PAIRS = list(itertools.combinations(range(4), 2))


def prior():
    family_mass = .8/4
    p = np.array([family_mass*.9, family_mass*(1-.9)]*4+[.1])
    return p/p.sum()


def transition(dt):
    if not np.isfinite(dt) or dt <= 0: raise ValueError('positive finite elapsed time required')
    p = prior(); out = np.zeros((9, 9))
    for i in range(9):
        weights = p.copy()
        if i < 8:
            weights[:8] *= np.where(np.arange(8)//2 == i//2, 4., 1.)
            weights[8] *= 2.
        weights[i] = 0.
        stay = math.exp(-dt/(.02 if i == 8 else .05))
        out[i] = (1-stay)*weights/weights.sum()
        out[i, i] = stay
    return out


def edge_key(previous, current):
    if previous == 20 or current == 20 or previous//2 == current//2: return None
    pair = tuple(sorted((previous//2, current//2)))
    return PAIRS.index(pair), pair.index(current//2), current%2


def observation_matrix(node_cost, edge_cost=None):
    costs = np.broadcast_to(np.asarray(node_cost)[MODES], (9, 9)).copy()
    if edge_cost is not None:
        for i, old in enumerate(MODES):
            for j, new in enumerate(MODES):
                key = edge_key(int(old), int(new))
                if key is not None and np.isfinite(edge_cost[key]): costs[i, j] = edge_cost[key]
    return costs


def likelihood(cost, weight):
    c = np.asarray(cost, float)
    shift = np.min(c[np.isfinite(c)])
    with np.errstate(under='ignore'):
        return np.exp(-weight*(c-shift))


def summarize(p, joint=None):
    support = np.r_[p[:8].reshape(4, 2).sum(axis=1), p[8]]
    group = int(np.argmax(support))
    mask = np.arange(9)//2 == group if group < 4 else np.arange(9) == 8
    if joint is None:
        current = int(np.argmax(np.where(mask, p, -1.))); previous = -1
    else:
        previous, current = np.unravel_index(np.argmax(np.where(mask[None], joint, -1.)), (9, 9))
        previous = int(MODES[previous])
    padded = np.zeros(21); padded[MODES] = p
    return dict(probability=padded, support_index=group if group < 4 else 10,
                confidence=float(support[group]), selected_previous_mode=previous,
                selected_current_mode=int(MODES[current]))


def first_step(node_cost):
    p = prior()*likelihood(np.asarray(node_cost)[MODES], .005/.020)
    return summarize(p/p.sum())


def history_step(previous, node_cost, edge_cost, dt, *, initial):
    weight = min(1., (.005 if initial else dt)/.020)
    if initial:
        p = prior()*likelihood(np.asarray(node_cost)[MODES], weight)
        p /= p.sum(); joint = None
    else:
        joint = previous[:, None]*transition(dt)*likelihood(observation_matrix(node_cost, edge_cost), weight)
        p = joint.sum(axis=0); p /= p.sum()
    return p, summarize(p, joint), weight


def geometry(packet, i, decision, edges):
    current, previous = decision['selected_current_mode'], decision['selected_previous_mode']
    point = np.full((4, 3), np.nan); ids = np.full(4, -1, np.int64); edge = -1
    if current != 20:
        family = current//2
        key = edge_key(previous, current) if edges and previous >= 0 else None
        if key is not None and np.isfinite(packet['edge_cost'][i][key]):
            pair, role, motion = key
            point[family] = packet['edge_endpoint_points_m'][i, pair, role, motion]
            ids[family] = packet['edge_candidate_index'][i, pair, role, motion, role]
            edge = (pair*2+role)*2+motion
        else:
            point[family] = packet['node_endpoint_points_m'][i, current]
            ids[family] = packet['node_candidate_index'][i, current, 0]
    return dict(points_m=point, candidate_index=ids, selected_edge_index=edge)


def derive(packet, limit=None):
    n = len(packet['time_s']) if limit is None else min(limit, len(packet['time_s']))
    rows = []; beliefs = {'state_hmm': prior(), 'hmm': prior()}; previous_time = None
    for i in range(n):
        t = float(packet['time_s'][i]); dt = .005 if previous_time is None else t-previous_time
        if dt <= 0: raise ValueError('time must increase strictly')
        reset = previous_time is not None and dt > .05
        first = previous_time is None or reset
        node = packet['node_cost'][i]; decisions = {'emission': first_step(node)}
        for label in ('state_hmm', 'hmm'):
            beliefs[label], decisions[label], weight = history_step(beliefs[label], node,
                packet['edge_cost'][i] if label == 'hmm' else None, dt, initial=first)
        row = dict(time_s=packet['time_s'][i], evidence_weight=weight, history_reset=reset,
                   emission_probability21=decisions['emission']['probability'],
                   state_hmm_probability21=decisions['state_hmm']['probability'],
                   posterior21=decisions['hmm']['probability'])
        for label, d in decisions.items():
            for key in ('support_index', 'confidence', 'selected_previous_mode', 'selected_current_mode'):
                row[label+'_'+key] = d[key]
            for key, value in geometry(packet, i, d, label == 'hmm').items():
                row[label+'_'+key] = value
        rows.append(row); previous_time = t
    return {key: np.asarray([row[key] for row in rows]) for key in rows[0]}


def compare(expected, actual):
    if set(expected) != set(actual): raise ValueError('prediction output schema differs')
    results = {}; passed = True
    for key, reference in expected.items():
        value = actual[key]
        if reference.shape != value.shape: raise ValueError('shape differs: '+key)
        exact = np.array_equal(reference, value, equal_nan=True)
        probabilistic = key.endswith('probability21') or key == 'posterior21' or key.endswith('_confidence')
        finite = np.isfinite(reference) & np.isfinite(value)
        difference = float(np.max(abs(reference[finite]-value[finite]), initial=0))
        patterns = np.array_equal(np.isfinite(reference), np.isfinite(value))
        okay = bool(patterns and difference <= 1e-12) if probabilistic else bool(exact)
        results[key] = dict(pass_check=okay, exact=bool(exact), max_absolute_difference=difference,
                           numeric_tolerance=1e-12 if probabilistic else 0., finite_pattern_exact=bool(patterns))
        passed &= okay
    return dict(all_pass=bool(passed), arrays=results)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--base', type=Path, required=True)
    ap.add_argument('--seal', type=Path, required=True); ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    sealed = json.loads(args.seal.read_text())
    for path, expected in sealed['sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected: raise ValueError('source/input/output binding differs')
    cases = {}
    for case in ('m100_B', 'p030_B', 'p090_B'):
        with np.load(args.base/'inputs/stage01'/case/'inference_packet.npz', allow_pickle=False) as p:
            packet = {key: p[key] for key in p.files}
        with np.load(args.base/'inference/stage01'/case/'predictions.npz', allow_pickle=False) as p:
            actual = {key: p[key] for key in p.files}
        expected = derive(packet); prefix = derive(packet, 101)
        result = compare(expected, actual)
        prefix_independent = {key: value[:101] for key, value in expected.items()}
        result['prefix_101_independent_exact'] = all(np.array_equal(prefix[key], prefix_independent[key], equal_nan=True) for key in prefix)
        result['production_prefix_vs_causal_reference'] = compare(prefix, {key: value[:101] for key, value in actual.items()})['all_pass']
        result['frames'] = len(packet['time_s']); cases[case] = result
    args.output.mkdir(parents=True)
    report = dict(status='Independent probability-domain audit; no production helper imports',
        source_input_output_seal_sha256=hashlib.sha256(args.seal.read_bytes()).hexdigest(), cases=cases,
        all_pass=all(c['all_pass'] and c['prefix_101_independent_exact'] and c['production_prefix_vs_causal_reference'] for c in cases.values()),
        truth_read=False, nnls_run=False, scientific_parameters_changed=False,
        prefix_scope='Independent full/prefix recurrences are identical; saved production first101 rows match that causal reference. Production prefix code was not imported/executed.')
    (args.output/'report.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n')
    print(json.dumps(dict(all_pass=report['all_pass'], cases={c: dict(frames=v['frames'], all_pass=v['all_pass'],
        max_probability_difference=max(x['max_absolute_difference'] for k,x in v['arrays'].items() if k.endswith('probability21') or k=='posterior21'),
        prefix_101_exact=v['prefix_101_independent_exact']) for c,v in cases.items()})))
    if not report['all_pass']: raise RuntimeError('independent audit failed; preserve this stage')


if __name__ == '__main__': main()
