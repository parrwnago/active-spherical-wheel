"""Independent probability-domain I70 recurrence audit; no method imports.

Source written before sealed inference outputs are released.  Reads only sealed
model costs/predictions.  It neither re-fits forces nor changes a prediction.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import itertools
import json
import math
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
GROUP_BITS = np.array([1, 2, 4, 8, 3, 5, 9, 6, 10, 12, 0])
BITS = np.r_[np.repeat(GROUP_BITS[:-1], 2), 0]
PAIRS = tuple(itertools.combinations(range(4), 2))
PRIOR = np.r_[np.tile([.8/4*.9, .8/4*.1], 4), np.tile([.1/6*.9, .1/6*.1], 6), .1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as packet:
        return {key:packet[key].copy() for key in packet.files}


def transition(dt):
    out = np.zeros((21, 21))
    for i, old in enumerate(BITS):
        weights = np.zeros(21)
        for j, new in enumerate(BITS):
            if i == j:
                continue
            if old == 0:
                affinity = 1.
            elif new == 0:
                affinity = 2.
            elif old == new:
                affinity = 4.
            elif old & new in (old, new):
                affinity = 3.
            elif old in (1, 2, 4, 8) and new in (1, 2, 4, 8):
                affinity = 1.
            elif old & new:
                affinity = .5
            else:
                affinity = .1
            weights[j] = affinity*PRIOR[j]
        stay = math.exp(-dt/(.02 if old == 0 else .05))
        out[i] = weights/weights.sum()*(1.-stay)
        out[i, i] = stay
    return out


def edge_index(previous, current):
    old, new = int(BITS[previous]), int(BITS[current])
    if not old or old == new or new not in (1, 2, 4, 8):
        return None
    union = old | new
    families = tuple(f for f in range(4) if union & (1 << f))
    if len(families) != 2:
        return None
    endpoint = (1, 2, 4, 8).index(new)
    return PAIRS.index(families), families.index(endpoint), current % 2


def collapsed(p):
    return np.r_[p[:20].reshape(10, 2).sum(axis=1), p[20]]


def expected(time, node, edge):
    state = PRIOR.copy(); temporal = PRIOR.copy(); previous = None
    rows = []
    for index, t in enumerate(time):
        first = previous is None
        dt = .005 if first else float(t-previous)
        if dt <= 0:
            raise ValueError('Increasing times required')
        if not first and dt > .05:
            first = True; state = PRIOR.copy(); temporal = PRIOR.copy()
        weight = min(1.,(.005 if first else dt)/.02)
        frame_log = -weight*node[index]
        frame_likelihood = np.exp(frame_log-np.max(frame_log))
        frame = PRIOR*frame_likelihood; frame /= frame.sum()
        if first:
            state = frame.copy(); temporal = frame.copy()
            selected_previous = -1
            group = int(np.argmax(collapsed(temporal)))
            selected_current = int(np.argmax(np.where(BITS == GROUP_BITS[group], temporal, -1.)))
        else:
            trans = transition(dt)
            state = (state @ trans)*frame_likelihood; state /= state.sum()
            edge_log = np.broadcast_to(-weight*node[index], (21, 21)).copy()
            for i in range(21):
                for j in range(21):
                    key = edge_index(i, j)
                    if key is not None and np.isfinite(edge[index][key]):
                        edge_log[i, j] = -weight*edge[index][key]
            joint = temporal[:, None]*trans*np.exp(edge_log-np.max(edge_log))
            temporal = joint.sum(axis=0); temporal /= temporal.sum()
            group = int(np.argmax(collapsed(temporal)))
            selected = np.where((BITS == GROUP_BITS[group])[None, :], joint, -1.)
            selected_previous, selected_current = map(int, np.unravel_index(np.argmax(selected), selected.shape))
        rows.append((frame.copy(), state.copy(), temporal.copy(), selected_previous, selected_current))
        previous = float(t)
    return dict(emission_probability21=np.array([r[0] for r in rows]),
        state_hmm_probability21=np.array([r[1] for r in rows]), posterior21=np.array([r[2] for r in rows]),
        hmm_selected_previous_mode=np.array([r[3] for r in rows]),
        hmm_selected_current_mode=np.array([r[4] for r in rows]))


def run(prediction_root, output):
    output.mkdir(parents=True, exist_ok=False)
    inputs = []
    for case in CASES:
        for name in ('predictions.npz', 'model_evidence.npz'):
            path = prediction_root/case/name
            inputs.append(dict(case=case, name=name, path=str(path), sha256=sha(path)))
    (output/'registered_before_read.json').write_text(json.dumps(dict(
        utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)), inputs=inputs,
        probability_absolute_tolerance=1e-12, map_support_exact=True), indent=2)+'\n')
    report = {}
    for case in CASES:
        pred = load(prediction_root/case/'predictions.npz')
        costs = load(prediction_root/case/'model_evidence.npz')
        independent = expected(pred['time_s'], costs['node_cost'], costs['edge_cost'])
        result = {}
        for key, value in independent.items():
            if value.ndim == 2:
                difference = float(np.max(abs(value-pred[key])))
                support_ind = np.argmax(value[:, :20].reshape(-1, 10, 2).sum(axis=2), axis=1)
                def groups(a):
                    return np.column_stack([a[:, :20].reshape(-1, 10, 2).sum(axis=2), a[:, 20]])
                support_exact = bool(np.array_equal(np.argmax(groups(value), axis=1), np.argmax(groups(pred[key]), axis=1)))
                result[key] = dict(maximum_absolute_difference=difference, support_MAP_exact=support_exact,
                                   passed=difference <= 1e-12 and support_exact)
            else:
                exact = bool(np.array_equal(value, pred[key]))
                result[key] = dict(exact=exact, passed=exact,
                    mismatch_rows=np.flatnonzero(value != pred[key]).tolist())
        # Prefix invariance is explicit; this audit also never uses future rows.
        prefix = expected(pred['time_s'][:101], costs['node_cost'][:101], costs['edge_cost'][:101])
        prefix_exact = all(np.array_equal(prefix[key], independent[key][:101]) for key in prefix)
        result['prefix101_exact'] = dict(passed=prefix_exact)
        result['all_passed'] = all(row['passed'] for row in result.values() if isinstance(row, dict))
        report[case] = result
    for item in inputs:
        if sha(item['path']) != item['sha256']:
            raise ValueError('Sealed inference input changed during audit')
    (output/'report.json').write_text(json.dumps(dict(cases=report,
        all_passed=all(value['all_passed'] for value in report.values()), no_truth_read=True,
        no_force_refitting=True, no_new_physics=True), indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prediction-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.prediction_root.resolve(), args.output.resolve())
