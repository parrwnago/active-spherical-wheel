"""Post-hoc I-68 audit registered after inspection of the fixed event rows.

Purely numerical zero-normal/nonzero-tangent detection, not a friction verdict.
Primary interval analysis is imported unchanged and never rerun or overwritten.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from interval_core import TAG_FAMILY, point_force_generalized, reduce_interval, vector_stats
from analyse_interval import clean

EVENTS = (7.875, 32.110, 35.055, 35.205)
BODY_NAMES = ('shell_A', 'shell_B', 'carrier', 'fork')
TAG_BODY = np.array([0, 1, 2, 2, 0, 1, 3])


def detect(rows):
    rows = np.asarray(rows, float)
    force, normal = rows[:, 7:10], rows[:, 4:7]
    fn = np.sum(force*normal, axis=1)
    ft = np.linalg.norm(force-fn[:, None]*normal, axis=1)
    norm = np.linalg.norm(force, axis=1)
    selected = (np.isfinite(rows).all(axis=1)
                & (np.abs(fn) <= 1e-10*np.maximum(1., norm)) & (ft > 1e-8))
    return selected, fn, ft, norm


def run(trace):
    rows, offsets = trace['contact_rows'], trace['contact_offsets']
    chosen, fn, ft, norm = detect(rows)
    n = len(trace['time_s'])
    step_index = np.repeat(np.arange(n), np.diff(offsets))
    row_end = trace['control_end_time_s'][step_index]
    row_dt = trace['step_dt_s'][step_index]
    tags = rows[:, 0].astype(int)
    admitted = rows[:, 12] >= .5
    ids = np.unique(trace['control_end_index']).astype(int)
    times = np.array([trace['control_end_time_s'][np.flatnonzero(trace['control_end_index'] == end)[0]] for end in ids])
    interval_chosen_count = np.zeros(len(ids), int)
    interval = {p: np.zeros((len(ids), 4, 9)) for p in ('before', 'after')}
    step_loads = {p: np.zeros((n, 4, 9)) for p in ('before', 'after')}
    for i in range(n):
        lo, hi = offsets[i:i+2]
        selected = chosen[lo:hi]
        if not selected.any():
            continue
        r = rows[lo:hi][selected]
        for p in ('before', 'after'):
            load, valid = point_force_generalized(trace[f'qpos_{p}'][i], r[:, 1:4], r[:, 7:10], r[:, 0])
            if not valid.all():
                raise ValueError('invalid selected force map')
            for body in range(4):
                step_loads[p][i, body] = load[TAG_BODY[r[:, 0].astype(int)] == body].sum(axis=0)
    for j, end in enumerate(ids):
        ix = np.flatnonzero(trace['control_end_index'] == end)
        q0, q1 = trace['qpos_before'][ix[0]], trace['qpos_after'][ix[-1]]
        interval_chosen_count[j] = sum(int(chosen[offsets[i]:offsets[i+1]].sum()) for i in ix)
        for p in ('before', 'after'):
            interval[p][j] = reduce_interval(step_loads[p][ix], trace['step_dt_s'][ix], q0, q1)
    groups = {'all_capture': np.ones(len(ids), bool),
              'primary_1_to_41': (times >= 1-1e-8) & (times < 41-1e-8),
              'broad_32_to_36': (times >= 32-1e-8) & (times < 36-1e-8)}
    for event in EVENTS:
        groups[f'interval_ending_{event:.3f}'] = abs(times-event) <= 1e-8
    report = dict(status='POSTHOC descriptive row audit; event anomaly seen before registration',
                  rule='abs(F dot n)<=1e-10*max(1,||F||) AND ||F-(F dot n)n||>1e-8N',
                  grouping='Use control_end_time_s for all five component steps; native relative clock.',
                  force_units='N', impulse_units='N s; joint impulse N m s',
                  caveat='Numerical pattern screen only; not a friction-law, convergence or reporting-failure verdict.',
                  published_normal_max_norm_error=float(np.max(abs(np.linalg.norm(rows[:, 4:7], axis=1)-1))),
                  groups={})
    for name, m in groups.items():
        selected_ids = ids[m]
        rm = np.isin(trace['control_end_index'][step_index], selected_ids)
        sx = np.isin(trace['control_end_index'], selected_ids)
        eligible = rm & chosen
        by_body = {}
        for body, label in enumerate(BODY_NAMES):
            br = eligible & (TAG_BODY[tags] == body)
            impulse = np.sum(rows[br, 7:10]*row_dt[br, None], axis=0)
            duration = float(np.sum(trace['step_dt_s'][sx]))
            by_body[label] = dict(rows=int(br.sum()), admitted_rows=int((br & admitted).sum()),
                                  rejected_rows=int((br & ~admitted).sum()),
                                  force_impulse_world_Ns=impulse,
                                  mean_force_over_group_world_N=impulse/duration if duration else np.full(3, np.nan),
                                  before_joint_load_Nm=vector_stats(interval['before'][:, body, 6:], m),
                                  after_joint_load_Nm=vector_stats(interval['after'][:, body, 6:], m),
                                  joint_impulse_Nms=np.sum(step_loads['before'][sx, body, 6:]*trace['step_dt_s'][sx, None], axis=0))
        report['groups'][name] = dict(total_intervals=int(m.sum()), total_rows=int(rm.sum()),
                                     zero_normal_force_bearing_rows=int(eligible.sum()),
                                     affected_intervals=int((m & (interval_chosen_count > 0)).sum()),
                                     admitted_rows=int((eligible & admitted).sum()),
                                     rejected_rows=int((eligible & ~admitted).sum()),
                                     normal_projection_max_abs_N=float(np.max(abs(fn[eligible]), initial=0)),
                                     tangent_force_max_N=float(np.max(ft[eligible], initial=0)),
                                     by_body=by_body)
    arrays = dict(time_s=times, control_end_index=ids, selected_row=chosen,
                  row_normal_projection_N=fn, row_tangent_norm_N=ft,
                  interval_selected_row_count=interval_chosen_count,
                  interval_before_generalized_by_body=interval['before'],
                  interval_after_generalized_by_body=interval['after'])
    return arrays, clean(report)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--trace', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    with np.load(args.trace, allow_pickle=False) as packet:
        trace = {key: packet[key] for key in packet.files}
    arrays, report = run(trace)
    args.output.mkdir(parents=True)
    report['trace_sha256'] = hashlib.sha256(args.trace.read_bytes()).hexdigest()
    np.savez_compressed(args.output/'frames.npz', **arrays)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n')
    manifest = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in args.output.iterdir()}
    (args.output/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
    print(json.dumps({key: {k: v for k, v in val.items() if k != 'by_body'} for key, val in report['groups'].items()}))


if __name__ == '__main__':
    main()
