"""Independent I-62 bound audit; invoke only after the outcome release.

Reads frozen I-61 evaluation projections, not the tuning evaluator's summaries.
Checks a necessary point-maximum condition and its label-loss consequences.
This source is deliberately independent of the campaign evaluator.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
EPS_MM = 1e-12


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k,v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def load_case(path, feature_path, phase):
    with np.load(path, allow_pickle=False) as archive:
        x = {k: archive[k].copy() for k in archive.files}
    keys = ('time_s', 'i60_accepted', 'proposed__valid', 'proposed__residual',
        'proposed__force_norm_N', 'proposed__radius_m', 'proposed_point_m')
    with np.load(feature_path, allow_pickle=False) as archive:
        f = {k: archive[k].copy() for k in keys}
    if not np.array_equal(x['time_s'], f['time_s']):
        raise ValueError('Frozen feature and evaluation time rows differ')
    primary = x['primary_mask'] & x['truth_eligible']
    truth = x['truth_family']
    accepted = x['A_uses_proposal'].astype(bool)
    if not np.array_equal(accepted, x['i60_accepted']) or not np.array_equal(accepted, f['i60_accepted']):
        raise ValueError('A flags differ from frozen accepted proposals')
    residual = f['proposed__residual']
    force = f['proposed__force_norm_N']
    radius = np.broadcast_to(f['proposed__radius_m'], force.shape)
    valid = f['proposed__valid'] & np.isfinite(residual).all(axis=1)
    valid &= np.isfinite(force) & (force >= 0) & np.isfinite(radius) & (radius > 0)
    if not np.array_equal(valid, x['load_valid']):
        raise ValueError('Independently reconstructed validity differs')
    m = np.max(np.abs(residual[:,6:9]), axis=1)
    L = radius * force
    point_mask = primary.copy()
    for key in ('N_point_m', 'A_point_m', 'truth_equivalent_point_m'):
        point_mask &= np.isfinite(x[key]).all(axis=1)
    point_mask &= np.isfinite(f['proposed_point_m']).all(axis=1)
    dN = np.linalg.norm(x['N_point_m']-x['truth_equivalent_point_m'], axis=1)*1000
    dA = np.linalg.norm(x['A_point_m']-x['truth_equivalent_point_m'], axis=1)*1000
    idx = np.flatnonzero(primary)
    ne = x['N_family'] != truth
    ae = x['A_family'] != truth
    return dict(phase=np.full(len(idx), phase), time=x['time_s'][idx],
        accepted=accepted[idx], valid=valid[idx], m=m[idx], L=L[idx],
        force=force[idx], radius=radius[idx], ne=ne[idx], ae=ae[idx], useful=(ne & ~ae)[idx],
        harmful=(~ne & ae)[idx], point_mask=point_mask[idx],
        dN=dN[idx], dA=dA[idx])


def audit_subset(x, mask, phase_names):
    z = {key: value[mask] for key, value in x.items()}
    p = z['point_mask']
    native_max = float(z['dN'][p].max())
    unsafe = z['accepted'] & p & (z['dA'] > native_max+EPS_MM)
    active_unsafe = unsafe & z['valid']
    # Invalid proposals never pass, independent of a,b. These useful losses
    # are therefore forced even without an unsafe-peak constraint.
    forced = z['useful'] & ~z['valid']
    for k in np.flatnonzero(active_unsafe):
        # Exact nonnegative affine-threshold implication. Minimizing the
        # unsafe threshold over a+b*L_i >= m_i gives
        # m_i*min(1,L_k/L_i), with L_i=0 handled by the product form.
        forces_unsafe = (z['m'] >= z['m'][k]) & (z['m']*z['L'][k] >= z['m'][k]*z['L'])
        forced |= z['useful'] & z['valid'] & forces_unsafe
    unsafe_rows = []
    for k in np.flatnonzero(unsafe):
        unsafe_rows.append(dict(case=phase_names[int(z['phase'][k])],
            time_s=float(z['time'][k]), dA_mm=float(z['dA'][k]),
            dN_mm=float(z['dN'][k]), m_Nm=float(z['m'][k]),
            L_Nm=float(z['L'][k]), valid=bool(z['valid'][k])))
    rows = []
    for ai in range(61):
        a = ai*.005
        for bi in range(61):
            b = bi*.005
            # Match the registered operation order, rather than b*L.
            threshold = a + (b*z['radius'])*z['force']
            keep = z['accepted'] & z['valid'] & (z['m'] <= threshold)
            peak_safe = not bool(np.any(unsafe & keep))
            lost = int(np.sum(z['useful'] & ~keep))
            removed = int(np.sum(z['harmful'] & ~keep))
            errors = int(np.sum(np.where(keep, z['ae'], z['ne'])))
            expected = int(np.sum(z['ae'])) + lost - removed
            if errors != expected:
                raise AssertionError('Exact error identity failed')
            if peak_safe and int(np.sum(forced & ~keep)) != int(np.sum(forced)):
                raise AssertionError('Continuous dominance bound violated')
            rows.append((ai, bi, peak_safe, lost, removed, errors))
    feasible = [r for r in rows if r[2]]
    useful_total = int(np.sum(z['useful']))
    harmful_total = int(np.sum(z['harmful']))
    best_loss = min((r[3] for r in feasible), default=None)
    best_errors = min((r[5] for r in feasible), default=None)
    best_harm = max((r[4] for r in feasible), default=None)
    # Retain every nondominated (lost useful, remaining harmful) objective
    # value. More detailed point Pareto diagnostics belong to the full grid.
    pairs = sorted(set((r[3], harmful_total-r[4]) for r in feasible))
    frontier = []
    smallest_remaining = float('inf')
    for loss, remaining in pairs:
        if remaining < smallest_remaining:
            frontier.append(dict(useful_lost=loss, harmful_remaining=remaining))
            smallest_remaining = remaining
    return dict(primary_count=int(mask.sum()), point_count=int(p.sum()),
        useful_A=useful_total, harmful_A=harmful_total,
        family_errors_N=int(z['ne'].sum()), family_errors_A=int(z['ae'].sum()),
        native_max_mm=native_max, unsafe_A_peak_rows=unsafe_rows,
        continuously_forced_useful_loss_lower_bound=int(forced.sum()),
        peak_feasible_grid_count=len(feasible),
        grid_minimum_useful_loss_given_peak=best_loss,
        grid_minimum_errors_given_peak=best_errors,
        grid_maximum_harmful_removal_given_peak=best_harm,
        grid_point_peak_label_frontier=frontier,
        pooled_error_nonregression_impossible_on_grid_given_peak=(
            best_errors is None or best_errors > int(z['ae'].sum())))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--evaluation-dir', type=Path, required=True)
    parser.add_argument('--feature-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.evaluation_dir/(case+'_evaluation.npz') for case in CASES]
    feature_paths = [args.feature_dir/case/'load_features_and_policies.npz' for case in CASES]
    chunks = [load_case(path, feature_paths[phase], phase) for phase, path in enumerate(paths)]
    x = {key: np.concatenate([z[key] for z in chunks]) for key in chunks[0]}
    masks = {'full': np.ones(len(x['phase']), bool)}
    for phase, case in enumerate(CASES):
        masks['train_without_'+case] = x['phase'] != phase
    result = dict(status='independent necessary-constraint audit, not controller validation',
        scope='registered nonnegative shared two-parameter grid; frozen feature radii',
        numpy_version=np.__version__, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths+feature_paths},
        subsets={name: audit_subset(x, mask, CASES) for name, mask in masks.items()})
    args.output.write_text(json.dumps(json_safe(result), indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: {m: v[m] for m in ('peak_feasible_grid_count',
        'continuously_forced_useful_loss_lower_bound','grid_minimum_useful_loss_given_peak',
        'grid_minimum_errors_given_peak')} for k,v in result['subsets'].items()}, indent=2))


if __name__ == '__main__':
    main()
