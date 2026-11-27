"""Independent I-63 causal risk reconstruction and exact dominance audit.

Written before new condition-feature arithmetic. Reads frozen primitive arrays
and old I-62 compact packets; does not import the producer or query physics.
"""
import argparse
from datetime import datetime, timezone
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
EPS_MM = 1e-12


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() for k in z.files}


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [finite_json(v) for v in value]
    if isinstance(value, np.generic):
        return finite_json(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def reconstruct(p):
    t = p['time_s']; gamma = p['observer_generalized']
    interval = p['observer_interval_mean']; axes = p['patch_axis_world']
    hint = p['intrinsic_normal_hint']; R = float(p['radius_m']); N = len(t)
    assert gamma.shape == interval.shape == (N, 9)
    assert hint.shape == (N, 3) and axes.shape == (N, 4, 3)
    assert np.isfinite(t).all() and np.all(np.diff(t) > 0)
    force = np.linalg.norm(gamma[:, :3], axis=1)
    delta = interval-gamma
    obs = (np.isfinite(gamma).all(axis=1) & np.isfinite(interval).all(axis=1)
           & np.isfinite(delta).all(axis=1) & np.isfinite(force) & (force >= 1.))
    hint_size = np.linalg.norm(hint, axis=1)
    geometry = (np.isfinite(hint).all(axis=1) & np.isfinite(hint_size) & (hint_size > 1e-12)
                & np.isfinite(axes).all(axis=(1, 2))
                & np.all(abs(np.linalg.norm(axes, axis=2)-1.) <= 1e-10, axis=1))
    axial = np.full((N, 4), np.nan)
    for i in np.flatnonzero(geometry):
        normal = hint[i]/np.linalg.norm(hint[i])
        for j in range(4):
            axial[i, j] = np.dot(axes[i, j], -R*normal)
    boundary = np.concatenate((p['patch_upper_m'][:2], p['patch_lower_m'][2:]))
    arc = R*(np.arccos(np.clip(axial/R, -1., 1.))-np.arccos(boundary/R)[None, :])
    rate = np.full((N, 4), np.nan); derivative_valid = np.zeros(N, bool)
    for i in range(1, N):
        dt = float(t[i]-t[i-1])
        if geometry[i] and geometry[i-1] and abs(dt-.005) <= 1e-9:
            rate[i] = (arc[i]-arc[i-1])/dt
            derivative_valid[i] = np.isfinite(rate[i]).all()
    scale = .03+(.10*R)*force
    u = np.full(N, np.nan)
    u[obs] = np.clip(np.max(abs(delta[obs, 6:9]), axis=1)/scale[obs], 0., 1.)
    width = .012+.12*abs(rate)
    per_seam = np.clip(1.-abs(arc)/width, 0., 1.)
    s = np.full(N, np.nan)
    s[derivative_valid] = np.max(per_seam[derivative_valid], axis=1)
    valid = obs & derivative_valid
    r = np.full(N, np.nan); r[valid] = np.maximum(u[valid], s[valid])
    return dict(time_s=t, observer_signal_valid=obs, geometry_valid=geometry,
                axial_coordinate_m=axial, seam_arc_signed_offset_m=arc,
                seam_arc_signed_rate_m_s=rate, seam_derivative_valid=derivative_valid,
                joint_load_risk_u=u, seam_risk_s=s, combined_risk_r=r,
                risk_valid=valid, seam_risk_width_m=width, per_seam_risk=per_seam)


def frac(x):
    return Fraction.from_float(float(x))


def bounds(rows, ids, risk_name):
    selected = [rows[k] for k in ids]
    keys = ('time_s', 'primary_eligible', 'point_mask', 'i60_accepted', 'N_family',
            'A_family', 'truth_family', 'gate_valid', 'joint_max_abs_Nm',
            'radius_m', 'force_norm_N', 'native_distance_m', 'proposed_distance_m')
    x = {key: np.concatenate([c[key] for c in selected]) for key in keys}
    risk = np.concatenate([c['independent_'+risk_name] for c in selected])
    risk_valid = np.isfinite(risk)
    if risk_name == 'constant':
        risk_valid[:] = True
    phase = np.concatenate([np.full(len(c['time_s']), k, int) for k, c in zip(ids, selected)])
    local = np.concatenate([np.arange(len(c['time_s'])) for c in selected])
    primary = x['primary_eligible']; accepted = x['i60_accepted']; valid = x['gate_valid']
    useful = primary & (x['N_family'] != x['truth_family']) & (x['A_family'] == x['truth_family'])
    harmful = primary & (x['N_family'] == x['truth_family']) & (x['A_family'] != x['truth_family'])
    native_max = float(np.max(x['native_distance_m'][x['point_mask']])*1000.)
    unsafe = (x['point_mask'] & accepted & (x['proposed_distance_m']*1000. > native_max+EPS_MM))
    m = x['joint_max_abs_Nm']; L = x['radius_m']*x['force_norm_N']
    # Invalid load features fall back N only when the risk is known (or c=0).
    forced = useful & risk_valid & ~valid
    witnesses = []
    for k in np.flatnonzero(unsafe & risk_valid & valid):
        assert np.isfinite([m[k], L[k], risk[k]]).all() and m[k] >= 0
        mk = frac(m[k]); Lk = frac(x['radius_m'][k])*frac(x['force_norm_N'][k]); sk = frac(risk[k])
        for i in np.flatnonzero(useful & risk_valid & valid):
            mi = frac(m[i]); Li = frac(x['radius_m'][i])*frac(x['force_norm_N'][i]); si = frac(risk[i])
            if mi >= mk and mi*Lk >= mk*Li and mi*sk >= mk*si:
                forced[i] = True
                witnesses.append(dict(unsafe_case=CASES[int(phase[k])], unsafe_index=int(local[k]),
                    useful_case=CASES[int(phase[i])], useful_index=int(local[i]),
                    margins=[float(mi-mk), float(mi*Lk-mk*Li), float(mi*sk-mk*si)]))
    E = int(np.sum(primary & (x['A_family'] != x['truth_family'])))
    H = int(harmful.sum()); U = int(useful.sum()); loss = int(forced.sum())
    unknown_unsafe = unsafe & ~risk_valid
    return dict(cases=[CASES[i] for i in ids], native_maximum_mm=native_max,
        unsafe_rows=[dict(case=CASES[int(phase[i])], index=int(local[i]), time_s=float(x['time_s'][i]),
                          distance_A_mm=float(x['proposed_distance_m'][i]*1000.), risk_known=bool(risk_valid[i]))
                     for i in np.flatnonzero(unsafe)],
        A_errors=E, harmful_opportunities=H, useful_corrections=U,
        continuously_forced_useful_losses=loss, conservative_error_lower_bound=E+loss-H,
        unknown_unsafe_count=int(unknown_unsafe.sum()),
        point_max_impossible_due_to_unknown=bool(unknown_unsafe.any()),
        nonregression_impossible_by_loss=bool(loss > H), witnesses=witnesses,
        forced_useful_rows=[dict(case=CASES[int(phase[i])], index=int(local[i]))
                            for i in np.flatnonzero(forced)])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--features-root', type=Path, required=True)
    p.add_argument('--compact-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    rows = []; inputs = {}; comparisons = {}
    for case in CASES:
        primitive_path = args.features_root/case/'primitives.npz'
        compact_path = args.compact_root/(case+'.npz')
        p = load(primitive_path); compact = load(compact_path); features = reconstruct(p)
        np.testing.assert_array_equal(p['time_s'], compact['time_s'])
        np.savez_compressed(args.output/(case+'_independent_conditions.npz'), **features)
        compact.update({'independent_'+key: features[key] for key in
                        ('joint_load_risk_u', 'seam_risk_s', 'combined_risk_r')})
        compact['independent_constant'] = np.zeros(len(p['time_s']))
        rows.append(compact)
        inputs[case] = dict(primitives_sha256=sha(primitive_path), compact_sha256=sha(compact_path))
        produced = load(args.features_root/case/'conditions.npz')
        comparisons[case] = {}
        pairs = {key: key for key in ('observer_signal_valid', 'geometry_valid', 'axial_coordinate_m',
                 'joint_load_risk_u', 'seam_risk_s', 'combined_risk_r', 'risk_valid',
                 'seam_derivative_valid', 'seam_risk_width_m', 'per_seam_risk')}
        pairs.update(seam_arc_signed_offset_m='seam_signed_offset_m',
                     seam_arc_signed_rate_m_s='seam_signed_rate_m_s')
        for key, produced_key in pairs.items():
            a, b = features[key], produced[produced_key]
            if a.dtype == bool:
                np.testing.assert_array_equal(a, b); delta = 0.
            else:
                np.testing.assert_allclose(a, b, atol=1e-12, rtol=0, equal_nan=True)
                finite = np.isfinite(a) & np.isfinite(b)
                delta = float(np.max(abs(a[finite]-b[finite]))) if finite.any() else 0.
            comparisons[case][key] = dict(maximum_absolute_difference=delta,
                                          byte_exact=a.dtype == b.dtype and a.tobytes() == b.tobytes())
    groups = {'full': [0, 1, 2], **{'train_without_'+case: [j for j in range(3) if j != i]
                                  for i, case in enumerate(CASES)}}
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), numpy_version=np.__version__,
                  implementation='Independent loops, dot products and Fraction dominance, no producer import',
                  source_sha256=sha(Path(__file__)), inputs=inputs, feature_comparisons=comparisons,
                  bounds={name: {risk: bounds(rows, ids, risk) for risk in
                                 ('constant', 'joint_load_risk_u', 'seam_risk_s', 'combined_risk_r')}
                          for name, ids in groups.items()})
    (args.output/'report.json').write_text(json.dumps(finite_json(report), indent=2)+'\n')
    print(json.dumps({name: {risk: {key: value[key] for key in
                      ('continuously_forced_useful_losses', 'unknown_unsafe_count',
                       'conservative_error_lower_bound', 'nonregression_impossible_by_loss')}
                     for risk, value in group.items()} for name, group in report['bounds'].items()}, indent=2))


if __name__ == '__main__':
    main()
