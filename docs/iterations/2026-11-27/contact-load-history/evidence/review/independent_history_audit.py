"""Independent I-64 history/identity, coverage, model-metric and rank audit.

Frozen before new historical arithmetic. Uses scalar chronological recurrences
and lambda*prior+alpha*input, not the producer's increment-form implementation.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
MODELS = ('M0', 'M1', 'M2', 'M3')


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() for k in z.files}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def safe(x):
    if isinstance(x, dict): return {k: safe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)): return [safe(v) for v in x]
    if isinstance(x, np.generic): return safe(x.item())
    if isinstance(x, float) and not np.isfinite(x): return None
    return x


def reconstruct(p):
    t = p['time_s']; g = p['observer_generalized']; raw = p['observer_interval_mean']; n = len(t)
    R = float(p['radius_m']); out = {}; H = {}; good = {}
    assert g.shape == raw.shape == (n, 9) and np.all(np.diff(t) > 0)
    force_norm = np.linalg.norm(g[:, :3], axis=1)
    obs = np.isfinite(g).all(1) & np.isfinite(raw).all(1) & np.isfinite(force_norm) & (force_norm >= 1.)
    adjacent = np.zeros(n, bool); adjacent[1:] = abs(np.diff(t)-.005) <= 1e-9
    for c in ('F', 'Nalt', 'Neq'):
        H[c] = p['H_'+c]
        good[c] = p['H_'+c+'_valid'] & np.isfinite(H[c]).all(axis=(1, 2))
    out.update(time_s=t.copy(), radius_m=np.full(n, R), observation_valid=obs,
               time_adjacent=adjacent, filtered_force_norm_N=np.linalg.norm(g[:, :3], axis=1))
    recurrence = np.full((n, 9), np.nan)
    for i in range(1, n):
        if adjacent[i]:
            lam = np.exp(-100.*float(t[i]-t[i-1])); alpha = 1.-lam
            recurrence[i] = g[i]-(lam*g[i-1]+alpha*raw[i])
    out['observer_recurrence_residual9'] = recurrence
    replayed = np.full_like(g, np.nan); control_seeded = np.zeros(n, bool); control_valid = np.zeros(n, bool)
    active = False
    if n and np.isfinite(g[0]).all() and np.isfinite(raw[0]).all():
        replayed[0] = g[0]; control_seeded[0] = True; active = True
    for i in range(1, n):
        if not adjacent[i] or not np.isfinite(g[i]).all() or not np.isfinite(raw[i]).all():
            active = False; continue
        if not active:
            replayed[i] = g[i]; control_seeded[i] = True; active = True; continue
        lam = np.exp(-100.*float(t[i]-t[i-1])); replayed[i] = lam*replayed[i-1]+(1.-lam)*raw[i]
        control_valid[i] = True
    out['observer_reconstructed9'] = replayed; out['observer_full_recurrence_error9'] = g-replayed
    out['observer_reconstruction_seeded'] = control_seeded; out['observer_reconstruction_valid'] = control_valid
    for c in ('F', 'Nalt'):
        fp = np.array([H[c][i]@g[i, :3] for i in range(n)])
        rp = np.array([H[c][i]@raw[i, :3] for i in range(n)])
        out['M0_'+c+'_prediction_Nm'] = fp; out['M0_'+c+'_residual_Nm'] = g[:, 6:9]-fp
        out['M1_'+c+'_prediction_Nm'] = rp; out['M1_'+c+'_residual_Nm'] = raw[:, 6:9]-rp
        out['M0_'+c+'_valid'] = good[c] & obs; out['M1_'+c+'_valid'] = good[c] & obs
        one = np.full((n, 3), np.nan); scaled = one.copy()
        for i in range(1, n):
            if adjacent[i] and obs[i] and obs[i-1] and good[c][i]:
                lam = np.exp(-100.*float(t[i]-t[i-1])); alpha = 1.-lam
                one[i] = g[i, 6:9]-(lam*g[i-1, 6:9]+alpha*rp[i])
                scaled[i] = alpha*(raw[i, 6:9]-rp[i])
        out['one_step_'+c+'_residual_Nm'] = one
        out['one_step_'+c+'_scaled_raw_residual_Nm'] = scaled
        out['one_step_'+c+'_identity_error_Nm'] = one-scaled
    for model in ('M2', 'M3'):
        prior = np.full((n, 3), np.nan); after = prior.copy(); age = np.full(n, np.nan)
        flags = {k: np.zeros(n, bool) for k in ('seeded', 'reset', 'anchor_valid', 'chain_active')}
        pred = {c: prior.copy() for c in ('F', 'Nalt')}
        score_valid = {c: np.zeros(n, bool) for c in pred}
        state = None; elapsed = 0.
        for i in range(n):
            anchor = ('F' if p['i60_accepted'][i] else 'Neq') if model == 'M2' else 'Neq'
            if not obs[i] or not good[anchor][i] or (i > 0 and not adjacent[i]):
                flags['reset'][i] = state is not None
                state = None; elapsed = 0.; continue
            flags['chain_active'][i] = True
            if state is None:
                state = g[i, 6:9].copy(); after[i] = state; age[i] = 0.
                flags['seeded'][i] = True; continue
            prior[i] = state
            dt = float(t[i]-t[i-1]); elapsed += dt; age[i] = elapsed
            warm = elapsed >= .2-1e-9; flags['anchor_valid'][i] = warm
            lam = np.exp(-100.*dt); alpha = 1.-lam
            for c in pred:
                if good[c][i]:
                    pred[c][i] = lam*state+alpha*(H[c][i]@raw[i, :3])
                    score_valid[c][i] = warm
            state = lam*state+alpha*(H[anchor][i]@raw[i, :3]); after[i] = state
        for key, values in dict(anchor_prior_Nm=prior, anchor_after_Nm=after, history_age_s=age, **flags).items():
            out[model+'_'+key] = values
        out[model+'_seed_weight'] = np.exp(-100.*age)
        inherited = prior.copy(); alpha = np.full(n, np.nan)
        for i in range(1, n):
            lam = np.exp(-100.*float(t[i]-t[i-1])); alpha[i] = 1.-lam
            inherited[i] = lam*(g[i-1, 6:9]-prior[i])
        out[model+'_inherited_Nm'] = inherited
        for c in pred:
            out[model+'_'+c+'_prediction_Nm'] = pred[c]
            out[model+'_'+c+'_residual_Nm'] = g[:, 6:9]-pred[c]
            out[model+'_'+c+'_valid'] = score_valid[c]
            increment = alpha[:, None]*out['M1_'+c+'_residual_Nm']
            error = out[model+'_'+c+'_residual_Nm']-inherited-increment
            hn = np.linalg.norm(inherited, axis=1); cn = np.linalg.norm(increment, axis=1); total = hn+cn
            hs = np.full(n, np.nan); cs = hs.copy(); finite = np.isfinite(total) & (total > 0)
            hs[finite] = hn[finite]/total[finite]; cs[finite] = cn[finite]/total[finite]
            out[model+'_'+c+'_current_increment_Nm'] = increment
            out[model+'_'+c+'_decomposition_error_Nm'] = error
            out[model+'_'+c+'_inherited_norm_share'] = hs
            out[model+'_'+c+'_current_norm_share'] = cs
        out[model+'_committed_residual_Nm'] = g[:, 6:9]-after
    return out


def stats(values):
    if not len(values): return dict(count=0, vector_RMS_Nm=None, vector_p95_Nm=None)
    square = np.sum(values*values, axis=1); norm = np.sqrt(square)
    return dict(count=len(values), vector_RMS_Nm=float(np.sqrt(np.mean(square))),
                vector_p95_Nm=float(np.percentile(norm, 95)))


def evaluate(cases):
    result = {}; score_rows = {}; stats_rows = {}
    dn = np.concatenate([x['base']['native_distance_m'][x['base']['point_mask']] for x in cases.values()])
    pooled_native_max = float(np.max(dn)*1000.)
    for name, case in cases.items():
        base, f, p = case['base'], case['features'], case['primitives']; primary = base['primary_eligible']
        accepted = base['i60_accepted']; truth = base['truth_family']
        useful = primary & (base['N_family'] != truth) & (base['A_family'] == truth)
        harmful = primary & (base['N_family'] == truth) & (base['A_family'] != truth)
        wrong = primary & accepted & (base['A_family'] != truth)
        unsafe = base['point_mask'] & accepted & (base['proposed_distance_m']*1000. > pooled_native_max+1e-12)
        correct = primary & accepted & (base['A_family'] == truth)
        paired = primary & f['M0_F_valid'] & f['M2_F_valid']
        stats_mask = correct & paired
        stats_rows[name] = {m: f[m+'_F_residual_Nm'][stats_mask] for m in ('M0', 'M2')}
        coverage = dict(primary_total=int(primary.sum()), primary_paired=int(paired.sum()),
            accepted_total=int((primary & accepted).sum()), accepted_paired=int((paired & accepted).sum()),
            harmful_total=int(harmful.sum()), harmful_paired=int((harmful & paired).sum()),
            accepted_wrong_total=int(wrong.sum()), accepted_wrong_paired=int((wrong & paired).sum()),
            unsafe_total=int(unsafe.sum()), unsafe_paired=int((unsafe & paired).sum()))
        scale = .03+(.10*base['radius_m'])*base['force_norm_N']
        decisions = {}; rank = {}
        for m in MODELS:
            sf = np.max(abs(f[m+'_F_residual_Nm']), axis=1)/scale
            sn = np.max(abs(f[m+'_Nalt_residual_Nm']), axis=1)/scale
            valid = f[m+'_F_valid'] & f[m+'_Nalt_valid'] & np.isfinite(sf) & np.isfinite(sn)
            decision = np.full(len(primary), -2, int)
            difference = sf-sn
            decision[valid & (difference < -1e-9)] = -1
            decision[valid & (difference > 1e-9)] = 1
            decision[valid & (abs(difference) <= 1e-9)] = 0
            decisions[m] = decision
            rank[m] = dict(useful_correct=int(np.sum(useful & (decision == -1))),
                harmful_correct=int(np.sum(harmful & (decision == 1))),
                useful_total=int(useful.sum()), harmful_total=int(harmful.sum()),
                useful_unknown=int(np.sum(useful & (decision == -2))), harmful_unknown=int(np.sum(harmful & (decision == -2))),
                useful_tied=int(np.sum(useful & (decision == 0))), harmful_tied=int(np.sum(harmful & (decision == 0))),
                useful_near_tie=int(np.sum(useful & valid & (abs(difference) <= 1e-9+2e-8))),
                harmful_near_tie=int(np.sum(harmful & valid & (abs(difference) <= 1e-9+2e-8))))
            case['outputs'][m+'_rank_decision'] = decision
            case['outputs'][m+'_F_score'] = sf; case['outputs'][m+'_Nalt_score'] = sn
        score_rows[name] = rank
        result[name] = dict(coverage=coverage, paired_family_correct={m: stats(v) for m, v in stats_rows[name].items()}, ranks=rank)
    keys = next(iter(result.values()))['coverage']
    pooled_coverage = {key: sum(result[c]['coverage'][key] for c in result) for key in keys}
    pooled_stats = {m: stats(np.concatenate([stats_rows[c][m] for c in cases])) for m in ('M0', 'M2')}
    pooled_ranks = {m: {key: sum(score_rows[c][m][key] for c in cases) for key in score_rows[next(iter(cases))][m]} for m in MODELS}
    c = pooled_coverage; a = pooled_stats['M0']; b = pooled_stats['M2']; old = pooled_ranks['M0']; new = pooled_ranks['M2']
    flags = dict(primary_coverage=c['primary_paired']/c['primary_total'] >= .99,
        accepted_coverage=c['accepted_paired']/c['accepted_total'] >= .99,
        critical_events=all(c[k+'_paired'] == c[k+'_total'] for k in ('harmful', 'accepted_wrong', 'unsafe')),
        RMS_reduction=b['vector_RMS_Nm'] is not None and b['vector_RMS_Nm'] <= .9*a['vector_RMS_Nm']+1e-10,
        P95_nonincrease=b['vector_p95_Nm'] is not None and b['vector_p95_Nm'] <= a['vector_p95_Nm']+1e-10,
        phase_RMS_nonincrease=all(result[k]['paired_family_correct']['M2']['vector_RMS_Nm'] is not None and
            result[k]['paired_family_correct']['M2']['vector_RMS_Nm'] <= result[k]['paired_family_correct']['M0']['vector_RMS_Nm']+1e-10 for k in result))
    q2 = (new['harmful_correct'] >= 3 and new['useful_correct']/new['useful_total'] >= .9 and
          new['harmful_correct'] >= old['harmful_correct'] and new['useful_correct'] >= old['useful_correct'] and
          new['harmful_correct']+new['useful_correct'] > old['harmful_correct']+old['useful_correct'])
    result['all'] = dict(coverage=pooled_coverage, paired_family_correct=pooled_stats, ranks=pooled_ranks,
                         Q1_flags=flags, Q1_pass=all(flags.values()), Q2_pass=bool(q2))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--features-root', type=Path, required=True); p.add_argument('--base-root', type=Path, required=True)
    p.add_argument('--history-file', default='history_features.npz'); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    cases = {}; comparisons = {}; inputs = {}
    for c in CASES:
        pp = args.features_root/c/'primitives.npz'; hp = args.features_root/c/args.history_file
        bp = args.base_root/(c+'.npz'); primitive = load(pp); produced = load(hp); base = load(bp)
        derived = reconstruct(primitive); np.testing.assert_array_equal(base['time_s'], primitive['time_s'])
        case = dict(primitives=primitive, features=derived, base=base, outputs={})
        cases[c] = case; comparisons[c] = {}
        for key in derived.keys() & produced.keys():
            a, b = derived[key], produced[key]
            allowance = 1e-10 if key.endswith('_s') or key.startswith('observer_') else 1e-9
            if a.dtype == bool:
                np.testing.assert_array_equal(a, b); difference = 0.; allowance = 0.
            else:
                # Auxiliary norm shares have a possible 0/0 conditioning issue;
                # their underlying vectors are checked, never used as gates.
                if not key.endswith('_norm_share'):
                    np.testing.assert_allclose(a, b, rtol=0, atol=allowance, equal_nan=True)
                finite = np.isfinite(a) & np.isfinite(b)
                difference = float(np.max(abs(a[finite]-b[finite]))) if finite.any() else 0.
            comparisons[c][key] = dict(maximum_absolute_difference=difference, allowance=allowance,
                                       byte_exact=a.dtype == b.dtype and a.tobytes() == b.tobytes(),
                                       finite_mask_difference_count=int(np.sum(np.isfinite(a) != np.isfinite(b))),
                                       auxiliary_ratio_report_only=key.endswith('_norm_share'))
        assert all(m+'_'+branch+'_valid' in produced for m in MODELS for branch in ('F', 'Nalt'))
        for branch in ('F', 'Nalt'):
            mask = primitive['I61_'+branch+'_valid'] & derived['M0_'+branch+'_valid']
            np.testing.assert_allclose(derived['M0_'+branch+'_residual_Nm'][mask],
                                       primitive['I61_'+branch+'_residual_Nm'][mask], rtol=0, atol=1e-10)
        for key in ('observer_full_recurrence_error9', 'one_step_F_identity_error_Nm',
                    'one_step_Nalt_identity_error_Nm', 'M2_F_decomposition_error_Nm',
                    'M2_Nalt_decomposition_error_Nm', 'M3_F_decomposition_error_Nm',
                    'M3_Nalt_decomposition_error_Nm'):
            values = derived[key]; finite = np.isfinite(values)
            assert not finite.any() or np.max(abs(values[finite])) <= 1e-10, key
        inputs[c] = dict(primitives_sha256=sha(pp), producer_history_sha256=sha(hp), base_sha256=sha(bp))
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)),
        numpy_version=np.__version__, input_hashes=inputs, feature_comparisons=comparisons, evaluation=evaluate(cases))
    for c, case in cases.items():
        np.savez_compressed(args.output/(c+'_independent_history.npz'), **case['features'], **case['outputs'])
    (args.output/'report.json').write_text(json.dumps(safe(report), indent=2)+'\n')
    print(json.dumps(safe(report['evaluation']), indent=2))


if __name__ == '__main__': main()
