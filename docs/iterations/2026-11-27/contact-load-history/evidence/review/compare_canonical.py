"""Thin post-result I-64 mapping plus fixed, independent shadow aggregation.

No producer import, model adjustment or threshold selection. The underlying
history reconstruction remains the immutable pre-execution independent source.
"""
import argparse
from datetime import datetime, timezone
import hashlib, json
from pathlib import Path
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B'); MODELS = ('M0', 'M1', 'M2', 'M3')


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() for k in z.files}


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def count_metrics(rows, keeps):
    errors = harmful = useful = retained = vetoes = 0; points = []
    for b, keep in zip(rows, keeps):
        p = b['primary_eligible']; truth = b['truth_family']; accepted = b['i60_accepted']
        chosen = np.where(keep, b['A_family'], b['N_family'])
        errors += int(np.sum(p & (chosen != truth)))
        harmful += int(np.sum(p & (b['N_family'] == truth) & (chosen != truth)))
        useful += int(np.sum(p & (b['N_family'] != truth) & (b['A_family'] == truth) & (chosen == truth)))
        retained += int(np.sum(p & keep)); vetoes += int(np.sum(p & accepted & ~keep))
        mask = b['point_mask']
        points.append(np.where(keep[mask], b['proposed_distance_m'][mask], b['native_distance_m'][mask])*1000.)
    d = np.concatenate(points)
    return dict(error_count=errors, harmful_count=harmful, useful_retained_count=useful,
        retained_proposal_count=retained, veto_count=vetoes, point_SSE_mm2=float(np.sum(d*d)),
        point_RMS_mm=float(np.sqrt(np.mean(d*d))), point_p95_mm=float(np.percentile(d, 95)),
        point_maximum_mm=float(np.max(d)))


def gates(result, N, A, phases, baseline_phases, phase_counts):
    out = dict(harmful_removal=(A['harmful_count']-result['harmful_count'])/A['harmful_count'] >= .5,
        useful_retention=result['useful_retained_count']/A['useful_retained_count'] >= .9,
        pooled_errors=result['error_count'] <= A['error_count'],
        point_RMS=result['point_RMS_mm'] <= A['point_RMS_mm']+1e-12,
        point_p95=result['point_p95_mm'] <= A['point_p95_mm']+1e-12,
        point_maximum=result['point_maximum_mm'] <= N['point_maximum_mm']+1e-12,
        nonempty_retention=result['retained_proposal_count'] >= 1)
    for c in CASES:
        out[c+'_error_rate'] = phases[c]['error_count']/phase_counts[c] <= baseline_phases[c]['error_count']/phase_counts[c]+.001
        out[c+'_harmful_nonincrease'] = phases[c]['harmful_count'] <= baseline_phases[c]['harmful_count']
    return out


def same_metrics(actual, expected):
    for key, value in actual.items():
        if isinstance(value, int): assert value == expected[key], (key, value, expected[key])
        else: np.testing.assert_allclose(value, expected[key], rtol=0, atol=1e-9, err_msg=key)


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, required=True)
    args = p.parse_args(); r = args.root
    own = json.loads((r/'review/results_stage01/report.json').read_text())
    science = json.loads((r/'analysis/stage01/science.json').read_text())
    bases = {}; derived = {}; maps = {}; frame_checks = {}; ranks = {}; identity = {}
    for case in CASES:
        b = load(r/'portable_inputs/i62_base'/(case+'.npz'))
        f = load(r/'review/results_stage01'/(case+'_independent_history.npz'))
        canon = load(r/'analysis/stage01'/(case+'_frames.npz'))
        primitive = load(r/'features/stage01'/case/'primitives.npz')
        bases[case] = b; derived[case] = f; maps[case] = primitive
        primary = b['primary_eligible']; np.testing.assert_array_equal(canon['time_s'], b['time_s'][primary])
        maximum_score_difference = 0.; ranks[case] = {}
        for model in MODELS:
            for branch in ('F', 'Nalt'):
                stem = model+'_'+branch
                np.testing.assert_array_equal(f[stem+'_valid'][primary], canon[stem+'_valid'])
                valid = canon[stem+'_score_valid']
                a = f[stem+'_score'][primary]; c = canon[stem+'_score']
                np.testing.assert_allclose(a[valid], c[valid], rtol=0, atol=1e-8)
                if valid.any(): maximum_score_difference = max(maximum_score_difference, float(np.max(abs(a[valid]-c[valid]))))
            sf, sn = canon[model+'_F_score'], canon[model+'_Nalt_score']
            valid = canon[model+'_F_score_valid'] & canon[model+'_Nalt_score_valid']
            decision = np.full(len(sf), -2, int); difference = sf-sn
            decision[valid & (difference < -1e-9)] = -1; decision[valid & (difference > 1e-9)] = 1
            decision[valid & (abs(difference) <= 1e-9)] = 0
            np.testing.assert_array_equal(decision, f[model+'_rank_decision'][primary])
            ranks[case][model] = True
        for key in ('M2_anchor_valid', 'M3_anchor_valid', 'M2_seeded', 'M3_seeded', 'M2_reset', 'M3_reset'):
            np.testing.assert_array_equal(f[key][primary], canon[key])
        frame_checks[case] = dict(score_maximum_difference=maximum_score_difference,
                                  all_current_branch_rank_decisions_exact=True, valid_warm_reset_masks_exact=True)
        identity[case] = {key: float(np.nanmax(abs(f[key]))) for key in
            ('observer_full_recurrence_error9', 'observer_recurrence_residual9',
             'one_step_F_identity_error_Nm', 'one_step_Nalt_identity_error_Nm',
             'M2_F_decomposition_error_Nm', 'M2_Nalt_decomposition_error_Nm',
             'M3_F_decomposition_error_Nm', 'M3_Nalt_decomposition_error_Nm')}
    for name, row in own['evaluation'].items():
        comp = science['comparisons']['all' if name == 'all' else name]
        for model, cnt in row['ranks'].items():
            for group, prefix in (('useful', 'useful'), ('harmful', 'harmful')):
                observed = comp[group]['ranking_all'][model]
                assert cnt[prefix+'_total'] == observed['denominator']
                assert cnt[prefix+'_unknown'] == observed['unknown']
                assert cnt[prefix+'_tied'] == observed['tied']
                assert cnt[prefix+'_near_tie'] == observed['near_tie_within_2e_minus8_verification_envelope']
                assert cnt[prefix+'_correct'] == observed['F_preferred' if group == 'useful' else 'Nalt_preferred']
        for model, st in row['paired_family_correct'].items():
            observed = science['Q1_model_consistency'][model] if name == 'all' else science['Q1_model_consistency']['per_case'][name][model]
            assert st['count'] == observed['count']
            for key in ('vector_RMS_Nm', 'vector_p95_Nm'): np.testing.assert_allclose(st[key], observed[key], rtol=0, atol=1e-9)
    q = own['evaluation']['all']; canonical_q1 = science['Q1_model_consistency']
    assert q['Q1_pass'] == canonical_q1['qualified'] and q['Q2_pass'] == science['Q2_discrimination']['promising']
    coverage = q['coverage']
    assert coverage['primary_paired']/coverage['primary_total'] == canonical_q1['coverage_primary']
    assert coverage['accepted_paired']/coverage['accepted_total'] == canonical_q1['coverage_accepted']
    assert canonical_q1['critical_unknown'] == 0 and coverage['harmful_paired'] == 6 and coverage['accepted_wrong_paired'] == 18 and coverage['unsafe_paired'] == 1
    keep = {model: {} for model in MODELS}; phases = {model: {} for model in MODELS}
    for case in CASES:
        b, f, p = bases[case], derived[case], maps[case]
        for model in MODELS:
            known = f[model+'_F_valid'] & np.isfinite(f[model+'_F_score'])
            retained = b['i60_accepted'] & p['H_F_valid']
            if model in ('M0', 'M1'): retained &= known & (f[model+'_F_score'] <= 1.)
            else: retained &= ~known | (f[model+'_F_score'] <= 1.)
            keep[model][case] = retained
            phases[model][case] = count_metrics([b], [retained])
            same_metrics(phases[model][case], science['shadow_policies'][model]['per_case'][case]['metrics'])
        np.testing.assert_array_equal(keep['M0'][case], p['I61_C_uses_proposal'])
    all_rows = [bases[c] for c in CASES]
    N = count_metrics(all_rows, [np.zeros(len(bases[c]['time_s']), bool) for c in CASES])
    A = count_metrics(all_rows, [bases[c]['i60_accepted'] for c in CASES])
    baseline_phases = {c: count_metrics([bases[c]], [bases[c]['i60_accepted']]) for c in CASES}
    phase_counts = {c: int(bases[c]['primary_eligible'].sum()) for c in CASES}
    shadow = {}
    for model in MODELS:
        metrics = count_metrics(all_rows, [keep[model][c] for c in CASES])
        same_metrics(metrics, science['shadow_policies'][model]['metrics'])
        flags = gates(metrics, N, A, phases[model], baseline_phases, phase_counts)
        assert flags == science['shadow_policies'][model]['gates']
        assert all(flags.values()) == science['shadow_policies'][model]['all_gates_pass']
        shadow[model] = dict(metrics=metrics, all_gate_flags_exact=True, all_gates_pass=all(flags.values()))
    assert science['new_physics_steps'] == 0 and science['live_changes'] is False and science['threshold_tuning'] is False
    result = dict(created_utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)),
        scope='Post-result field mapper and fixed shadow verification; underlying independent77407 source unchanged',
        canonical_science_sha256=sha(r/'analysis/stage01/science.json'), independent_report_sha256=sha(r/'review/results_stage01/report.json'),
        frames=frame_checks, identity_maxima=identity, all_rank_counts_and_decisions_exact=True,
        all_Q1_coverage_correct_subset_counts_exact=True, critical_coverage_6H_18wrong_1unsafe_exact=True,
        Q1_pass=q['Q1_pass'], Q2_pass=q['Q2_pass'], shadow_policies=shadow, no_physics_or_live_change=True)
    (r/'review/canonical_comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__': main()
