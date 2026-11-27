"""Independent I-60 count/window/gate audit; no evaluator or ranker import."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import numpy as np

MODES = ('geometry', 'wrench', 'velocity', 'fused')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def rate(a, b):
    return a/b if b else None


def label_loads(loads, minimum, fraction):
    finite = np.isfinite(loads).all(axis=1)&(loads >= 0).all(axis=1)
    clean = np.where(np.isfinite(loads), loads, 0.)
    total = clean.sum(axis=1)
    maximum = clean.max(axis=1)
    unique = (clean == maximum[:, None]).sum(axis=1) == 1
    share = np.divide(maximum, total, out=np.zeros_like(total), where=total > 0)
    loaded = finite&(total >= minimum)&(maximum > 0)
    eligible = loaded&unique&(share >= fraction)
    labels = np.full(len(total), -1, int)
    for i in np.flatnonzero(eligible):
        labels[i] = next(j for j in range(4) if clean[i, j] == maximum[i])
    return labels, eligible, loaded, share, unique


def classify(labels, truth, selected):
    rows = np.flatnonzero(selected)
    errors = sum(int(labels[i] != truth[i]) for i in rows)
    unknown = sum(int(labels[i] < 0) for i in rows)
    n = len(rows)
    return dict(count=n, correct_count=n-errors, error_count=errors,
                unknown_count=unknown, error_fraction=rate(errors, n),
                sampled_disagreement_equivalent_s=.005*errors)


def switches(labels, truth, eligible, window):
    return sum(int(labels[i] != labels[i+1]) for i in range(len(labels)-1)
               if window[i] and window[i+1] and eligible[i] and eligible[i+1]
               and truth[i] == truth[i+1])


def window_summary(frames, native, truth, eligible, loaded, excitation, window):
    selected = window&eligible
    family = frames['candidate_family_valid']
    number = family.sum(axis=1)
    motion = excitation >= .02
    ambiguous = selected&(number >= 2)&motion
    covered = np.zeros(len(truth), bool)
    for i in np.flatnonzero(eligible):
        covered[i] = family[i, truth[i]]
    count = int(selected.sum())
    baseline = classify(native, truth, selected)
    baseline['false_switch_count'] = switches(native, truth, eligible, window)
    result = dict(counts=dict(window_rows=int(window.sum()), dominant_eligible=count,
        truth_loaded=int(np.sum(window&loaded)),
        truth_loaded_nondominant_or_tied=int(np.sum(window&loaded&~eligible)),
        truth_unloaded_or_invalid=int(np.sum(window&~loaded)),
        candidate_no_family=int(np.sum(selected&(number == 0))),
        candidate_single_family=int(np.sum(selected&(number == 1))),
        candidate_multi_family=int(np.sum(selected&(number >= 2))),
        motion_eligible=int(np.sum(selected&motion)), ambiguous_motion_eligible=int(ambiguous.sum()),
        physical_solver_point_count_above_one=int(np.sum(window&(frames['truth_patch_contact_counts'].sum(axis=1)>1)))),
        candidate_coverage=dict(covered_count=int(np.sum(selected&covered)), denominator=count,
                                fraction=rate(int(np.sum(selected&covered)), count)),
        native=baseline, modes={})
    for mode in MODES:
        labels = frames[mode+'__chosen_family']
        accepted = selected&(frames[mode+'__accepted_family'] >= 0)
        raw = classify(labels, truth, selected)
        raw['false_switch_count'] = switches(labels, truth, eligible, window)
        score = frames[mode+'__family_scores']
        minimum = score.min(axis=1)
        tied = np.isfinite(minimum)&((score == minimum[:, None]).sum(axis=1) > 1)
        n_accepted = int(accepted.sum())
        result['modes'][mode] = dict(raw=raw,
            accepted=classify(frames[mode+'__accepted_family'], truth, accepted),
            native_on_same_accepted_subset=classify(native, truth, accepted),
            accepted_count=n_accepted,
            accepted_ambiguous_motion_count=int(np.sum(accepted&ambiguous)),
            accepted_motion_count=int(np.sum(accepted&motion)),
            accepted_coverage_all=rate(n_accepted, count),
            accepted_coverage_motion=rate(int(np.sum(accepted&motion)), int(np.sum(selected&motion))),
            accepted_coverage_ambiguous_motion=rate(int(np.sum(accepted&ambiguous)), int(ambiguous.sum())),
            geometry_forced=classify(labels, truth, selected&(number == 1)),
            score_tie_count=int(np.sum(selected&tied)))
    return result


def aggregate(cases):
    primary = [case['primary'] for case in cases.values()]
    n = sum(x['counts']['dominant_eligible'] for x in primary)
    covered = sum(x['candidate_coverage']['covered_count'] for x in primary)
    errors = sum(x['native']['error_count'] for x in primary)
    ambiguous = sum(x['counts']['ambiguous_motion_eligible'] for x in primary)
    motion = sum(x['counts']['motion_eligible'] for x in primary)
    result = dict(dominant_eligible_count=n, candidate_true_family_coverage=rate(covered, n),
        candidate_true_family_covered_count=covered, native_error_count=errors,
        native_error_fraction=rate(errors, n), ambiguous_motion_eligible_count=ambiguous,
        motion_eligible_count=motion, modes={})
    for mode in MODES:
        rows = [x['modes'][mode] for x in primary]
        raw = sum(x['raw']['error_count'] for x in rows)
        accepted = sum(x['accepted_count'] for x in rows)
        accepted_error = sum(x['accepted']['error_count'] for x in rows)
        native_error = sum(x['native_on_same_accepted_subset']['error_count'] for x in rows)
        accepted_ambiguous = sum(x['accepted_ambiguous_motion_count'] for x in rows)
        accepted_motion = sum(x['accepted_motion_count'] for x in rows)
        result['modes'][mode] = dict(error_count=raw, error_fraction=rate(raw, n),
            accepted_count=accepted, accepted_error_count=accepted_error,
            accepted_error_fraction=rate(accepted_error, accepted),
            native_same_accepted_error_count=native_error,
            native_same_accepted_error_fraction=rate(native_error, accepted),
            confidence_coverage_ambiguous_motion=rate(accepted_ambiguous, ambiguous),
            confidence_coverage_all=rate(accepted, n), confidence_coverage_motion=rate(accepted_motion, motion))
    return result


def gates(a, cases, protocol):
    p = protocol['gates']
    result = []
    def compare(name, actual, threshold, greater=False):
        valid = actual is not None and threshold is not None
        passed = valid and (actual >= threshold if greater else actual <= threshold)
        result.append(dict(name=name, actual=actual, threshold=threshold,
                           scorable=actual is not None, passed=bool(passed)))
    compare('candidate_true_family_coverage', a['candidate_true_family_coverage'], p['candidate_true_family_coverage_min'], True)
    native, fused, wrench = a['native_error_fraction'], a['modes']['fused'], a['modes']['wrench']
    effect = None if native is None or fused['error_fraction'] is None else native-fused['error_fraction']
    requirement = None if native is None else max(p['fused_native_error_reduction_absolute_min'], p['fused_native_error_reduction_relative_min']*native)
    compare('fused_improves_native', effect, requirement, True)
    effect = None if wrench['error_fraction'] is None or fused['error_fraction'] is None else wrench['error_fraction']-fused['error_fraction']
    compare('velocity_adds_over_wrench', effect, p['fused_wrench_error_reduction_absolute_min'], True)
    for case, windows in cases.items():
        primary = windows['primary']
        f, n = primary['modes']['fused']['raw'], primary['native']
        difference = None if f['error_fraction'] is None or n['error_fraction'] is None else f['error_fraction']-n['error_fraction']
        compare(case+'_raw_error_nonregression', difference, p['per_phase_fused_error_above_native_allowance'])
        compare(case+'_false_switch_nonregression', f['false_switch_count']-n['false_switch_count'], p['per_phase_false_switch_above_native_allowance_count'])
    compare('confidence_coverage_ambiguous_motion', fused['confidence_coverage_ambiguous_motion'], p['confidence_ambiguous_motion_eligible_coverage_min'], True)
    compare('confidence_error', fused['accepted_error_fraction'], p['confidence_error_max'])
    own, native = fused['accepted_error_fraction'], fused['native_same_accepted_error_fraction']
    difference = None if own is None or native is None else own-native
    compare('confidence_error_vs_native_same_subset', difference, p['confidence_error_above_native_same_accepted_subset_allowance'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation-root', type=Path, required=True)
    parser.add_argument('--features-root', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    protocol = json.loads(args.protocol.read_text())
    canonical = json.loads((args.evaluation_root/'science.json').read_text())
    cases, controls = {}, {}
    for case in protocol['datasets']:
        folder = args.features_root/case
        context = load(folder/'measurement_context.npz')
        frames = load(args.evaluation_root/(case+'_evaluation.npz'))
        meta = json.loads((folder/'metadata.json').read_text())
        t = context['time_s']
        assert np.array_equal(t, frames['time_s'])
        truth, eligible, loaded, share, unique = label_loads(frames['truth_patch_loads_N'], protocol['evaluation']['truth_minimum_total_load_N'], protocol['evaluation']['truth_minimum_dominant_fraction'])
        native, _, _, _, native_unique = label_loads(context['exact_estimated_support_loads_N'], np.finfo(float).tiny, 0.)
        radius = meta['radius_m']
        qv, previous = context['exact_qvel'], context['exact_previous_applied']
        excitation = np.linalg.norm(qv[:, :3], axis=1)+radius*np.linalg.norm(qv[:, 3:6], axis=1)+radius*np.linalg.norm(qv[:, 6:9], axis=1)+np.linalg.norm(previous[:, 3:], axis=1)
        for left, right in ((truth,frames['truth_family']), (eligible,frames['truth_eligible']),
                            (loaded,frames['truth_loaded']), (unique,frames['truth_unique_max']),
                            (native,frames['native_family']), (native_unique,frames['native_unique_max'])):
            np.testing.assert_array_equal(left, right)
        np.testing.assert_array_equal(share,frames['truth_dominant_fraction'])
        np.testing.assert_array_equal(excitation,frames['measured_excitation_m_s'])
        stable = np.zeros(len(t), bool)
        for i in range(2,len(t)-2):
            stable[i] = all(eligible[j] and truth[j] == truth[i] for j in range(i-2,i+3))
        np.testing.assert_array_equal(stable,frames['stable_truth_mask'])
        definitions = {'primary':protocol['evaluation']['primary_window_s'], **protocol['evaluation']['secondary_windows_s']}
        masks = {name:(t >= limits[0]-1e-9)&(t < limits[1]-1e-9) for name,limits in definitions.items()}
        masks['stable_primary'] = masks['primary']&stable
        assert masks['primary'].sum() == 8000 and not masks['full'][-1]
        windows = {name:window_summary(frames,native,truth,eligible,loaded,excitation,mask) for name,mask in masks.items()}
        assert windows == canonical['cases'][case]['windows'], case
        cases[case] = windows
        controls[case] = dict(rows=len(t), windows_exact=list(windows),
                              baseline_truth_eligibility_excitation_and_stable_masks_exact=True,
                              evaluation_sha256=sha(args.evaluation_root/(case+'_evaluation.npz')))
    pooled = aggregate(cases)
    assert pooled == canonical['aggregate_primary']
    evaluated = gates(pooled,cases,protocol)
    assert evaluated == canonical['gates']
    failed = [x['name'] for x in evaluated if not x['passed']]
    assert failed == canonical['failed_gates']
    assert all(x['passed'] for x in evaluated) == canonical['all_gates_passed']
    report = dict(iteration='I-60', completed_utc=datetime.now(timezone.utc).isoformat(),
        scope='Independent scalar counts, derived labels, all fixed windows/pooled metrics and gate values; no evaluator/ranker import.',
        new_physics_runs=0, cases=controls, aggregate_primary=pooled, gates=evaluated,
        failed_gates=failed, all_comparisons_exact=True,
        descriptive_timing_source_only_reviewed=True,
        canonical_science_sha256=sha(args.evaluation_root/'science.json'))
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(all_comparisons_exact=True, failed_gates=failed, aggregate_primary=pooled)))


if __name__ == '__main__':
    main()
