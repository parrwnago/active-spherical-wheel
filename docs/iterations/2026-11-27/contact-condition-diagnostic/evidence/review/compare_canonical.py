"""Post-result thin comparison; does not alter the presealed risk/bound audit.

Checks every primary bound/witness identity and independently counts fixed
diagnostic groups/clusters from frozen packets. No score or threshold fitting.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse, hashlib, json
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() for k in z.files}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stamp(case, time):
    return case, int(round(float(time)*1e9))


def clusters(case, t, mask):
    result = []; current = []
    for k in np.flatnonzero(mask):
        if current and abs(t[k]-t[current[-1]]-.005) > 1e-9:
            result.append(current); current = []
        current.append(int(k))
    if current:
        result.append(current)
    return [dict(case=case, start_s=float(t[x[0]]), end_s=float(t[x[-1]]), frame_count=len(x))
            for x in result]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    args = p.parse_args(); root = args.root
    own = json.loads((root/'review/results_stage01/report.json').read_text())
    canonical = json.loads((root/'analysis/stage01/science.json').read_text())
    packets = {case: load(root/'portable_inputs/i62_base'/(case+'.npz')) for case in CASES}
    produced = {case: load(root/'features/stage01'/case/'conditions.npz') for case in CASES}
    bound_checks = {}
    for group, expected in own['bounds'].items():
        expected = expected['combined_risk_r']; observed = canonical['dynamic_dominance'][group]
        pairs = dict(native_maximum_mm='native_maximum_mm',
                     continuously_forced_useful_losses='forced_useful_loss_lower_bound',
                     harmful_opportunities='harmful_count', useful_corrections='useful_count',
                     unknown_unsafe_count='unsafe_unknown_risk_count')
        for a, b in pairs.items():
            assert expected[a] == observed[b], (group, a, expected[a], observed[b])
        assert len(expected['unsafe_rows']) == observed['unsafe_count']
        actual_set = {stamp(x['case'], x['time_s']) for x in observed['witnesses']}
        expected_set = {stamp(x['case'], packets[x['case']]['time_s'][x['index']])
                        for x in expected['forced_useful_rows']}
        assert actual_set == expected_set
        bound_checks[group] = dict(counts_and_maximum_exact=True, witness_identity_set_exact=True,
            forced_count=expected['continuously_forced_useful_losses'],
            risk_envelope_robust_forced_count=expected['numerically_robust_forced_useful_losses'])
    static_set = {stamp(x['case'], packets[x['case']]['time_s'][x['index']])
                  for x in own['bounds']['full']['constant']['forced_useful_rows']}
    old = json.loads((root/'portable_inputs/i62_old_witnesses.json').read_text())
    old_set = {stamp(x['case'], x['time_s']) for x in old['witnesses']}
    assert old_set == static_set and len(static_set) == 99
    rows = {}; all_masks = {}; feature_stats_exact = True
    pooled_max = max(float(np.max(x['native_distance_m'][x['point_mask']])) for x in packets.values())*1000.
    for case in CASES:
        d = packets[case]; f = produced[case]; primary = d['primary_eligible']
        Nwrong = d['N_family'] != d['truth_family']; Awrong = d['A_family'] != d['truth_family']
        old_tag = np.array([stamp(case, t) in static_set for t in d['time_s']])
        masks = dict(all_primary=primary, accepted=primary & d['i60_accepted'],
            harmful=primary & ~Nwrong & Awrong, useful=primary & Nwrong & ~Awrong,
            accepted_wrong=primary & d['i60_accepted'] & Awrong,
            accepted_correct=primary & d['i60_accepted'] & ~Awrong,
            static_forced_useful=primary & old_tag,
            unsafe_point=d['point_mask'] & d['i60_accepted'] & (d['proposed_distance_m']*1000. > pooled_max+1e-12),
            label_neutral_point_change=primary & d['i60_accepted'] & (d['N_family'] == d['A_family'])
                & (d['native_distance_m'] != d['proposed_distance_m']))
        # Canonical phase diagnostic unsafe uses its phase-native maximum;
        # verify the pooled unsafe set independently for the all-phase report.
        local_max = float(np.max(d['native_distance_m'][d['point_mask']]))*1000.
        local_masks = dict(masks)
        local_masks['unsafe_point'] = d['point_mask'] & d['i60_accepted'] & (d['proposed_distance_m']*1000. > local_max+1e-12)
        rows[case] = {}
        for group, mask in local_masks.items():
            observed = canonical['diagnostics'][case]['groups'][group]
            rows[case][group] = int(mask.sum())
            for field in ('joint_load_risk_u', 'seam_risk_s', 'combined_risk_r'):
                values = f[field][mask]; values = values[np.isfinite(values)]
                reported = observed[field]
                assert reported['count'] == int(mask.sum())
                assert reported['finite_count'] == len(values)
                assert reported['unknown_count'] == int(mask.sum())-len(values)
                if len(values):
                    assert reported['minimum'] == float(values.min())
                    assert reported['maximum'] == float(values.max())
                    assert reported['mean'] == float(values.mean())
                    assert reported['percentiles'] == {str(k): float(np.percentile(values, k)) for k in (10, 25, 50, 75, 90, 95)}
            if group in canonical['diagnostics'][case]['event_clusters']:
                assert canonical['diagnostics'][case]['event_clusters'][group] == clusters(case, d['time_s'], mask)
        for group, mask in masks.items():
            all_masks.setdefault(group, []).append(mask)
    rows['all'] = {}
    for group, pieces in all_masks.items():
        mask = np.concatenate(pieces); rows['all'][group] = int(mask.sum())
        observed = canonical['diagnostics']['all']['groups'][group]
        for field in ('joint_load_risk_u', 'seam_risk_s', 'combined_risk_r'):
            values = np.concatenate([produced[c][field] for c in CASES])[mask]
            values = values[np.isfinite(values)]; reported = observed[field]
            assert reported['count'] == int(mask.sum())
            assert reported['finite_count'] == len(values)
            if len(values):
                assert reported['minimum'] == float(values.min()) and reported['maximum'] == float(values.max())
                assert reported['mean'] == float(values.mean())
                assert reported['percentiles'] == {str(k): float(np.percentile(values, k)) for k in (10, 25, 50, 75, 90, 95)}
        if group in canonical['diagnostics']['all']['event_clusters']:
            expected = sum([clusters(c, packets[c]['time_s'], pieces[i]) for i, c in enumerate(CASES)], [])
            assert canonical['diagnostics']['all']['event_clusters'][group] == expected
    expected_stop = (own['bounds']['full']['combined_risk_r']['unknown_unsafe_count'] > 0 or
        own['bounds']['full']['combined_risk_r']['continuously_forced_useful_losses'] >
        own['bounds']['full']['combined_risk_r']['harmful_opportunities'])
    assert canonical['conditional_stop'] == expected_stop
    assert canonical['schedule_evaluation'] is None and canonical['new_physics_steps'] == 0
    out = dict(created_utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)),
        scope='Post-result thin field mapping and independent fixed-group aggregation; no change to presealed audit',
        input_hashes={'canonical_science': sha(root/'analysis/stage01/science.json'),
                      'independent_report': sha(root/'review/results_stage01/report.json')},
        primary_bounds=bound_checks, old99_identity_set_exact=True,
        diagnostic_counts=rows, fixed_risk_statistics_exact_from_producer_arrays=feature_stats_exact,
        event_clusters_exact=True, stop_decision_exact=True, no_schedule_or_new_physics=True,
        qualification='Independent raw risk reconstruction has separately reported tiny floating differences; exact descriptor aggregation here uses the frozen producer arrays without replacing independent values.')
    (root/'review/canonical_comparison.json').write_text(json.dumps(out, indent=2)+'\n')
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
