"""Explanatory post-result I-64 useful-correction row-set comparison.

Not a preregistered gate. Reuses immutable independent scores and frozen I-62
evaluation packets. No model, parameter, inference policy or history changes.
"""
import argparse
from datetime import datetime, timezone
import hashlib, json
from pathlib import Path
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {key: z[key].copy() for key in z.files}


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def run(root, output):
    if output.exists(): raise FileExistsError(output)
    totals = {key: 0 for key in ('M0_lost', 'M2_lost', 'both_lost', 'restored', 'new_lost')}
    per_case = {}; identities = {key: [] for key in totals}; inputs = {}
    for case in CASES:
        paths = dict(base=root/'portable_inputs/i62_base'/(case+'.npz'),
            independent=root/'review/results_stage01'/(case+'_independent_history.npz'),
            primitives=root/'features/stage01'/case/'primitives.npz')
        b, f, p = (load(paths[key]) for key in ('base', 'independent', 'primitives'))
        np.testing.assert_array_equal(b['time_s'], f['time_s'])
        np.testing.assert_array_equal(b['time_s'], p['time_s'])
        accepted = b['i60_accepted']; current = p['H_F_valid']
        old_known = f['M0_F_valid'] & np.isfinite(f['M0_F_score'])
        old_keep = accepted & current & old_known & (f['M0_F_score'] <= 1.)
        np.testing.assert_array_equal(old_keep, b['i61_keep_proposal'])
        new_known = f['M2_F_valid'] & np.isfinite(f['M2_F_score'])
        new_keep = accepted & current & (~new_known | (f['M2_F_score'] <= 1.))
        useful = (b['primary_eligible'] & accepted & (b['N_family'] != b['truth_family'])
                  & (b['A_family'] == b['truth_family']))
        old = useful & ~old_keep; new = useful & ~new_keep
        masks = dict(M0_lost=old, M2_lost=new, both_lost=old & new,
                     restored=old & ~new, new_lost=new & ~old)
        per_case[case] = {key: int(mask.sum()) for key, mask in masks.items()}
        for key, mask in masks.items():
            totals[key] += int(mask.sum())
            for i in np.flatnonzero(mask):
                identities[key].append(dict(case=case, full_rate_index=int(i), time_s=float(b['time_s'][i]),
                    native_family=int(b['N_family'][i]), proposal_family=int(b['A_family'][i]),
                    dominant_truth_family=int(b['truth_family'][i]),
                    M0_F_score=float(f['M0_F_score'][i]), M2_F_score=float(f['M2_F_score'][i]),
                    current_F_map_valid=bool(current[i]), M2_warmed_score_known=bool(new_known[i])))
        inputs[case] = {key: dict(path=str(path), sha256=sha(path)) for key, path in paths.items()}
    totals['net_fewer_lost'] = totals['M0_lost']-totals['M2_lost']
    assert totals['restored']-totals['new_lost'] == totals['net_fewer_lost']
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)),
        scope='Post-result explanatory row identities; not a registered acceptance criterion or new method',
        previously_observed_counts=True, model_or_rule_changes=False, new_physics_steps=0,
        policy='Frozen M0/I61 and M2 fixed .03+.10R||filteredF|| shadow policies; original unknown/fallback semantics',
        group='Original primary eligible accepted frames where N family is wrong and A family is correct',
        inputs=inputs, totals=totals, per_case=per_case, row_identities=identities)
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(totals=totals, per_case=per_case), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); run(args.root, args.output)
