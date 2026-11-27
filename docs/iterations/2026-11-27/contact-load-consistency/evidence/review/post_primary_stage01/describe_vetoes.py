"""Post-primary I-61 veto composition and fixed adverse-load interpretation."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import numpy as np

LABELS=('shell_A','shell_B','belt_A','belt_B')
TIMES={'m100_B':35.205,'p030_B':38.380,'p090_B':17.765}


def load(path):
    with np.load(path,allow_pickle=False)as z:return {key:z[key].copy()for key in z.files}


def point_stats(f,mask):
    result={}
    for policy in ('N','A','C'):
        point=f[policy+'_point_m'][mask];target=f['truth_equivalent_point_m'][mask]
        d=np.linalg.norm(point-target,axis=1)*1000.
        result[policy]=dict(count=len(d),mean_mm=float(d.mean())if len(d)else None,
                           RMS_mm=float(np.sqrt(np.mean(d*d)))if len(d)else None,
                           maximum_mm=float(d.max())if len(d)else None)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evaluation-root',type=Path,required=True);p.add_argument('--features-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    cases={}
    for case in TIMES:
        f=load(a.evaluation_root/(case+'_evaluation.npz'));pack=load(a.features_root/case/'load_features_and_policies.npz')
        primary=f['primary_mask']&f['truth_eligible'];veto=primary&f['A_uses_proposal']&~f['C_uses_proposal']
        lost=veto&f['useful_A'];removed=veto&f['harmful_A']
        neutral=veto&~lost&~removed;stable=f['stable_truth_mask']
        per_joint=np.sum(veto[:,None]&(abs(f['proposed_normalized_joint_residual'])>1.),axis=0)
        masks={'vetoed':veto,'useful_lost':lost,'harmful_removed':removed,'neutral_label':neutral}
        composition={name:dict(count=int(mask.sum()),stable_count=int(np.sum(mask&stable)),
            oracle_compatible_count=int(np.sum(mask&f['oracle_geometry_pass'])),
            point_comparison=point_stats(f,mask))for name,mask in masks.items()}
        primary_table={}
        for name,mask in [('primary',primary),('stable_primary',primary&stable)]:
            primary_table[name]=dict(rows=int(mask.sum()),
                errors={policy:int(np.sum(mask&(f[policy+'_family']!=f['truth_family'])))for policy in ('N','A','C')},
                harmful_A=int(np.sum(mask&f['harmful_A'])),harmful_C=int(np.sum(mask&f['harmful_C'])),
                useful_A=int(np.sum(mask&f['useful_A'])),useful_retained=int(np.sum(mask&f['retained_useful'])))
        i=int(np.argmin(abs(f['time_s']-TIMES[case])));assert abs(f['time_s'][i]-TIMES[case])<1e-8
        tau=float(f['threshold_Nm'][i]);force=float(f['force_norm_N'][i])
        event=dict(time_s=float(f['time_s'][i]),truth_family=int(f['truth_family'][i]),
            N_family=int(f['N_family'][i]),A_family=int(f['A_family'][i]),C_family=int(f['C_family'][i]),
            I60_accepted=bool(f['A_uses_proposal'][i]),retained=bool(f['C_uses_proposal'][i]),
            load_pass=bool(f['load_passes'][i]),force_norm_N=force,tolerance_each_joint_Nm=tau,
            observed_joint_Nm=pack['observer_generalized'][i,6:9].tolist(),
            proposed_joint_Nm=pack['proposed__prediction'][i,6:9].tolist(),
            proposed_residual_Nm=f['proposed_joint_residual_Nm'][i].tolist(),
            proposed_ratio=f['proposed_normalized_joint_residual'][i].tolist(),
            native_family_alternative_prediction_Nm=pack['native_alternative__prediction'][i,6:9].tolist(),
            oracle_geometry_prediction_Nm=f['oracle_joint_prediction_Nm'][i].tolist(),
            oracle_ratio=f['oracle_joint_normalized_residual'][i].tolist(),
            point_distance_equivalent_mm={policy:float(1000*np.linalg.norm(f[policy+'_point_m'][i]-f['truth_equivalent_point_m'][i]))for policy in ('N','A','C')},
            meaning='Compatibility only under fixed tolerance; do not infer unique physical cause, exact contact truth, or threshold optimum.')
        cases[case]=dict(veto_composition=composition,per_joint_rejection_count_may_overlap=per_joint.tolist(),
                         primary_and_stable=primary_table,fixed_I60_worst_selected_point=event)
    out=dict(iteration='I-61',completed_utc=datetime.now(timezone.utc).isoformat(),cases=cases,new_physics_runs=0,
        scope='Post-primary descriptive fixed-event/veto composition; no score, threshold, policy or acceptance decision changed.',
        qualification='Oracle geometry still uses observed total F and last-1ms pressure-weighted means; stable +/-10ms does not eliminate filtering/model/force-distribution ambiguity.')
    a.output.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out))


if __name__=='__main__':main()
