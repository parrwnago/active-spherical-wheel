"""Independent I-61 truth-label, oracle, paired-policy/point metric and gate audit."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import numpy as np

POLICIES=('N','A','C')
TARGETS=('equivalent_raw','dominant_patch_raw','equivalent_radial_R_proxy','dominant_patch_radial_R_proxy')
SUBSETS=('all','native_correct','A_accepted','C_retained','stable','one_resolved_contact_gt005N')


def load(path):
    with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def rate(a,b):return a/b if b else None


def compare(actual,expected,path=''):
    if isinstance(expected,dict):
        assert set(actual)==set(expected),path
        return max((compare(actual[k],expected[k],path+'/'+k)for k in expected),default=0.)
    if isinstance(expected,list):
        assert len(actual)==len(expected),path
        return max((compare(a,b,path+'/'+str(i))for i,(a,b)in enumerate(zip(actual,expected))),default=0.)
    if isinstance(expected,float):
        assert math.isclose(actual,expected,rel_tol=1e-12,abs_tol=1e-10),(path,actual,expected)
        return abs(actual-expected)
    assert actual==expected,(path,actual,expected)
    return 0.


def distance_stats(values):
    if not len(values):return dict(count=0,mean_mm=None,RMS_mm=None,median_mm=None,p95_mm=None,maximum_mm=None)
    d=1000.*values
    return dict(count=len(d),mean_mm=float(np.mean(d)),RMS_mm=math.sqrt(float(d@d)/len(d)),
                median_mm=float(np.quantile(d,.5)),p95_mm=float(np.quantile(d,.95)),maximum_mm=float(max(d)))


def masks_and_oracle(f,pack,protocol):
    loads=f['truth_patch_loads_N'];valid=np.isfinite(loads).all(axis=1)&(loads>=0).all(axis=1)
    clean=np.where(np.isfinite(loads),loads,0.);total=clean.sum(axis=1);maximum=clean.max(axis=1)
    unique=(clean==maximum[:,None]).sum(axis=1)==1
    share=np.divide(maximum,total,out=np.zeros_like(total),where=total>0)
    loaded=valid&(total>=protocol['evaluation']['truth_total_min_N'])&(maximum>0)
    eligible=loaded&unique&(share>=protocol['evaluation']['truth_dominant_fraction_min'])
    truth=np.full(len(total),-1,int)
    for row in np.flatnonzero(eligible):truth[row]=next(j for j in range(4)if clean[row,j]==maximum[row])
    stable=np.zeros(len(truth),bool)
    for row in range(2,len(truth)-2):stable[row]=all(eligible[j]and truth[j]==truth[row]for j in range(row-2,row+3))
    for actual,saved in ((truth,f['truth_family']),(eligible,f['truth_eligible']),(loaded,f['truth_loaded']),(stable,f['stable_truth_mask'])):
        np.testing.assert_array_equal(actual,saved)
    np.testing.assert_array_equal(share,f['truth_dominant_fraction'])
    oracle=np.full((len(truth),3),np.nan)
    for row in np.flatnonzero(eligible):
        family=truth[row];point=f['truth_dominant_patch_point_m'][row]
        moment=np.cross(point-pack['map__centre_m'][row],pack['observer_wrench'][row,:3])
        for joint in range(3):
            oracle[row,joint]=pack['map__joint_at_centre'][row,family,joint]+np.dot(pack['map__joint_moment_map'][row,family,joint],moment)
    np.testing.assert_allclose(oracle,f['oracle_joint_prediction_Nm'],atol=1e-10,rtol=1e-12)
    residual=pack['observer_generalized'][:,6:9]-oracle
    np.testing.assert_allclose(residual,f['oracle_joint_residual_Nm'],atol=1e-10,rtol=1e-12)
    oracle_valid=eligible&pack['map__valid']&np.isfinite(f['truth_dominant_patch_point_m']).all(axis=1)&np.isfinite(oracle).all(axis=1)&np.isfinite(residual).all(axis=1)
    tau=pack['proposed_gate__threshold_Nm']
    passes=oracle_valid&np.all(abs(residual)<=tau[:,None],axis=1)
    np.testing.assert_array_equal(oracle_valid,f['oracle_geometry_valid'])
    np.testing.assert_array_equal(passes,f['oracle_geometry_pass'])
    true=f['truth_family']
    harmful_a=eligible&(f['N_family']==true)&(f['A_family']!=true)
    harmful_c=eligible&(f['N_family']==true)&(f['C_family']!=true)
    useful=eligible&(f['N_family']!=true)&(f['A_family']==true)
    retained=useful&(f['C_family']==true)
    for actual,key in ((harmful_a,'harmful_A'),(harmful_c,'harmful_C'),(useful,'useful_A'),(retained,'retained_useful')):
        np.testing.assert_array_equal(actual,f[key])


def summary(f,window):
    truth=f['truth_family'];selected=window&f['truth_eligible'];n=int(selected.sum())
    A=int(np.sum(window&f['harmful_A']));C=int(np.sum(window&f['harmful_C']))
    useful=int(np.sum(window&f['useful_A']));retained=int(np.sum(window&f['retained_useful']))
    o=selected&f['stable_truth_mask'];on=int(o.sum());ov=int(np.sum(o&f['oracle_geometry_valid']));op=int(np.sum(o&f['oracle_geometry_pass']))
    result=dict(counts=dict(window_rows=int(window.sum()),dominant_eligible=n,
        loaded=int(np.sum(window&f['truth_loaded'])),loaded_nondominant=int(np.sum(window&f['truth_loaded']&~f['truth_eligible'])),
        A_proposals=int(np.sum(selected&f['A_uses_proposal'])),C_retained_proposals=int(np.sum(selected&f['C_uses_proposal'])),
        vetoed_proposals=int(np.sum(selected&f['A_uses_proposal']&~f['C_uses_proposal'])),
        unknown_load_proposals=int(np.sum(selected&f['A_uses_proposal']&f['load_unknown'])),
        A_changed_family=int(np.sum(selected&f['A_uses_proposal']&(f['A_family']!=f['N_family']))),
        C_changed_family=int(np.sum(selected&f['C_uses_proposal']&(f['C_family']!=f['N_family'])))),
        policies={},harmful_overrides=dict(A=A,C=C,relative_reduction=rate(A-C,A)),
        useful_corrections=dict(A=useful,C_retained=retained,retention=rate(retained,useful)),
        oracle_geometry=dict(denominator=on,valid_count=ov,pass_count=op,valid_fraction=rate(ov,on),pass_fraction=rate(op,on)),points={})
    for policy in POLICIES:
        label=f[policy+'_family'];errors=int(np.sum(selected&(label!=truth)))
        switching=sum(int(label[i]!=label[i+1])for i in range(len(label)-1)
                      if window[i]and window[i+1]and f['truth_eligible'][i]and f['truth_eligible'][i+1]and truth[i]==truth[i+1])
        result['policies'][policy]=dict(error_count=errors,error_fraction=rate(errors,n),unknown_count=int(np.sum(selected&(label<0))),
            sampled_disagreement_equivalent_s=.005*errors,false_switch_count=switching)
    subset_masks=[selected,selected&(f['N_family']==truth),selected&f['A_uses_proposal'],selected&f['C_uses_proposal'],
                  selected&f['stable_truth_mask'],selected&(f['truth_resolved_contact_counts_gt005N'].sum(axis=1)==1)]
    target_values=[f['truth_equivalent_point_m'],f['truth_dominant_patch_point_m'],f['equivalent_radial_R_proxy'],f['dominant_patch_radial_R_proxy']]
    finite={p:np.isfinite(f[p+'_point_m']).all(axis=1)for p in POLICIES}
    for target_name,target in zip(TARGETS,target_values):
        result['points'][target_name]={};tf=np.isfinite(target).all(axis=1)
        for name,mask in zip(SUBSETS,subset_masks):
            matched=mask&tf&finite['N']&finite['A']&finite['C']
            stats={p:distance_stats(np.sqrt(np.sum((f[p+'_point_m'][matched]-target[matched])**2,axis=1)))for p in POLICIES}
            result['points'][target_name][name]=dict(eligible_count=int(mask.sum()),matched_count=int(matched.sum()),
                matched_fraction=rate(int(matched.sum()),int(mask.sum())),missing_target=int(np.sum(mask&~tf)),
                missing_policy={p:int(np.sum(mask&~finite[p]))for p in POLICIES},policies=stats)
    return result


def gate_values(total,cases,p):
    cfg=p['gates'];output=[]
    def add(name,value,limit,upper=False,group='performance'):
        scorable=value is not None
        passes=scorable and limit is not None and (value<=limit if upper else value>=limit)
        output.append(dict(name=name,actual=value,threshold=limit,scorable=scorable,passed=bool(passes),group=group))
    add('harmful_override_reduction',total['harmful_overrides']['relative_reduction'],cfg['harmful_override_relative_reduction_min'])
    add('useful_correction_retention',total['useful_corrections']['retention'],cfg['useful_correction_retention_min'])
    n,a,c=[total['policies'][p]['error_fraction']for p in POLICIES]
    diff=None if a is None or c is None else c-a
    add('pooled_C_vs_A_error',diff,cfg['pooled_C_error_above_A_allowance'],True)
    for case,windows in cases.items():
        row=windows['primary'];a,c=row['policies']['A']['error_fraction'],row['policies']['C']['error_fraction']
        diff=None if a is None or c is None else c-a
        add(case+'_C_vs_A_error',diff,cfg['per_phase_C_error_above_A_allowance'],True)
    c=total['policies']['C']['error_fraction'];diff=None if n is None or c is None else n-c
    limit=None if n is None else max(cfg['C_native_error_reduction_absolute_min'],cfg['C_native_error_reduction_relative_min']*n)
    add('C_improves_native',diff,limit)
    stats=total['points']['equivalent_raw']['all']['policies']
    for key,allowance in [('RMS_mm','point_C_above_native_RMS_m_allowance'),('p95_mm','point_C_above_native_p95_m_allowance'),('maximum_mm','point_C_above_native_maximum_m_allowance')]:
        n,c=stats['N'][key],stats['C'][key];diff=None if n is None or c is None else .001*(c-n)
        add('point_'+key+'_C_vs_native_m',diff,cfg[allowance],True)
    for case,windows in cases.items():add(case+'_oracle_geometry_compatibility',windows['primary']['oracle_geometry']['pass_fraction'],cfg['per_phase_oracle_geometry_pass_fraction_min'],False,'model_validity')
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evaluation-root',type=Path,required=True);p.add_argument('--features-root',type=Path,required=True)
    p.add_argument('--protocol',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    protocol=json.loads(a.protocol.read_text());saved=json.loads((a.evaluation_root/'science.json').read_text())
    cases={};frames=[];checks={}
    for case in protocol['datasets']:
        f=load(a.evaluation_root/(case+'_evaluation.npz'));pack=load(a.features_root/case/'load_features_and_policies.npz')
        masks_and_oracle(f,pack,protocol);t=f['time_s'];definitions={'primary':protocol['evaluation']['primary_window_s'],**protocol['evaluation']['secondary_windows_s']}
        masks={key:(t>=win[0]-1e-9)&(t<win[1]-1e-9)for key,win in definitions.items()}
        masks['stable_primary']=masks['primary']&f['stable_truth_mask'];assert masks['primary'].sum()==8000 and not masks['full'][-1]
        np.testing.assert_array_equal(masks['primary'],f['primary_mask'])
        windows={key:summary(f,mask)for key,mask in masks.items()};difference=compare(windows,saved['cases'][case]['windows'])
        cases[case]=windows;frames.append(f);checks[case]=dict(rows=len(t),all_windows_counts_and_statistics_match=True,max_absolute_statistic_difference=difference,
                                                               derived_truth_stability_oracle_and_confusion_flags_exact=True)
    combined={key:np.concatenate([f[key]for f in frames])for key in frames[0]}
    total=summary(combined,combined['primary_mask']);pooled_diff=compare(total,saved['aggregate_primary'])
    gates=gate_values(total,cases,protocol);compare(gates,saved['gates'])
    failed=[g['name']for g in gates if not g['passed']];assert failed==saved['failed_gates']
    assert all(g['passed']for g in gates)==saved['all_gates_passed']
    assert all(g['passed']for g in gates if g['group']=='performance')==saved['performance_gates_passed']
    assert all(g['passed']for g in gates if g['group']=='model_validity')==saved['model_validity_gates_passed']
    report=dict(iteration='I-61',completed_utc=datetime.now(timezone.utc).isoformat(),new_physics_runs=0,cases=checks,
        pooled_max_absolute_statistic_difference=pooled_diff,primary_counts=total['counts'],primary_policy_metrics=total['policies'],
        harmful_overrides=total['harmful_overrides'],useful_corrections=total['useful_corrections'],
        primary_point_stats=total['points']['equivalent_raw']['all'],gates=gates,failed_gates=failed,all_checks_pass=True,
        canonical_sha256=sha(a.evaluation_root/'science.json'),scope='Independent all fixed-window/paired-point/count/gate arithmetic and oracle geometry computation; no evaluator/new load helper import; no force calibration or control-benefit claim.')
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
