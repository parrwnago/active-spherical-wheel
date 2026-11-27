"""Independent scalar I-62 grid, training feasibility, selection and OOF audit.

Reads frozen I-61 features/frames, not the new sweep implementation. Every
parameter uses direct label choices and exact point distances/percentiles.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import numpy as np

COUNT_KEYS=('count','point_count','errors','harmful','useful_retained','retained','vetoes')
POINT_KEYS=('SSE_mm2','RMS_mm','p95_mm','maximum_mm')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}


def safe(value):
    if isinstance(value,dict):return {k:safe(v)for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [safe(v)for v in value]
    if isinstance(value,np.generic):return safe(value.item())
    if isinstance(value,float)and not np.isfinite(value):return None
    return value


def read_case(root,case):
    features=load(root/'campaign/features_stage01'/case/'load_features_and_policies.npz')
    f=load(root/'analysis/stage01'/(case+'_evaluation.npz'))
    p=f['primary_mask']&f['truth_eligible'];truth=f['truth_family']
    point=p&np.isfinite(f['N_point_m']).all(axis=1)&np.isfinite(f['A_point_m']).all(axis=1)&np.isfinite(f['truth_equivalent_point_m']).all(axis=1)
    dn=np.linalg.norm(f['N_point_m'][point]-f['truth_equivalent_point_m'][point],axis=1)*1000.
    da=np.linalg.norm(f['A_point_m'][point]-f['truth_equivalent_point_m'][point],axis=1)*1000.
    residual=features['proposed__residual'];radius=features['proposed__radius_m'];force=features['proposed__force_norm_N']
    valid=(features['proposed__valid']&np.isfinite(residual).all(axis=1)
           &np.isfinite(radius)&np.isfinite(force)&(radius>0)&(force>=0))
    stable=p&f['stable_truth_mask']
    return dict(name=case,features=features,frames=f,primary=p,point_mask=point,
        dn=dn,da=da,truth=truth,native=f['N_family'],proposal=f['A_family'],
        accepted=features['i60_accepted'],valid=valid,
        L=np.max(abs(residual[:,6:9]),axis=1),radius=radius,force=force,
        useful=p&(f['N_family']!=truth)&(f['A_family']==truth),
        harmful_A=int(np.sum(p&(f['N_family']==truth)&(f['A_family']!=truth))),
        useful_A=int(np.sum(p&(f['N_family']!=truth)&(f['A_family']==truth))),
        stable=stable,oracle_valid=f['oracle_geometry_valid'],
        oracle_L=np.max(abs(f['oracle_joint_residual_Nm']),axis=1))


def keep_for(case,a,b):
    threshold=a+(b*case['radius'])*case['force']
    return case['accepted']&case['valid']&(case['L']<=threshold),threshold


def measure(case,keep):
    mask=case['primary'];truth=case['truth'];label=np.where(keep,case['proposal'],case['native'])
    distance=np.where(keep[case['point_mask']],case['da'],case['dn'])
    metric=dict(count=int(mask.sum()),point_count=len(distance),
        errors=int(np.sum(mask&(label!=truth))),
        harmful=int(np.sum(mask&(case['native']==truth)&(label!=truth))),
        useful_retained=int(np.sum(case['useful']&(label==truth))),
        retained=int(np.sum(mask&keep)),vetoes=int(np.sum(mask&case['accepted']&~keep)),
        SSE_mm2=float(np.sum(distance*distance,dtype=np.float64)),
        RMS_mm=float(np.sqrt(np.mean(distance*distance))),
        p95_mm=float(np.percentile(distance,95)),maximum_mm=float(np.max(distance)))
    return metric,distance


def combined(metrics,distances):
    d=np.concatenate(distances)
    result={key:sum(x[key]for x in metrics)for key in COUNT_KEYS}
    result.update(SSE_mm2=float(np.sum(d*d,dtype=np.float64)),RMS_mm=float(np.sqrt(np.mean(d*d))),
                  p95_mm=float(np.percentile(d,95)),maximum_mm=float(np.max(d)))
    return result


def training_gates(metric,phase_metrics,native,A,cases,indices,protocol):
    cfg=protocol['training_gates'];allow=cfg['point_comparison_roundoff_allowance_mm']
    harmful=sum(cases[i]['harmful_A']for i in indices);useful=sum(cases[i]['useful_A']for i in indices)
    result=dict(harmful_removal=bool(harmful>0 and (harmful-metric['harmful'])/harmful>=cfg['harmful_removal_fraction_min']),
        useful_retention=bool(useful>0 and metric['useful_retained']/useful>=cfg['useful_retention_fraction_min']),
        pooled_error=metric['errors']<=A['errors'],
        point_RMS=metric['RMS_mm']<=A['RMS_mm']+allow,
        point_p95=metric['p95_mm']<=A['p95_mm']+allow,
        point_maximum=metric['maximum_mm']<=native['maximum_mm']+allow,
        retained_nonempty=metric['retained']>=cfg['retained_proposal_count_min'])
    for i in indices:
        row=phase_metrics[i];baseline=cases[i]['baseline_A']
        result[cases[i]['name']+'_error']=row['errors']/row['count']<=baseline['errors']/baseline['count']+cfg['each_phase_error_rate_excess_over_A_max']
        result[cases[i]['name']+'_harmful']=row['harmful']<=cases[i]['harmful_A']
    return result


def arrays_for(n):
    result={key:np.zeros(n,np.int64)for key in COUNT_KEYS}
    result.update({key:np.zeros(n,float)for key in POINT_KEYS})
    return result


def select(grid,feasible,aindex,bindex):
    eligible=np.flatnonzero(feasible)
    if not len(eligible):return None
    return int(min(eligible,key=lambda k:(int(grid['errors'][k]),float(grid['SSE_mm2'][k]),
        int(grid['vetoes'][k]),int(aindex[k]),int(bindex[k]))))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--i61-root',type=Path,required=True);p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    protocol=json.loads(a.protocol.read_text());cases=[read_case(a.i61_root,k)for k in protocol['cases']]
    ai=np.repeat(np.arange(61),61);bi=np.tile(np.arange(61),61);alpha=ai*.005;beta=bi*.005;G=len(ai)
    assert G==protocol['grid']['count']==3721
    groups={**{'train_without_'+case['name']:[i for i in range(3)if i!=j]for j,case in enumerate(cases)},'full':[0,1,2]}
    for c in cases:
        c['baseline_N'],c['distance_N']=measure(c,np.zeros(len(c['accepted']),bool))
        c['baseline_A'],c['distance_A']=measure(c,c['accepted'])
        keep,_=keep_for(c,.03,.10)
        np.testing.assert_array_equal(keep,c['frames']['C_uses_proposal'])
        np.testing.assert_array_equal(np.where(keep,c['proposal'],c['native']),c['frames']['C_family'])
        point=np.where(keep[:,None],c['frames']['A_point_m'],c['frames']['N_point_m'])
        np.testing.assert_array_equal(point,c['frames']['C_point_m'])
    baseline={name:{policy:combined([cases[i]['baseline_'+policy]for i in ids],[cases[i]['distance_'+policy]for i in ids])for policy in ('N','A')}for name,ids in groups.items()}
    case_grid={c['name']:arrays_for(G)for c in cases};group_grid={name:arrays_for(G)for name in groups}
    feasible={name:np.zeros(G,bool)for name in groups};gate_grid={name:{}for name in groups}
    oracle={c['name']:np.zeros(G,np.int64)for c in cases}
    for k,(aa,bb)in enumerate(zip(alpha,beta)):
        metrics=[];distances=[]
        for c in cases:
            keep,threshold=keep_for(c,float(aa),float(bb));m,d=measure(c,keep);metrics.append(m);distances.append(d)
            for key,value in m.items():case_grid[c['name']][key][k]=value
            op=c['stable']&c['oracle_valid']&(c['oracle_L']<=threshold)
            oracle[c['name']][k]=int(op.sum())
        for name,ids in groups.items():
            m=combined([metrics[i]for i in ids],[distances[i]for i in ids])
            for key,value in m.items():group_grid[name][key][k]=value
            gates=training_gates(m,metrics,baseline[name]['N'],baseline[name]['A'],cases,ids,protocol)
            for key,value in gates.items():
                if key not in gate_grid[name]:gate_grid[name][key]=np.zeros(G,bool)
                gate_grid[name][key][k]=value
            feasible[name][k]=all(gates.values())
    report=dict(iteration='I-62',completed_utc=datetime.now(timezone.utc).isoformat(),numpy_version=np.__version__,
        protocol_sha256=sha(a.protocol),new_physics_runs=0,groups={},folds={},
        benchmark_I61_all_row_policies_exact=True,baselines=baseline,
        classification='Retrospective internal leave-one-start-out assessment; earlier cases known, no unseen validation or physical-control claim.')
    packed={'a_index':ai,'b_index':bi,'a_Nm':alpha,'b':beta}
    for c in cases:
        name=c['name']
        for key,value in case_grid[name].items():packed['case__'+name+'__'+key]=value
        packed['case__'+name+'__oracle_pass_count']=oracle[name]
    selections={name:select(group_grid[name],feasible[name],ai,bi)for name in groups}
    for name in groups:
        for key,value in group_grid[name].items():packed['group__'+name+'__'+key]=value
        packed['group__'+name+'__feasible']=feasible[name]
        for key,value in gate_grid[name].items():packed['group__'+name+'__gate__'+key]=value
        selected=selections[name]
        report['groups'][name]=dict(training_cases=[cases[i]['name']for i in groups[name]],feasible_count=int(feasible[name].sum()),
            selected_index=selected,qualified=selected is not None,
            selected_parameters=None if selected is None else dict(a_Nm=float(alpha[selected]),b=float(beta[selected]),a_index=int(ai[selected]),b_index=int(bi[selected])),
            best_metrics=None if selected is None else {key:safe(value[selected])for key,value in group_grid[name].items()})
    oof_metrics=[];oof_distances=[];oof_arrays={}
    for holdout,c in enumerate(cases):
        name='train_without_'+c['name'];selected=selections[name]
        keep=c['accepted'].copy()if selected is None else keep_for(c,float(alpha[selected]),float(beta[selected]))[0]
        m,d=measure(c,keep);oof_metrics.append(m);oof_distances.append(d)
        report['folds'][c['name']]=dict(training_group=name,qualified=selected is not None,
            selected_index=selected,diagnostic_fallback='off/A'if selected is None else None,heldout_metrics=m,
            relative_harm_removal=None if c['harmful_A']==0 else (c['harmful_A']-m['harmful'])/c['harmful_A'],
            harm_relative_scorable=c['harmful_A']>0)
        oof_arrays[c['name']+'__keep']=keep
        oof_arrays[c['name']+'__family']=np.where(keep,c['proposal'],c['native'])
        oof_arrays[c['name']+'__point_m']=np.where(keep[:,None],c['frames']['A_point_m'],c['frames']['N_point_m'])
    total=combined(oof_metrics,oof_distances)
    gates=training_gates(total,oof_metrics,baseline['full']['N'],baseline['full']['A'],cases,[0,1,2],protocol)
    fits=all(f['qualified']for f in report['folds'].values())
    report['OOF']=dict(metrics=total,gates=gates,every_fold_qualified=fits,success=bool(fits and all(gates.values())))
    report['full_data_fit_descriptive_only']=report['groups']['full']
    a.output.mkdir(parents=True);np.savez_compressed(a.output/'grid_metrics.npz',**packed)
    np.savez_compressed(a.output/'oof_policies.npz',**oof_arrays)
    (a.output/'report.json').write_text(json.dumps(safe(report),indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(feasible_counts={k:v['feasible_count']for k,v in report['groups'].items()},folds=report['folds'],OOF=report['OOF'])))


if __name__=='__main__':main()
