"""I-64 fixed matched-history model, ranking and shadow-policy diagnostics.

Consumes separately sealed causal feature packets. No tuning, new physics,
candidate/history feedback, or automatic live estimator adoption.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse,json,shutil

import numpy as np

import i62_metrics as M

CASES=('m100_B','p030_B','p090_B')
MODELS=('M0','M1','M2','M3')


def join(base,features,primitives):
    if not np.array_equal(base['time_s'],features['time_s']) or not np.array_equal(base['time_s'],primitives['time_s']):
        raise ValueError('Frozen input times differ')
    data={k:v.copy() for k,v in base.items()}
    for key,value in features.items():
        if key=='time_s' or value.ndim<1 or len(value)!=len(base['time_s']):continue
        data[('feature_'+key) if key in data else key]=value
    for key in ('observer_generalized','observer_interval_mean','H_F_valid','H_Nalt_valid','H_Neq_valid'):
        data[key]=primitives[key]
    for key in ('I61_F_residual_Nm','I61_F_valid','I61_Nalt_residual_Nm','I61_Nalt_valid','I61_C_uses_proposal',
                'material_F_valid','material_Nalt_valid','material_Neq_valid'):
        data[key]=primitives[key]
    scale=.03+(.10*data['radius_m'])*data['force_norm_N']
    data['fixed_scale_Nm']=scale
    for model in MODELS:
        for branch in ('F','Nalt'):
            prefix=model+'_'+branch;residual=data[prefix+'_residual_Nm']
            valid=data[prefix+'_valid']&np.isfinite(residual).all(axis=1)&np.isfinite(scale)&(scale>0)
            score=np.full(len(scale),np.nan);score[valid]=np.max(abs(residual[valid]),axis=1)/scale[valid]
            data[prefix+'_score']=score;data[prefix+'_score_valid']=valid
    return data


def domain(cases):return M.domain(list(cases.values()))


def classify_pair(F,N,valid,epsilon):
    delta=F-N;out=np.full(len(F),-2,dtype=np.int8)
    out[valid&(delta < -epsilon)]=-1 # F preferred
    out[valid&(delta > epsilon)]=1   # Nalt preferred
    out[valid&(abs(delta)<=epsilon)]=0
    return out


def shadow_keep(data,model):
    accepted=data['i60_accepted'];current_map=data['H_F_valid']
    known=data[model+'_F_score_valid']
    # Invalid current physical F maps keep original unknown/native semantics.
    # A valid current map with missing/unwarmed history disables the new veto.
    if model in ('M0','M1'):return accepted&current_map&known&(data[model+'_F_score']<=1.)
    return accepted&current_map&(~known|(data[model+'_F_score']<=1.))


def statistics(residual,mask):
    selected=mask&np.isfinite(residual).all(axis=1);x=residual[selected]
    if not len(x):return dict(count=0,vector_RMS_Nm=None,vector_p95_Nm=None,per_joint_RMS_Nm=None,maxabs_p95_Nm=None)
    norm=np.linalg.norm(x,axis=1)
    return dict(count=len(x),vector_RMS_Nm=float(np.sqrt(np.mean(np.sum(x*x,axis=1)))),
        vector_p95_Nm=float(np.percentile(norm,95)),per_joint_RMS_Nm=np.sqrt(np.mean(x*x,axis=0)).tolist(),
        maxabs_p95_Nm=float(np.percentile(np.max(abs(x),axis=1),95)))


def groups(data,old99,pooled_native_max=None):
    Nwrong=data['N_family']!=data['truth_family'];Awrong=data['A_family']!=data['truth_family'];accepted=data['i60_accepted']
    point=data['point_mask'];native_max=(float(data['native_distance_m'][point].max()*1000.) if pooled_native_max is None else pooled_native_max)
    A_distance=np.where(accepted,data['proposed_distance_m'],data['native_distance_m'])*1000.
    return dict(primary=np.ones(len(accepted),bool),accepted=accepted,family_correct=accepted&~Awrong,
        useful=Nwrong&~Awrong,harmful=~Nwrong&Awrong,accepted_wrong=accepted&Awrong,
        lost_useful_I61=Nwrong&~Awrong&~data['i61_keep_proposal'],old99=old99,
        unsafe_point=accepted&point&(A_distance>native_max+1e-12))


def pair_report(data,mask,model,epsilon):
    valid=data[model+'_F_score_valid']&data[model+'_Nalt_score_valid']
    preference=classify_pair(data[model+'_F_score'],data[model+'_Nalt_score'],valid,epsilon)
    delta=data[model+'_F_score']-data[model+'_Nalt_score']
    return dict(denominator=int(mask.sum()),known=int(np.sum(mask&valid)),unknown=int(np.sum(mask&~valid)),
        F_preferred=int(np.sum(mask&(preference==-1))),Nalt_preferred=int(np.sum(mask&(preference==1))),
        tied=int(np.sum(mask&(preference==0))),
        near_tie_within_2e_minus8_verification_envelope=int(np.sum(mask&valid&(abs(delta)<=epsilon+2e-8))))


def compare_models(data,masks,protocol):
    epsilon=protocol['current_branch_discrimination']['tie_epsilon_dimensionless'];result={}
    for name,mask in masks.items():
        common=mask&data['M0_F_score_valid']&data['M2_F_score_valid']
        pairs=common&data['M0_Nalt_score_valid']&data['M2_Nalt_score_valid']
        row=dict(denominator=int(mask.sum()),common_M0_M2_F_count=int(common.sum()),
            common_M0_M2_pair_count=int(pairs.sum()),models={},paired_common={},compatibility={})
        for model in MODELS:
            row['models'][model]=statistics(data[model+'_F_residual_Nm'],mask&data[model+'_F_score_valid'])
            row['paired_common'][model]=statistics(data[model+'_F_residual_Nm'],common&data[model+'_F_score_valid'])
            row['compatibility'][model]=dict(denominator=int(mask.sum()),known=int(np.sum(mask&data[model+'_F_score_valid'])),
                compatible=int(np.sum(mask&data[model+'_F_score_valid']&(data[model+'_F_score']<=1.))))
        row['ranking_all']={model:pair_report(data,mask,model,epsilon) for model in MODELS}
        row['ranking_identical_M0_M2_frames']={model:pair_report(data,pairs,model,epsilon) for model in ('M0','M2')}
        result[name]=row
    return result


def history_family_correct(data):
    good=(data['truth_family']>=0)&(data['A_family']==data['truth_family']);answer=np.zeros(len(good),bool)
    for i in range(10,len(good)):
        indices=slice(i-10,i+1)
        answer[i]=good[indices].all() and np.all(abs(np.diff(data['time_s'][indices])-.005)<=1e-9)
    return answer


def decomposition(data):
    t=data['time_s'];gamma=data['observer_generalized'][:,6:9];n=len(t);out={}
    alpha=np.full(n,np.nan);alpha[1:]=1.-np.exp(-100.*np.diff(t))
    for model in ('M2','M3'):
        prior=data[model+'_anchor_prior_Nm'];history=np.full((n,3),np.nan)
        history[1:]=(1.-alpha[1:,None])*(gamma[:-1]-prior[1:])
        for branch in ('F','Nalt'):
            prefix=model+'_'+branch;current=alpha[:,None]*data['M1_'+branch+'_residual_Nm']
            total=history+current;error=data[prefix+'_residual_Nm']-total
            data[prefix+'_inherited_term_Nm']=history.copy();data[prefix+'_current_term_Nm']=current
            data[prefix+'_analysis_decomposition_error_Nm']=error
            valid=data[prefix+'_score_valid']&np.isfinite(total).all(axis=1)
            out[prefix]=dict(valid_count=int(valid.sum()),maximum_identity_error_Nm=None if not valid.any() else float(np.max(abs(error[valid]))))
    return out


def decomposition_summary(data,mask,model='M2'):
    prefix=model+'_F';h=data[prefix+'_inherited_term_Nm'];c=data[prefix+'_current_term_Nm']
    valid=mask&data[prefix+'_score_valid']&np.isfinite(h).all(axis=1)&np.isfinite(c).all(axis=1)
    hn=np.linalg.norm(h[valid],axis=1);cn=np.linalg.norm(c[valid],axis=1);den=hn+cn
    share=np.divide(hn,den,out=np.zeros_like(hn),where=den>0)
    return dict(count=int(valid.sum()),inherited=statistics(h,valid),current=statistics(c,valid),
        cancellation_count=int(np.sum(np.sum(h[valid]*c[valid],axis=1)<0)),
        inherited_norm_share_median=None if not len(share) else float(np.median(share)))


def qualify_Q1(full,percase,masks,protocol):
    q=protocol['model_consistency_qualification'];eps=q['roundoff_allowance_Nm']
    common=masks['family_correct']&full['M0_F_score_valid']&full['M2_F_score_valid']
    old=statistics(full['M0_F_residual_Nm'],common);new=statistics(full['M2_F_residual_Nm'],common)
    known=full['M0_F_score_valid']&full['M2_F_score_valid'];accepted=full['i60_accepted']
    coverage_primary=float(np.mean(known));coverage_accepted=float(np.sum(known&accepted)/accepted.sum())
    critical=masks['harmful']|masks['accepted_wrong']|masks['unsafe_point']
    flags=dict(primary_coverage=coverage_primary>=q['coverage_min'],accepted_coverage=coverage_accepted>=q['coverage_min'],
        critical_events_scorable=bool(np.all(known[critical])),
        pooled_RMS_reduction=bool(old['count'] and new['vector_RMS_Nm']<=(1.-q['minimum_pooled_vector_RMS_reduction_fraction'])*old['vector_RMS_Nm']+eps),
        pooled_p95_nonincrease=bool(old['count'] and new['vector_p95_Nm']<=old['vector_p95_Nm']+eps))
    phases={}
    for case,d in percase.items():
        mask=d['i60_accepted']&(d['A_family']==d['truth_family'])&d['M0_F_score_valid']&d['M2_F_score_valid']
        a=statistics(d['M0_F_residual_Nm'],mask);b=statistics(d['M2_F_residual_Nm'],mask)
        flags[case+'_RMS_nonincrease']=bool(a['count'] and b['vector_RMS_Nm']<=a['vector_RMS_Nm']+eps)
        phases[case]=dict(M0=a,M2=b)
    return dict(common_correct_count=int(common.sum()),M0=old,M2=new,coverage_primary=coverage_primary,
        coverage_accepted=coverage_accepted,critical_denominator=int(critical.sum()),critical_unknown=int(np.sum(critical&~known)),
        per_case=phases,gates=flags,qualified=all(flags.values()))


def qualify_Q2(full,masks,protocol):
    epsilon=protocol['current_branch_discrimination']['tie_epsilon_dimensionless'];counts={}
    for model in ('M0','M2'):
        h=pair_report(full,masks['harmful'],model,epsilon);u=pair_report(full,masks['useful'],model,epsilon)
        counts[model]=dict(harmful=h,useful=u,correct_harmful=h['Nalt_preferred'],correct_useful=u['F_preferred'])
    old=counts['M0'];new=counts['M2'];hd=int(masks['harmful'].sum());ud=int(masks['useful'].sum())
    flags=dict(half_harmful_correct=new['correct_harmful']>=.5*hd,
        useful_90pct_correct=new['correct_useful']>=.9*ud,
        harmful_nonregression=new['correct_harmful']>=old['correct_harmful'],
        useful_nonregression=new['correct_useful']>=old['correct_useful'],
        at_least_one_improvement=(new['correct_harmful']>old['correct_harmful'] or new['correct_useful']>old['correct_useful']))
    if old['correct_harmful']==hd:
        flags['retain_all_previously_correct_harmful']=new['correct_harmful']==hd
        flags['improve_useful_when_all_harmful_already_correct']=new['correct_useful']>old['correct_useful']
    return dict(counts=counts,gates=flags,promising=all(flags.values()))


def shadow_report(data_by_case,protocol):
    cases={c:domain({c:d}) for c,d in data_by_case.items()};full=domain(data_by_case)
    bases={c:M.constants(d) for c,d in cases.items()};base=M.constants(full);out={}
    for model in MODELS:
        phase={c:M.score_keeps(d,shadow_keep(d,model)) for c,d in cases.items()}
        keep=shadow_keep(full,model);scores=M.score_keeps(full,keep)
        passed,flags=M.training_feasibility(scores,base,phase,bases,protocol)
        accepted=full['i60_accepted']
        out[model]=dict(metrics=M.scalar_metrics(scores),baseline=base,
            gates={k:bool(v[0]) for k,v in flags.items()},all_gates_pass=bool(passed[0]),
            invalid_current_F_count=int(np.sum(accepted&~full['H_F_valid'])),
            unknown_score_retaining_A_count=int(np.sum(accepted&full['H_F_valid']&~full[model+'_F_score_valid'])) if model in ('M2','M3') else 0,
            per_case={c:dict(metrics=M.scalar_metrics(phase[c]),baseline=bases[c]) for c in cases})
    return out


def clusters(t,predicate):
    rows=np.flatnonzero(predicate);groups=[]
    for i in rows:
        if not groups or abs(t[i]-t[groups[-1][-1]]-.005)>1e-9:groups.append([])
        groups[-1].append(int(i))
    return [dict(start_s=float(t[g[0]]),end_s=float(t[g[-1]]),frame_count=len(g)) for g in groups]


def old_witness_tags(data_by_case,certificate):
    tags={c:np.zeros(len(d['time_s']),bool) for c,d in data_by_case.items()}
    for row in certificate['witnesses']:
        case=row['case'];times=data_by_case[case]['time_s'];i=int(np.argmin(abs(times-row['time_s'])))
        if abs(times[i]-row['time_s'])>1e-9 or tags[case][i]:raise ValueError('Old witness time mismatch/duplicate')
        tags[case][i]=True
    if sum(int(v.sum()) for v in tags.values())!=99:raise ValueError('Expected original99 pooled witnesses')
    return tags


def verify(data_by_case,protocol):
    eps=protocol['numeric_verification']['H_times_force_I61_Nm_atol'];out={}
    for case,d in data_by_case.items():
        known=d['M0_F_score_valid'];old=d['i61_keep_proposal'];current=shadow_keep(d,'M0')
        if not np.array_equal(old,d['I61_C_uses_proposal']):raise ValueError('Primitive and compact I61 flags differ')
        if not np.array_equal(current,old):raise ValueError('M0 shadowflags differ from I61')
        family=np.where(current,d['A_family'],d['N_family'])
        if not np.array_equal(family,d['i61_C_family']):raise ValueError('M0 family differs from I61')
        point=d['primary_eligible']&d['point_mask'];distance=np.where(current,d['proposed_distance_m'],d['native_distance_m'])
        if not np.array_equal(distance[point],d['i61_distance_m'][point]):raise ValueError('M0 distances differ from I61')
        vector_checks={}
        for branch in ('F','Nalt'):
            if not np.array_equal(d['M0_'+branch+'_valid'],d['I61_'+branch+'_valid']):raise ValueError('I61 map/observation valid flags differ')
            selected=d['I61_'+branch+'_valid'];error=d['M0_'+branch+'_residual_Nm']-d['I61_'+branch+'_residual_Nm']
            maximum=float(np.max(abs(error[selected]))) if selected.any() else None
            if maximum is not None and maximum>eps:raise ValueError('Endpoint map-times-force differs from I61')
            vector_checks[branch]=maximum
        recurrence=d['observer_recurrence_residual9'];finite=np.isfinite(recurrence).all(axis=1)
        recurrence_max=float(np.max(abs(recurrence[finite]))) if finite.any() else None
        identity={}
        for branch in ('F','Nalt'):
            error=d['one_step_'+branch+'_identity_error_Nm'];ok=np.isfinite(error).all(axis=1)
            maximum=float(np.max(abs(error[ok]))) if ok.any() else None
            if maximum is not None and maximum>protocol['numeric_verification']['history_algebra_identity_Nm_atol']:
                raise ValueError('Observed-prior rawresidual scaling identity fails')
            identity[branch]=maximum
        if recurrence_max is None or recurrence_max>protocol['numeric_verification']['observer_recurrence_moment_and_joint_Nm_atol']:
            raise ValueError('Observed filter recurrence differs')
        full_error=d['observer_full_recurrence_error9'];ok=np.isfinite(full_error).all(axis=1)
        full_max=float(np.max(abs(full_error[ok]))) if ok.any() else None
        if full_max is None or full_max>protocol['numeric_verification']['observer_recurrence_moment_and_joint_Nm_atol']:
            raise ValueError('Full observer recurrence differs')
        out[case]=dict(M0_I61_flags_exact=True,M0_I61_families_exact=True,M0_I61_distances_exact=True,
            H_times_force_I61_residual_difference_Nm=vector_checks,filtered_recurrence_max9=recurrence_max,
            full_filtered_reconstruction_max9=full_max,one_step_identity_max_Nm=identity)
    return out


def execute(data_by_case,old_certificate,protocol,output):
    benchmarks=verify(data_by_case,protocol);tags=old_witness_tags(data_by_case,old_certificate)
    decomp={}
    for case,d in data_by_case.items():
        d['old99']=tags[case];d['history_family_correct']=history_family_correct(d);decomp[case]=decomposition(d)
        for record in decomp[case].values():
            maximum=record['maximum_identity_error_Nm']
            if maximum is not None and maximum>protocol['numeric_verification']['history_algebra_identity_Nm_atol']:
                raise ValueError('Matched-history residual decomposition identity differs')
    percase={c:domain({c:d}) for c,d in data_by_case.items()};full=domain(data_by_case)
    masks=groups(full,full['old99']);masks['history_family_correct']=full['i60_accepted']&full['history_family_correct']
    expected=dict(primary=23423,accepted=6902,family_correct=6884,useful=294,harmful=6,accepted_wrong=18,old99=99)
    for name,count in expected.items():
        if int(masks[name].sum())!=count:raise ValueError('Frozen diagnostic group differs: '+name)
    pooled_native_max=float(full['native_distance_m'][full['point_mask']].max()*1000.)
    comparisons={'all':compare_models(full,masks,protocol)};events={}
    for case,d in data_by_case.items():
        primary=d['primary_eligible'];eligible=percase[case];case_masks=groups(eligible,eligible['old99'],pooled_native_max)
        case_masks['history_family_correct']=eligible['i60_accepted']&eligible['history_family_correct']
        comparisons[case]=compare_models(eligible,case_masks,protocol)
        harmful=primary&(d['N_family']==d['truth_family'])&(d['A_family']!=d['truth_family'])
        useful=primary&(d['N_family']!=d['truth_family'])&(d['A_family']==d['truth_family'])
        centers=[dict(time_s=c['start_s'],reason='harmful_cluster') for c in clusters(d['time_s'],harmful)]
        point=primary&d['point_mask'];distance=np.where(d['i60_accepted'],d['proposed_distance_m'],d['native_distance_m'])
        candidates=np.flatnonzero(point);worst=int(candidates[np.argmax(distance[point])]);centers.append(dict(time_s=float(d['time_s'][worst]),reason='phase_worst_A_point'))
        if useful.any():centers.append(dict(time_s=float(d['time_s'][np.flatnonzero(useful)[0]]),reason='first_useful'))
        if case=='m100_B':
            i=int(np.argmin(abs(d['time_s']-7.875)))
            if primary[i]:centers.append(dict(time_s=float(d['time_s'][i]),reason='prior_7.875_useful'))
        trace=np.zeros(len(primary),bool)
        for c in centers:trace|=abs(d['time_s']-c['time_s'])<=.100+1e-9
        wanted={k:v for k,v in d.items() if k in ('time_s','N_family','A_family','truth_family','primary_eligible','i60_accepted','old99','history_family_correct','fixed_scale_Nm','H_F_valid','H_Nalt_valid') or k.startswith(('M0_','M1_','M2_','M3_'))}
        np.savez_compressed(output/(case+'_frames.npz'),**{k:v[primary] for k,v in wanted.items()})
        np.savez_compressed(output/(case+'_event_traces.npz'),**{k:v[trace] for k,v in wanted.items()})
        events[case]=dict(centers=centers,harmful_clusters=clusters(d['time_s'],harmful),
            accepted_wrong_clusters=clusters(d['time_s'],primary&d['i60_accepted']&(d['A_family']!=d['truth_family'])))
    result=dict(iteration='I-64',numpy_version=np.__version__,scope='retrospective endpoint/filter-model diagnostic, not new contact inference or physical improvement',
        benchmarks=benchmarks,history_decomposition_identity=decomp,comparisons=comparisons,
        geometry_coverage={c:{name:int(np.sum(d['primary_eligible']&d[name])) for name in ('H_F_valid','H_Nalt_valid','H_Neq_valid',
            'material_F_valid','material_Nalt_valid','material_Neq_valid')} for c,d in data_by_case.items()},
        history_decomposition={g:{m:decomposition_summary(full,mask,m) for m in ('M2','M3')} for g,mask in masks.items()},
        Q1_model_consistency=qualify_Q1(full,percase,masks,protocol),Q2_discrimination=qualify_Q2(full,masks,protocol),
        shadow_policies=shadow_report(data_by_case,protocol),events=events,new_physics_steps=0,live_changes=False,
        automatic_adoption=False,threshold_tuning=False)
    M.save(output/'science.json',result);return result


def run(protocol_path,base_root,feature_root,old_witnesses,output):
    protocol=json.loads(protocol_path.read_text());output.mkdir(parents=True,exist_ok=False)
    if protocol['iteration']!='I-64':raise ValueError('Expected I-64')
    paths=[base_root/(c+'.npz') for c in CASES]+[feature_root/c/name for c in CASES for name in ('primitives.npz','history_features.npz')]+[old_witnesses]
    M.save(output/'registered_before_evaluation.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=M.sha(Path(__file__)),metric_dependency_sha256=M.sha(Path(M.__file__)),protocol_sha256=M.sha(protocol_path),
        inputs={str(p):M.sha(p) for p in paths},status='before loading feature/truth numerical arrays'))
    shutil.copy2(Path(__file__),output/'history_diagnostic.py');shutil.copy2(Path(M.__file__),output/'i62_metrics.py')
    data={}
    for case in CASES:data[case]=join(M.load(base_root/(case+'.npz')),M.load(feature_root/case/'history_features.npz'),M.load(feature_root/case/'primitives.npz'))
    result=execute(data,json.loads(old_witnesses.read_text()),protocol,output)
    M.save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=M.sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(Q1_qualified=result['Q1_model_consistency']['qualified'],Q2_promising=result['Q2_discrimination']['promising'],
        M2_shadow_gates=result['shadow_policies']['M2']['all_gates_pass'],new_physics_steps=0)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('protocol','base-root','feature-root','old-witnesses','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run(a.protocol.resolve(),a.base_root.resolve(),a.feature_root.resolve(),a.old_witnesses.resolve(),a.output.resolve())
