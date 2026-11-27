"""I-63 causal-risk diagnostic and conditionally enabled offline schedule fit.

No physics or controller actuation. Risk is generated separately before truth
evaluation. All schedules, stopping conditions and gates are fixed beforehand.
"""
from datetime import datetime, timezone
from fractions import Fraction
from pathlib import Path
import argparse, hashlib, json, shutil

import numpy as np

import i62_metrics as metrics

CASES = ('m100_B','p030_B','p090_B')
ANCHORS = ((0.,.06),(.005,.075),(.03,.05))
C_LEVELS = (0.,.025,.05,.10,.20,.40)
FEATURE_KEYS = ('joint_load_risk_u','seam_risk_s','combined_risk_r','risk_valid',
    'observer_signal_valid','geometry_valid','seam_derivative_valid',
    'seam_signed_offset_m','seam_signed_rate_m_s','nearest_seam_distance_m',
    'seam_approach_rate_m_s','seam_approach_valid')


def clusters(data,mask):
    groups=[];current=[]
    for i in np.flatnonzero(mask):
        adjacent=(current and data['phase_index'][i]==data['phase_index'][current[-1]] and
            abs(data['time_s'][i]-data['time_s'][current[-1]]-.005)<=1e-9)
        if current and not adjacent:groups.append(current);current=[]
        current.append(int(i))
    if current:groups.append(current)
    return [dict(case=CASES[int(data['phase_index'][ids[0]])],start_s=float(data['time_s'][ids[0]]),
        end_s=float(data['time_s'][ids[-1]]),frame_count=len(ids)) for ids in groups]


def schedules():
    result=[]
    for anchor,(a,b) in enumerate(ANCHORS):
        for ci,c in enumerate(C_LEVELS):
            result.append(dict(name='anchor%d_c%d'%(anchor,ci),a=a,b=b,c=c,
                anchor_index=anchor,c_index=ci,eligible_for_fit=True))
    result.extend([dict(name='original_I61',a=.03,b=.10,c=0.,anchor_index=3,c_index=0,eligible_for_fit=False),
        dict(name='I62_minimum_labels',a=0.,b=.15,c=0.,anchor_index=4,c_index=0,eligible_for_fit=False)])
    return result


def keep(data,a,b,c,off=False):
    accepted=np.asarray(data['i60_accepted'],bool)
    if off:return accepted.copy()
    if not np.isfinite([a,b,c]).all() or min(a,b,c)<0:raise ValueError('Nonnegative finite coefficients required')
    if c==0:return metrics.keep_proposal(data,a,b)
    known=np.asarray(data['risk_valid'],bool)&np.isfinite(data['combined_risk_r'])
    threshold=a+(b*data['radius_m'])*data['force_norm_N']+c*np.where(known,data['combined_risk_r'],0.)
    valid=data['gate_valid']&np.isfinite(threshold)&(threshold>=0)&np.isfinite(data['joint_max_abs_Nm'])
    return accepted&(~known|(valid&(data['joint_max_abs_Nm']<=threshold)))


def combine(data_by_case):
    return metrics.domain([data_by_case[case] for case in data_by_case])


def merge_features(data,features):
    if not np.array_equal(data['time_s'],features['time_s']):raise ValueError('Condition/compact times differ')
    for key,value in features.items():
        if key=='time_s' or value.ndim<1 or len(value)!=len(data['time_s']):continue
        # Policy inputs stay byte-identical; condition normalizations may
        # reduce the same primitive along another axis/order.
        target='feature_'+key if key in data else key
        data[target]=value
    return data


def verify_controls(data_by_case):
    output={}
    for case,data in data_by_case.items():
        selected=data['primary_eligible'];point=selected&data['point_mask']
        old=keep(data,.03,.10,0.)
        if not np.array_equal(old,data['i61_keep_proposal']):raise ValueError('Inherited I61 keep flags differ')
        family=np.where(old,data['A_family'],data['N_family'])
        if not np.array_equal(family,data['i61_C_family']):raise ValueError('Inherited I61 family flags differ')
        distance=np.where(old,data['proposed_distance_m'],data['native_distance_m'])
        if not np.array_equal(distance[point],data['i61_distance_m'][point]):raise ValueError('Inherited I61 distances differ')
        domain=combine({case:data})
        output[case]=dict(I61_flags_exact=True,I61_families_exact=True,I61_distances_exact=True,
            I61=metrics.scalar_metrics(metrics.score_keeps(domain,old[selected])),
            I62_minimum_labels=metrics.scalar_metrics(metrics.score_keeps(domain,keep(data,0.,.15,0.)[selected])),
            baselines=metrics.constants(domain))
    return output


def point_unsafe(data):
    p=data['point_mask'];native_max=float(np.max(data['native_distance_m'][p])*1000.)
    A_distance=np.where(data['i60_accepted'],data['proposed_distance_m'],data['native_distance_m'])*1000.
    return data['i60_accepted']&p&(A_distance>native_max+1e-12),native_max


def exact_dominance(data,include_risk):
    """Necessary useful loss for rejecting all point-maximum violations.

    Positive-c unknown-risk rows retain A. c=0 controls are covered by the
    separate static bound. Exact Fraction products avoid sign-boundary guesses.
    """
    unsafe,native_max=point_unsafe(data)
    known=data['risk_valid']&np.isfinite(data['combined_risk_r'])
    useful=(data['N_family']!=data['truth_family'])&(data['A_family']==data['truth_family'])
    force_invalid=(~data['gate_valid'])|~np.isfinite(data['joint_max_abs_Nm'])
    force_invalid |= (~np.isfinite(data['radius_m']))|(~np.isfinite(data['force_norm_N']))
    force_invalid |= (data['radius_m']<=0)|(data['force_norm_N']<0)
    covered=known if include_risk else np.ones(len(known),bool)
    forced=useful&force_invalid&covered
    unknown_unsafe=unsafe&~known if include_risk else np.zeros(len(unsafe),bool)
    active=unsafe&~force_invalid&covered
    witnesses=[]
    frac=lambda v:Fraction.from_float(float(v))
    for i in np.flatnonzero(useful&~force_invalid&covered):
        mi=frac(data['joint_max_abs_Nm'][i]);Li=frac(data['radius_m'][i])*frac(data['force_norm_N'][i])
        ri=frac(data['combined_risk_r'][i]) if include_risk else Fraction(0)
        for k in np.flatnonzero(active):
            mk=frac(data['joint_max_abs_Nm'][k]);Lk=frac(data['radius_m'][k])*frac(data['force_norm_N'][k])
            rk=frac(data['combined_risk_r'][k]) if include_risk else Fraction(0)
            margins=(mi-mk,mi*Lk-mk*Li,mi*rk-mk*ri)
            if all(v>=0 for v in margins):
                forced[i]=True
                witnesses.append(dict(case=CASES[int(data['phase_index'][i])],time_s=float(data['time_s'][i]),
                    unsafe_case=CASES[int(data['phase_index'][k])],unsafe_time_s=float(data['time_s'][k]),
                    exact_signs_nonnegative=True,margins_as_float=[float(v) for v in margins]))
                break
    return dict(forced_mask=forced,report=dict(positive_c_class=bool(include_risk),
        native_maximum_mm=native_max,unsafe_count=int(unsafe.sum()),
        unsafe_unknown_risk_count=int(unknown_unsafe.sum()),
        forced_useful_loss_lower_bound=int(forced.sum()),
        useful_count=int(useful.sum()),
        harmful_count=int(np.sum((data['N_family']==data['truth_family'])&(data['A_family']!=data['truth_family']))),
        witnesses=witnesses))


def summarize(values,mask):
    values=np.asarray(values);finite=mask&np.isfinite(values);v=values[finite]
    return dict(count=int(mask.sum()),finite_count=int(finite.sum()),unknown_count=int(np.sum(mask&~np.isfinite(values))),
        minimum=None if not len(v) else float(v.min()),maximum=None if not len(v) else float(v.max()),
        mean=None if not len(v) else float(v.mean()),
        percentiles=None if not len(v) else {str(p):float(np.percentile(v,p)) for p in (10,25,50,75,90,95)})


def match_old_witnesses(actual,expected):
    # The source times are the same 5 ms records. Nine decimals retain ample
    # separation while allowing JSON's harmless decimal representation.
    key=lambda row:(row['case'],round(float(row['time_s']),9))
    a={key(row) for row in actual};b={key(row) for row in expected}
    if len(a)!=len(actual) or len(b)!=len(expected) or a!=b:
        raise ValueError('Pooled static witness identities differ from I62 certificate')
    return dict(exact_case_and_rounded_1ns_time_set_match=True,count=len(a))


def diagnostics(data,static_forced):
    accepted=data['i60_accepted'];Nwrong=data['N_family']!=data['truth_family'];Awrong=data['A_family']!=data['truth_family']
    unsafe,_=point_unsafe(data)
    groups=dict(all_primary=np.ones(len(accepted),bool),accepted=accepted,
        harmful=~Nwrong&Awrong,useful=Nwrong&~Awrong,accepted_wrong=accepted&Awrong,
        accepted_correct=accepted&~Awrong,static_forced_useful=static_forced,unsafe_point=unsafe,
        label_neutral_point_change=accepted&(data['N_family']==data['A_family'])&
            (data['native_distance_m']!=data['proposed_distance_m']))
    fields=('joint_load_risk_u','seam_risk_s','combined_risk_r','nearest_seam_distance_m','seam_approach_rate_m_s')
    fields += tuple(k for k in data if k not in fields and data[k].ndim==1 and
        ('innovation' in k or 'axial' in k or 'arclength' in k) and np.issubdtype(data[k].dtype,np.floating))
    output={name:{field:summarize(data[field],mask) for field in fields} for name,mask in groups.items()}
    comparisons={}
    # Higher risk is intended to excuse the load veto for useful hypotheses.
    # These pair ranks are descriptive, not independent trials or probabilities.
    for field in fields[:3]:
        value=data[field];good=value[groups['useful']&np.isfinite(value)]
        bad=value[groups['harmful']&np.isfinite(value)]
        delta=good[:,None]-bad[None,:]
        comparisons[field]=dict(useful_finite=len(good),harmful_finite=len(bad),pairs=int(delta.size),
            useful_higher_fraction=None if not delta.size else float(np.mean(delta>0)),
            tied_fraction=None if not delta.size else float(np.mean(delta==0)))
    events=[]
    event_rows=np.flatnonzero(groups['accepted_wrong']|unsafe)
    for i in event_rows:
        event=dict(case=CASES[int(data['phase_index'][i])],time_s=float(data['time_s'][i]),
            native_family=int(data['N_family'][i]),A_family=int(data['A_family'][i]),truth_family=int(data['truth_family'][i]),
            harmful=bool(groups['harmful'][i]),unsafe_point=bool(unsafe[i]),risk_valid=bool(data['risk_valid'][i]),
            joint_max_abs_Nm=float(data['joint_max_abs_Nm'][i]))
        event.update({field:float(data[field][i]) for field in fields});events.append(event)
    return dict(groups=output,ranks=comparisons,accepted_wrong_events=events,
        event_clusters={name:clusters(data,groups[name]) for name in ('harmful','useful','accepted_wrong','static_forced_useful','unsafe_point')})


def scan(data,options):
    choices=np.stack([keep(data,o['a'],o['b'],o['c']) for o in options])
    return metrics.score_keeps(data,choices)


def select(scores,feasible,options):
    ids=np.flatnonzero(feasible&np.array([o['eligible_for_fit'] for o in options],bool))
    if not len(ids):return dict(selection_failed=True,policy='off',schedule_index=None)
    order=np.lexsort((np.array([options[i]['c_index'] for i in ids]),np.array([options[i]['anchor_index'] for i in ids]),
        scores['veto_count'][ids],scores['point_SSE_mm2'][ids],scores['error_count'][ids]))
    i=int(ids[order[0]])
    return dict(selection_failed=False,policy='numeric',schedule_index=i,**options[i])


def selected_evaluation(data_by_case,selections,protocol):
    phase_scores={};bases={};chosen={}
    for case,data in data_by_case.items():
        selection=selections[case]
        chosen[case]=(data['i60_accepted'].copy() if selection['policy']=='off' else keep(data,selection['a'],selection['b'],selection['c']))
        domain=combine({case:data});phase_scores[case]=metrics.score_keeps(domain,chosen[case][data['primary_eligible']]);bases[case]=metrics.constants(domain)
    full=combine(data_by_case);choices=np.concatenate([chosen[c][data_by_case[c]['primary_eligible']] for c in data_by_case])
    aggregate=metrics.score_keeps(full,choices);base=metrics.constants(full)
    passed,flags=metrics.training_feasibility(aggregate,base,phase_scores,bases,protocol)
    return dict(metrics=metrics.scalar_metrics(aggregate),baseline=base,gates={k:bool(v[0]) for k,v in flags.items()},
        all_pooled_gates_pass=bool(passed[0]),per_case={c:dict(metrics=metrics.scalar_metrics(phase_scores[c]),baseline=bases[c],
        harmful_removal_scorable=bases[c]['A_harmful_count']>0) for c in data_by_case}),chosen


def fit(data_by_case,protocol,output):
    options=schedules();case_metrics={};case_bases={};arrays={}
    for case in data_by_case:
        domain=combine({case:data_by_case[case]});case_metrics[case]=scan(domain,options);case_bases[case]=metrics.constants(domain)
        arrays.update({case+'__'+k:v for k,v in case_metrics[case].items()})
    folds=[];selections={}
    for heldout in data_by_case:
        training={c:d for c,d in data_by_case.items() if c!=heldout};domain=combine(training)
        scores=scan(domain,options);base=metrics.constants(domain)
        feasible,flags=metrics.training_feasibility(scores,base,{c:case_metrics[c] for c in training},{c:case_bases[c] for c in training},protocol)
        feasible &= np.array([o['eligible_for_fit'] for o in options])
        selection=select(scores,feasible,options);selections[heldout]=selection
        prefix='train_without_'+heldout
        arrays.update({prefix+'__'+k:v for k,v in scores.items()});arrays[prefix+'__feasible']=feasible
        arrays.update({prefix+'__gate_'+k:v for k,v in flags.items()})
        folds.append(dict(heldout=heldout,training=list(training),qualified_count=int(feasible.sum()),selection=selection))
    domain=combine(data_by_case);scores=scan(domain,options);base=metrics.constants(domain)
    feasible,flags=metrics.training_feasibility(scores,base,case_metrics,case_bases,protocol)
    feasible &= np.array([o['eligible_for_fit'] for o in options]);selection=select(scores,feasible,options)
    arrays.update({'all__'+k:v for k,v in scores.items()});arrays['all__feasible']=feasible
    arrays.update({'all__gate_'+k:v for k,v in flags.items()})
    oof,oof_keep=selected_evaluation(data_by_case,selections,protocol)
    oof['all_folds_qualified']=all(not s['selection_failed'] for s in selections.values())
    oof['success']=oof['all_folds_qualified'] and oof['all_pooled_gates_pass']
    final,final_keep=selected_evaluation(data_by_case,{c:selection for c in data_by_case},protocol)
    final.update(selection=selection,qualified_count=int(feasible.sum()),descriptive_retrospective_only=True)
    for case in data_by_case:
        np.savez_compressed(output/(case+'_schedule_choices.npz'),time_s=data_by_case[case]['time_s'],oof_keep=oof_keep[case],full_fit_keep=final_keep[case])
    np.savez_compressed(output/'schedule_metrics.npz',**arrays)
    return dict(options=options,folds=folds,out_of_fold=oof,full_data_fit=final,
        eligible_for_new_native_episode=oof['success'] and final['all_pooled_gates_pass'] and not selection['selection_failed'])


def execute(data_by_case,protocol,output,old_witnesses):
    benchmarks=verify_controls(data_by_case)
    full=combine(data_by_case);static=exact_dominance(full,False);dynamic=exact_dominance(full,True)
    if static['report']['forced_useful_loss_lower_bound']!=99 or len(static['report']['witnesses'])!=99:
        raise ValueError('Pooled static bound differs from original I62 99-witness set')
    old_match=match_old_witnesses(static['report']['witnesses'],old_witnesses['witnesses'])
    reports=dict(full=dynamic['report']);groups={}
    for case,data in data_by_case.items():
        domain=combine({case:data});phase=int(data['phase_index'][0])
        old_tag=static['forced_mask'][full['phase_index']==phase]
        groups[case]=diagnostics(domain,old_tag)
    groups['all']=diagnostics(full,static['forced_mask'])
    for case in data_by_case:
        domain=combine({c:d for c,d in data_by_case.items() if c!=case});reports['train_without_'+case]=exact_dominance(domain,True)['report']
    stop=(dynamic['report']['unsafe_unknown_risk_count']>0 or
        dynamic['report']['forced_useful_loss_lower_bound']>dynamic['report']['harmful_count'])
    # Full-rate trace extraction is evaluation-only. Every risk value itself
    # was generated causally. Trace windows may display later observations.
    for case,data in data_by_case.items():
        primary=data['primary_eligible'];wrong=data['A_family']!=data['truth_family']
        harmful=primary&(data['N_family']==data['truth_family'])&wrong
        event=harmful.copy()
        centers=[dict(time_s=row['start_s'],reason='harmful_cluster_start') for row in clusters(data,harmful)]
        point=primary&data['point_mask']
        if point.any():
            distance=np.where(data['i60_accepted'],data['proposed_distance_m'],data['native_distance_m'])
            candidates=np.flatnonzero(point);worst=candidates[np.argmax(distance[point])];event[worst]=True
            centers.append(dict(time_s=float(data['time_s'][worst]),reason='phase_worst_A_point'))
        useful=primary&(data['N_family']!=data['truth_family'])&~wrong
        if useful.any():
            first=int(np.flatnonzero(useful)[0]);event[first]=True
            centers.append(dict(time_s=float(data['time_s'][first]),reason='first_useful'))
        for field in ('joint_load_risk_u','seam_risk_s'):
            selected=useful&np.isfinite(data[field])
            if selected.any():
                rows=np.flatnonzero(selected);peak=rows[np.argmax(data[field][selected])];event[peak]=True
                centers.append(dict(time_s=float(data['time_s'][peak]),reason='largest_useful_'+field))
        times=data['time_s'][event];trace=np.zeros(len(primary),bool)
        for t in times:trace |= abs(data['time_s']-t)<=.10+1e-9
        fields=('time_s','N_family','A_family','truth_family','primary_eligible','i60_accepted','gate_valid','joint_max_abs_Nm','radius_m','force_norm_N')+FEATURE_KEYS
        np.savez_compressed(output/(case+'_event_traces.npz'),**{key:data[key][trace] for key in fields})
        metrics.save(output/(case+'_event_centers.json'),centers)
        frame_fields={key:value[primary] for key,value in data.items() if key in fields or
            ('innovation' in key) or ('axial' in key) or ('arclength' in key)}
        frame_fields['old_forced_useful_I62']=static['forced_mask'][full['phase_index']==int(data['phase_index'][0])]
        np.savez_compressed(output/(case+'_diagnostic_frames.npz'),**frame_fields)
    result=dict(iteration='I-63',scope='retrospective diagnostics on previously inspected runs; no physics or live actuation',
        new_physics_steps=0,live_controller_changed=False,numpy_version=np.__version__,
        benchmarks=benchmarks,old_I62_witness_match=old_match,static_dominance=static['report'],dynamic_dominance=reports,diagnostics=groups,
        conditional_stop=bool(stop),conditional_stop_rule='unsafe positive-c unknown risk OR full dynamic forced useful loss > total harmful A',
        schedule_evaluation=None if stop else fit(data_by_case,protocol,output))
    metrics.save(output/'science.json',result)
    return result


def run(protocol_path,data_source,feature_source,output,old_witness_path):
    protocol=json.loads(protocol_path.read_text())
    if protocol['iteration']!='I-63':raise ValueError('Expected I-63')
    output.mkdir(parents=True,exist_ok=False)
    input_paths=[data_source/(c+'.npz') for c in CASES]+[feature_source/c/'conditions.npz' for c in CASES]+[old_witness_path]
    metrics.save(output/'registered_before_evaluation.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=metrics.sha(Path(__file__)),metrics_dependency_sha256=metrics.sha(Path(metrics.__file__)),
        protocol_sha256=metrics.sha(protocol_path),inputs={str(p):metrics.sha(p) for p in input_paths},
        status='before condition feature or truth array arithmetic in this evaluator'))
    shutil.copy2(Path(__file__),output/'condition_diagnostic.py');shutil.copy2(Path(metrics.__file__),output/'i62_metrics.py')
    data={}
    for phase,case in enumerate(CASES):
        d=metrics.load(data_source/(case+'.npz'));f=metrics.load(feature_source/case/'conditions.npz')
        for key in FEATURE_KEYS:
            if key not in f:raise ValueError('Missing condition feature '+key)
        d=merge_features(d,f)
        known=d['risk_valid'];risk=d['combined_risk_r']
        if np.any(known&(~np.isfinite(risk)|(risk<0)|(risk>1))):raise ValueError('Known risk must be finite in [0,1]')
        if not np.array_equal(risk[known],np.maximum(d['joint_load_risk_u'],d['seam_risk_s'])[known]):
            raise ValueError('Combined risk differs from frozen maximum')
        d['phase_index']=np.full(len(d['time_s']),phase,dtype=np.int64);data[case]=d
    result=execute(data,protocol,output,json.loads(old_witness_path.read_text()))
    metrics.save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=metrics.sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(conditional_stop=result['conditional_stop'],dynamic_forced_loss=result['dynamic_dominance']['full']['forced_useful_loss_lower_bound'],
        schedules_run=result['schedule_evaluation'] is not None,new_physics_steps=0)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('protocol','data-source','feature-source','output','old-witnesses'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run(a.protocol.resolve(),a.data_source.resolve(),a.feature_source.resolve(),a.output.resolve(),a.old_witnesses.resolve())
