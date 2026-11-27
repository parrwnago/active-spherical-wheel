"""I67 fixed-bank geometry and truth-assisted classification-capacity audit."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,numpy as np

CASES=('m100_B','p030_B','p090_B')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}


def original_active(packet):
    angle=packet['old__normal_angle_rad'];speed=packet['old__velocity_residual_norm_m_s'];moment=packet['old__moment_residual_norm'];vectors=packet['old__velocity_residual_m_s']
    finite=np.isfinite(angle)&np.isfinite(speed)&np.isfinite(moment)&np.isfinite(vectors).all(axis=2)&(angle>=0)&(speed>=0)&(moment>=0)
    active=packet['old__valid']&finite&packet['old__frame_valid'][:,None]&np.isfinite(packet['old__excitation_m_s'])[:,None]
    if np.any(active & ~np.isclose(np.linalg.norm(vectors,axis=2),speed,atol=1e-12,rtol=1e-10)):raise ValueError('Original speed/vector norm differs')
    return active


def original_costs(packet):
    components=np.stack(((packet['old__normal_angle_rad']/np.deg2rad(10.))**2,
                         packet['old__moment_residual_norm']**2,
                         (packet['old__velocity_residual_norm_m_s']/.02)**2),axis=2)
    total=components[:,:,0]+components[:,:,1]+components[:,:,2]
    return components,total


def minimum(values,mask):
    options=np.where(mask & np.isfinite(values),values,np.inf)
    index=np.argmin(options,axis=1);row=np.arange(len(index));value=options[row,index]
    known=np.isfinite(value);return np.where(known,index,-1),np.where(known,value,np.nan)


def gather(values,index):
    good=index>=0;out=np.full((len(index),)+values.shape[2:],np.nan)
    out[good]=values[np.arange(len(index))[good],index[good]]
    return out


def scores(residuals,packet):
    values=np.asarray(residuals,float);flat=values.reshape(-1,3)
    valid=np.isfinite(flat).all(axis=1);out=np.full(len(flat),np.nan)
    if np.any(valid):
        z=np.linalg.solve(packet['frozen_cholesky'],(flat[valid]-packet['frozen_mean']).T).T
        out[valid]=np.sum(z*z,axis=1)
    return out.reshape(values.shape[:-1])


def pair_capacity(correct_e,correct_score,wrong_e,wrong_score,available):
    """One test per existing point, before an existence reduction; no labels."""
    known=available & np.isfinite(correct_score)&np.isfinite(wrong_score[:,None])&np.isfinite(correct_e).all(axis=2)&np.isfinite(wrong_e).all(axis=1)[:,None]
    ambiguous=known & (np.linalg.norm(correct_e-wrong_e[:,None,:],axis=2)<=.01)
    epsilon=1e-9*np.maximum(1.,np.maximum(abs(correct_score),abs(wrong_score[:,None])))
    difference=correct_score-wrong_score[:,None]
    tied=known & ~ambiguous & (abs(difference)<=epsilon)
    win=known & ~ambiguous & (difference < -epsilon)
    return dict(known=known,ambiguous=ambiguous,tied=tied,win=win)


def analyse(packet,bank):
    n=len(packet['time_s']);rows=np.arange(n);family=bank['family_index'];truth=packet['truth_family']
    if not np.array_equal(packet['time_s'],bank['time_s']) or not np.array_equal(packet['source_index'],bank['source_index']):raise ValueError('ROI clocks differ')
    if not np.array_equal(family,packet['old__family_index']):raise ValueError('Original family layout changed')
    active=original_active(packet);components,total=original_costs(packet)
    reproduced_ids=[];reproduced_scores=[]
    for label in range(4):
        index,value=minimum(total,active & (family==label)[None,:]);reproduced_ids.append(index);reproduced_scores.append(value)
    ids=np.stack(reproduced_ids,axis=1);fused_scores=np.stack(reproduced_scores,axis=1)
    if not np.array_equal(ids,packet['old_fused__candidate_index']):raise ValueError('Original fused per-family selections changed')
    old=packet['old_fused__family_scores'];known=np.isfinite(old)
    if not np.array_equal(np.isfinite(fused_scores),known) or (known.any() and np.max(abs(fused_scores[known]-old[known]))>1e-10):raise ValueError('Original fused scores changed')
    target=packet['sampled_reference_point_m'];points=bank['points_m']
    distance=np.linalg.norm(points-target[:,None,:],axis=2)
    same=(family[None,:]==truth[:,None])&(truth>=0)[:,None]
    usable=active & same & bank['reconstructed']
    raw=same & bank['reconstructed']
    nearest_id,nearest_distance=minimum(distance,usable)
    raw_id,raw_distance=minimum(distance,raw)
    branch=packet['correct_branch']
    selected_id=np.where(branch==0,packet['saved_F_index'],np.where(branch==1,packet['saved_Nalt_index'],-1))
    safe=np.maximum(selected_id,0)
    selected_member=(selected_id>=0)&usable[rows,safe]
    opportunity=packet['cohort_useful']|packet['cohort_harmful']
    if not np.all(selected_member[opportunity]):raise ValueError('Original correct-family selected point not in usable bank')
    oldpoint=np.where((branch==0)[:,None],packet['saved_point_F_m'],packet['saved_point_Nalt_m'])
    selected_point=gather(points,selected_id)
    valid=selected_member & np.isfinite(oldpoint).all(axis=1)
    point_difference=float(np.max(abs(selected_point[valid]-oldpoint[valid]))) if valid.any() else 0.
    if point_difference>1e-12:raise ValueError('Original selected coordinates changed')
    force=packet['observer_interval_mean'][:,:3];rawjoint=packet['observer_interval_mean'][:,6:9]
    L=packet['linear_at_centre'][:,family];A=packet['angular_world'][:,family]
    prediction=np.einsum('ncij,nj->nci',L,force)+np.einsum('ncij,ncj->nci',A,np.cross(points-packet['centre_m'][:,None,:],force[:,None,:]))
    residual=rawjoint[:,None,:]-prediction
    e=residual/packet['fixed_scale_Nm'][:,None,None]
    score=np.full(e.shape[:-1],np.nan)
    comparable=branch>=0
    score[comparable]=scores(e[comparable],packet)
    minscore_id,minscore=minimum(score,usable)
    reference_prediction=rawjoint-packet['reference_residual_Nm']
    torque_distance=np.linalg.norm(prediction-reference_prediction[:,None,:],axis=2)
    torque_distance[~comparable]=np.nan
    torque_id,torque_gap=minimum(torque_distance,usable)
    selected_e=gather(e,selected_id)
    old_selected_e=np.where((branch==0)[:,None],packet['old_I65_e_F'],packet['old_I65_e_Nalt'])
    if opportunity.any() and (not np.isfinite(selected_e[opportunity]).all() or not np.isfinite(old_selected_e[opportunity]).all() or np.max(abs(selected_e[opportunity]-old_selected_e[opportunity]))>1e-10):raise ValueError('Original selected load residual changed')
    old_wrong_e=np.where((branch==0)[:,None],packet['old_I65_e_Nalt'],packet['old_I65_e_F'])
    old_wrong_e[~opportunity]=np.nan
    wrong_score=scores(old_wrong_e,packet)
    capacity=pair_capacity(e,score,old_wrong_e,wrong_score,usable & opportunity[:,None])
    any_win=np.any(capacity['win'],axis=1)
    best_win_id,best_win_score=minimum(score,capacity['win'])
    baseline_correct=opportunity & (packet['old_I65_code']==np.where(branch==0,-1,1))
    selected_correct=capacity['win'][rows,safe] & (selected_id>=0)
    selected_code=np.full(n,3,np.int8);valid_selected=opportunity & (selected_id>=0)
    selected_known=capacity['known'][rows,safe]&valid_selected
    selected_ambiguous=capacity['ambiguous'][rows,safe]&valid_selected
    selected_tied=capacity['tied'][rows,safe]&valid_selected
    selected_code[selected_ambiguous]=2;selected_code[selected_tied]=0
    resolved=selected_known & ~selected_ambiguous & ~selected_tied
    desired=np.where(branch==0,-1,1)
    selected_code[resolved]=np.where(selected_correct[resolved],desired[resolved],-desired[resolved])
    if not np.array_equal(selected_code[opportunity],packet['old_I65_code'][opportunity]):raise ValueError('Original I65 classification codes not reproduced')
    if np.any(baseline_correct & ~any_win):raise ValueError('Existence capacity violates selected-member inclusion')
    reference_correct=opportunity & (packet['reference_cf_code']==np.where(branch==0,-1,1))
    reference_score=np.where(branch==0,packet['reference_cf_F_score'],packet['reference_cf_Nalt_score'])
    selected_distance=gather(distance,selected_id);selected_score=gather(score,selected_id)
    # Only family-member selections are meaningful for the geometry gap.
    selected_distance[~selected_member]=np.nan;selected_score[~selected_member]=np.nan
    if np.any(selected_member & (nearest_distance>selected_distance+1e-12)):raise ValueError('Nearest geometry is worse than selected member')
    if np.any(selected_member & (minscore>selected_score+1e-7)):raise ValueError('Minimum score worse than selected member')
    output=dict(time_s=packet['time_s'],source_index=packet['source_index'],truth_family=truth,event_anchor=packet['event_anchor'],
        opportunity=opportunity,selected_id=selected_id,selected_member=selected_member,nearest_id=nearest_id,pre_admission_nearest_id=raw_id,
        minscore_id=minscore_id,nearest_load_reference_id=torque_id,best_success_id=best_win_id,
        selected_distance_m=selected_distance,nearest_distance_m=nearest_distance,pre_admission_distance_m=raw_distance,
        reference_radial_error_m=np.linalg.norm(target-packet['centre_m'],axis=1)-float(packet['radius_m']),
        geometric_selection_gap_m=selected_distance-nearest_distance,geometric_admission_gap_m=nearest_distance-raw_distance,
        selected_load_score=selected_score,min_load_score=minscore,reference_load_score=reference_score,
        load_selection_gap=selected_score-minscore,min_minus_reference_score=minscore-reference_score,
        minimum_reference_torque_distance_Nm=torque_gap,nearest_geometry_load_score=gather(score,nearest_id),
        minimum_score_point_distance_m=gather(distance,minscore_id),wrong_load_score=wrong_score,
        baseline_correct=baseline_correct,selected_original_code=selected_code,reference_correct=reference_correct,any_bank_correct=any_win,
        nearest_geometry_correct=(capacity['win'][rows,np.maximum(nearest_id,0)] & (nearest_id>=0)),
        minscore_correct=(capacity['win'][rows,np.maximum(minscore_id,0)] & (minscore_id>=0)),
        usable_correct_count=np.sum(usable,axis=1),pre_admission_correct_count=np.sum(raw,axis=1),
        nearest_exact_tie_count=np.sum(usable & (distance==nearest_distance[:,None]),axis=1),
        nearest_near_tie_count=np.sum(usable & (abs(distance-nearest_distance[:,None])<=1e-12),axis=1),
        minscore_exact_tie_count=np.sum(usable & (score==minscore[:,None]),axis=1),
        minscore_near_tie_count=np.sum(usable & (abs(score-minscore[:,None])<=1e-9*np.maximum(1.,abs(minscore[:,None]))),axis=1))
    for label,index in (('selected',selected_id),('nearest_geometry',nearest_id),('minimum_load',minscore_id),('best_success',best_win_id)):
        output[label+'_fused_cost']=gather(total,index)
        output[label+'_cost_components']=gather(components,index)
        output[label+'_point_m']=gather(points,index)
        output[label+'_residual_Nm']=gather(residual,index)
    for name in ('focus','useful','harmful','retained_useful'):output['cohort_'+name]=packet['cohort_'+name]
    verification=dict(original_fused_IDs_exact=True,original_I65_codes_exact_on_opportunities=True,selected_membership=True,
        original_selected_point_maximum_difference_m=point_difference,original_fused_score_maximum_difference=float(np.max(abs(fused_scores[known]-old[known]))) if known.any() else None,
        existence_nonregression_is_inclusion_QA=True)
    return output,verification


def scalar(value,mask,factor=1.):
    v=value[mask & np.isfinite(value)]*factor
    return dict(known=len(v),median=None if not len(v) else float(np.median(v)),p95=None if not len(v) else float(np.percentile(v,95)),maximum=None if not len(v) else float(np.max(v)))


def report_group(a,mask):
    reference_new=mask & a['reference_correct'] & ~a['baseline_correct']
    return dict(denominator=int(mask.sum()),known_bank=int(np.sum(mask & (a['usable_correct_count']>0))),
        original_correct=int(np.sum(mask & a['baseline_correct'])),sampled_reference_correct=int(np.sum(mask & a['reference_correct'])),
        any_existing_candidate_correct=int(np.sum(mask & a['any_bank_correct'])),nearest_geometry_correct=int(np.sum(mask & a['nearest_geometry_correct'])),minimum_score_correct=int(np.sum(mask & a['minscore_correct'])),
        newly_correct_reference_denominator=int(reference_new.sum()),reference_repairs_supported=int(np.sum(reference_new & a['any_bank_correct'])),reference_repairs_unsupported=int(np.sum(reference_new & ~a['any_bank_correct'])),
        capacity_matrix=dict(reference_yes_bank_yes=int(np.sum(mask & a['reference_correct'] & a['any_bank_correct'])),reference_yes_bank_no=int(np.sum(mask & a['reference_correct'] & ~a['any_bank_correct'])),
            reference_no_bank_yes=int(np.sum(mask & ~a['reference_correct'] & a['any_bank_correct'])),reference_no_bank_no=int(np.sum(mask & ~a['reference_correct'] & ~a['any_bank_correct']))),
        geometry={k:scalar(a[k],mask,1000.) for k in ('selected_distance_m','nearest_distance_m','pre_admission_distance_m','geometric_selection_gap_m','geometric_admission_gap_m','reference_radial_error_m')},
        load={k:scalar(a[k],mask) for k in ('selected_load_score','min_load_score','reference_load_score','load_selection_gap','min_minus_reference_score','minimum_reference_torque_distance_Nm','nearest_geometry_load_score')},
        minimum_score_point_distance_mm=scalar(a['minimum_score_point_distance_m'],mask,1000.))


def run(protocol_path,packets_root,banks_root,output):
    protocol=read(protocol_path)
    if protocol['iteration']!='I-67':raise ValueError('Wrong protocol')
    seal=read(protocol_path.with_name('analysis_input_seal.json'))
    roots=dict(experiment=protocol_path.parent.parent,packets=packets_root,banks=banks_root)
    for item in seal['files']:
        if sha(roots[item['root']]/item['path'])!=item['sha256']:raise ValueError('Sealed input/source changed')
    output.mkdir(parents=True,exist_ok=False)
    write(output/'registered_before_capacity_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),protocol_sha256=sha(protocol_path),seal_sha256=sha(protocol_path.with_name('analysis_input_seal.json')),numpy_version=np.__version__,new_fit=False,new_physics=False))
    manifest=read(packets_root/'manifest.json');all_arrays={};report=dict(iteration='I-67',numpy_version=np.__version__,protocol_sha256=sha(protocol_path),per_case={},events=[])
    for case in CASES:
        packet=load(packets_root/(case+'.npz'));bank=load(banks_root/(case+'.npz'))
        a,verification=analyse(packet,bank);all_arrays[case]=a
        report['per_case'][case]=dict(verification=verification,cohorts={name:report_group(a,a['cohort_'+name]) for name in ('focus','useful','harmful','retained_useful')})
        for event in manifest['cases'][case]['events']:
            where=np.flatnonzero(a['source_index']==event['index'])
            if len(where)!=1:raise ValueError('Missing fixed event')
            i=int(where[0]);entry=dict(case=case,**event)
            entry.update({k:v[i].tolist() if isinstance(v[i],np.ndarray) else v[i].item() for k,v in a.items() if k not in ('time_s','source_index')})
            report['events'].append(entry)
        np.savez_compressed(output/(case+'_frames.npz'),**a)
    merged={k:np.concatenate([d[k] for d in all_arrays.values()]) for k in next(iter(all_arrays.values()))}
    report['cohorts']={name:report_group(merged,merged['cohort_'+name]) for name in ('focus','useful','harmful','retained_useful')}
    unsupported=[]
    for case,a in all_arrays.items():
        mask=(a['cohort_focus']|a['cohort_harmful']) & a['reference_correct'] & ~a['baseline_correct'] & ~a['any_bank_correct']
        unsupported += [dict(case=case,source_index=int(a['source_index'][i]),time_s=float(a['time_s'][i]),truth_family=int(a['truth_family'][i])) for i in np.flatnonzero(mask)]
    report['unsupported_reference_repairs']=unsupported
    count=sum(report['cohorts'][g]['newly_correct_reference_denominator'] for g in ('focus','harmful'))
    report['primary_capacity']=dict(reference_repaired_denominator=count,supported=count-len(unsupported),unsupported=len(unsupported),all_reference_repairs_reproducible=bool(count==29 and not unsupported),
        qualification='Truth-assisted existence in frozen bank, not a new selector or deployable improvement. Original inclusion guarantees baseline nonregression; that invariant is not scientific success.')
    report['decision']='Keep live baseline and original bank/rules unchanged. Interpret capacity, geometry and original fused objective separately; no automatic bank expansion or score retuning.'
    def safe(x):
        if isinstance(x,dict):return {k:safe(v) for k,v in x.items()}
        if isinstance(x,list):return [safe(v) for v in x]
        if isinstance(x,float) and not np.isfinite(x):return None
        return x
    write(output/'science.json',safe(report))
    bindings={}
    for p in sorted(output.glob('*.npz')):
        z=load(p);bindings[p.name]={k:dict(dtype=v.dtype.str,shape=list(v.shape),sha256=hashlib.sha256(v.tobytes()).hexdigest()) for k,v in z.items()}
    write(output/'array_bindings.json',bindings)
    print(json.dumps(dict(primary_capacity=report['primary_capacity'],focus=report['cohorts']['focus'],harmful=report['cohorts']['harmful'])),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('protocol','packets-root','banks-root','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run(a.protocol.resolve(),a.packets_root.resolve(),a.banks_root.resolve(),a.output.resolve())
