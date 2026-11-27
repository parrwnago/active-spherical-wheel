"""I-66 evaluation-only geometry and load-representation audit, no physics."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json
import numpy as np

CASES=('m100_B','p030_B','p090_B')
ROUTES=('C_candidate','T_sampled_point','W_single_family_wrench','S_normal_share_proxy')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def point_prediction(L,A,centre,point,force):
    return np.einsum('nij,nj->ni',L,force)+np.einsum('nij,nj->ni',A,np.cross(point-centre,force))


def choose_correct_branch(F_family,N_family,truth_family,F_valid,N_valid):
    out=np.full(len(truth_family),-1,np.int8)
    F=F_valid & (F_family==truth_family) & (truth_family>=0)
    N=~F & N_valid & (N_family==truth_family) & (truth_family>=0)
    out[F]=0;out[N]=1
    return out


def split_prediction(L,A,centre,points,loads,force,available):
    total=np.sum(loads,axis=1)
    shares=np.divide(loads,total[:,None],out=np.zeros_like(loads),where=total[:,None]>0)
    positive=shares>0
    valid=(available & np.isfinite(loads).all(axis=1) & (loads>=0).all(axis=1)
           & np.isfinite(total) & (total>=1.)
           & np.all(~positive | np.isfinite(points).all(axis=2),axis=1))
    clean=np.where(positive[:,:,None],points,centre[:,None,:])
    allocated=shares[:,:,None]*force[:,None,:]
    pred=np.sum(np.einsum('nkij,nkj->nki',L,allocated)
                +np.einsum('nkij,nkj->nki',A,np.cross(clean-centre[:,None,:],allocated)),axis=1)
    pred[~valid]=np.nan
    return pred,valid,shares


def routes(pose,primitive,truth,base):
    t=base['time_s'];n=len(t);rows=np.arange(n)
    for time in (pose['time_s'],primitive['time_s'],truth['truth_time_s']):
        if not np.array_equal(t,time):raise ValueError('Input timestamps differ')
    raw=primitive['observer_interval_mean'];filtered=primitive['observer_generalized']
    force=raw[:,:3];moment=raw[:,3:6];target=raw[:,6:9]
    observed=np.isfinite(raw).all(axis=1)&np.isfinite(filtered).all(axis=1)&(np.linalg.norm(filtered[:,:3],axis=1)>=1.)
    available=pose['valid'] & observed
    family=base['truth_family'];safe=np.maximum(family,0)
    centre=pose['centre_m'];L=pose['linear_at_centre'];A=pose['angular_world']
    Lk=L[rows,safe];Ak=A[rows,safe]
    reference=truth['truth_patch_points_m'][rows,safe].copy();reference[family<0]=np.nan
    Tvalid=available & (family>=0) & np.isfinite(reference).all(axis=1)
    predT=point_prediction(Lk,Ak,centre,reference,force);predT[~Tvalid]=np.nan
    predW=np.einsum('nij,nj->ni',Lk,force)+np.einsum('nij,nj->ni',Ak,moment)
    Wvalid=available & (family>=0);predW[~Wvalid]=np.nan
    predS,Svalid,shares=split_prediction(L,A,centre,truth['truth_patch_points_m'],truth['truth_patch_loads_N'],force,available)
    Fvalid=available & primitive['H_F_valid'];Nvalid=available & primitive['H_Nalt_valid']
    branch=choose_correct_branch(primitive['proposed_family'],primitive['native_family'],family,Fvalid,Nvalid)
    Cvalid=branch>=0
    pointC=np.where((branch==0)[:,None],primitive['point_F_m'],primitive['point_Nalt_m']).copy();pointC[~Cvalid]=np.nan
    H=np.where((branch==0)[:,None,None],primitive['H_F'],primitive['H_Nalt'])
    predC=np.einsum('nij,nj->ni',H,force);predC[~Cvalid]=np.nan
    output=dict(time_s=t,correct_branch=branch,reference_point_m=reference,candidate_point_m=pointC,
                fixed_scale_Nm=.03+(.10*base['radius_m'])*base['force_norm_N'],normal_shares=shares)
    for name,pred,valid in zip(ROUTES,(predC,predT,predW,predS),(Cvalid,Tvalid,Wvalid,Svalid)):
        output[name+'_prediction_Nm']=pred
        output[name+'_residual_Nm']=target-pred
        output[name+'_valid']=valid & np.isfinite(pred).all(axis=1)
        output[name+'_compatible']=output[name+'_valid'] & (np.max(abs(target-pred),axis=1)<=output['fixed_scale_Nm'])
    delta=predT-predC;eC=target-predC;eT=target-predT
    free=moment-np.cross(reference-centre,force)
    free_joint=np.einsum('nij,nj->ni',Ak,free)
    output.update(geometry_load_delta_Nm=delta,geometry_identity_error_Nm=eC-eT-delta,
        SSE_gain_Nm2=np.sum(eC*eC-eT*eT,axis=1),
        SSE_gain_identity_error_Nm2=np.sum(eC*eC-eT*eT,axis=1)-(2*np.sum(eC*delta,axis=1)-np.sum(delta*delta,axis=1)),
        free_couple_Nm=free,free_couple_joint_Nm=free_joint,
        free_couple_identity_error_Nm=eT-output['W_single_family_wrench_residual_Nm']-free_joint,
        point_delta_m=reference-pointC,reference_radial_error_m=np.linalg.norm(reference-centre,axis=1)-float(pose['radius_m']))
    distance=np.linalg.norm(reference-pointC,axis=1)
    force_norm=np.linalg.norm(force,axis=1)
    unit=np.divide(force,force_norm[:,None],out=np.full_like(force,np.nan),where=force_norm[:,None]>0)
    parallel=np.sum((reference-pointC)*unit,axis=1)
    output['point_distance_m']=distance
    output['point_parallel_m']=abs(parallel)
    output['point_perpendicular_m']=np.sqrt(np.maximum(0.,distance*distance-parallel*parallel))
    axial=np.sum(pose['patch_axis_world'][rows,safe]*(reference-centre),axis=1)
    output['reference_material_band_valid']=Tvalid & (abs(output['reference_radial_error_m'])<=1e-9) & (axial>=pose['patch_lower_m'][safe]-1e-9) & (axial<=pose['patch_upper_m'][safe]+1e-9)
    for label,valid in (('F',Fvalid),('Nalt',Nvalid)):
        residual=target-np.einsum('nij,nj->ni',primitive['H_'+label],force)
        output[label+'_original_residual_Nm']=residual
        output[label+'_original_valid']=valid & np.isfinite(residual).all(axis=1)
    return output


def stable_truth(family):
    out=np.zeros(len(family),bool)
    for i in range(2,len(family)-2):out[i]=(family[i]>=0 and np.all(family[i-2:i+3]==family[i]))
    return out


def cohorts(base,arrays,oof):
    selected=base['primary_eligible'];Nwrong=base['N_family']!=base['truth_family'];Awrong=base['A_family']!=base['truth_family']
    useful=selected & Nwrong & ~Awrong;harmful=selected & ~Nwrong & Awrong
    keep=oof['M1__shared_covariance_keep'];correct=selected & base['i60_accepted'] & (arrays['correct_branch']>=0)
    return dict(focus=useful & ~keep,useful=useful,harmful=harmful,retained_useful=useful & keep,
                correct_pair=correct,stable_correct_pair=correct & stable_truth(base['truth_family']),full_reference=selected)


def vector_stats(value,mask,unit='Nm'):
    mask=mask & np.isfinite(value).all(axis=1);v=value[mask]
    if not len(v):result=dict(count=0,vector_RMS_Nm=None,vector_p95_Nm=None,vector_max_Nm=None,per_joint_RMS_Nm=None)
    else:
        norm=np.linalg.norm(v,axis=1)
        result=dict(count=len(v),vector_RMS_Nm=float(np.sqrt(np.mean(np.sum(v*v,axis=1)))),
                    vector_p95_Nm=float(np.percentile(norm,95)),vector_max_Nm=float(norm.max()),per_joint_RMS_Nm=np.sqrt(np.mean(v*v,axis=0)).tolist())
    return result if unit=='Nm' else {k.replace('_Nm','_'+unit).replace('per_joint','per_component'):v for k,v in result.items()}


def scalar_stats(value,mask,scale=1.):
    v=value[mask & np.isfinite(value)]*scale
    return dict(count=len(v),median=None if not len(v) else float(np.median(v)),
                RMS=None if not len(v) else float(np.sqrt(np.mean(v*v))),p95=None if not len(v) else float(np.percentile(v,95)),maximum=None if not len(v) else float(v.max()))


def describe(arrays,mask):
    paired=mask & arrays['C_candidate_valid'] & arrays['T_sampled_point_valid']
    C=arrays['C_candidate_residual_Nm'];T=arrays['T_sampled_point_residual_Nm']
    nrC=np.linalg.norm(C,axis=1);nrT=np.linalg.norm(T,axis=1);effect=np.linalg.norm(arrays['geometry_load_delta_Nm'],axis=1)
    eps=1e-10
    out=dict(denominator=int(mask.sum()),paired_count=int(paired.sum()),unknown_pair=int(np.sum(mask & ~paired)),
        paired_C=vector_stats(C,paired),paired_T=vector_stats(T,paired),routes={})
    for route in ROUTES:
        known=mask & arrays[route+'_valid']
        out['routes'][route]=dict(**vector_stats(arrays[route+'_residual_Nm'],known),
            compatible_count=int(np.sum(mask & arrays[route+'_compatible'])),unknown=int(np.sum(mask & ~arrays[route+'_valid'])))
    validC=arrays['C_candidate_compatible'];validT=arrays['T_sampled_point_compatible']
    out['point_replacement']=dict(vector_error_improves=int(np.sum(paired & (nrT<nrC-eps))),
        vector_error_worsens=int(np.sum(paired & (nrT>nrC+eps))),vector_error_unchanged=int(np.sum(paired & (abs(nrT-nrC)<=eps))),
        C_fail_T_pass=int(np.sum(paired & ~validC & validT)),C_fail_T_fail=int(np.sum(paired & ~validC & ~validT)),
        C_pass_T_fail=int(np.sum(paired & validC & ~validT)),C_pass_T_pass=int(np.sum(paired & validC & validT)))
    nonzero=paired & (nrC>1e-12)
    ratio=np.divide(effect,nrC,out=np.full(len(nrC),np.nan),where=nrC>1e-12)
    out['geometry']=dict(point_distance_mm=scalar_stats(arrays['point_distance_m'],paired,1000.),
        point_perpendicular_to_F_mm=scalar_stats(arrays['point_perpendicular_m'],paired,1000.),
        lever_load_effect_Nm=vector_stats(arrays['geometry_load_delta_Nm'],paired),
        lever_effect_over_C_error=scalar_stats(ratio,nonzero),
        reference_material_point_count=int(np.sum(mask & arrays['reference_material_band_valid'])),
        reference_radial_error_mm=scalar_stats(arrays['reference_radial_error_m'],mask,1000.))
    out['free_couple']=dict(world_Nm=vector_stats(arrays['free_couple_Nm'],mask),joint_projection_Nm=vector_stats(arrays['free_couple_joint_Nm'],mask))
    return out


def counterfactual(arrays,base,model,score_method,previous_protocol):
    scale=arrays['fixed_scale_Nm']
    F=arrays['F_original_residual_Nm']/scale[:,None];N=arrays['Nalt_original_residual_Nm']/scale[:,None]
    validF=arrays['F_original_valid'];validN=arrays['Nalt_original_valid']
    opportunity=base['primary_eligible'] & ((base['N_family']!=base['truth_family']) != (base['A_family']!=base['truth_family']))
    original=score_method.score_pair(F[opportunity],N[opportunity],model,pattern_epsilon=previous_protocol['comparison']['pattern_epsilon_dimensionless'],valid_F=validF[opportunity],valid_N=validN[opportunity])
    # Replace exactly the known-correct family branch. No calibration fitting.
    CF=F.copy();CN=N.copy();vF=validF.copy();vN=validN.copy()
    branch=arrays['correct_branch'];replacement=arrays['T_sampled_point_residual_Nm']/scale[:,None]
    takeF=opportunity & (branch==0);takeN=opportunity & (branch==1)
    CF[takeF]=replacement[takeF];CN[takeN]=replacement[takeN]
    vF[takeF]=arrays['T_sampled_point_valid'][takeF];vN[takeN]=arrays['T_sampled_point_valid'][takeN]
    changed=score_method.score_pair(CF[opportunity],CN[opportunity],model,pattern_epsilon=previous_protocol['comparison']['pattern_epsilon_dimensionless'],valid_F=vF[opportunity],valid_N=vN[opportunity])
    output={'applicable':opportunity}
    for label,pair in (('original',original),('sampled_correct_point',changed)):
        selected=np.flatnonzero(opportunity);code=np.full(len(scale),3,np.int8)
        code[selected[pair['ambiguous']]]=2;code[selected[pair['tie']]]=0;code[selected[pair['F_lower']]]=-1;code[selected[pair['N_lower']]]=1
        scoreF=np.full(len(scale),np.nan);scoreN=scoreF.copy();known=np.zeros(len(scale),bool)
        scoreF[opportunity]=pair['score_F'];scoreN[opportunity]=pair['score_N'];known[opportunity]=pair['valid_pair']
        output[label+'_code']=code;output[label+'_F_score']=scoreF;output[label+'_Nalt_score']=scoreN;output[label+'_valid']=known
    return output


def counterfactual_report(arrays,masks):
    result={}
    for cohort,desired in (('useful',-1),('harmful',1),('focus',-1)):
        mask=masks[cohort];old=arrays['cf_original_code'];new=arrays['cf_sampled_correct_point_code']
        result[cohort]=dict(denominator=int(mask.sum()),original_correct=int(np.sum(mask & (old==desired))),
            replacement_correct=int(np.sum(mask & (new==desired))),
            newly_correct=int(np.sum(mask & (old!=desired) & (new==desired))),
            newly_wrong=int(np.sum(mask & (old==desired) & (new!=desired))),
            unknown=int(np.sum(mask & (new==3))),ambiguous=int(np.sum(mask & (new==2))),tied=int(np.sum(mask & (new==0))))
    return result


def qualify(summary,percase,arrays,masks,protocol):
    c=protocol['primary_comparison'];s=summary['focus'];old=s['paired_C'];new=s['paired_T'];eps=c['roundoff_Nm']
    coverage=s['paired_count']/s['denominator'] if s['denominator'] else 0.
    flags=dict(coverage=coverage>=c['coverage_min'],all_focus_known=s['paired_count']==s['denominator'],
        all_harmful_known=summary['harmful']['paired_count']==summary['harmful']['denominator'],
        RMS_reduction=bool(old['count'] and new['vector_RMS_Nm']<=(1.-c['minimum_vector_RMS_reduction_fraction'])*old['vector_RMS_Nm']+eps),
        p95_nonincrease=bool(old['count'] and new['vector_p95_Nm']<=old['vector_p95_Nm']+eps))
    for case,r in percase.items():
        x=r['cohorts']['focus'];flags[case+'_RMS_nonincrease']=None if not x['denominator'] else bool(x['paired_count'] and x['paired_T']['vector_RMS_Nm']<=x['paired_C']['vector_RMS_Nm']+eps)
    return dict(gates=flags,qualified_geometric_improvement=all(v for v in flags.values() if v is not None),qualification='Sampled-reference sensitivity only, not causal proof or deployment qualification. Zero-focus phases are unscorable/null.')


def run(protocol_path,pose_root,i61,i64,i65,context_root,output):
    protocol=read(protocol_path);oldscience=read(i65/'analysis/stage01/science.json');oldprotocol=read(i65/'protocol/protocol.json')
    if protocol['iteration']!='I-66':raise ValueError('Wrong iteration')
    seal=read(protocol_path.with_name('analysis_input_seal.json'))
    roots=dict(experiment=protocol_path.parent.parent,i61=i61,i64=i64,i65=i65,pose=pose_root,context=context_root)
    for row in seal['files']:
        if sha(roots[row['root']]/row['path'])!=row['sha256']:raise ValueError('Sealed analysis source/input changed: '+row['path'])
    output.mkdir(parents=True,exist_ok=False)
    write(output/'registered_before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),protocol_sha256=sha(protocol_path),source_sha256=sha(__file__),
        qualification='Evaluation-only truth-assisted models; no fitting, actuation, new physics or true interval contact reconstruction.'))
    score_method=module(i65/'method/covariance_score.py','_i65_frozen_score')
    all_arrays={};all_masks={};bases={};report=dict(iteration='I-66',protocol_sha256=sha(protocol_path),numpy_version=np.__version__,per_case={},events=[])
    for case in CASES:
        pose=load(pose_root/case/'pose_maps.npz');primitive=load(i64/'features/portable_stage01'/case/'primitives.npz')
        truth=load(i61/'analysis/evaluation_compact_stage01'/f'{case}_truth_projection.npz')
        base=load(i64/'portable_inputs/i62_base'/f'{case}.npz');oof=load(i65/'analysis/stage01'/f'{case}_oof.npz')
        context=load(context_root/case/'endpoint_context.npz')
        a=routes(pose,primitive,truth,base);m=cohorts(base,a,oof);bases[case]=base
        for branch in ('F','Nalt'):
            known=a[branch+'_original_valid'];error=a[branch+'_original_residual_Nm'][known]/a['fixed_scale_Nm'][known,None]-oof['M1__e_'+branch][known]
            if np.max(abs(error))>1e-10:raise ValueError('Original I65 residual differs')
        model={k:np.asarray(v,float) if k in ('mean','covariance','cholesky') else v for k,v in oldscience['models'][case]['M1']['fitted'].items()}
        cf=counterfactual(a,base,model,score_method,oldprotocol)
        a.update({'cf_'+k:v for k,v in cf.items()})
        relevant=m['useful']|m['harmful']
        if not np.array_equal(a['cf_original_code'][relevant],oof['M1__shared_covariance_code'][relevant]):raise ValueError('Frozen I65 original decisions changed')
        a['endpoint_force_difference_N']=primitive['observer_interval_mean'][:,:3]-context['contact_force_N']
        a['endpoint_moment_difference_Nm']=primitive['observer_interval_mean'][:,3:6]-context['contact_moment_Nm']
        for k in ('fork_load_N','push_N'):a[k]=context[k]
        for k in ('geometry_identity_error_Nm','free_couple_identity_error_Nm','SSE_gain_identity_error_Nm2'):
            values=a[k];finite=np.isfinite(values);bound=np.max(abs(values[finite])) if finite.any() else 0.
            if bound>1e-10:raise ValueError('Load decomposition does not match: '+k)
        row=dict(cohorts={name:describe(a,mask) for name,mask in m.items()},counterfactual=counterfactual_report(a,m),
                 identity_maximum={k:float(np.nanmax(abs(a[k]))) for k in ('geometry_identity_error_Nm','free_couple_identity_error_Nm','SSE_gain_identity_error_Nm2')})
        row['endpoint_context']={name:dict(raw_observer_minus_sampled_force_N=vector_stats(a['endpoint_force_difference_N'],mask,'N'),
            raw_observer_minus_sampled_moment_Nm=vector_stats(a['endpoint_moment_difference_Nm'],mask),
            fork_load_N=scalar_stats(a['fork_load_N'],mask),push_frames=int(np.sum(mask & (np.linalg.norm(a['push_N'],axis=1)>0.)))) for name,mask in m.items()}
        report['per_case'][case]=row
        for event in oldscience['events']:
            if event['case']!=case:continue
            i=event['index'];saved=dict(event)
            saved.update(correct_branch=int(a['correct_branch'][i]),point_distance_mm=None if not np.isfinite(a['point_distance_m'][i]) else float(a['point_distance_m'][i]*1000.),
                         routes={route:dict(valid=bool(a[route+'_valid'][i]),compatible=bool(a[route+'_compatible'][i]),
                                residual_Nm=a[route+'_residual_Nm'][i].tolist()) for route in ROUTES},
                         free_couple_Nm=a['free_couple_Nm'][i].tolist(),geometry_load_delta_Nm=a['geometry_load_delta_Nm'][i].tolist(),
                         original_code=int(a['cf_original_code'][i]),sampled_point_code=int(a['cf_sampled_correct_point_code'][i]),
                         sampled_correct_point_F_score=float(a['cf_sampled_correct_point_F_score'][i]),sampled_correct_point_Nalt_score=float(a['cf_sampled_correct_point_Nalt_score'][i]))
            report['events'].append(saved)
        np.savez_compressed(output/f'{case}_frames.npz',**a,**{'cohort_'+k:v for k,v in m.items()})
        all_arrays[case]=a;all_masks[case]=m
    merged={k:np.concatenate([x[k] for x in all_arrays.values()]) for k in next(iter(all_arrays.values()))}
    masks={k:np.concatenate([x[k] for x in all_masks.values()]) for k in next(iter(all_masks.values()))}
    report['cohorts']={name:describe(merged,mask) for name,mask in masks.items()}
    report['counterfactual']=counterfactual_report(merged,masks)
    report['geometry_question']=qualify(report['cohorts'],report['per_case'],merged,masks,protocol)
    report['decision']='Diagnostic only: retain live baseline; sampled endpoints cannot establish exact5ms load routing or a unique physical cause.'
    # Convert absent finite event values to JSON null, keeping NPZ NaNs explicit.
    def safe(x):
        if isinstance(x,dict):return {k:safe(v) for k,v in x.items()}
        if isinstance(x,list):return [safe(v) for v in x]
        if isinstance(x,float) and not np.isfinite(x):return None
        return x
    write(output/'science.json',safe(report))
    bindings={}
    for p in output.glob('*.npz'):
        z=load(p);bindings[p.name]={k:dict(dtype=v.dtype.str,shape=list(v.shape),sha256=hashlib.sha256(v.tobytes()).hexdigest()) for k,v in z.items()}
    write(output/'array_bindings.json',bindings)
    print(json.dumps(dict(iteration='I-66',focus=report['cohorts']['focus'],counterfactual=report['counterfactual'],geometry_question=report['geometry_question'])),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('protocol','pose-root','i61','i64','i65','context-root','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run(a.protocol.resolve(),a.pose_root.resolve(),a.i61.resolve(),a.i64.resolve(),a.i65.resolve(),a.context_root.resolve(),a.output.resolve())
