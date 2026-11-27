"""Independent I66 evaluation from hinge axes/anchors; no audit imports or fits."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json
import numpy as np

CASES=('m100_B','p030_B','p090_B')
ROUTES=('C_candidate','T_sampled_point','W_single_family_wrench','S_normal_share_proxy')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def load(path):
    with np.load(path,allow_pickle=False) as z:return {key:z[key].copy() for key in z.files}
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def array_hash(value):return hashlib.sha256(np.asarray(value).tobytes()).hexdigest()


def hinge_load(pose,point,force,family):
    """axis dot[(point-anchor) cross force], gated by body ancestry."""
    axis=pose['hinge_axis_world'];anchor=pose['hinge_anchor_m']
    moment=np.cross(point[:,None,:]-anchor,force[:,None,:])
    active=pose['ancestor_active'][np.clip(family,0,3)]
    return np.where(active,np.sum(axis*moment,axis=2),0.)


def wrench_load(pose,force,moment,family):
    about_anchor=moment[:,None,:]+np.cross(pose['centre_m'][:,None,:]-pose['hinge_anchor_m'],force[:,None,:])
    return np.where(pose['ancestor_active'][np.clip(family,0,3)],np.sum(pose['hinge_axis_world']*about_anchor,axis=2),0.)


def independent_routes(pose,p,b,truth):
    n=len(b['time_s']);row=np.arange(n);family=b['truth_family'];good_family=(family>=0)&(family<4);safe=np.clip(family,0,3)
    raw=p['observer_interval_mean'];filtered=p['observer_generalized'];force=raw[:,:3];moment=raw[:,3:6];target=raw[:,6:9]
    available=pose['valid']&np.isfinite(raw).all(axis=1)&np.isfinite(filtered).all(axis=1)&(np.sqrt(np.sum(filtered[:,:3]**2,axis=1))>=1.)
    validF=available&p['H_F_valid'];validN=available&p['H_Nalt_valid']
    correct=np.full(n,-1,np.int8)
    correct[good_family&validF&(p['proposed_family']==family)]=0
    chooseN=(correct<0)&good_family&validN&(p['native_family']==family);correct[chooseN]=1
    pointC=np.where((correct==0)[:,None],p['point_F_m'],p['point_Nalt_m']);pointC[correct<0]=np.nan
    H=np.where((correct==0)[:,None,None],p['H_F'],p['H_Nalt'])
    predC=np.sum(H*force[:,None,:],axis=2);Cvalid=correct>=0;predC[~Cvalid]=np.nan
    analyticC=hinge_load(pose,pointC,force,family)
    reference=truth['truth_patch_points_m'][row,safe].copy();reference[~good_family]=np.nan
    Tvalid=available&good_family&np.isfinite(reference).all(axis=1)
    predT=hinge_load(pose,reference,force,family);predT[~Tvalid]=np.nan
    Wvalid=available&good_family;predW=wrench_load(pose,force,moment,family);predW[~Wvalid]=np.nan
    loads=truth['truth_patch_loads_N'];total=np.sum(loads,axis=1)
    weights=np.divide(loads,total[:,None],out=np.zeros_like(loads),where=total[:,None]>0)
    positive=weights>0;points=truth['truth_patch_points_m']
    Svalid=available&np.isfinite(loads).all(axis=1)&(loads>=0).all(axis=1)&np.isfinite(total)&(total>=1.)
    Svalid&=np.all(~positive|np.isfinite(points).all(axis=2),axis=1)
    predS=np.zeros((n,3))
    for k in range(4):
        clean=np.where(positive[:,k,None],points[:,k],pose['centre_m'])
        predS+=hinge_load(pose,clean,weights[:,k,None]*force,np.full(n,k))
    predS[~Svalid]=np.nan
    out=dict(correct_branch=correct,normal_shares=weights,reference_point_m=reference,candidate_point_m=pointC,
        fixed_scale_Nm=.03+(.10*b['radius_m'])*b['force_norm_N'])
    for route,pred,valid in zip(ROUTES,[predC,predT,predW,predS],[Cvalid,Tvalid,Wvalid,Svalid]):
        residual=target-pred;valid=valid&np.isfinite(pred).all(axis=1)
        out[route+'_prediction_Nm']=pred;out[route+'_residual_Nm']=residual;out[route+'_valid']=valid
        out[route+'_compatible']=valid&(np.max(abs(residual),axis=1)<=out['fixed_scale_Nm'])
    out['C_analytic_prediction_Nm']=analyticC
    out['geometry_load_delta_Nm']=predT-predC
    out['free_couple_Nm']=moment-np.cross(reference-pose['centre_m'],force)
    out['free_couple_joint_Nm']=np.where(pose['ancestor_active'][safe],np.sum(pose['hinge_axis_world']*out['free_couple_Nm'][:,None,:],axis=2),0.)
    out['free_couple_joint_Nm'][~pose['valid']|~np.isfinite(out['free_couple_Nm']).all(axis=1)]=np.nan
    out['F_original_residual_Nm']=target-np.sum(p['H_F']*force[:,None,:],axis=2)
    out['Nalt_original_residual_Nm']=target-np.sum(p['H_Nalt']*force[:,None,:],axis=2)
    out['F_original_valid']=validF&np.isfinite(out['F_original_residual_Nm']).all(axis=1)
    out['Nalt_original_valid']=validN&np.isfinite(out['Nalt_original_residual_Nm']).all(axis=1)
    return out


def groups(base,out,oof):
    primary=base['primary_eligible'];Nbad=base['N_family']!=base['truth_family'];Abad=base['A_family']!=base['truth_family']
    useful=primary&Nbad&~Abad;harmful=primary&~Nbad&Abad;keep=oof['M1__shared_covariance_keep']
    correct=primary&base['i60_accepted']&(out['correct_branch']>=0)
    stable=np.zeros(len(primary),bool);f=base['truth_family']
    for i in range(2,len(f)-2):stable[i]=(f[i]>=0 and all(f[i+d]==f[i] for d in [-2,-1,0,1,2]))
    return dict(focus=useful&~keep,useful=useful,harmful=harmful,retained_useful=useful&keep,
        correct_pair=correct,stable_correct_pair=correct&stable,full_reference=primary)


def classify(first,second,validF,validN,mean,covariance,config):
    eigenvalue,eigenvector=np.linalg.eigh(covariance)
    if np.any(eigenvalue<=0):raise ValueError('Frozen calibration is not positive definite')
    def score(values):
        projected=(values-mean)@eigenvector
        return np.sum(projected*projected/eigenvalue,axis=1)
    F=score(first);N=score(second);valid=validF&validN&np.isfinite(F)&np.isfinite(N)
    distance=np.sqrt(np.sum((first-second)**2,axis=1));pattern=valid&(distance<=config['pattern_epsilon_dimensionless'])
    code=np.full(len(F),3,np.int8);code[pattern]=2;rest=valid&~pattern
    eps=config['relative_score_tie_epsilon']*np.maximum(1.,np.maximum(abs(F),abs(N)));delta=F-N
    code[rest&(delta < -eps)]=-1;code[rest&(delta>eps)]=1;code[rest&(abs(delta)<=eps)]=0
    return dict(code=code,F_score=F,Nalt_score=N,valid=valid)


def counterfactual(out,masks,model,config):
    opportunity=masks['useful']|masks['harmful'];indices=np.flatnonzero(opportunity);n=len(opportunity)
    scale=out['fixed_scale_Nm'];F=out['F_original_residual_Nm']/scale[:,None];N=out['Nalt_original_residual_Nm']/scale[:,None]
    validF=out['F_original_valid'];validN=out['Nalt_original_valid']
    mean=np.asarray(model['mean']);cov=np.asarray(model['covariance'])
    original=classify(F[indices],N[indices],validF[indices],validN[indices],mean,cov,config)
    Fnew=F.copy();Nnew=N.copy();vF=validF.copy();vN=validN.copy()
    replacement=out['T_sampled_point_residual_Nm']/scale[:,None];branch=out['correct_branch']
    takeF=opportunity&(branch==0);takeN=opportunity&(branch==1)
    Fnew[takeF]=replacement[takeF];Nnew[takeN]=replacement[takeN]
    vF[takeF]=out['T_sampled_point_valid'][takeF];vN[takeN]=out['T_sampled_point_valid'][takeN]
    changed=classify(Fnew[indices],Nnew[indices],vF[indices],vN[indices],mean,cov,config)
    result={'cf_applicable':opportunity}
    for label,pair in [('original',original),('sampled_correct_point',changed)]:
        code=np.full(n,3,np.int8);code[indices]=pair['code'];result['cf_'+label+'_code']=code
        known=np.zeros(n,bool);known[indices]=pair['valid'];result['cf_'+label+'_valid']=known
        for branch_name in ['F','Nalt']:
            value=np.full(n,np.nan);value[indices]=pair[branch_name+'_score'];result['cf_'+label+'_'+branch_name+'_score']=value
    return result


def compare_numeric(actual,expected,atol,rtol=0.):
    finite=np.isfinite(actual)&np.isfinite(expected)
    patterns=np.array_equal(np.isfinite(actual),np.isfinite(expected))
    differences=np.abs(actual[finite]-expected[finite]);limit=atol+rtol*np.abs(expected[finite])
    return dict(finite_patterns_exact=bool(patterns),max_absolute_difference=float(differences.max()) if len(differences) else 0.,
        absolute_tolerance=atol,relative_tolerance=rtol,passes=bool(patterns and np.all(differences<=limit)))


def vector_stats(values,mask):
    value=values[mask&np.isfinite(values).all(axis=1)]
    if not len(value):return dict(count=0,vector_RMS_Nm=None,vector_p95_Nm=None,vector_max_Nm=None,per_component_RMS_Nm=None)
    magnitude=np.sqrt(np.sum(value*value,axis=1))
    return dict(count=len(value),vector_RMS_Nm=float(np.linalg.norm(value)/np.sqrt(len(value))),
        vector_p95_Nm=float(np.quantile(magnitude,.95)),vector_max_Nm=float(max(magnitude)),
        per_component_RMS_Nm=np.sqrt(np.sum(value*value,axis=0)/len(value)).tolist())


def check_stats(out,masks,canonical):
    checks={}
    for name,mask in masks.items():
        expected=canonical[name];counts=dict(denominator=int(mask.sum()),paired_count=int(np.sum(mask&out['C_candidate_valid']&out['T_sampled_point_valid'])))
        row={'denominators_exact':all(expected[key]==value for key,value in counts.items()),'routes':{}}
        for route in ROUTES:
            actual=vector_stats(out[route+'_residual_Nm'],mask&out[route+'_valid']);target=expected['routes'][route]
            matches=dict(count=actual['count']==target['count'],compatible_count=int(np.sum(mask&out[route+'_compatible']))==target['compatible_count'],
                unknown=int(np.sum(mask&~out[route+'_valid']))==target['unknown'])
            differences={}
            for key in ['vector_RMS_Nm','vector_p95_Nm','vector_max_Nm','per_component_RMS_Nm']:
                if actual[key] is None or target[key] is None:matches[key]=actual[key] is target[key]
                else:differences[key]=float(np.max(abs(np.asarray(actual[key])-np.asarray(target[key]))))
            row['routes'][route]=dict(exact_counts=matches,float_differences=differences,
                passes=all(matches.values()) and all(x<=1e-9 for x in differences.values()))
        row['passes']=row['denominators_exact'] and all(x['passes'] for x in row['routes'].values());checks[name]=row
    return checks


def cf_summary(out,masks):
    result={};old=out['cf_original_code'];new=out['cf_sampled_correct_point_code']
    for group,wanted in [('useful',-1),('harmful',1),('focus',-1)]:
        mask=masks[group]
        result[group]=dict(denominator=int(mask.sum()),original_correct=int(np.sum(mask&(old==wanted))),replacement_correct=int(np.sum(mask&(new==wanted))),
            newly_correct=int(np.sum(mask&(old!=wanted)&(new==wanted))),newly_wrong=int(np.sum(mask&(old==wanted)&(new!=wanted))),
            unknown=int(np.sum(mask&(new==3))),ambiguous=int(np.sum(mask&(new==2))),tied=int(np.sum(mask&(new==0))))
    return result


def run(args):
    protocol=read(args.experiment/'protocol/protocol.json');oldprotocol=read(args.i65/'protocol/protocol.json')
    roots=dict(experiment=args.experiment,i61=args.i61,i64=args.i64,i65=args.i65,pose=args.pose_root,context=args.experiment/'inputs/context_stage01')
    seal_path=args.experiment/'protocol/analysis_input_seal.json';seal=read(seal_path)
    for item in seal['files']:
        if sha(roots[item['root']]/item['path'])!=item['sha256']:raise ValueError('Changed frozen analysis input')
    args.output.mkdir(parents=True,exist_ok=False)
    write(args.output/'registered_before_independent_arithmetic.json',dict(source_sha256=sha(Path(__file__)),protocol_sha256=sha(args.experiment/'protocol/protocol.json'),
        analysis_input_seal_sha256=sha(seal_path),registered_utc=datetime.now(timezone.utc).isoformat(),numpy_version=np.__version__,
        method='Independent hinge cross/dot loads and eigen scores; no audit or method-module imports, no fitting.'))
    oldscience=read(args.i65/'analysis/stage01/science.json');science=read(args.canonical/'science.json')
    report=dict(iteration='I-66',cases={},new_fits=False,new_physics=False);outputs={};masks_all={}
    for case in CASES:
        pose=load(args.pose_root/case/'pose_maps.npz');p=load(args.i64/'features/portable_stage01'/case/'primitives.npz')
        b=load(args.i64/'portable_inputs/i62_base'/f'{case}.npz');truth=load(args.i61/'analysis/evaluation_compact_stage01'/f'{case}_truth_projection.npz')
        oof=load(args.i65/'analysis/stage01'/f'{case}_oof.npz');canonical=load(args.canonical/f'{case}_frames.npz')
        out=independent_routes(pose,p,b,truth);masks=groups(b,out,oof)
        out.update(counterfactual(out,masks,oldscience['models'][case]['M1']['fitted'],oldprotocol['comparison']))
        exact={key:bool(np.array_equal(out[key],canonical[key])) for key in ['correct_branch','cf_applicable','cf_original_code','cf_sampled_correct_point_code','cf_original_valid','cf_sampled_correct_point_valid']}
        for route in ROUTES:
            for suffix in ['valid','compatible']:exact[route+'_'+suffix]=bool(np.array_equal(out[route+'_'+suffix],canonical[route+'_'+suffix]))
        for name,mask in masks.items():exact['cohort_'+name]=bool(np.array_equal(mask,canonical['cohort_'+name]))
        numeric={}
        for route in ROUTES:
            for suffix in ['prediction_Nm','residual_Nm']:
                key=route+'_'+suffix;numeric[key]=compare_numeric(out[key],canonical[key],1e-9)
        for key in ['normal_shares','reference_point_m','candidate_point_m','geometry_load_delta_Nm','free_couple_Nm','free_couple_joint_Nm']:
            numeric[key]=compare_numeric(out[key],canonical[key],1e-9)
        known=out['C_candidate_valid'];numeric['analytic_C_vs_frozen_H']=compare_numeric(out['C_analytic_prediction_Nm'][known],out['C_candidate_prediction_Nm'][known],1e-9)
        for label in ['original','sampled_correct_point']:
            for branch in ['F','Nalt']:
                key='cf_'+label+'_'+branch+'_score';numeric[key]=compare_numeric(out[key],canonical[key],protocol['verification']['counterfactual_score_atol'],protocol['verification']['counterfactual_score_rtol'])
        stats=check_stats(out,masks,science['per_case'][case]['cohorts']);counter=cf_summary(out,masks)
        counter_exact=counter==science['per_case'][case]['counterfactual']
        hashes={key:dict(dtype=str(value.dtype),shape=list(value.shape),sha256=array_hash(value)) for key,value in out.items() if value.dtype.kind in 'biu'}
        hashes.update({'cohort_'+key:dict(dtype=str(value.dtype),shape=list(value.shape),sha256=array_hash(value)) for key,value in masks.items()})
        report['cases'][case]=dict(exact_masks_decisions=exact,numeric_checks=numeric,cohort_statistics=stats,
            counterfactual_counts=counter,counterfactual_counts_exact=counter_exact,mask_decision_hashes=hashes,
            passes=all(exact.values()) and all(x['passes'] for x in numeric.values()) and all(x['passes'] for x in stats.values()) and counter_exact)
        outputs[case]=out;masks_all[case]=masks
    merged={key:np.concatenate([outputs[case][key] for case in CASES]) for key in outputs[CASES[0]]}
    merged_masks={key:np.concatenate([masks_all[case][key] for case in CASES]) for key in masks_all[CASES[0]]}
    report['pooled_statistics']=check_stats(merged,merged_masks,science['cohorts'])
    report['pooled_counterfactual_counts']=cf_summary(merged,merged_masks);report['pooled_counterfactual_exact']=report['pooled_counterfactual_counts']==science['counterfactual']
    report['all_checks_passed']=all(case['passes'] for case in report['cases'].values()) and all(x['passes'] for x in report['pooled_statistics'].values()) and report['pooled_counterfactual_exact']
    report['source_sha256']=sha(Path(__file__));write(args.output/'report.json',report)
    print(json.dumps(dict(all_checks_passed=report['all_checks_passed'],cases={c:r['passes'] for c,r in report['cases'].items()},pooled_counterfactual=report['pooled_counterfactual_counts'])))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--experiment',type=Path,default=Path(__file__).resolve().parents[2])
    for name in ['i61','i64','i65','pose-root','canonical','output']:parser.add_argument('--'+name,type=Path)
    a=parser.parse_args();a.experiment=a.experiment.resolve();protocol=read(a.experiment/'protocol/protocol.json');repo=a.experiment.parents[1]
    for name in ['i61','i64','i65']:setattr(a,name,getattr(a,name).resolve() if getattr(a,name) else repo/protocol['dependencies'][name])
    a.pose_root=a.pose_root.resolve() if a.pose_root else a.experiment/'features/stage01'
    a.canonical=a.canonical.resolve() if a.canonical else a.experiment/'analysis/stage01'
    a.output=a.output.resolve() if a.output else Path(__file__).resolve().parent/'output_stage01'
    run(a)
