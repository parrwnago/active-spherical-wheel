"""Independent I-65 fit/rank/policy review; no primary scorer imports.

The archived I-64 history transform is shared frozen input preparation. Means,
covariances, eigen scores, teacher rows, classification and metrics are
recomputed here. Execute only after root releases the historical comparison.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,importlib.util,json
import numpy as np

CASES=('m100_B','p030_B','p090_B')
MODELS=('M1','M2')
SCORERS=('maxabs','euclidean','shared_covariance')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False) as data:return {k:data[k].copy() for k in data.files}
def module(path):
    spec=importlib.util.spec_from_file_location('_shared_frozen_history',path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def normalize(raw,base):
    scale=.03+(.10*base['radius_m'])*base['force_norm_N']
    good=np.isfinite(raw).all(axis=1)&np.isfinite(scale)&(scale>0)
    good &= np.isfinite(base['radius_m'])&(base['radius_m']>0)&np.isfinite(base['force_norm_N'])&(base['force_norm_N']>=0)
    result=np.full(raw.shape,np.nan);result[good]=raw[good]/scale[good,None]
    return result,good


def select_rows(base,primitive,feature,model):
    e={};valid={}
    for branch in ('F','Nalt'):
        e[branch],finite=normalize(feature[model+'_'+branch+'_residual_Nm'],base)
        valid[branch]=finite&feature[model+'_'+branch+'_valid']&primitive['material_'+branch+'_valid']
    allowed=base['primary_eligible']&base['i60_accepted'];truth=base['truth_family']
    first=allowed&valid['F']&(primitive['proposed_family']==truth)
    second=allowed&~first&valid['Nalt']&(primitive['native_family']==truth)
    teacher=np.full(len(truth),-1,np.int8);teacher[first]=0;teacher[second]=1
    chosen=np.flatnonzero(teacher>=0)
    rows=np.empty((len(chosen),3))
    for j,i in enumerate(chosen):rows[j]=e['F'][i] if teacher[i]==0 else e['Nalt'][i]
    return rows,teacher


def fit(episode_rows,config):
    if any(len(x)<config['minimum_selected_rows_per_episode'] for x in episode_rows):return None
    # Equal episode mass, explicit per-episode Gram matrix about GLOBAL mean.
    mean=sum(np.mean(x,axis=0) for x in episode_rows)/len(episode_rows)
    population=sum((x-mean).T@(x-mean)/len(x) for x in episode_rows)/len(episode_rows)
    population=(population+population.T)/2
    rho=config['shrinkage_rho'];floor=config['floor_std_dimensionless']
    covariance=(1-rho)*population+rho*np.trace(population)/3*np.eye(3)+floor**2*np.eye(3)
    if not np.isfinite(covariance).all():return None
    eigenvalues,eigenvectors=np.linalg.eigh(covariance)
    if np.any(eigenvalues<=0):return None
    result=dict(mean=mean,population_covariance=population,covariance=covariance,
        eigenvalues=eigenvalues,eigenvectors=eigenvectors,inverse=np.linalg.inv(covariance))
    rows=np.concatenate(episode_rows)
    weights=np.concatenate([np.full(len(x),1/(len(episode_rows)*len(x))) for x in episode_rows]);weights/=weights.sum()
    result['normalized_weights']=weights
    score=eigen_score(rows,result);order=np.argsort(score,kind='stable');CDF=np.cumsum(weights[order])
    index=min(int(np.searchsorted(CDF,.95,side='left')),len(rows)-1)
    result['training_weighted_p95_score']=float(score[order[index]])
    result['p95_cumulative_weight']=float(CDF[index]);result['p95_previous_cumulative_weight']=float(CDF[index-1]) if index else 0.
    return result


def eigen_score(values,fit):
    scores=np.full(len(values),np.nan);good=np.isfinite(values).all(axis=1)
    centered=values[good]-fit['mean'];rotated=centered@fit['eigenvectors']
    scores[good]=np.sum(rotated*rotated/fit['eigenvalues'],axis=1)
    return scores


def decision(F,N,eF,eN,valid,config):
    code=np.full(len(F),3,np.int8);valid=valid&np.isfinite(F)&np.isfinite(N)
    difference=eF-eN;distance=np.sqrt(np.einsum('ij,ij->i',difference,difference))
    ambiguous=valid&(distance<=config['pattern_epsilon_dimensionless']);code[ambiguous]=2
    tolerance=config['relative_score_tie_epsilon']*np.maximum(1,np.maximum(abs(F),abs(N)))
    remaining=valid&~ambiguous
    code[remaining&(F<N-tolerance)]=-1;code[remaining&(N<F-tolerance)]=1
    code[remaining&(abs(F-N)<=tolerance)]=0
    return code


def scores(base,primitive,feature,model,fit,protocol):
    eF,goodF=normalize(feature[model+'_F_residual_Nm'],base);eN,goodN=normalize(feature[model+'_Nalt_residual_Nm'],base)
    valid=goodF&goodN&feature[model+'_F_valid']&feature[model+'_Nalt_valid'];result=dict(e_F=eF,e_Nalt=eN,pair_valid=valid)
    for scorer in SCORERS:
        if scorer=='maxabs':F=np.max(abs(eF),axis=1);N=np.max(abs(eN),axis=1)
        elif scorer=='euclidean':F=np.einsum('ij,ij->i',eF,eF);N=np.einsum('ij,ij->i',eN,eN)
        elif fit is not None:F=eigen_score(eF,fit);N=eigen_score(eN,fit)
        else:F=np.full(len(valid),np.nan);N=F.copy()
        code=decision(F,N,eF,eN,valid,protocol['comparison'])
        result[scorer+'_F_score']=F;result[scorer+'_Nalt_score']=N;result[scorer+'_code']=code
        result[scorer+'_known']=valid&np.isfinite(F)&np.isfinite(N)
        result[scorer+'_keep']=base['i60_accepted']&primitive['H_F_valid']&(code!=1)
    if fit is not None:
        result['both_above_training_p95']=(result['shared_covariance_known']&
            (result['shared_covariance_F_score']>fit['training_weighted_p95_score'])&
            (result['shared_covariance_Nalt_score']>fit['training_weighted_p95_score']))
    else:result['both_above_training_p95']=np.zeros(len(valid),bool)
    return result


def numeric_check(actual,expected,atol,rtol=0):
    a=np.asarray(actual);b=np.asarray(expected)
    shape=a.shape==b.shape
    if not shape:return dict(passed=False,reason='shape differs',actual_shape=list(a.shape),expected_shape=list(b.shape))
    finite=np.isfinite(a)&np.isfinite(b);patterns=np.array_equal(np.isfinite(a),np.isfinite(b))
    maximum=float(np.max(abs(a[finite]-b[finite]))) if finite.any() else 0.
    passed=patterns and bool(np.all(abs(a[finite]-b[finite])<=atol+rtol*abs(b[finite])))
    return dict(passed=bool(passed),finite_patterns_exact=bool(patterns),maximum_absolute_difference=maximum,atol=atol,rtol=rtol)


def outcome_metrics(base,keep):
    selected=base['primary_eligible'];truth=base['truth_family'][selected]
    native=base['N_family'][selected];proposal=base['A_family'][selected];chosen=keep[selected]
    Nwrong=native!=truth;Awrong=proposal!=truth
    p=selected&base['point_mask'];dist=np.where(keep[p],base['proposed_distance_m'][p],base['native_distance_m'][p])*1000.
    return dict(error_count=int(np.sum(np.where(chosen,Awrong,Nwrong))),
        harmful_count=int(np.sum(chosen&~Nwrong&Awrong)),useful_retained_count=int(np.sum(chosen&Nwrong&~Awrong)),
        retained_proposal_count=int(chosen.sum()),veto_count=int(base['i60_accepted'][selected].sum()-chosen.sum()),
        point_SSE_mm2=float(dist@dist),point_RMS_mm=float(np.sqrt(np.mean(dist**2))),
        point_p95_mm=float(np.percentile(dist,95)),point_maximum_mm=float(np.max(dist)))


def ranking(base,arrays,scorer):
    domain=base['primary_eligible'];Nwrong=base['N_family']!=base['truth_family'];Awrong=base['A_family']!=base['truth_family']
    masks=dict(primary=domain,accepted=domain&base['i60_accepted'],useful=domain&Nwrong&~Awrong,
        harmful=domain&~Nwrong&Awrong,accepted_wrong=domain&base['i60_accepted']&Awrong)
    code=arrays[scorer+'_code'];known=arrays[scorer+'_known'];out={}
    for name,mask in masks.items():
        n=int(mask.sum());row=dict(denominator=n,known=int(np.sum(mask&known)),unknown=int(np.sum(mask&(code==3))),
            F_preferred=int(np.sum(mask&(code==-1))),Nalt_preferred=int(np.sum(mask&(code==1))),
            tied=int(np.sum(mask&(code==0))),ambiguous=int(np.sum(mask&(code==2))))
        row['coverage']=row['known']/n if n else None
        if name in ('useful','harmful'):
            row['correct']=row['F_preferred' if name=='useful' else 'Nalt_preferred'];row['correct_fraction']=row['correct']/n if n else None
        out[name]=row
    return out


def shadow_gates(base,phase_bases,metrics,phase_metrics,protocol):
    all_A=outcome_metrics(base,base['i60_accepted']);all_N=outcome_metrics(base,np.zeros(len(base['time_s']),bool))
    selected=base['primary_eligible'];Nwrong=base['N_family']!=base['truth_family'];Awrong=base['A_family']!=base['truth_family']
    H=int(np.sum(selected&~Nwrong&Awrong));U=int(np.sum(selected&Nwrong&~Awrong));cfg=protocol['training_gates'];eps=cfg['point_comparison_roundoff_allowance_mm']
    gates=dict(harmful_removal=bool(H and (H-metrics['harmful_count'])/H>=cfg['harmful_removal_fraction_min']),
        useful_retention=bool(U and metrics['useful_retained_count']/U>=cfg['useful_retention_fraction_min']),pooled_errors=metrics['error_count']<=all_A['error_count'])
    for case,b in phase_bases.items():
        a=outcome_metrics(b,b['i60_accepted']);n=int(b['primary_eligible'].sum());m=phase_metrics[case]
        gates[case+'_error_rate']=m['error_count']/n<=a['error_count']/n+cfg['each_phase_error_rate_excess_over_A_max']
        gates[case+'_harmful_nonincrease']=m['harmful_count']<=a['harmful_count']
    gates.update(point_RMS=metrics['point_RMS_mm']<=all_A['point_RMS_mm']+eps,
        point_p95=metrics['point_p95_mm']<=all_A['point_p95_mm']+eps,
        point_maximum=metrics['point_maximum_mm']<=all_N['point_maximum_mm']+eps,
        nonempty_retention=metrics['retained_proposal_count']>=cfg['retained_proposal_count_min'])
    return {k:bool(v) for k,v in gates.items()}


def discrimination_gates(base,arrays,protocol):
    reports={s:ranking(base,arrays,s) for s in SCORERS};old=reports['maxabs'];new=reports['shared_covariance'];g=protocol['discrimination_gates']
    selected=base['primary_eligible'];Nwrong=base['N_family']!=base['truth_family'];Awrong=base['A_family']!=base['truth_family']
    point=selected&base['point_mask'];native_max=float(np.max(base['native_distance_m'][point]))
    distance=np.where(base['i60_accepted'],base['proposed_distance_m'],base['native_distance_m'])
    critical=selected&((~Nwrong&Awrong)|(base['i60_accepted']&Awrong)|
        (base['i60_accepted']&point&(distance>native_max+1e-15)))
    flags=dict(accepted_pair_coverage=new['accepted']['coverage'] is not None and new['accepted']['coverage']>=g['coverage_min'],
        critical_scorable=bool(np.all(arrays['shared_covariance_known'][critical])),
        half_harmful_correct=new['harmful']['correct_fraction'] is not None and new['harmful']['correct_fraction']>=g['harmful_correct_fraction_min'],
        useful_90pct_correct=new['useful']['correct_fraction'] is not None and new['useful']['correct_fraction']>=g['useful_correct_fraction_min'],
        harmful_nonregression=new['harmful']['correct']>=old['harmful']['correct'],useful_nonregression=new['useful']['correct']>=old['useful']['correct'],
        strict_improvement=new['harmful']['correct']+new['useful']['correct']>old['harmful']['correct']+old['useful']['correct'])
    return reports,{k:bool(v) for k,v in flags.items()}


def run(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    protocol=read(args.protocol);canonical=read(args.canonical/'science.json');verification=protocol['verification'];previous=args.previous.resolve()
    write(output/'registered_before_independent_review.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),protocol_sha256=sha(args.protocol),canonical_science_sha256=sha(args.canonical/'science.json'),
        status='before historical numerical arithmetic in independent verifier',numpy_version=np.__version__,
        independence='No I65 method/evaluator imports. Equal-episode Gram covariance, eig inverse score, own rows/ranks/points/gates. Shared archived I64 history transform only.'))
    history=module(previous/'features/portable_stage01/m100_B/source/history_features.py')
    datasets={}
    for case in CASES:
        b=load(previous/'portable_inputs/i62_base'/(case+'.npz'));p=load(previous/'features/portable_stage01'/case/'primitives.npz')
        datasets[case]=(b,p,history.derive_history(p))
    reports={};oof={c:{} for c in CASES};checks=[]
    for heldout in CASES:
        training=[c for c in CASES if c!=heldout];saved=load(args.canonical/(heldout+'_training_selections.npz'));actual=load(args.canonical/(heldout+'_oof.npz'))
        reports[heldout]={}
        for model in MODELS:
            rows=[];teacher_checks={};counts={}
            for case in training:
                e,teacher=select_rows(*datasets[case],model);rows.append(e)
                exact=np.array_equal(teacher,saved[model+'__'+case+'__teacher_branch']);teacher_checks[case]=bool(exact)
                checks.append(bool(exact));counts[case]=dict(rows=len(e),F=int(np.sum(teacher==0)),Nalt=int(np.sum(teacher==1)))
            fit_model=fit(rows,protocol['calibration']);reference=canonical['models'][heldout][model];fit_checks={}
            checks.append(counts==reference['training_counts'])
            checks.append((fit_model is None)==reference['calibration_failed'])
            if fit_model is None:
                checks.append(bool(reference['calibration_failed']))
            else:
                for key in ('mean','population_covariance','covariance'):
                    c=numeric_check(fit_model[key],reference['fitted'][key],verification['mean_covariance_atol']);fit_checks[key]=c;checks.append(c['passed'])
                q=numeric_check(fit_model['training_weighted_p95_score'],reference['fitted']['training_weighted_p95_score'],verification['score_atol'],verification['score_rtol'])
                fit_checks['training_weighted_p95_score']=q;checks.append(q['passed'])
                L=np.asarray(reference['fitted']['cholesky']);fit_checks['primary_cholesky_reconstructs_independent_C']=numeric_check(L@L.T,fit_model['covariance'],verification['mean_covariance_atol'])
                checks.append(fit_checks['primary_cholesky_reconstructs_independent_C']['passed'])
            arr=scores(*datasets[heldout],model,fit_model,protocol);oof[heldout][model]=arr;array_checks={}
            for key,values in arr.items():
                expected=actual[model+'__'+key]
                if values.dtype.kind in 'biu':
                    c=dict(passed=bool(np.array_equal(values,expected)),exact=bool(np.array_equal(values,expected)))
                else:
                    atol=verification['normalized_residual_atol'] if key.startswith('e_') else verification['score_atol']
                    rtol=0 if key.startswith('e_') else verification['score_rtol'];c=numeric_check(values,expected,atol,rtol)
                array_checks[key]=c;checks.append(c['passed'])
            reports[heldout][model]=dict(training=training,teacher_masks_exact=teacher_checks,training_counts=counts,fit_checks=fit_checks,array_checks=array_checks,
                independent_fit=None if fit_model is None else {k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in fit_model.items()})
    bases={c:d[0] for c,d in datasets.items()};full={k:np.concatenate([bases[c][k] for c in CASES]) for k in bases[CASES[0]]}
    outcomes={}
    for model in MODELS:
        arr={k:np.concatenate([oof[c][model][k] for c in CASES]) for k in oof[CASES[0]][model]}
        ranks,dgates=discrimination_gates(full,arr,protocol)
        expected_d=canonical['discrimination'][model]
        checks.append(ranks==expected_d['ranking']);checks.append(dgates==expected_d['gates']);policies={}
        for scorer in SCORERS:
            phase={c:outcome_metrics(bases[c],oof[c][model][scorer+'_keep']) for c in CASES}
            metrics=outcome_metrics(full,arr[scorer+'_keep']);gates=shadow_gates(full,bases,metrics,phase,protocol)
            expected=canonical['shadows'][model][scorer];metric_checks={}
            for key,value in metrics.items():
                if isinstance(value,int):c=dict(passed=value==expected['metrics'][key],actual=value,expected=expected['metrics'][key])
                else:c=numeric_check(value,expected['metrics'][key],verification['point_metric_atol_mm'])
                metric_checks[key]=c;checks.append(c['passed'])
            checks.append(gates==expected['gates'])
            for case in CASES:
                for key,value in phase[case].items():
                    expected_value=expected['per_case'][case]['metrics'][key]
                    check=(value==expected_value if isinstance(value,int) else numeric_check(value,expected_value,verification['point_metric_atol_mm'])['passed'])
                    checks.append(bool(check))
            policies[scorer]=dict(metrics=metrics,gates=gates,metric_checks=metric_checks,all_gates_pass=all(gates.values()),per_case=phase)
        outcomes[model]=dict(ranking=ranks,discrimination_gates=dgates,discrimination_exact=ranks==expected_d['ranking'],
            shadow=policies,promising=all(dgates.values()))
    qualified=outcomes['M1']['promising'] and outcomes['M1']['shadow']['shared_covariance']['all_gates_pass'];checks.append(qualified==canonical['primary_qualified'])
    result=dict(iteration='I-65',numpy_version=np.__version__,all_checks_passed=all(checks),check_count=len(checks),
        folds=reports,outcomes=outcomes,primary_qualified=qualified,source_sha256=sha(Path(__file__)),new_physics_steps=0,
        limitation='Retrospective review on three previously examined episodes. Shared frozen history preparation, independent covariance/ranking/evaluation; not unseen or physical validation.')
    write(output/'comparison.json',result)
    print(json.dumps(dict(all_checks_passed=result['all_checks_passed'],check_count=len(checks),primary_qualified=qualified,numpy_version=np.__version__)))
    return result['all_checks_passed']


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('protocol','previous','canonical','output'):p.add_argument('--'+name,type=Path,required=True)
    raise SystemExit(0 if run(p.parse_args()) else 1)
