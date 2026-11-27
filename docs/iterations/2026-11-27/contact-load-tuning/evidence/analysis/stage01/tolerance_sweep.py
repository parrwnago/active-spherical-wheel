"""I-62 fixed-grid, grouped retrospective tolerance selection; NumPy only."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,csv,hashlib,json,math,shutil
import numpy as np

CASES=('m100_B','p030_B','p090_B')
STEP=.005


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [safe(v) for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float) and not math.isfinite(x):return None
    return x
def save(path,x):Path(path).write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}


def parameter_grid(protocol):
    g=protocol['grid']
    if g['a_indices']!=[0,60] or g['b_indices']!=[0,60] or g['step']!=STEP or g['count']!=3721:
        raise ValueError('Only the frozen 61 by 61 grid is implemented')
    ai=np.repeat(np.arange(61,dtype=np.int64),61);bi=np.tile(np.arange(61,dtype=np.int64),61)
    return dict(a_index=ai,b_index=bi,a_Nm=ai*STEP,b_dimensionless=bi*STEP)


def keep_proposal(data,a,b,off=False):
    """Original shared component bound, with explicit zero/off boundary rules."""
    if off:return np.asarray(data['i60_accepted'],bool).copy()
    if not np.isfinite([a,b]).all() or a<0 or b<0:raise ValueError('Nonnegative finite global coefficients required')
    tolerance=a+(b*data['radius_m'])*data['force_norm_N']
    valid=np.asarray(data['gate_valid'],bool)&np.isfinite(tolerance)&(tolerance>=0)
    valid &= np.isfinite(data['joint_max_abs_Nm'])
    return data['i60_accepted']&valid&(data['joint_max_abs_Nm']<=tolerance)


def domain(cases):
    result={}
    for name in cases[0]:result[name]=np.concatenate([case[name] for case in cases],axis=0)
    selected=result['primary_eligible']
    out={name:value[selected] for name,value in result.items()}
    return out


def score_keeps(data,keep):
    """Candidate-major arrays preserve deterministic float64 reduction order."""
    keep=np.asarray(keep,bool)
    if keep.ndim==1:keep=keep[None,:]
    if keep.shape[1]!=len(data['N_family']):raise ValueError('One policy choice per eligible frame required')
    truth=data['truth_family'];Nwrong=data['N_family']!=truth;Awrong=data['A_family']!=truth
    corrected=Nwrong&~Awrong;harmful=~Nwrong&Awrong
    wrong=np.where(keep,Awrong[None,:],Nwrong[None,:])
    point_mask=data['point_mask']
    distances=np.where(keep[:,point_mask],data['proposed_distance_m'][None,point_mask],data['native_distance_m'][None,point_mask])*1000.
    npoint=int(point_mask.sum());count=len(truth)
    if npoint:
        SSE=np.sum(distances*distances,axis=1,dtype=np.float64)
        RMS=np.sqrt(SSE/npoint);p95=np.percentile(distances,95,axis=1);maximum=np.max(distances,axis=1)
    else:SSE=RMS=p95=maximum=np.full(len(keep),np.nan)
    retained=np.sum(keep,axis=1,dtype=np.int64)
    return dict(error_count=np.sum(wrong,axis=1,dtype=np.int64),
        harmful_count=np.sum(keep&harmful[None,:],axis=1,dtype=np.int64),
        useful_retained_count=np.sum(keep&corrected[None,:],axis=1,dtype=np.int64),
        retained_proposal_count=retained,veto_count=int(data['i60_accepted'].sum())-retained,
        point_SSE_mm2=SSE,point_RMS_mm=RMS,point_p95_mm=p95,point_maximum_mm=maximum)


def constants(data):
    Nwrong=data['N_family']!=data['truth_family'];Awrong=data['A_family']!=data['truth_family']
    off=score_keeps(data,data['i60_accepted']);native=score_keeps(data,np.zeros(len(Nwrong),bool))
    scalar=lambda d:{k:(int(v[0]) if np.issubdtype(v.dtype,np.integer) else float(v[0])) for k,v in d.items()}
    return dict(eligible_count=len(Nwrong),point_count=int(data['point_mask'].sum()),
        A_harmful_count=int(np.sum(~Nwrong&Awrong)),A_useful_count=int(np.sum(Nwrong&~Awrong)),
        A_proposal_count=int(data['i60_accepted'].sum()),oracle_count=int(data['oracle_mask'].sum()),
        N=scalar(native),A=scalar(off))


def scan_group(data,grid,chunk=128):
    results=[];oracle=[]
    for start in range(0,len(grid['a_index']),chunk):
        sl=slice(start,min(start+chunk,len(grid['a_index'])))
        a=grid['a_Nm'][sl,None];b=grid['b_dimensionless'][sl,None]
        tolerance=a+(b*data['radius_m'][None,:])*data['force_norm_N'][None,:]
        valid=data['gate_valid'][None,:]&np.isfinite(tolerance)&(tolerance>=0)&np.isfinite(data['joint_max_abs_Nm'])[None,:]
        keep=data['i60_accepted'][None,:]&valid&(data['joint_max_abs_Nm'][None,:]<=tolerance)
        results.append(score_keeps(data,keep))
        oracle_pass=(data['oracle_valid'][None,:]&data['oracle_mask'][None,:]
            &np.isfinite(data['oracle_joint_max_abs_Nm'])[None,:]
            &(data['oracle_joint_max_abs_Nm'][None,:]<=tolerance))
        oracle.append(np.sum(oracle_pass,axis=1,dtype=np.int64))
    output={key:np.concatenate([part[key] for part in results]) for key in results[0]}
    output['oracle_pass_count']=np.concatenate(oracle)
    return output


def training_feasibility(metrics,base,phase_metrics,phase_bases,protocol):
    g=protocol['training_gates'];n=len(metrics['error_count']);flags={}
    harmful=base['A_harmful_count'];useful=base['A_useful_count']
    flags['harmful_removal']=(np.full(n,False) if harmful==0 else
        (harmful-metrics['harmful_count'])/harmful>=g['harmful_removal_fraction_min'])
    flags['useful_retention']=(np.full(n,False) if useful==0 else
        metrics['useful_retained_count']/useful>=g['useful_retention_fraction_min'])
    flags['pooled_errors']=metrics['error_count']<=base['A']['error_count']
    for case,row in phase_metrics.items():
        b=phase_bases[case];den=b['eligible_count']
        flags[case+'_error_rate']=(np.full(n,False) if den==0 else
            row['error_count']/den<=b['A']['error_count']/den+g['each_phase_error_rate_excess_over_A_max'])
        flags[case+'_harmful_nonincrease']=row['harmful_count']<=b['A_harmful_count']
    eps=g['point_comparison_roundoff_allowance_mm']
    flags['point_RMS']=np.isfinite(metrics['point_RMS_mm'])&(metrics['point_RMS_mm']<=base['A']['point_RMS_mm']+eps)
    flags['point_p95']=np.isfinite(metrics['point_p95_mm'])&(metrics['point_p95_mm']<=base['A']['point_p95_mm']+eps)
    flags['point_maximum']=np.isfinite(metrics['point_maximum_mm'])&(metrics['point_maximum_mm']<=base['N']['point_maximum_mm']+eps)
    flags['nonempty_retention']=metrics['retained_proposal_count']>=g['retained_proposal_count_min']
    feasible=np.logical_and.reduce(list(flags.values()))
    return feasible,flags


def select_candidate(metrics,feasible,grid):
    options=np.flatnonzero(feasible)
    if not len(options):return dict(selection_failed=True,policy='off',grid_index=None,a_index=None,b_index=None,a_Nm=None,b_dimensionless=None)
    order=np.lexsort((grid['b_index'][options],grid['a_index'][options],metrics['veto_count'][options],
                      metrics['point_SSE_mm2'][options],metrics['error_count'][options]))
    i=int(options[order[0]])
    return dict(selection_failed=False,policy='numeric',grid_index=i,
        a_index=int(grid['a_index'][i]),b_index=int(grid['b_index'][i]),
        a_Nm=float(grid['a_Nm'][i]),b_dimensionless=float(grid['b_dimensionless'][i]))


def tradeoffs(metrics,base,protocol,grid):
    g=protocol['training_gates'];h=base['A_harmful_count'];u=base['A_useful_count']
    harm=(np.zeros(len(metrics['error_count']),bool) if not h else (h-metrics['harmful_count'])/h>=g['harmful_removal_fraction_min'])
    useful=(np.zeros(len(metrics['error_count']),bool) if not u else metrics['useful_retained_count']/u>=g['useful_retention_fraction_min'])
    def minimum(name,mask):return None if not np.any(mask) else float(np.min(metrics[name][mask]))
    def witness(mask):
        choice=select_candidate(metrics,mask,grid)
        return dict(selection=choice,metrics=None if choice['selection_failed'] else scalar_metrics(metrics,choice['grid_index']))
    frontier=[];best=-1
    for count in np.unique(metrics['harmful_count']):
        mask=metrics['harmful_count']==count;retained=int(np.max(metrics['useful_retained_count'][mask]))
        if retained<=best:continue
        best=retained;options=mask&(metrics['useful_retained_count']==retained)
        frontier.append(dict(harmful_count=int(count),useful_retained_count=retained,
            useful_retention=None if not u else retained/u,
            candidate_count=int(options.sum()),min_error_count=int(np.min(metrics['error_count'][options])),
            min_point_maximum_mm=minimum('point_maximum_mm',options)))
    point_safe=np.isfinite(metrics['point_maximum_mm'])&(metrics['point_maximum_mm']<=base['N']['point_maximum_mm']+g['point_comparison_roundoff_allowance_mm'])
    return dict(minimum_error_all_grid=witness(np.ones(len(harm),bool)),
        minimum_error_with_harmful_removal=witness(harm),minimum_error_with_point_maximum_safe=witness(point_safe),
        harmful_and_useful_jointly_qualified_count=int(np.sum(harm&useful)),
        minimum_harmful_count_with_useful_retention=minimum('harmful_count',useful),
        minimum_error_count_with_harmful_removal=minimum('error_count',harm),
        maximum_useful_retained_count_with_harmful_removal=None if not harm.any() else int(np.max(metrics['useful_retained_count'][harm])),
        minimum_point_maximum_with_harmful_and_useful=minimum('point_maximum_mm',harm&useful),
        harmful_useful_frontier=frontier)


def scalar_metrics(metrics,index=0):
    return {key:(int(value[index]) if np.issubdtype(value.dtype,np.integer) else float(value[index])) for key,value in metrics.items()}


def evaluate_selected(data_by_case,selections,protocol):
    kept={};phase={};bases={}
    for case,data in data_by_case.items():
        selection=selections[case]
        kept[case]=keep_proposal(data,selection.get('a_Nm'),selection.get('b_dimensionless'),off=selection['policy']=='off')
        selected=data['primary_eligible'];phase[case]=score_keeps(domain([data]),kept[case][selected]);bases[case]=constants(domain([data]))
    combined=domain(list(data_by_case.values()))
    combined_kept=np.concatenate([kept[case][data_by_case[case]['primary_eligible']] for case in data_by_case])
    aggregate=score_keeps(combined,combined_kept);base=constants(combined)
    passed,flags=training_feasibility(aggregate,base,phase,bases,protocol)
    return dict(aggregate=scalar_metrics(aggregate),baseline=base,
        per_case={case:dict(metrics=scalar_metrics(phase[case]),baseline=bases[case],
            harmful_removal_scorable=bool(bases[case]['A_harmful_count']>0)) for case in phase},
        gates={key:bool(value[0]) for key,value in flags.items()},all_pooled_gates_pass=bool(passed[0])),kept


def verify_benchmarks(data_by_case,grid,case_metrics):
    i=int(np.flatnonzero((grid['a_index']==6)&(grid['b_index']==20))[0]);result={}
    for case,data in data_by_case.items():
        keep=keep_proposal(data,.03,.10)
        if not np.array_equal(keep,data['i61_keep_proposal']):raise ValueError('Original I-61 gate flags not reproduced: '+case)
        family=np.where(keep,data['A_family'],data['N_family'])
        if not np.array_equal(family,data['i61_C_family']):raise ValueError('Original I-61 family policy differs: '+case)
        if not np.array_equal(keep_proposal(data,0.,0.,off=True),data['i60_accepted']):raise ValueError('Off policy is not A')
        selected=data['primary_eligible'];point=selected&data['point_mask']
        new_distance=np.where(keep,data['proposed_distance_m'],data['native_distance_m'])
        if not np.array_equal(new_distance[point],data['i61_distance_m'][point]):raise ValueError('I-61 distances differ: '+case)
        direct=score_keeps(domain([data]),keep[selected])
        counts={key:bool(case_metrics[case][key][i]==direct[key][0]) for key in direct if np.issubdtype(direct[key].dtype,np.integer)}
        floats={key:float(abs(case_metrics[case][key][i]-direct[key][0])) for key in direct if not np.issubdtype(direct[key].dtype,np.integer)}
        if not all(counts.values()) or any(value>1e-10 for value in floats.values()):raise ValueError('Grid and original baseline metric mismatch')
        result[case]=dict(I61_policy_flags_exact=True,I61_families_exact=True,I61_distances_exact=True,off_is_A=True,
            original_I61_C_primary_error_count=int(np.sum(selected&(data['i61_C_family']!=data['truth_family']))),
            integer_metrics_exact=counts,floating_metric_max_difference=floats)
    return result


def execute(data_by_case,protocol,output,chunk=128):
    grid=parameter_grid(protocol);arrays={key:value for key,value in grid.items()};groups={};case_metrics={};case_bases={}
    for case in CASES:
        data=domain([data_by_case[case]]);metrics=scan_group(data,grid,chunk);base=constants(data)
        case_metrics[case]=metrics;case_bases[case]=base
        arrays.update({case+'__'+key:value for key,value in metrics.items()});groups[case]=dict(baseline=base)
    benchmark=verify_benchmarks(data_by_case,grid,case_metrics)
    folds=[];oof_selections={}
    for heldout in CASES:
        training=[case for case in CASES if case!=heldout];name='train_without_'+heldout
        data=domain([data_by_case[case] for case in training]);metrics=scan_group(data,grid,chunk);base=constants(data)
        feasible,flags=training_feasibility(metrics,base,{case:case_metrics[case] for case in training},
            {case:case_bases[case] for case in training},protocol)
        selection=select_candidate(metrics,feasible,grid);oof_selections[heldout]=selection
        arrays.update({name+'__'+key:value for key,value in metrics.items()});arrays[name+'__feasible']=feasible
        arrays.update({name+'__gate_'+key:value for key,value in flags.items()})
        selected_eval,keep=evaluate_selected({heldout:data_by_case[heldout]},{heldout:selection},protocol)
        folds.append(dict(heldout=heldout,training=training,selection=selection,training_feasible_count=int(feasible.sum()),
            training_baseline=base,training_gate_failure_counts={key:int((~value).sum()) for key,value in flags.items()},
            training_selected_metrics=(base['A'] if selection['selection_failed'] else scalar_metrics(metrics,selection['grid_index'])),
            heldout_evaluation=selected_eval,training_tradeoffs=tradeoffs(metrics,base,protocol,grid)))
        print(json.dumps(dict(heldout=heldout,training_feasible_count=int(feasible.sum()),selection=selection)),flush=True)
    full=domain([data_by_case[case] for case in CASES]);metrics=scan_group(full,grid,chunk);base=constants(full)
    feasible,flags=training_feasibility(metrics,base,case_metrics,case_bases,protocol);selection=select_candidate(metrics,feasible,grid)
    arrays.update({'all__'+key:value for key,value in metrics.items()});arrays['all__feasible']=feasible
    arrays.update({'all__gate_'+key:value for key,value in flags.items()})
    oof,oof_keep=evaluate_selected(data_by_case,oof_selections,protocol)
    oof['all_folds_have_qualified_fit']=all(not fold['selection']['selection_failed'] for fold in folds)
    oof['success']=oof['all_folds_have_qualified_fit'] and oof['all_pooled_gates_pass']
    final,final_keep=evaluate_selected(data_by_case,{case:selection for case in CASES},protocol)
    final.update(selection=selection,feasible_count=int(feasible.sum()),descriptive_retrospective_only=True,
        tradeoffs=tradeoffs(metrics,base,protocol,grid),gate_failure_counts={key:int((~value).sum()) for key,value in flags.items()})
    np.savez_compressed(output/'all_grid_metrics.npz',**arrays)
    for case in CASES:
        np.savez_compressed(output/(case+'_selected_policies.npz'),time_s=data_by_case[case]['time_s'],
            oof_keep=oof_keep[case],full_data_keep=final_keep[case],
            oof_family=np.where(oof_keep[case],data_by_case[case]['A_family'],data_by_case[case]['N_family']),
            full_data_family=np.where(final_keep[case],data_by_case[case]['A_family'],data_by_case[case]['N_family']))
    with (output/'all_grid_case_metrics.csv').open('w',newline='') as stream:
        fields=('case','policy','a_index','b_index','a_Nm','b_dimensionless')+tuple(case_metrics[CASES[0]])
        writer=csv.DictWriter(stream,fieldnames=fields,lineterminator='\n');writer.writeheader()
        for case in CASES:
            for i in range(len(grid['a_index'])):
                writer.writerow(dict(case=case,policy='numeric',**{k:safe(v[i]) for k,v in grid.items()},
                    **{k:safe(v[i]) for k,v in case_metrics[case].items()}))
            row={name:None for name in fields};row.update(case=case,policy='off',**case_bases[case]['A']);writer.writerow(row)
    science=dict(iteration='I-62',numpy_version=np.__version__,numeric_grid_count=len(grid['a_index']),
        explicit_off_baseline=True,groups=groups,benchmarks=benchmark,folds=folds,out_of_fold=oof,full_data_fit=final,
        scope='Retrospective grouped selection on already examined episodes, not unseen validation or physical trials. One global pair for any final policy; fold-specific pairs are validation-only.',
        source_case_order=list(CASES),new_physics_steps=0,live_controller_changed=False)
    save(output/'science.json',science)
    return science


def run(protocol_path,data_source,output,chunk):
    protocol=json.loads(protocol_path.read_text());output.mkdir(parents=True,exist_ok=False)
    if protocol['iteration']!='I-62':raise ValueError('Expected I-62')
    manifest=json.loads((data_source/'artifact_manifest.json').read_text())
    for item in manifest['files']:
        if sha(data_source/item['path'])!=item['sha256']:raise ValueError('Changed compact input')
    save(output/'registered_before_sweep.json',dict(iteration='I-62',registered_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),protocol_sha256=sha(protocol_path),input_manifest_sha256=sha(data_source/'artifact_manifest.json'),
        numpy_version=np.__version__,chunk_size=chunk,case_order=list(CASES)))
    shutil.copy2(Path(__file__),output/'tolerance_sweep.py');shutil.copy2(protocol_path,output/'protocol.json')
    data={case:load(data_source/(case+'.npz')) for case in CASES};science=execute(data,protocol,output,chunk)
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and '__pycache__'not in p.parts and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(oof_success=science['out_of_fold']['success'],full_data_feasible_count=science['full_data_fit']['feasible_count'],
        full_data_selection=science['full_data_fit']['selection'],new_physics_steps=0)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--data-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--chunk-size',type=int,default=128);a=p.parse_args();run(a.protocol.resolve(),a.data_source.resolve(),a.output.resolve(),a.chunk_size)
