"""Read-only I57 paired response reduction; never imports simulator/controller.

Freeze source before physics:
  .venv/bin/python .../analysis/reduce_responses.py --freeze PATH
Reduce a sealed/partial campaign (preserving missing cases):
  .venv/bin/python .../analysis/reduce_responses.py --study PATH --model-stage PATH --output NEW_PATH --frozen-source PATH
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,csv,hashlib,json,shutil,sys
import numpy as np
import response_metrics as arithmetic

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
ROLES=dict(native='native_center_amplitude_only_counterfactual',governed='governed_center',
    duplicate='deterministic_duplicate_control',midpoint='native_governed_midpoint_amplitude_only_counterfactual',
    continuation='exact_recorded_policy_control',column='governed_center_column_probe')
SOURCES=('response_metrics.py','reduce_responses.py','plot_responses.py','check_arithmetic.py')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def now():return datetime.now(timezone.utc).isoformat()
def safe(x):
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {str(k):safe(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v) for v in x]
    if isinstance(x,Path):return str(x)
    if isinstance(x,float) and not np.isfinite(x):return None
    return x

def save(path,value):Path(path).write_text(json.dumps(safe(value),indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def exact(a,b):
    a,b=np.asarray(a),np.asarray(b)
    return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
def manifest(output):
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))

def freeze(output):
    output.mkdir(parents=True,exist_ok=False)
    for name in SOURCES:shutil.copyfile(HERE/name,output/name)
    save(output/'registered_before_outcomes.json',dict(registered_utc=now(),scope='Freeze I57 pure paired-endpoint reduction, scoring, plots and synthetic checks before consuming new native outcomes. No physics.',
        source_sha256={name:sha(HERE/name) for name in SOURCES},groups=arithmetic.GROUPS,thresholds=arithmetic.THRESHOLDS,
        contact_mode_load_threshold_N=.05,contact_scope='Five endpoint last-solve samples only, not continuity within5ms or integrated impulses.',
        control_requirement='Every completed branch prefix bitwise matched; original continuation matches savedfuture; governed duplicate matches states, inputhistory and raw contact rows.',
        inference_scope='Finite matched-command physical response at two selected states, not a full I56 drift cause, derivative proof, stability or adoption.'))
    manifest(output)


def model_path(model_stage,index):
    found=[]
    for p in model_stage.glob('*/predictions.json'):
        if json.loads(p.read_text()).get('saved_index')==index:found.append(p.parent)
    if len(found)!=1:raise ValueError(f'Expected unique frozen prediction at index{index}: {found}')
    return found[0]


def contacts(endpoint):
    offsets=endpoint['contact_offsets'];rows=endpoint['contact_rows'];names=('shell_A','shell_B','belt_A','belt_B','shell_A','shell_B','body')
    answer=[]
    for k in range(5):
        values=rows[offsets[k]:offsets[k+1]];loads={name:0. for name in ('shell_A','shell_B','belt_A','belt_B','body')}
        counts={name:0 for name in loads}
        for row in values:
            if row[12]<.5:continue
            name=names[int(row[0])];force=max(0.,float(row[4:7]@row[7:10]));loads[name]+=force;counts[name]+=1
        answer.append(dict(offset_s=float(endpoint['offset_s'][k]),normal_loads_N=loads,admitted_counts=counts,
            active_support=[name for name,value in loads.items() if value>.05],threshold_N=.05))
    return answer


def reduce(study,model_stage,output,frozen_source):
    binding=json.loads((frozen_source/'registered_before_outcomes.json').read_text())
    for name,digest in binding['source_sha256'].items():
        if sha(HERE/name)!=digest or sha(frozen_source/name)!=digest:raise ValueError('Changed frozen analysis source: '+name)
    if output.exists():raise FileExistsError('Preserve earlier reduction attempts')
    output.mkdir(parents=True)
    protocol=json.loads((study/'protocol.json').read_text())
    candidates=[]
    for job in protocol['cases']:
        folder=study/'campaign/runs'/job['case_id']
        candidates += [p for p in (folder/'endpoints.npz',folder/'prefix.npz',folder/'provenance.json') if p.exists()]
    models={event['event_id']:model_path(model_stage,event['index']) for event in protocol['events']}
    input_paths=[study/'protocol.json',ROOT/protocol['source_data']['precision_path']]+candidates
    input_paths += [p/name for p in models.values() for name in ('matrices.npz','predictions.json')]
    hashes={str(p):sha(p) for p in input_paths}
    save(output/'registered_before_arithmetic.json',dict(registered_utc=now(),frozen_analysis=str(frozen_source),frozen_manifest_sha256=sha(frozen_source/'artifact_manifest.json'),input_sha256=hashes,
        no_physics=True,scope='Saved endpoint arrays only; missing/failed branches remain explicit. No new controller/model/observer calls.'))
    snap=output/'source_snapshot';snap.mkdir()
    for name in SOURCES:shutil.copyfile(HERE/name,snap/name)
    shutil.copyfile(study/'protocol.json',output/'protocol.json')
    source=load(ROOT/protocol['source_data']['precision_path'])
    portable=output/'portable_inputs';portable.mkdir()
    report=dict(iteration='I-57',groups=arithmetic.GROUPS,thresholds=arithmetic.THRESHOLDS,events={},available_cases=[],missing_cases=[],no_new_physics=True,
        inference_scope='Finite open-loop matched-input responses at selected saved states. No full-trajectory cause, true derivative, stability, friction guarantee or controller adoption established.',
        angular_semantics='Endpoint angular velocities rotated separately into world, subtracted, then expressed in the common initial fork frame. Last componentworldyaw.',
        contact_semantics='Raw last-solve contacts at five endpoints; changes retained. No assertion about contact continuity inside5ms or integrated impulse.')
    flat_rows=[];linear_rows=[];paired_arrays={}
    for event in protocol['events']:
        event_id,index=event['event_id'],event['index'];jobs=[j for j in protocol['cases'] if j['event_id']==event_id];branches={};prefix_checks={};all_prefix=True
        for job in jobs:
            cid=job['case_id'];folder=study/'campaign/runs'/cid
            if not (folder/'endpoints.npz').exists() or not (folder/'provenance.json').exists():report['missing_cases'].append(cid);continue
            metadata=json.loads((folder/'provenance.json').read_text());endpoint=load(folder/'endpoints.npz')
            if not metadata.get('success') or len(endpoint.get('time_s',[]))!=5:
                report['missing_cases'].append(cid);prefix_checks[cid]=dict(success=False,reason='failed_or_incomplete_branch');all_prefix=False;continue
            prefix=load(folder/'prefix.npz');flags={k:exact(prefix[k],source[k][:index+1]) for k in ('time_s','exact_qpos','exact_qvel','exact_previous_applied')}
            endpoint_match={k:exact(endpoint[k][0],source[k][index]) for k in flags}
            prefix_ok=all(flags.values()) and all(endpoint_match.values());all_prefix &= prefix_ok
            prefix_checks[cid]=dict(success=prefix_ok,prefix_fields=flags,anchor_fields=endpoint_match,prefix_sha256=sha(folder/'prefix.npz'))
            endpoint['job_json_utf8']=np.frombuffer(json.dumps(job,sort_keys=True).encode(),np.uint8)
            np.savez_compressed(portable/(cid+'_endpoints.npz'),**endpoint)
            branches[cid]=dict(job=job,endpoint=endpoint,metadata=metadata,contacts=contacts(endpoint));report['available_cases'].append(cid)
        model=load(models[event_id]/'matrices.npz');mp=json.loads((models[event_id]/'predictions.json').read_text());np.savez_compressed(portable/(event_id+'_model.npz'),**model)
        sfields={k:source[k][index:index+5].copy() for k in ('time_s','exact_qpos','exact_qvel','exact_previous_applied','exact_applied')};np.savez_compressed(portable/(event_id+'_original_continuation.npz'),**sfields)
        def role(name):
            choices=[b for b in branches.values() if b['job']['role']==ROLES[name]]
            if len(choices)>1:raise ValueError('Role not unique: '+name)
            return choices[0] if choices else None
        continuation,gov,dup,native,mid=(role(x) for x in ('continuation','governed','duplicate','native','midpoint'))
        continuation_exact=None if continuation is None else all(exact(continuation['endpoint'][k],sfields[k]) for k in ('time_s','exact_qpos','exact_qvel','exact_previous_applied'))
        duplicate_exact=None if gov is None or dup is None else all(exact(gov['endpoint'][k],dup['endpoint'][k]) for k in ('time_s','exact_qpos','exact_qvel','exact_previous_applied','actual_applied','bridge_state','contact_rows','contact_offsets','command_sequence'))
        controls=dict(all_available_prefixes_exact=all_prefix,prefix_checks=prefix_checks,continue_saved_exact=continuation_exact,governed_duplicate_exact=duplicate_exact,
            required_controls_available=all(x is not None for x in (continuation,gov,dup)))
        controls['valid_for_interpretation']=bool(all_prefix and continuation_exact is True and duplicate_exact is True)
        ereport=dict(saved_index=index,saved_time_s=event['time_s'],controls=controls,branches={cid:dict(role=b['job']['role'],contacts=b['contacts'],first_slew_violation=b['metadata'].get('first_slew_violation')) for cid,b in branches.items()},comparisons=[],linearity=[])
        pairs=[]
        if native is not None and gov is not None:pairs.append(('native_to_governed',gov,native,'actual_allocation_finite_secant'))
        if native is not None and mid is not None:pairs.append(('native_to_midpoint',mid,native,'actual_allocation_half_secant'))
        columns={}
        for b in branches.values():
            if b['job']['role']==ROLES['column']:
                column=int(b['job']['input_column']);amplitude=b['job']['amplitude'];columns[(column,amplitude)]=b
                if gov is not None:pairs.append((f'u{column}_{amplitude}',b,gov,'one_sided_finite_column_probe'))
        observed_by_pair={}
        for pair_id,a,b,kind in pairs:
            ua,ub=a['endpoint']['command_sequence'],b['endpoint']['command_sequence']
            if not np.all(ua==ua[0]) or not np.all(ub==ub[0]):raise ValueError('Responsepair input is not a constant hold')
            du=ua[0]-ub[0]
            for endpoint_index,horizon_key in ((1,'h005ms'),(4,'h020ms')):
                R=model[horizon_key+'_physical_map'][3:6,3:6]
                observed=arithmetic.physical_difference(a['endpoint']['exact_qpos'][endpoint_index],a['endpoint']['exact_qvel'][endpoint_index],b['endpoint']['exact_qpos'][endpoint_index],b['endpoint']['exact_qvel'][endpoint_index],R)
                pred=arithmetic.prediction(model[horizon_key+'_response'],du,R);free=arithmetic.prediction(model[horizon_key+'_free_response'],du,R)
                scores=arithmetic.score_groups(pred,observed);free_scores=arithmetic.score_groups(free,observed)
                initial_mode=a['contacts'][0]['active_support'];amode=a['contacts'][endpoint_index]['active_support'];bmode=b['contacts'][endpoint_index]['active_support']
                result=dict(pair_id=pair_id,kind=kind,horizon_s=endpoint_index*.005,branch_a=a['job']['case_id'],branch_b=b['job']['case_id'],actual_delta_input=du,
                    predicted=pred,free_predicted=free,observed=observed,groups=scores,free_groups=free_scores,valid_for_interpretation=controls['valid_for_interpretation'],
                    contact_initial_same=initial_mode==b['contacts'][0]['active_support'],contact_modes_at_endpoint=dict(initial=initial_mode,branch_a=amode,branch_b=bmode),
                    endpoint_support_changed=amode!=initial_mode or bmode!=initial_mode,pair_endpoint_support_differs=amode!=bmode)
                ereport['comparisons'].append(result);observed_by_pair[(pair_id,horizon_key)]=observed
                key=event_id+'__'+pair_id+'__'+horizon_key;paired_arrays[key+'__observed']=observed;paired_arrays[key+'__predicted']=pred;paired_arrays[key+'__free_predicted']=free;paired_arrays[key+'__delta_input']=du
                for group,score in scores.items():
                    flat_rows.append(dict(event_id=event_id,pair_id=pair_id,horizon_ms=round(endpoint_index*5),group=group,units=score['units'],
                        valid_for_interpretation=controls['valid_for_interpretation'],endpoint_support_changed=result['endpoint_support_changed'],
                        **{k:score[k] for k in ('predicted_norm','observed_norm','floor','absolute_error','relative_vector_error','directional_cosine','observed_over_predicted_gain','status','passes_screen','transverse_nearzero_absolute_pass')}))
        for full_id,half_id in [('native_to_governed','native_to_midpoint')]+[(f'u{k}_full',f'u{k}_half') for k in range(5)]:
            for horizon_key in ('h005ms','h020ms'):
                if (full_id,horizon_key) not in observed_by_pair or (half_id,horizon_key) not in observed_by_pair:continue
                entry=dict(full_pair=full_id,half_pair=half_id,horizon_s=.005 if horizon_key=='h005ms' else .02,
                    valid_for_interpretation=controls['valid_for_interpretation'],groups=arithmetic.linearity(observed_by_pair[(full_id,horizon_key)],observed_by_pair[(half_id,horizon_key)]))
                ereport['linearity'].append(entry)
                for group,score in entry['groups'].items():linear_rows.append(dict(event_id=event_id,full_pair=full_id,half_pair=half_id,horizon_ms=5 if horizon_key=='h005ms' else 20,group=group,valid_for_interpretation=controls['valid_for_interpretation'],**{k:v for k,v in score.items() if k not in ('full_observed','twice_half_observed')}))
        report['events'][event_id]=ereport
    np.savez_compressed(output/'paired_responses.npz',**paired_arrays)
    for name,rows in [('response_scores.csv',flat_rows),('linearity_scores.csv',linear_rows)]:
        with (output/name).open('w',newline='') as stream:
            if rows:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    report['all_registered_cases_available']=not report['missing_cases'];report['completed_utc']=now();report['all_event_controls_valid']=all(e['controls']['valid_for_interpretation'] for e in report['events'].values())
    save(output/'report.json',report)
    from plot_responses import plot
    plot(output)
    if hashes!={str(p):sha(p) for p in input_paths}:raise ValueError('Input changed during analysis')
    save(output/'input_hashes_unchanged.json',dict(passed=True,input_sha256=hashes));manifest(output)
    print(json.dumps(dict(output=str(output),available_cases=len(report['available_cases']),missing_cases=len(report['missing_cases']),all_event_controls_valid=report['all_event_controls_valid'],physics_executions=0)))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--freeze',type=Path);parser.add_argument('--study',type=Path);parser.add_argument('--model-stage',type=Path);parser.add_argument('--output',type=Path);parser.add_argument('--frozen-source',type=Path);args=parser.parse_args()
    if args.freeze:freeze(args.freeze.resolve());return
    if any(x is None for x in (args.study,args.model_stage,args.output,args.frozen_source)):parser.error('Reduction requires --study --model-stage --output --frozen-source')
    reduce(args.study.resolve(),args.model_stage.resolve(),args.output.resolve(),args.frozen_source.resolve())

if __name__=='__main__':main()
