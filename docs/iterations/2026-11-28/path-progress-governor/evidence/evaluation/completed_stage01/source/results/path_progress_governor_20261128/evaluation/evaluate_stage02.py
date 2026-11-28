"""I78 canonical stage02 sealed evaluation: checked-mask/path bookkeeping fixes.

All numeric thresholds and metric denominators are fixed before physics.
Original-clock timing remains secondary to geometric path/goal completion.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import numpy as np
import metrics_stage02 as metrics

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
FRAGMENT=Path(__file__).with_name('protocol_metrics_fragment.json')
ALIASES=dict(time_s='time_s',qpos='exact_qpos',qvel='exact_qvel',
    reference_xy='exact_reference_xy',reference_velocity='exact_reference_velocity_m_s',
    reference_acceleration='exact_reference_acceleration_m_s2',reference_quat='exact_reference_quat',
    requested='exact_requested',applied='exact_applied',previous_applied='exact_previous_applied',
    caps='exact_input_bounds',held='command_held_after_row',
    observer_update_count='native_observer_update_count',
    no_truth_read='no_truth_or_terrain_read_during_command',
    prior_matches_physical='mirror_previous_applied_matches_physical',heading_command='exact_heading_command')
GOVERNOR_ALIASES=dict(virtual_time_s='gov__virtual_time_s',u='gov__progress_u',
    udot='gov__progress_rate_s_inv',uddot='gov__progress_acceleration_s_inv2',
    clock_rate='gov__clock_rate',clock_acceleration='gov__clock_acceleration_s_inv')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def safe(value):
    if isinstance(value,dict):return{str(k):safe(v)for k,v in value.items()}
    if isinstance(value,(list,tuple)):return[safe(v)for v in value]
    if isinstance(value,np.ndarray):return safe(value.tolist())
    if isinstance(value,np.generic):return safe(value.item())
    if isinstance(value,float)and not np.isfinite(value):return None
    return value
def write(path,value):
    with Path(path).open('x')as f:json.dump(safe(value),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def load(path):
    with np.load(path,allow_pickle=False)as z:return{k:z[k].copy()for k in z.files}
def identical(a,b):
    a,b=np.asarray(a),np.asarray(b)
    return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()

def paths_for(base,protocol,job):
    cid=job['case_id'];native=cid.rsplit('_',1)[0]+'_N'
    case=base/protocol['run_root']/cid
    child=case/'child/campaign/runs'/native
    return dict(case=case,precision=child/'precision.npz',sidecar=case/'governor_diagnostics.npz',
        outcome=case/'i78_outcome.json',summary=child/'summary.json',provenance=child/'provenance.json',
        historical=ROOT/'results/tangent_heading_routes_20261127/campaign/runs'/native/'precision.npz')

def protocol_check(protocol):
    if protocol['iteration']!='I-78' or protocol['run_root']!='campaign/runs':
        raise ValueError('Frozen I78 campaign schema required')
    if protocol['evaluation']!=json.loads(FRAGMENT.read_text()):
        raise ValueError('Evaluation thresholds/definitions differ from approved fragment')
    expected={(s,p)for s in (-1.,.3,.9)for p in ('N','G')}
    jobs=protocol['cases']
    if len(jobs)!=6 or len({j['case_id']for j in jobs})!=6 or {(j['phase_rad'],j['policy'])for j in jobs}!=expected:
        raise ValueError('Exactly three straight N/G phase pairs required')
    for j in jobs:
        if j['route_id']!='straight' or j['yaw_deg']!=113. or j['goal_hold_s']!=10.:
            raise ValueError('Only frozen straight geometry/yaw/10s hold permitted')
        if j['maximum_duration_s']!=(45. if j['policy']=='N' else 75.):
            raise ValueError('Native45s and governed75s budget required')
    return jobs

def release_check(base,protocol,seal,jobs,protocol_path):
    if seal['protocol_sha256']!=sha(protocol_path):
        raise ValueError('Final outputs are not bound to registered I78 protocol')
    files=seal['files']
    for rel,bound in files.items():
        p=base/rel
        if sha(p)!=bound['sha256'] or p.stat().st_size!=bound['bytes']:
            raise ValueError('Final output changed: '+rel)
    if 'processes' in seal:
        processes=seal['processes']
        if ({p['case_id']for p in processes}!={j['case_id']for j in jobs}
                or any(p['returncode']!=0 for p in processes)):
            raise ValueError('Every registered process must have observed successful exit')
    for job in jobs:
        paths=paths_for(base,protocol,job)
        for key in ('precision','sidecar','outcome','summary','provenance'):
            rel=str(paths[key].relative_to(base))
            if rel not in files:raise ValueError('Essential output not sealed: '+rel)
    for group in ('source_sha256','input_sha256'):
        for rel,bound in protocol[group].items():
            if sha(ROOT/rel)!=bound:raise ValueError('Registered source/input changed: '+rel)

def canonical(raw,side):
    np.testing.assert_array_equal(raw['time_s'],side['time_s'])
    packet={k:np.asarray(raw[v]).copy()for k,v in ALIASES.items()}
    packet.update({k:np.asarray(side[v]).copy()for k,v in GOVERNOR_ALIASES.items()})
    n=len(packet['time_s']);shapes=dict(qpos=(n,10),qvel=(n,9),reference_xy=(n,2),
        reference_velocity=(n,2),reference_acceleration=(n,2),reference_quat=(n,4),
        requested=(n,5),applied=(n,5),previous_applied=(n,5),caps=(n,5),heading_command=(n,5))
    for key,value in packet.items():
        expected=shapes.get(key,(n,))
        if value.shape!=expected:raise ValueError('Incorrect field shape: '+key)
    return packet

def row_bitwise_equal(a,b):
    a,b=np.asarray(a),np.asarray(b)
    if a.ndim<1 or a.shape!=b.shape or a.dtype!=b.dtype:
        return np.zeros(len(a),bool)
    av=np.ascontiguousarray(a).view(np.uint8).reshape(len(a),-1)
    bv=np.ascontiguousarray(b).view(np.uint8).reshape(len(b),-1)
    return np.all(av==bv,axis=1)


def governor_checks(packet,side,outcome,job):
    checks=[]
    def add(name,okay,**detail):checks.append(dict(name=name,passed=bool(okay),**detail))
    t=packet['time_s'];tau=packet['virtual_time_s'];lam=packet['clock_rate'];alpha=packet['clock_acceleration']
    u,du,ddu,_,_,_=metrics.native_reference(tau)
    expected=(u,du*lam,ddu*lam*lam+du*alpha)
    for name,actual,wanted in zip(('u','udot','uddot'),(packet['u'],packet['udot'],packet['uddot']),expected):
        residual=float(np.max(abs(actual-wanted)))
        add('coherent_clock_chain_'+name,np.isfinite(residual)and residual<=1e-10,max_error=residual)
    add('clock_rate_bounds_and_finiteness',np.isfinite(np.r_[tau,lam,alpha]).all()
        and np.all((lam>=-1e-12)&(lam<=1.+1e-12)))
    add('selection_truth_boundary',side['no_physical_contact_or_terrain_read_during_selection'].all())
    expected_probe=np.full(len(t),job['policy']=='G',bool)
    present=np.asarray(side['private_probe_present'],bool)
    checked=np.asarray(side['live_probe_sensor_stages_checked'],bool)
    match=np.asarray(side['live_probe_sensor_stages_match'],bool)
    add('private_probe_presence',np.array_equal(present,expected_probe))
    add('probe_sensor_check_mask',np.array_equal(checked,expected_probe))
    add('probe_sensor_stages_match_live',match[checked].all() and not match[~checked].any(),
        checked_rows=int(checked.sum()),unchecked_rows=int((~checked).sum()))
    same=np.ones(len(t),bool)
    for actual,nominal in [('governed_xy','nominal_virtual_xy'),
        ('governed_velocity_m_s','nominal_virtual_velocity_m_s'),
        ('governed_acceleration_m_s2','nominal_virtual_acceleration_m_s2')]:
        same &= row_bitwise_equal(side[actual],side[nominal])
    full_checked=np.asarray(side['full_output_identity_checked'],bool)
    full_match=np.asarray(side['full_output_identity_when_references_equal'],bool)
    add('equal_reference_full_output_check_mask',np.array_equal(full_checked,expected_probe&same))
    add('equal_reference_probe_full_output_identity',full_match[full_checked].all()
        and not full_match[~full_checked].any(),checked_rows=int(full_checked.sum()))
    for canonical_key,side_key in [('reference_xy','governed_xy'),('reference_velocity','governed_velocity_m_s'),
                                  ('reference_acceleration','governed_acceleration_m_s2')]:
        add('sidecar_matches_actual_'+canonical_key,row_bitwise_equal(packet[canonical_key],side[side_key]).all())
    add('no_private_dynamic_integration',np.all(side['private_dynamic_steps']==0))
    expected_probes=0 if job['policy']=='N' else 1
    add('private_observer_updates_only_in_G_probe',np.all(side['private_observer_updates']==expected_probes))
    probe_counts=np.zeros(len(t),int) if job['policy']=='N' else np.arange(1,len(t)+1)
    add('nominal_probe_count',np.array_equal(side['gov__nominal_probe_calls'],probe_counts))
    add('same_final_held_mask',np.array_equal(side['command_held_after_row'],packet['held']))
    arrival=outcome['first_complete_sample_s']
    complete=np.asarray(side['gov__reference_complete'],bool)
    stamps=np.asarray(side['gov__first_complete_sample_s'],float)
    if arrival is None:
        add('no_invented_arrival',not complete.any()and np.isnan(stamps).all())
    else:
        arrival=float(arrival);idx=np.flatnonzero(abs(t-arrival)<=1e-9)
        expected_complete=t>=arrival-1e-9
        add('first_complete_sample_time',len(idx)==1 and np.array_equal(complete,expected_complete))
        add('first_complete_stamp_history',np.isnan(stamps[~expected_complete]).all()
            and np.all(abs(stamps[expected_complete]-arrival)<=1e-9))
        dense=outcome.get('finished_at_s')
        add('dense_completion_is_diagnostic_only',dense is not None and np.isfinite(dense)
            and dense<=arrival+1e-9 and arrival-dense<=metrics.DT+1e-9)
    planned_stop=min(job['maximum_duration_s'],arrival+10.)if arrival is not None else job['maximum_duration_s']
    add('physical_stop_matches_preplanned_rule',abs(t[-1]-planned_stop)<=1e-8
        or outcome.get('native_failure')is not None,
        actual_end_s=float(t[-1]),expected_end_s=float(planned_stop))
    add('outcome_clock_matches_arrays',abs(outcome['actual_end_s']-t[-1])<=1e-9)
    return checks,arrival,planned_stop

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--protocol',type=Path,default=BASE/'protocol/protocol.json')
    ap.add_argument('--seal',type=Path,default=BASE/'campaign/outputs_sealed.json')
    ap.add_argument('--output',type=Path,default=BASE/'evaluation/completed_stage01')
    args=ap.parse_args();base=args.protocol.resolve().parent;out=args.output.resolve()
    if base.name=='protocol':base=base.parent
    protocol=json.loads(args.protocol.read_text());jobs=protocol_check(protocol)
    seal=json.loads(args.seal.read_text());release_check(base,protocol,seal,jobs,args.protocol.resolve())
    inputs=[args.protocol.resolve(),args.seal.resolve(),FRAGMENT,
            Path(__file__).with_name('mask_correction_registered_before_G')/'receipt.json',
            Path(__file__).with_name('arrival_boundary_correction_registered_before_arrays')/'receipt.json']
    old_seal_path=ROOT/'results/tangent_heading_routes_20261127/campaign/outputs_sealed.json'
    old_seal=json.loads(old_seal_path.read_text());inputs.append(old_seal_path)
    for job in jobs:
        paths=paths_for(base,protocol,job)
        inputs.extend(paths[k]for k in ('precision','sidecar','outcome','summary','provenance'))
        if job['policy']=='N':
            p=paths['historical'];bound=old_seal['files'][str(p.relative_to(p.parents[3]))]
            if sha(p)!=bound['sha256']or p.stat().st_size!=bound['bytes']:raise ValueError('I76 original baseline changed')
            inputs.append(p)
    sources=[Path(__file__).resolve(),Path(metrics.__file__).resolve()]
    bindings={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in set(inputs+sources)}
    out.mkdir(parents=True,exist_ok=False)
    for source in sources:
        dest=out/'source'/source.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
    write(out/'registered_before_arrays.json',dict(iteration='I-78',utc=datetime.now(timezone.utc).isoformat(),
        files=bindings,protocol_sha256=sha(args.protocol),output_seal_sha256=sha(args.seal),
        no_new_case_arrays_read_before_registration=True,no_new_physics=True,
        canonical_analysis_stage='stage02 checked-mask/protocol-base/arrival-roundoff corrections before array reads; numerical performance gates unchanged',
        evaluation='First complete traversal plus first10s goalhold. Originalclockerror issecondary; progress/time/peak/stalldenominators explicit. Allthreephasepairs retained.'))
    rows={};signals={};setup=[];raws={}
    for job in jobs:
        cid=job['case_id'];paths=paths_for(base,protocol,job)
        raw,side=load(paths['precision']),load(paths['sidecar']);raws[cid]=raw
        packet=canonical(raw,side);summary=json.loads(paths['summary'].read_text());outcome=json.loads(paths['outcome'].read_text())
        checks,arrival,planned_stop=governor_checks(packet,side,outcome,job)
        setup.extend({**c,'name':cid+'_'+c['name']}for c in checks)
        effective_summary=dict(completed=bool(outcome['survived_without_native_failure']
            and abs(packet['time_s'][-1]-planned_stop)<=1e-8),failure=outcome.get('native_failure'))
        cfg={**protocol['evaluation'],'max_duration_s':job['maximum_duration_s']}
        row,sig=metrics.case_metrics(packet,effective_summary,cfg,arrival)
        row.update(case=job,inherited_summary_completed=summary.get('completed'),
            inherited_requested_horizon_s=summary.get('requested_duration_s'),
            authoritative_path_outcome=outcome)
        rows[cid]=row;signals[cid]=sig
        for name in ('grid_ok','held_mask_ok','reference_consistency_ok','finite_reference','progress_valid','virtual_clock_valid','native_measurement_boundary_ok'):
            setup.append(dict(name=cid+'_'+name,passed=bool(row[name])))
        if job['policy']=='N':
            previous=load(paths['historical']);checks=[]
            for field in sorted(set(raw)|set(previous)):
                okay=field in raw and field in previous and identical(raw[field],previous[field])
                checks.append(dict(field=field,bit_exact=okay))
            setup.append(dict(name=cid+'_all_original_precision_fields_bit_exact',passed=bool(checks and all(c['bit_exact']for c in checks))))
            write(out/(cid+'_native_replay.json'),dict(shared_fields=len(checks),rows=len(raw['time_s']),all_bit_exact=all(c['bit_exact']for c in checks),checks=checks))
        np.savez_compressed(out/(cid+'_signals.npz'),**sig)
    for phase in (-1.,.3,.9):
        jobs_by_policy={j['policy']:j for j in jobs if j['phase_rad']==phase}
        n,g=(raws[jobs_by_policy[p]['case_id']]for p in ('N','G'))
        count=int(round(3./metrics.DT))+1
        same_fields=set(n)==set(g)
        prefix=same_fields and all(identical(n[k][:count],g[k][:count])for k in n)
        setup.append(dict(name=str(phase)+'_all_field_initial_three_second_native_prefix',passed=bool(prefix),samples=count))
    grouped={(j['phase_rad'],j['policy']):rows[j['case_id']]for j in jobs}
    decision=metrics.decide(grouped,protocol['evaluation'],setup_valid=all(c['passed']for c in setup))
    for rel,bound in bindings.items():
        if sha(ROOT/rel)!=bound['sha256']or (ROOT/rel).stat().st_size!=bound['bytes']:raise AssertionError('Input/source changed during evaluation: '+rel)
    write(out/'report.json',dict(iteration='I-78',cases=rows,setup_checks=setup,decision=decision,
        no_new_physics=True,source_inputs_unchanged=True,no_automatic_adoption=True,
        canonical_analysis_stage='stage02',registered_original_analysis_retained=True,
        progress_completion_separate_from_physical_goal=True,full_repo_suite_claim=False,
        scope='Six fresh declared cases; only pathprogresschanges. Passing narrowerstraightcase screen isnotallterrain/globalstability evidence.'))
    write(out/'artifact_manifest.json',dict(files={str(p.relative_to(out)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in sorted(out.rglob('*'))if p.is_file()}))
    print(json.dumps(safe(dict(status=decision['status'],setup_checks=len(setup),passed_setup=sum(c['passed']for c in setup),
        cases={cid:dict(arrival_s=r['declared_reference_arrival_s'],goal_converged=r['settling']['physical_position_passed'],
            geometric_peak_m=r['tracking']['geometric_traversal']['maximum'],
            progress_rms_m=r['tracking']['geometric_progress_rms']['rms'])for cid,r in rows.items()}))),flush=True)

if __name__=='__main__':main()
