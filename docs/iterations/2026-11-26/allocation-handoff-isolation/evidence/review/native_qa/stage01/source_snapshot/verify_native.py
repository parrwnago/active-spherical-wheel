"""Independent I51 saved-native integrity and modifier-contract QA, no physics.

The source helpers are not imported or called. Reference and handoff projection
rules are reconstructed independently from protocol settings and saved current/
past inputs. Completion and scientific benefit are reported separately from
data integrity. Run only after all four case manifests have been written.
"""
from pathlib import Path
import argparse
import ast
import hashlib
import json
import shutil

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
FAMILY=ROOT/'results/allocation_handoff_isolation_20261126'
CASES=('base030','reference030','base060','handoff060')
TERMS=('equilibrium_command','rolling_feedforward','support_command','lqr_command','transition_command_delta','heading_command','fade_command_delta')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def load(p):
    with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def gap(a,b):return float(np.max(abs(np.asarray(a)-np.asarray(b))))if np.size(a)else 0.
def same(a,b):return a.shape==b.shape and a.dtype==b.dtype and a.tobytes()==b.tobytes()
def identity(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',action='store_true');parser.add_argument('--tag',default='stage01');args=parser.parse_args()
    if not args.run:raise SystemExit('Use --run only after all four cases are sealed')
    folders={case:FAMILY/'campaign/runs'/case for case in CASES}
    for case,path in folders.items():
        if not(path/'artifact_manifest.json').is_file():raise FileNotFoundError(f'Case not sealed: {case}')
    out=HERE/args.tag;out.mkdir(exist_ok=False)
    protocol=read(FAMILY/'protocol.json');source=Path(__file__).resolve()
    paths=[source,FAMILY/'protocol.json',FAMILY/'run_isolation_case.py',ROOT/'scripts/run-lqr-contact-forecast.py']
    paths += [ROOT/protocol['inputs'][name]for name in ('i49_base030_precision','i49_base060_precision')]
    for path in folders.values():paths += [path/name for name in ('artifact_manifest.json','precision.npz','rollout.npz','summary.json','provenance.json','candidate_diagnostics.json','execution_result.json')]
    input_hashes={str(p.relative_to(ROOT)):sha(p)for p in paths}
    snap=out/'source_snapshot';snap.mkdir()
    for p in [source,FAMILY/'protocol.json',FAMILY/'run_isolation_case.py',ROOT/'scripts/run-lqr-contact-forecast.py']:
        dest=snap/p.name;shutil.copy2(p,dest)
    (out/'before_checks.json').write_text(json.dumps({'scope':__doc__,'input_sha256':input_hashes,'no_new_physics_or_controller':True},indent=2)+'\n')
    source_checks={str(p):sha(ROOT/p)==h for p,h in protocol['source_sha256'].items()}
    old_checks={name:sha(ROOT/protocol['inputs'][name])==h for name,h in protocol['input_sha256'].items()}
    driver=ast.parse((FAMILY/'run_isolation_case.py').read_text())
    command=next(node for node in ast.walk(driver)if isinstance(node,ast.FunctionDef)and node.name=='command')
    original_calls=[node for node in ast.walk(command)if isinstance(node,ast.Call)and isinstance(node.func,ast.Name)and node.func.id=='original_command']
    guard_calls=[node for node in ast.walk(command)if isinstance(node,ast.Call)and isinstance(node.func,ast.Attribute)and node.func.attr=='guards']
    source_boundary={'one_original_command_call_in_wrapper':len(original_calls)==1,'truth_guard_context_present':len(guard_calls)==1,
        'physical_contact_api_forbidden':"patch.object(physical, 'contact_records', helper.forbidden)"in(FAMILY/'run_isolation_case.py').read_text(),
        'mirror_state_assertions_present':all(s in(FAMILY/'run_isolation_case.py').read_text()for s in ('mirror.last_input, physical.last_input','mirror.data.qpos, physical.data.qpos','mirror.data.qvel, physical.data.qvel','mirror.ground is None and mirror.terrain is None and mirror.data.ncon == 0'))}
    cases={};raw={};provenances={};jobs={j['case_id']:j for j in protocol['cases']}
    for case,folder in folders.items():
        manifest=read(folder/'artifact_manifest.json');manifest_ok=all(sha(folder/r['path'])==r['sha256']for r in manifest['files'])
        p=load(folder/'precision.npz');raw[case]=p;log=load(folder/'rollout.npz');summary=read(folder/'summary.json');prov=read(folder/'provenance.json');provenances[case]=prov;diag=read(folder/'candidate_diagnostics.json');execution=read(folder/'execution_result.json')
        n=len(p['time_s']);t=p['time_s'];bounds=p['exact_input_bounds'];native=p['exact_native_applied'];requested=p['exact_native_requested'];final=p['exact_applied'];previous=p['exact_previous_applied'];cap=jobs[case]['cap_m_s']
        component=sum((p['exact_'+term]for term in TERMS),np.zeros_like(requested))
        errors={'native_component_closure':gap(component,requested),'native_box_projection':gap(native,np.clip(requested,-bounds,bounds)),
            'native_clip_delta':gap(p['exact_native_clipping_command_delta'],native-requested),
            'handoff_delta':gap(p['exact_handoff_command_delta'],final-native),
            'total_projection_delta':gap(p['exact_clipping_command_delta'],final-requested),
            'minus_K_error_lqr':gap(p['exact_lqr_command'],-np.einsum('nij,nj->ni',p['exact_feedback_gain'],p['exact_feedback_error'])),
            'reference_delta':gap(p['exact_reference_guard_delta'],p['exact_guarded_joint_reference']-p['exact_native_joint_reference'])}
        expected_bounds=np.tile([1.5,.6,.6,cap,cap],(n,1))
        checks={'case_manifest_valid':manifest_ok,'positive_samples':n>0,'5ms_clock':bool(np.allclose(np.diff(t),.005,rtol=0,atol=1e-10)),
            'truth_endpoint_clock_identical':same(t,p['truth_time_s']),
            'previous_actual_is_prior_final':same(previous[1:],final[:-1]),
            'mirror_runtime_alignment_assertions_true':bool(p['mirror_previous_applied_matches_physical'].all()),
            'bounds_are_declared_operating_cap':same(bounds,expected_bounds),
            'final_within_operating_bounds':bool(np.all(abs(final)<=bounds+1e-12)),
            'no_handoff_torque_changes':same(final[:,:3],native[:,:3]),
            'raw_projection_and_component_arithmetic':all(e<1e-9 for e in errors.values()),
            'diagnostic_rows_and_clock':len(diag)==n and all(abs(row['time_s']-t[i])<1e-10 for i,row in enumerate(diag)),
            'controller_call_count_matches_samples':summary['contact_forecast_case']['guarded_controller_calls']==n==execution['guarded_calls'],
            'provenance_source_unchanged_flag':bool(prov['source_hashes_unchanged_after']),
            'executed_sources_current_and_snapshot_match':all(sha(ROOT/r['origin'])==r['sha256']==sha(folder/r['snapshot'])for r in prov['sources']),
            'protocol_hash_matches_provenance':prov['protocol_sha256']==sha(FAMILY/'protocol.json'),
            'bridge_binary_matches_provenance':sha(ROOT/prov['bridge_binary']['path'])==prov['bridge_binary']['sha256'],
            'parameters_hardware_observer_and_sync':prov['parameters']['mode']=='hardware'and prov['parameters']['rolling_allocation']=='least_squares'and not prov['parameters']['track_contact_point']and prov['parameters']['synchronize'],
            'initial_and_reference_XY_program_preserved':same(p['exact_reference_qpos'][:,:2],p['exact_reference_xy']),
            'common_log_final_input_is_returned_applied':np.allclose(log['input'],final,rtol=0,atol=1e-7),
            'common_log_request_stays_native_raw':np.allclose(log['command'],requested,rtol=1e-7,atol=1e-7)}
        feature={}
        if case=='reference030':
            native_ref=p['exact_native_joint_reference'];guarded=p['exact_guarded_joint_reference'];prev=np.zeros(3);pred=[]
            limits=np.asarray(protocol['execution']['interventions']['reference']['joint_rate_limits_rad_s']);acc=np.asarray(protocol['execution']['interventions']['reference']['joint_reference_acceleration_limits_rad_s2'])
            for i in range(n):
                elapsed=.005 if i==0 else t[i]-t[i-1]
                expected=np.clip(native_ref[i],np.maximum(-limits,prev-acc*elapsed),np.minimum(limits,prev+acc*elapsed));pred.append(expected);prev=expected
            feature={'independent_reference_projection_gap':gap(pred,guarded),'reference_speed_max_rad_s':float(abs(guarded).max()),'reference_max_adjacent_step_rad_s':float(abs(np.diff(guarded,axis=0)).max())if n>1 else 0.,
                'only_reference_helper_diagnostics':all(r['reference']is not None and r['handoff']is None for r in diag),
                'reference_matches_native_feedback_input':same(guarded,p['exact_feedback_reference_qvel'][:,6:9])}
            checks.update(reference_projection_independently_reproduced=feature['independent_reference_projection_gap']<1e-10,
                reference_speed_bound=feature['reference_speed_max_rad_s']<=20+1e-10,
                only_reference_feature_active=feature['only_reference_helper_diagnostics'],reference_feedback_input_correct=feature['reference_matches_native_feedback_input'],handoff_inactive_delta_zero=bool(not np.any(p['exact_handoff_command_delta'])))
        elif case=='handoff060':
            loads=p['exact_estimated_support_loads_N'];last_loaded=None;episode=np.zeros(2,dtype=bool);pred=[];diagnostic_mismatches=[];active_steps=[];rate=protocol['execution']['interventions']['handoff']['acceleration_m_s2'];threshold=protocol['execution']['interventions']['handoff']['estimated_load_share_threshold']
            for i in range(n):
                total=float(loads[i].sum());loaded=(loads[i,2:]>=threshold*total)&(total>0)
                trigger=np.zeros(2,dtype=bool)if last_loaded is None else loaded!=last_loaded;active=episode|trigger
                expected=native[i].copy();lower=np.maximum(-bounds[i,3:],previous[i,3:]-rate*.005);upper=np.minimum(bounds[i,3:],previous[i,3:]+rate*.005)
                expected[3:]=np.where(active,np.clip(native[i,3:],lower,upper),native[i,3:]);limited=expected[3:]!=native[i,3:];after=active&limited
                hd=diag[i]['handoff']
                if hd is None or any(not np.array_equal(np.asarray(hd[key]),value)for key,value in [('loaded',loaded),('triggered',trigger),('episode_active',active),('episode_after',after),('limited',limited)]):diagnostic_mismatches.append(i)
                active_steps.extend(abs(final[i,3:]-previous[i,3:])[active].tolist());pred.append(expected);last_loaded=loaded;episode=after
            feature={'independent_handoff_projection_gap':gap(pred,final),'diagnostic_mismatch_indices':diagnostic_mismatches,
                'max_active_episode_belt_step_m_s':max(active_steps,default=0.),'only_handoff_helper_diagnostics':all(r['handoff']is not None and r['reference']is None for r in diag),
                'first_call_has_no_artificial_trigger':not np.any(diag[0]['handoff']['triggered'])}
            checks.update(handoff_projection_independently_reproduced=feature['independent_handoff_projection_gap']<1e-12,
                handoff_diagnostics_match_independent_state_machine=not diagnostic_mismatches,
                active_episode_slew_bound=feature['max_active_episode_belt_step_m_s']<=.015+1e-12,
                first_call_initialization_only=feature['first_call_has_no_artificial_trigger'],only_handoff_feature_active=feature['only_handoff_helper_diagnostics'],
                reference_guard_inactive=same(p['exact_native_joint_reference'],p['exact_guarded_joint_reference']))
        else:
            checks.update(no_candidate_calls=all(r['reference']is None and r['handoff']is None for r in diag),zero_candidate_deltas=bool(not np.any(p['exact_handoff_command_delta'])and not np.any(p['exact_reference_guard_delta'])))
            old=load(ROOT/protocol['inputs']['i49_'+case+'_precision']);shared=sorted(set(p)&set(old));parity={k:same(p[k],old[k])for k in shared}
            feature={'historical_common_field_count':len(shared),'historical_common_fields_byte_identical':parity}
            checks['historical_baseline_all_common_fields_byte_identical']=all(parity.values())
        cases[case]={'integrity_passed':all(checks.values()),'checks':checks,'samples':n,'completed':summary['completed'],'failure':summary['failure'],'duration_s':float(t[-1]),'arithmetic_max_errors':errors,'feature_contract':feature,'case_manifest_files':len(manifest['files'])}
    baseline=raw['base030'];initial={};references={};fixed={}
    for case,p in raw.items():
        prov=provenances[case];n=len(p['time_s']);initial[case]=all(same(p[k][0],baseline[k][0])for k in ('exact_qpos','exact_qvel','exact_nominal_joints_before','exact_feedback_gain'))
        references[case]=all(same(p[k],baseline[k][:n])for k in ('time_s','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2'))
        fixed[case]={'gain_bank':prov['design']['gain_bank_sha256']==provenances['base030']['design']['gain_bank_sha256'],
            'ordinary_design':prov['ordinary_design_sha256']==provenances['base030']['ordinary_design_sha256'],
            'gain_cache':prov['gain_cache_sha256']==protocol['execution']['gain_cache_sha256'],
            'design_bounds':prov['design']['design_bounds']==protocol['execution']['design_input_bounds'],
            'bridge_binary':prov['bridge_binary']['sha256']==provenances['base030']['bridge_binary']['sha256']}
    global_checks={'all_protected_sources_match_protocol':all(source_checks.values()),'all_historical_inputs_match_protocol':all(old_checks.values()),
        'source_guard_contract_present':all(source_boundary.values()),'all_initial_states_and_initial_gains_match':all(initial.values()),
        'same_reference_time_program':all(references.values()),'same_fixed_design_cache_and_bridge':all(all(v.values())for v in fixed.values())}
    result={'iteration':'I-51','integrity_passed':all(global_checks.values())and all(c['integrity_passed']for c in cases.values()),'global_checks':global_checks,'cases':cases,
        'initial_state_and_gain_identity':initial,'fixed_reference_program':references,'fixed_design_and_bridge':fixed,'source_boundary_checks':source_boundary,'source_sha256_checks':source_checks,'historical_input_checks':old_checks,
        'scope':__doc__,'input_sha256':input_hashes,'no_physics_controller_observer_or_forecast_execution':True,
        'conventions':['Native means the one ordinary controller call with that arm\u2019s reference intervention, if present. It is not a same-state counterfactual unmodified controller.','Common requested is native raw; common clippingdelta is TOTAL box+continuity projection. Separate nativeboxclipping andhandoffdelta identify origins.','Native actuator_saturated flags indicate operating-box saturation; handoff diagnostics.limited indicates continuity intervention.','Passing integrity/reference/slew contracts does not establish better motion, completion or acceptance-gate success.']}
    assert input_hashes=={str(p.relative_to(ROOT)):sha(p)for p in paths}
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'artifact_manifest.json').write_text(json.dumps({'files':[{'path':str(p.relative_to(out)),'sha256':sha(p),'bytes':p.stat().st_size}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']},indent=2)+'\n')
    print(json.dumps({'integrity_passed':result['integrity_passed'],'global_checks':global_checks,'cases':{k:{'integrity_passed':v['integrity_passed'],'completed':v['completed'],'samples':v['samples'],'failed_checks':[c for c,ok in v['checks'].items()if not ok]}for k,v in cases.items()}},indent=2))
    if not result['integrity_passed']:raise AssertionError('Preserve failed QA stage before correction')

if __name__=='__main__':main()
