"""Registered I52 per-phase2x2 reduction from sealed native logs only.

No controller, observer, forecast, model construction, kinematics or physics is
executed. Immutable I49 outcome metrics and corrected I41 reference-event
definitions are reused. New joint/support/phase and factorial calculations live
in the separately frozen pure factorial_metrics.py. No phase is pooled away.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import csv
import hashlib
import json
import shutil
import types
import xml.etree.ElementTree as ET

import numpy as np

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
I49=ROOT/'results/belt_capacity_sweep_20261126/analyze_capacity.py'
I49_SHA='cbaf7ae18989e1a07e9dfaf4167dd2eaa75fd59d064c4259a7b75f79d05c3a6d'
PROTOCOL_SHA='fd33439c9db9a6bc31183409df9f00d13e82c3393581d4c66bab7878d59767bf'
PHASES=('m100','p030','p090')
VARIANTS=('B','R','N','NR')
BOUNDS=np.array([1.5,.6,.6,.3,.3])
DT=.005


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def safe(value):
    if isinstance(value,dict):return {str(k):safe(v)for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [safe(v)for v in value]
    if isinstance(value,np.ndarray):return safe(value.tolist())
    if isinstance(value,np.generic):return safe(value.item())
    if isinstance(value,Path):return str(value)
    if isinstance(value,float)and not np.isfinite(value):return None
    return value


def save(path,value):
    path.write_text(json.dumps(safe(value),indent=2,allow_nan=False)+'\n')


def load(path):
    with np.load(path,allow_pickle=False)as arrays:return {k:arrays[k].copy()for k in arrays.files}


def same(a,b):
    return a.shape==b.shape and a.dtype==b.dtype and a.tobytes()==b.tobytes()


def module(path,name):
    result=types.ModuleType(name);result.__file__=str(path)
    exec(compile(path.read_text(),str(path),'exec'),result.__dict__)
    return result


def freeze(output,protocol,checks_only):
    output.mkdir(parents=True,exist_ok=False)
    assert sha(FAMILY/'protocol.json')==PROTOCOL_SHA
    assert sha(I49)==I49_SHA
    sources=[Path(__file__).resolve(),HERE/'factorial_metrics.py',HERE/'current_residual.py',I49,FAMILY/'protocol.json',
        ROOT/protocol['inputs']['event_helper'],ROOT/protocol['inputs']['event_turns'],ROOT/'src/spherical_wheel/mobility.py']
    records=[]
    for source in sources:
        dest=output/'source_snapshot'/source.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
        records.append(dict(origin=str(source.relative_to(ROOT)),snapshot=str(dest.relative_to(output)),sha256=sha(source)))
    save(output/'declaration_before_execution.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),sources=records,checks_only=checks_only,
        phases=list(PHASES),variants=list(VARIANTS),scope=__doc__,
        comparisons='R−B,N−B,NR−B,NR−N,NR−R within each phase; interaction NR−N−R+B. Policy screens versus ownphaseB only; all3phases must pass.',
        no_new_physics_controller_observer_or_forecast=True))
    return (module(I49,'_i52_fixed_i49_metrics'),module(ROOT/protocol['inputs']['event_helper'],'_i52_corrected_i41_events'),module(HERE/'factorial_metrics.py','_i52_new_pure_metrics'),module(HERE/'current_residual.py','_i52_current_residual'))


def checks(metrics,helper,new,residual):
    inherited=metrics.checks(helper)
    added=new.arithmetic_checks()
    introduced=residual.arithmetic_checks()
    assert scalar_outcomes(dict(available=False))['moving_rms_m']is None
    partial=dict(available=True,fixed_horizon_coverage=dict(full_0_44_observed=False,moving_1_41_observed=True),
        timed_tracking=dict(moving_rms_m=.1,full_peak_m=.2,final_m=.15),attitude=dict(peak_reference_error_deg=2.),
        joint_tracking=dict(windows=dict(full=dict(actual_minus_native_ref=dict(rms_norm=1.),actual_minus_guarded_ref=dict(rms_norm=.5)))),
        slip_contact=dict(slip_distance_m=1.),support_modes=dict(comparison=dict(loaded_set_disagreement_duration_s=.1)))
    partial_values=scalar_outcomes(partial)
    assert partial_values['moving_rms_m']==.1 and all(v is None for k,v in partial_values.items()if k!='moving_rms_m')
    return dict(passed=inherited['passed']and added['passed']and introduced['passed'],inherited=inherited,new=added,introduced_residual=introduced,
        partial_horizon_fixture_passed=True,
        count_note='Upstream repeated arithmetic checks are not extra repository tests; six grouped new arithmetic fixtures plus introduced-reference vector and partial-horizon eligibility fixtures, not a full repo suite.')


def declare_inputs(output,protocol):
    inputs={}
    for key,relative in protocol['inputs'].items():
        assert sha(ROOT/relative)==protocol['input_sha256'][key]
        inputs[relative]=protocol['input_sha256'][key]
    for relative,expected in protocol['source_sha256'].items():assert sha(ROOT/relative)==expected
    availability={};models={}
    for job in protocol['cases']:
        case=job['case_id'];folder=FAMILY/'campaign/runs'/case
        if not(folder/'artifact_manifest.json').is_file():raise FileNotFoundError('Wait for sealed case: '+case)
        manifest=json.loads((folder/'artifact_manifest.json').read_text())
        for row in manifest['files']:assert sha(folder/row['path'])==row['sha256'],(case,row['path'])
        present=(folder/'precision.npz').is_file()and(folder/'rollout.npz').is_file()and(folder/'summary.json').is_file()
        availability[case]=dict(full_raw_products_present=present,execution_failure=(folder/'execution_failure.json').is_file())
        for name in ('artifact_manifest.json','precision.npz','rollout.npz','summary.json','provenance.json','candidate_diagnostics.json','execution_result.json','execution_failure.json','partial_precision.npz'):
            path=folder/name
            if path.is_file():inputs[str(path.relative_to(ROOT))]=sha(path)
        if job['reference_guard_enabled']and present:
            matches=list((FAMILY/'campaign/_workspaces'/case).glob('worker_*/mirror/model.xml'))
            models[case]=matches[0]if len(matches)==1 else None
            if models[case]is not None:
                model_path=models[case];inputs[str(model_path.relative_to(ROOT))]=sha(model_path)
                for mesh in ET.parse(model_path).getroot().findall('asset/mesh'):
                    asset=(model_path.parent/mesh.attrib['file']).resolve()
                    inputs[str(asset.relative_to(ROOT))]=sha(asset)
    save(output/'inputs_before_reduction.json',dict(input_sha256=inputs,availability=availability,current_kinematics_models={case:None if path is None else str(path.relative_to(ROOT))for case,path in models.items()},
        residual_scope='Modified R/NR rows only; private current-pose mj_kinematics/mj_comPos/mj_jac. No integration, forward, forecast or true-contact input.',
        missing_case_policy='Setup failures remain explicit/unavailable, not dropped or replaced. Valid early-terminated precision logs retain their actual duration, missing windows and failed completion screen.'))
    return inputs,availability,models


def contracts(raw,job,diagnostics,new):
    n=len(raw['time_s']);normal=job['normal_phase_enabled'];guard=job['reference_guard_enabled']
    flags=np.asarray(raw['exact_reference_guard_enabled'],bool)
    native=raw['exact_native_requested'];applied=raw['exact_applied']
    checks=dict(time_step_5ms=bool(np.allclose(np.diff(raw['time_s']),DT,rtol=0,atol=1e-9)),
        actual_previous_command_aligned=same(raw['exact_previous_applied'][1:],applied[:-1]),
        operating_bounds_unchanged=same(raw['exact_input_bounds'],np.broadcast_to(BOUNDS,(n,5)).copy()),
        actual_is_native_box_clipped=same(applied,np.clip(native,-BOUNDS,BOUNDS)),
        no_belt_continuity_intervention=bool(not np.any(raw['exact_handoff_command_delta'])),
        native_box_delta=same(raw['exact_native_clipping_command_delta'],applied-native),
        reference_delta=same(raw['exact_reference_guard_delta'],raw['exact_guarded_joint_reference']-raw['exact_native_joint_reference']),
        used_reference_reaches_feedback=same(raw['exact_feedback_reference_qvel'][:,6:9],raw['exact_guarded_joint_reference']),
        used_reference_logged_in_qvel=same(raw['exact_reference_qvel'][:,6:9],raw['exact_guarded_joint_reference']),
        reference_flag_matches_policy=bool(np.all(flags==guard)),
        no_explicit_XY_reference_relaxation=same(raw['exact_reference_qpos'][:,:2],raw['exact_reference_xy']),
        measurement_mirror_alignment_asserted=bool(np.all(raw['mirror_previous_applied_matches_physical'])),
        evaluator_clock_matches_state=same(raw['truth_time_s'],raw['time_s']),
        diagnostics_count=len(diagnostics)==n)
    changed=np.any(raw['exact_reference_guard_delta']!=0,axis=1)
    if guard:
        used=raw['exact_guarded_joint_reference'];step=np.diff(used,axis=0)/np.diff(raw['time_s'])[:,None]
        checks.update(reference_speed_bound=bool(np.all(abs(used)<=20.+1e-12)),
            reference_slew_bound=bool(np.all(abs(step)<=100.+1e-8)),
            first_reference_from_zero_slew=bool(np.all(abs(used[0])<=100.*DT+1e-12)))
    else:checks['reference_unchanged_when_disabled']=not changed.any()
    factors=[r['factors']for r in diagnostics]
    checks['guard_diagnostics_presence_matches_policy']=all((f['guard_diagnostics']is not None)==guard for f in factors)
    phase=new.phase_metrics(raw,normal)
    checks.update(selected_phase_matches_source=phase['selected_phase_matches_registered_source'],phase_flag_matches_policy=phase['flag_matches_registered_policy'])
    held=new.held_selection(raw['time_s']);fresh=np.array([bool(f['current_gain_lookup_requested'])for f in factors])
    phase.update(fresh_gain_lookup_held_duration_s=float(np.sum(fresh[held])*DT),
        gain_held_without_fresh_lookup_duration_s=float(np.sum(~fresh[held])*DT),
        world_vertical_coordinate_undefined_held_samples=int(sum(not f['world_z_phase_coordinate_defined']for f in factors[:-1])),
        estimated_normal_coordinate_undefined_held_samples=int(sum(not f['normal_phase_coordinate_defined']for f in factors[:-1])),
        note='Current phase is logged even while a previous gain is held; phase changes are not automatically fresh gain lookups. N changes only lookup angle in the existing flat gainbank, not all dynamics/error/input frames.')
    K=raw['exact_feedback_gain'][held];change=np.linalg.norm(np.diff(K,axis=0).reshape(max(len(K)-1,0),-1),axis=1)if len(K)>1 else np.empty(0)
    previous_norm=np.linalg.norm(K[:-1].reshape(max(len(K)-1,0),-1),axis=1)if len(K)>1 else np.empty(0)
    relative=np.divide(change,np.maximum(previous_norm,1e-12))
    phase['observed_gain_changes']=dict(adjacent_changes=int(np.sum(change>0)),max_relative_coordinate_matrix_step=float(relative.max())if len(relative)else None,
        interpretation='Relative Frobenius change in the saved matrix representation; contact-load routing and phase can both change gains. It is not a physical energy/force measure or unique phase attribution.')
    return dict(passed=all(checks.values()),checks=checks,phase=phase)


def history_parity(data,logs,protocol):
    result={}
    for case,key in [('p090_B','i51_base030_precision'),('p090_R','i51_reference030_precision')]:
        if case not in data:result[case]=dict(required=True,available=False,passed=False);continue
        old=load(ROOT/protocol['inputs'][key]);common=sorted(set(old)&set(data[case]));byfield={k:same(data[case][k],old[k])for k in common}
        result[case]=dict(required=True,available=True,passed=all(byfield.values()),common_fields_byte_identical=byfield,
            source=protocol['inputs'][key],interpretation='Float64 precise common-field parity to the matched I51 arm.')
    for phase in PHASES:
        case=phase+'_B'
        if case not in logs:continue
        old=load(ROOT/protocol['inputs']['old_'+phase+'_rollout']);common=sorted(set(old)&set(logs[case]));fields={k:same(logs[case][k],old[k])for k in common}
        old_summary=json.loads((ROOT/protocol['inputs']['old_'+phase+'_summary']).read_text())
        result[phase+'_old_encoded']=dict(required=False,available=True,passed=all(fields.values()),common_rollout_fields_byte_identical=fields,
            old_path=old_summary['path'],source=protocol['inputs']['old_'+phase+'_rollout'],
            limitation='Older float32 encoded rollout/summary has no percaseprecision/provenance file. Byte equality of encoded shared fields and nominal setup is not a claim of independently archived float64 execution-source parity.')
    return result


def trace_context(times,geometry,window):
    mask=(times>=window[0]-1e-9)&(times<=window[1]+1e-9);values=geometry['local_path_distance_m'][mask]
    return dict(window_s=window,samples=int(mask.sum()),complete=bool(times[0]<=window[0]+1e-9 and times[-1]>=window[1]-1e-9),
        rms_m=float(np.sqrt(np.mean(values**2)))if len(values)else None,max_m=float(values.max())if len(values)else None)


def scalar_outcomes(case):
    if not case.get('available'):return {k:None for k in ('moving_rms_m','peak_error_m','final_error_m','peak_reference_lean_deg','actual_minus_native_joint_error_rms_rad_s','actual_minus_used_joint_error_rms_rad_s','slip_distance_m','support_set_disagreement_duration_s')}
    full=case['joint_tracking']['windows']['full'];complete=case['fixed_horizon_coverage']['full_0_44_observed']
    return dict(moving_rms_m=case['timed_tracking']['moving_rms_m']if case['fixed_horizon_coverage']['moving_1_41_observed']else None,
        peak_error_m=case['timed_tracking']['full_peak_m']if complete else None,final_error_m=case['timed_tracking']['final_m']if complete else None,
        peak_reference_lean_deg=case['attitude']['peak_reference_error_deg']if complete else None,
        actual_minus_native_joint_error_rms_rad_s=full['actual_minus_native_ref'].get('rms_norm')if complete else None,
        actual_minus_used_joint_error_rms_rad_s=full['actual_minus_guarded_ref'].get('rms_norm')if complete else None,
        slip_distance_m=case['slip_contact']['slip_distance_m']if complete else None,
        support_set_disagreement_duration_s=case['support_modes']['comparison']['loaded_set_disagreement_duration_s']if complete else None)


def reduce(output,protocol,metrics,helper,new,residual):
    inputs,availability,models=declare_inputs(output,protocol);program=load(ROOT/protocol['inputs']['historical_precision'])
    data={};logs={};summaries={};provenance={};cases={};geometries={};metadata={}
    for job in protocol['cases']:
        case=job['case_id'];folder=FAMILY/'campaign/runs'/case
        if not availability[case]['full_raw_products_present']:
            cases[case]=dict(available=False,completed=False,failure='Missing full raw products; retained setup/execution failure',phase_rad=job['phase_rad'],variant=job['variant']);continue
        data[case]=load(folder/'precision.npz');logs[case]=load(folder/'rollout.npz');summaries[case]=json.loads((folder/'summary.json').read_text());provenance[case]=json.loads((folder/'provenance.json').read_text())
        metadata[case]=summaries[case]['contact_scheduling_joint_tracking']
    parity=history_parity(data,logs,protocol);save(output/'historical_parity_before_metrics.json',parity)
    required_history_ok=all(row['passed']for row in parity.values()if row.get('required'))
    initial={};ordinary={};initial_K={};fixed={};program_checks={}
    bank0=next(iter(metadata.values()))['fixed_design']if metadata else None
    for phase in PHASES:
        base=phase+'_B';initial[phase]={};ordinary[phase]={};initial_K[phase]={}
        for variant in VARIANTS:
            case=phase+'_'+variant
            if case not in data or base not in data:continue
            initial[phase][variant]={key:same(data[case][key][0],data[base][key][0])for key in ('exact_qpos','exact_qvel','exact_observer_wrench','exact_estimated_normal','exact_nominal_joints_before','exact_previous_applied','exact_reference_qpos')}
            ordinary[phase][variant]=metadata[case]['ordinary_design_sha256']==metadata[base]['ordinary_design_sha256']
        for a,b in [('B','R'),('N','NR')]:
            if phase+'_'+a in data and phase+'_'+b in data:initial_K[phase][a+'='+b]=same(data[phase+'_'+a]['exact_feedback_gain'][0],data[phase+'_'+b]['exact_feedback_gain'][0])
    events=json.loads((ROOT/protocol['inputs']['event_turns']).read_text())['rows'];assert len(events)==6
    from spherical_wheel.mobility import TARGETS
    for job in protocol['cases']:
        case=job['case_id']
        if case not in data:continue
        raw=data[case];log=logs[case];t=raw['time_s'];summary=summaries[case];meta=metadata[case]
        fixed[case]=dict(gain_bank=meta['fixed_design']['gain_bank_sha256']==bank0['gain_bank_sha256'],
            design_configuration=meta['fixed_design']['design_config_sha256']==bank0['design_config_sha256'],weights=meta['fixed_design']['weights']==bank0['weights'],
            design_bounds=meta['fixed_design']['design_bounds']==BOUNDS.tolist(),gain_cache=meta['gain_cache_sha256']==protocol['execution']['gain_cache_sha256'],
            configuration_unchanged=meta['configuration_unchanged']is True,cap_030=meta['operating_belt_cap_m_s']==.3)
        program_checks[case]={key:same(raw[key],program[key][:len(t)])for key in ('time_s','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2')}
        diagnostics=json.loads((FAMILY/'campaign/runs'/case/'candidate_diagnostics.json').read_text());mechanism=contracts(raw,job,diagnostics,new)
        track=metrics.tracking_metrics(raw,protocol['evaluation']['timed_moving_window_s']);absolute,relative=metrics.attitude(raw)
        settle=metrics.settling(t,raw['exact_qvel'],relative,summary['completed']);commands=metrics.command_metrics(raw,BOUNDS)
        closure=float(np.linalg.norm(raw['exact_qpos'][-1,:2]-raw['exact_qpos'][0,:2]))
        gates=dict(completed=bool(summary['completed']),rms=(track['moving_rms_m']or 0.)<=TARGETS['rms_m'],
            p95=track['moving_p95_m']is not None and track['moving_p95_m']<=TARGETS['p95_m'],max=track['full_peak_m']<=TARGETS['max_m'],final=track['final_m']<=TARGETS['final_m'],
            attitude=float(np.degrees(relative).max())<=TARGETS['attitude_deg'],saturation=commands['original_any_request_strict_saturated_fraction']<=TARGETS['saturated_fraction'],settled=settle['passed'],closure=closure<=TARGETS['closure_m'])
        mechanism['checks']['original_gate_reconstruction_matches_native_summary']=gates==summary['mars']['gates'];mechanism['passed']=all(mechanism['checks'].values())
        geometry=metrics.geometry(raw,program,helper,protocol['evaluation']['timed_moving_window_s']);geometries[case]=geometry;np.savez_compressed(output/(case+'_geometry.npz'),**geometry)
        windows={'full':[0.,44.],'moving':[1.,41.],'early':[6.4,7.4],'late':[38.,44.]}
        coverage=dict(full_0_44_observed=bool(t[0]<=1e-9 and t[-1]>=44.-1e-9),moving_1_41_observed=bool(t[0]<=1.+1e-9 and t[-1]>=41.-1e-9))
        track.update(observed_end_time_s=float(t[-1]),final_at_planned_44_m=track['final_m']if coverage['full_0_44_observed']else None,
            fixed_0_44_peak_m=track['full_peak_m']if coverage['full_0_44_observed']else None,
            semantics='Legacy full_peak/final/full_RMS use the observed trace/end. Fixed44s paired/factorial metrics are unavailable on short runs; movingRMS requires complete1–41s coverage.')
        cases[case]=dict(available=True,phase_rad=job['phase_rad'],phase_key=job['phase_key'],variant=job['variant'],cap_m_s=.3,
            normal_phase_enabled=job['normal_phase_enabled'],reference_guard_enabled=job['reference_guard_enabled'],completed=bool(summary['completed']),failure=summary['failure'],samples=len(t),completed_duration_s=float(t[-1]),
            fixed_horizon_coverage=coverage,
            timed_tracking=track,attitude=dict(peak_absolute_deg=float(np.degrees(absolute).max()),peak_reference_error_deg=float(np.degrees(relative).max()),rms_reference_error_deg=float(np.degrees(np.sqrt(np.mean(relative**2)))),final_reference_error_deg=float(np.degrees(relative[-1]))),
            original_settling=settle,original_route_gates=dict(gates=gates,passed=all(gates.values()),targets=TARGETS,closure_m=closure),commands=commands,belts=metrics.belt_metrics(raw,.3),
            departure=metrics.departure(t,geometry,helper,protocol['evaluation']['early_window_s']),turns=metrics.turns(raw,geometry,events,helper),
            local_path_contexts={name:trace_context(t,geometry,window)for name,window in [('early',[6.4,7.4]),('late',[38.,44.])]},
            joint_tracking=new.joint_tracking(raw,job['reference_guard_enabled'],windows),support_modes=new.support_modes(raw,provenance[case]['weight_N']),
            mechanistic=mechanism,slip_contact=metrics.slip_contact(log,provenance[case]['weight_N']),fixed_design=meta)
        if job['reference_guard_enabled']:
            cases[case]['mechanistic']['introduced_rolling_reference_residual']=residual.summarize(output,case,raw,log,models.get(case))
    per_phase={};comparisons={}
    for phase in PHASES:
        phase_cases={variant:cases[phase+'_'+variant]for variant in VARIANTS};contrasts={}
        for before,after in [('B','R'),('B','N'),('B','NR'),('N','NR'),('R','NR')]:
            a,b=phase+'_'+before,phase+'_'+after;key=after+'-'+before
            if cases[a].get('available')and cases[b].get('available'):
                comparison=metrics.compare(cases[b],cases[a],protocol['provisional_screen']);comparison.update(available=True,baseline_id=a,candidate_id=b,registered_adoption_comparison=before=='B')
                comparison['fixed_44s_horizon_comparison_available']=cases[a]['fixed_horizon_coverage']['full_0_44_observed']and cases[b]['fixed_horizon_coverage']['full_0_44_observed']
                comparison['checks']['common_planned44_coverage']=comparison['fixed_44s_horizon_comparison_available']
                comparison['passes_all_conditions']=all(comparison['checks'].values())
                comparison['observed_durations_s']={a:cases[a]['completed_duration_s'],b:cases[b]['completed_duration_s']}
                for change,c,z in zip(comparison['turn_changes'],cases[b]['turns'],cases[a]['turns']):
                    change.update(baseline_signed_lag_s=z['actual_reversal_lag_s'],candidate_signed_lag_s=c['actual_reversal_lag_s'],absolute_lag_error_change_s=None if c['actual_reversal_lag_s']is None or z['actual_reversal_lag_s']is None else abs(c['actual_reversal_lag_s'])-abs(z['actual_reversal_lag_s']))
                comparison['interpretation']='Matched starting phase; deterministic descriptive comparison. Only own-B comparisons feed all-three-phase screen. On incomplete44s traces, observed legacy checks are not equal-horizon performance claims and the screen fails coverage. Signed lag guard and absolute lag changes remain separate.'
            else:comparison=dict(available=False,baseline_id=a,candidate_id=b,passes_all_conditions=False,reason='Missing case outcome; not dropped from screen')
            contrasts[key]=comparison
            if before=='B':comparisons[b]=comparison
        scalar={variant:scalar_outcomes(row)for variant,row in phase_cases.items()};metric_names=list(scalar['B'])
        factorials={name:new.factorial({variant:scalar[variant][name]for variant in VARIANTS})for name in metric_names}
        for name,effect in factorials.items():
            effect['fixed_window_eligibility']={variant:scalar[variant][name]is not None for variant in VARIANTS}
            effect['window_s']=[1.,41.]if name=='moving_rms_m'else[0.,44.]
        per_phase[phase]=dict(phase_rad=protocol['factorial']['phases_rad'][phase],case_ids={v:phase+'_'+v for v in VARIANTS},comparisons=contrasts,factorial_effects=factorials,
            no_pooled_override=True)
    global_checks=dict(required_p090_B_R_history_parity=required_history_ok,
        fixed_bank_configuration_and_cap=all(all(r.values())for r in fixed.values()),
        initial_physical_observer_reference_within_phase=all(all(all(v.values())for v in phase.values())for phase in initial.values()),
        ordinary_design_within_phase=all(all(phase.values())for phase in ordinary.values()),
        initial_gain_pairs_B_R_and_N_NR_only=all(all(phase.values())for phase in initial_K.values()),
        fixed_XY_time_velocity_acceleration_program=all(all(r.values())for r in program_checks.values()),
        all_available_mechanistic_contracts=all(row['mechanistic']['passed']for row in cases.values()if row.get('available')))
    integrity_ok=all(global_checks.values())
    policy={variant:dict(passes_all_three_phases=integrity_ok and all(comparisons[phase+'_'+variant]['passes_all_conditions']for phase in PHASES),integrity_eligible=integrity_ok,
        phase_passes={phase:comparisons[phase+'_'+variant]['passes_all_conditions']for phase in PHASES},
        eligibility='Descriptive registered screen only; even a pass requires later broader routes and motor-load validation, not default adoption.')for variant in ('R','N','NR')}
    report=dict(iteration='I-52',cases=cases,comparisons=comparisons,per_phase=per_phase,policy_screens=policy,
        integrity=dict(passed=all(global_checks.values()),checks=global_checks,historical_parity=parity,initial_state_within_phase=initial,ordinary_within_phase=ordinary,initial_gain_pair_checks=initial_K,fixed_bank_checks=fixed,program_checks=program_checks,availability=availability),
        conventions=dict(held_dt_s=DT,terminal_command_excluded_from_held_metrics=True,turns='Six identical correctedI41reference events, not I41baseline outcomes. Signed reversal guard plus absolute timing changes reported.',
            factorial='Perphase2x2 contrasts only; no statisticalreplication or pooledphasefailure masking.',joint_tracking='Actual−native andactual−used references are distinct; easier reference targets do not bythemselves prove improved motor tracking or body motion.',
            phase='N changes only current lookup angle in existingflatbank; originalshaftrow andnativeerror/inputframes retained. Requestedphase can vary while a priorgain is held.'),limitations=protocol['limitations'])
    save(output/'report.json',report);write_tables(output,report);plots(output,report,data,geometries)
    for relative,expected in inputs.items():assert sha(ROOT/relative)==expected
    for row in json.loads((output/'declaration_before_execution.json').read_text())['sources']:assert sha(ROOT/row['origin'])==row['sha256']
    for relative,expected in protocol['source_sha256'].items():assert sha(ROOT/relative)==expected
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),bytes=p.stat().st_size,sha256=sha(p))for p in sorted(output.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))


def write_tables(output,report):
    fields=['case_id','phase_rad','variant','available','completed','moving_rms_m','peak_error_m','final_error_m','peak_reference_lean_deg','settled','route_passed','actual_minus_native_joint_error_rms_rad_s','actual_minus_used_joint_error_rms_rad_s','slip_distance_m','support_set_disagreement_duration_s']
    with(output/'case_metrics.csv').open('w',newline='')as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for case,row in report['cases'].items():
            values=dict(case_id=case,phase_rad=row['phase_rad'],variant=row['variant'],available=row['available'],completed=row['completed'],**scalar_outcomes(row))
            values.update(settled=row.get('original_settling',{}).get('passed'),route_passed=row.get('original_route_gates',{}).get('passed'));writer.writerow(values)
    with(output/'turn_metrics.csv').open('w',newline='')as stream:
        fields=['case_id','event_id','reference_time_s','axis','window_complete','geometric_component_overshoot_m','actual_reversal_lag_s','actual_reversal_missing_in_window'];writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for case,row in report['cases'].items():
            for turn in row.get('turns',[]):writer.writerow(dict(case_id=case,**{key:turn[key]for key in fields if key!='case_id'}))
    with(output/'factorial_effects.csv').open('w',newline='')as stream:
        fields=['phase_key','metric','available','R_minus_B','N_minus_B','NR_minus_B','NR_minus_N','NR_minus_R','reference_main_effect','normal_main_effect','interaction_difference_of_differences'];writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for phase,row in report['per_phase'].items():
            for name,effect in row['factorial_effects'].items():writer.writerow(dict(phase_key=phase,metric=name,**{key:effect.get(key)for key in fields if key not in ('phase_key','metric')}))


def plots(output,report,data,geometries):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors={'B':'#d99035','R':'#269fc5','N':'#8b63b0','NR':'#3aa983'}
    fig,axes=plt.subplots(2,3,figsize=(15,8),sharex=True,sharey='row',constrained_layout=True)
    for col,phase in enumerate(PHASES):
        for variant in VARIANTS:
            case=phase+'_'+variant
            if case not in data:continue
            raw=data[case];t=raw['time_s'];error=np.linalg.norm(raw['exact_qpos'][:,:2]-raw['exact_reference_xy'],axis=1)
            axes[0,col].plot(t,error*1000,color=colors[variant],label=variant,lw=1.)
            axes[1,col].plot(t,geometries[case]['local_path_distance_m']*1000,color=colors[variant],label=variant,lw=1.)
        axes[0,col].set_title(f'Initial phase {report["per_phase"][phase]["phase_rad"]:+.2f}rad');axes[0,col].legend(ncol=4,fontsize=8);axes[1,col].set_xlabel('Recorded time(s)')
    axes[0,0].set_ylabel('Timed XYerror(mm)');axes[1,0].set_ylabel('Branch-local path distance(mm)')
    for ax in axes.flat:ax.grid(alpha=.25)
    fig.suptitle('I52: full native histories, allfour policies at allthree starting phases\nShared scales; missing/earlyterminated outcomes remain explicit in the report')
    for ext in ('png','pdf'):fig.savefig(output/f'tracking_all_phases.{ext}',dpi=170)
    plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(15,8),constrained_layout=True)
    for col,phase in enumerate(PHASES):
        for j,variant in enumerate(VARIANTS):
            row=report['cases'][phase+'_'+variant]
            if not row.get('available'):continue
            axes[0,col].bar(j,row['timed_tracking']['moving_rms_m']*1000 if row['timed_tracking']['moving_rms_m']is not None else np.nan,color=colors[variant])
            axes[1,col].bar(j,row['attitude']['peak_reference_error_deg'],color=colors[variant])
        axes[0,col].set_title(f'{phase}: separate phase-specific outcomes')
        for ax in axes[:,col]:ax.set_xticks(range(4),VARIANTS);ax.grid(axis='y',alpha=.2)
    axes[0,0].set_ylabel('Moving timed RMS(mm)');axes[1,0].set_ylabel('Peak reference-relative lean(deg)')
    fig.suptitle('I52: tracking and attitude can trade off; command bounds alone are not acceptance')
    for ext in ('png','pdf'):fig.savefig(output/f'headline_tradeoffs.{ext}',dpi=170)
    plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,4.8),constrained_layout=True)
    for ax,phase in zip(axes,PHASES):
        x=np.arange(6)
        for j,variant in enumerate(VARIANTS):
            row=report['cases'][phase+'_'+variant]
            if not row.get('available'):continue
            ax.bar(x+(j-1.5)*.19,[1000*t['geometric_component_overshoot_m']if t['geometric_component_overshoot_m']is not None else np.nan for t in row['turns']],width=.19,color=colors[variant],label=variant)
        ax.set(title=phase,xticks=x,xticklabels=['1y','2x','3y','4y','5x','6y'],xlabel='Fixed reference component reversal');ax.grid(axis='y',alpha=.2);ax.legend(fontsize=8,ncol=4)
    axes[0].set_ylabel('Geometric component overshoot(mm)')
    fig.suptitle('Allsix immutable reference turns; signed/absolute reversal timing remains in the tables')
    for ext in ('png','pdf'):fig.savefig(output/f'turns_all_phases.{ext}',dpi=170)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--checks-only',action='store_true');args=parser.parse_args()
    protocol=json.loads((FAMILY/'protocol.json').read_text());assert protocol['iteration']=='I-52'
    assert [(j['phase_key'],j['variant'])for j in protocol['cases']]==[(phase,variant)for phase in PHASES for variant in VARIANTS]
    output=args.output.resolve();metrics,helper,new,residual=freeze(output,protocol,args.checks_only)
    try:
        save(output/'pure_checks.json',checks(metrics,helper,new,residual))
        if not args.checks_only:reduce(output,protocol,metrics,helper,new,residual)
    except Exception:
        import traceback
        save(output/'failure.json',dict(traceback=traceback.format_exc(),scope='Exact source/input stage preserved. No automatic retry, overwrite, physics or outcome-driven tuning.'))
        raise


if __name__=='__main__':main()
