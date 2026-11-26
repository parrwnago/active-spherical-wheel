"""Independent I52 raw QA: NumPy/JSON only, no candidate/controller/model imports.

--early checks the first sealed +.90 B/R pair and mandatory historical parity.
Final mode retains every registered case, including physical early termination.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json

import numpy as np

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parents[1]
ROOT=FAMILY.parents[1]
BOUNDS=np.array([1.5,.6,.6,.3,.3])


def sha(path):
    h=hashlib.sha256()
    with path.open('rb')as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [safe(v)for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float)and not np.isfinite(x):return None
    return x
def save(path,x):path.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def same(a,b):return a.shape==b.shape and a.dtype==b.dtype and a.tobytes()==b.tobytes()
def gap(a,b):
    a,b=np.asarray(a),np.asarray(b)
    if a.shape!=b.shape:return np.inf
    return float(np.max(abs(a.astype(float)-b.astype(float))))if a.size else 0.
def wrap(a):return (np.asarray(a)+np.pi)%(2*np.pi)-np.pi


def carrier_rotation(qpos):
    q=qpos[:,3:7].copy();q/=np.linalg.norm(q,axis=1)[:,None]
    w,x,y,z=q.T;rotation=np.empty((len(q),3,3))
    rotation[:,0,0]=1-2*(y*y+z*z);rotation[:,0,1]=2*(x*y-w*z);rotation[:,0,2]=2*(x*z+w*y)
    rotation[:,1,0]=2*(x*y+w*z);rotation[:,1,1]=1-2*(x*x+z*z);rotation[:,1,2]=2*(y*z-w*x)
    rotation[:,2,0]=2*(x*z-w*y);rotation[:,2,1]=2*(y*z+w*x);rotation[:,2,2]=1-2*(x*x+y*y)
    a=qpos[:,7];c,s=np.cos(a),np.sin(a);joint=np.zeros_like(rotation)
    joint[:,0,0]=c;joint[:,0,2]=s;joint[:,1,1]=1.;joint[:,2,0]=-s;joint[:,2,2]=c
    return np.einsum('nij,njk->nik',rotation,joint)


def main(output,early,resolver_path=None):
    output.mkdir(parents=True,exist_ok=False)
    protocol=json.loads((FAMILY/'protocol.json').read_text())
    assert protocol['iteration']=='I-52'
    jobs=[j for j in protocol['cases']if not early or j['case_id']in ('p090_B','p090_R')]
    assert len(jobs)==(2 if early else 12)
    folders={j['case_id']:FAMILY/'campaign/runs'/j['case_id']for j in jobs}
    technical=[]
    sources=[Path(__file__).resolve(),FAMILY/'protocol.json',FAMILY/'campaign/jobs.json']
    if resolver_path is not None:
        resolver_path=resolver_path.resolve();resolver=json.loads(resolver_path.read_text())
        assert set(resolver['cases'])=={j['case_id']for j in protocol['cases']}
        for case,relative in resolver['cases'].items():
            assert not Path(relative).is_absolute() and (ROOT/relative).resolve().is_relative_to(ROOT)
            if case in folders:folders[case]=ROOT/relative
        technical=[(ROOT/relative).resolve()for relative in resolver['technical_attempts']]
        sources.extend([resolver_path,*technical])
    elif not early:
        raise ValueError('Final QA requires explicit --resolver for the registered technical-attempt lineage')
    records=[]
    for source in sources:
        target=output/'source_snapshot'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes())
        records.append(dict(origin=str(source.relative_to(ROOT)),snapshot=str(target.relative_to(output)),sha256=sha(source)))
    inputs={relative:sha(ROOT/relative)for relative in protocol['inputs'].values()}
    technical_metadata=[]
    for path in technical:
        record=json.loads(path.read_text());inputs[str(path.relative_to(ROOT))]=sha(path)
        if 'process_exit_code'in record:
            assert record['process_exit_code']==139 and record['usable_rollout']is False and record['robot_fall_scored']is None
            for row in record['preserved_files']:inputs[row['path']]=row['sha256']
            technical_metadata.append(dict(origin=str(path.relative_to(ROOT)),sha256=sha(path),case_id=record['case_id'],attempt=record['attempt'],
                process_exit_code=139,usable_rollout=False,robot_fall_scored=None,preserved_files=len(record['preserved_files']),
                limitation=record['limitations']))
        else:
            assert record['source_and_control_parameters_unchanged']is True and record['accept_completion_or_robot_failure_as_observed']is True
            assert record['protocol_sha256']==sha(FAMILY/'protocol.json') and record['runner_sha256']==sha(FAMILY/'run_factorial_case.py')
            technical_metadata.append(dict(origin=str(path.relative_to(ROOT)),sha256=sha(path),case_id=record['case_id'],attempt=record['attempt'],
                identical_source_and_parameters=True,serial_execution=record['serial_execution'],accept_observed_outcome=True))
    if resolver_path is not None:inputs[str(resolver_path.relative_to(ROOT))]=sha(resolver_path)
    for job in jobs:
        folder=folders[job['case_id']]
        for name in ('precision.npz','rollout.npz','summary.json','provenance.json','candidate_diagnostics.json','artifact_manifest.json'):
            path=folder/name;inputs[str(path.relative_to(ROOT))]=sha(path)
    save(output/'before_arithmetic.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),early_pair_only=early,sources=records,input_sha256=inputs,
        registered_case_ids=[j['case_id']for j in jobs],resolved_case_paths={case:str(folder.relative_to(ROOT))for case,folder in folders.items()},technical_attempts=technical_metadata,
        scope='Independent saved raw integrity only. No candidate implementation imported. Referencebox reconstructed fromzero+timestamps; phasecarrier fromquaternion/shaft independently; no model/controller/observer/physics/performance reduction.'))
    checks=[]
    def check(name,passed,**detail):
        checks.append(dict(name=name,passed=bool(passed),**safe(detail)))
    def close(name,a,b,tolerance=1e-11):
        value=gap(a,b);check(name,value<=tolerance,maximum_absolute_gap=value,tolerance=tolerance)
    for relative,expected in protocol['source_sha256'].items():check('frozen_source/'+relative,sha(ROOT/relative)==expected)
    for key,expected in protocol['input_sha256'].items():check('registered_input/'+key,sha(ROOT/protocol['inputs'][key])==expected)
    data={};metadata={};reports={};parameter_records={}
    for job in jobs:
        case=job['case_id'];folder=folders[case]
        manifest=json.loads((folder/'artifact_manifest.json').read_text())
        check(case+'/artifact_manifest',all(sha(folder/r['path'])==r['sha256']for r in manifest['files']),entries=len(manifest['files']))
        provenance=json.loads((folder/'provenance.json').read_text());summary=json.loads((folder/'summary.json').read_text())
        metadata[case]=summary['contact_scheduling_joint_tracking']
        check(case+'/executed_sources_and_copies',all(sha(ROOT/r['origin'])==r['sha256']and sha(folder/r['snapshot'])==r['sha256']for r in provenance['sources']))
        binary=provenance['bridge_binary'];check(case+'/bridge_binary',sha(ROOT/binary['path'])==binary['sha256'])
        check(case+'/registered_job',provenance['case']==job)
        params=provenance['parameters'];check(case+'/parameters',params['schedule_on_ground_normal']==job['normal_phase_enabled']and params['mode']=='hardware'and params['rolling_allocation']=='least_squares'and params['synchronize']and not params['track_contact_point']and params.get('tilt_reference')is None)
        parameter_records[case]={key:value for key,value in params.items()if key!='schedule_on_ground_normal'}
        p=load(folder/'precision.npz');data[case]=p;t=p['time_s'];n=len(t);diag=json.loads((folder/'candidate_diagnostics.json').read_text())
        check(case+'/row_clock',n>=1 and len(diag)==n and np.all(abs(np.diff(t)-.005)<1e-9),rows=n,completed=summary['completed'],failure=summary['failure'])
        close(case+'/truth_clock',p['truth_time_s'],t,0.)
        close(case+'/actual_previous_input',p['exact_previous_applied'][1:],p['exact_applied'][:-1],0.)
        close(case+'/fixed_bounds',p['exact_input_bounds'],np.broadcast_to(BOUNDS,(n,5)),0.)
        close(case+'/native_actual_boxclip',p['exact_applied'],np.clip(p['exact_native_requested'],-BOUNDS,BOUNDS),0.)
        close(case+'/native_applied',p['exact_native_applied'],p['exact_applied'],0.)
        close(case+'/native_clipping',p['exact_native_clipping_command_delta'],p['exact_applied']-p['exact_native_requested'],0.)
        close(case+'/common_clipping',p['exact_clipping_command_delta'],p['exact_native_clipping_command_delta'],0.)
        close(case+'/no_belt_handoff_modifier',p['exact_handoff_command_delta'],np.zeros((n,5)),0.)
        check(case+'/mirror_previous_applied_flag',bool(p['mirror_previous_applied_matches_physical'].all()))
        check(case+'/whole_run_factor_flags',bool(np.all(p['exact_schedule_on_ground_normal']==job['normal_phase_enabled'])and np.all(p['exact_reference_guard_enabled']==job['reference_guard_enabled'])))

        # Independent reference envelope; no imported candidate code.
        native=p['exact_native_joint_reference'];expected=np.empty_like(native);previous=np.zeros(3);guard_details=[]
        for i in range(n):
            elapsed=.005 if i==0 else float(t[i]-t[i-1])
            lower=np.maximum(-20.,previous-100.*elapsed);upper=np.minimum(20.,previous+100.*elapsed)
            used=np.minimum(np.maximum(native[i],lower),upper)if job['reference_guard_enabled']else native[i].copy()
            expected[i]=used
            if job['reference_guard_enabled']:
                d=diag[i]['factors']['guard_diagnostics']
                guard_details.append(bool(d is not None and d['initialized_now']==(i==0)and not d['repeated_timestamp']and d['update_count']==i+1 and
                    gap(d['native_joint_reference_rad_s'],native[i])==0. and gap(d['guarded_joint_reference_rad_s'],used)==0. and
                    gap(d['previous_joint_reference_rad_s'],previous)==0. and gap(d['lower_reference_rad_s'],lower)==0. and gap(d['upper_reference_rad_s'],upper)==0.))
                previous=used.copy()
            else:guard_details.append(diag[i]['factors']['guard_diagnostics']is None)
        close(case+'/independent_reference_box',p['exact_guarded_joint_reference'],expected,0.)
        check(case+'/guard_state_clock_diagnostics',all(guard_details))
        close(case+'/reference_delta',p['exact_reference_guard_delta'],expected-native,0.)
        close(case+'/main_reference_joint_entries',p['exact_reference_qvel'][:,6:9],expected,0.)
        close(case+'/feedback_reference_joint_entries',p['exact_feedback_reference_qvel'][:,6:9],expected,0.)
        close(case+'/allocation_reference_joint_entries',p['exact_allocation_drive_reference'][:,:3],expected,0.)
        close(case+'/allocation_reference_belt_entries',p['exact_allocation_drive_reference'][:,3:],p['exact_rolling_feedforward'][:,3:],0.)
        close(case+'/actual_minus_used_error',p['exact_qvel'][:,6:9]-expected,p['exact_feedback_error'][:,15:18],0.)

        vertical=p['exact_world_vertical_carrier'];normal=p['exact_estimated_normal_carrier']
        phase_z=np.arctan2(-vertical[:,0],vertical[:,2]);phase_n=np.arctan2(-normal[:,0],normal[:,2])
        used_phase=phase_n if job['normal_phase_enabled']else phase_z
        close(case+'/phase_worldZ_from_logged_vector',wrap(p['exact_phase_world_vertical_rad']-phase_z),np.zeros(n),0.)
        close(case+'/phase_normal_from_logged_vector',wrap(p['exact_phase_estimated_normal_rad']-phase_n),np.zeros(n),0.)
        close(case+'/selected_effective_phase',wrap(p['exact_effective_phase_rad']-used_phase),np.zeros(n),1e-14)
        close(case+'/normal_xz_norm',p['exact_phase_normal_xz_norm'],np.linalg.norm(normal[:,[0,2]],axis=1),0.)
        close(case+'/worldZ_xz_norm',p['exact_phase_world_xz_norm'],np.linalg.norm(vertical[:,[0,2]],axis=1),0.)
        carrier=carrier_rotation(p['exact_qpos']);independent_z=carrier[:,2,:];independent_n=np.einsum('nji,nj->ni',carrier,p['exact_estimated_normal'])
        close(case+'/quaternion_shaft_worldZ_carrier',vertical,independent_z,2e-10)
        close(case+'/quaternion_shaft_normal_carrier',normal,independent_n,2e-10)
        touching=p['exact_estimated_support_loads_N'].sum(axis=1)>.001
        phase_detail=[]
        for i,row in enumerate(diag):
            d=row['factors']
            phase_detail.append(bool(d['normal_phase_enabled']==job['normal_phase_enabled']and d['reference_guard_enabled']==job['reference_guard_enabled']and
                d['current_gain_lookup_requested']==bool(touching[i])and d['original_feedback_calls_by_this_helper']==0 and
                abs(float(wrap(d['phase_used_rad']-used_phase[i])))<1e-14 and
                d['world_z_phase_coordinate_defined']==bool(np.linalg.norm(vertical[i,[0,2]])>1e-12)and
                d['normal_phase_coordinate_defined']==bool(np.linalg.norm(normal[i,[0,2]])>1e-12)and
                gap(d['actual_joint_rates_rad_s'],p['exact_qvel'][i,6:9])==0.))
        check(case+'/phase_factor_and_held_lookup_diagnostics',all(phase_detail),unloaded_current_gain_held_or_ordinary_rows=int(np.sum(~touching)))
        held_gain=None;held_gain_checks=[]
        for i in range(n):
            if touching[i]:held_gain=p['exact_feedback_gain'][i].copy()
            elif held_gain is not None:held_gain_checks.append(same(p['exact_feedback_gain'][i],held_gain))
        check(case+'/unloaded_gain_history_is_held',all(held_gain_checks),checked_rows=len(held_gain_checks))
        close(case+'/ordinary_shaft_row_constant',p['exact_feedback_gain'][:,0,:],np.broadcast_to(p['exact_feedback_gain'][0,0,:],(n,18)),0.)
        lqr=-np.einsum('nij,nj->ni',p['exact_feedback_gain'],p['exact_feedback_error'])
        close(case+'/LQR_algebra',lqr,p['exact_lqr_command'],1e-10)
        requested=sum(p[key]for key in ('exact_equilibrium_command','exact_rolling_feedforward','exact_support_command','exact_lqr_command','exact_transition_command_delta','exact_heading_command','exact_fade_command_delta'))
        close(case+'/native_request_components',requested,p['exact_native_requested'],1e-10)
        error_native=p['exact_qvel'][:,6:9]-native;error_used=p['exact_qvel'][:,6:9]-expected;held=slice(None,-1)
        reports[case]=dict(rows=n,completed=summary['completed'],failure=summary['failure'],reference_guard_enabled=job['reference_guard_enabled'],normal_phase_enabled=job['normal_phase_enabled'],
            max_native_reference_rad_s=float(abs(native).max()),max_used_reference_rad_s=float(abs(expected).max()),
            max_used_reference_acceleration_rad_s2=float(np.max(abs(np.diff(expected,axis=0))/np.diff(t)[:,None]))if n>1 else None,
            actual_minus_native_reference_rms_perjoint_rad_s=np.sqrt(np.mean(error_native[held]**2,axis=0))if n>1 else None,
            actual_minus_used_reference_rms_perjoint_rad_s=np.sqrt(np.mean(error_used[held]**2,axis=0))if n>1 else None,
            actual_minus_native_reference_rms_vector_norm_rad_s=float(np.sqrt(np.mean(np.sum(error_native[held]**2,axis=1))))if n>1 else None,
            actual_minus_used_reference_rms_vector_norm_rad_s=float(np.sqrt(np.mean(np.sum(error_used[held]**2,axis=1))))if n>1 else None)
    # Globalbank versus phase-localordinary/initial states are different scopes.
    first=metadata[jobs[0]['case_id']]
    for case,meta in metadata.items():
        check(case+'/global_bank_config',meta['fixed_design']==first['fixed_design']and meta['gain_cache_sha256']==first['gain_cache_sha256']==protocol['execution']['gain_cache_sha256']and meta['design_belt_cap_m_s']==.3 and meta['configuration_unchanged']is True)
        check(case+'/only_normal_parameter_changes',parameter_records[case]==parameter_records[jobs[0]['case_id']])
    for code in ('m100','p030','p090'):
        group=[j['case_id']for j in jobs if j['phase_key']==code]
        if not group:continue
        anchor=code+'_B';reference=data[anchor]
        for case in group:
            check(case+'/phase_local_ordinary',metadata[case]['ordinary_design_sha256']==metadata[anchor]['ordinary_design_sha256'])
            keys=('exact_qpos','exact_qvel','exact_previous_applied','exact_observer_wrench','exact_estimated_normal','exact_nominal_joints_before','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2','exact_reference_quat')
            check(case+'/phase_local_initial_state',all(same(data[case][key][0],reference[key][0])for key in keys))
        for a,b in ((code+'_B',code+'_R'),(code+'_N',code+'_NR')):
            if a in data and b in data:check(a+'='+b+'/initialK',same(data[a]['exact_feedback_gain'][0],data[b]['exact_feedback_gain'][0]))
    parity={}
    for case,key in (('p090_B','i51_base030_precision'),('p090_R','i51_reference030_precision')):
        if case not in data:continue
        original=load(ROOT/protocol['inputs'][key]);parity[case]={field:field in data[case]and same(data[case][field],value)for field,value in original.items()}
        check(case+'/I51_all_common_precision_byte_parity',all(parity[case].values()),fields=len(parity[case]))
    compatibility={}
    if not early:
        for code in ('m100','p030','p090'):
            old=load(ROOT/protocol['inputs']['old_'+code+'_rollout']);fresh=load(folders[code+'_B']/'rollout.npz')
            compatibility[code]={field:same(fresh[field],old[field])for field in ('time_s','qpos','qvel','command','input','reference_xy')if field in fresh and field in old}
    for relative,expected in inputs.items():check('input_unchanged_after/'+relative,sha(ROOT/relative)==expected)
    for row in records:check('qa_source_unchanged_after/'+row['origin'],sha(ROOT/row['origin'])==row['sha256'])
    report=dict(iteration='I-52',passed=all(row['passed']for row in checks),early_pair_only=early,case_count=len(jobs),checks=checks,case_contract_summary=reports,
        exact_I51_p090_parity=parity,historical_float32_rollout_compatibility=compatibility,
        resolved_case_paths={case:str(folder.relative_to(ROOT))for case,folder in folders.items()},technical_attempts=technical_metadata,
        attempt_scope='Twelve registered policycells; onepreserved originalnativeexit139hasnoscorabletrajectory, andoneidentical-policytechnicalretry suppliesm100_N. Nooutcomeselectionorcontrollerretuning.',
        historical_scope='Oldfloat32compatibility isdescriptive, notnewbaselineacceptance orcompletehistoricalsource/provenance equivalence. Allconfiguredcases retained includingearlytermination.',
        scope='Independent rawsource/clock/phase/reference/projection integrity. No candidateimports/controller/physics/performanceanalysis. Assertioncount isnotrepo pytestcount.')
    save(output/'report.json',report)
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),bytes=p.stat().st_size,sha256=sha(p))for p in sorted(output.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
    print(json.dumps(safe(dict(passed=report['passed'],early_pair_only=early,checks=len(checks),cases=len(jobs),failed=[row for row in checks if not row['passed']])),indent=2))
    if not report['passed']:raise AssertionError('Independent native QA found an integrity failure')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--early',action='store_true');parser.add_argument('--resolver',type=Path);args=parser.parse_args();main(args.output.resolve(),args.early,args.resolver)
