"""Bounded descriptive I75 audit after the frozen six-case screen.

No physics, new contact/terrain inference, Jacobian, tuning, or gate changes.
Use each case's earliest maximum held clipping-excess row and +/-25ms ledger.
Report the existing native LQR angular-rate block and overlay motor alignment;
alignment in command coordinates does not establish physical yaw authority.
"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
import numpy as np
import yaml

STAGE=Path(__file__).resolve().parent
BASE=STAGE.parents[1]
ROOT=BASE.parents[1]
THRESHOLD_NM=1e-4
BLOCKS=(('XY_position',0,2),('height',2,3),('roll_pitch_attitude',3,5),
        ('heading_position',5,6),('derived_rolling_shaft_error',6,7),
        ('cyclic_shell_position',7,9),('body_linear_velocity',9,12),
        ('body_angular_velocity',12,15),('shaft_velocity',15,16),('shell_velocities',16,18))
FIELDS=('time_s','command_held_after_row','exact_requested','exact_applied','exact_input_bounds',
        'exact_equilibrium_command','exact_rolling_feedforward','exact_lqr_command','exact_support_command',
        'exact_transition_command_delta','exact_heading_command','exact_fade_command_delta','exact_clipping_command_delta',
        'exact_feedback_gain','exact_error','exact_qpos','exact_qvel','exact_reference_qpos','exact_reference_qvel',
        'exact_feedback_reference_qvel','exact_allocation_drive_reference','exact_estimated_support_loads_N',
        'exact_estimated_normal','exact_effective_phase_rad','exact_drive_fade',
        'heading__raw_heading_rad','heading__raw_heading_rate_rad_s','heading__command_heading_rad',
        'heading__raw_error_rad','heading__command_error_rad','heading__error_used_rad','heading__eligible',
        'heading__vertical_lever','heading__horizontal_lever','heading__free_shell_mask',
        'heading__effective_motor_limit_Nm','heading__wanted_world_yaw_torque_Nm',
        'heading__euler_heading_rate_rad_s','heading__world_z_rate_rad_s')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def safe(v):
    if isinstance(v,dict):return {str(k):safe(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [safe(x) for x in v]
    if isinstance(v,np.ndarray):return safe(v.tolist())
    if isinstance(v,np.generic):return safe(v.item())
    if isinstance(v,float) and not np.isfinite(v):return None
    return v
def write(path,value):path.write_text(json.dumps(safe(value),indent=2,allow_nan=False)+'\n')
def stats(a):
    a=np.asarray(a,float)
    return dict(count=len(a),rms=float(np.sqrt(np.mean(a*a))) if len(a) else None,
        mean=float(np.mean(a)) if len(a) else None,minimum=float(np.min(a)) if len(a) else None,
        maximum=float(np.max(a)) if len(a) else None)
def peak_components(t,a,mask):
    indices=np.flatnonzero(mask);values=[]
    if a.ndim==1:a=a[:,None]
    for j in range(a.shape[1]):
        i=int(indices[np.argmax(abs(a[indices,j]))])
        values.append(dict(channel=j,time_s=float(t[i]),source_frame=i,signed_value=float(a[i,j]),absolute_value=float(abs(a[i,j]))))
    return values


def main():
    if (STAGE/'registered_before_arithmetic.json').exists():raise FileExistsError('Preserve prior stages')
    protocol_path=BASE/'protocol/protocol.json';seal_path=BASE/'campaign/outputs_sealed.json'
    science_path=BASE/'evaluation/completed_stage01/science.json'
    protocol=json.loads(protocol_path.read_text());seal=json.loads(seal_path.read_text());jobs=protocol['cases']
    if protocol['iteration']!='I-75' or len(jobs)!=6 or seal['protocol_sha256']!=sha(protocol_path):raise ValueError('I75 six-case seal required')
    paths=[Path(__file__),protocol_path,seal_path,science_path,ROOT/'config/model.yaml',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/sim.py']
    paths += [BASE/'campaign/runs'/j['case_id']/'precision.npz' for j in jobs]
    paths += [BASE/'evaluation/completed_stage01'/(j['case_id']+'_signals.npz') for j in jobs]
    for path in paths:
        relative=str(path.relative_to(BASE)) if path.is_relative_to(BASE) else None
        if relative in seal['files'] and sha(path)!=seal['files'][relative]['sha256']:raise ValueError('Recorded case changed')
    for path in (ROOT/'config/model.yaml',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/sim.py'):
        if protocol['source_sha256'].get(str(path.relative_to(ROOT)))!=sha(path):raise ValueError('Native source/config changed')
    bindings=[dict(path=str(path),sha256=sha(path),bytes=path.stat().st_size) for path in paths]
    write(STAGE/'registered_before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),inputs=bindings,
        posthoc=True,root_bounded_audit_approval_received=True,no_new_physics=True,no_new_contact_or_terrain_inference=True,
        no_Jacobian_or_new_model=True,no_gate_changes=True,selection='Each case earliest maximum held normalized clipping excess',
        context_half_width_s=.025,interaction_window='[3,35)',motor_norm_threshold_Nm=THRESHOLD_NM,
        interaction_definition='First3 torque components. Cosine=(a dot h)/(norm(a)norm(h)) only if both norms>=1e-4Nm; a=-K[:3,12:15]e[12:15],h=heading_command[:3].',
        interpretation_limit='Motor-space alignment is not net yaw torque, physical authority, or proof of a causal disturbance.',
        source_error_blocks=BLOCKS,derived_shaft_note='Native error6=heading-relative XY error0/radius minus pitch error4; keep this separate from angular-rate feedback.'))
    radius=float(yaml.safe_load((ROOT/'config/model.yaml').read_text())['geometry']['sphere_radius_m'])
    science=json.loads(science_path.read_text());reports={}
    for job in jobs:
        case=job['case_id'];path=BASE/'campaign/runs'/case/'precision.npz'
        with np.load(path,allow_pickle=False) as z:
            missing=set(FIELDS)-set(z.files)
            if missing:raise ValueError(f'{case} missing {missing}')
            d={k:z[k].copy() for k in FIELDS}
        with np.load(BASE/'evaluation/completed_stage01'/(case+'_signals.npz'),allow_pickle=False) as z:
            signals={k:z[k].copy() for k in ('time_s','goal_distance_m','heading_error_abs_deg','body_xy_speed_m_s','reference_up_error_deg')}
        t=d['time_s'];held=d['command_held_after_row'].astype(bool);moving=held&(t>=3.-1e-9)&(t<35.-1e-9)
        K=d['exact_feedback_gain'];e=d['exact_error']
        blocks={name:-np.einsum('nij,nj->ni',K[:,:,lo:hi],e[:,lo:hi]) for name,lo,hi in BLOCKS}
        clipped=np.clip(d['exact_requested'],-d['exact_input_bounds'],d['exact_input_bounds'])
        excess=np.linalg.norm((d['exact_requested']-clipped)/d['exact_input_bounds'],axis=1)
        held_indices=np.flatnonzero(held);index=int(held_indices[np.argmax(excess[held_indices])]);center=float(t[index])
        context=np.flatnonzero(abs(t-center)<=.025+1e-9)
        pre=d['exact_equilibrium_command']+d['exact_rolling_feedforward']+d['exact_support_command']+d['exact_lqr_command']
        requested=pre+d['exact_transition_command_delta']+d['exact_heading_command']+d['exact_fade_command_delta']
        actual=requested+d['exact_clipping_command_delta']
        pairs=dict(lqr_full=(-np.einsum('nij,nj->ni',K,e),d['exact_lqr_command']),
            lqr_blocks=(sum(blocks.values()),d['exact_lqr_command']),requested_ledger=(requested,d['exact_requested']),
            applied_ledger=(actual,d['exact_applied']),applied_is_clipped=(clipped,d['exact_applied']),
            derived_rolling_shaft_error=(e[:,0]/radius-e[:,4],e[:,6]),
            body_angular_velocity_error=(d['exact_qvel'][:,3:6]-d['exact_feedback_reference_qvel'][:,3:6],e[:,12:15]),
            joint_velocity_error=(d['exact_qvel'][:,6:9]-d['exact_feedback_reference_qvel'][:,6:9],e[:,15:18]))
        verification={}
        for name,(a,b) in pairs.items():
            passed=bool(np.allclose(a,b,atol=1e-9,rtol=1e-12));verification[name]=dict(passed=passed,max_abs_difference=float(np.max(abs(a-b))))
            if not passed:raise AssertionError(case+' ledger '+name)
        zero_refs={name:bool(np.array_equal(d[name][:,3:6],np.zeros_like(d[name][:,3:6]))) for name in ('exact_reference_qvel','exact_feedback_reference_qvel')}
        if not all(zero_refs.values()):raise AssertionError('Declared zero angular-reference policy changed')
        a=blocks['body_angular_velocity'][:,:3];h=d['exact_heading_command'][:,:3]
        an=np.linalg.norm(a,axis=1);hn=np.linalg.norm(h,axis=1);dot=np.sum(a*h,axis=1)
        qualify=moving&(an>=THRESHOLD_NM)&(hn>=THRESHOLD_NM)
        cosine=np.full(len(t),np.nan);cosine[qualify]=dot[qualify]/(an[qualify]*hn[qualify])
        angular=dict(zero_angular_references_verified=zero_refs,window='[3,35)',motor_torque_norm_threshold_Nm=THRESHOLD_NM,
            moving_samples=int(moving.sum()),qualifying_samples=int(qualify.sum()),qualifying_seconds=float(qualify.sum()*.005),
            angular_rate_block_motor_norm_Nm=stats(an[moving]),heading_helper_motor_norm_Nm=stats(hn[moving]),
            derived_shaft_block_motor_norm_Nm=stats(np.linalg.norm(blocks['derived_rolling_shaft_error'][:,:3],axis=1)[moving]),
            qualifying_torque_dot_Nm2=stats(dot[qualify]),qualifying_cosine=stats(cosine[qualify]),
            negative_cosine_fraction=float(np.mean(cosine[qualify]<0.)) if qualify.any() else None,
            strong_negative_cosine_fraction_below_minus_half=float(np.mean(cosine[qualify]<-.5)) if qualify.any() else None,
            interpretation_limit='Command-coordinate torque alignment; not inferred net physical yaw torque or a unique physical cause.')
        current={key:value[context].copy() for key,value in d.items()};current['source_frame_index']=context
        current['normalized_clipping_excess']=excess[context]
        for name,value in blocks.items():current['lqr_block__'+name]=value[context].copy()
        np.savez_compressed(STAGE/(case+'_context.npz'),**current)
        np.savez_compressed(STAGE/(case+'_interaction.npz'),time_s=t[moving],
            angular_rate_block_motor_Nm=a[moving],heading_helper_motor_Nm=h[moving],
            derived_rolling_shaft_block_motor_Nm=blocks['derived_rolling_shaft_error'][moving,:3],
            qualifies=qualify[moving],torque_dot_Nm2=dot[moving],cosine=cosine[moving])
        fixed={}
        for value in (35.,45.):
            i=int(np.argmin(abs(t-value)))
            if abs(t[i]-value)>1e-9:raise ValueError('Missing fixed endpoint sample')
            fixed[str(value)]=dict(time_s=float(t[i]),source_frame=i,command_is_held=bool(held[i]),
                goal_distance_m=signals['goal_distance_m'][i],raw_heading_error_deg=signals['heading_error_abs_deg'][i],
                body_xy_speed_m_s=signals['body_xy_speed_m_s'][i],raw_reference_up_error_deg=signals['reference_up_error_deg'][i],
                heading_helper_command=d['exact_heading_command'][i],heading_eligible=d['heading__eligible'][i],
                free_shell_mask=d['heading__free_shell_mask'][i],vertical_lever=d['heading__vertical_lever'][i],
                estimated_support_loads_N=d['exact_estimated_support_loads_N'][i],
                requested_command=d['exact_requested'][i],applied_command=d['exact_applied'][i])
        ledger={key:d[key][index] for key in ('exact_requested','exact_applied','exact_input_bounds','exact_equilibrium_command',
            'exact_rolling_feedforward','exact_lqr_command','exact_support_command','exact_transition_command_delta',
            'exact_heading_command','exact_fade_command_delta','exact_clipping_command_delta')}
        reports[case]=dict(case=job,angular_rate_block_interaction=angular,full_held_extrema=dict(
            actual_joint_velocity_rad_s=peak_components(t,d['exact_qvel'][:,6:9],np.ones(len(t),bool)),
            raw_joint_reference_rad_s=peak_components(t,d['exact_reference_qvel'][:,6:9],held),
            feedback_joint_reference_rad_s=peak_components(t,d['exact_feedback_reference_qvel'][:,6:9],held),
            allocation_reference_mixed_units=peak_components(t,d['exact_allocation_drive_reference'],held),
            rolling_feedforward_mixed_units=peak_components(t,d['exact_rolling_feedforward'],held),
            lqr_command_mixed_units=peak_components(t,d['exact_lqr_command'],held),
            requested_mixed_units=peak_components(t,d['exact_requested'],held),applied_mixed_units=peak_components(t,d['exact_applied'],held)),
            maximum_held_clipping_event=dict(source_frame=index,time_s=center,normalized_clipping_excess=excess[index],
                earliest_zero_excess_tie=bool(excess[index]==0.),ledger=ledger,
                lqr_error_blocks={name:value[index] for name,value in blocks.items()},error18=e[index],
                actual_joint_velocity_rad_s=d['exact_qvel'][index,6:9],reference_joint_velocity_rad_s=d['exact_reference_qvel'][index,6:9],
                feedback_reference_joint_velocity_rad_s=d['exact_feedback_reference_qvel'][index,6:9],
                heading={key:d[key][index] for key in FIELDS if key.startswith('heading__')},
                estimated_support_loads_N=d['exact_estimated_support_loads_N'][index],
                context=dict(first_s=float(t[context[0]]),last_s=float(t[context[-1]]),samples=len(context),path=case+'_context.npz')),
            fixed_endpoint_samples=fixed,verification=verification)
        if abs(excess[index]-science['cases'][case]['commands']['normalized_clipping_excess']['maximum'])>1e-9:raise AssertionError('Canonical peak mismatch')
    for item in bindings:
        if sha(item['path'])!=item['sha256']:raise ValueError('Audit source/input changed')
    write(STAGE/'report.json',dict(iteration='I-75',posthoc=True,cases=reports,all_ledgers_verified=True,
        canonical_status_unchanged=science['decision']['status'],no_new_physics=True,no_new_inference=True,no_gate_changes=True,
        mixed_unit_note='5-command first3 are motor torques Nm; last2 are prescribed belt speeds m/s. Joint qvel channels6:9 are shaft,shellA,shellB rad/s.',
        interpretation_limit='Algebraic decomposition and motor-space alignment do not establish unique physical cause; no new Jacobian or contact prediction.'))
    write(STAGE/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size) for p in sorted(STAGE.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(safe({case:dict(interaction=r['angular_rate_block_interaction'],clip_event_s=r['maximum_held_clipping_event']['time_s'],
        clip_excess=r['maximum_held_clipping_event']['normalized_clipping_excess'],fixed=r['fixed_endpoint_samples']) for case,r in reports.items()})))


if __name__=='__main__':main()
