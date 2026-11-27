"""Bounded, posthoc I74 command-ledger audit. No physics or estimator replay.

Select each of the six recorded maximum HELD normalized clipping-excess rows,
earliest tie, with fixed +/-25ms context. Identify algebraic command sources;
do not infer a unique physical cause or revise the original screen.
"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
import numpy as np
import yaml

STAGE=Path(__file__).resolve().parent
BASE=STAGE.parents[1]
ROOT=BASE.parents[1]
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
        'heading__target_rad','heading__raw_error_rad','heading__error_used_rad','heading__eligible',
        'heading__vertical_lever','heading__horizontal_lever','heading__free_shell_mask',
        'heading__effective_motor_limit_Nm','heading__wanted_world_yaw_torque_Nm')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
    if isinstance(v,dict):return {str(k):safe(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [safe(x) for x in v]
    if isinstance(v,np.ndarray):return safe(v.tolist())
    if isinstance(v,np.generic):return safe(v.item())
    if isinstance(v,float) and not np.isfinite(v):return None
    return v
def write(p,v):p.write_text(json.dumps(safe(v),indent=2,sort_keys=True,allow_nan=False)+'\n')


def main():
    if (STAGE/'report.json').exists() or (STAGE/'registered_before_arithmetic.json').exists():
        raise FileExistsError('Keep prior audit execution/stages unchanged')
    protocol_path=BASE/'protocol/protocol.json';seal_path=BASE/'campaign/outputs_sealed.json'
    science_path=BASE/'evaluation/completed_stage01/science.json'
    protocol=json.loads(protocol_path.read_text());seal=json.loads(seal_path.read_text())
    jobs=protocol['cases']
    if len(jobs)!=6 or seal['protocol_sha256']!=sha(protocol_path):raise ValueError('Registered I74 six-case seal required')
    paths=[Path(__file__),protocol_path,seal_path,science_path,ROOT/'config/model.yaml',ROOT/'src/spherical_wheel/offroad_control.py']
    paths += [BASE/'campaign/runs'/job['case_id']/'precision.npz' for job in jobs]
    for p in paths:
        relative=str(p.relative_to(BASE)) if p.is_relative_to(BASE) else None
        if relative in seal['files'] and sha(p)!=seal['files'][relative]['sha256']:
            raise ValueError('Recorded case data changed')
    for p in (ROOT/'config/model.yaml',ROOT/'src/spherical_wheel/offroad_control.py'):
        if protocol['source_sha256'].get(str(p.relative_to(ROOT)))!=sha(p):raise ValueError('Native source/config changed')
    bindings=[dict(path=str(p),sha256=sha(p),bytes=p.stat().st_size) for p in paths]
    write(STAGE/'registered_before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        inputs=bindings,root_bounded_audit_approval_received=True,selection='Each case maximum held norm((request-clip(request))/caps), earliest tie',
        context_half_width_s=.025,ledger_atol=1e-9,ledger_rtol=1e-12,no_new_physics=True,no_new_truth_or_estimator_inference=True,
        original_decision_unchanged=True,source_error_blocks=BLOCKS,
        derived_shaft_note='Native error6 is heading-relative XY error0/R minus pitch error4; it is not an independent measured shaft-position error.'))
    radius=float(yaml.safe_load((ROOT/'config/model.yaml').read_text())['geometry']['sphere_radius_m'])
    science=json.loads(science_path.read_text());reports={}
    for job in jobs:
        case=job['case_id'];path=BASE/'campaign/runs'/case/'precision.npz'
        with np.load(path,allow_pickle=False) as z:
            missing=set(FIELDS)-set(z.files)
            if missing:raise ValueError(f'Missing logged ledger fields for {case}: {missing}')
            d={k:z[k].copy() for k in FIELDS}
            authority_names=sorted(k for k in z.files if any(s in k.lower() for s in ('authority','condition','jacobian')))
            authority={k:z[k].copy() for k in authority_names}
        t=d['time_s'];held=d['command_held_after_row'].astype(bool)
        clipped=np.clip(d['exact_requested'],-d['exact_input_bounds'],d['exact_input_bounds'])
        excess=np.linalg.norm((d['exact_requested']-clipped)/d['exact_input_bounds'],axis=1)
        eligible=np.flatnonzero(held)
        index=int(eligible[np.argmax(excess[eligible])])
        center=float(t[index]);context=np.flatnonzero(abs(t-center)<=.025+1e-9)
        current={key:value[context].copy() for key,value in d.items()}
        current['source_frame_index']=context
        current['normalized_clipping_excess']=excess[context]
        for key,value in authority.items():current[key]=value[context].copy()
        np.savez_compressed(STAGE/f'{case}_context.npz',**current)
        K=d['exact_feedback_gain'][context];e=d['exact_error'][context]
        lqr=np.array([(-k)@x for k,x in zip(K,e)])
        blocks={name:np.array([(-k[:,lo:hi])@x[lo:hi] for k,x in zip(K,e)]) for name,lo,hi in BLOCKS}
        pre=d['exact_equilibrium_command'][context]+d['exact_rolling_feedforward'][context]+d['exact_support_command'][context]+d['exact_lqr_command'][context]
        requested=pre+d['exact_transition_command_delta'][context]+d['exact_heading_command'][context]+d['exact_fade_command_delta'][context]
        actual=requested+d['exact_clipping_command_delta'][context]
        derived=e[:,0]/radius-e[:,4]
        pairs=dict(lqr_full=(lqr,d['exact_lqr_command'][context]),
            lqr_blocks=(sum(blocks.values()),d['exact_lqr_command'][context]),
            requested_ledger=(requested,d['exact_requested'][context]),
            applied_ledger=(actual,d['exact_applied'][context]),
            applied_is_clipped=(clipped[context],d['exact_applied'][context]),
            derived_rolling_shaft_error=(derived,e[:,6]),
            joint_velocity_error=(d['exact_qvel'][context,6:9]-d['exact_feedback_reference_qvel'][context,6:9],e[:,15:18]))
        verification={}
        for name,(a,b) in pairs.items():
            passed=bool(np.allclose(a,b,atol=1e-9,rtol=1e-12))
            verification[name]=dict(passed=passed,max_abs_difference=float(np.max(abs(a-b))))
            if not passed:raise AssertionError(f'{case}: ledger failure {name}')
        local=int(np.flatnonzero(context==index)[0]);ledger={key:d[key][index] for key in (
            'exact_requested','exact_applied','exact_input_bounds','exact_equilibrium_command','exact_rolling_feedforward',
            'exact_lqr_command','exact_support_command','exact_transition_command_delta','exact_heading_command',
            'exact_fade_command_delta','exact_clipping_command_delta')}
        center_blocks={name:value[local] for name,value in blocks.items()}
        reports[case]=dict(source_frame=index,time_s=center,held=True,normalized_clipping_excess=float(excess[index]),
            zero_excess_tie=bool(excess[index]==0.),ledger=ledger,lqr_error_blocks=center_blocks,
            actual_joint_velocity_rad_s=d['exact_qvel'][index,6:9],reference_joint_velocity_rad_s=d['exact_reference_qvel'][index,6:9],
            feedback_reference_joint_velocity_rad_s=d['exact_feedback_reference_qvel'][index,6:9],
            joint_velocity_error_rad_s=d['exact_error'][index,15:18],
            rolling_allocation_reference=d['exact_allocation_drive_reference'][index],
            error18=d['exact_error'][index],qpos10=d['exact_qpos'][index],
            native_estimated_support_loads_N=d['exact_estimated_support_loads_N'][index],
            native_estimated_normal=d['exact_estimated_normal'][index],
            effective_phase_rad=d['exact_effective_phase_rad'][index],drive_fade=d['exact_drive_fade'][index],
            heading={k:d[k][index] for k in FIELDS if k.startswith('heading__')},
            context=dict(first_s=float(t[context[0]]),last_s=float(t[context[-1]]),samples=len(context),half_width_s=.025,
                         path=f'{case}_context.npz',source_frame_indices=context),
            verification=verification,normalized_contact_geometry_authority=dict(available=bool(authority_names),
                logged_field_names=authority_names,values_at_row={k:v[index] for k,v in authority.items()},
                scope='Only already logged fields; no Jacobian/model/terrain/contact recomputation.'),
            canonical_peak_matches=bool(abs(excess[index]-science['cases'][case]['commands']['normalized_clipping_excess']['maximum'])<=1e-9))
        if not reports[case]['canonical_peak_matches']:raise AssertionError('Canonical maximum mismatch')
    for item in bindings:
        if sha(item['path'])!=item['sha256']:raise ValueError('Audit inputs changed')
    write(STAGE/'report.json',dict(iteration='I-74',posthoc=True,source='Original full-precision records',cases=reports,
        all_ledgers_verified=True,no_new_physics=True,no_new_inference=True,original_decision_unchanged=True,
        interpretation_limit='Algebraic command decomposition is not a unique physical cause. Reference singularity/authority is not established when not logged.'))
    write(STAGE/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(STAGE.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(safe({case:dict(time_s=r['time_s'],normalized_excess=r['normalized_clipping_excess'],
        request=r['ledger']['exact_requested'],applied=r['ledger']['exact_applied'],blocks=r['lqr_error_blocks'],
        actual_joint_rates=r['actual_joint_velocity_rad_s'],reference_joint_rates=r['reference_joint_velocity_rad_s']) for case,r in reports.items()})))


if __name__=='__main__':main()
