"""Independent I78 saved-array verification; no controller, observer or physics.

Reconstruct the scalar ODE integration and straight-path chain rule directly,
without importing the candidate generator. Producer flags supplement rather
than replace numerical reference/policy/native-prefix checks.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import math
import shutil
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
OLD=ROOT/'results/tangent_heading_routes_20261127'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):
    with Path(p).open('x') as f:
        json.dump(x,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def token(a):
    a=np.asarray(a);return a.dtype.str,a.shape,np.ascontiguousarray(a).tobytes()
def row_change(a,b):
    a,b=np.asarray(a),np.asarray(b)
    if a.dtype!=b.dtype or a.shape!=b.shape:raise AssertionError('Row comparison schema differs')
    return np.any(a.view(np.uint8).reshape(len(a),-1)!=b.view(np.uint8).reshape(len(b),-1),axis=1)
def max_error(a,b):return float(np.max(np.abs(np.asarray(a)-np.asarray(b))))

def straight_reference(virtual,rate,rate_dot):
    """Independent closed-form straight geometry and quintic clock derivatives."""
    t=np.asarray(virtual,float);rate=np.asarray(rate,float);rate_dot=np.asarray(rate_dot,float)
    r=np.clip((t-3.)/32.,0.,1.)
    u=np.clip(10*r**3-15*r**4+6*r**5,0.,1.)
    du=(30*r*r-60*r**3+30*r**4)/32.
    ddu=(60*r-180*r*r+120*r**3)/(32.*32.)
    stopped=(t<=3.)|(t>=35.)
    du=np.where(stopped,0.,du);ddu=np.where(stopped,0.,ddu)
    angle=np.deg2rad(113.);first=1.2*np.array([np.cos(angle),np.sin(angle)])
    xy=np.array([-2.4,10.])+u[:,None]*first
    udot=du*rate;uddot=ddu*rate*rate+du*rate_dot
    return dict(u=u,udot=udot,uddot=uddot,xy=xy,
                velocity=udot[:,None]*first,acceleration=uddot[:,None]*first)

def independent_clock(times,new_targets,settings):
    """Same declared ODE, independently arranged Heun arithmetic/dense crossing."""
    tau=float(settings['time_constant_s']);limit=float(settings['maximum_stage_rate_s_inv'])
    maxstep=float(settings['maximum_substep_s'])
    z=rate=target=1.;virtual=float(times[0]);previous=float(times[0])
    ever=False;finished=None;first_complete=None;rows=[]
    def f(diff):return limit*math.tanh(diff/(limit*tau))
    for index,now_ in enumerate(times):
        now=float(now_)
        if index:
            if finished is None:
                if not ever and z==rate==target==1.:
                    virtual=now
                    if virtual>=35.:
                        virtual=35.;finished=35.;z=rate=target=0.
                else:
                    count=max(1,math.ceil((now-previous)/maxstep));h=(now-previous)/count
                    physical=previous
                    for _ in range(count):
                        fz=f(target-z);fr=f(z-rate)
                        ez=z+h*fz;er=rate+h*fr
                        nz=z+.5*h*(fz+f(target-ez))
                        nr=rate+.5*h*(fr+f(ez-er))
                        nt=virtual+h*rate+.5*h*h*fr
                        if nt>=35.:
                            distance=35.-virtual
                            denominator=rate+math.sqrt(max(0.,rate*rate+2*fr*distance))
                            crossing=2*distance/denominator if denominator>0 else h
                            finished=physical+min(h,max(0.,crossing))
                            virtual=35.;z=rate=target=0.;break
                        z,rate,virtual=nz,nr,nt
                        physical+=h
            previous=now
        rate_dot=0. if finished is not None else f(z-rate)
        if finished is not None and first_complete is None:first_complete=now
        rows.append((virtual,z,rate,rate_dot,
                     np.nan if finished is None else finished,
                     np.nan if first_complete is None else first_complete))
        target=float(new_targets[index])
        if finished is None and now>3. and target<1.:ever=True
    return np.asarray(rows)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--protocol',type=Path,required=True)
    ap.add_argument('--case-output',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();case=a.case_output.resolve();out=a.output.resolve()
    outcome_path=case/'i78_outcome.json';manifest_path=case/'artifact_manifest.json'
    # A final producer manifest is mandatory; never inspect a running case.
    manifest=json.loads(manifest_path.read_text())['files']
    outcome=json.loads(outcome_path.read_text());job=outcome['case'];policy=job['policy']
    parent_id=job['case_id'][:-1]+'N'
    precision=case/'child/campaign/runs'/parent_id/'precision.npz'
    sidecar=case/'governor_diagnostics.npz';prefix=case/'precision_prefix_receipt.json'
    baseline=OLD/'campaign/runs'/parent_id/'precision.npz'
    seal_path=OLD/'campaign/outputs_sealed.json'
    seal=json.loads(seal_path.read_text())['files'][str(baseline.relative_to(OLD))]
    if sha(baseline)!=seal['sha256'] or baseline.stat().st_size!=seal['bytes']:
        raise AssertionError('Original I76 recording differs from final seal')
    for p in (outcome_path,precision,sidecar,prefix):
        bound=manifest[str(p.relative_to(case))]
        if sha(p)!=bound['sha256'] or p.stat().st_size!=bound['bytes']:
            raise AssertionError('Closed producer output differs from manifest: '+str(p))
    paths=[Path(__file__).resolve(),a.protocol.resolve(),precision,sidecar,prefix,
           outcome_path,manifest_path,baseline,seal_path]
    bindings={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in paths}
    out.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),out/'verify_saved.py')
    write(out/'registered_before_array_reads.json',dict(iteration='I-78',
        utc=datetime.now(timezone.utc).isoformat(),files=bindings,policy=policy,
        methods='Independent direct scalar-Heun/quintic/chain-rule formulas, policy arithmetic, original-array prefix; no candidate core/controller/observer calls and no model or physics integration.',
        tolerances=dict(clock_time_s=2e-9,clock_states=2e-11,clock_acceleration_s_inv=2e-10,
                        references=3e-11,policy=2e-13),
        prefix_rule='Original79 fields byte-exact before first actual reference difference; physical/sensor-stage fields inclusive through that row. Terminal unheld-mask exception is separately explicit.'))
    p,s,b=load(precision),load(sidecar),load(baseline)
    if len(p)!=79 or set(p)!=set(b):raise AssertionError('Original precision schema differs')
    if any(v.dtype==object for v in p.values()) or any(v.dtype==object for v in s.values()):
        raise AssertionError('Object-array sidecar is not permitted')
    t=p['time_s'];n=len(t)
    np.testing.assert_array_equal(t,s['time_s'])
    if not np.all(np.isfinite(t)) or not np.allclose(np.diff(t),.005,rtol=0,atol=1e-10):
        raise AssertionError('Control clock is not contiguous5ms')
    for key in ('native_observer_update_count',):
        np.testing.assert_array_equal(p[key],np.ones(n,dtype=p[key].dtype))
    for key in ('native_feedback_boundary_verified','mirror_previous_applied_matches_physical',
                'no_truth_or_terrain_read_during_command'):
        if not np.all(p[key]):raise AssertionError('Native information/update guard failed: '+key)
    if np.any(p['exact_heading_command']):raise AssertionError('Heading overlay enabled')
    if np.any(s['private_dynamic_steps']):raise AssertionError('Private probe reports physics integration')
    np.testing.assert_array_equal(s['private_observer_updates'],np.full(n,1 if policy=='G'else 0))
    if policy=='G':
        for key in ('private_probe_present','live_probe_sensor_stages_checked',
                    'live_probe_sensor_stages_match','no_physical_contact_or_terrain_read_during_selection'):
            if not np.all(s[key]):raise AssertionError('Private probe boundary check failed: '+key)
    reference_fields=(('governed_xy','exact_reference_xy'),('governed_velocity_m_s','exact_reference_velocity_m_s'),
                      ('governed_acceleration_m_s2','exact_reference_acceleration_m_s2'))
    for left,right in reference_fields:np.testing.assert_array_equal(s[left],p[right])
    report=dict(iteration='I-78',case_id=job['case_id'],policy=policy,rows=n)
    common=min(n,len(b['time_s']));np.testing.assert_array_equal(t[:common],b['time_s'][:common])
    changed=np.zeros(common,bool)
    for _,right in reference_fields:changed|=row_change(p[right][:common],b[right][:common])
    first=int(np.flatnonzero(changed)[0])if changed.any()else common
    prefix_checks={};mask_exception=False
    for key in b:
        left=p[key][:first].copy();right=b[key][:first].copy()
        if key=='command_held_after_row' and first==n and n<len(b[key]):
            right[-1]=left[-1];mask_exception=True
        if key=='command_held_after_row' and first==len(b[key]) and n>first:
            right[-1]=left[-1];mask_exception=True
        prefix_checks[key]=token(left)==token(right)
    if not all(prefix_checks.values()):raise AssertionError('Pre-intervention original-array mismatch')
    inclusive={}
    if first<common:
        for key in ('exact_qpos','exact_qvel','exact_previous_applied','exact_nominal_joints_before',
                    'exact_observer_wrench','exact_observer_generalized','exact_observer_interval_mean',
                    'exact_estimated_normal','exact_feedback_gain','exact_support_command'):
            inclusive[key]=token(p[key][first])==token(b[key][first])
        if not all(inclusive.values()):raise AssertionError('First changed-reference sensor/state mismatch')
    if policy=='N':
        if n!=9001 or any(token(p[k])!=token(b[k])for k in b):
            raise AssertionError('Native baseline does not reproduce all79 complete arrays')
    report.update(original_prefix_fields=prefix_checks,complete_original_prefix_rows=first,
        first_changed_reference_row=None if first==common else first,
        inclusive_first_state_stage_checks=inclusive,terminal_prefix_mask_exception=mask_exception)
    virtual=s['gov__virtual_time_s'];rate=s['gov__clock_rate'];rate_dot=s['gov__clock_acceleration_s_inv']
    ref=straight_reference(virtual,rate,rate_dot)
    errors=dict(position_m=max_error(ref['xy'],s['governed_xy']),
        velocity_m_s=max_error(ref['velocity'],s['governed_velocity_m_s']),
        acceleration_m_s2=max_error(ref['acceleration'],s['governed_acceleration_m_s2']),
        geometric_u=max_error(ref['u'],s['gov__progress_u']),
        geometric_udot_s_inv=max_error(ref['udot'],s['gov__progress_rate_s_inv']),
        geometric_uddot_s_inv2=max_error(ref['uddot'],s['gov__progress_acceleration_s_inv2']))
    if max(errors.values())>3e-11:raise AssertionError('Coherent straight reference reconstruction failed')
    report['reference_formula_max_errors']=errors
    if policy=='G':
        protocol=json.loads(a.protocol.read_text());settings=protocol['governor_settings']
        sigma=np.linalg.svd(s['probe_authority_matrix']*np.array([5.,5.,5.,.3,.3]),compute_uv=False)[:,-1]
        joint=np.max(np.abs(s['probe_joint_reference']),axis=1)
        ratio=np.max(np.abs(s['probe_requested'])/p['exact_input_bounds'],axis=1)
        factor_sigma=np.minimum(1.,sigma/settings['authority_scale_m_s'])
        factor_joint=np.ones(n);active=joint>settings['joint_reference_scale_rad_s']
        factor_joint[active]=settings['joint_reference_scale_rad_s']/joint[active]
        factor_command=np.ones(n);active=ratio>settings['command_margin_fraction']
        factor_command[active]=settings['command_margin_fraction']/ratio[active]
        raw=np.minimum(np.minimum(factor_sigma,factor_joint),factor_command)
        gamma=np.where(s['gov__reference_complete'],0.,np.where(t<=3.,1.,raw))
        policy_errors=dict(sigma=max_error(sigma,s['gov__authority_sigma_min_m_s']),
            joint_demand=max_error(joint,s['gov__maximum_nominal_joint_reference_rad_s']),
            command_ratio=max_error(ratio,s['gov__nominal_command_ratio']),
            raw_target=max_error(raw,s['gov__raw_target_rate']),
            registered_target=max_error(gamma,s['gov__new_target_rate']))
        if max(policy_errors.values())>2e-13:raise AssertionError('Governor pressure policy differs')
        c=independent_clock(t,gamma,settings)
        clock_errors=dict(virtual_time_s=max_error(c[:,0],virtual),
            cascade_state=max_error(c[:,1],s['gov__cascade_state']),clock_rate=max_error(c[:,2],rate),
            clock_acceleration_s_inv=max_error(c[:,3],rate_dot))
        if (clock_errors['virtual_time_s']>2e-9 or clock_errors['cascade_state']>2e-11
                or clock_errors['clock_rate']>2e-11 or clock_errors['clock_acceleration_s_inv']>2e-10):
            raise AssertionError('Independent scalar integration differs')
        present=np.isfinite(c[:,4])
        if not np.array_equal(present,s['gov__reference_complete']):raise AssertionError('Reference completion mask differs')
        if present.any():
            if max_error(c[present,4],s['gov__finished_at_s'][present])>2e-8:
                raise AssertionError('Dense reference completion timestamp differs')
            if max_error(c[present,5],s['gov__first_complete_sample_s'][present])>1e-12:
                raise AssertionError('First reference-complete sample differs')
            if not np.all(s['governed_velocity_m_s'][present]==0.) or not np.all(s['governed_acceleration_m_s2'][present]==0.):
                raise AssertionError('Terminal reference is not stationary')
        if (np.min(rate)<-1e-12 or np.max(rate)>1.+1e-12
                or np.min(np.diff(virtual))<-2e-11
                or np.min(np.diff(t-virtual))<-2e-11):
            raise AssertionError('Clock sped above envelope, reversed or erased delay')
        identical=np.ones(n,bool)
        for left,right in [('governed_xy','nominal_virtual_xy'),('governed_velocity_m_s','nominal_virtual_velocity_m_s'),
                           ('governed_acceleration_m_s2','nominal_virtual_acceleration_m_s2')]:
            identical&=~row_change(s[left],s[right])
        np.testing.assert_array_equal(identical,s['full_output_identity_checked'])
        np.testing.assert_array_equal(s['gov__nominal_probe_calls'],np.arange(1,n+1))
        report.update(policy_formula_max_errors=policy_errors,clock_ode_max_errors=clock_errors,
            full_output_identity_rows=int(identical.sum()),reference_complete=bool(present.any()),
            observed_reference_completion_s=None if not present.any()else float(c[present,4][0]))
    for relative,bound in bindings.items():
        pth=ROOT/relative
        if sha(pth)!=bound['sha256'] or pth.stat().st_size!=bound['bytes']:
            raise AssertionError('Verification source/input changed')
    report.update(passed=True,source_inputs_unchanged=True,new_physics=False,
        repo_suite_rerun=False,interpretation='Integrity/identity/reference checks, not proof of tracking/balance improvement, hard feasibility, physical goal completion or absence of stall.')
    write(out/'report.json',report)
    write(out/'artifact_manifest.json',dict(files={p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(out.iterdir())if p.is_file()}))
    print(json.dumps(dict(passed=True,case_id=job['case_id'],rows=n,reference_errors=errors)),flush=True)

if __name__=='__main__':main()
