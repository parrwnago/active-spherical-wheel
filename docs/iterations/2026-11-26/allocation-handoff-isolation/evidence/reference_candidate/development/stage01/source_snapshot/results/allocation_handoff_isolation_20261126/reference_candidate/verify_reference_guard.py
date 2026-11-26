"""I51 pure contracts and saved-reference-input checks; no controller or physics."""
from datetime import datetime,timezone
from pathlib import Path
from types import SimpleNamespace
import argparse
import copy
import hashlib
import importlib.util
import json
import traceback

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
BASE=ROOT/'results/belt_capacity_sweep_20261126/campaign/runs/mars_cap030'
MATRIX=ROOT/'results/belt_handoff_event_20261126/sync_audit/stage05_rolling_geometry/report.json'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [safe(v)for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float)and not np.isfinite(x):return None
    return x
def save(path,x):path.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def digest(x):return hashlib.sha256(json.dumps(safe(x),sort_keys=True).encode()).hexdigest()
def require_raises(kind,callback):
    try:callback()
    except kind:return
    raise AssertionError('Expected '+kind.__name__)


def main(stage):
    stage.mkdir(parents=True,exist_ok=False)
    paths=[Path(__file__).resolve(),HERE/'joint_reference_guard.py',ROOT/'src/spherical_wheel/rolling_allocation.py',
        ROOT/'src/spherical_wheel/synchronization.py',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'ITERATION_LOG.md']
    records=[]
    for path in paths:
        dest=stage/'source_snapshot'/path.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(path.read_bytes())
        records.append(dict(origin=str(path.relative_to(ROOT)),snapshot=str(dest.relative_to(stage)),sha256=sha(path)))
    inputs={str(path.relative_to(ROOT)):sha(path)for path in (BASE/'precision.npz',MATRIX)}
    save(stage/'before_execution.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),sources=records,input_sha256=inputs,
        scope='Pure referenceguard unitcontracts and filtering the already saved ORIGINALcontrollerreferenceinputs. No controller/model/observer/preview/physics calls; filteredhistory is not a candidateclosedlooptrajectory.',
        selection='Existing20rad/s and100rad/s² assumptions. Saved matrixcases fixed31.645/.650/.655/.660; wellconditionedraised31.650; no parameter search.'))
    spec=importlib.util.spec_from_file_location('i51_reference_guard_frozen',HERE/'joint_reference_guard.py')
    module=importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name]=module;spec.loader.exec_module(module)
    checks=[]
    try:
        guard=module.JointReferenceBoxGuard()
        target=np.array([.2875419135140016,-0.,0.])
        emitted,diag=guard.update(target,0.,.005)
        assert emitted.tobytes()==target.tobytes() and diag['unmodified']
        assert target.tobytes()==np.array([.2875419135140016,-0.,0.]).tobytes()
        checks.append('Identity and signed-zero bit preservation when native target is inside speed/slew box')

        guard=module.JointReferenceBoxGuard();previous=np.zeros(3);previous_time=0.
        time=0.;largest_speed=largest_acceleration=0.
        for i in range(301):
            dt=.003 if i%3==0 else .005
            if i:time+=dt
            target=np.array([10000.,-20000.,30000.])*(1 if i<150 else -1)
            emitted,diag=guard.update(target,time,dt)
            elapsed=dt if i==0 else time-previous_time
            assert np.all(abs(emitted)<=20.+1e-12)
            assert np.all(abs(emitted-previous)<=100.*elapsed+1e-12)
            largest_speed=max(largest_speed,float(np.max(abs(emitted))))
            largest_acceleration=max(largest_acceleration,float(np.max(abs(emitted-previous)/elapsed)))
            previous,previous_time=emitted.copy(),time
        checks.append('Variable-step persistent speed/slew bounds under extreme targets and reversals')

        before=digest(guard.state_dict());emitted,diag=guard.update(target,time,.005)
        assert digest(guard.state_dict())==before and diag['repeated_timestamp'] and not diag['initialized_now']
        require_raises(ValueError,lambda:guard.update(target+1.,time,.005))
        require_raises(ValueError,lambda:guard.update(target,time-.001,.005))
        require_raises(ValueError,lambda:guard.update([np.nan,0.,0.],time+.005,.005))
        assert digest(guard.state_dict())==before
        copy_guard=copy.deepcopy(guard);copy_guard.update([0.,0.,0.],time+.005,.005)
        assert digest(guard.state_dict())==before
        returned=guard.state_dict();returned['previous_reference_rad_s'][:]=999.
        assert digest(guard.state_dict())==before
        guard.reset();assert guard.time_s is None and np.array_equal(guard.previous,np.zeros(3))
        checks.append('Clock/idempotence/rejection/reset/deepcopy and returned-state isolation')

        qpos=np.arange(10.);qvel=np.arange(9.);opaque=np.arange(6.)
        controller=SimpleNamespace(plant=SimpleNamespace(data=SimpleNamespace(time=0.,qpos=qpos,qvel=qvel),dt=.005),observer_payload=opaque)
        vref=np.array([.06,-.02,.003,.01,.02,.03,500.,-2000.,1000.]);before_body=vref[:6].copy()
        diagnostic=module.apply(controller,vref)
        np.testing.assert_array_equal(vref[:6],before_body);np.testing.assert_array_equal(vref[6:9],[.5,-.5,.5])
        np.testing.assert_array_equal(qpos,np.arange(10.));np.testing.assert_array_equal(qvel,np.arange(9.));np.testing.assert_array_equal(opaque,np.arange(6.))
        for i in range(1,50):
            controller.plant.data.time=i*.005;vref=np.r_[np.zeros(6),[100.,-100.,100.]];module.apply(controller,vref)
        controller.plant.data.time=.25;vref=np.zeros(9);module.apply(controller,vref)
        assert np.all(abs(vref[6:9])>0), 'Zero-motion command intentionally decelerates owned reference rather than resetting history'
        old=digest(controller._i51_joint_reference_guard.state_dict());readonly=vref.copy();readonly.flags.writeable=False
        require_raises(ValueError,lambda:module.apply(controller,readonly));assert digest(controller._i51_joint_reference_guard.state_dict())==old
        module.reset(controller);assert controller._i51_joint_reference_guard.time_s is None
        checks.append('Feedback hook mutates only joint refs; covers zero-motion deceleration, reset and read-only rejection')

        with np.load(BASE/'precision.npz',allow_pickle=False)as z:
            t=z['time_s'].copy();native=z['exact_reference_qvel'][:,6:9].copy();full_vref=z['exact_reference_qvel'].copy()
            gains=z['exact_feedback_gain'].copy();requests=z['exact_native_requested'].copy()
        guard=module.JointReferenceBoxGuard();emitted=[];modified=[]
        for now,row in zip(t,native):
            value,diag=guard.update(row,now,.005);emitted.append(value);modified.append(diag['modified'])
        emitted=np.asarray(emitted);modified=np.asarray(modified)
        assert np.max(abs(emitted))<=20.+1e-12
        assert np.max(abs(np.diff(emitted,axis=0))/np.diff(t)[:,None])<=100.+1e-9
        assert np.all(np.isfinite(emitted))
        # This is a saved INPUT replay of the pure referencefilter, not a new
        # state/control rollout. Candidate motion/contacts/history are unknown.
        np.savez_compressed(stage/'saved_reference_input_filter.npz',time_s=t,native_reference_rad_s=native,
            guarded_reference_rad_s=emitted,modified=modified)
        checks.append('All8801 saved ORIGINAL native referenceinputs yield finite speed/slew-bounded filter output')

        matrix_report=json.loads(MATRIX.read_text())
        event=next(row for row in matrix_report['cases']['mars_cap030']if abs(row['time_s']-31.65)<1e-8)
        i=int(event['index']);matrix=np.zeros((2,5));matrix[:,event['active_command_columns']]=np.asarray(event['D_world_xy'])
        residual=module.rolling_residual(matrix,np.asarray(event['rhs_m_s']),np.asarray(event['saved_drive_references']),emitted[i])
        assert residual['belt_feedforward_unchanged'] and residual['guarded_residual_norm_m_s']>residual['native_residual_norm_m_s']
        assert np.max(abs(emitted[i]))<=20.
        local_guard=module.JointReferenceBoxGuard();well=np.asarray(matrix_report['cases']['mars_cap060'][1]['saved_drive_references'])[:3]
        well_emitted,well_diag=local_guard.update(well,0.,.005)
        np.testing.assert_array_equal(well,well_emitted);assert well_diag['unmodified']
        checks.append('Saved31.650 singular case bounded with explicit increased rolling residual; wellconditioned feasible local case unchanged')

        counterfactual_delta=gains[i,:,15:18]@(emitted[i]-native[i])
        same_state_request=requests[i]+counterfactual_delta
        # Exact feedback algebra for unchanged pose/error except desiredjoint
        # velocity: error_new=error_old-delta_vref. No original controller run.
        save(stage/'saved_case.json',dict(time_s=t[i],native_joint_reference_rad_s=native[i],guarded_reference_rad_s=emitted[i],
            residual=residual,exact_body_reference_unchanged=full_vref[i,:6],
            same_state_algebra_only=dict(native_requested=requests[i],request_delta=counterfactual_delta,
                guarded_requested=same_state_request,meaning='Fixed native K/error/pose/loads only; not candidateclosedloopcontrol or physicalresponse.'),
            wellconditioned_local_case=dict(raw=well,emitted=well_emitted,initial_history='zero reference, one.005s feasibilitybox; not claimed as the candidatehistory at31.650.')))
        checks.append('Same-state feedback reference delta computed algebraically; no controller execution or feasibility claim')
        for row in records:assert sha(ROOT/row['origin'])==row['sha256']
        for rel,expected in inputs.items():assert sha(ROOT/rel)==expected
        report=dict(passed=True,checks=checks,contract_group_count=len(checks),source_hashes_unchanged_after=True,sources=records,input_sha256=inputs,
            pure_bound_check=dict(max_speed_rad_s=largest_speed,max_acceleration_rad_s2=largest_acceleration),
            saved_native_input_filter=dict(samples=len(t),modified_samples=int(modified.sum()),first_modified_time_s=float(t[np.flatnonzero(modified)[0]])if modified.any()else None,
                max_native_reference_rad_s=float(np.max(abs(native))),max_guarded_reference_rad_s=float(np.max(abs(emitted))),
                max_guarded_reference_acceleration_rad_s2=float(np.max(abs(np.diff(emitted,axis=0))/np.diff(t)[:,None]))),
            scope='Seven distinct pure/saved-input contractgroups, not repo pytest counts or robottrials. No candidateclosedloopoutcome or integration validation claimed.')
        save(stage/'report.json',report);print(json.dumps(safe(report),indent=2))
    except Exception:
        save(stage/'failure.json',dict(completed_checks=checks,traceback=traceback.format_exc(),sources=records,input_sha256=inputs));raise
    finally:
        save(stage/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(stage)),bytes=p.stat().st_size,sha256=sha(p))for p in sorted(stage.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--stage',type=Path,required=True);args=parser.parse_args();main(args.stage.resolve())
