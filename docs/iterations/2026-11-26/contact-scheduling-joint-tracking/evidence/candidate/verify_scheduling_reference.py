"""I52 pure composition/frame/source contracts; no model/controller execution."""
from datetime import datetime,timezone
from pathlib import Path
from types import SimpleNamespace as NS
import argparse
import copy
import hashlib
import importlib.util
import json

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OLD=ROOT/'results/allocation_handoff_isolation_20261126/reference_candidate/joint_reference_guard.py'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v)for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    return x
def save(path,x):path.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def require_raises(kind,callback):
    try:callback()
    except kind:return
    raise AssertionError('Expected '+kind.__name__)
def ry(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rx(a):
    c,s=np.cos(a),np.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def wrap(a):return float((a+np.pi)%(2*np.pi)-np.pi)
def fake(phase,normal_phase):
    return NS(params=NS(schedule_on_ground_normal=normal_phase),carrier_id=0,
        plant=NS(dt=.005,data=NS(time=0.,xmat=ry(phase).reshape(1,9),qvel=np.arange(9.)/10)),
        ordinary=dict(design=dict(K=np.full((5,18),phase))))


def main(stage):
    stage.mkdir(parents=True,exist_ok=False)
    paths=[Path(__file__).resolve(),HERE/'scheduling_reference.py',OLD,
        ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/recovery_control.py']
    records=[]
    for source in paths:
        target=stage/'source_snapshot'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes())
        records.append(dict(origin=str(source.relative_to(ROOT)),snapshot=str(target.relative_to(stage)),sha256=sha(source)))
    save(stage/'before_execution.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),sources=records,
        scope='Pure structural controllers containingarraysonly, mathematicalrotationfixtures andfrozen-source binding. No MuJoCo, simulator, observer or nativecontroller imports/calls. Original phases−1,+.3,+.9 withallfourfactorbits.'))
    spec=importlib.util.spec_from_file_location('i52_candidate_adapter',HERE/'scheduling_reference.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    checks=[]
    try:
        identity=module.guard_identity();assert identity['sha256']==sha(OLD)
        assert identity['speed_limits_rad_s']==[20.]*3 and identity['reference_acceleration_limits_rad_s2']==[100.]*3
        tampered=stage/'tampered_guard_fixture.py';tampered.write_bytes(OLD.read_bytes()+b'\n# deliberate wrong-hash fixture\n')
        require_raises(ValueError,lambda:module.bind_guard(tampered));assert sha(OLD)==identity['sha256']
        checks.append('ExactI51sourcehash/settings bound; deliberatelytamperedcopy rejected withoutoldsourcechange')

        for phase in (-1.,.3,.9):
            for roll in (-.3,0.,.25):
                rotation=rz(.7)@ry(phase)@rx(roll)
                diag=module.phase_diagnostics(rotation,np.array([0.,0.,1.]),normal_phase=False)
                other=module.phase_diagnostics(rotation,np.array([0.,0.,1.]),normal_phase=True)
                assert diag['phase_world_z_rad']==diag['phase_estimated_normal_rad']==other['phase_used_rad']
        checks.append('WorldZnormal givesidenticalnative/normal lookupangle acrossallphases andobliquerotations')

        for phase in (-1.,.3,.9):
            for slope in (-.2,.15):
                normal=np.array([np.sin(slope),0.,np.cos(slope)])
                diag=module.phase_diagnostics(ry(phase),normal,normal_phase=True)
                assert abs(wrap(diag['phase_world_z_rad']-phase))<1e-14
                assert abs(wrap(diag['phase_used_rad']-(phase-slope)))<1e-14
                scaled=module.phase_diagnostics(ry(phase),normal*3.1,normal_phase=True)
                assert abs(wrap(scaled['phase_used_rad']-diag['phase_used_rad']))<1e-14
        checks.append('Independenttiltedplaneformula phase=carrierpitch−normalpitch andnormalscalinginvariance')

        for phase in (-1.,.3,.9):
            rotation=rz(-.8)@ry(phase)@rx(.23)
            for expected in (-2.7,-.7,.5,2.9):
                body_normal=np.array([-.8*np.sin(expected),.6,.8*np.cos(expected)])
                diag=module.phase_diagnostics(rotation,rotation@body_normal,normal_phase=True)
                assert abs(wrap(diag['phase_used_rad']-expected))<1e-14
            undefined=module.phase_diagnostics(rotation,rotation@np.array([0.,1.,0.]),normal_phase=True)
            assert not undefined['normal_phase_coordinate_defined']
        checks.append('Obliqueframe/order/wrappingchecks andundefinedphasecoordinate diagnostic')

        for phase in (-1.,.3,.9):
            controllers={(guard,normal):fake(phase,normal)for guard in (False,True)for normal in (False,True)}
            for i in range(60):
                rotation=rz(.1*i)@ry(phase+.001*i)@rx(.1)
                normal=np.array([np.sin(.15),0.,np.cos(.15)])
                target=np.array([.1*i,-.05*i,.02*i]) if i<15 else np.array([500.,-900.,800.])
                outputs={};diags={}
                for bits,controller in controllers.items():
                    controller.plant.data.time=i*.005;controller.plant.data.xmat[:]=rotation.reshape(1,9)
                    vref=np.r_[np.array([.06,.02,.003,0.,0.,0.]),target];before=vref[:6].copy()
                    diag=module.prepare(controller,vref,normal,reference_guard=bits[0],normal_phase=bits[1],touching=True)
                    np.testing.assert_array_equal(vref[:6],before)
                    np.testing.assert_array_equal(diag['actual_joint_rates_rad_s'],controller.plant.data.qvel[6:9])
                    assert diag['original_feedback_calls_by_this_helper']==0 and diag['current_gain_lookup_requested'] is True
                    outputs[bits]=vref.copy();diags[bits]=diag
                    if not bits[0]:assert not hasattr(controller,'_i51_joint_reference_guard')
                np.testing.assert_array_equal(outputs[(False,False)],outputs[(False,True)])
                np.testing.assert_array_equal(outputs[(True,False)],outputs[(True,True)])
                for guard in (False,True):
                    assert diags[(guard,False)]['phase_used_rad']==diags[(False,False)]['phase_world_z_rad']
                    assert diags[(guard,True)]['phase_used_rad']==diags[(False,False)]['phase_estimated_normal_rad']
                assert not np.shares_memory(controllers[(True,False)]._i51_joint_reference_guard.previous,
                    controllers[(True,True)]._i51_joint_reference_guard.previous)
        checks.append('All12phase×factorcompositions: rateguardindependentofphasebit andphaseformulaindependentofguardbit onidenticalinputs; baseline/phase-onlyinactivehistory')

        controller=fake(.9,True);vref=np.r_[np.zeros(6),[500.,-900.,800.]]
        diag=module.prepare(controller,vref,np.array([0.,0.,1.]),reference_guard=True,normal_phase=True,touching=False)
        expected=controller.ordinary['design']['K'].copy()
        feedback=dict(effective_phase_rad=diag['phase_used_rad'],reference_qvel=vref.copy(),K=expected)
        assert module.validate_feedback(diag,feedback,controller)['passed']
        bad=copy.deepcopy(feedback);bad['K'][0,0]+=1.
        require_raises(AssertionError,lambda:module.validate_feedback(diag,bad,controller))
        bad=copy.deepcopy(feedback);bad['effective_phase_rad']+=.1
        require_raises(AssertionError,lambda:module.validate_feedback(diag,bad,controller))
        assert diag['current_gain_lookup_requested'] is False
        checks.append('Native-returnphase/rate/OWNordinaryshaftrow validators rejectmismatches; heldgainphase distinction exposed')

        baseline=fake(.3,False);native=np.zeros(9)
        require_raises(ValueError,lambda:module.prepare(baseline,native,np.array([0.,0.,1.]),reference_guard=True,normal_phase=True))
        require_raises(ValueError,lambda:module.prepare(baseline,native,np.array([np.nan,0.,1.]),reference_guard=True,normal_phase=False))
        assert not hasattr(baseline,'_i51_joint_reference_guard') and not baseline.params.schedule_on_ground_normal
        clone=copy.deepcopy(controller);controller.plant.data.time=.005;vref=np.r_[np.zeros(6),np.zeros(3)]
        module.prepare(controller,vref,np.array([0.,0.,1.]),reference_guard=True,normal_phase=True)
        assert clone._i51_joint_reference_guard.time_s==0.
        module.reset(controller);assert controller._i51_joint_reference_guard.time_s is None
        checks.append('Flag/inputvalidationbeforemutation andownedguardclone/resetlifecycle')
        for row in records:assert sha(ROOT/row['origin'])==row['sha256']
        report=dict(passed=True,contract_group_count=len(checks),checks=checks,sources=records,exact_guard_identity=identity,
            scope='SevenI52composition/source/framecontracts; no repositorytests/controller/physicsornewperformanceclaims. Reuse ofI51guardisnotanewguarddesign.',source_hashes_unchanged_after=True)
        save(stage/'report.json',report);print(json.dumps(safe(report),indent=2))
    except Exception:
        import traceback
        save(stage/'failure.json',dict(completed_checks=checks,traceback=traceback.format_exc(),sources=records));raise
    finally:
        save(stage/'artifact_manifest.json',dict(files=[dict(path=str(path.relative_to(stage)),bytes=path.stat().st_size,sha256=sha(path))for path in sorted(stage.rglob('*'))if path.is_file()and path.name!='artifact_manifest.json']))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--stage',type=Path,required=True);args=parser.parse_args();main(args.stage.resolve())
