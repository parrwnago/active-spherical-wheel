"""Fixed-state differential sanity screen against the retained I55 first hold."""
from pathlib import Path
from types import SimpleNamespace
from datetime import datetime, timezone
import hashlib,json,shutil
import mujoco
import numpy as np
from spherical_wheel.contact_sensing import SupportLocator,measure_records
from spherical_wheel.handoff_model import MeasuredContactVelocityModel
ROOT=Path.cwd();BASE=ROOT/'results/constrained_handoff_20261126';OUT=BASE/'model_sensitivity_stage02';OUT.mkdir(exist_ok=False)
OLD=ROOT/'results/causal_replay_20261126/campaign';NATIVE=OLD/'native_substeps';TREATED=OLD/'zero_free_shellA'
metadata=json.loads((NATIVE/'i55_provenance.json').read_text());model_path=Path(metadata['models']['mirror_xml']);sdf_path=Path(metadata['models']['physical_sdf'])
inputs=[Path(__file__),ROOT/'src/spherical_wheel/handoff_model.py',ROOT/'src/spherical_wheel/contact_sensing.py',NATIVE/'campaign/runs/m100_R/precision.npz',NATIVE/'substeps.npz',TREATED/'substeps.npz',NATIVE/'i55_provenance.json',model_path,sdf_path]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
registration={'utc':datetime.now(timezone.utc).isoformat(),'scope':'Coarse same-state differential screen; no Gazebo, integration or observer execution. I55 response was already known; this is not unseen-data model validation.','time_s':32.25,'horizon_s':.005,'delta_input':[0,.6,0,0,0],'expected_observed_world_wz_delta_rad_s':-.2708335994728328,'primary_configuration':'contact_strength10; tangent_confidence=1/(1+(estimated_slip/.02)^2), estimated_slip reconstructed only from measuredq/v, knownpreviousinput and suppliedobserverwrench/locatorrecords','primary_criterion':'Finite correct negative sign and absolute relative error <=0.5 versus held-input I55 pairedworldwz effect. Broad sanity gate, not accepted dynamics/balance proof. Confidence1 and0 reported only as diagnostic comparisons.','source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in inputs}}
(OUT/'declared_before_arithmetic.json').write_text(json.dumps(registration,indent=2)+'\n');shutil.copyfile(__file__,OUT/'check_model_sensitivity.py')
def load(p):
 with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
p=load(NATIVE/'campaign/runs/m100_R/precision.npz');native=load(NATIVE/'substeps.npz');treated=load(TREATED/'substeps.npz');i=int(np.argmin(abs(p['time_s']-32.25)));assert abs(p['time_s'][i]-32.25)<1e-9
model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model);data.qpos[:]=p['exact_qpos'][i];data.qvel[:]=p['exact_qvel'][i]
mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
fork=int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'fork'));radius=.12;weight=model.body_subtreemass[fork]*np.linalg.norm(model.opt.gravity)
for side,speed in zip(('A','B'),p['exact_previous_applied'][i,3:]):
 for name in (f'belt_{side}',f'belt_{side}_smooth'):
  geom=int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name))
  if geom>=0:model.geom_surfacevel[geom,:3]=[0.,speed,0.]
locator=SupportLocator(model,fork,radius,.7,.05*weight);w=p['exact_observer_wrench'][i];records=locator.records(data,w[:3],w[3:])
measurement=measure_records(SimpleNamespace(model=model,data=data,fork_id=fork),records)
slip=float(measurement['slip_speed_m_s']);confidence=1./(1.+(slip/.02)**2)
end=int(np.argmin(abs(native['time_s']-32.255)));assert abs(native['time_s'][end]-32.255)<1e-9
actual=float(treated['bridge_state'][end,14]-native['bridge_state'][end,14]);assert abs(actual-registration['expected_observed_world_wz_delta_rad_s'])<1e-12
delta=np.array(registration['delta_input']);helper=MeasuredContactVelocityModel(model,contact_strength=10.);results={}
for label,c in [('actual_confidence',confidence),('full_tangent_confidence',1.),('normal_only',0.)]:
 r=helper.evaluate(p['exact_qpos'][i],p['exact_qvel'][i],records,horizon_s=.005,tangent_confidence=c)
 predicted=float((r.body_angular_velocity_maps_world[0]@(r.response@delta))[2]);relative=abs(predicted-actual)/abs(actual)
 results[label]={'tangent_confidence':c,'predicted_world_wz_delta_rad_s':predicted,'observed_world_wz_delta_rad_s':actual,'absolute_relative_error':relative,'sanity_gate_passed':bool(np.isfinite(predicted) and predicted<0 and relative<=.5),'contact_rank':r.constraint_rank,'shell_axis_world':r.shell_axis_world.tolist(),'shell_axial_inertias_kg_m2':r.shell_axial_inertias_kg_m2.tolist()}
for n,h in registration['source_sha256'].items():assert sha(ROOT/n)==h
report={'measured_time_s':float(p['time_s'][i]),'estimated_slip_m_s':slip,'estimated_supports':[r['name'] for r in records],'estimated_loads_N':[r['normal_force'] for r in records],'results':results,'primary_gate_passed':results['actual_confidence']['sanity_gate_passed'],'limitations':['The known historical response is a singlefixedstate sanityscreen, not a validationcampaign.','Soft no-slip penalty is a local differential approximation; finitefriction/unilateral constraints are not guaranteed.','No offset/contactforecastaccuracy or closed-loopimprovementclaim.']}
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n');(OUT/'manifest.json').write_text(json.dumps({'files':[{'path':q.name,'bytes':q.stat().st_size,'sha256':sha(q)} for q in sorted(OUT.iterdir()) if q.is_file()]},indent=2)+'\n');print(json.dumps(report,indent=2))
