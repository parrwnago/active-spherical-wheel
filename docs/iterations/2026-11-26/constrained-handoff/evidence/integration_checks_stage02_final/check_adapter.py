"""Static measured-state adapter smoke check; no integration/controller update."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import asdict
import hashlib,importlib.util,json,sys
import numpy as np
import mujoco
from spherical_wheel.contact_sensing import SupportLocator,measure_records
from spherical_wheel.handoff_control import HandoffSettings
ROOT=Path.cwd();BASE=ROOT/'results/constrained_handoff_20261126';OUT=Path(__file__).resolve().parent
src=BASE/'campaign_tools/adapter.py';spec=importlib.util.spec_from_file_location('_i56_adapter_smoke',src);a=importlib.util.module_from_spec(spec);sys.modules[spec.name]=a;spec.loader.exec_module(a)
old=ROOT/'results/contact_scheduling_joint_tracking_20261126/campaign/runs/m100_R'
model_path=ROOT/'results/contact_scheduling_joint_tracking_20261126/campaign/_workspaces/m100_R/worker_397964/mirror/model.xml'
model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model)
with np.load(old/'precision.npz',allow_pickle=False) as p:
 i=int(np.argmin(abs(p['time_s']-32.25)));data.qpos[:]=p['exact_qpos'][i];data.qvel[:]=p['exact_qvel'][i];data.time=float(p['time_s'][i]);previous=p['exact_previous_applied'][i].copy();wrench=p['exact_observer_wrench'][i].copy();request=p['exact_native_requested'][i].copy();applied=p['exact_applied'][i].copy();weights=p['exact_sync_weights'][i].copy();vref=p['exact_reference_qvel'][i].copy();loads=p['exact_estimated_support_loads_N'][i].copy()
# Mirror the prior ideal belt input for measured material-velocity residuals.
for side in ['A','B']:
 for suffix in ['', '_smooth']:
  g=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'belt_'+side+suffix)
  if g>=0:model.geom_surfacevel[g,:3]=[0.,previous[3+int(side=='B')],0.]
mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
fork=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'fork');mass=float(model.body_subtreemass[fork]);plant=SimpleNamespace(model=model,data=data,bounds=np.array([1.5,.6,.6,.3,.3]),last_input=previous.copy(),dt=.005,fork_id=fork)
locator=SupportLocator(model,fork,.12,.7,.05*mass*9.81)
controller=SimpleNamespace(params=SimpleNamespace(mode='hardware'),locator=locator,plant=plant,observer=SimpleNamespace(wrench=lambda:(wrench[:3].copy(),wrench[3:].copy())),synchronizer=SimpleNamespace(shell_blend_limit=.35),_sync={'targets':[]})
measurement=measure_records(plant,locator.records(data,*controller.observer.wrench()))
info=dict(measurement=measurement,sync_weights=weights,reference_qvel=vref,estimated_support_loads_N=loads)
settings={'allocator':asdict(HandoffSettings()),'contact_strength':10.,'slip_confidence_scale_m_s':.02}
before=[x.copy() for x in [data.qpos,data.qvel,plant.last_input,model.geom_surfacevel]]
instance=a.create(controller,settings);final,actual,diag=a.apply(instance,controller,request,applied,info)
for x,y in zip([data.qpos,data.qvel,plant.last_input,model.geom_surfacevel],before):np.testing.assert_array_equal(x,y)
np.testing.assert_array_equal(final,actual);assert np.all(abs(actual)<=plant.bounds);assert np.all(abs(actual-previous)<=np.array(HandoffSettings().slew_limits)*.005+1e-14)
assert np.max(abs(diag['secondary_balance_delta']))<=1e-8
safe=lambda x: x.tolist() if isinstance(x,np.ndarray) else (x.item() if isinstance(x,np.generic) else x)
report={'scope':__doc__,'passed':True,'input_case':'oldI52m100_R32.250s, staticdiagnosticnotcandidateclosedloop','native_requested':request.tolist(),'native_applied':applied.tolist(),'governed':actual.tolist(),'previous_actual':previous.tolist(),'balance_delta':diag['first_stage_balance_delta'].tolist(),'secondary_delta':diag['secondary_balance_delta'].tolist(),'current_yaw':diag['current_world_yaw_rate_rad_s'],'proxy_yaw':diag['proxy_world_yaw_rate_rad_s'],'tangent_confidence':diag['tangent_confidence_proxy'],'runtime_model_sha256':hashlib.sha256((ROOT/'src/spherical_wheel/handoff_model.py').read_bytes()).hexdigest(),'runtime_allocator_sha256':hashlib.sha256((ROOT/'src/spherical_wheel/handoff_control.py').read_bytes()).hexdigest(),'adapter_sha256':hashlib.sha256(src.read_bytes()).hexdigest()}
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
