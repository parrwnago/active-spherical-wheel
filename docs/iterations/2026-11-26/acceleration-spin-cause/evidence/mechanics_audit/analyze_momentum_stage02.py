"""Saved-state rigid-body momentum audit; only static kinematics/Jacobians."""
from pathlib import Path
from contextlib import ExitStack
from unittest.mock import patch
import csv,hashlib,json,xml.etree.ElementTree as ET
import numpy as np
import mujoco
ROOT=Path.cwd();HERE=Path(__file__).resolve().parent/'stage02_corrected';HERE.mkdir(exist_ok=False)
CASE=ROOT/'results/contact_scheduling_joint_tracking_20261126/campaign/runs/m100_R'
WORK=ROOT/'results/contact_scheduling_joint_tracking_20261126/campaign/_workspaces/m100_R/worker_397964'
MODEL=WORK/'mirror/model.xml';SDF=WORK/'gazebo/world/models/active_spherical_wheel/model.sdf'
NAMES=('fork','carrier','shell_A','shell_B')
TIMES=(31.765,31.770,32.245,32.250,32.275,32.305)
INTERVALS=((32.245,32.275),(32.245,32.305),(32.250,32.275))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 return v
def save(p,v):p.write_text(json.dumps(safe(v),indent=2,allow_nan=False)+'\n')
def rot(q):
 w,x,y,z=q/np.linalg.norm(q)
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def forbidden(*a,**k):raise AssertionError('No integration/forward/dynamics/contact query allowed')
source_paths=['src/spherical_wheel/model.py','src/spherical_wheel/sim.py','src/spherical_wheel/contact_sensing.py','src/spherical_wheel/offroad_control.py','src/spherical_wheel/offroad_experiments.py','src/spherical_wheel/gazebo/plant.py','src/spherical_wheel/gazebo/model_sdf.py','gazebo/plugin/SphericalWheelBridge.cc','YAW_BEHAVIOR.md','config/model.yaml','results/contact_scheduling_joint_tracking_20261126/run_factorial_case.py']
inputs=[CASE/'precision.npz',CASE/'rollout.npz',CASE/'provenance.json',MODEL,SDF]
for node in ET.parse(MODEL).getroot().findall('./asset/mesh'):
 inputs.append((MODEL.parent/node.attrib['file']).resolve())
paths=list(dict.fromkeys([Path(__file__),*[ROOT/p for p in source_paths],*inputs]))
registration=dict(question='Does the observed m100_R spin burst exchange shell and platform angular momentum, and what net external yaw impulse is consistent with saved rigid-body state?',scope=__doc__,case='m100_R',snapshots_s=TIMES,intervals_s=INTERVALS,context_window_s=[31.5,32.5],allowed_mujoco=['MjModel.from_xml_path','MjData','mj_kinematics','mj_comPos','mj_jacBodyCom'],forbidden='No forward/dynamics/integration/observer/controller/preview/contact-force API or new simulator execution.',contact_timing='rollout contact wrench is float32 last1ms endpoint solve per saved5mscontrol row, not preceding5msmean or impulse',comparison='Net ΔH_COM,z from exact saved q/v under checked mirror/SDF masses/inertias; perbody changes are not uniquely motor-caused.',sources_and_inputs={str(p.relative_to(ROOT)):sha(p)for p in paths})
if(HERE/'registration.json').exists():raise FileExistsError('Fresh audit only')
snap=HERE/'source_snapshot'
for p in [Path(__file__),*[ROOT/p for p in source_paths],MODEL,SDF]:
 target=snap/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.read_bytes())
save(HERE/'registration.json',registration)
with np.load(CASE/'precision.npz',allow_pickle=False)as z: raw={k:z[k].copy()for k in z.files}
with np.load(CASE/'rollout.npz',allow_pickle=False)as z: log={k:z[k].copy()for k in z.files}
model=mujoco.MjModel.from_xml_path(str(MODEL));data=mujoco.MjData(model)
ids={name:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)for name in NAMES}
assert model.nq==10and model.nv==9and all(b>0for b in ids.values())
links={node.attrib['name']:node for node in ET.parse(SDF).getroot().findall('./model/link')}
inertia_checks={}
for name,b in ids.items():
 node=links[name].find('inertial');mass=float(node.findtext('mass'));pose=np.fromstring(node.findtext('pose'),sep=' ');I=node.find('inertia')
 sdfI=np.array([[float(I.findtext('ixx')),float(I.findtext('ixy')),float(I.findtext('ixz'))],[float(I.findtext('ixy')),float(I.findtext('iyy')),float(I.findtext('iyz'))],[float(I.findtext('ixz')),float(I.findtext('iyz')),float(I.findtext('izz'))]])
 assert np.array_equal(pose[3:],np.zeros(3))
 bodyI=rot(model.body_iquat[b])@np.diag(model.body_inertia[b])@rot(model.body_iquat[b]).T
 gaps=dict(mass_kg=float(abs(mass-model.body_mass[b])),com_m=float(np.max(abs(pose[:3]-model.body_ipos[b]))),inertia_kg_m2=float(np.max(abs(sdfI-bodyI))))
 assert max(gaps.values())<1e-10
 inertia_checks[name]=dict(mass_kg=mass,body_frame_COM_m=pose[:3],body_frame_inertia_kg_m2=sdfI,max_differences=gaps)
masses=model.body_mass[list(ids.values())];totalmass=masses.sum();t=raw['time_s'];indices=np.flatnonzero((t>=31.5-1e-10)&(t<=32.5+1e-10));rows=[];max_omega_gap=0.
with ExitStack()as stack:
 for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_fwdPosition','mj_fwdVelocity','mj_fwdAcceleration','mj_integratePos','mj_contactForce'):
  stack.enter_context(patch.object(mujoco,name,forbidden))
 for i in indices:
  q=raw['exact_qpos'][i];v=raw['exact_qvel'][i];data.qpos[:]=q;data.qvel[:]=v
  mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
  positions={name:data.xipos[b].copy()for name,b in ids.items()};velocity={};omega={};spin={}
  for name,b in ids.items():
   jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBodyCom(model,data,jp,jr,b);velocity[name]=jp@v;omega[name]=jr@v;R=data.ximat[b].reshape(3,3);spin[name]=(R@np.diag(model.body_inertia[b])@R.T)@omega[name]
  com=sum((model.body_mass[b]*positions[name]for name,b in ids.items()),np.zeros(3))/totalmass
  vc=sum((model.body_mass[b]*velocity[name]for name,b in ids.items()),np.zeros(3))/totalmass
  orbital={name:model.body_mass[b]*np.cross(positions[name]-com,velocity[name]-vc)for name,b in ids.items()};H={name:spin[name]+orbital[name]for name in NAMES};total=sum(H.values(),np.zeros(3))
  Rfork=rot(q[3:7]);axis=data.xmat[ids['carrier']].reshape(3,3)[:,0].copy();shaftaxis=Rfork[:,1]
  direct={'fork':Rfork@v[3:6]};direct['carrier']=direct['fork']+v[6]*shaftaxis;direct['shell_A']=direct['carrier']+v[7]*axis;direct['shell_B']=direct['carrier']+v[8]*axis
  gap=max(np.max(abs(omega[name]-direct[name]))for name in NAMES);max_omega_gap=max(max_omega_gap,float(gap));assert gap<1e-10
  contactF=log['contact_force_N'][i].astype(float);contactM=log['contact_moment_Nm'][i].astype(float);aboutCOM=contactM+np.cross(q[:3]-com,contactF)
  applied=raw['exact_applied'][i];previous=raw['exact_previous_applied'][i]
  row=dict(index=int(i),time_s=float(t[i]),qpos=q,qvel=v,omega_world_rad_s=omega,spin_H_world_kg_m2_s=spin,orbital_H_about_systemCOM_world_kg_m2_s=orbital,H_about_systemCOM_world_kg_m2_s=H,total_H_about_systemCOM_world_kg_m2_s=total,systemCOM_world_m=com,systemCOM_velocity_world_m_s=vc,shell_axis_world=axis,shaft_axis_world=shaftaxis,applied_command=applied,previous_applied_command=previous,actuator_shell_pair_reaction_on_carrier_z_Nm=float(-(applied[1]+applied[2])*axis[2]),endpoint_contact_force_world_N=contactF,endpoint_contact_moment_about_wheel_world_Nm=contactM,endpoint_contact_moment_about_systemCOM_world_Nm=aboutCOM,truth_patch_loads_N=raw['truth_patch_loads_N'][i],estimated_patch_loads_N=raw['exact_estimated_support_loads_N'][i],heading_command=raw['exact_heading_command'][i],phase_rad=float(raw['exact_effective_phase_rad'][i]))
  rows.append(row)
byindex={r['index']:r for r in rows}
def nearest(time):
 i=int(np.argmin(abs(t-time)));assert abs(t[i]-time)<1e-8;return byindex[i]
interval_rows=[]
for a,b in INTERVALS:
 first,last=nearest(a),nearest(b);window=[r for r in rows if first['index']<=r['index']<=last['index']]
 delta={name:last['H_about_systemCOM_world_kg_m2_s'][name][2]-first['H_about_systemCOM_world_kg_m2_s'][name][2]for name in NAMES};dtotal=last['total_H_about_systemCOM_world_kg_m2_s'][2]-first['total_H_about_systemCOM_world_kg_m2_s'][2]
 rotor=delta['shell_A']+delta['shell_B'];base=delta['fork']+delta['carrier'];endpoint_approx=float(np.trapz([r['endpoint_contact_moment_about_systemCOM_world_Nm'][2]for r in window],[r['time_s']for r in window]))
 interval_rows.append(dict(start_s=a,end_s=b,delta_H_total_systemCOM_z_kg_m2_s=float(dtotal),delta_H_by_body_systemCOM_z_kg_m2_s=delta,delta_H_fork_carrier_z_kg_m2_s=float(base),delta_H_shells_z_kg_m2_s=float(rotor),abs_net_to_shell_exchange_ratio=float(abs(dtotal)/abs(rotor))if rotor else None,endpoint_only_trapezoid_contact_yaw_impulse_Nm_s=endpoint_approx,endpoint_quadrature_minus_delta_H_Nm_s=endpoint_approx-dtotal,interpretation='ΔHtotal is state-derived netexternalyaw angularimpulse under matched modeled mass/inertia assumptions; endpoint contact quadrature is only sparse descriptive approximation and doesnotclose impulsebudget. Bodymomentum changes include motor,bearing,contact and gyroscopic redistribution.'))
assert all(sha(ROOT/p)==h for p,h in registration['sources_and_inputs'].items())
report=dict(case='m100_R',input_model=str(MODEL.relative_to(ROOT)),physical_model=str(SDF.relative_to(ROOT)),body_mass_inertia_checks=inertia_checks,total_mass_kg=float(totalmass),max_independent_chain_omega_gap_rad_s=max_omega_gap,platform_model_scope='fork body combines platform and rigid fork (1.2kg), carrier includes fixed polar modules (0.36kg); shell_A/B0.15kg each. No physical rotor/gear inertia separate from configured shells.',snapshots=[nearest(x)for x in TIMES],intervals=interval_rows,sources_and_inputs_unchanged=True,no_physics_or_forward=True,heading_command_max_abs_in_context=float(np.max(np.abs(raw['exact_heading_command'][indices]))),logged_push_max_abs_N=float(np.max(np.abs(log['push_N'][indices]))),limitations=['Saved data at5ms, endpoint collisionforce from last1ms; no fullsubstep forces/impulses.','Mirror/SDF tensoridentity verified to1e-10; actualdynamicengine internal inertia adaptation/contact solverimpulses not independentlyexported.','Worldω_z is physicalvertical angularvelocity; generally differsfromEulerheading derivative when tilted.','LQR stillusesbodyangularrates; zero separateheadinghelper doesnotmean zero yaw-sensitivefeedback.','No spin experiment/new replay/controllercalls.'])
save(HERE/'report.json',report);save(HERE/'context_rows.json',rows)
with(HERE/'momentum_timeline.csv').open('w')as stream:
 names=['time_s','fork_omega_z','carrier_omega_z','shell_A_omega_z','shell_B_omega_z','fork_Hz','carrier_Hz','shell_A_Hz','shell_B_Hz','total_Hz','shell_axis_z','shell_pair_reaction_z_Nm','endpoint_contact_COM_Mz_Nm'];writer=csv.writer(stream);writer.writerow(names)
 for r in rows:writer.writerow([r['time_s'],*[r['omega_world_rad_s'][n][2]for n in NAMES],*[r['H_about_systemCOM_world_kg_m2_s'][n][2]for n in NAMES],r['total_H_about_systemCOM_world_kg_m2_s'][2],r['shell_axis_world'][2],r['actuator_shell_pair_reaction_on_carrier_z_Nm'],r['endpoint_contact_moment_about_systemCOM_world_Nm'][2]])
print(json.dumps(safe(dict(total_mass_kg=totalmass,intervals=interval_rows,snapshots=[dict(time_s=nearest(x)['time_s'],omega_z=nearest(x)['omega_world_rad_s']['fork'][2],axis_z=nearest(x)['shell_axis_world'][2],applied=nearest(x)['applied_command'],H_z={n:nearest(x)['H_about_systemCOM_world_kg_m2_s'][n][2]for n in NAMES},total_Hz=nearest(x)['total_H_about_systemCOM_world_kg_m2_s'][2])for x in TIMES])),indent=2))
