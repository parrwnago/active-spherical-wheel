"""Independent saved-data checks; direct world angular-velocity chains, no reducer imports."""
from pathlib import Path
from contextlib import ExitStack
from unittest.mock import patch
import json,hashlib,xml.etree.ElementTree as ET
import numpy as np
import mujoco
ROOT=Path.cwd();FAMILY=ROOT/'results/causal_replay_20261126';HERE=Path(__file__).resolve().parent
CASES=('native_substeps','zero_free_shellA','hold_attitude');TIMES=(32.251,32.255);INTERVALS=((32.250,32.275),(32.250,32.305));NAMES=('fork','carrier','shell_A','shell_B')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 return v
def save(p,v):p.write_text(json.dumps(safe(v),indent=2)+'\n')
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def rot(q):
 w,x,y,z=q/np.linalg.norm(q)
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
inputs=[FAMILY/'protocol.json',FAMILY/'analysis/stage01/report.json'];provs={}
for case in CASES:
 f=FAMILY/'campaign'/case;provs[case]=json.loads((f/'i55_provenance.json').read_text());inputs += [f/'substeps.npz',f/'i55_sidecar.npz',f/'i55_provenance.json']
 model=Path(provs[case]['models']['mirror_xml']);inputs.append(model)
 for n in ET.parse(model).getroot().findall('./asset/mesh'):inputs.append((model.parent/n.attrib['file']).resolve())
inputs=list(dict.fromkeys(inputs));hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
save(HERE/'registered_before_arithmetic.json',dict(scope=__doc__,case_ids=CASES,first_hold_endpoints_s=TIMES,closure_intervals_s=INTERVALS,input_sha256=hashes,source_sha256=sha(Path(__file__)),method='Body origins allcoincident withwheelcenter, verifiedzerochildoffsets/jointpositions; absoluteomega directfromrawWORLDforkomega+relativejointaxes, bodyCOMvelocity=v_origin+omega crossbodyoffset. Private mj_kinematics only; noJacobian/reducer/sourcehelpercalls.'))
report=json.loads((FAMILY/'analysis/stage01/report.json').read_text());traces={c:load(FAMILY/'campaign'/c/'substeps.npz')for c in CASES};side={c:load(FAMILY/'campaign'/c/'i55_sidecar.npz')for c in CASES}
checks={};results={}
def check(name,a,b,atol=2e-12):
 a,b=np.asarray(a),np.asarray(b);ok=a.shape==b.shape and np.allclose(a,b,rtol=0,atol=atol,equal_nan=True);checks[name]=bool(ok)
 if not ok:raise AssertionError(name+': '+str(a)+' != '+str(b))
def idx(times,at):
 i=int(np.argmin(abs(times-at)));assert abs(times[i]-at)<1e-8;return i
def forbidden(*a,**k):raise AssertionError('No integration, forward, dynamics, contact or controller APIs allowed')
def body_momentum(model,states):
 data=mujoco.MjData(model);ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n)for n in NAMES];mass=model.body_mass[ids];assert np.max(abs(model.body_pos[ids[1:]]))==0and np.max(abs(model.jnt_pos))==0
 H=[];C=[];VC=[];axes=[]
 for state in states:
  q=np.r_[state[2:5],state[5:9]/np.linalg.norm(state[5:9]),state[15:18]];data.qpos[:]=q;mujoco.mj_kinematics(model,data)
  rf=rot(q[3:7]);axis=data.xmat[ids[1]].reshape(3,3)[:,0].copy();omega=np.zeros((4,3));omega[0]=state[12:15];omega[1]=omega[0]+state[18]*rf[:,1];omega[2]=omega[1]+state[19]*axis;omega[3]=omega[1]+state[20]*axis
  x=data.xipos[ids].copy();velocity=state[9:12]+np.cross(omega,x-q[:3]);center=np.sum(mass[:,None]*x,axis=0)/mass.sum();vcenter=np.sum(mass[:,None]*velocity,axis=0)/mass.sum();h=[]
  for j,b in enumerate(ids):
   r=data.ximat[b].reshape(3,3);I=r@np.diag(model.body_inertia[b])@r.T;h.append(I@omega[j]+mass[j]*np.cross(x[j]-center,velocity[j]-vcenter))
  H.append(h);C.append(center);VC.append(vcenter);axes.append(rf[:,1])
 return np.array(H),np.array(C),np.array(VC),np.array(axes)
model_results={}
with ExitStack()as stack:
 for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_fwdPosition','mj_fwdVelocity','mj_fwdAcceleration','mj_integratePos','mj_contactForce'):
  stack.enter_context(patch.object(mujoco,name,forbidden))
 for case in CASES:
  z=traces[case];model=mujoco.MjModel.from_xml_path(provs[case]['models']['mirror_xml']);hb,cb,vb,ab=body_momentum(model,z['bridge_state_before']);ha,ca,va,aa=body_momentum(model,z['bridge_state']);model_results[case]=(hb,ha,cb,ca,vb,va,ab,aa,model.body_mass.sum())
  for at in TIMES:
   i=idx(z['time_s'],at);expected=report['cases'][case]['substep_endpoints'][str(at)]
   check(case+'worldomega_'+str(at),z['bridge_state'][i,12:15],expected['platform_world_omega_rad_s'])
   check(case+'Hbody_'+str(at),ha[i],expected['H_by_body_world_kg_m2_s'])
   check(case+'Htotal_'+str(at),ha[i].sum(axis=0),expected['H_total_world_kg_m2_s'])
   check(case+'actualheld_'+str(at),z['held_input'][i],expected['actual_held_input'],atol=0)
native=traces['native_substeps'];zero=traces['zero_free_shellA'];mask=(native['time_s']>32.25+1e-9)&(native['time_s']<=32.255+1e-9);check('first_hold_5_substeps',int(mask.sum()),5,atol=0)
check('same_before_treatment_state',native['bridge_state_before'][np.flatnonzero(mask)[0]],zero['bridge_state_before'][np.flatnonzero(mask)[0]],atol=0)
check('first_hold_other_inputs_exact',zero['held_input'][mask][:,[0,2,3,4]],native['held_input'][mask][:,[0,2,3,4]],atol=0)
check('first_hold_disturbance_exact',zero['disturbance_world'][mask],native['disturbance_world'][mask],atol=0);check('first_hold_friction_exact',zero['friction'][mask],native['friction'][mask],atol=0)
for at in TIMES:
 i=idx(native['time_s'],at);j=idx(zero['time_s'],at);expected=report['paired_substep_outcomes']['zero_free_shellA'][str(at)];dh=model_results['zero_free_shellA'][1][j]-model_results['native_substeps'][1][i]
 check('paired_wz_'+str(at),zero['bridge_state'][j,14]-native['bridge_state'][i,14],expected['world_z_rate_delta_rad_s'])
 check('paired_Hbodyz_'+str(at),dh[:,2],expected['H_by_body_z_delta_kg_m2_s']);check('paired_totalHz_'+str(at),dh[:,2].sum(),expected['H_total_z_delta_kg_m2_s'])
 check('paired_held_delta_'+str(at),zero['held_input'][j]-native['held_input'][i],expected['actual_held_input_delta'],atol=0)
 results['zero_A_'+str(at)]=dict(world_wz_delta=zero['bridge_state'][j,14]-native['bridge_state'][i,14],delta_H_body_z=dh[:,2],delta_H_total_z=dh[:,2].sum())
hb,ha,cb,ca,vb,va,ab,aa,mass=model_results['native_substeps'];centers=.5*(cb+ca);rows=native['contact_rows'];offset=native['contact_offsets'];dt=native['bridge_state'][:,0]-native['bridge_state_before'][:,0]
for a,b in INTERVALS:
 ids=np.flatnonzero((native['time_s']>a+1e-9)&(native['time_s']<=b+1e-9));impulse=0.;linear=np.zeros(3)
 for i in ids:
  c=rows[offset[i]:offset[i+1]];impulse+=np.cross(c[:,1:4]-centers[i],c[:,7:10])[:,2].sum()*dt[i];linear+=(c[:,7:10].sum(axis=0)+mass*np.array([0.,0.,-9.81]))*dt[i]
 delta=ha[ids[-1]].sum(axis=0)[2]-hb[ids[0]].sum(axis=0)[2];residual=impulse-delta;tol=max(1e-4,.1*abs(delta));expected=next(row for row in report['cases']['native_substeps']['momentum_intervals']if row['start_s']==a and row['end_s']==b);closed=expected['closure_by_center']['midpoint']
 check('native_deltaHz_'+str(b),delta,expected['delta_H_total_z_kg_m2_s']);check('native_contact_impulse_'+str(b),impulse,closed['contact_yaw_impulse_Nm_s']);check('native_closure_residual_'+str(b),residual,closed['contact_minus_delta_Hz_Nm_s']);check('native_closure_pass_'+str(b),abs(residual)<=tol,closed['passes_desired_closure'],atol=0)
 dp=mass*(va[ids[-1]]-vb[ids[0]]);check('native_linear_residual_'+str(b),linear-dp,expected['linear_momentum_convention_QA']['force_minus_state_residual_N_s'])
 shaft=np.sum(native['held_input'][ids,0]*.5*(ab[ids,2]+aa[ids,2])*dt[ids]);check('shaft_motor_fork_'+str(b),-shaft,expected['known_shaft_motor_couple_impulse_by_body_Nm_s']['fork']);check('shaft_motor_carrier_'+str(b),shaft,expected['known_shaft_motor_couple_impulse_by_body_Nm_s']['carrier'])
 results['native_interval_'+str(b)]=dict(delta_Hz=delta,contact_impulse_midpoint=impulse,residual=residual,registered_tolerance=tol,registered_closure_pass=abs(residual)<=tol,linear_impulse_residual_N_s=linear-dp)
for case in ['native_substeps','hold_attitude']:
 p=FAMILY/'campaign'/case/'campaign/runs/m100_R/precision.npz';data=load(p);s=side[case];i=idx(data['time_s'],31.745);j=idx(side['native_substeps']['time_s'],31.745);actual=rot(data['exact_qpos'][i,3:7])[:,2];target=rot(side['native_substeps']['native_qref_quat'][j])[:,2];angle=np.degrees(np.arccos(np.clip(actual@target,-1,1)))
 check('fixed_untreated_up_error_'+case,angle,report['cases'][case]['endpoints']['31.745']['relative_to_untreated_same_clock_target_deg'],atol=1e-10)
results['all_original_sources_and_inputs_unchanged']=all(sha(ROOT/p)==h for p,h in hashes.items());assert results['all_original_sources_and_inputs_unchanged']
answer=dict(passed=all(checks.values()),assertions=len(checks),scope=__doc__,independence='No canonical reducer imports/Jacobianvelocity reuse; directworldframejointchain andcoincidentoriginCOMkinematics. No integration/forward/solver/controllercalls.',checks=checks,results=results,canonical_report_sha256=hashes[str((FAMILY/'analysis/stage01/report.json').relative_to(ROOT))])
save(HERE/'report.json',answer);save(HERE/'artifact_manifest.json',dict(self_hash_excluded=True,files=[dict(path=str(p.relative_to(HERE)),sha256=sha(p),bytes=p.stat().st_size)for p in sorted(HERE.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
print(json.dumps(safe(dict(passed=answer['passed'],assertions=len(checks),report_sha256=sha(HERE/'report.json'),results=results)),indent=2))
