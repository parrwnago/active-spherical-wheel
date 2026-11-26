"""I51 saved-log and static kinematic balance audit; no controller/dynamics."""
from pathlib import Path
from datetime import datetime,timezone
from contextlib import ExitStack
from unittest.mock import patch
import xml.etree.ElementTree as ET
import json,hashlib,shutil,csv
import numpy as np
import mujoco
H=Path(__file__).resolve().parent;ROOT=H.parents[2];OLD=ROOT/'results/belt_capacity_sweep_20261126'
CASES=('mars_cap030','mars_cap060','mars_cap090');LABELS=('shell_A','shell_B','belt_A','belt_B')
WINDOWS={'control10_14':[10.,14.],'control20_24':[20.,24.],'approach34_38':[34.,38.],'late38_42':[38.,42.],'raised_comparison38_40':[38.,40.]}
TIMES=[12.,22.,36.,38.,38.64,39.,39.63,40.,40.625,41.,42.11]
GROUPS={'position':slice(0,3),'attitude':slice(3,6),'joint_position':slice(6,9),'linear_velocity':slice(9,12),'angular_velocity':slice(12,15),'joint_velocity':slice(15,18)}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,float)and not np.isfinite(v):return None
 return v
def save(p,v):p.write_text(json.dumps(safe(v),indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p)as z:return {k:z[k].copy()for k in z.files}
def rotate(q,v):
 q=q/np.linalg.norm(q);cross=np.cross(q[1:],v);return v+2*q[0]*cross+2*np.cross(q[1:],cross)
def qmul(q,r):return np.r_[q[0]*r[0]-q[1:]@r[1:],q[0]*r[1:]+r[0]*q[1:]+np.cross(q[1:],r[1:])]
def angle(a,b):return np.degrees(np.arccos(np.clip(a@b/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1)))
def main():
 out=H/'stage01';out.mkdir(exist_ok=False);src=out/'source';src.mkdir();records=[]
 sources=[Path(__file__),ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/experiments.py',ROOT/'src/spherical_wheel/recovery_control.py',ROOT/'src/spherical_wheel/sim.py',ROOT/'src/spherical_wheel/model.py',ROOT/'src/spherical_wheel/contact_sensing.py']
 for p in sources:
  target=src/p.name;shutil.copyfile(p,target);records.append({'origin':str(p.relative_to(ROOT)),'snapshot':str(target.relative_to(out)),'sha256':sha(p)})
 xmls={c:next((OLD/'campaign/_workspaces'/c).glob('worker_*/mirror/model.xml'))for c in CASES}
 inputs=[]
 for c in CASES:
  inputs.extend([OLD/'campaign/runs'/c/f for f in('precision.npz','rollout.npz','summary.json','provenance.json')]);inputs.append(xmls[c])
 for mesh in ET.parse(xmls[CASES[0]]).getroot().findall('asset/mesh'):inputs.append(xmls[CASES[0]].parent/mesh.attrib['file'])
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
 save(out/'registered_before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),sources=records,input_sha256=hashes,windows_s=WINDOWS,fixed_times_s=TIMES,scope='Savedarray arithmetic and private mj_kinematics,mj_comPos,mj_jac/mj_jacBodyCom only; no controller/observer/forward/inverse dynamics/integration/contactquery.',static_check='Atcurrentandrequestedattitude useactualjointconfiguration, samecenter, estimatedfilterednormal plane, spherepointcenter−.12normal, forceworld[0,0,W]. Sumknownmassgravity via COM Jacobians pluscontactforce. Neededmotorholdtau=−sum[6:9]; floatingbase residual diagnoseswhetherthis frozenpointcanbalanceatthatup. Compareholdingcommand and COMverticalalignment. This is a hypothetical staticplane algebra check, not new simulated equilibrium or actualfuturecontact.',metrics='ExactdesiredXY+velocity comparisons allrows; LSresidual atknown single support from analytic co-centeredaxes; no-slip velocity includesactualbodyangularmotion and previousheldbelt. Rawtruth last1ms endpoints; no physicalnormaltruth. Comparefixedwindowsdescriptively, notindependentstatisticaltrials.'))
 assert len({sha(x)for x in xmls.values()})==1
 model=mujoco.MjModel.from_xml_path(str(xmls[CASES[0]]));data=mujoco.MjData(model);fork=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'fork');R=.12;mass=model.body_subtreemass[fork];W=mass*9.81
 bodies={name:int(model.geom_bodyid[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name+'_smooth')])for name in LABELS}
 def forbidden(*a,**kw):raise AssertionError('Dynamics/integration/controller/contact forbidden')
 result={'scope':'Diagnostic staticforce algebra andsavedrows; no liveintervention.','cases':{},'known_source_settings':{},'limitations':['Staticplane estimate freezescurrentnormal andone equivalentpoint; multicontact/frictionalpatch/normalerrorcanviolateit.','LoggedDEMnormal is a terrainproxy, not physicalcollisionnormal.','No offsetfree/integralstate exists inselectedcontroller; this alone doesnotprovebiascause.']}
 csvrows=[]
 def static(q,n,body):
  data.qpos[:]=q;data.qvel[:]=0;mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
  gravity=np.zeros(model.nv);com=np.zeros(3);total=0
  for b in range(1,model.nbody):
   if model.body_mass[b]<=0:continue
   jp=np.zeros((3,model.nv));mujoco.mj_jacBodyCom(model,data,jp,None,b);gravity+=jp.T@np.array([0.,0.,-model.body_mass[b]*9.81]);com+=model.body_mass[b]*data.xipos[b];total+=model.body_mass[b]
  com/=total;point=data.xpos[fork]-R*n
  jac=np.zeros((3,model.nv));mujoco.mj_jac(model,data,jac,None,point,body);load=jac.T@np.array([0.,0.,W]);generalized=gravity+load
  return dict(COM_m=com,contact_m=point,COM_minus_support_xy_m=com[:2]-point[:2],floating_force_balance_generalized=generalized[:6],needed_hold_torque_Nm=-generalized[6:9])
 with ExitStack()as stack:
  for name in('mj_step','mj_step1','mj_step2','mj_forward','mj_inverse','mj_integratePos','mj_contactForce'):
   stack.enter_context(patch.object(mujoco,name,forbidden))
  for c in CASES:
   d=OLD/'campaign/runs'/c;p=load(d/'precision.npz');log=load(d/'rollout.npz');summary=json.loads((d/'summary.json').read_text());provenance=json.loads((d/'provenance.json').read_text());params=provenance['parameters'];t=p['time_s'];n=len(t)
   assert params==provenance['preflight']['parameters'];assert summary['parameters']==params
   result['known_source_settings'][c]={k:params[k]for k in('mode','rolling_allocation','rolling_damping_m_s','rolling_speed_limits','rolling_acceleration_limits','rolling_effort_m_s','rolling_change_m_s','rolling_reference_balance_guard','track_contact_point','shaft_error','schedule_on_ground_normal','support_feedforward','lean_gain','lqr_weights')}
   xy_difference=p['exact_reference_qpos'][:,:2]-p['exact_reference_xy'];v_difference=p['exact_reference_qvel'][:,:2]-p['exact_reference_velocity_m_s'];omega_ref=p['exact_feedback_reference_qvel'][:,3:6]
   assert np.max(abs(xy_difference))==0 and np.max(abs(v_difference))==0
   assert np.max(abs(omega_ref))==0
   # Native oldLS defaultdiagnostic has noaccepted/projection feedbacksubstitution.
   allocation_residual=np.full(n,np.nan);noslip=np.full(n,np.nan);cond=np.full(n,np.nan);single=np.zeros(n,bool)
   pointgap=np.linalg.norm(p['exact_estimated_contact_point_m']-p['truth_equivalent_point_m'],axis=1)
   ed=np.argmax(p['exact_estimated_support_loads_N'],axis=1);td=np.argmax(p['truth_patch_loads_N'],axis=1)
   for i in np.flatnonzero((t>=10.-1e-9)&(t<=42.2+1e-9)):
    load=p['exact_estimated_support_loads_N'][i];total=load.sum();j=ed[i]
    if total<.05*W or load[j]<.999999*total:continue
    single[i]=True;q=p['exact_qpos'][i];vel=p['exact_qvel'][i];axisY=rotate(q[3:7],np.array([0.,1.,0.]));axisX=rotate(q[3:7],np.array([np.cos(q[7]),0.,-np.sin(q[7])]))
    r=p['exact_estimated_contact_point_m'][i]-q[:3];J=np.zeros((3,5));J[:,0]=np.cross(axisY,r)
    if j<2:J[:,1+j]=np.cross(axisX,r)
    else:J[:,1+j]=axisY
    allocated=np.r_[p['exact_reference_qvel'][i,6:9],p['exact_rolling_feedforward'][i,3:5]]
    allocation_residual[i]=np.linalg.norm(J[:2]@allocated+p['exact_reference_velocity_m_s'][i])
    s=np.linalg.svd(J[:2]*np.array([5,5,5,.3,.3]),compute_uv=False);cond[i]=s[0]/max(s[-1],1e-30)
    material=vel[:3]+np.cross(rotate(q[3:7],vel[3:6]),r)+J[:,:3]@vel[6:9]
    if j>=2:material+=J[:,j+1]*p['exact_previous_applied'][i,j+1]
    normal=p['exact_estimated_normal'][i];material-=normal*(normal@material);noslip[i]=np.linalg.norm(material)
   windows={}
   for name,(lo,hi)in WINDOWS.items():
    ii=np.flatnonzero((t>=lo-1e-9)&(t<hi-1e-9));m=ii[np.isfinite(allocation_residual[ii])];vel=p['exact_qvel'][ii,:2];ref=p['exact_reference_velocity_m_s'][ii];truth=p['truth_patch_loads_N'][ii];estimated=p['exact_estimated_support_loads_N'][ii]
    windows[name]=dict(samples=len(ii),single_estimated_samples=len(m),allocated_planar_residual_max_m_s=None if not len(m)else np.max(allocation_residual[m]),condition_median=None if not len(m)else np.median(cond[m]),condition_max=None if not len(m)else cond[m].max(),body_velocity_error_rms_m_s=np.sqrt(np.mean(np.sum((vel-ref)**2,axis=1))),actual_speed_median_m_s=np.median(np.linalg.norm(vel,axis=1)),desired_speed_median_m_s=np.median(np.linalg.norm(ref,axis=1)),estimated_pointgap_median_m=np.nanmedian(pointgap[ii]),estimated_vs_truth_dominant_disagree_samples=np.sum(ed[ii]!=td[ii]),logged_true_slip_rms_m_s=np.sqrt(np.mean(log['slip_speed_m_s'][ii]**2)),inferred_contact_tangent_velocity_rms_m_s=None if not len(m)else np.sqrt(np.mean(noslip[m]**2)),requested_cap_samples_perdrive=np.sum(abs(p['exact_native_requested'][ii])>p['exact_input_bounds'][ii]+1e-12,axis=0),max_abs_raw_reference_angular_velocity=abs(omega_ref[ii]).max())
   snapshots=[]
   for at in TIMES:
    i=int(np.argmin(abs(t-at)));q=p['exact_qpos'][i];j=ed[i];normal=p['exact_estimated_normal'][i];raw=p['exact_reference_qpos'][i].copy();yaw=np.arctan2(2*(q[3]*q[6]+q[4]*q[5]),1-2*(q[5]**2+q[6]**2));target=q.copy();target[3:7]=qmul(raw[3:7],np.array([np.cos(yaw/2),0.,0.,np.sin(yaw/2)]));body=bodies[LABELS[j]]
    current=static(q,normal,body);targetstatic=static(target,normal,body);K=p['exact_feedback_gain'][i];e=p['exact_error'][i];effectiveposition=-K[:,:2].copy();effectiveposition[:,0]-=K[:,6]/R
    raw_pos=q[:2]-raw[:2];cosa,sina=np.cos(yaw),np.sin(yaw);heading=np.array([[cosa,sina],[-sina,cosa]])
    np.testing.assert_allclose(heading@raw_pos,e[:2],atol=1e-14,rtol=0)
    np.testing.assert_allclose(e[6],e[0]/R-e[4],atol=1e-14,rtol=0)
    expected_up=rotate(target[3:7],np.array([0.,0.,1.]));actualup=rotate(q[3:7],np.array([0.,0.,1.]))
    row=dict(time_s=t[i],estimated_patch=LABELS[j],truth_patch=LABELS[td[i]],actual_velocity_m_s=p['exact_qvel'][i,:3],reference_velocity_m_s=p['exact_reference_velocity_m_s'][i],timed_error_m=np.linalg.norm(raw_pos),heading_position_error_m=e[:2],relative_lean_deg=angle(actualup,expected_up),estimated_vs_DEM_normal_deg=angle(normal,log['ground_normal'][i]),current_condition=cond[i],allocation_residual_m_s=allocation_residual[i],inferred_contact_tangent_speed_m_s=noslip[i],logged_true_slip_m_s=log['slip_speed_m_s'][i],requested=p['exact_native_requested'][i],applied=p['exact_applied'][i],LQRgroups={name:-K[:,s]@e[s]for name,s in GROUPS.items()},support_feedforward=p['exact_support_command'][i,:3],effective_position_command_gain=effectiveposition,current_static=current,reference_attitude_static=targetstatic,reference_static_hold_minus_logged_support_Nm=targetstatic['needed_hold_torque_Nm']-p['exact_support_command'][i,:3])
    snapshots.append(row)
    flat={'case':c,'time_s':t[i],'timed_error_mm':row['timed_error_m']*1000,'reference_COM_support_gap_mm':np.linalg.norm(targetstatic['COM_minus_support_xy_m'])*1000,'current_COM_support_gap_mm':np.linalg.norm(current['COM_minus_support_xy_m'])*1000,'relative_lean_deg':row['relative_lean_deg'],'condition':row['current_condition'],'allocation_residual_m_s':row['allocation_residual_m_s'],'logged_true_slip_m_s':row['logged_true_slip_m_s']}
    for name,vec in row['LQRgroups'].items():flat.update({'LQR_'+name+'_'+str(k):value for k,value in enumerate(vec)})
    csvrows.append(flat)
   result['cases'][c]=dict(unchanged_targets=dict(max_abs_XY_difference_m=abs(xy_difference).max(),max_abs_velocity_difference_m_s=abs(v_difference).max(),max_reference_angular_rate_rad_s=abs(omega_ref).max()),windows=windows,snapshots=snapshots)
 save(out/'report.json',result)
 with(out/'snapshots.csv').open('w',newline='')as f:w=csv.DictWriter(f,fieldnames=csvrows[0].keys());w.writeheader();w.writerows(csvrows)
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 for r in records:assert sha(ROOT/r['origin'])==r['sha256']
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('I51 lateaudit complete')
if __name__=='__main__':main()
