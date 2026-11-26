"""I51 fixed-snapshot phase/frame/velocity audit; pure algebra, no controller."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil
import numpy as np
H=Path(__file__).resolve().parent;ROOT=H.parents[2];OLD=ROOT/'results/belt_capacity_sweep_20261126';CASES=('mars_cap030','mars_cap060','mars_cap090');TIMES=[12.,22.,36.,38.64,39.63,40.625,41.,42.11]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 return v
def save(p,v):p.write_text(json.dumps(safe(v),indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p)as z:return {k:z[k].copy()for k in z.files}
def rotate(q,v):
 q=q/np.linalg.norm(q);c=np.cross(q[1:],v);return v+2*q[0]*c+2*np.cross(q[1:],c)
def main():
 out=H/'stage03_phase_frames';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'audit_phase_frames.py')
 cache=ROOT/'generated/offroad/flat/control_schedule.npz';inputs=[cache,ROOT/'src/spherical_wheel/recovery_control.py',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/sim.py']
 for c in CASES:inputs.extend([OLD/'campaign/runs'/c/f for f in('precision.npz','rollout.npz')])
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
 save(out/'registered_before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=hashes,fixed_times_s=TIMES,scope='Algebraic snapshots only: nativeworldZ vsfilterednormal phase, purebankgaininterpolation copied mathematically, originalshaftrow retained, shadow -dK e only. No controller/model/observer calls or integration. Decomposev_actual−v_des into yawbodyZ,transversetilt andjointvelocitymismatch usingcurrentestimatedpoint; remainder reflectsestimatedpointcontactresidual. Componentkinematics isnotcausal dynamicintervention.'))
 bank=load(cache);reg=bank['regime_code'];side=bank['side_code'];phases=bank['phase_rad'];Ks=bank['K']
 def gain(phase,loads):
  K=np.zeros((5,18));total=loads.sum()
  for j,force in enumerate(loads):
   if force<.001:continue
   indices=np.flatnonzero((reg==(j>=2))&(side==(j%2==1)));indices=indices[np.argsort(phases[indices])];x=phases[indices];mat=Ks[indices]
   local=min((phase,phase-2*np.pi,phase+2*np.pi),key=lambda p:max(x[0]-p,0.,p-x[-1]));hi=int(np.searchsorted(x,local));lo=max(0,hi-1);hi=min(len(x)-1,hi);weight=np.clip((local-x[lo])/max(x[hi]-x[lo],1e-9),0,1)
   K+=force/total*((1-weight)*mat[lo]+weight*mat[hi])
  return K
 report={'cases':{},'limitations':['Pure same-errorgainphasechange isnotnewphysicaltrajectory orrecommendedswitch; priorglobalnormal-schedulingtrialsregressed.','Freeheadingmeanspositionyawtargetfollowstheactualyaw; angular-rate targetisstillzero, soyawrateisweaklydampedbyLQR notwhollyuncontrolled.','Kinematicdecompositiondoesnotidentifywhichdynamicfeedback/historygeneratedrates.']}
 for c in CASES:
  p=load(OLD/'campaign/runs'/c/'precision.npz');l=load(OLD/'campaign/runs'/c/'rollout.npz');t=p['time_s'];rows=[]
  for at in TIMES:
   i=int(np.argmin(abs(t-at)));q=p['exact_qpos'][i];vel=p['exact_qvel'][i];n=p['exact_estimated_normal'][i];K=p['exact_feedback_gain'][i];e=p['exact_error'][i];loads=p['exact_estimated_support_loads_N'][i];j=int(np.argmax(loads));assert j<2
   R=np.column_stack([rotate(q[3:7],basis)for basis in np.eye(3)]);cy,sy=np.cos(q[7]),np.sin(q[7]);Ry=np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]]);C=R@Ry
   s=C.T@np.array([0.,0.,1.]);native=np.arctan2(-s[0],s[2]);s=C.T@n;ground=np.arctan2(-s[0],s[2]);raw=gain(native,loads);raw[0]=K[0];np.testing.assert_allclose(raw,K,atol=1e-11,rtol=1e-11)
   shadow=gain(ground,loads);shadow[0]=K[0]
   r=p['exact_estimated_contact_point_m'][i]-q[:3];J=np.zeros((3,3));J[:,0]=np.cross(R[:,1],r);J[:,j+1]=np.cross(C[:,0],r)
   yawvelocity=-np.cross(R[:,2]*vel[5],r);tiltvelocity=-np.cross(R[:,:2]@vel[3:5],r);joint_mismatch=-J@(vel[6:9]-p['exact_reference_qvel'][i,6:9]);bodydifference=vel[:3]-p['exact_reference_qvel'][i,:3]
   yaw=np.arctan2(2*(q[3]*q[6]+q[4]*q[5]),1-2*(q[5]**2+q[6]**2));worldtoheading=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
   np.testing.assert_allclose(e[9:11],worldtoheading@(vel[:2]-p['exact_reference_qvel'][i,:2]),atol=1e-14,rtol=0)
   np.testing.assert_allclose(e[12:15],vel[3:6],atol=0,rtol=0)
   rows.append(dict(time_s=t[i],yaw_rad=yaw,body_angular_velocity_rad_s=vel[3:6],native_worldZ_phase_rad=native,normal_relative_phase_rad=ground,phase_change_deg=np.degrees(np.arctan2(np.sin(ground-native),np.cos(ground-native))),native_gain_reproduced_max_abs=np.max(abs(raw-K)),alternative_gain_relative_change=np.linalg.norm(shadow-K)/np.linalg.norm(K),same_error_phase_only_command_delta=-(shadow-K)@e,native_requested=p['exact_native_requested'][i],body_velocity_error_m_s=bodydifference,yaw_contribution_m_s=yawvelocity,tilt_contribution_m_s=tiltvelocity,joint_velocity_mismatch_contribution_m_s=joint_mismatch,remaining_velocity_residual_m_s=bodydifference-yawvelocity-tiltvelocity-joint_mismatch,heading_velocity_error_reproduced=True,body_angular_velocity_error_reproduced=True))
  report['cases'][c]=rows
 save(out/'report.json',report)
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('Phase/frame audit complete')
if __name__=='__main__':main()
