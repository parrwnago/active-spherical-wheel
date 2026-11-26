"""Fixed I54 entering-lean followup, saved arrays and gain-bank indexing only."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil,csv
import numpy as np

H=Path(__file__).resolve().parent;ROOT=H.parents[2];OLD=ROOT/'results/contact_scheduling_joint_tracking_20261126';CASES=('m100_B','m100_R','m100_N','m100_NR');WINDOW=(31.4,31.745);SNAP=(31.4,31.5,31.6,31.65,31.685,31.7,31.725,31.74,31.745);LABELS=('shell_A','shell_B','belt_A','belt_B')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
 if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.generic):return safe(x.item())
 if isinstance(x,float)and not np.isfinite(x):return None
 return x
def save(p,x):p.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def up(q):
 q=q/np.linalg.norm(q,axis=1)[:,None];v=np.tile([0.,0.,1.],(len(q),1));c=np.cross(q[:,1:],v);return v+2*q[:,:1]*c+2*np.cross(q[:,1:],c)
def angle(a,b):
 a=a/np.linalg.norm(a,axis=-1,keepdims=True);b=b/np.linalg.norm(b,axis=-1,keepdims=True)
 return np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b),axis=-1),np.sum(a*b,axis=-1)))
def bracket(phase,phases):
 phase=min((phase,phase-2*np.pi,phase+2*np.pi),key=lambda p:max(phases[0]-p,0,p-phases[-1]));j=int(np.searchsorted(phases,phase));lo=max(0,j-1);hi=min(len(phases)-1,j);fraction=float(np.clip((phase-phases[lo])/max(phases[hi]-phases[lo],1e-9),0,1))
 return dict(local_phase_rad=phase,low_rad=phases[lo],high_rad=phases[hi],fraction=fraction,endpoint_clamped=lo==hi,span_rad=phases[hi]-phases[lo])

def main():
 out=H/'entering_stage02';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'analyze_entering.py')
 mapping=OLD/'campaign/resolved_case_paths.json';paths=json.loads(mapping.read_text())['cases'];folders={c:ROOT/paths[c]for c in CASES};bank=ROOT/'generated/offroad/flat/control_schedule.npz'
 inputs=[mapping,bank,H/'stage01/report.json',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/recovery_control.py']
 for f in folders.values():inputs.extend(f/n for n in ['precision.npz','rollout.npz','candidate_diagnostics.json','provenance.json'])
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
 save(out/'registered_before_arithmetic.json',dict(iteration='I-54',utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=hashes,window_s=WINDOW,snapshots_s=SNAP,
 rules=['All four arms over fixed31.400..31.745s inclusive; no outcome-picked subwindow.',
 'Decompose relative lean into actualup vs saveddesired up. Use their own net/arc changes and5msincrements, plus backward50msupvector chord rotation rates.',
 'ground_normal is logged smoothDEM terrain tangent at actualXY, not forceweighted actualcollisionnormal. Its discrepancy with estimatednormal is not an unqualified estimatorerror.',
 'Saved reference acceleration/normal/quat characterize desired target; no referencecontroller is invoked.',
 'Map currentphase/load to exact registered cached supportphase brackets using native periodic-nearest-interval convention; no newgainmodel/dynamics. Preserve loadedmixtures.',
 'Currentrequested/appliedtorques andbeltsprecede NEXT5ms; previousinput drives alreadyobserved stateincrement. No causal uniqueness claimed.']))
 cache=load(bank);groups={name:np.sort(cache['phase_rad'][(cache['regime_code']==(1 if name.startswith('belt')else 0))&(cache['side_code']==(1 if name.endswith('B')else 0))])for name in LABELS};report={'iteration':'I-54','window_s':WINDOW,'source_equation':'lean_xy=a_ref_xy/g - lean_gain*lean_factor*estimatednormal_xy, magnitude capped atsin(maxlean); q_ref=q_equilibrium*quat(rotvec(lean)); actualup uses measuredforkquat. Heading rightcomposition leavesup unchanged.','normal_truth_scope':'Logged smoothDEM tangent atactualcenterXY, not forceweighted contactcollisionnormal','gain_bank_phase_groups':groups,'cases':{}};csvrows=[]
 for c,folder in folders.items():
  p=load(folder/'precision.npz');l=load(folder/'rollout.npz');d=json.loads((folder/'candidate_diagnostics.json').read_text());t=p['time_s'];ii=np.flatnonzero((t>=WINDOW[0]-1e-9)&(t<=WINDOW[1]+1e-9));a=up(p['exact_qpos'][:,3:7]);b=up(p['exact_reference_quat']);lean=angle(a,b);actualtilt=angle(a,np.tile([0.,0.,1.],(len(t),1)));reftilt=angle(b,np.tile([0.,0.,1.],(len(t),1)));n=p['exact_estimated_normal'];g=l['ground_normal'].astype(float);K=p['exact_feedback_gain'];v=p['exact_qvel'];native=p['exact_native_joint_reference'];used=p['exact_guarded_joint_reference'];bounded=np.array([1.5,.6,.6,.3,.3]);request=p['exact_native_requested'];first,last=ii[[0,-1]];snapshots=[];phase_rows=[]
  for i in ii:
   loads=p['exact_estimated_support_loads_N'][i];brackets={LABELS[j]:bracket(p['exact_effective_phase_rad'][i],groups[LABELS[j]])for j in np.flatnonzero(loads>=.001)};phase_rows.append(dict(time_s=t[i],loads_N=loads,brackets=brackets,lookup=bool(d[i]['factors']['current_gain_lookup_requested'])))
   row=dict(case=c,time_s=t[i],relative_lean_deg=lean[i],absolute_tilt_deg=actualtilt[i],desired_absolute_tilt_deg=reftilt[i],actual_up_rate_5ms_deg_s=angle(a[i],a[i-1])/(t[i]-t[i-1]),desired_up_rate_5ms_deg_s=angle(b[i],b[i-1])/(t[i]-t[i-1]),actual_up_rate_50ms_deg_s=angle(a[i],a[i-10])/(t[i]-t[i-10]),desired_up_rate_50ms_deg_s=angle(b[i],b[i-10])/(t[i]-t[i-10]),estimated_normal_vs_smoothDEM_deg=angle(n[i],g[i]),lookup_phase_rad=p['exact_effective_phase_rad'][i],body_up_angular_rate_deg_s=np.degrees(np.linalg.norm(v[i,3:5])),gain_step_fraction=np.linalg.norm(K[i]-K[i-1])/np.linalg.norm(K[i-1]))
   for stem,arr in [('actual_joint',v[:,6:9]),('native_joint_ref',native),('used_joint_ref',used),('requested',request),('applied',p['exact_applied']),('previous_applied',p['exact_previous_applied'])]:row.update({stem+'_'+str(j):x for j,x in enumerate(arr[i])})
   csvrows.append(safe(row))
   if any(abs(t[i]-x)<1e-8 for x in SNAP):snapshots.append(dict(**row,actual_up=a[i],desired_up=b[i],estimated_normal=n[i],smoothDEM_normal=g[i],reference_acceleration_m_s2=p['exact_reference_acceleration_m_s2'][i],gain_brackets=brackets,estimated_loads_N=loads,truth_loads_N=p['truth_patch_loads_N'][i],sync_weights=p['exact_sync_weights'][i],guard_delta=p['exact_reference_guard_delta'][i],LQR_command=p['exact_lqr_command'][i],rolling_feedforward=p['exact_rolling_feedforward'][i],transition_delta=p['exact_transition_command_delta'][i]))
  report['cases'][c]=dict(samples=len(ii),relative_lean_start_end_deg=lean[[first,last]],actual_absolute_tilt_start_end_deg=actualtilt[[first,last]],desired_absolute_tilt_start_end_deg=reftilt[[first,last]],actual_up_net_change_deg=angle(a[first],a[last]),desired_up_net_change_deg=angle(b[first],b[last]),actual_up_arc_deg=np.sum(angle(a[ii[1:]],a[ii[:-1]])),desired_up_arc_deg=np.sum(angle(b[ii[1:]],b[ii[:-1]])),maximum_desired_up_step_deg=np.max(angle(b[ii],b[ii-1])),maximum_actual_up_step_deg=np.max(angle(a[ii],a[ii-1])),estimated_normal_net_change_deg=angle(n[first],n[last]),smoothDEM_normal_net_change_deg=angle(g[first],g[last]),normal_difference_range_deg=[np.min(angle(n[ii],g[ii])),np.max(angle(n[ii],g[ii]))],reference_acceleration_xy_range_m_s2=[np.min(p['exact_reference_acceleration_m_s2'][ii],axis=0),np.max(p['exact_reference_acceleration_m_s2'][ii],axis=0)],modified_guard_samples=int(np.sum(np.any(p['exact_reference_guard_delta'][ii]!=0,axis=1))),requested_at_cap_samples=np.sum(abs(request[ii])>=bounded-1e-12,axis=0),actual_joint_range_rad_s=[np.min(v[ii,6:9],axis=0),np.max(v[ii,6:9],axis=0)],native_joint_ref_range_rad_s=[np.min(native[ii],axis=0),np.max(native[ii],axis=0)],snapshots=snapshots,phase_brackets=phase_rows)
 save(out/'report.json',report)
 with(out/'all_samples.csv').open('w',newline='')as f:w=csv.DictWriter(f,fieldnames=list(csvrows[0]));w.writeheader();w.writerows(csvrows)
 for p,h in hashes.items():assert sha(ROOT/p)==h,p
 save(out/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(out)),sha256=sha(p),bytes=p.stat().st_size)for p in sorted(out.iterdir())if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps(safe({c:{k:v for k,v in r.items()if k not in('snapshots','phase_brackets')}for c,r in report['cases'].items()}),indent=2))
if __name__=='__main__':main()
