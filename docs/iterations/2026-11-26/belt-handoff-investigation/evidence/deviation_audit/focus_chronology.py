"""I50 second-stage declared rate-spike/late-run characterization, saved arrays only."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil,csv
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];OLD=ROOT/'results/belt_capacity_sweep_20261126'
CASES=('mars_cap030','mars_cap060','mars_cap090');LABELS=('shell_A','shell_B','belt_A','belt_B','none')
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
def main():
 out=HERE/'stage02_focused';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'focus_chronology.py')
 inputs=[]
 for case in CASES:inputs.extend([OLD/'campaign/runs'/case/f for f in ('precision.npz','rollout.npz','provenance.json')]);inputs.append(OLD/'analysis/stage01'/(case+'_geometry.npz'))
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
 save(out/'declaration_before_arithmetic.json',dict(source_sha256=sha(__file__),utc=datetime.now(timezone.utc).isoformat(),input_sha256=hashes,registered_queries={'rate_spikes':'First absjointreference>5/10/50/100rad/s and max absjointreference in fixed30–34+39–44s. Thresholds diagnostic, not hardwarelimits. Current exactreference_qvel[6:9] and allocationdrive[0:3] separately.','late_onset':'For each fulltrace localdistance max in39–44, find contiguous above20/50/100/150mm run containingthatpeak usingfulltrace toavoidwindowcensor; report onset+runlength, no firstcause claim.','snapshots':'All5ms30–34 and39–44 already preserved in stage01; add detailed every25ms31.5–31.85 and every50ms42.8–44 plus detectedthreshold onsets andallocationpeaks.'},scope='Descriptive supplementarysavedarray arithmetic; no controller/model/simulation/forecast calls. Native synchronousclock, no CPUlatencyclaim.'))
 report={};csvrows=[]
 for case in CASES:
  folder=OLD/'campaign/runs'/case;p=load(folder/'precision.npz');l=load(folder/'rollout.npz');g=load(OLD/'analysis/stage01'/(case+'_geometry.npz'));t=p['time_s'];K=p['exact_feedback_gain'];e=p['exact_error'];local=g['local_path_distance_m'];weight=json.loads((folder/'provenance.json').read_text())['weight_N']
  def dom(load):return LABELS[int(np.argmax(load))]if load.max()>=.05*weight else'none'
  def snapshot(i):
   b=max(0,i-1);rate=p['exact_reference_qvel'][i,6:9]
   groups={name:-K[i,:,s]@e[i,s]for name,s in GROUPS.items()}
   gj=-(K[i]-K[b])@((e[i]+e[b])/2);ej=-((K[i]+K[b])/2)@(e[i]-e[b])
   return dict(index=int(i),time_s=t[i],local_distance_m=local[i],timed_error_m=np.linalg.norm(p['exact_qpos'][i,:2]-p['exact_reference_xy'][i]),reference_joint_rates_rad_s=rate,allocation_joint_reference_rad_s=p['exact_allocation_drive_reference'][i,:3],actual_joint_rates_rad_s=p['exact_qvel'][i,6:9],raw_joint_errors_rad=e[i,6:9],linear_error_m_s=e[i,9:12],angular_error_rad_s=e[i,12:15],requested=p['exact_native_requested'][i],applied=p['exact_applied'][i],request_step=p['exact_native_requested'][i]-p['exact_native_requested'][b],gain_jump_LQR=gj,error_jump_LQR=ej,LQR=p['exact_lqr_command'][i],LQRgroups=groups,rolling=p['exact_rolling_feedforward'][i],support=p['exact_support_command'][i],syncdelta=p['exact_transition_command_delta'][i],syncweights=p['exact_sync_weights'][i],reference_xy=p['exact_reference_xy'][i],reference_velocity_m_s=p['exact_reference_velocity_m_s'][i],reference_acceleration_m_s2=p['exact_reference_acceleration_m_s2'][i],actual_linear_velocity_m_s=p['exact_qvel'][i,:3],actual_angular_velocity_rad_s=p['exact_qvel'][i,3:6],true_patch=dom(p['truth_patch_loads_N'][i]),estimated_patch=dom(p['exact_estimated_support_loads_N'][i]),true_loads_N=p['truth_patch_loads_N'][i],estimated_loads_N=p['exact_estimated_support_loads_N'][i],estimated_normal=p['exact_estimated_normal'][i],true_contact_force_N=l['contact_force_N'][i],slip_speed_m_s=l['slip_speed_m_s'][i])
  selected=set(int(np.argmin(abs(t-x)))for x in np.r_[np.arange(31.5,31.85001,.025),np.arange(42.8,44.00001,.05),[39,40,41,42,43]])
  windows={}
  for key,(lo,hi)in {'twitch':[30.,34.],'late':[39.,44.]}.items():
   mask=(t>=lo-1e-9)&(t<=hi+1e-9);ii=np.flatnonzero(mask);queries={}
   for field in ('exact_reference_qvel','exact_allocation_drive_reference'):
    values=abs(p[field][:,6:9]if field=='exact_reference_qvel'else p[field][:,:3]);mag=values.max(axis=1);found={}
    for threshold in (5.,10.,50.,100.):
     jj=np.flatnonzero(mask&(mag>threshold));found[str(threshold)]=None if not len(jj)else snapshot(int(jj[0]));selected.update([]if not len(jj)else[int(jj[0])])
    im=int(ii[np.argmax(mag[ii])]);selected.add(im);queries[field]=dict(first_threshold_exceedance_rad_s=found,maximum=snapshot(im))
   windows[key]=queries
  ii=np.flatnonzero((t>=39.-1e-9)&(t<=44.+1e-9));peak=int(ii[np.argmax(local[ii])]);onsets={}
  for th in (.02,.05,.1,.15):
   if local[peak]<=th:onsets[str(th)]=None;continue
   start=peak;end=peak
   while start>0 and local[start-1]>th:start-=1
   while end<len(t)-1 and local[end+1]>th:end+=1
   selected.add(start);onsets[str(th)]=dict(onset=snapshot(start),last_s=t[end],span_s=t[end]-t[start],reaches_trace_end=bool(end==len(t)-1))
  snapshots=[snapshot(i)for i in sorted(selected)]
  for row in snapshots:
   x={k:v for k,v in row.items()if not isinstance(v,dict)};flat={'case':case}
   for k,v in x.items():
    if isinstance(v,np.ndarray):flat.update({k+'_'+str(j):a for j,a in enumerate(v)})
    else:flat[k]=v
   for group,vec in row['LQRgroups'].items():flat.update({'LQR_'+group+'_'+str(j):a for j,a in enumerate(vec)})
   csvrows.append(safe(flat))
  report[case]=dict(rate_queries=windows,late_threshold_run_containing_peak=onsets,late_peak=snapshot(peak),snapshots=snapshots)
 save(out/'report.json',report)
 with(out/'focused_samples.csv').open('w',newline='')as f:w=csv.DictWriter(f,fieldnames=csvrows[0].keys());w.writeheader();w.writerows(csvrows)
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 save(out/'artifact_manifest.json',{'files':[{'path':str(f.relative_to(out)),'bytes':f.stat().st_size,'sha256':sha(f)}for f in sorted(out.rglob('*'))if f.is_file()and f.name!='artifact_manifest.json']})
 print('I50 focused chronology complete')
if __name__=='__main__':main()
