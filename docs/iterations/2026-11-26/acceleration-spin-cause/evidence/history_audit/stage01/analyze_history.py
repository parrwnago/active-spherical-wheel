"""I54 saved-history audit only. No controller, model or simulator imports."""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import csv
import shutil
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OLD=ROOT/'results/contact_scheduling_joint_tracking_20261126'
CASES=('m100_B','m100_R','m100_N','m100_NR')
LABELS=('shell_A','shell_B','belt_A','belt_B','none')
BOUNDS=np.array([1.5,.6,.6,.3,.3])
TERMS=('equilibrium_command','rolling_feedforward','support_command','lqr_command','transition_command_delta','heading_command','fade_command_delta')
GROUPS={'position':slice(0,3),'attitude':slice(3,6),'joint_position':slice(6,9),'linear_velocity':slice(9,12),'angular_velocity':slice(12,15),'joint_velocity':slice(15,18)}
WINDOWS={'whole_held':[0.,44.],'before_context':[0.,28.],'lead_in':[28.,31.77],'belt_interval':[31.77,32.25],'burst':[32.25,32.31],'post_burst':[32.31,36.],'final_settle':[41.,44.]}
FIXED_TIMES=(0.,1.,5.,10.,15.,20.,25.,28.,29.,30.,30.225,31.,31.5,31.7,31.77,32.,32.245,32.25,32.305,33.,34.,36.,41.,44.)

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
 if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.generic):return safe(x.item())
 if isinstance(x,float)and not np.isfinite(x):return None
 return x
def save(p,x):p.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def rotate(q,v):
 q=q/np.linalg.norm(q,axis=1)[:,None];c=np.cross(q[:,1:],v)
 return v+2*q[:,:1]*c+2*np.cross(q[:,1:],c)
def segments(mask):
 ii=np.flatnonzero(mask)
 return [g for g in np.split(ii,np.flatnonzero(np.diff(ii)>1)+1)if len(g)]
def first(mask,t):
 ii=np.flatnonzero(mask)
 return None if not len(ii)else dict(index=int(ii[0]),time_s=t[ii[0]])
def sustained(mask,t,dwell=.1):
 for s in segments(mask):
  if t[s[-1]]-t[s[0]]>=dwell-1e-9:return dict(start_s=t[s[0]],end_s=t[s[-1]],samples=len(s),span_s=t[s[-1]]-t[s[0]])
 return None
def idx(t,x):
 i=int(np.argmin(abs(t-x)))
 assert abs(t[i]-x)<.00251
 return i
def rms(x):return np.sqrt(np.mean(x*x,axis=0))if len(x)else None
def register():
 path=HERE/'registered_before_arithmetic.json'
 if path.exists():raise FileExistsError(path)
 mapping=OLD/'campaign/resolved_case_paths.json';resolved=json.loads(mapping.read_text())
 folders={c:ROOT/resolved['cases'][c]for c in CASES}
 inputs=[mapping,OLD/'protocol.json',OLD/'analysis/stage01/report.json',ROOT/'results/acceleration_spin_cause_20261126/registration.json',
  ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/recovery_control.py',ROOT/'src/spherical_wheel/experiments.py',
  OLD/'run_factorial_case.py',OLD/'candidate/scheduling_reference.py',ROOT/'results/allocation_handoff_isolation_20261126/reference_candidate/joint_reference_guard.py']
 inputs.extend(ROOT/p for p in resolved['technical_attempts'])
 for c,f in folders.items():
  inputs.extend(f/n for n in('precision.npz','rollout.npz','candidate_diagnostics.json','provenance.json','summary.json','artifact_manifest.json'))
  inputs.append(OLD/'analysis/stage01'/(c+'_geometry.npz'))
  if c.endswith('_R')or c.endswith('_NR'):inputs.append(OLD/'analysis/stage01'/(c+'_introduced_reference_residual.npz'))
 save(path,dict(iteration='I-54',registered_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),
  input_sha256={str(p.relative_to(ROOT)):sha(p)for p in inputs},case_paths={c:str(f.relative_to(ROOT))for c,f in folders.items()},
  method=dict(first_divergence='First row with any unequal value, also >1e-12 maximumabsolute component difference; compare B/R and N/NR separately, and B/N phase-policy effects.',
   guard='All nonzero exactused-minus-native samples; adjacent held rows cluster with5ms adjacency. Reconstruct clip from priorused output,100rad/s2*actualdt,20rad/s; no helper execution.',
   direct_guard_algebra='At each OWNsavedstate, used-minus-native LQR = K[:,15:18]@(used-native). Decompose by targetjoint and outputdrive. Only a pre-preparation/heading/fade static algebra diagnostic; no alternative postprocessed command or nextstate is claimed.',
   cross_arm='Same-time LQR difference = -(K1-K0)@(e1+e0)/2 -(K1+K0)/2@(e1-e0). This is an exact descriptive algebra identity across different histories, not an independent gain intervention.',
   windows_s=WINDOWS,snapshots_s=FIXED_TIMES,lean_onsets_deg=[5,8,10,12],lean_dwell_s=.1,
   support='Dominant raw estimated/currentlast1ms truth patch above max(.05N,5%weight); explicit held-gain lookup flag from saved diagnostics; no whole5ms truth implied.',
   turns='Copy six original per-case I52 corrected-reference results; do not substitute I41 baseline outcomes.',
   residual='Reuse sealed I52 currentestimatedXY-constraint native/used residuals on modified rows; no freshmodel/contact/geometry execution.',
   clock='State and observer at rowt precede currentcommand; currentcommand affects NEXT5ms. No causal inference from coincident times after histories diverge.'),
  restrictions=['No control/model/observer invocation','No integration, newphysics or renderer','No runtime/default/source edits','No outcome-dependent selection; allfourarms/allguardclusters retained']))
 print('registered',path)

def run():
 reg=json.loads((HERE/'registered_before_arithmetic.json').read_text());assert sha(__file__)==reg['source_sha256']
 for p,h in reg['input_sha256'].items():assert sha(ROOT/p)==h,p
 out=HERE/'stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'analyze_history.py')
 shutil.copyfile(HERE/'registered_before_arithmetic.json',out/'registered_before_arithmetic.json')
 old=json.loads((OLD/'analysis/stage01/report.json').read_text());P={};L={};D={};C={};CSV=[];checks={}
 for c in CASES:
  folder=ROOT/reg['case_paths'][c];p=load(folder/'precision.npz');l=load(folder/'rollout.npz');d=json.loads((folder/'candidate_diagnostics.json').read_text())
  P[c]=p;L[c]=l;D[c]=d;t=p['time_s'];q=p['exact_qpos'];v=p['exact_qvel'];K=p['exact_feedback_gain'];e=p['exact_error'];delta=p['exact_reference_guard_delta'];native=p['exact_native_joint_reference'];used=p['exact_guarded_joint_reference'];held=np.arange(len(t))<len(t)-1
  weight=json.loads((folder/'provenance.json').read_text())['weight_N'];threshold=max(.05,.05*weight)
  def dominant(a):return int(np.argmax(a))if a.sum()>=threshold else 4
  est=np.array([dominant(a)for a in p['exact_estimated_support_loads_N']]);truth=np.array([dominant(a)for a in p['truth_patch_loads_N']])
  actual_up=rotate(q[:,3:7],np.tile([0.,0.,1.],(len(t),1)));target_up=rotate(p['exact_reference_quat'],np.tile([0.,0.,1.],(len(t),1)))
  lean=np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n',actual_up,target_up),-1,1)));absolute=np.degrees(np.arccos(np.clip(actual_up[:,2],-1,1)))
  speed=np.linalg.norm(v[:,:2],axis=1);timed=np.linalg.norm(q[:,:2]-p['exact_reference_xy'],axis=1)
  groups={n:-np.einsum('nij,nj->ni',K[:,:,s],e[:,s])for n,s in GROUPS.items()}
  by_joint=K[:,:,15:18]*delta[:,None,:];direct=by_joint.sum(axis=2);pre=sum(p['exact_'+n]for n in TERMS[:4]);pre_native=pre-direct
  emod=e.copy();emod[:,15:18]+=delta
  checks[c+'_joint_velocity_error']=bool(np.max(abs(e[:,15:18]-(v[:,6:9]-used)))<1e-10)
  checks[c+'_LQRgroups']=bool(np.max(abs(sum(groups.values())-p['exact_lqr_command']))<1e-9)
  checks[c+'_direct_counterfactual']=bool(np.max(abs((-np.einsum('nij,nj->ni',K,e)+np.einsum('nij,nj->ni',K,emod))-direct))<1e-9)
  checks[c+'_term_request_sum']=bool(np.max(abs(sum(p['exact_'+n]for n in TERMS)-p['exact_native_requested']))<1e-9)
  checks[c+'_previous_applied']=bool(np.array_equal(p['exact_previous_applied'][1:],p['exact_applied'][:-1]))
  modified=np.any(delta!=0,axis=1);res=None
  if c.endswith('_R')or c.endswith('_NR'):
   res=load(OLD/'analysis/stage01'/(c+'_introduced_reference_residual.npz'))
   prev=np.zeros(3);outputs=[]
   for i in range(len(t)):
    dt=.005 if i==0 else t[i]-t[i-1];prev=np.clip(native[i],np.maximum(-20,prev-100*dt),np.minimum(20,prev+100*dt));outputs.append(prev.copy())
   checks[c+'_guard_exact_reconstruction']=bool(np.array_equal(np.asarray(outputs),used))
  lookup=np.array([bool(x['factors']['current_gain_lookup_requested'])for x in d])
  def snapshot(i):
   before=max(0,i-1);gainstep=np.linalg.norm(K[i]-K[before])/max(np.linalg.norm(K[before]),1e-30)
   z=dict(index=i,time_s=t[i],preceding_hold_time_s=t[before],xy=q[i,:2],reference_xy=p['exact_reference_xy'][i],reference_vxy=p['exact_reference_velocity_m_s'][i],actual_vxy=v[i,:2],speed_m_s=speed[i],timed_error_m=timed[i],lean_error_deg=lean[i],absolute_tilt_deg=absolute[i],body_angular_velocity_rad_s=v[i,3:6],actual_joint_rates_rad_s=v[i,6:9],native_reference_rad_s=native[i],used_reference_rad_s=used[i],reference_delta_rad_s=delta[i],joint_velocity_error_rad_s=e[i,15:18],native_joint_velocity_error_rad_s=v[i,6:9]-native[i],position_error_heading_m=e[i,:3],attitude_error_chart_rad=e[i,3:6],joint_position_error_rad=e[i,6:9],angular_velocity_error_rad_s=e[i,12:15],phase_rad=p['exact_effective_phase_rad'][i],estimated_patch=LABELS[est[i]],truth_last1ms_patch=LABELS[truth[i]],estimated_loads_N=p['exact_estimated_support_loads_N'][i],truth_loads_N=p['truth_patch_loads_N'][i],current_gain_lookup=lookup[i],gain_relative_step=gainstep,requested=p['exact_native_requested'][i],applied=p['exact_applied'][i],preceding_applied=p['exact_previous_applied'][i],sync_weights=p['exact_sync_weights'][i],terms={n:p['exact_'+n][i]for n in TERMS},LQR_groups={n:a[i]for n,a in groups.items()},guard_direct_LQR_used_minus_native=direct[i],guard_direct_by_target_joint=by_joint[i],pre_sync_native_reference_command=pre_native[i],pre_sync_used_reference_command=pre[i],native_vs_used_pre_sync_box_difference=np.clip(pre[i],-BOUNDS,BOUNDS)-np.clip(pre_native[i],-BOUNDS,BOUNDS))
   if res is not None:z['saved_introduced_residual']=dict(native_norm_m_s=res['native_residual_norm_m_s'][i],used_norm_m_s=res['used_residual_norm_m_s'][i],point_quality=int(res['point_quality'][i]))
   return z
  clusters=[]
  for a in segments(modified&held):
   ii=int(a[0]);jj=int(a[-1]);select=a[np.argmax(np.linalg.norm(direct[a],axis=1))]
   clusters.append(dict(cluster=len(clusters)+1,start_s=t[ii],end_s=t[jj],samples=len(a),held_duration_s=np.sum(np.diff(t)[a]),first=snapshot(ii),last=snapshot(jj),max_direct_feedback_change=snapshot(int(select)),max_reference_delta_rad_s=np.max(abs(delta[a]),axis=0),max_direct_feedback_change_by_input=np.max(abs(direct[a]),axis=0),net_integrated_reference_delta_rad=np.sum(delta[a]*np.diff(t)[a,None],axis=0),support_labels=sorted(set(LABELS[x]for x in est[a])),all_indices=a))
  windows={}
  for name,(lo,hi)in WINDOWS.items():
   m=held&(t>=lo-1e-9)&(t<hi-1e-9);ii=np.flatnonzero(m)
   windows[name]=dict(samples=len(ii),modified_held_samples=int(np.sum(m&modified)),lean_max_deg=lean[m].max(),lean_start_deg=lean[ii[0]],lean_end_deg=lean[ii[-1]],speed_max_m_s=speed[m].max(),actual_joint_rms_rad_s=rms(v[m,6:9]),actual_minus_native_joint_rms_rad_s=rms(v[m,6:9]-native[m]),actual_minus_used_joint_rms_rad_s=rms(v[m,6:9]-used[m]),timed_rms_m=rms(timed[m]),requested_at_or_above_cap_by_input=np.sum(abs(p['exact_native_requested'][m])>=BOUNDS-1e-12,axis=0),LQR_group_rms_by_input={n:rms(a[m])for n,a in groups.items()},current_gain_lookup_holds=int(lookup[m].sum()),estimated_patch_holds={LABELS[k]:int(np.sum(est[m]==k))for k in range(5)})
  support=[]
  context=held&(t>=28-1e-9)&(t<32.31-1e-9)
  for name,arr in [('estimate',est),('truth_last1ms',truth)]:
   ii=np.flatnonzero(context)
   for g in np.split(ii,np.flatnonzero(np.diff(arr[ii])!=0)+1):
    if len(g):support.append(dict(source=name,start_s=t[g[0]],end_s=t[g[-1]],label=LABELS[arr[g[0]]],samples=len(g)))
  onsets={str(th):dict(first_anywhere=sustained((lean>=th)&held,t),first_after28=sustained((lean>=th)&held&(t>=28-1e-9),t))for th in(5,8,10,12)}
  C[c]=dict(modified_held_samples=int(np.sum(modified&held)),modified_all_emissions=int(modified.sum()),guard_clusters=clusters,windows=windows,lean_sustained_onsets=onsets,support_segments_28_32p31=support,fixed_snapshots=[snapshot(idx(t,x))for x in FIXED_TIMES],original_turn_results=old['cases'][c]['turns'],guard_direct_change_scope='Same own state/gain and current feedforward; pre-sync static feedback only. Boxes here are illustrative pre-sync boxes, not rerun final actuation.',max_direct_feedback_change_by_input=np.max(abs(direct[held]),axis=0),net_integrated_guard_reference_delta_rad=np.sum(delta[held]*np.diff(t)[:,None],axis=0))
  if res is not None:
   m=modified&held&(res['point_quality']>0);C[c]['introduced_reference_residual']=dict(samples=int(m.sum()),native_rms_m_s=rms(res['native_residual_norm_m_s'][m]),used_rms_m_s=rms(res['used_residual_norm_m_s'][m]))
  # Every guard-active row remains inspectable, with all output/target coupling terms.
  for i in np.flatnonzero(modified&held):
   row=dict(case=c,index=int(i),time_s=t[i],lean_error_deg=lean[i],estimated_patch=LABELS[est[i]],truth_patch=LABELS[truth[i]])
   for stem,a in [('actual',v[:,6:9]),('native',native),('used',used),('guard_delta',delta),('direct_LQR_delta',direct),('requested',p['exact_native_requested']),('applied',p['exact_applied'])]:
    row.update({stem+'_'+str(k):val for k,val in enumerate(a[i])})
   if res is not None:row.update(native_residual_m_s=res['native_residual_norm_m_s'][i],used_residual_m_s=res['used_residual_norm_m_s'][i])
   CSV.append(safe(row))
  # Arrays for independent saved-data review/plots, no fresh policy execution.
  np.savez_compressed(out/(c+'_history.npz'),time_s=t,lean_error_deg=lean,absolute_tilt_deg=absolute,speed_m_s=speed,timed_error_m=timed,estimated_patch=est,truth_last1ms_patch=truth,modified=modified,current_gain_lookup=lookup,direct_guard_LQR_delta=direct,direct_guard_by_joint=by_joint,**{'LQR_group_'+n:a for n,a in groups.items()})
 comparisons={}
 fields=('exact_native_joint_reference','exact_guarded_joint_reference','exact_native_requested','exact_applied','exact_qpos','exact_qvel','exact_observer_wrench','exact_estimated_normal','exact_estimated_support_loads_N','exact_feedback_gain','exact_nominal_joints_before','exact_reference_quat')
 for a,b in [('m100_B','m100_R'),('m100_N','m100_NR'),('m100_B','m100_N')]:
  x,y=P[a],P[b];t=x['time_s'];assert np.array_equal(t,y['time_s']);div={}
  for name in fields:
   z=abs(y[name]-x[name]).reshape(len(t),-1);m=z.max(axis=1)
   div[name]=dict(exact=first(np.any(y[name].reshape(len(t),-1)!=x[name].reshape(len(t),-1),axis=1),t),above1e12=first(m>1e-12,t))
  K0,K1=x['exact_feedback_gain'],y['exact_feedback_gain'];e0,e1=x['exact_error'],y['exact_error'];dk=-np.einsum('nij,nj->ni',K1-K0,(e1+e0)/2);de=-np.einsum('nij,nj->ni',(K1+K0)/2,e1-e0);total=y['exact_lqr_command']-x['exact_lqr_command']
  checks[a+'_'+b+'_crossarm_identity']=bool(np.max(abs(dk+de-total))<1e-9)
  times=sorted(set(list(FIXED_TIMES)+[float(z['above1e12']['time_s'])for z in div.values()if z['above1e12']is not None]));rows=[]
  for now in times:
   i=idx(t,now);rows.append(dict(time_s=t[i],qpos_difference=y['exact_qpos'][i]-x['exact_qpos'][i],qvel_difference=y['exact_qvel'][i]-x['exact_qvel'][i],requested_difference=y['exact_native_requested'][i]-x['exact_native_requested'][i],applied_difference=y['exact_applied'][i]-x['exact_applied'][i],LQR_difference=total[i],gain_algebra=dk[i],error_algebra=de[i],error_group_algebra={n:-.5*(K1[i,:,s]+K0[i,:,s])@(e1[i,s]-e0[i,s])for n,s in GROUPS.items()}))
  comparisons[a+'_vs_'+b]=dict(first_differences=div,selected_same_time_algebra=rows,scope='Different histories after first applied divergence; these are descriptive saved-array identities, not same-state alternative trajectories.')
 report=dict(iteration='I-54',cases=C,comparisons=comparisons,checks=checks,passed=all(checks.values()),limitations=reg['restrictions']+['All-time guard replay reconstructs numerical target projection only; no controller or physical trajectory is replayed.','The guard can directly change every input row via fullK joint-velocity columns, including shaft row; it does not impose actual speed/acceleration bounds.','Nominal material joint references integrate guarded rates, but native feedback explicitly zeroes shell material-angle errors and replaces shaft angle error with rolling position/pitch relation; not a direct shaft material-angle servo.','Only saved reference residuals on modified rows are used. Residuals describe reference geometry, not actual no-slip or torque feasibility.','Lean and command histories can show entering-state chronology, not identify a unique full-trajectory physical cause.'])
 save(out/'report.json',report)
 with(out/'guard_samples.csv').open('w',newline='')as f:
  keys=list(dict.fromkeys(k for row in CSV for k in row));w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(CSV)
 for p,h in reg['input_sha256'].items():assert sha(ROOT/p)==h,p
 save(out/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(out)),bytes=p.stat().st_size,sha256=sha(p))for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps(safe(dict(passed=report['passed'],checks=checks,cases={c:dict(modified_held_samples=C[c]['modified_held_samples'],clusters=len(C[c]['guard_clusters']))for c in CASES},first_BR=comparisons['m100_B_vs_m100_R']['first_differences'])),indent=2))
 if not report['passed']:raise AssertionError('Preserve failed audit stage')

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--register',action='store_true');parser.add_argument('--run',action='store_true');args=parser.parse_args()
 if args.register:register()
 elif args.run:run()
 else:parser.error('Choose --register or --run')
