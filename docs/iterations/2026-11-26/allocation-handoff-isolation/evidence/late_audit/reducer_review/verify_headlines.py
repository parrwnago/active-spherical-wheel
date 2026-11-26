"""Independent I51 headline saved-array arithmetic; no reducer imports/dynamics."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil,traceback
import numpy as np
H=Path(__file__).resolve().parent;F=H.parents[1];ROOT=F.parents[1];CASES=('base030','reference030','base060','handoff060');DT=.005
PHASE=F/'phase_schedule_isolation/campaign/runs/phase_late030'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
 if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.generic):return safe(x.item())
 return x

def save(p,x):p.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def up(q):
 q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
 return np.column_stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
def rotate(q,v):
 q=q/np.linalg.norm(q,axis=1)[:,None];c=np.cross(q[:,1:],v);return v+2*q[:,:1]*c+2*np.cross(q[:,1:],c)
def headline(p):
 t=p['time_s'];err=np.linalg.norm(p['exact_qpos'][:,:2]-p['exact_reference_xy'],axis=1);moving=(t>=1)&(t<=41)
 a=up(p['exact_qpos'][:,3:7]);r=up(p['exact_reference_qpos'][:,3:7]);relative=np.arccos(np.clip(np.sum(a*r,axis=1),-1,1));absolute=np.arccos(np.clip(a[:,2],-1,1));tail=t>=t[-1]-1
 settled=bool(np.all(np.degrees(relative[tail])<1)&np.all(np.linalg.norm(p['exact_qvel'][tail,:2],axis=1)<.03)&np.all(np.linalg.norm(p['exact_qvel'][tail,3:6],axis=1)<.08))
 return dict(moving_rms_m=np.sqrt(np.mean(err[moving]**2)),moving_p95_m=np.quantile(err[moving],.95),full_peak_m=err.max(),final_m=err[-1],peak_absolute_deg=np.degrees(absolute).max(),peak_reference_error_deg=np.degrees(relative).max(),settled=settled)
def turn(p,e):
 t=p['time_s'];a=e['axis_index'];lo,hi=e['window_s'];m=(t>=lo-1e-9)&(t<=hi+1e-9);s=e['incoming_sign']*(p['exact_qpos'][:,a]-e['reference_component_extremum_m']);overshoot=max(0,float(s[m].max()))
 velocity=p['exact_qvel'][:,a];outgoing=np.sign(velocity)==e['outgoing_sign'];starts=np.flatnonzero(outgoing&~np.r_[False,outgoing[:-1]]);ends=np.flatnonzero(outgoing&~np.r_[outgoing[1:],False]);found=[]
 for first,last in zip(starts,ends):
  if not lo-1e-9<=t[first]<=hi+1e-9 or t[last]-t[first]<.1-1e-10 or t[first]+.1>hi+1e-9:continue
  previous=first-1
  while previous>=0 and velocity[previous]==0:previous-=1
  if previous<0 or np.sign(velocity[previous])!=e['incoming_sign']:continue
  frac=abs(velocity[first-1])/(abs(velocity[first-1])+abs(velocity[first]));found.append(t[first-1]+frac*(t[first]-t[first-1])-e['time_s'])
 return dict(overshoot_m=overshoot,signed_lag_s=None if not found else found[0])
def main():
 out=H/'numeric_stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'verify_headlines.py')
 proto=json.loads((F/'protocol.json').read_text());inputs=[F/'analysis/stage01/report.json',F/'analysis/stage01/artifact_manifest.json',F/'protocol.json',ROOT/proto['inputs']['event_turns'],PHASE/'precision.npz',PHASE/'artifact_manifest.json',F/'review/late_phase_stage01/report.json',F/'analysis/stage01/reference030_current_rolling_residual.npz']
 for c in CASES:inputs.extend([F/'campaign/runs'/c/f for f in('precision.npz','rollout.npz','summary.json','candidate_diagnostics.json','artifact_manifest.json')])
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs};save(out/'before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=hashes,scope='IndependentpureNumPyheadlines/all6turns/mechanisticcontracts andfifthphasepair, no canonicalreducerimport/controller/model/physics. Reference030Jresidualreconstructedanalytically fromcurrentestimatedgeometry for allusable rows. No criterionchanges.'))
 assertions=[]
 def eq(name,actual,expected,tol=1e-11):
  if actual is None or expected is None:assert actual is expected,(name,actual,expected)
  elif isinstance(actual,(bool,np.bool_)):assert bool(actual)==bool(expected),(name,actual,expected)
  else:np.testing.assert_allclose(actual,expected,atol=tol,rtol=1e-11,err_msg=name)
  assertions.append(name)
 report=json.loads((F/'analysis/stage01/report.json').read_text());events=json.loads((ROOT/proto['inputs']['event_turns']).read_text())['rows'];allraw={c:load(F/'campaign/runs'/c/'precision.npz')for c in CASES};independent={}
 for c,p in allraw.items():
  r=report['cases'][c];h=headline(p)
  for key in('moving_rms_m','moving_p95_m','full_peak_m','final_m'):eq(c+'/'+key,h[key],r['timed_tracking'][key])
  for key in('peak_absolute_deg','peak_reference_error_deg'):eq(c+'/'+key,h[key],r['attitude'][key])
  eq(c+'/settled',h['settled'],r['original_settling']['passed'])
  turns=[]
  for e,expected in zip(events,r['turns']):
   value=turn(p,e);eq(c+'/overshoot/'+e['event_id'],value['overshoot_m'],expected['geometric_component_overshoot_m']);eq(c+'/lag/'+e['event_id'],value['signed_lag_s'],expected['actual_reversal_lag_s']);turns.append(value)
  change=np.any(p['exact_reference_guard_delta']!=0,axis=1);projected=np.any(p['exact_applied']!=p['exact_native_applied'],axis=1)
  eq(c+'/modified_reference_held',int(change[:-1].sum()),r['mechanistic']['reference']['modified_held_samples']);eq(c+'/modified_handoff_held',int(projected[:-1].sum()),r['mechanistic']['handoff']['modified_held_samples'])
  eq(c+'/nativeclip',p['exact_native_applied'],np.clip(p['exact_native_requested'],-p['exact_input_bounds'],p['exact_input_bounds']))
  eq(c+'/totalprojection',p['exact_clipping_command_delta'],p['exact_applied']-p['exact_native_requested']);eq(c+'/handoffprojection',p['exact_handoff_command_delta'],p['exact_applied']-p['exact_native_applied'])
  eq(c+'/requestedcapfraction',np.mean(np.any(abs(p['exact_native_requested'][:-1])>p['exact_input_bounds'][:-1],axis=1)),r['commands']['original_any_request_strict_saturated_fraction'])
  independent[c]=dict(headline=h,turns=turns,reference_modified_held=int(change[:-1].sum()),handoff_modified_held=int(projected[:-1].sum()))
 p=allraw['reference030'];l=load(F/'campaign/runs/reference030/rollout.npz');native=guarded=None;n=len(p['time_s']);q=p['exact_qpos'];axisY=rotate(q[:,3:7],np.tile([0.,1.,0.],(n,1)));axisX=rotate(q[:,3:7],np.column_stack([np.cos(q[:,7]),np.zeros(n),-np.sin(q[:,7])]))
 loaded=p['exact_estimated_support_loads_N']>0;counts=loaded.sum(axis=1);squares_native=np.zeros(n);squares_guard=np.zeros(n);good=counts>0
 for j in range(4):
  point=l['estimated_contact_points_m'][:,j].astype(float);point[counts==1]=p['exact_estimated_contact_point_m'][counts==1];r=point-q[:,:3];shaft=np.cross(axisY,r)[:,:2]
  driven=np.cross(axisX,r)[:,:2]if j<2 else axisY[:,:2]
  ndrive=p['exact_native_joint_reference'][:,j+1]if j<2 else p['exact_rolling_feedforward'][:,j+1]
  gdrive=p['exact_guarded_joint_reference'][:,j+1]if j<2 else ndrive
  nr=shaft*p['exact_native_joint_reference'][:,:1]+driven*ndrive[:,None]+p['exact_reference_velocity_m_s'];gr=shaft*p['exact_guarded_joint_reference'][:,:1]+driven*gdrive[:,None]+p['exact_reference_velocity_m_s'];active=loaded[:,j]
  good&=~active|np.isfinite(point).all(axis=1);squares_native[active]+=np.sum(nr[active]**2,axis=1);squares_guard[active]+=np.sum(gr[active]**2,axis=1)
 native=np.sqrt(squares_native);guarded=np.sqrt(squares_guard);sidecar=load(F/'analysis/stage01/reference030_current_rolling_residual.npz')
 eq('reference/analytic_native_residual_allusable',native[good],sidecar['native_residual_norm_m_s'][good],1e-9);eq('reference/analytic_guard_residual_allusable',guarded[good],sidecar['guarded_residual_norm_m_s'][good],1e-9)
 modified=np.any(p['exact_reference_guard_delta']!=0,axis=1)&good;kinematic=dict(modified_rows=int(modified.sum()),max_native_m_s=native[modified].max(),max_guarded_m_s=guarded[modified].max(),rms_guarded_m_s=np.sqrt(np.mean(guarded[modified]**2)))
 mech=report['cases']['reference030']['mechanistic']['rolling_reference_residual'];eq('reference/modified_geometry_rows',kinematic['modified_rows'],mech['modified_rows_with_geometry']);eq('reference/max_native_residual',kinematic['max_native_m_s'],mech['max_native_residual_on_modified_m_s'],1e-9);eq('reference/max_guarded_residual',kinematic['max_guarded_m_s'],mech['max_guarded_residual_on_modified_m_s'],1e-9)
 phase=load(PHASE/'precision.npz');base=allraw['base030'];mask=base['time_s']<38-1e-9;common=set(phase)&set(base)
 for key in common:
  actual,expected=phase[key][mask],base[key][mask];assert actual.shape==expected.shape and actual.dtype==expected.dtype and actual.tobytes()==expected.tobytes(),key;assertions.append('phase/exactbyteprefix/'+key)
 phase_head=headline(phase);base_head=headline(base);phase_summary=dict(base=base_head,phase=phase_head,prefix_rows=int(mask.sum()),common_prefix_fields=len(common))
 for name,p in [('base',base),('phase',phase)]:
  err=np.linalg.norm(p['exact_qpos'][:,:2]-p['exact_reference_xy'],axis=1);m=(p['time_s']>=38-1e-9)&(p['time_s']<=44+1e-9);phase_summary[name]['late38_44_rms_m']=np.sqrt(np.mean(err[m]**2))
 phase_expected=json.loads((F/'review/late_phase_stage01/report.json').read_text())['cases'];phase_key=next(key for key in phase_expected if key!='baseline')
 for label,expected in [('base',phase_expected['baseline']),('phase',phase_expected[phase_key])]:
  for ours,theirs in [('moving_rms_m','moving_rms_m'),('full_peak_m','peak_error_m'),('final_m','final_error_m'),('peak_reference_error_deg','peak_reference_lean_deg')]:eq('phase/'+label+'/'+ours,phase_summary[label][ours],expected[theirs])
  eq('phase/'+label+'/late38_44_rms',phase_summary[label]['late38_44_rms_m'],expected['windows']['38_44']['position_rms_m'])
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 save(out/'report.json',dict(status='pass',assertion_count=len(assertions),assertions=assertions,cases=independent,reference_current_kinematic_residual=kinematic,phase_samehistory_pair=phase_summary,scope='Independentnumericassertions, not new repositorytestcount. No physics/model/controllercalls.'))
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('IndependentheadlineQApassed',len(assertions))
if __name__=='__main__':main()
