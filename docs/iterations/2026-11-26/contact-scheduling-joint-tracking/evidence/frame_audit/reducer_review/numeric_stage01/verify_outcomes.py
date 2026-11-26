"""Independent I52 raw headline/turn/joint/factorial audit, no reducer imports."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil,traceback
import numpy as np
H=Path(__file__).resolve().parent;F=H.parents[1];ROOT=F.parents[1];PHASES=('m100','p030','p090');VARIANTS=('B','R','N','NR');DT=.005

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):p.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def safe(x):
 if isinstance(x,dict):return {k:safe(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.generic):return safe(x.item())
 if isinstance(x,float)and not np.isfinite(x):return None
 return x
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def up(q):
 q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
 return np.column_stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
def headlines(p,completed):
 t=p['time_s'];err=np.linalg.norm(p['exact_qpos'][:,:2]-p['exact_reference_xy'],axis=1);m=(t>=1)&(t<=41);tail=t>=t[-1]-1
 a=up(p['exact_qpos'][:,3:7]);b=up(p['exact_reference_qpos'][:,3:7]);relative=np.arccos(np.clip(np.sum(a*b,axis=1),-1,1));absolute=np.arccos(np.clip(a[:,2],-1,1))
 settled=bool(completed and np.all(np.degrees(relative[tail])<1)and np.all(np.linalg.norm(p['exact_qvel'][tail,:2],axis=1)<.03)and np.all(np.linalg.norm(p['exact_qvel'][tail,3:6],axis=1)<.08))
 return dict(moving_rms_m=None if not m.any()else np.sqrt(np.mean(err[m]**2)),moving_p95_m=None if not m.any()else np.quantile(err[m],.95),full_peak_m=err.max(),final_m=err[-1],peak_absolute_deg=np.degrees(absolute).max(),peak_reference_error_deg=np.degrees(relative).max(),settled=settled)
def turns(p,event):
 t=p['time_s'];axis=event['axis_index'];lo,hi=event['window_s'];m=(t>=lo-1e-9)&(t<=hi+1e-9);signed=event['incoming_sign']*(p['exact_qpos'][:,axis]-event['reference_component_extremum_m']);overshoot=None if not m.any()else max(0,float(signed[m].max()));vel=p['exact_qvel'][:,axis];mask=np.sign(vel)==event['outgoing_sign']
 starts=np.flatnonzero(mask&~np.r_[False,mask[:-1]]);ends=np.flatnonzero(mask&~np.r_[mask[1:],False]);cross=[]
 for first,last in zip(starts,ends):
  if not lo-1e-9<=t[first]<=hi+1e-9 or t[last]-t[first]<.1-1e-10 or t[first]+.1>hi+1e-9:continue
  previous=first-1
  while previous>=0 and vel[previous]==0:previous-=1
  if previous<0 or np.sign(vel[previous])!=event['incoming_sign']:continue
  frac=abs(vel[first-1])/(abs(vel[first-1])+abs(vel[first]));cross.append(t[first-1]+frac*(t[first]-t[first-1])-event['time_s'])
 return dict(geometric_component_overshoot_m=overshoot,actual_reversal_lag_s=None if not cross else cross[0],window_complete=bool(t[0]<=lo+1e-9 and t[-1]>=hi-1e-9),actual_reversal_missing_in_window=not bool(cross))
def main():
 out=H/'numeric_stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'verify_outcomes.py');proto=json.loads((F/'protocol.json').read_text());mapping=F/'campaign/resolved_case_paths.json';resolved=json.loads(mapping.read_text())['cases'];assert set(resolved)=={j['case_id']for j in proto['cases']};case_folder=lambda case:ROOT/resolved[case];canonical=F/'analysis/stage01/report.json';paths=[F/'protocol.json',mapping,canonical,F/'analysis/stage01/artifact_manifest.json',ROOT/proto['inputs']['event_turns']]
 for job in proto['cases']:
  folder=case_folder(job['case_id'])
  for name in('precision.npz','rollout.npz','summary.json','provenance.json','artifact_manifest.json'):
   if(folder/name).exists():paths.append(folder/name)
 hashes={str(p.relative_to(ROOT)):sha(p)for p in paths};save(out/'before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=hashes,scope='Independentrawall12headlines,all72fixedturns,heldjointtracking(nativeandused), factorialsigneddifferences. No canonicalreducer/helper/model/controller/physicsimports. Missing/incompleteoutcomesretained; no criteria/windowchanges.'))
 checks=[]
 def eq(name,a,b,tol=1e-10):
  if a is None or b is None:assert a is b,(name,a,b)
  elif isinstance(a,(bool,np.bool_)):assert bool(a)==bool(b),(name,a,b)
  else:np.testing.assert_allclose(a,b,atol=tol,rtol=1e-11,err_msg=name)
  checks.append(name)
 r=json.loads(canonical.read_text());assert len(r['cases'])==12;checks.append('all12retained')
 events=json.loads((ROOT/proto['inputs']['event_turns']).read_text())['rows'];independent={};scalars={}
 for job in proto['cases']:
  case=job['case_id'];expected=r['cases'][case];folder=case_folder(case)
  if not expected.get('available'):
   assert expected['completed']is False;independent[case]={'available':False};scalars[case]={};continue
  p=load(folder/'precision.npz');log=load(folder/'rollout.npz');summary=json.loads((folder/'summary.json').read_text());t=p['time_s'];h=headlines(p,summary['completed'])
  for key in('moving_rms_m','moving_p95_m','full_peak_m','final_m'):eq(case+'/'+key,h[key],expected['timed_tracking'][key])
  for key in('peak_absolute_deg','peak_reference_error_deg'):eq(case+'/'+key,h[key],expected['attitude'][key])
  eq(case+'/settled',h['settled'],expected['original_settling']['passed']);tt=[]
  for event,e in zip(events,expected['turns']):
   value=turns(p,event)
   for key,v in value.items():eq(case+'/'+event['event_id']+'/'+key,v,e[key])
   tt.append(value)
  actual=p['exact_qvel'][:,6:9];native=p['exact_native_joint_reference'];used=p['exact_guarded_joint_reference'];windowstats={}
  for name,w in expected['joint_tracking']['windows'].items():
   mask=np.arange(len(t))<len(t)-1;window=w['window_s']
   if window is not None:mask&=(t>=window[0]-1e-9)&(t<window[1]-1e-9)
   eq(case+'/'+name+'/heldsamplecount',int(mask.sum()),w['held_samples']);ww={}
   for key,values in [('actual_minus_native_ref',actual-native),('actual_minus_guarded_ref',actual-used)]:
    if mask.any():
     err=values[mask];norm=np.linalg.norm(err,axis=1);val=dict(rms_byjoint=np.sqrt(np.mean(err**2,axis=0)),rms_norm=np.sqrt(np.mean(norm**2)),max_abs_byjoint=abs(err).max(axis=0))
     for k,v in val.items():eq(case+'/'+name+'/'+key+'/'+k,v,w[key][k])
     ww[key]=val
    else:assert not w[key]['available'];ww[key]=None
   windowstats[name]=ww
  modified=np.any(used!=native,axis=1);eq(case+'/modifiedheld',int(modified[:-1].sum()),expected['joint_tracking']['modified_held_samples'])
  m=np.arange(len(t))<len(t)-1;weight=json.loads((folder/'provenance.json').read_text())['weight_N'];codes={};active={}
  for source,key in [('estimated','exact_estimated_support_loads_N'),('truth','truth_patch_loads_N')]:
   loads=p[key][m];total=loads.sum(axis=1);active[source]=total>=max(.05,.05*weight);shares=np.divide(loads,total[:,None],out=np.zeros_like(loads),where=total[:,None]>0);loaded=(shares>=.05)&active[source][:,None];codes[source]=np.sum(loaded*(1<<np.arange(4)),axis=1)
  common=active['estimated']&active['truth'];mismatch=int(np.sum(common&(codes['estimated']!=codes['truth'])));eq(case+'/common_loaded',int(common.sum()),expected['support_modes']['comparison']['common_loaded_samples']);eq(case+'/supportsetmismatch',mismatch,expected['support_modes']['comparison']['loaded_set_disagreement_samples'])
  scalar=dict(moving_rms_m=h['moving_rms_m'],peak_error_m=h['full_peak_m'],final_error_m=h['final_m'],peak_reference_lean_deg=h['peak_reference_error_deg'],actual_minus_native_joint_error_rms_rad_s=windowstats['full']['actual_minus_native_ref']['rms_norm']if m.any()else None,actual_minus_used_joint_error_rms_rad_s=windowstats['full']['actual_minus_guarded_ref']['rms_norm']if m.any()else None,slip_distance_m=float(log['slip_speed_m_s'][:-1].astype(float).sum()*DT),support_set_disagreement_duration_s=mismatch*DT)
  scalars[case]=scalar;independent[case]=dict(available=True,headlines=h,turns=tt,joint_windows=windowstats,completed=summary['completed'],completed_duration_s=t[-1])
 factorials={}
 for phase,info in r['per_phase'].items():
  phaseeffects={}
  for metric,effect in info['factorial_effects'].items():
   minimum_end=41. if metric=='moving_rms_m'else 44.
   values={v:(scalars[phase+'_'+v].get(metric)if independent[phase+'_'+v].get('available')and independent[phase+'_'+v]['completed_duration_s']>=minimum_end-1e-9 else None)for v in VARIANTS}
   for v,value in values.items():eq(phase+'/'+metric+'/value/'+v,value,effect['values'][v])
   eq(phase+'/'+metric+'/availability',all(value is not None for value in values.values()),effect['available'])
   if not effect.get('available'):
    assert any(v is None for v in values.values());phaseeffects[metric]={'available':False};continue
   b,ref,n,both=(values[v]for v in VARIANTS);contrasts=dict(R_minus_B=ref-b,N_minus_B=n-b,NR_minus_B=both-b,NR_minus_N=both-n,NR_minus_R=both-ref,reference_main_effect=.5*((ref-b)+(both-n)),normal_main_effect=.5*((n-b)+(both-ref)),interaction_difference_of_differences=both-n-ref+b)
   for key,value in contrasts.items():eq(phase+'/'+metric+'/'+key,value,effect[key])
   phaseeffects[metric]=contrasts
  factorials[phase]=phaseeffects
 for relative,h in hashes.items():assert sha(ROOT/relative)==h
 save(out/'report.json',dict(status='pass',assertion_count=len(checks),checks=checks,cases=independent,factorial_effects=factorials,scope='Independentnumericalassertions, not newrepositorytests. No physics/controller/modelcalls.'))
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('I52independentnumericQApassed',len(checks))
if __name__=='__main__':main()
