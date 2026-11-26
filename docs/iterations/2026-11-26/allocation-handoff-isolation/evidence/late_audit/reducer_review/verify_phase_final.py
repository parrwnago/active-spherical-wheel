"""Independent frozen-window fifth-addendum arithmetic; no model/reducer imports."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil
import numpy as np
H=Path(__file__).resolve().parent;F=H.parents[1];ROOT=F.parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
 if isinstance(x,dict):return {k:safe(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 return x
def save(p,x):p.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def main():
 out=H/'phase_numeric_stage02';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'verify_phase_final.py')
 folders={'base030':F/'campaign/runs/base030','phase_late030':F/'phase_schedule_isolation/campaign/runs/phase_late030'};canonical=F/'analysis/phase_stage01/report.json';protocol=F/'phase_schedule_isolation/protocol.json';inputs=[canonical,protocol,*[p/'precision.npz'for p in folders.values()]]
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs};save(out/'before_arithmetic.json',{'utc':datetime.now(timezone.utc).isoformat(),'source_sha256':sha(__file__),'input_sha256':hashes,'scope':'Independentrawtimed/heading/perjoint-rate/lean arithmetic overbothregisteredlatewindows. No model/controller/physics/oldreducer imports; no window changes.'})
 r=json.loads(canonical.read_text());checks=[];values={}
 for case,folder in folders.items():
  with np.load(folder/'precision.npz')as z:p={k:z[k].copy()for k in z.files}
  t=p['time_s'];q=p['exact_qpos'][:,3:7];w,x,y,z=q.T;heading=np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)));err=np.linalg.norm(p['exact_qpos'][:,:2]-p['exact_reference_xy'],axis=1);rate=p['exact_qvel'][:,6:9]-p['exact_feedback_reference_qvel'][:,6:9]
  def up(a):
   a=a/np.linalg.norm(a,axis=1)[:,None];w,x,y,z=a.T;return np.column_stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
  a=up(q);b=up(p['exact_reference_qpos'][:,3:7]);lean=np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i',a,b),-1,1)))
  windows={}
  for name,hi in [('primary38_42',42),('secondary38_44',44)]:
   m=(t>=38-1e-9)&(t<=hi+1e-9);ii=np.flatnonzero(m);computed=dict(samples=int(m.sum()),timed_rms_m=np.sqrt(np.mean(err[m]**2)),timed_peak_m=err[m].max(),timed_final_m=err[ii[-1]],peak_reference_relative_lean_deg=lean[m].max(),yaw_change_from_window_start_deg=np.degrees(heading[ii[-1]]-heading[ii[0]]),maximum_yaw_change_from_window_start_deg=np.degrees(abs(heading[m]-heading[ii[0]]).max()),joint_rate_tracking_rms_rad_s=np.sqrt(np.mean(rate[m]**2,axis=0)),joint_rate_tracking_max_abs_rad_s=abs(rate[m]).max(axis=0))
   for key,value in computed.items():np.testing.assert_allclose(value,r['cases'][case]['late'][name][key],atol=1e-10,rtol=1e-11);checks.append(case+'/'+name+'/'+key)
   windows[name]=computed
  values[case]=windows
 assert r['cases']['base030']['turns']==r['cases']['phase_late030']['turns'];checks.append('all6turnsidenticalbefore38intervention')
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 save(out/'report.json',{'status':'pass','assertion_count':len(checks),'checks':checks,'independent_late_values':values,'interpretation':'Lateerrorreductionwithlargerheadingchangeandcontinuedunsettledmotion; noproofofuniversalphasepolicy. No new repositorytestcount.'})
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('PhasefinalindependentQApased',len(checks))
if __name__=='__main__':main()
