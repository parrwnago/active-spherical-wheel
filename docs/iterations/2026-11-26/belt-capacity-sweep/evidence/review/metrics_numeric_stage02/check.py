from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path.cwd();F=ROOT/'results/belt_capacity_sweep_20261126';O=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):
 with np.load(p) as z:return {k:z[k].copy() for k in z.files}
def up(q):
 q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
 return np.column_stack((2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)))
def check(name,x,y,tol=1e-10):
 gap=float(np.max(np.abs(np.asarray(x)-np.asarray(y))));checks[name]={'passed':gap<=tol,'maximum_absolute_gap':gap,'tolerance':tol}
reportpath=F/'analysis/stage01/report.json';turnpath=ROOT/'results/figure8_event_audit_20261125/events/selection_stage02_corrected/turn_events.json'
inputs={str(p.relative_to(ROOT)):sha(p) for p in [reportpath,turnpath]};r=json.loads(reportpath.read_text());events=json.loads(turnpath.read_text())['rows'];checks={};arrays={}
for ident,c in r['cases'].items():
 p=F/'campaign/runs'/ident/'precision.npz';inputs[str(p.relative_to(ROOT))]=sha(p);z=load(p);arrays[ident]=z;t=z['time_s'];xy=z['exact_qpos'][:,:2];err=np.linalg.norm(xy-z['exact_reference_xy'],axis=1);moving=(t>=1)&(t<=41)
 stats={'moving_rms_m':np.sqrt(np.mean(err[moving]**2)),'moving_p95_m':np.quantile(err[moving],.95),'full_peak_m':err.max(),'final_m':err[-1],'full_rms_m':np.sqrt(np.mean(err**2))}
 for key,value in stats.items():check(ident+'/tracking/'+key,value,c['timed_tracking'][key])
 angle=np.degrees(np.arccos(np.clip(np.sum(up(z['exact_qpos'][:,3:7])*up(z['exact_reference_quat']),axis=1),-1,1)))
 check(ident+'/peak_reference_lean',angle.max(),c['attitude']['peak_reference_error_deg'],1e-8)
 for event,row in zip(events,c['turns']):
  a,b=event['window_s'];mask=(t>=a-1e-12)&(t<=b+1e-12);overshoot=max(0.,float(np.max(event['incoming_sign']*(xy[mask,event['axis_index']]-event['reference_component_extremum_m']))))
  check(ident+'/'+event['event_id']+'/overshoot',overshoot,row['geometric_component_overshoot_m'])
 check(ident+'/requested_max_belts',np.max(abs(z['exact_native_requested'][:-1,3:]),axis=0),[c['belts']['belt_A']['max_requested_abs_m_s'],c['belts']['belt_B']['max_requested_abs_m_s']])
 check(ident+'/applied_max_belts',np.max(abs(z['exact_applied'][:-1,3:]),axis=0),[c['belts']['belt_A']['max_applied_abs_m_s'],c['belts']['belt_B']['max_applied_abs_m_s']])
checks['higher_cap_body_motion_identical']={'passed':all(np.array_equal(arrays['mars_cap060'][k],arrays['mars_cap090'][k]) for k in ['exact_qpos','exact_qvel','exact_observer_wrench','exact_native_requested'])}
checks['higher_cap_applied_NOT_identical']={'passed':not np.array_equal(arrays['mars_cap060']['exact_applied'],arrays['mars_cap090']['exact_applied'])}
checks['inputs_unchanged']={'passed':all(sha(ROOT/p)==digest for p,digest in inputs.items())}
result={'iteration':'I-49','passed':all(v['passed'] for v in checks.values()),'scope':'Independent direct saved-array RMS/peak/final/up-vector attitude/6fixedturn overshoots and input maxima. No canonical reducer import, model/controller/physics or reference retiming.','checks':checks,'assertions':len(checks),'inputs_sha256':inputs,'source_sha256':sha(Path(__file__))}
(O/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));assert result['passed']
