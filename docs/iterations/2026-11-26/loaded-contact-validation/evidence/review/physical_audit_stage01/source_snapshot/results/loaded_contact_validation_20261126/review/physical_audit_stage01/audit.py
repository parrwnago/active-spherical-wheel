"""Independent I57 saved-endpoint audit; no physics/model/control imports.

All physical comparisons subtract endpoint angular velocities in world axes
before projecting onto the common initial fork axes. Constants/criteria derive
from the frozen protocol. Predictive coefficients are never fitted to outcomes.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse
import hashlib
import json
import shutil
import numpy as np

HERE=Path(__file__).resolve().parent
BASE=HERE.parents[1]
ROOT=BASE.parents[1]
PRED=BASE/'model_tools/predictions_stage01'
GROUPS={'planar':([0,1],'planar_m_s'),'transverse':([6,7],'transverse_rad_s'),
 'world_yaw':([5],'world_yaw_rad_s'),'shaft':([9],'hinges_rad_s'),
 'shell_A':([10],'hinges_rad_s'),'shell_B':([11],'hinges_rad_s')}


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def save(p,v):Path(p).write_text(json.dumps(safe(v),indent=2,allow_nan=False)+'\n')
def safe(v):
 if isinstance(v,dict):return {k:safe(x)for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,float)and not np.isfinite(v):return None
 return v
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def exact(a,b):
 a,b=np.asarray(a),np.asarray(b)
 return a.shape==b.shape and a.dtype==b.dtype and a.tobytes()==b.tobytes()
def rot(q):
 w,x,y,z=np.asarray(q)/np.linalg.norm(q)
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def delta(a,b,i,r0):
 va,vb=a['exact_qvel'][i],b['exact_qvel'][i]
 angular=rot(a['exact_qpos'][i,3:7])@va[3:6]-rot(b['exact_qpos'][i,3:7])@vb[3:6]
 return np.r_[va[:3]-vb[:3],angular,r0.T@angular,va[6:9]-vb[6:9]]
def scores(observed,predicted,protocol):
 result={}
 screen=protocol['screening']
 for name,(indices,key)in GROUPS.items():
  o,p=observed[indices],predicted[indices];on,pn=np.linalg.norm(o),np.linalg.norm(p);floor=screen['floors'][key]
  err=np.linalg.norm(o-p)/max(on,floor)
  cosine=float(o@p/(on*pn))if on>floor and pn>floor else None
  ratio=on/pn if pn>floor else None
  if on<=floor and pn<=floor:status='unresolved_below_floor'
  elif pn<=floor:status='false_authority_observed_resolved'
  elif on<=floor:status='overprediction_observed_unresolved'
  else:
   passed=err<=screen['relative_vector_error_max']and cosine>=screen['cosine_min']and screen['gain_ratio_bounds'][0]<=ratio<=screen['gain_ratio_bounds'][1]
   status='resolved_pass'if passed else'resolved_fail'
  result[name]=dict(observed=o,predicted=p,observed_norm=on,predicted_norm=pn,floor=floor,error=err,cosine=cosine,gain_ratio=ratio,status=status,
   predicted_nullspace=(pn<=floor),observed_quiet=(on<=floor))
 return result

def support_modes(z):
 rows=z['contact_rows'];offset=z['contact_offsets'];allloads=[]
 for lo,hi in zip(offset[:-1],offset[1:]):
  loads=np.zeros(5)
  for r in rows[lo:hi]:
   if r[12]<.5:continue
   tag=int(r[0]);group={0:0,4:0,1:1,5:1,2:2,3:3,6:4}[tag]
   loads[group]+=max(0,float(r[4:7]@r[7:10]))
  allloads.append(loads)
 modes=[tuple(np.array(['shell_A','shell_B','belt_A','belt_B','body'])[x>.05])for x in allloads]
 return dict(normal_loads_N=allloads,modes=modes,endpoint_mode_changed=any(x!=modes[0]for x in modes[1:]),
  limitation='Endpoint force samples only; no certification of mode or friction feasibility within5ms holds.')

def register():
 path=HERE/'registration.json'
 sources=[Path(__file__),BASE/'protocol.json',BASE/'campaign_tools/run_probe.py',BASE/'model_tools/evaluate_frozen_model.py',
  PRED/'artifact_manifest.json',PRED/'declared_before_prediction.json']
 sources += sorted(PRED.glob('*/matrices.npz'))+sorted(PRED.glob('*/predictions.json'))
 if path.exists():
  record=read(path)
  for p,h in record['source_sha256'].items():
   if sha(ROOT/p)!=h:raise ValueError('Frozen audit source/prediction changed: '+p)
  return record
 snapshot=HERE/'source_snapshot';snapshot.mkdir()
 for p in sources:
  target=snapshot/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
 record=dict(registered_before_first_endpoint_read_utc=datetime.now(timezone.utc).isoformat(),
  source_sha256={str(p.relative_to(ROOT)):sha(p)for p in sources},known_before_registration='Root relayed two continue_saved pilots passed exact prefixes and20ms continuations; no native endpoint values read by this audit.',
  purpose=__doc__,metrics='Actual endpoint-own rotations, common initial projection; compare frozen loaded/free matrices independently, retain physical mode changes.',
  limitations='Two prior-outcome-selected states; no full-trajectory causal attribution, fitting, retuning or controller adoption.')
 save(path,record);return record

def audit(name):
 registered=register();protocol=read(BASE/'protocol.json');out=HERE/name;out.mkdir(exist_ok=False)
 manifest=read(PRED/'artifact_manifest.json')
 for row in manifest['files']:
  if sha(PRED/row['path'])!=row['sha256']:raise ValueError('Prediction artifact changed')
 jobs={r['case_id']:r for r in protocol['cases']};events={r['event_id']:r for r in protocol['events']}
 cases={};inputs={};failures={};available=[]
 for case in jobs:
  folder=BASE/'campaign/runs'/case
  if(folder/'failure.json').exists():failures[case]=read(folder/'failure.json')
  if(folder/'artifact_manifest.json').exists()and(folder/'provenance.json').exists():
   prov=read(folder/'provenance.json')
   if prov.get('success'):available.append(case)
 for case in available:
  folder=BASE/'campaign/runs'/case;sealed=read(folder/'artifact_manifest.json')
  for row in sealed['files']:
   if sha(folder/row['path'])!=row['sha256']:raise ValueError('Sealed branch artifact changed: '+case+'/'+row['path'])
  for name_ in ('endpoints.npz','prefix.npz','provenance.json','artifact_manifest.json'):
   path=folder/name_;inputs[str(path.relative_to(ROOT))]=sha(path)
 save(out/'registered_before_endpoint_arithmetic.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),
  available_cases=available,input_sha256=inputs,static_registration_sha256=sha(HERE/'registration.json')))
 source=load(ROOT/protocol['source_data']['precision_path'])
 for case in available:
  folder=BASE/'campaign/runs'/case;z=load(folder/'endpoints.npz');prefix=load(folder/'prefix.npz');prov=read(folder/'provenance.json');job=jobs[case];event=events[job['event_id']];index=event['index']
  checks={k:exact(prefix[k],source[k][:index+1])for k in('time_s','exact_qpos','exact_qvel','exact_previous_applied')}
  checks['prefix_count']=len(prefix['time_s'])==index+1
  checks['anchor_equals_original']=all(exact(z[k][0],source[k][index])for k in('time_s','exact_qpos','exact_qvel','exact_previous_applied'))
  checks['all_five_endpoints']=exact(z['offset_steps'],np.arange(5))
  checks['command_amplitude']=bool(np.all(abs(z['command_sequence'])<=np.array([1.5,.6,.6,.3,.3])))
  checks['no_post_init_reset']=prov['reset_count_initial']==prov['reset_count_final']
  if job['mode']=='continue_saved':
   checks.update({'original_continuation_'+k:exact(z[k],source[k][index:index+5])for k in('time_s','exact_qpos','exact_qvel','exact_previous_applied')})
  model=load(PRED/job['event_id']/'matrices.npz');r0=rot(z['exact_qpos'][0,3:7])
  checks['frozen_model_anchor_qv']=exact(z['exact_qpos'][0],model['qpos'])and exact(z['exact_qvel'][0],model['qvel'])
  checks['rotation_map_world']=bool(np.max(abs(r0-model['h020ms_physical_map'][3:6,3:6]))<1e-14)
  bridgeomega=z['bridge_state'][:,12:15]
  ownworld=np.array([rot(q[3:7])@v[3:6]for q,v in zip(z['exact_qpos'],z['exact_qvel'])])
  bridge_error=float(np.max(abs(bridgeomega-ownworld)))
  checks['bridge_world_omega_consistent']=bridge_error<=1e-12
  cases[case]=dict(job=job,checks=checks,all_checks=all(checks.values()),bridge_world_omega_max_error=bridge_error,
   contact=support_modes(z),data=z,provenance=prov)
 report=dict(iteration='I-57',available_cases=available,missing_cases=[c for c in jobs if c not in available],failures=failures,
  frame_convention='Endpointworldomega difference then R0.T projection; compare full worldtranslation and relativehingerates. Both loaded/free predictions use the same frozen source state/terms.',
  cases={k:{n:v for n,v in r.items()if n not in('data','provenance')}for k,r in cases.items()},events={})
 for event_id,event in events.items():
  selected={k:r for k,r in cases.items()if r['job']['event_id']==event_id};base=selected.get(event_id+'_governed');repeat=selected.get(event_id+'_governed_repeat');native=selected.get(event_id+'_native');mid=selected.get(event_id+'_midpoint')
  er=dict(duplicate_available=base is not None and repeat is not None,pairs={},columns={},nonlinearity={})
  if er['duplicate_available']:
   equal={k:exact(base['data'][k],repeat['data'][k])for k in base['data']}
   er['duplicate_fields_exact']=equal;er['duplicate_all_exact']=all(equal.values())
  model=load(PRED/event_id/'matrices.npz');r0=rot(model['qpos'][3:7])
  if base is not None:
   pairs=[]
   if native is not None:pairs.append(('governed_minus_native',base,native))
   if mid is not None and native is not None:pairs.append(('midpoint_minus_native',mid,native))
   pairs.extend((case,r,base)for case,r in selected.items()if r['job'].get('role')=='governed_center_column_probe')
   for label,a,b in pairs:
    du=a['data']['command_sequence'][0]-b['data']['command_sequence'][0];pr={}
    for h,offset in(('h005ms',1),('h020ms',4)):
     observed=delta(a['data'],b['data'],offset,r0);loaded=model[h+'_physical_map']@model[h+'_response']@du;free=model[h+'_physical_map']@model[h+'_free_response']@du
     pr[h]=dict(delta_input=du,observed_physical_delta=observed,loaded_prediction=loaded,free_prediction=free,
      loaded=scores(observed,loaded,protocol),free=scores(observed,free,protocol))
    er['pairs'][label]=pr
   for j in range(5):
    full=selected.get(event_id+f'_u{j}_full');half=selected.get(event_id+f'_u{j}_half')
    if full is None or half is None:continue
    full_delta=full['data']['command_sequence'][0]-base['data']['command_sequence'][0]
    half_delta=half['data']['command_sequence'][0]-base['data']['command_sequence'][0]
    rows={}
    for h,offset in(('h005ms',1),('h020ms',4)):
     of=delta(full['data'],base['data'],offset,r0);oh=delta(half['data'],base['data'],offset,r0);items={}
     for group,(indices,key)in GROUPS.items():
      floor=protocol['screening']['floors'][key];den=max(np.linalg.norm(of[indices]),floor)
      value=float(np.linalg.norm(of[indices]-2*oh[indices])/den)
      items[group]=dict(ratio=value,unresolved=bool(max(np.linalg.norm(of[indices]),np.linalg.norm(oh[indices]))<=floor),
       passes=bool(value<=protocol['screening']['full_half_nonlinearity_max']))
     rows[h]=items
    er['nonlinearity'][f'u{j}']=dict(full_delta=full_delta,half_delta=half_delta,full_half_delta_exact=bool(np.max(abs(full_delta-2*half_delta))<=1e-14),horizons=rows)
  er['all_available_case_checks']=all(r['all_checks']for r in selected.values())
  report['events'][event_id]=er
 save(out/'report.json',report)
 for path,h in inputs.items():
  if sha(ROOT/path)!=h:raise ValueError('Input changed while reading: '+path)
 for path,h in registered['source_sha256'].items():
  if sha(ROOT/path)!=h:raise ValueError('Frozen source changed while reading: '+path)
 save(out/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in out.iterdir()if p.is_file()]))
 print(json.dumps(dict(available=len(available),missing=len(report['missing_cases']),failures=list(failures),
  controls={k:dict(checks=v['all_available_case_checks'],duplicate=v.get('duplicate_all_exact'))for k,v in report['events'].items()})))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--snapshot',required=True);a=p.parse_args();audit(a.snapshot)
