"""I59 minimal NumPy coefficient audit; fixed recorded errors, zero new physics.

Shaft sync weight is structurally zero; unload fade lies in[0,1]. Therefore
absolute pre-fade shaft differences bound raw native and amplitude-clipped
shaft differences, without treating float32 fade as exact.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
import numpy as np


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def exact(a,b):
 a,b=np.asarray(a),np.asarray(b);return a.shape==b.shape and a.dtype==b.dtype and a.tobytes()==b.tobytes()
def safe(x):
 if isinstance(x,dict):return {k:safe(v)for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 return x
def save(p,x):Path(p).write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')


def run(protocol_path,output):
 base=protocol_path.parent.parent;root=base.parents[1];protocol=json.loads(protocol_path.read_text());seal=json.loads((protocol_path.parent/'input_source_seal.json').read_text())
 for row in seal['inputs']+seal['sources']:
  if not row.get('missing')and sha(root/row['path'])!=row['sha256']:raise ValueError('Sealed input/source changed: '+row['path'])
 output.mkdir(parents=True,exist_ok=False);shutil.copyfile(Path(__file__),output/'coefficient_audit_source.py')
 save(output/'registered_before_arithmetic.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(Path(__file__)),
  protocol_sha256=sha(protocol_path),input_source_seal_sha256=sha(protocol_path.parent/'input_source_seal.json'),
  input_sha256={r['path']:r['sha256']for r in seal['inputs']},scope=__doc__))
 data={};meta={}
 fields=('time_s','exact_feedback_gain','exact_error','exact_equilibrium_command','exact_sync_weights','exact_native_requested','exact_native_applied')
 for case in protocol['datasets']:
  item=next(r for r in seal['inputs']if r['case']==case and Path(r['path']).name=='precision.npz')
  with np.load(root/item['path'],allow_pickle=False)as z:data[case]={k:z[k].copy()for k in fields}
  p=next(r for r in seal['inputs']if r['case']==case and Path(r['path']).name=='provenance.json');meta[case]=json.loads((root/p['path']).read_text())
 anchors={case:dict(K0=data[case]['exact_feedback_gain'][0,0],trim=data[case]['exact_equilibrium_command'][0])for case in protocol['anchor_cases']}
 checks={};effects={};pairs=[]
 for left,a in anchors.items():
  for right,b in anchors.items():pairs.append(dict(left=left,right=right,K0_byte_exact=exact(a['K0'],b['K0']),trim_byte_exact=exact(a['trim'],b['trim']),
   K0_max_abs_difference=float(np.max(abs(a['K0']-b['K0']))),trim_max_abs_difference=float(np.max(abs(a['trim']-b['trim'])))))
 for case,d in data.items():
  K=d['exact_feedback_gain'][:,0,:];trim=d['exact_equilibrium_command'];own=case[:-1]+'B';anchor=anchors[own]
  checks[case]=dict(constant_shaft_row=exact(K,np.broadcast_to(K[0],K.shape)),constant_trim=exact(trim,np.broadcast_to(trim[0],trim.shape)),
   B_P_shaft_row_same=exact(K[0],anchor['K0']),B_P_trim_same=exact(trim[0],anchor['trim']),shaft_sync_zero=bool(np.all(d['exact_sync_weights'][:,0]==0)),
   finite=bool(np.isfinite(K).all()and np.isfinite(d['exact_error']).all()))
  if not all(checks[case].values()):raise ValueError('Coefficient assumption failed: '+case+' '+str(checks[case]))
  contributions=np.stack([-(a['K0']-K[0])[None,:]*d['exact_error']for a in anchors.values()],axis=1)
  delta=contributions.sum(axis=2);np.savez_compressed(output/(case+'_gain_row_inputs_and_delta.npz'),time_s=d['time_s'],
   error=d['exact_error'],actual_K0=K[0],anchor_K0=np.stack([a['K0']for a in anchors.values()]),delta_pre_fade_shaft=delta,
   contributions=contributions,native_requested=d['exact_native_requested'],native_applied=d['exact_native_applied'])
  effects[case]={a:dict(max_abs_pre_fade_shaft_Nm=float(np.max(abs(delta[:,j]))),RMS_pre_fade_shaft_Nm=float(np.sqrt(np.mean(delta[:,j]**2))),
   count_above_shaft_material_threshold=int(np.sum(abs(delta[:,j])>protocol['absolute_material_thresholds'][0])),
   maximum_raw_and_native_clipped_effect_upper_bound_Nm=float(np.max(abs(delta[:,j]))))for j,a in enumerate(anchors)}
 result=dict(iteration='I-59',new_physics_runs=0,coefficient_pairs=pairs,assumption_checks=checks,gain_row_effects=effects,
  anchors={k:dict(K0=v['K0'],trim=v['trim'])for k,v in anchors.items()},
  interpretation='Preliminary coefficient/gain-row calculation. Bounds exploit zero shaft sync and fade in[0,1]; this does not substitute for detailed fade/trim/native-stage reconstruction or imply a trajectory change.')
 save(output/'coefficients_and_pre_fade_effects.json',result)
 save(output/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in output.iterdir()if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps(dict(all_coefficients_byte_identical=all(x['K0_byte_exact']for x in pairs),maximum_K0_difference=max(x['K0_max_abs_difference']for x in pairs),
  maximum_trim_difference=max(x['trim_max_abs_difference']for x in pairs),maximum_gain_row_pre_fade_effect=max(x['max_abs_pre_fade_shaft_Nm']for c in effects.values()for x in c.values()),
  material_gain_row_samples=sum(x['count_above_shaft_material_threshold']for c in effects.values()for x in c.values())),indent=2))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--protocol',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.protocol.resolve(),a.output.resolve())
