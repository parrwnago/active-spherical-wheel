"""Standalone NumPy replay of the sealed minimal I59 coefficient/bound audit."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def replay(source,output,threshold):
 original=json.loads((source/'coefficients_and_pre_fade_effects.json').read_text())
 for row in json.loads((source/'artifact_manifest.json').read_text())['files']:
  if sha(source/row['path'])!=row['sha256']:raise ValueError('Changed sealed coefficient artifact: '+row['path'])
 anchor_ids=list(original['anchors']);effects={};delta_exact={}
 for case in original['gain_row_effects']:
  with np.load(source/(case+'_gain_row_inputs_and_delta.npz'),allow_pickle=False)as z:
   error=z['error'];actual=z['actual_K0'];anchors=z['anchor_K0'];saved=z['delta_pre_fade_shaft']
  contributions=-(anchors[None,:,:]-actual[None,None,:])*error[:,None,:]
  delta=contributions.sum(axis=2)
  delta_exact[case]=delta.dtype==saved.dtype and delta.shape==saved.shape and delta.tobytes()==saved.tobytes()
  effects[case]={name:dict(max_abs_pre_fade_shaft_Nm=float(np.max(abs(delta[:,j]))),RMS_pre_fade_shaft_Nm=float(np.sqrt(np.mean(delta[:,j]**2))),
   count_above_shaft_material_threshold=int(np.sum(abs(delta[:,j])>threshold)),maximum_raw_and_native_clipped_effect_upper_bound_Nm=float(np.max(abs(delta[:,j]))))for j,name in enumerate(anchor_ids)}
 result=dict(effects_exact=effects==original['gain_row_effects'],all_delta_arrays_byte_exact=all(delta_exact.values()),
  delta_arrays_byte_exact=delta_exact,absolute_material_threshold_Nm=threshold,effects=effects,
  source_report_sha256=sha(source/'coefficients_and_pre_fade_effects.json'),new_physics_runs=0,
  limitation='Fixed-state native-command bound; P is native-policy shadow. No changed trajectory or stability inference.')
 output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
 if not result['effects_exact']or not result['all_delta_arrays_byte_exact']:raise ValueError('Replay differs')
 print(json.dumps(dict(effects_exact=True,all_delta_arrays_byte_exact=True,cases=len(effects))))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--data-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--material-threshold',type=float,default=.001);a=p.parse_args()
 if a.output.exists():raise FileExistsError(a.output)
 replay(a.data_source.resolve(),a.output.resolve(),a.material_threshold)
