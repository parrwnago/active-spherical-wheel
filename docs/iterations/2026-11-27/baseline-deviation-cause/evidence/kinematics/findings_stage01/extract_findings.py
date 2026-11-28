"""I77 fixed event/cluster table from sealed full-history reconstruction."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
import numpy as np
BASE=Path(__file__).resolve().parent
CASES=('straight_m100_N','straight_p090_N')
FAMILIES=('shell_A','shell_B','belt_A','belt_B')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {k:safe(x)for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,float)and not np.isfinite(v):return None
 return v
def save(p,v):
 with p.open('x')as f:json.dump(safe(v),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def groups(mask,t):
 ids=np.flatnonzero(mask)
 if not len(ids):return []
 chunks=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
 return [dict(first_frame=int(x[0]),last_frame=int(x[-1]),first_s=float(t[x[0]]),last_s=float(t[x[-1]]),rows=len(x),held_duration_s=float(t[x[-1]]-t[x[0]]+.005))for x in chunks]
def main():
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();source=a.input.resolve();out=a.output.resolve()
 files=[Path(__file__).resolve(),source/'report.json',source/'artifact_manifest.json']+[source/(c+'_history.npz')for c in CASES]
 bindings={str(p):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
 out.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),out/'extract_findings.py')
 save(out/'registered_before_calculation.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),inputs=bindings,window_s=[18,22],detailed_window_s=[20.56,20.62],diagnostic_joint_reference_threshold_rad_s=20.,threshold_meaning='Existing assumed experimental reference scale only; not a hard or measured motor limit.',no_physics=True,no_new_model=True,no_control_change=True))
 report=json.loads((source/'report.json').read_text());results={}
 for case in CASES:
  with np.load(source/(case+'_history.npz'),allow_pickle=False)as z:h={k:z[k].copy()for k in z.files}
  t=h['time_s'];mask=(t>=20.56-1e-9)&(t<=20.62+1e-9);table=[]
  for i in np.flatnonzero(mask):
   table.append(dict(time_s=t[i],frame=i,branch=int(h['branch_code'][i]),rank=int(h['native_rank'][i]),singular_values=h['native_singular_values'][i],condition=h['native_condition_resolved'][i],joint_reference=h['native_reference'][i,:3],target_modal_projection=h['native_target_modal_projection'][i],scaled_reference_modal_coefficients=h['native_scaled_reference_modal_coefficients'][i],minimum_direction_collinearity_deg=h['native_minimum_acute_direction_angle_deg'][i],inferred_families=[FAMILIES[j]for j in np.flatnonzero(h['inferred_family_mask'][i])],positive_pressure_families=[FAMILIES[j]for j in np.flatnonzero(h['truth_positive_family_mask'][i])],point_error_m=h['estimated_to_pressure_equivalent_point_error_m'][i],truth_aggregate_reference=h['truth_aggregate_reference'][i],truth_aggregate_condition=h['truth_aggregate_condition_resolved'][i],same_family_point_oracle_available=h['same_family_point_oracle_available'][i],same_family_point_reference=h['same_family_point_reference'][i],same_family_point_condition=h['same_family_point_condition_resolved'][i]))
  window=(t>=18-1e-9)&(t<=22+1e-9)
  over=np.any(abs(h['native_reference'][:,:3])>20.,axis=1)
  clusters=groups(window&over,t)
  for g in clusters:
   idx=np.arange(g['first_frame'],g['last_frame']+1);g.update(maximum_absolute_joint_reference_rad_s=float(np.max(abs(h['native_reference'][idx,:3]))),maximum_resolved_condition=float(np.nanmax(h['native_condition_resolved'][idx])))
  key=min(report['cases'][case]['snapshots'],key=lambda k:abs(float(k)-20.585));snapshot=report['cases'][case]['snapshots'][key]
  results[case]=dict(reference_exact_rows=report['cases'][case]['reference_exact_rows'],total_rows=report['cases'][case]['rows'],detailed_5ms_rows=table,clusters_over_diagnostic_20rad_s=clusters,fixed20_585_snapshot=snapshot)
 if any(sha(Path(p))!=v['sha256']for p,v in bindings.items()):raise ValueError('Sealed findings input changed')
 save(out/'report.json',dict(iteration='I-77',cases=results,no_physics=True,no_control_change=True,comparison_limit='All oracle commands are offline algebraic substitutions. Positive-pressure contact aggregates are last-solve labels, not full5ms force truth.'))
 save(out/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in sorted(out.iterdir())if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps({c:dict(clusters=r['clusters_over_diagnostic_20rad_s'],focus=r['fixed20_585_snapshot']['native'])for c,r in results.items()},default=lambda x:x.tolist()if isinstance(x,np.ndarray)else x))
if __name__=='__main__':main()
