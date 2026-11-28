"""I77 fixed late-eight window from sealed exact-state reconstruction."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
import numpy as np
CASE='eight_m100_N'
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
def summary(t,x,mask):
 ids=np.flatnonzero(mask&np.isfinite(x))
 if not len(ids):return dict(samples=0)
 peak=int(ids[np.argmax(x[ids])]);low=int(ids[np.argmin(x[ids])])
 return dict(samples=len(ids),minimum=float(x[low]),minimum_time_s=float(t[low]),median=float(np.median(x[ids])),maximum=float(x[peak]),maximum_time_s=float(t[peak]))
def main():
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();source=a.input.resolve();out=a.output.resolve()
 files=[Path(__file__).resolve(),source/'report.json',source/'artifact_manifest.json',source/(CASE+'_history.npz')]
 bindings={str(p):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
 out.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),out/'extract_eight_findings.py')
 save(out/'registered_before_calculation.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),inputs=bindings,fixed_window_s=[98.5,102.5],recovery_exit_s=99.180,no_physics=True,no_control_change=True))
 r=json.loads((source/'report.json').read_text())['cases'][CASE]
 with np.load(source/(CASE+'_history.npz'),allow_pickle=False)as z:h={k:z[k].copy()for k in z.files}
 t=h['time_s'];mask=(t>=98.5-1e-9)&(t<=102.5+1e-9);rows=[]
 for i in np.flatnonzero(mask):
  rows.append(dict(time_s=t[i],frame=i,branch=int(h['branch_code'][i]),rank=int(h['native_rank'][i]),condition=h['native_condition_resolved'][i],singular_values=h['native_singular_values'][i],joint_reference=h['native_reference'][i,:3],belt_reference=h['native_reference'][i,3:],native_minimum_direction_angle_deg=h['native_minimum_acute_direction_angle_deg'][i],inferred_families=[FAMILIES[j]for j in np.flatnonzero(h['inferred_family_mask'][i])],last_solve_positive_pressure_families=[FAMILIES[j]for j in np.flatnonzero(h['truth_positive_family_mask'][i])],pressure_equivalent_point_error_m=h['estimated_to_pressure_equivalent_point_error_m'][i],raw_filtered_normal_angle_deg=h['raw_filtered_normal_angle_deg'][i],current_no_slip_tangent_rms_m_s=h['current_no_slip_tangent_rms_m_s'][i],truth_aggregate_reference=h['truth_aggregate_reference'][i],truth_aggregate_condition=h['truth_aggregate_condition_resolved'][i],same_family_point_available=h['same_family_point_oracle_available'][i],same_family_point_reference=h['same_family_point_reference'][i]))
 angular=np.max(abs(h['native_reference'][:,:3]),axis=1)
 stats={name:summary(t,x,mask)for name,x in [('resolved_condition',h['native_condition_resolved']),('max_absolute_joint_reference_rad_s',angular),('pressure_point_error_m',h['estimated_to_pressure_equivalent_point_error_m']),('raw_filtered_normal_angle_deg',h['raw_filtered_normal_angle_deg']),('current_no_slip_tangent_rms_m_s',h['current_no_slip_tangent_rms_m_s'])]}
 result=dict(iteration='I-77',case_id=CASE,reconstructed_rows=r['rows'],exact_reference_rows=r['reference_exact_rows'],window_s=[98.5,102.5],window_rows=len(rows),statistics=stats,
  branch_counts={name:int(np.sum(mask&(h['branch_code']==i)))for i,name in enumerate(('stationary_zero','current_record_allocation','dropout_held'))},
  instantaneous_positive_pressure_family_set_agreement_rows=int(np.sum(mask&h['instantaneous_positive_pressure_family_set_agreement'])),
  fixed_timeline_snapshots=r['snapshots'],every5ms_window=rows,
  no_physics=True,no_control_change=True,qualification='Pressurelabels/aggregatepoints are last-solve diagnostics, not full5mswrench truth; algebraic map substitutions are not physical interventions.')
 if any(sha(Path(p))!=v['sha256']for p,v in bindings.items()):raise ValueError('Findings inputs changed')
 save(out/'report.json',result)
 save(out/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in sorted(out.iterdir())if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps(dict(rows=len(rows),statistics=stats,branch_counts=result['branch_counts'])))
if __name__=='__main__':main()
