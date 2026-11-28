"""I77 fixed 96.0–99.2s late-eight precursor table; no dynamics or tuning."""
from pathlib import Path
from datetime import datetime,timezone
import csv,hashlib,json,shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent;BASE=HERE.parent
DATA=ROOT/'results/tangent_heading_routes_20261127';OUT=HERE/'late_eight_stage02';CID='eight_m100_N'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def safe(x):
 if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.generic):return safe(x.item())
 if isinstance(x,float)and not np.isfinite(x):return None
 return x
def write(p,v):
 with p.open('x')as f:json.dump(safe(v),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def main():
 files=[Path(__file__),HERE/'completed_stage01'/f'{CID}_timeline.npz',HERE/'refinement_stage01'/f'{CID}_refinement.npz',HERE/'refinement_stage01/report.json',DATA/f'campaign/runs/{CID}/precision.npz',BASE/'kinematics/reconstruction_stage02'/f'{CID}_history.npz']
 bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
 OUT.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),OUT/'late_trace.py')
 write(OUT/'registered_before_array_reads.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),files=bindings,window_s=[96.,99.2],fixed_sample_step_s=.1,markers='Every savedDEMproxy step>=.1deg, every exactgainmatrixchange, supportsetchanges, and first referenceup-vector rate>=1deg/s within fixedwindow. Markers are diagnostics, not new performance gates.',no_new_physics=True,no_controller_changes=True))
 with np.load(files[1],allow_pickle=False)as z:a={k:z[k].copy()for k in z.files}
 with np.load(files[2],allow_pickle=False)as z:d={k:z[k].copy()for k in z.files}
 with np.load(files[4],allow_pickle=False)as z:K=z['exact_feedback_gain'];e=z['exact_error'];normal=z['exact_estimated_normal']
 with np.load(files[5],allow_pickle=False)as z:k={q:z[q].copy()for q in ('time_s','native_condition_resolved','native_reference','branch_code')}
 t=a['time_s'];np.testing.assert_array_equal(t,k['time_s']);mask=(t>=96-1e-9)&(t<=99.2+1e-9);idx=np.flatnonzero(mask)
 change=np.r_[False,np.any(K[1:]!=K[:-1],axis=(1,2))]
 def row(i):
  return dict(time_s=t[i],source_row=int(i),timed_mm=1000*a['timed_error_m'][i],geo_mm=1000*a['geometric_error_m'][i],velocity_mismatch_mm_s=1000*d['velocity_mismatch_m_s'][i],timed_error_growth_mm_s=1000*d['timed_error_growth_m_s'][i],geo_backward50ms_growth_mm_s=1000*d['geo_growth_backward50ms_m_s'][i],body_tilt_deg=a['absolute_tilt_deg'][i],reference_tilt_deg=d['reference_tilt_deg'][i],up_error_deg=a['reference_up_error_deg'][i],reference_up_rate_deg_s=d['reference_up_rate_deg_s'][i],actual_up_rate_deg_s=d['actual_up_rate_deg_s'][i],body_angular_rad_s=d['angular_velocity_body_rad_s'][i],angular_reference_rad_s=d['angular_reference_rad_s'][i],estimated_normal=normal[i],saved_DEM_proxy_normal=a['saved_ground_normal_float32'][i],normal_comparator_deg=a['normal_estimate_vs_DEM_proxy_deg'][i],proxy_step_deg=a['ground_normal_step_deg'][i],estimated_normal_step_deg=a['estimated_normal_step_deg'][i],estimated_family=int(a['estimated_dominant'][i]),pressure_family=int(a['pressure_dominant'][i]),estimated_mask=int(a['estimated_loadedset_bits'][i]),pressure_mask=int(a['pressure_loadedset_bits'][i]),gain_changed_thisrow=bool(change[i]),gain_command_delta=a['gain_change_contribution_mixed'][i],error_command_delta=a['error_change_contribution_mixed'][i],native_condition=k['native_condition_resolved'][i],native_branch=int(k['branch_code'][i]),joint_refs_rad_s=a['joint_rate_reference_rad_s'][i],joint_actual_rad_s=a['joint_rate_actual_rad_s'][i],actual_velocity_xy_m_s=a['actual_velocity_world_m_s'][i,:2],reference_velocity_xy_m_s=a['reference_velocity_xy_m_s'][i],requested=a['requested_command_mixed'][i],applied=a['applied_command_mixed'][i],lqr_total=a['term_lqr_command'][i],support=a['term_support_command'][i],rolling=a['term_rolling_feedforward'][i],prep_delta=a['term_transition_command_delta'][i],sync_weights=a['sync_weights'][i],fade=a['drive_fade'][i],shaft_LQR_blocks={key[len('lqr_'):]:v[i,0]for key,v in a.items()if key.startswith('lqr_')and v.ndim==2 and v.shape[1]==5},normalized_excess=a['normalized_requested_excess'][i],saved_instant_slip_proxy_m_s=a['saved_instantaneous_slip_speed_float32_m_s'][i])
 selected=[int(np.argmin(abs(t-s)))for s in np.linspace(96.,99.2,33)]
 timeseries=[row(i)for i in selected]
 def transitions(key):
  ids=idx[(idx>0)&(a[key][idx]!=a[key][np.maximum(idx-1,0)])]
  return [dict(time_s=t[i],before=int(a[key][i-1]),after=int(a[key][i]))for i in ids]
 normalsteps=idx[a['ground_normal_step_deg'][idx]>=.1];gainids=idx[change[idx]];target=idx[d['reference_up_rate_deg_s'][idx]>=1.]
 report=dict(iteration='I-77',case_id=CID,source_inputs_unchanged=True,window_s=[96.,99.2],sampled_rows=timeseries,all_proxy_normal_steps_ge_point1deg=[row(i)for i in normalsteps],gain_change_times_s=t[gainids],first_reference_up_rate_ge_1deg_s=None if not len(target)else row(target[0]),estimated_loadedset_changes=transitions('estimated_loadedset_bits'),pressure_loadedset_changes=transitions('pressure_loadedset_bits'),any_clipping=bool(np.any(a['any_requested_clipping'][idx])),maximum_reference_joint_abs_rad_s=np.max(abs(a['joint_rate_reference_rad_s'][idx]),axis=0),maximum_native_condition=float(np.nanmax(k['native_condition_resolved'][idx])),normal_qualification='Float32 localDEM-nearest-centre tangent proxy, not DART contact normal; anglecomparator is not observeraccuracy proof.',interpretation_limit='Chronology and exactalgebra do not prove an initiating mechanism. Lowposition/velocity interval does not imply zero bodyrotation.')
 if not all(sha(ROOT/p)==v['sha256']for p,v in bindings.items()):raise ValueError('Frozen source/input changed')
 write(OUT/'report.json',report)
 columns=['time_s','timed_mm','geo_mm','velocity_mismatch_mm_s','timed_error_growth_mm_s','geo_backward50ms_growth_mm_s','body_tilt_deg','reference_tilt_deg','up_error_deg','reference_up_rate_deg_s','actual_up_rate_deg_s','normal_comparator_deg','proxy_step_deg','estimated_normal_step_deg','estimated_family','pressure_family','gain_changed_thisrow','native_condition','native_branch']
 with (OUT/'fixed_100ms_table.csv').open('x',newline='')as f:
  writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader();writer.writerows({key:safe(r[key])for key in columns}for r in timeseries)
 write(OUT/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in sorted(OUT.iterdir())if p.is_file()]))
 print(json.dumps(safe({key:report[key]for key in ['first_reference_up_rate_ge_1deg_s','gain_change_times_s','estimated_loadedset_changes','pressure_loadedset_changes','any_clipping','maximum_native_condition']})))
if __name__=='__main__':main()
