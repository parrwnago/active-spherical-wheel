"""I77 measured velocity-error identity from sealed state/kinematic records.

v-vdes = material_residual - base_rotation - A*(drive_actual-drive_ref)
         - (A*drive_ref+vdes).
All are correlated kinematic terms, not independent forces or causal effects.
Actual belt material motion uses the previous held applied input. No new
controller calls, model integration or solver/terrain queries occur.
"""
from datetime import datetime,timezone
from importlib.metadata import version
from pathlib import Path
import argparse,hashlib,json,shutil
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
OLD=ROOT/'results/tangent_heading_routes_20261127'
CASE='eight_m100_N'
WINDOWS={'chronology_96_to99_2':[96.,99.2],'late_initiation_98_5_to100_8':[98.5,100.8]}
TIMES=[96.,96.5,97.,97.5,98.,98.5,98.690,99.,99.175,99.180,99.5,100.040,100.705,100.8,101.,101.505,102.135]
TERMS=('material_residual','negative_base_rotation','negative_drive_tracking_effect','negative_allocation_residual')
FAMILIES=('shell_A','shell_B','belt_A','belt_B')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,float)and not np.isfinite(v):return None
 return v
def write(p,v):
 with p.open('x')as f:json.dump(safe(v),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def stats(x):
 x=np.asarray(x);finite=x[np.isfinite(x)]
 return dict(samples=len(finite),minimum=float(np.min(finite))if len(finite)else None,maximum=float(np.max(finite))if len(finite)else None,rms=float(np.sqrt(np.mean(finite**2)))if len(finite)else None)
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();out=args.output.resolve()
 reconstruction=BASE/'kinematics/reconstruction_stage02'
 files=[Path(__file__).resolve(),OLD/'protocol/protocol.json',OLD/'campaign/outputs_sealed.json',OLD/'campaign/runs'/CASE/'precision.npz',OLD/'campaign/runs'/CASE/'provenance.json',reconstruction/'report.json',reconstruction/'artifact_manifest.json',reconstruction/(CASE+'_history.npz'),BASE/'timeline/completed_stage01/report.json',BASE/'timeline/completed_stage01/artifact_manifest.json']
 bindings={str(p):dict(bytes=p.stat().st_size,sha256=sha(p))for p in files}
 out.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),out/'velocity_decomposition.py')
 write(out/'registered_before_array_reads.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),files=bindings,case_id=CASE,windows=WINDOWS,fixed_times_s=TIMES,
 identity='v_actual-v_des=material_residual-base_rotation-A@(drive_actual-drive_ref)-(A@drive_ref+v_des)',
 actual_belt_boundary='previous actually applied input at saved measurement; next command acts after this state',
 projection='WorldXY, matching original native allocator. Material residual is independently checked against the previously reconstructed MuJoCo fullpoint Jacobian XY residual norm.',
 weights='Per-contact and current inferred-loadweighted means; retain RMS to expose opposing residuals. Native allocation itself is unweighted.',
 no_physics=True,no_controller_calls=True,no_solver_or_terrain_queries=True,no_causal_force_claim=True))
 provenance=json.loads((OLD/'campaign/runs'/CASE/'provenance.json').read_text())
 for package in ('numpy','mujoco'):
  if version(package)!=provenance['versions'][package]:raise ValueError('Use original numerical library versions')
 with np.load(OLD/'campaign/runs'/CASE/'precision.npz',allow_pickle=False)as z:
  data={key:z[key].copy()for key in ['time_s','exact_qpos','exact_qvel','exact_previous_applied','exact_reference_velocity_m_s','exact_reference_xy','exact_reference_qvel','exact_allocation_drive_reference']}
 with np.load(reconstruction/(CASE+'_history.npz'),allow_pickle=False)as z:
  h={key:z[key].copy()for key in ['time_s','record_count','native_matrix','native_reference','native_family_loads_N','native_family_points_m','current_no_slip_xy_rms_m_s','measured_base_rotation_contact_xy_rms_m_s','branch_code']}
 if not np.array_equal(data['time_s'],h['time_s'])or not np.array_equal(data['exact_allocation_drive_reference'],h['native_reference']):raise ValueError('Raw/reconstructed clocks or references differ')
 t=data['time_s'];n=len(t);m=4
 arrays=dict(time_s=t,valid=np.zeros(n,bool),record_count=h['record_count'],actual_body_velocity_xy=data['exact_qvel'][:,:2],desired_velocity_xy=data['exact_reference_velocity_m_s'],velocity_error_xy=data['exact_qvel'][:,:2]-data['exact_reference_velocity_m_s'],actual_drive=np.c_[data['exact_qvel'][:,6:9],data['exact_previous_applied'][:,3:]],reference_drive=h['native_reference'],contact_family=np.full((n,m),-1,int),contact_weight=np.zeros((n,m)),contact_base_rotation_by_body_axis_xy=np.full((n,m,3,2),np.nan),contact_drive_tracking_by_drive_xy=np.full((n,m,5,2),np.nan),material_xy_rms=np.full(n,np.nan),base_rotation_xy_rms=np.full(n,np.nan),row_identity_error=np.full(n,np.nan))
 for term in TERMS:
  arrays['per_contact_'+term+'_xy']=np.full((n,m,2),np.nan);arrays['weighted_'+term+'_xy']=np.full((n,2),np.nan)
 arrays['weighted_identity_rhs_xy']=np.full((n,2),np.nan)
 for i in range(n):
  count=int(h['record_count'][i]);families=np.flatnonzero(h['native_family_loads_N'][i]>0.)
  if not count:continue
  if len(families)!=count:raise ValueError('Per-family point ordering cannot represent records')
  rotation_flat=np.empty(9);mujoco.mju_quat2Mat(rotation_flat,data['exact_qpos'][i,3:7]);rotation=rotation_flat.reshape(3,3)
  omega=rotation@data['exact_qvel'][i,3:6]
  weights=h['native_family_loads_N'][i,families];weights=weights/weights.sum()
  arrays['contact_family'][i,:count]=families;arrays['contact_weight'][i,:count]=weights
  material=[];base=[]
  for j,family in enumerate(families):
   a=h['native_matrix'][i,2*j:2*j+2];lever=h['native_family_points_m'][i,family]-data['exact_qpos'][i,:3]
   rot=np.cross(omega,lever)[:2]
   by_axis=np.array([np.cross(rotation[:,k]*data['exact_qvel'][i,3+k],lever)[:2]for k in range(3)])
   actual=arrays['actual_drive'][i];reference=arrays['reference_drive'][i]
   drive_by_axis=np.array([a[:,k]*(actual[k]-reference[k])for k in range(5)])
   mismatch=a@(actual-reference);allocation=a@reference+data['exact_reference_velocity_m_s'][i]
   slip=data['exact_qvel'][i,:2]+rot+a@actual
   terms=dict(material_residual=slip,negative_base_rotation=-rot,negative_drive_tracking_effect=-mismatch,negative_allocation_residual=-allocation)
   arrays['contact_base_rotation_by_body_axis_xy'][i,j]=by_axis
   arrays['contact_drive_tracking_by_drive_xy'][i,j]=drive_by_axis
   for key,value in terms.items():arrays['per_contact_'+key+'_xy'][i,j]=value
   material.append(slip);base.append(rot)
  for term in TERMS:arrays['weighted_'+term+'_xy'][i]=weights@arrays['per_contact_'+term+'_xy'][i,:count]
  rhs=sum(arrays['weighted_'+key+'_xy'][i]for key in TERMS)
  arrays['weighted_identity_rhs_xy'][i]=rhs;arrays['row_identity_error'][i]=np.max(abs(rhs-arrays['velocity_error_xy'][i]))
  arrays['material_xy_rms'][i]=np.sqrt(np.mean(np.sum(np.asarray(material)**2,axis=1)))
  arrays['base_rotation_xy_rms'][i]=np.sqrt(np.mean(np.sum(np.asarray(base)**2,axis=1)))
  arrays['valid'][i]=True
 valid=arrays['valid'];identity_max=float(np.max(arrays['row_identity_error'][valid]))
 material_error=float(np.max(abs(arrays['material_xy_rms'][valid]-h['current_no_slip_xy_rms_m_s'][valid])))
 rotation_error=float(np.max(abs(arrays['base_rotation_xy_rms'][valid]-h['measured_base_rotation_contact_xy_rms_m_s'][valid])))
 if max(identity_max,material_error,rotation_error)>1e-10:raise AssertionError('Independent Jacobian/velocity identity failed')
 snapshots={}
 for time in TIMES:
  i=int(np.argmin(abs(t-time)))
  if abs(t[i]-time)>1e-9:raise ValueError('Fixed time missing')
  snapshots[f'{time:.3f}']=dict(time_s=t[i],source_frame=i,valid=bool(valid[i]),families=[FAMILIES[k]for k in arrays['contact_family'][i]if k>=0],weights=arrays['contact_weight'][i],actual_body_velocity_xy=arrays['actual_body_velocity_xy'][i],desired_velocity_xy=arrays['desired_velocity_xy'][i],velocity_error_xy=arrays['velocity_error_xy'][i],actual_drive=arrays['actual_drive'][i],reference_drive=arrays['reference_drive'][i],weighted_terms_xy={term:arrays['weighted_'+term+'_xy'][i]for term in TERMS},per_contact_terms_xy={term:arrays['per_contact_'+term+'_xy'][i]for term in TERMS},body_rotation_by_body_axis_xy=arrays['contact_base_rotation_by_body_axis_xy'][i],drive_tracking_by_drive_xy=arrays['contact_drive_tracking_by_drive_xy'][i],identity_error=arrays['row_identity_error'][i])
 windows={}
 position_error=data['exact_qpos'][:,:2]-data['exact_reference_xy']
 for name,(lo,hi)in WINDOWS.items():
  ids=np.flatnonzero((t>=lo-1e-9)&(t<=hi+1e-9));good=ids[valid[ids]]
  if len(good)!=len(ids):raise ValueError('Selected fixed window has unavailable current contacts')
  weighted={term:arrays['weighted_'+term+'_xy'][ids]for term in TERMS}
  windows[name]=dict(window_s=[lo,hi],state_samples=len(ids),all_valid=True,actual_position_error_change_xy=position_error[ids[-1]]-position_error[ids[0]],trapezoidal_integral_velocity_error_xy=np.trapz(arrays['velocity_error_xy'][ids],t[ids],axis=0),trapezoidal_integral_terms_xy={key:np.trapz(value,t[ids],axis=0)for key,value in weighted.items()},position_velocity_quadrature_defect_xy=(position_error[ids[-1]]-position_error[ids[0]])-np.trapz(arrays['velocity_error_xy'][ids],t[ids],axis=0),arithmetic_mean_velocity_error_xy=np.mean(arrays['velocity_error_xy'][ids],axis=0),arithmetic_mean_terms_xy={key:np.mean(value,axis=0)for key,value in weighted.items()},vector_norm_rms_m_s={key:stats(np.linalg.norm(value,axis=1))for key,value in weighted.items()},term_interpretation='Correlated velocity-accounting terms; sums and integralclosure are identities, not independently caused position changes or forces.')
 np.savez_compressed(out/'eight_m100_N_velocity_decomposition.npz',**arrays)
 unchanged=all(Path(p).stat().st_size==v['bytes']and sha(p)==v['sha256']for p,v in bindings.items())
 if not unchanged:raise ValueError('Decomposition source/input changed')
 write(out/'report.json',dict(iteration='I-77',case_id=CASE,rows=n,valid_current_contact_rows=int(valid.sum()),identity_max_absolute_error_m_s=identity_max,independent_previous_StateMirror_material_rms_max_error_m_s=material_error,independent_previous_StateMirror_rotation_rms_max_error_m_s=rotation_error,all_declared_body_angular_references_zero=bool(np.all(data['exact_reference_qvel'][:,3:6]==0.)),snapshots=snapshots,windows=windows,no_new_physics=True,no_control_change=True,no_causal_force_claim=True,source_input_unchanged=True,scope='Pure WorldXY kinematic accounting. Bodyangular contributions and actuatortracking terms are correlated; no claim of force/causal independence or a tested rotation-aware controller.'))
 write(out/'artifact_manifest.json',dict(files=[dict(path=p.name,bytes=p.stat().st_size,sha256=sha(p))for p in sorted(out.iterdir())if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps(dict(identity_max=identity_max,material_independent_error=material_error,rotation_independent_error=rotation_error,windows=windows),default=lambda x:x.tolist()if isinstance(x,np.ndarray)else x))
if __name__=='__main__':main()
