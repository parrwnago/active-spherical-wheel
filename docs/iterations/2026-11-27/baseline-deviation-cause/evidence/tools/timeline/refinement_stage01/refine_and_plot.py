"""I77 fixed onset refinement and annotated baseline figures; no new physics."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,importlib.util,json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent;BASE=HERE.parent
DATA=ROOT/'results/tangent_heading_routes_20261127';INPUT=HERE/'completed_stage01';OUT=HERE/'refinement_stage01'
CASES=('straight_m100_N','straight_p090_N','eight_m100_N')
FIXED_STRAIGHT=(19.585,20.,20.2,20.3,20.335,20.4,20.425,20.430,20.435,20.480,20.485,20.525,20.555,20.560,20.585,20.600,20.605,20.610,20.635,20.7,21.04,21.25,22.635)
FIXED_EIGHT=(98.555,98.690,99.040,99.075,99.175,99.180,99.5,99.630,99.640,99.745,100.040,100.705,100.835,101.505,102.135)
NAMES=('shaft','shell A','shell B','belt A','belt B')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def safe(x):
 if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.generic):return safe(x.item())
 if isinstance(x,float)and not np.isfinite(x):return None
 return x
def write(p,v):
 with p.open('x')as f:json.dump(safe(v),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def last_interval(t,good,peak):
 groups=[];start=None
 for i in range(peak+1):
  okay=bool(good[i]and i<peak)
  if okay and(start is None or(i and abs(t[i]-t[i-1]-.005)>1e-9)):start=i
  if start is not None and(not okay or i==peak):
   end=i-1 if not okay else i
   if t[end]-t[start]>=.1-1e-9:groups.append((start,end))
   start=None
 return None if not groups else groups[-1]
def rows(t,mask):return np.flatnonzero(mask)
def snap(cid,a,d,i):
 return dict(source_row=int(i),time_s=float(a['time_s'][i]),timed_error_mm=1000*a['timed_error_m'][i],geometric_error_mm=1000*a['geometric_error_m'][i],velocity_mismatch_mm_s=1000*d['velocity_mismatch_m_s'][i],timed_error_growth_mm_s=1000*d['timed_error_growth_m_s'][i],geometric_backward50ms_growth_mm_s=1000*d['geo_growth_backward50ms_m_s'][i],cross_error_signed_mm=None if cid.startswith('eight')else 1000*d['signed_cross_m'][i],cross_error_instant_growth_mm_s=None if cid.startswith('eight')else 1000*d['signed_abs_cross_growth_m_s'][i],actual_velocity_xy_m_s=a['actual_velocity_world_m_s'][i,:2],reference_velocity_xy_m_s=a['reference_velocity_xy_m_s'][i],actual_angular_velocity_body_rad_s=d['angular_velocity_body_rad_s'][i],actual_up_vector_rate_deg_s=d['actual_up_rate_deg_s'][i],reference_up_vector_rate_backward5ms_deg_s=d['reference_up_rate_deg_s'][i],LQR_angular_reference_rad_s=d['angular_reference_rad_s'][i],reference_up_error_deg=a['reference_up_error_deg'][i],absolute_tilt_deg=a['absolute_tilt_deg'][i],reference_tilt_deg=d['reference_tilt_deg'][i],normal_estimate_vs_DEM_proxy_deg=a['normal_estimate_vs_DEM_proxy_deg'][i],ground_proxy_step_deg=a['ground_normal_step_deg'][i],estimated_normal_step_deg=a['estimated_normal_step_deg'][i],estimated_dominant=int(a['estimated_dominant'][i]),pressure_dominant=int(a['pressure_dominant'][i]),estimated_loadedset=int(a['estimated_loadedset_bits'][i]),pressure_loadedset=int(a['pressure_loadedset_bits'][i]),joint_reference_rad_s=a['joint_rate_reference_rad_s'][i],joint_actual_rad_s=a['joint_rate_actual_rad_s'][i],requested=a['requested_command_mixed'][i],applied=a['applied_command_mixed'][i],gain_change_effect=a['gain_change_contribution_mixed'][i],error_change_effect=a['error_change_contribution_mixed'][i],sync_weights=a['sync_weights'][i],transition_delta=a['term_transition_command_delta'][i],fade=a['drive_fade'][i],fade_delta=a['term_fade_command_delta'][i],instantaneous_slip_proxy_m_s=a['saved_instantaneous_slip_speed_float32_m_s'][i])
def main():
 files=[Path(__file__),INPUT/'report.json',INPUT/'artifact_manifest.json',DATA/'evaluation/metrics.py',DATA/'protocol/protocol.json']
 optional={}
 for cid in CASES:
  files += [INPUT/f'{cid}_timeline.npz',DATA/f'campaign/runs/{cid}/precision.npz']
  stage='reconstruction_stage02'if cid=='eight_m100_N'else'reconstruction_stage01'
  p=BASE/'kinematics'/stage/f'{cid}_history.npz'
  if p.exists():optional[cid]=p;files.append(p)
 bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
 OUT.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),OUT/'refine_and_plot.py')
 write(OUT/'registered_before_array_reads.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),files=bindings,no_new_physics=True,no_control_changes=True,refinements=dict(straight_window_s=[19.585,23.635],straight_pre_pulse_trough_search_s=[19.585,20.585],straight_fixed_sample_times_s=FIXED_STRAIGHT,eight_fixed_sample_times_s=FIXED_EIGHT,joint_recovery='Latest prepeak moving interval≥100ms with timed<=20mm,geometric<=10mm,worldXYvelocitymismatch<=10mm/s,rawreferenceuperror<=2deg; ifnone reportnone, no retuning.',plot_eight_window='Union of last low-position recovery±1s and latepeak±1s; latest jointly recovered interval reported even if outside this window.',derivatives='Exact norm derivative from recordedworldvelocityerror for timederror; straight signedcross derivative from worldvelocity projection; backward50ms geometry derivative, no future samples.',optional_condition_paths={k:str(v.relative_to(ROOT))for k,v in optional.items()})))
 spec=importlib.util.spec_from_file_location('_i77_refine_math',DATA/'evaluation/metrics.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 original=json.loads((INPUT/'report.json').read_text());arrays={};derived={};reports={};conditions={}
 for cid in CASES:
  with np.load(INPUT/f'{cid}_timeline.npz',allow_pickle=False)as z:a={k:z[k].copy()for k in z.files};arrays[cid]=a
  with np.load(DATA/f'campaign/runs/{cid}/precision.npz',allow_pickle=False)as z:q=z['exact_qpos'].copy();v=z['exact_qvel'].copy();qref=z['exact_reference_quat'].copy();vref=z['exact_feedback_reference_qvel'].copy()
  t=a['time_s'];offset=a['offset_xy_m'];velerr=a['velocity_error_world_xy_m_s'];norm=np.linalg.norm(offset,axis=1)
  growth=np.divide(np.sum(offset*velerr,axis=1),norm,out=np.zeros(len(t)),where=norm>1e-12)
  R=m.rotation(q[:,3:7]);RR=m.rotation(qref);up=R[:,:,2];refup=RR[:,:,2];omega=np.einsum('nij,nj->ni',R,v[:,3:6]);up_rate=np.linalg.norm(np.cross(omega,up),axis=1)
  refturn=np.r_[np.nan,np.degrees(np.arccos(np.clip(np.sum(refup[1:]*refup[:-1],axis=1),-1,1)))/.005]
  georate=np.full(len(t),np.nan);georate[10:]=(a['geometric_error_m'][10:]-a['geometric_error_m'][:-10])/.05
  angle=np.radians(113.);trans=np.array([-np.sin(angle),np.cos(angle)]);cross=offset@trans;crossrate=velerr@trans
  d=dict(velocity_mismatch_m_s=np.linalg.norm(velerr,axis=1),timed_error_growth_m_s=growth,geo_growth_backward50ms_m_s=georate,signed_cross_m=cross,signed_cross_rate_m_s=crossrate,signed_abs_cross_growth_m_s=np.sign(cross)*crossrate,actual_up_rate_deg_s=np.degrees(up_rate),reference_up_rate_deg_s=refturn,reference_tilt_deg=np.degrees(np.arccos(np.clip(refup[:,2],-1,1))),angular_velocity_body_rad_s=v[:,3:6],angular_reference_rad_s=vref[:,3:6]);derived[cid]=d
  if cid in optional:
   with np.load(optional[cid],allow_pickle=False)as z:
    np.testing.assert_array_equal(t,z['time_s']);conditions[cid]={k:z[k].copy()for k in ('native_condition_resolved','native_minimum_acute_direction_angle_deg','native_rank','branch_code')}
  query=FIXED_EIGHT if cid.startswith('eight')else FIXED_STRAIGHT
  report=dict(snapshots=[snap(cid,a,d,int(np.argmin(abs(t-s))))for s in query])
  if cid.startswith('straight'):
   ix=rows(t,(t>=19.585-1e-9)&(t<=20.585+1e-9));j=int(ix[np.argmin(a['geometric_error_m'][ix])]);report['minimum_geometric_in_pre_pulse_window']=dict(snapshot=snap(cid,a,d,j),at_window_boundary=bool(j in (ix[0],ix[-1])))
  else:
   peak=next(e['event']['source_row']for e in original['cases'][cid]['events']if e.get('event',{}).get('kind')=='largest_moving_geometric_peak')
   moving=(t>=3-1e-9)&(t<123-1e-9);good=moving&(a['timed_error_m']<=.020)&(a['geometric_error_m']<=.010)&(d['velocity_mismatch_m_s']<=.010)&(a['reference_up_error_deg']<=2.)
   recovered=last_interval(t,good,peak)
   report['last_jointly_recovered_interval']=None if recovered is None else dict(first_s=t[recovered[0]],last_s=t[recovered[1]],duration_s=t[recovered[1]]-t[recovered[0]],start=snap(cid,a,d,recovered[0]),end=snap(cid,a,d,recovered[1]),gap_from_last_jointly_recovered_end_to_peak_s=t[peak]-t[recovered[1]])
   report['no_joint_recovery_after_90s_before_peak']=bool(not np.any(good&(t>=90)&(np.arange(len(t))<peak)))
  reports[cid]=report;np.savez_compressed(OUT/f'{cid}_refinement.npz',time_s=t,**d)
 with PdfPages(OUT/'onset_timelines.pdf')as pdf:
  for cid in ('straight_m100_N','eight_m100_N'):
   a=arrays[cid];d=derived[cid];t=a['time_s'];is_eight=cid.startswith('eight');rec=original['late_eight']['recovered_interval']
   low,high=(float(rec['first_s'])-1, float(original['late_eight']['peak_time_s'])+1)if is_eight else(19.585,23.635)
   mask=(t>=low-1e-10)&(t<=high+1e-10);ix=np.flatnonzero(mask)
   held=ix[ix<len(t)-1];worst=int(held[np.argmax(a['normalized_requested_excess'][held])]);channel=int(np.argmax(abs(a['requested_command_mixed'][worst]-a['applied_command_mixed'][worst])/a['input_bounds_mixed'][worst]))
   marks=[(99.175,'pos recovery ends'),(100.040,'geo30'),(100.705,'timed50'),(101.505,'maxclip'),(102.135,'geo peak')]if is_eight else[(20.430,'first >20rad/s'),(20.585,'maxclip'),(21.040,'geo30'),(22.635,'geo peak')]
   fig,ax=plt.subplots(2,3,figsize=(15.5,8.2),constrained_layout=True);axes=ax.ravel();anchor=a['reference_xy_m'][ix[0]]
   xy=a['actual_xy_m'][ix]-anchor;ref=a['reference_xy_m'][ix]-anchor
   axes[0].plot(ref[:,0],ref[:,1],'k--',label='Timed reference');axes[0].plot(xy[:,0],xy[:,1],color='#0072b2',label='Actual');axes[0].scatter(*xy[0],c='green',s=24,label='Window start');axes[0].scatter(*xy[-1],c='red',s=24,label='Window end');axes[0].set(xlabel='World x from reference at window start [m]',ylabel='World y [m]',title='Saved actual / reference XY');axes[0].axis('equal');axes[0].legend(fontsize=7)
   axes[1].plot(t[mask],1000*a['timed_error_m'][mask],label='Timed error');axes[1].plot(t[mask],1000*a['geometric_error_m'][mask],label='Geometric error')
   if not is_eight:
    c=arrays['straight_p090_N'];cm=(c['time_s']>=low)&(c['time_s']<=high);axes[1].plot(c['time_s'][cm],1000*c['geometric_error_m'][cm],':',c='gray',label='+.9 control geo')
   axes[1].set(ylabel='Position error [mm]',title='Thresholds follow the initial dynamics');axes[1].legend(fontsize=7)
   axes[2].plot(t[mask],np.max(abs(a['joint_rate_reference_rad_s'][mask]),axis=1),color='#b75c00',label='max |joint ref|');axes[2].set(ylabel='Angular joint reference [rad/s]',title='Recorded rolling references')
   if cid in conditions:
    right=axes[2].twinx();right.semilogy(t[mask],conditions[cid]['native_condition_resolved'][mask],color='#777777',alpha=.7,label='native condition');right.set_ylabel('Native unscaled resolved condition',fontsize=8)
   axes[2].legend(fontsize=7,loc='upper left')
   axes[3].step(t[mask],a['requested_command_mixed'][mask,channel],where='post',label='Requested');axes[3].step(t[mask],a['applied_command_mixed'][mask,channel],where='post',label='Applied');cap=a['input_bounds_mixed'][0,channel];axes[3].axhline(cap,c='gray',ls=':');axes[3].axhline(-cap,c='gray',ls=':');axes[3].set(ylabel='N m'if channel<3 else'm/s',title=f'{NAMES[channel]}: emitted command, acts AFTER state');axes[3].legend(fontsize=7)
   axes[4].plot(t[mask],a['absolute_tilt_deg'][mask],label='Body tilt');axes[4].plot(t[mask],d['reference_tilt_deg'][mask],label='Reference tilt');axes[4].plot(t[mask],a['normal_estimate_vs_DEM_proxy_deg'][mask],label='Normal estimate vs DEM proxy');axes[4].set(ylabel='Degrees',title='Attitude target / qualified normal comparator');axes[4].legend(fontsize=7)
   axes[5].step(t[mask],a['estimated_dominant'][mask],where='post',label='Estimated dominant');axes[5].step(t[mask],a['pressure_dominant'][mask]+.07,where='post',alpha=.7,label='Last1ms pressure dominant');axes[5].set(yticks=[-1,0,1,2,3],yticklabels=['unknown','shell A','shell B','belt A','belt B'],title='Pressure labels are not full interval truth');axes[5].legend(fontsize=7)
   for k in range(1,6):
    axes[k].set(xlim=(low,high),xlabel='Saved simulation time [s]');axes[k].grid(alpha=.15)
    for mt,label in marks:
     if low<=mt<=high:axes[k].axvline(mt,color='#555555',alpha=.35,ls='--',lw=.7)
   labeltext='; '.join(f'{mt:g}s {label}'for mt,label in marks)
   joint=reports[cid].get('last_jointly_recovered_interval');note=('No jointly recovered interval'if joint is None else f'Last jointly recovered: {joint["first_s"]:.3f}–{joint["last_s"]:.3f}s')if is_eight else'At-row state precedes the command emitted on that row; timing alone is not causality.'
   fig.suptitle(f'I-77 native {cid}: recorded chronology, no controller change\n{labeltext}\n{note}',fontsize=12)
   fig.savefig(OUT/f'{cid}_onset.png',dpi=160);fig.savefig(OUT/f'{cid}_onset.pdf');pdf.savefig(fig);plt.close(fig)
 if not all(sha(ROOT/p)==v['sha256']for p,v in bindings.items()):raise ValueError('Frozen source/input changed')
 write(OUT/'report.json',dict(iteration='I-77',cases=reports,source_inputs_unchanged=True,no_new_physics=True,no_control_changes=True,derivative_timing='Instantaneous recorded velocity projections or backward differences only; no future-state derivative.',figures=['straight_m100_N_onset.png','eight_m100_N_onset.png'],causality_limit='Observed state/command ordering and algebraic decomposition cannot establish the complete initiating physical cause.'))
 write(OUT/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in sorted(OUT.iterdir())if p.is_file()]))
 print(json.dumps(safe(dict(straight_pre_pulse=reports['straight_m100_N']['minimum_geometric_in_pre_pulse_window'],eight_joint_recovery=reports['eight_m100_N']['last_jointly_recovered_interval'],eight_no_joint_after90=reports['eight_m100_N']['no_joint_recovery_after_90s_before_peak']))))
if __name__=='__main__':main()
