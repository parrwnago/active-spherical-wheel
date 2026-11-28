"""Compact I77 observational findings/figures, with bound source reports."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent;BASE=HERE.parent;OUT=HERE/'findings_stage01'
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
def main():
 files=[Path(__file__),HERE/'events_stage01/report.json',HERE/'refinement_stage01/report.json',HERE/'late_eight_stage02/report.json',HERE/'refinement_stage01/straight_m100_N_onset.png',HERE/'refinement_stage01/straight_m100_N_onset.pdf',HERE/'completed_stage01/eight_m100_N_timeline.npz',HERE/'refinement_stage01/eight_m100_N_refinement.npz',BASE/'kinematics/reconstruction_stage02/eight_m100_N_history.npz',BASE/'kinematics/findings_stage01/report.json',BASE/'kinematics/findings_stage02/report.json',HERE/'completed_stage01/report.json']
 bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
 OUT.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),OUT/'build_findings.py')
 write(OUT/'registered_before_calculation.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),files=bindings,no_new_physics=True,no_control_changes=True,figure='Eight fixed96.0s to selectedpeak+1s, now including last jointly recovered state and preceding target motion. Old later-window figure retained unchanged.'))
 events=json.loads(files[1].read_text());refine=json.loads(files[2].read_text());late=json.loads(files[3].read_text())
 with np.load(files[6],allow_pickle=False)as z:a={k:z[k].copy()for k in z.files}
 with np.load(files[7],allow_pickle=False)as z:d={k:z[k].copy()for k in z.files}
 with np.load(files[8],allow_pickle=False)as z:k={q:z[q].copy()for q in ('time_s','native_condition_resolved')}
 t=a['time_s'];np.testing.assert_array_equal(t,k['time_s']);low=96.;high=events['cases']['eight_m100_N']['events'][2]['time_s']+1.;mask=(t>=low-1e-9)&(t<=high+1e-9)
 fig,axes=plt.subplots(2,3,figsize=(15.5,8.2),constrained_layout=True);ax=axes.ravel();idx=np.flatnonzero(mask);anchor=a['reference_xy_m'][idx[0]];xy=a['actual_xy_m'][mask]-anchor;rr=a['reference_xy_m'][mask]-anchor
 ax[0].plot(rr[:,0],rr[:,1],'k--',label='Timed reference');ax[0].plot(xy[:,0],xy[:,1],label='Actual');ax[0].scatter(*xy[0],c='green',s=20,label='96s start');ax[0].scatter(*xy[-1],c='red',s=20,label='Window end');ax[0].set(title='Actual / reference XY',xlabel='World x from reference at96s [m]',ylabel='World y [m]');ax[0].axis('equal');ax[0].legend(fontsize=7)
 ax[1].plot(t[mask],1000*a['timed_error_m'][mask],label='Timed');ax[1].plot(t[mask],1000*a['geometric_error_m'][mask],label='Geometric');ax[1].set(ylabel='Position error [mm]',title='Near-path crossing does not mean velocity recovery');ax[1].legend(fontsize=7,loc='upper left')
 right=ax[1].twinx();right.plot(t[mask],1000*d['velocity_mismatch_m_s'][mask],c='green',alpha=.55,label='XY velocity mismatch');right.set_ylabel('XY velocity mismatch [mm/s]',color='green',fontsize=8)
 ax[2].plot(t[mask],np.max(abs(a['joint_rate_reference_rad_s'][mask]),axis=1),c='#b75c00',label='max |joint ref|');ax[2].set(ylabel='Joint reference [rad/s]',title='Early growth is well-conditioned / uncapped');ax[2].legend(fontsize=7,loc='upper left');right=ax[2].twinx();right.semilogy(t[mask],k['native_condition_resolved'][mask],c='gray',alpha=.7);right.set_ylabel('Native unscaled resolved condition',fontsize=8)
 ax[3].step(t[mask],a['requested_command_mixed'][mask,1],where='post',label='Requested');ax[3].step(t[mask],a['applied_command_mixed'][mask,1],where='post',label='Applied');ax[3].axhline(.6,c='gray',ls=':');ax[3].axhline(-.6,c='gray',ls=':');ax[3].set(ylabel='Shell A torque [N m]',title='Commands emitted AFTER each recorded state');ax[3].legend(fontsize=7)
 ax[4].plot(t[mask],a['absolute_tilt_deg'][mask],label='Body tilt');ax[4].plot(t[mask],d['reference_tilt_deg'][mask],label='Reference tilt');ax[4].plot(t[mask],a['normal_estimate_vs_DEM_proxy_deg'][mask],label='Estimate vs local DEM proxy');ax[4].set(ylabel='Degrees',title='Reference target moves before contact transfer');ax[4].legend(fontsize=7)
 ax[5].step(t[mask],a['estimated_dominant'][mask],where='post',label='Estimated dominant');ax[5].step(t[mask],a['pressure_dominant'][mask]+.07,where='post',alpha=.7,label='Last1ms pressure dominant');ax[5].set(yticks=[-1,0,1,2,3],yticklabels=['unknown','shell A','shell B','belt A','belt B'],title='Pressure labels are qualified, not full interval truth');ax[5].legend(fontsize=7)
 marks=[(96.150,'target motion'),(98.555,'belt A→shell A'),(99.175,'position-only recovery ends'),(100.040,'geo30'),(101.505,'max clip')]
 for i in range(1,6):
  ax[i].set(xlim=(low,high),xlabel='Saved simulation time [s]');ax[i].grid(alpha=.15)
  for tm,label in marks:ax[i].axvline(tm,c='gray',ls='--',alpha=.4,lw=.7)
 fig.suptitle('I-77 native eight−1: persistent motion from an earlier terrain/reference response\n96.150s targetup motion;98.555s transfer;99.175s low-position crossing ends;100.040s geo30;101.505s maxclip\nLast joint recovery96.035–96.160s still has body yaw rate≈−.30rad/s; chronology is not causal proof.',fontsize=11)
 fig.savefig(OUT/'eight_m100_N_precursor.png',dpi=160);fig.savefig(OUT/'eight_m100_N_precursor.pdf');plt.close(fig)
 for ext in ('png','pdf'):shutil.copy2(HERE/f'refinement_stage01/straight_m100_N_onset.{ext}',OUT/f'straight_m100_N_onset.{ext}')
 initials={cid:dict(initial_timed_mm=1000*r['initial']['timed_error_m'],moving_start_timed_mm=1000*r['moving_start']['timed_error_m'])for cid,r in events['cases'].items()}
 first=late['first_reference_up_rate_ge_1deg_s']
 report=dict(iteration='I-77',scope='Existing nine nativeN I76 recordings only; no heading intervention, tuning or new physics.',status='Observational source localization; physical initiating causes remain hypotheses.',input_bindings=bindings,initial_offsets=initials,straight=dict(geo30_start_s=21.040,timed50_start_s=21.250,geometric_peak_mm=148.93441607973237,geometric_peak_s=22.635,first_gt20rad_s_reference_s=20.430,max_clipping_event_s=20.585,geometric_error_at_maxclip_mm=11.63486187275268,instant_abs_cross_growth_at_maxclip_mm_s=-23.772894207251206,pre_pulse_trough='No interior minimum within fixed19.585–20.585s; error is still decreasing at the right boundary.',important_qualification='Attitude error and raw references already grow before maxpulse. Reference pulses precede largecross-track growth; later support/gain/normal changes can contribute. Timing alone is not causal proof.',kinematics_at_maxclip=dict(native_condition=571.82,acute_direction_deg=.389,point_comparator_mm=.140,pressure_point_condition=384.,pressure_point_reference_rad_s=[17.727,-61.675]),negative_control='Same-route+.9 is comparative, not zero-error:59.718mm peak; same-time condition≈3.074, directionangle≈86.21deg.'),eight=dict(last_position_recovered=json.loads((HERE/'completed_stage01/report.json').read_text())['late_eight']['recovered_interval'],last_joint_recovered=refine['cases']['eight_m100_N']['last_jointly_recovered_interval'],first_target_up_rate_ge1deg_s={key:first[key]for key in ('time_s','reference_up_rate_deg_s','actual_up_rate_deg_s','timed_mm','geo_mm','velocity_mismatch_mm_s','native_condition','estimated_family','pressure_family')},first_gain_change_s=float(late['gain_change_times_s'][0]),first_saved_DEM_proxy_step_ge_point1deg_s=late['all_proxy_normal_steps_ge_point1deg'][0]['time_s'],no_clipping_96_99p2=not late['any_clipping'],max_native_condition_96_99p2=late['maximum_native_condition'],geo30_start_s=100.040,timed50_start_s=100.705,max_clipping_s=101.505,geometric_peak_s=102.135,geometric_peak_mm=316.714,classification='Earlier target-normal/attitude and velocity dynamics, followed by transfer and larger excursion; not the straight weak-allocation-burst pattern at initial late growth.'),limits=['Saved ground normal is a float32 localDEM-nearest-centre tangent proxy, not DART per-contact normals.','Pressure family labels/points are last1ms positivepressure, not full previous5ms wrench or tangential force routing.','Correct family/close equivalentpoint do not exonerate all estimator geometry/normal/load assumptions.','Current-row command follows the recorded state; same-row association cannot establish it caused that state.','LQR algebraic block cancellation is not evidence of a gain bug; holding/support and preparation terms remain in total commands.'],missing_causal_evidence=['Same-prefix physical interventions that isolate rolling reference allocation in straight, before its first large pulses, with actualprefix verified.','For eight, isolate normal-to-reference/holding/rate paths or record aligned contactnormal/full5ms wrench data before calling filtered normal wrong.','Distinguish initiating growth from later handoff/preparation/clipping amplification.'],figures=['straight_m100_N_onset.png','eight_m100_N_precursor.png'],no_new_physics=True,no_control_changes=True)
 write(OUT/'report.json',report)
 md='''# I-77 baseline deviation: what the saved chronology establishes

Only original native N is analyzed. No heading helper, parameter tuning or new simulation is used here.

## Two different event patterns

**Straight −1:** the rolling map approaches a weak configuration while cross-track error is still closing. Large alternating reference pulses begin around20.430s; the largest normalized clipping event is20.585s. At that latter row geometric error is11.635mm and still decreasing, while attitude error is already growing. The sustained30mm crossing is21.040s and the148.934mm peak is22.635s. The exact reconstructed map is ill-conditioned even with the saved pressure-point comparator; this supports an allocation/configuration hypothesis, not a claim that every contact estimate is correct or that the maximum pulse alone initiates the event.

[Straight timeline](straight_m100_N_onset.png)

**Eight −1:** the last jointly low-position/low-velocity interval is96.035–96.160s under the declared diagnostic thresholds. Body yaw rate is already about−.30rad/s there. The first reference-up rate≥1deg/s marker is96.150s while the LQR angular-rate reference remains zero; at96.4s desired up-vector rate is84.32deg/s versus actual0.46deg/s, and velocity mismatch reaches82mm/s at96.5s. The rolling allocation is well conditioned and no command clips in96.0–99.2s. Later support transfers and the low-position crossing near99s occur with persistent velocity mismatch. The101.505s maximum clipping event follows, rather than defines the onset of, the late excursion.

[Eight precursor timeline](eight_m100_N_precursor.png)

The99s interval was only a position recovery: at99.175s error is about10mm but XY velocity mismatch is29.7mm/s. It must not be called a recovered physical state or an initiating disturbance. The later30mm crossing is100.040s; peak geometric error is316.714mm at102.135s.

## What is not established

- The local saved DEM normal is a nearest-centre tangent proxy. Its disagreement with a filtered force-derived normal does not prove the observer is wrong or delayed.
- Instantaneous positive-pressure family labels do not represent all forces over the observer's prior5ms interval. Correct labels and a close point do not rule out every geometry/load/normal issue.
- A current command is generated after the recorded state. Peak feedback can be a reaction; motor-reference conditioning, earlier attitude motion, handoff and preparation need separate causal tests.
- LQR component cancellation is algebraic. Total commands also include support/holding and preparation; cancellation alone is not a gain bug.

The next defensible evidence is a verified same-prefix intervention at one component, not an estimator replacement or event-specific retuning. Full timelines/context arrays remain in the local evidence bundle; the compact JSON records source bindings, exact event rules, qualifications and outstanding hypotheses.
'''
 (OUT/'findings.md').write_text(md)
 if not all(sha(ROOT/p)==v['sha256']for p,v in bindings.items()):raise ValueError('Frozen source/input changed')
 write(OUT/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in sorted(OUT.iterdir())if p.is_file()]))
 print(json.dumps(dict(findings=str(OUT),report_bytes=(OUT/'report.json').stat().st_size,markdown_bytes=(OUT/'findings.md').stat().st_size)))
if __name__=='__main__':main()
