"""Bounded independent I50 interpretation/arithmetic review; saved files only."""
from pathlib import Path
import hashlib
import json
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
FAMILY=ROOT/'results/belt_handoff_event_20261126'
I49=ROOT/'results/belt_capacity_sweep_20261126'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def load(case):
    with np.load(I49/'campaign/runs'/case/'precision.npz')as z:return {k:z[k].copy()for k in z.files}
def sample(p,t):
    i=int(np.argmin(abs(p['time_s']-t)));K=p['exact_feedback_gain'][i];e=p['exact_feedback_error'][i]
    lqr=-K@e;joint=-K[:,15:18]@e[15:18]
    return dict(index=i,time_s=float(p['time_s'][i]),drive_reference=p['exact_allocation_drive_reference'][i].tolist(),
        requested=p['exact_native_requested'][i].tolist(),applied=p['exact_applied'][i].tolist(),
        lqr=lqr.tolist(),joint_velocity_lqr=joint.tolist(),other_lqr_groups=(lqr-joint).tolist(),
        estimated_loads_N=p['exact_estimated_support_loads_N'][i].tolist(),truth_loads_N=p['truth_patch_loads_N'][i].tolist(),
        sync_weights=p['exact_sync_weights'][i].tolist(),transition=p['exact_transition_command_delta'][i].tolist(),
        rolling_feedforward=p['exact_rolling_feedforward'][i].tolist(),
        timed_error_mm=float(1000*np.linalg.norm(p['exact_qpos'][i,:2]-p['exact_reference_xy'][i])),
        final_goal_error_mm=float(1000*np.linalg.norm(p['exact_qpos'][i,:2]-p['exact_reference_xy'][-1])))

def main():
    target=HERE/'independent_review.json'
    if target.exists():raise FileExistsError('Preserve previous review')
    paths=[Path(__file__).resolve(),FAMILY/'sync_audit/stage05_rolling_geometry/report.json',FAMILY/'sync_audit/audit_rolling_geometry.py',FAMILY/'deviation_audit/stage01/report.json',FAMILY/'deviation_audit/analyze_deviation.py',FAMILY/'video_audit/artifact_manifest.json',FAMILY/'video_audit/qa.json',ROOT/'src/spherical_wheel/experiments.py',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/synchronization.py']
    paths += [I49/'campaign/runs'/case/'precision.npz'for case in ('mars_cap030','mars_cap060')]
    inputs={str(p.relative_to(ROOT)):sha(p)for p in paths}
    geometry=read(paths[1]);deviation=read(paths[3]);base=load('mars_cap030');raised=load('mars_cap060')
    b0=sample(base,31.645);b1=sample(base,31.650);b2=sample(base,31.655)
    r0=sample(raised,32.165);r1=sample(raised,32.170);r2=sample(raised,32.185);r3=sample(raised,32.190)
    late0=sample(base,40.625);late1=sample(base,43.985)
    g=next(row for row in geometry['cases']['mars_cap030']if row['index']==b1['index'])
    matrix=np.asarray(g['D_world_xy']);scaled=matrix*np.array([5.,5.]);sigma=np.linalg.svd(scaled,compute_uv=False)
    solved=np.linalg.solve(scaled,np.asarray(g['rhs_m_s']))*5.
    errors=[]
    for case,p,points in [('mars_cap030',base,[b1]),('mars_cap060',raised,[r0,r1,r2,r3])]:
        for s in points:
            candidates=deviation['cases'][case]['fixed_and_change_snapshots'];d=next(row for row in candidates if row['index']==s['index'])
            errors.append(float(np.max(abs(np.asarray(d['lqr_groups']['joint_velocity'])-np.asarray(s['joint_velocity_lqr'])))))
    checks={
        'baseline_raw_drive_rates_match_geometry_report':np.allclose(b1['drive_reference'],g['saved_drive_references'],rtol=0,atol=1e-10),
        'baseline_normalized_matrix_singular_values_match':np.allclose(sigma,g['singular_values_native_normalized_m_s'],rtol=0,atol=1e-12),
        'baseline_matrix_solution_reconstructs_rates':np.max(abs(solved-np.asarray(b1['drive_reference'])[[0,2]]))<1e-7,
        'baseline_joint_velocity_error_is_measured_minus_reference':np.allclose(base['exact_feedback_error'][b1['index'],15:18],base['exact_qvel'][b1['index'],6:9]-base['exact_feedback_reference_qvel'][b1['index'],6:9],rtol=0,atol=1e-10),
        'baseline_lqr_matches_saved_minus_K_error':np.allclose(b1['lqr'],base['exact_lqr_command'][b1['index']],rtol=0,atol=1e-10),
        'joint_velocity_decomposition_matches_selected_report_samples':max(errors)<1e-10,
        'baseline_sync_zero_at_singular_sample':not np.any(b1['sync_weights'])and not np.any(b1['transition']),
        'baseline_all3_applied_torques_reverse_next5ms':np.allclose(b1['applied'][:3],[1.5,-.6,-.6],rtol=0,atol=1e-12)and np.allclose(b2['applied'][:3],[-1.5,.6,.6],rtol=0,atol=1e-12),
        'raised_first_loaded_belt_command_is_lqr_plus_rolling':abs(r1['requested'][4]-r1['lqr'][4]-r1['rolling_feedforward'][4])<1e-12 and r0['sync_weights'][4]==1 and r1['sync_weights'][4]==0,
        'baseline_terminal_error_matches_separate_late_report_sample':abs(late1['timed_error_mm']-1000*deviation['cases']['mars_cap030']['fulltrace_max_timed']['timed_error_m'])<1e-8,
    }
    result={'iteration':'I-50','passed':bool(all(checks.values())),'scope':__doc__,'checks':{k:bool(v)for k,v in checks.items()},'input_sha256':inputs,
        'baseline_singular_sample':dict(before=b0,event=b1,next=b2,normalized_singular_values_m_s=sigma.tolist(),condition_number=float(sigma[0]/sigma[-1]),acute_drive_column_angle_deg=g['acute_drive_direction_angle_deg'],independent_reconstructed_shaft_shellB_rates_rad_s=solved.tolist()),
        'raised_handoff_samples':[r0,r1,r2,r3],'late_samples':[late0,late1],
        'findings':[
            'Confirmed command origin at baseline31.650s: near-collinear shell-contact rolling columns produce huge desired joint rates. Joint-velocity LQR contributions dominate the requested torque, then the physical torque clip bounds allthree motor torques. The raw drive-rate numbers are not actual joint velocities or torque commands.',
            'Incoming synchronization is zero at that singular sample. This does not make the event unrelated to contact switching: observer and truth say belt_B at31.645 and shell_B at31.650, and prior belt blips exist. It is incorrect to say the singular sample predates every shell/belt handoff.',
            'Raised32.170s is a distinct source mechanism: incoming-belt preparation ceases when belt_B is classified loaded, and the belt LQR channel activates. Its speed request is−0.319806 LQR+0.016445 rolling=−0.303361m/s, from the prior−0.012112 prep request. The subsequent32.190 return to shell support removes belt feedback and returns the belt command tozero.',
            'The190.901mm baseline maximum at43.985s is a separate terminal excursion. Baseline already reaches50.096mm timed error at40.625s while both truth/observer are shell_B, before the later~43s belt activity. This trace identifies command origins and chronology, not the sole physical cause of later drift or the counterfactual effect of removing one earlier event.'
        ],
        'source_references':[
            {'path':'src/spherical_wheel/experiments.py','lines':'29–44,79–135','meaning':'Normalized least-squares rolling allocation and joint-rate versus belt-speed output split; scales are preferences, not physical limits.'},
            {'path':'src/spherical_wheel/offroad_control.py','lines':'596–626,757–816','meaning':'Native least-squares reference selection and terrain-feedback command ueq+feedforward+support−Kerror.'},
            {'path':'src/spherical_wheel/synchronization.py','lines':'211–255','meaning':'Incoming unloaded-fraction preparation weights and target construction; a blend weight is not achieved speed matching.'}
        ],
        'qualifications':['Normalized singular values use5rad/s shaft/shell preferences; D physical columns are m/rad, not torque-to-velocity gains.',
            'Current recorded state precedes application of the command computed at that timestamp; same-tick state/command association alone is not a measured response to that command.',
            'Late deviation may carry prior history; this review neither proves nor rules out influence from earlier events. It rejects sole-cause claims unsupported by a controlled counterfactual.',
            'The sealed slow-replay phrase before the principal belt-supported phase must not be paraphrased as before any contact switch.',
            'This review_after_seal subdirectory was added after video_audit/artifact_manifest.json. That earlier manifest remains immutable and does not cover this new review; this report provides its own source/input hashes.'],
        'no_new_physics_or_controller':True,'no_old_artifact_changes':True,'blocking_scientific_findings':[]}
    assert inputs=={str(p.relative_to(ROOT)):sha(p)for p in paths}
    target.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'passed':result['passed'],'checks':len(checks),'matrix_condition':result['baseline_singular_sample']['condition_number'],'report_sha256':sha(target)},indent=2))
    if not result['passed']:raise AssertionError('Preserve failed review')

if __name__=='__main__':main()
