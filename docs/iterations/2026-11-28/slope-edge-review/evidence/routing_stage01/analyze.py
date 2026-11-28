"""Current-vs-interval contact routing ledger; no causal or new-physics claim."""
from pathlib import Path
import hashlib
import json
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
RUN=ROOT/'results/tangent_heading_routes_20261127/campaign/runs/eight_m100_N'
NAMES=('shell_A','shell_B','belt_A','belt_B')


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def groups(mask,t):
    ii=np.flatnonzero(mask)
    if not len(ii):return []
    gs=np.split(ii,np.flatnonzero(np.diff(ii)>1)+1)
    return [dict(first_s=float(t[g[0]]),last_s=float(t[g[-1]]),rows=int(len(g))) for g in gs]


def labels(load):
    result=np.argmax(load,axis=1)
    return np.where(np.sum(load,axis=1)>1e-9,result,-1)


def main():
    receipt=OUT/'registered_before_analysis.json'
    if receipt.exists():raise RuntimeError('Refuse to overwrite closed routing stage')
    p=RUN/'precision.npz';binding=dict(iteration='I-80',source_sha256=sha(Path(__file__)),
        inputs=[dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=sha(p))],
        scope='Retrospective99–103s dominant currentpositivepressure vs filtered inferredload routing. Not time-aligned ground-truth classification error or newphysics.',
        snapshot_times_s=[100.835,101.285,101.490,101.495,101.500,101.505,101.510,101.515],
        no_new_physics=True)
    receipt.write_text(json.dumps(binding,indent=2)+'\n')
    with np.load(p,allow_pickle=False) as z:d={k:z[k].copy() for k in z.files}
    t=d['time_s'];m=(t>=99.-1e-8)&(t<=103.+1e-8);tt=t[m]
    truth=d['truth_patch_loads_N']; est=d['exact_estimated_support_loads_N']
    tl=labels(truth);el=labels(est)
    pd=np.linalg.norm(d['exact_estimated_contact_point_m']-d['truth_equivalent_point_m'],axis=1)
    rows=[]
    for target in binding['snapshot_times_s']:
        i=int(np.argmin(abs(t-target)));assert abs(t[i]-target)<1e-8
        requested=d['exact_requested'][i];applied=d['exact_applied'][i];bounds=d['exact_input_bounds'][i]
        rows.append(dict(time_s=float(t[i]),pressure_loads_N=truth[i].tolist(),inferred_loads_N=est[i].tolist(),
            pressure_dominant_family=NAMES[tl[i]] if tl[i]>=0 else 'none',
            inferred_dominant_family=NAMES[el[i]] if el[i]>=0 else 'none',
            current_point_disagreement_m=float(pd[i]),
            current_truth_point_m=d['truth_equivalent_point_m'][i].tolist(),
            current_inferred_point_m=d['exact_estimated_contact_point_m'][i].tolist(),
            observer_wrench=d['exact_observer_wrench'][i].tolist(),
            observer_interval_mean_generalized=d['exact_observer_interval_mean'][i].tolist(),
            inferred_normal=d['exact_estimated_normal'][i].tolist(),
            sync_weights=d['exact_sync_weights'][i].tolist(),
            preparation_delta=d['exact_transition_command_delta'][i].tolist(),
            fade_delta=d['exact_fade_command_delta'][i].tolist(),
            feedback_reference_joint_rates=d['exact_feedback_reference_qvel'][i,6:9].tolist(),
            native_allocation_drive_reference=d['exact_allocation_drive_reference'][i].tolist(),
            lqr_command=d['exact_lqr_command'][i].tolist(),
            rolling_feedforward=d['exact_rolling_feedforward'][i].tolist(),
            support_feedforward=d['exact_support_command'][i].tolist(),
            equilibrium_command=d['exact_equilibrium_command'][i].tolist(),
            requested_command=requested.tolist(),applied_command=applied.tolist(),
            previous_applied_command=d['exact_previous_applied'][i].tolist(),
            clipped_channels=(abs(requested)>bounds+1e-12).tolist(),
            gain_row_norms=np.linalg.norm(d['exact_feedback_gain'][i],axis=1).tolist(),
            gain_change_from_prior_row_norms=np.linalg.norm(d['exact_feedback_gain'][i]-d['exact_feedback_gain'][i-1],axis=1).tolist(),
            exact_feedback_error=d['exact_error'][i].tolist()))
    result=dict(iteration='I-80',case='eight_m100_N',no_new_physics=True,
        comparison_qualification='Dominant current positive-pressure family vs current inferred load record. The latter uses a filtered completed-interval wrench with current geometry; disagreement is not a proven observer error.',
        window_s=[float(tt[0]),float(tt[-1])],window_rows=int(m.sum()),families=list(NAMES),
        pressure_dominant_changes=int(np.count_nonzero(np.diff(tl[m]))),
        inferred_dominant_changes=int(np.count_nonzero(np.diff(el[m]))),
        dominant_family_disagreement_rows=int(np.count_nonzero(tl[m]!=el[m])),
        disagreement_groups=groups(tl[m]!=el[m],tt),
        pressure_family_row_counts={name:int(np.count_nonzero(tl[m]==i)) for i,name in enumerate(NAMES)},
        inferred_family_row_counts={name:int(np.count_nonzero(el[m]==i)) for i,name in enumerate(NAMES)},
        current_point_disagreement_m=dict(max=float(np.max(pd[m])),rms=float(np.sqrt(np.mean(pd[m]**2))),median=float(np.median(pd[m]))),
        selected_rows=rows)
    assert sha(p)==binding['inputs'][0]['sha256']
    (OUT/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (OUT/'manifest.json').write_text(json.dumps(dict(inputs_unchanged=True,no_new_physics=True,
        files=[dict(path=x.name,bytes=x.stat().st_size,sha256=sha(x)) for x in sorted(OUT.iterdir()) if x.is_file()]),indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['pressure_dominant_changes','inferred_dominant_changes','dominant_family_disagreement_rows','pressure_family_row_counts','inferred_family_row_counts','current_point_disagreement_m']}))


if __name__=='__main__':main()
