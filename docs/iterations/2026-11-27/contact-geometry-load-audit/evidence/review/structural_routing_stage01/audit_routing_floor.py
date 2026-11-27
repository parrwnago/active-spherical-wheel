"""Post-result explanatory check of single-family structural load restrictions."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,numpy as np


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k] for k in z.files}


def run(pose_root,frames_root,i64,science_path,output):
    science=json.loads(science_path.read_text());output.mkdir(exist_ok=False)
    bound=[Path(__file__).resolve(),science_path]
    for c in ('m100_B','p030_B','p090_B'):
        bound += [pose_root/c/'pose_maps.npz',frames_root/f'{c}_frames.npz',i64/'features/portable_stage01'/c/'primitives.npz',i64/'portable_inputs/i62_base'/f'{c}.npz']
    receipt=dict(utc=datetime.now(timezone.utc).isoformat(),status='Registered after main I66 results and before this structural-floor calculation.',
        hypothesis='A structurally non-ancestor joint cannot receive external generalized load from any point/wrench assigned entirely to the current dominant family. Its raw observer joint residual is an unavoidable lower bound for C/T/W, not evidence of why it exists.',
        constraints='Same fixed cohorts/14events. No score, threshold, calibration, route or controller changes. Not extended to the multifamily share proxy. Correlated raw estimates may reflect earlier interval contact or model/actuation/constraint error; exact5ms truth unavailable.',
        files=[dict(path=str(p),sha256=sha(p)) for p in bound])
    (output/'registered_before_structural_arithmetic.json').write_text(json.dumps(receipt,indent=2)+'\n')
    all_data=[];events=[];percase={};identity_max=0.
    for case in ('m100_B','p030_B','p090_B'):
        p=load(pose_root/case/'pose_maps.npz');a=load(frames_root/f'{case}_frames.npz')
        primitive=load(i64/'features/portable_stage01'/case/'primitives.npz');b=load(i64/'portable_inputs/i62_base'/f'{case}.npz')
        family=b['truth_family'];safe=np.maximum(family,0);inactive=~p['ancestor_active'][safe]
        raw_joint=primitive['observer_interval_mean'][:,6:9]
        floor=np.where(inactive,raw_joint,0.)
        valid=p['valid'] & (family>=0) & np.isfinite(raw_joint).all(axis=1)
        floor2=np.sum(floor*floor,axis=1)
        for route in ('C_candidate','T_sampled_point','W_single_family_wrench'):
            known=valid & a[route+'_valid'];residual=a[route+'_residual_Nm']
            delta=np.where(inactive,residual-floor,0.)[known]
            if len(delta):identity_max=max(identity_max,float(np.max(abs(delta))))
            if len(delta) and np.max(abs(delta))>1e-10:raise AssertionError('Inactive-joint identity failed')
            if np.any(np.sum(residual[known]**2,axis=1)+1e-12<floor2[known]):raise AssertionError('Single-family error below structural floor')
        rows=dict(floor2=floor2,valid=valid,reference_error2=np.sum(a['T_sampled_point_residual_Nm']**2,axis=1),
                  single_positive_sampled_family=np.sum(a['normal_shares']>0,axis=1)==1,
                  **{k:v for k,v in a.items() if k.startswith('cohort_')})
        all_data.append(rows)
        def summary(d,name):
            mask=d['cohort_'+name]&d['valid']&np.isfinite(d['reference_error2'])
            den=float(np.sum(d['reference_error2'][mask]));num=float(np.sum(d['floor2'][mask]))
            return dict(denominator=int(np.sum(d['cohort_'+name])),known=int(mask.sum()),
                        inactive_joint_floor_RMS_Nm=None if not mask.any() else float(np.sqrt(np.mean(d['floor2'][mask]))),
                        floor_share_of_reference_squared_error=None if den==0 else num/den,
                        sampled_single_positive_family_count=int(np.sum(mask & d['single_positive_sampled_family'])))
        percase[case]={k:summary(rows,k) for k in ('focus','useful','harmful','correct_pair','stable_correct_pair')}
        for e in science['events']:
            if e['case']!=case:continue
            i=e['index'];events.append(dict(case=case,time_s=e['time_s'],truth_family=int(family[i]),
                inactive_joint_mask=inactive[i].tolist(),raw_joint_load_Nm=raw_joint[i].tolist(),
                unavoidable_inactive_joint_load_Nm=floor[i].tolist(),floor_norm_Nm=float(np.sqrt(floor2[i]))))
    combined={k:np.concatenate([d[k] for d in all_data]) for k in all_data[0]}
    result=dict(iteration='I-66',status='Post-result explanatory algebra; not a new scored estimator.',
                cohorts={k:summary(combined,k) for k in ('focus','useful','harmful','correct_pair','stable_correct_pair')},
                per_case=percase,events=events,maximum_inactive_identity_error_Nm=identity_max,
                limitation='A model-consistency lower bound, not independent force truth or a causal attribution to contact lag. Single sampled positive family can contain multiple contacts; hidden interval transfers remain possible.')
    (output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result['cohorts'],indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('pose-root','frames-root','i64','science','output'):p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();run(a.pose_root,a.frames_root,a.i64,a.science,a.output)
