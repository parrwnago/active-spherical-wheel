"""Append-only post-result check of single-family inactive-joint load floors."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json
import numpy as np

CASES=('m100_B','p030_B','p090_B')
GROUPS=('focus','useful','harmful','correct_pair','stable_correct_pair')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def load(path):
    with np.load(path,allow_pickle=False) as z:return {key:z[key].copy() for key in z.files}
def write(path,data):Path(path).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')


def run(root,output):
    protocol=read(root/'protocol/protocol.json');i64=root.parents[1]/protocol['dependencies']['i64']
    structural_path=root/'review/structural_routing_stage01/output/report.json';original=read(structural_path)
    output.mkdir(parents=True,exist_ok=False)
    inputs=[structural_path,root/'analysis/stage01/science.json']
    for case in CASES:inputs += [root/'features/stage01'/case/'pose_maps.npz',root/'analysis/stage01'/f'{case}_frames.npz',i64/'features/portable_stage01'/case/'primitives.npz',i64/'portable_inputs/i62_base'/f'{case}.npz']
    write(output/'registered_before_floor_arithmetic.json',dict(source_sha256=sha(Path(__file__)),registered_utc=datetime.now(timezone.utc).isoformat(),
        status='Append-only explanatory algebra after canonical I66 results; no score/threshold/estimator changes.',
        inputs=[dict(path=str(path),sha256=sha(path)) for path in inputs],
        qualification='Inactive-ancestor raw joint loads bound C/T/W under single-family assignment. This identifies no physical cause and is not extended to split route S.'))
    cases={};rows=[];maximum_identity=0.;maximum_prediction=0.;opportunities=0;single_positive=0;split_difference=0.
    for case in CASES:
        pose=load(root/'features/stage01'/case/'pose_maps.npz');a=load(root/'analysis/stage01'/f'{case}_frames.npz')
        p=load(i64/'features/portable_stage01'/case/'primitives.npz');base=load(i64/'portable_inputs/i62_base'/f'{case}.npz')
        f=base['truth_family'];inactive=~pose['ancestor_active'][np.clip(f,0,3)];raw=p['observer_interval_mean'][:,6:9]
        lower=np.where(inactive,raw,0.);floor2=np.sum(lower**2,axis=1);T2=np.sum(a['T_sampled_point_residual_Nm']**2,axis=1)
        valid=pose['valid']&(f>=0)&np.isfinite(raw).all(axis=1)
        for route in ['C_candidate','T_sampled_point','W_single_family_wrench']:
            mask=valid&a[route+'_valid'];pred=a[route+'_prediction_Nm'];res=a[route+'_residual_Nm']
            if mask.any():
                maximum_prediction=max(maximum_prediction,float(np.max(abs(np.where(inactive,pred,0.)[mask]))))
                maximum_identity=max(maximum_identity,float(np.max(abs(np.where(inactive,res-raw,0.)[mask]))))
                if np.any(np.sum(res[mask]**2,axis=1)<floor2[mask]-1e-12):raise ValueError('Route residual violates inactive-joint lower bound')
        opportunity=a['cohort_useful']|a['cohort_harmful'];one=np.sum(a['normal_shares']>0,axis=1)==1
        opportunities+=int(opportunity.sum());single_positive+=int(np.sum(opportunity&one))
        known=opportunity&a['T_sampled_point_valid']&a['S_normal_share_proxy_valid']
        split_difference=max(split_difference,float(np.max(abs(a['T_sampled_point_prediction_Nm'][known]-a['S_normal_share_proxy_prediction_Nm'][known]))))
        data=dict(floor2=floor2,T2=T2,valid=valid,one=one,**{key:a['cohort_'+key] for key in GROUPS});rows.append(data)
        def summarize(values):
            result={}
            for name in GROUPS:
                mask=values[name]&values['valid']&np.isfinite(values['T2']);den=float(np.sum(values['T2'][mask]));num=float(np.sum(values['floor2'][mask]))
                result[name]=dict(denominator=int(values[name].sum()),known=int(mask.sum()),
                    inactive_joint_floor_RMS_Nm=float(np.sqrt(num/mask.sum())) if mask.any() else None,
                    floor_share_of_reference_squared_error=num/den if den else None,sampled_single_positive_family_count=int(np.sum(mask&values['one'])))
            return result
        cases[case]=summarize(data)
    pooled=summarize({key:np.concatenate([row[key] for row in rows]) for key in rows[0]})
    differences=[];counts=True
    for observed,expected in [(pooled,original['cohorts'])]+[(cases[c],original['per_case'][c]) for c in CASES]:
        for group in GROUPS:
            for key in observed[group]:
                x,y=observed[group][key],expected[group][key]
                if isinstance(x,int):counts &= x==y
                elif x is None or y is None:counts &= x is y
                else:differences.append(abs(x-y))
    passed=counts and maximum_identity<=1e-10 and maximum_prediction<=1e-10 and max(differences,default=0.)<=1e-12 and opportunities==300 and single_positive==300 and split_difference<=1e-10
    report=dict(passed=bool(passed),cohorts=pooled,per_case=cases,exact_counts=bool(counts),maximum_structural_summary_difference=max(differences,default=0.),
        maximum_inactive_prediction_Nm=maximum_prediction,maximum_inactive_residual_identity_Nm=maximum_identity,
        opportunity_count=opportunities,single_positive_family_opportunities=single_positive,T_vs_S_maximum_prediction_difference_Nm=split_difference,
        limitation='One sampled positive family does not imply one physical contact or a constant5ms route. The floor does not establish lag, model error, or a noise cause.')
    write(output/'report.json',report);print(json.dumps(report['cohorts']));print(json.dumps(dict(passed=passed)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--experiment',type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument('--output',type=Path)
    a=p.parse_args();run(a.experiment.resolve(),a.output.resolve() if a.output else Path(__file__).resolve().parent/'floor_output_stage01')
