"""Independent fixed-frame physical-force QP review; no truth inputs.

Reconstruct frozen model outputs at six prespecified frames. Reference SLSQP
uses3/6 Cartesian force variables and linear square-cone/share constraints;
it never uses the model's nonnegative ray coefficients or ray generators.
Force vectors may be nonunique: objectives/feasibility, not vector equality,
are the verification criteria. Every optimizer status is retained.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import LinearConstraint, minimize


CASES=('m100_B','p030_B','p090_B')
TIMES=(7.875,35.205)
CONSTRAINT_TOLERANCE_N=1e-7
OBJECTIVE_RELATIVE_TOLERANCE=1e-7


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    with np.load(path,allow_pickle=False) as z:return {key:z[key] for key in z.files}


def physical_constraints(normals,mu=.7,minimum_share=.1):
    """C F>=0 directly, using the registered inscribed square orientation."""
    normals=np.asarray(normals,float);k=len(normals);rows=[]
    for i,normal in enumerate(normals):
        n=normal/np.linalg.norm(normal)
        axis=np.zeros(3);axis[int(np.argmin(np.abs(n)))]=1.
        t=np.cross(n,axis);t/=np.linalg.norm(t);s=np.cross(n,t)
        local=[n,mu/np.sqrt(2)*n+t,mu/np.sqrt(2)*n-t,
                  mu/np.sqrt(2)*n+s,mu/np.sqrt(2)*n-s]
        for direction in local:
            row=np.zeros(3*k);row[3*i:3*i+3]=direction;rows.append(row)
    if k==2:
        n0=normals[0]/np.linalg.norm(normals[0]);n1=normals[1]/np.linalg.norm(normals[1])
        rows.extend([np.r_[(1-minimum_share)*n0,-minimum_share*n1],
                     np.r_[-minimum_share*n0,(1-minimum_share)*n1]])
    return np.asarray(rows)


def reference_solve(observation,maps,normals,sigma,mu=.7,minimum_share=.1):
    """One independent feasible normal-only start, fixed SLSQP settings."""
    y=np.asarray(observation,float);maps=np.asarray(maps,float);sigma=np.asarray(sigma,float)
    k=len(maps);a=np.concatenate(maps,axis=1)/sigma[:,None];b=y/sigma
    c=physical_constraints(normals,mu,minimum_share)
    def objective(force):
        error=a@force-b
        return .5*float(error@error)
    def gradient(force):return a.T@(a@force-b)
    normal=np.asarray(normals,float);normal=normal/np.linalg.norm(normal,axis=1)[:,None]
    start=(normal*max(1.,float(np.linalg.norm(y[:3])))/k).ravel()
    solved=minimize(objective,start,jac=gradient,method='SLSQP',
        constraints=[LinearConstraint(c,0.,np.inf)],options=dict(ftol=1e-12,maxiter=1000,disp=False))
    return solved,objective,c


def run(args):
    base=Path(args.base).resolve();output=Path(args.output)
    if output.exists():raise FileExistsError(output)
    seal=json.loads(Path(args.seal).read_text())
    for row in seal['files']:
        if sha(row['path'])!=row['sha256']:raise AssertionError('Sealed review input/source changed: '+row['path'])
    sys.path.insert(0,str(base/'inputs'))
    from input_features import frame_features
    from spherical_wheel.shadow_contact_models import evaluate_frame,ModelConfig
    config=ModelConfig();frames=[]
    for case in CASES:
        directory=base/'inputs/stage01'/case
        primitives,cached=load(directory/'primitives.npz'),load(directory/'features.npz')
        evidence=load(base/'inference/stage01'/case/'model_evidence.npz')
        for time_s in TIMES:
            indices=np.flatnonzero(np.abs(primitives['time_s']-time_s)<1e-9)
            if len(indices)!=1:raise AssertionError('Fixed frame unavailable')
            i=int(indices[0]);features=frame_features(primitives,i,cached)
            model=evaluate_frame(features['observation9'],features['points_m'],features['force_normals'],
                features['material_velocity_m_s'],features['force_maps'],features['family_index'],
                features['valid'],features['normal_hint'],features['candidate_index'],
                dt_s=float(primitives['time_s'][i]-primitives['time_s'][i-1]),observation_valid=features['interval_valid'])
            for actual,expected in [(model['node_cost'],evidence['node_cost'][i]),(model['edge_cost'],evidence['edge_cost'][i]),
                    (model['node']['selected_candidate_index'],evidence['node_candidate_index'][i]),
                    (model['edge']['selected_candidate_index'],evidence['edge_candidate_index'][i])]:
                if actual.dtype!=expected.dtype or actual.shape!=expected.shape or actual.tobytes()!=expected.tobytes():
                    raise AssertionError('Frozen fixed-frame reconstruction differs')
            # All valid node/edge MAP alternatives, deduplicated by their coupled
            # original IDs. Motion/endpoint changes never alter the force fit.
            selected={}
            for category in ('node','edge'):
                storage=model[category]
                for index in np.ndindex(storage['valid'].shape):
                    if not storage['valid'][index]:continue
                    ids=storage['selected_candidate_index'][index];ids=tuple(int(x) for x in ids if x>=0)
                    label=category+':'+','.join(map(str,index))
                    forces=storage['estimated_force_N'][index][:len(ids)]
                    if ids in selected:
                        if forces.tobytes()!=selected[ids]['forces'].tobytes():raise AssertionError('Same coupled fit reconstructed inconsistently')
                        selected[ids]['models'].append(label)
                    else:selected[ids]=dict(forces=forces,models=[label],stored_cost=float(storage['force_fit_cost'][index]))
            record=dict(case=case,time_s=time_s,frame_index=i,model_reconstruction_byte_exact=True,
                        selected_model_count=sum(len(x['models']) for x in selected.values()),unique_force_fits=[])
            for ids,item in selected.items():
                rows=[int(np.flatnonzero(features['candidate_index']==ident)[0]) for ident in ids]
                maps=features['force_maps'][rows];normal=features['force_normals'][rows]
                solved,objective,c=reference_solve(features['observation9'],maps,normal,config.residual_sigma,
                    config.friction_coefficient,config.minimum_pair_normal_share)
                baseline=item['forces'].ravel();native_cost=objective(baseline)
                reference_cost=objective(solved.x)
                native_violation=max(0.,float(np.max(-(c@baseline))))
                reference_violation=max(0.,float(np.max(-(c@solved.x))))
                cost_gap=abs(reference_cost-native_cost)
                limit=OBJECTIVE_RELATIVE_TOLERANCE*(1.+native_cost)
                numeric_pass=(np.isfinite(reference_cost) and native_violation<=CONSTRAINT_TOLERANCE_N
                    and reference_violation<=CONSTRAINT_TOLERANCE_N and cost_gap<=limit)
                record['unique_force_fits'].append(dict(candidate_ids=list(ids),model_uses=item['models'],
                    nnls_force_N=item['forces'].tolist(),slsqp_force_N=solved.x.reshape(-1,3).tolist(),
                    force_vector_distance_N=float(np.linalg.norm(solved.x-baseline)),
                    nnls_recomputed_cost=native_cost,nnls_stored_cost=item['stored_cost'],
                    stored_objective_gap=abs(native_cost-item['stored_cost']),slsqp_cost=reference_cost,
                    objective_absolute_gap=cost_gap,objective_tolerance=limit,
                    nnls_max_constraint_violation_N=native_violation,slsqp_max_constraint_violation_N=reference_violation,
                    slsqp_success=bool(solved.success),slsqp_status=int(solved.status),slsqp_message=str(solved.message),
                    slsqp_iterations=int(solved.nit),numerical_crosscheck_passed=bool(numeric_pass),
                    fully_validated=bool(numeric_pass and solved.success)))
            frames.append(record)
    fits=[fit for frame in frames for fit in frame['unique_force_fits']]
    report=dict(scope='Six preregistered measured-state frames; all valid selected node/edge alternatives; no labels.',
        source_sha256=sha(__file__),numpy_version=np.__version__,config=vars(config),
        constraint_tolerance_N=CONSTRAINT_TOLERANCE_N,objective_relative_tolerance=OBJECTIVE_RELATIVE_TOLERANCE,
        frames=frames,summary=dict(frame_count=len(frames),unique_fit_count=len(fits),
            selected_model_count=sum(x['selected_model_count'] for x in frames),
            reference_optimizer_success_count=sum(x['slsqp_success'] for x in fits),
            numerical_crosscheck_pass_count=sum(x['numerical_crosscheck_passed'] for x in fits),
            fully_validated_count=sum(x['fully_validated'] for x in fits),
            max_nnls_constraint_violation_N=max((x['nnls_max_constraint_violation_N'] for x in fits),default=0.),
            max_reference_constraint_violation_N=max((x['slsqp_max_constraint_violation_N'] for x in fits),default=0.),
            max_objective_absolute_gap=max((x['objective_absolute_gap'] for x in fits),default=0.)),
        qualification='Cartesian-force convex QP comparison, not a truth/estimator-performance evaluation. Force differences can reflect nonunique solutions. Failed reference statuses remain failures despite close objectives.')
    output.mkdir(parents=True)
    (output/'report.json').write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n')
    print(json.dumps(report['summary'],sort_keys=True))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('base','seal','output'):p.add_argument('--'+name,type=Path,required=True)
    run(p.parse_args())
