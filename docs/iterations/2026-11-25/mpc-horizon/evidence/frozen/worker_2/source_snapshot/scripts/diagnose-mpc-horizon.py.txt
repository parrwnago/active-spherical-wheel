"""Frozen-state horizon comparison through the actual MPC runtime class.

Full-stage derivatives, improved present-state initialization, unchanged costs,
physical bounds and 20 ms stage time. No Gazebo integration or future actual
commands enter the candidate generation. Raw total costs are compared only to
a native-policy candidate at the SAME horizon, not ranked across horizons.
"""
from contextlib import ExitStack
from dataclasses import asdict
from datetime import datetime, timezone
from importlib.metadata import version
import argparse
import copy
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
from time import perf_counter
import traceback
from unittest.mock import patch

import mujoco
import numpy as np

from spherical_wheel.model import REPO_ROOT
from spherical_wheel.offroad_control import OffroadController
from spherical_wheel.predictive_control import AffineMPCSettings

SOURCE = REPO_ROOT/'results/feedback_recovery_20261125/runs/slope/mpc'
DEFAULT_PROTOCOL = REPO_ROOT/'results/mpc_horizon_20261125/protocol.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def clean(value):
    if isinstance(value, dict): return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [clean(v) for v in value]
    if isinstance(value, np.ndarray): return clean(value.tolist())
    if isinstance(value, np.generic): return clean(value.item())
    if isinstance(value, Path): return str(value)
    if isinstance(value, float) and not np.isfinite(value): return None
    return value


def save(path, value):
    path.write_text(json.dumps(clean(value), indent=2, allow_nan=False)+'\n')


def controller_snapshot(controller):
    plant = controller.plant
    return clean({'qpos': plant.data.qpos.copy(), 'qvel': plant.data.qvel.copy(),
        'time_s': float(plant.data.time), 'last_input': plant.last_input.copy(),
        'ctrl': plant.data.ctrl.copy(), 'surfacevel': plant.model.geom_surfacevel.copy(),
        'observer_wrench': np.r_[controller.observer.wrench()],
        'observer_previous': copy.deepcopy(controller.observer._previous),
        'observer_generalized': controller.observer.generalized.copy(),
        'observer_interval_mean': controller.observer.interval_mean.copy(),
        'estimator_normal': controller.estimator.normal.copy(),
        'nominal_joints': controller.nominal_joints.copy(),
        'native_vref': controller._vref.copy(), 'native_sync': copy.deepcopy(controller._sync),
        'native_held': copy.deepcopy(controller._held),
        'sync_qpos': controller.synchronizer._data.qpos.copy(),
        'sync_qvel': controller.synchronizer._data.qvel.copy()})


def trajectory_metrics(experiment, q0, v0, states, target, goal, inputs):
    predictor = experiment.predictor
    poses, velocities = zip(*(predictor.perturb(q0, v0, x) for x in states))
    poses, velocities = np.asarray(poses), np.asarray(velocities)
    error = 1000*np.linalg.norm(poses[:, :2]-goal, axis=1)
    initial = float(1000*np.linalg.norm(q0[:2]-goal))
    tilt = np.degrees(np.arccos(np.clip(1-2*np.sum(poses[:, 4:6]**2, axis=1), -1, 1)))
    speed = 1000*np.linalg.norm(velocities[:, :2], axis=1)
    relative = []
    for q, xref in zip(poses, target):
        qref, _ = predictor.perturb(q0, v0, xref)
        matrix, reference = np.empty(9), np.empty(9)
        mujoco.mju_quat2Mat(matrix, q[3:7]);mujoco.mju_quat2Mat(reference, qref[3:7])
        relative.append(float(np.degrees(np.arccos(np.clip(
            matrix.reshape(3, 3)[:, 2]@reference.reshape(3, 3)[:, 2], -1, 1)))))
    end_error = poses[-1, :2]-goal
    radial = float(1000*(end_error@velocities[-1, :2])/max(np.linalg.norm(end_error), 1e-15))
    contacts = []
    zero_labels = ('shell_', 'belt_')
    q, v = q0.copy(), v0.copy()
    substeps = int(round(experiment.settings.step_s/predictor.model.opt.timestep))
    stage_states = []
    def contact_state(instant):
        # Contact forces are the last physics substep's solve, matching the
        # simulator logging convention; this never reads actual robot contacts.
        wheel_load = nonwheel_load = 0.
        wheel_names, nonwheel_names = set(), set()
        nonwheel_geoms = {}
        for index, contact in enumerate(predictor.data.contact):
            if contact.efc_address < 0 or predictor.floor_id not in contact.geom:continue
            geom = int(contact.geom[1] if contact.geom[0] == predictor.floor_id else contact.geom[0])
            name = mujoco.mj_id2name(predictor.model, mujoco.mjtObj.mjOBJ_GEOM, geom) or str(geom)
            force = np.zeros(6);mujoco.mj_contactForce(predictor.model, predictor.data, index, force)
            if force[0] <= 0.:continue
            if name.startswith(zero_labels):wheel_load += force[0];wheel_names.add(name)
            else:
                nonwheel_load += force[0];nonwheel_names.add(name)
                nonwheel_geoms[name] = nonwheel_geoms.get(name, 0.)+float(force[0])
        quat = predictor.data.qpos[3:7]
        return {'prediction_time_s': instant, 'wheel_normal_load_N': float(wheel_load),
            'nonwheel_normal_load_N': float(nonwheel_load),
            'maximum_nonwheel_geom_load_N': max(nonwheel_geoms.values(), default=0.),
            'absolute_tilt_deg': float(np.degrees(np.arccos(np.clip(1-2*np.sum(quat[1:3]**2),-1,1)))),
            'goal_error_mm':float(1000*np.linalg.norm(predictor.data.qpos[:2]-goal)),
            'xy_speed_mm_s':float(1000*np.linalg.norm(predictor.data.qvel[:2])),
            'loaded_wheel_geoms': sorted(wheel_names), 'loaded_nonwheel_geoms': sorted(nonwheel_names)}
    for stage, command in enumerate(inputs):
        # Match predictor.rollout exactly: reset integration once per20ms
        # transition, then inspect its20physics steps without an extra forward.
        predictor._assign(q, v, command)
        if stage == 0:contacts.append(contact_state(0.))
        for substep in range(substeps):
            mujoco.mj_step(predictor.model,predictor.data)
            contacts.append(contact_state((stage*substeps+substep+1)*predictor.model.opt.timestep))
        q, v = predictor.data.qpos.copy(), predictor.data.qvel.copy()
        stage_states.append(predictor.tangent(q0,v0,q,v))
    stage_states = np.asarray(stage_states)
    stage_replay_exact = bool(np.array_equal(stage_states,np.asarray(states)))
    np.testing.assert_allclose(stage_states,np.asarray(states),rtol=1e-12,atol=1e-12)
    tail=[c for c in contacts if c['prediction_time_s']>=experiment.settings.horizon_steps*experiment.settings.step_s-.2-1e-12]
    tail_slope=float(np.polyfit([c['prediction_time_s'] for c in tail],[c['goal_error_mm'] for c in tail],1)[0])
    return {'initial_goal_error_mm': initial, 'terminal_goal_error_mm': float(error[-1]),
        'goal_progress_mm': initial-float(error[-1]), 'peak_goal_error_mm': max(c['goal_error_mm'] for c in contacts),
        'final_200ms_goal_error_slope_mm_s':tail_slope,
        'final_200ms_goal_error_change_mm':tail[-1]['goal_error_mm']-tail[0]['goal_error_mm'],
        'terminal_xy_speed_mm_s': float(speed[-1]), 'peak_xy_speed_mm_s': float(speed.max()),
        'terminal_radial_velocity_mm_s': radial,
        'terminal_absolute_tilt_deg': float(tilt[-1]), 'peak_absolute_tilt_deg': float(tilt.max()),
        'terminal_reference_lean_error_deg': relative[-1],
        'peak_reference_lean_error_deg': max(relative),
        'private_nonwheel_loaded_substeps': sum(c['maximum_nonwheel_geom_load_N'] > .05 for c in contacts),
        'private_min_wheel_normal_load_N': min(c['wheel_normal_load_N'] for c in contacts),
        'private_max_nonwheel_geom_normal_load_N': max(c['maximum_nonwheel_geom_load_N'] for c in contacts),
        'private_peak_substep_absolute_tilt_deg': max(c['absolute_tilt_deg'] for c in contacts),
        'contact_substep_s': float(predictor.model.opt.timestep),
        'contact_replay_stage_states_bit_exact': stage_replay_exact,
        'contact_sampling_caveat': 'Private1msphysics-substep forces/contactnames, lastsolverstep timing; not actual contact or probabilities.',
        'goal_error_mm': error.tolist(), 'xy_speed_mm_s': speed.tolist(),
        'absolute_tilt_deg': tilt.tolist(), 'reference_lean_error_deg': relative,
        'contact_samples': contacts}


def nonlinear_progress(plan, seed_costs):
    best = min(seed_costs.values(), default=np.inf)
    rows = [{'budget': 0, 'best_cost': best}]
    budget = 1
    for trial in plan.get('trials', []):
        current = int(trial.get('refinement_iteration', 1))
        if current != budget:
            rows.append({'budget': budget, 'best_cost': best});budget = current
        if 'nominal_cost' in trial:best = min(best, trial['nominal_cost'])
        for line in trial.get('line_search', []):best = min(best, line['nonlinear_cost'])
    rows.append({'budget': budget, 'best_cost': best})
    return rows


def run(args):
    output = args.output.resolve()
    if output.exists():raise FileExistsError(f'Preserve earlier stage; choose a new output: {output}')
    assert args.protocol.is_file(), 'Declare the experimental protocol before calculations'
    output.mkdir(parents=True)
    paths = [Path(__file__).resolve(), args.protocol.resolve(),
        REPO_ROOT/'scripts/run-mpc-comparison.py', REPO_ROOT/'scripts/diagnose-mpc-search.py',
        REPO_ROOT/'src/spherical_wheel/predictive_seed.py',
        REPO_ROOT/'src/spherical_wheel/predictive_control.py',
        REPO_ROOT/'src/spherical_wheel/predictive_model.py',
        REPO_ROOT/'src/spherical_wheel/offroad_control.py',
        REPO_ROOT/'src/spherical_wheel/contact_sensing.py', REPO_ROOT/'config/model.yaml']
    entries=[]
    for index, path in enumerate(paths):
        target=output/'source_snapshot'/str(path.relative_to(REPO_ROOT))
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(path.read_bytes())
        entries.append({'origin':str(path.relative_to(REPO_ROOT)),
            'snapshot':str(target.relative_to(output)),'sha256':digest(path)})
    inputs=[SOURCE/name for name in ('plans.json','precision.npz','provenance.json')]
    save(output/'provenance.json',{'captured_before_calculations_utc':datetime.now(timezone.utc).isoformat(),
        'arguments':vars(args),'sources':entries,'inputs':{str(p.relative_to(REPO_ROOT)):digest(p) for p in inputs},
        'versions':{p:version(p) for p in ('numpy','scipy','mujoco','osqp')},
        'method':'Actual runtime class, private estimated-plane physics, full-stage Jacobians, same five refinement budget. Cold optimizer for each horizon.',
        'warm_extension':'At2s preserve old preceding20-stage sequence, pad last bounded command toN; runtime shifts one stage exactlyasbefore. Native full-horizon rollout independent candidate.',
        'cost_comparison':'Do not rank rawcosts acrossdifferentN; relativecandidatecost andstate/contactquality are perhorizon.'})
    helper=load_module('horizon_history_helper',REPO_ROOT/'scripts/diagnose-mpc-search.py')
    runtime=load_module('horizon_runtime',REPO_ROOT/'scripts/run-mpc-comparison.py')
    with np.load(SOURCE/'precision.npz') as data:exact={k:data[k] for k in data.files}
    plans=json.loads((SOURCE/'plans.json').read_text())
    report={'status':'in_progress','scope':'Frozen-state private-model horizon comparison only; no Gazebo or actual future commands.','rows':[],'details':[]}
    rows=[]
    for at in args.anchors:
        anchor=output/f'anchor_{at:g}';anchor.mkdir()
        i=int(np.argmin(abs(exact['time_s']-at)))
        assert abs(exact['time_s'][i]-at)<1e-8
        controller,native,info,path,replay=helper.replay_native(exact,i,anchor)
        before=controller_snapshot(controller)
        original=min(plans,key=lambda p:abs(p['time_s']-at))
        q0,v0=exact['exact_qpos'][i],exact['exact_qvel'][i]
        goal=path.origin.copy()
        for horizon in args.horizons:
            n=int(round(horizon/.02));assert n>=1 and np.isclose(n*.02,horizon)
            case=anchor/f'horizon_{horizon:g}';case.mkdir()
            settings=AffineMPCSettings(horizon_steps=n,step_s=.02)
            experiment=runtime.TrajectoryPredictiveExperiment(controller.plant,path,settings,
                load_residual_fraction=.1,jacobian_stride=1,seed_shaft_torque=.03,
                refinement_steps=args.refinements,native_rollout_seed=True,
                native_command_function=OffroadController.command)
            if at>.4+1e-8:
                previous=min(plans,key=lambda p:abs(p['time_s']-(at-.02)))
                prior=np.asarray(previous['inputs'],float)
                if n>=len(prior):prior=np.vstack((prior,np.tile(prior[-1],(n-len(prior),1))))
                else:prior=prior[:n].copy()
                experiment.sequence=prior
                warm=np.vstack((prior[1:],prior[-1:]))
                np.testing.assert_array_equal(warm[:min(n,20)],np.asarray(original['model_anchor_inputs'])[:min(n,20)])
            start=perf_counter()
            with ExitStack() as stack:
                for guard in helper.GUARDS:stack.enter_context(patch(guard,helper.forbidden))
                command,diagnostics=experiment.command(controller,at,native,info)
            elapsed=perf_counter()-start;plan=experiment.plan_rows[0]
            assert controller_snapshot(controller)==before,'Horizon calculation changed current real controller history'
            save(case/'plan.json',plan);save(case/'runtime_diagnostics.json',diagnostics)
            record={'anchor_s':at,'horizon_s':horizon,'horizon_steps':n,'refinement_budget':args.refinements,
                'jacobian_stride':1,'success':bool(plan.get('success',False)),
                'native_seed_failure':plan.get('native_rollout_seed_failure'),
                'selected_source':plan.get('command_source'),'wall_s':elapsed,
                'qp_solved_count':plan.get('qp_solved_count',0),
                'qp_calls':sum('solver' in trial for trial in plan.get('trials',[])),
                'failed_trials':sum('failure' in trial for trial in plan.get('trials',[])),
                'solver_status_counts':{},'cost':plan.get('nonlinear_cost'),
                'horizon_raw_costs_comparable':False}
            for trial in plan.get('trials',[]):
                status=trial.get('solver',{}).get('status',trial.get('failure','no_solver'))
                record['solver_status_counts'][status]=record['solver_status_counts'].get(status,0)+1
            if plan.get('success'):
                assert np.all(abs(command)<=controller.plant.bounds+1e-10)
                target=np.asarray(plan['target_states']);states=np.asarray(plan['nonlinear_states'])
                selected=trajectory_metrics(experiment,q0,v0,states,target,goal,np.asarray(plan['inputs']))
                detail={'selected_trajectory':selected,'native_history_reconstruction':replay}
                seed_costs={}
                for trial in plan['trials']:
                    if trial['refinement_iteration']==1 and 'nominal_cost' in trial:
                        seed_costs[trial['seed']]=trial['nominal_cost']
                if 'native_rollout_seed' in plan:
                    fine=np.asarray(plan['native_rollout_seed']['fine_commands'])
                    seed=fine.reshape(n,4,5).mean(axis=1)
                    transform=np.eye(18);transform[2,:3]=plan['normal'];transform[11,9:12]=plan['normal']
                    ns,nc,components=experiment.evaluate(q0,v0,seed,target,transform,controller.plant.bounds,controller.plant.last_input)
                    seed_costs['native_rollout']=nc
                    native_metrics=trajectory_metrics(experiment,q0,v0,ns,target,goal,seed)
                    detail.update(native_candidate_cost=nc,native_candidate_components=components,native_candidate_trajectory=native_metrics)
                    coarse_q,coarse_v=experiment.predictor.perturb(q0,v0,ns[-1])
                    fine_q=np.asarray(plan['native_rollout_seed']['fine_rollout_terminal_qpos'])
                    fine_v=np.asarray(plan['native_rollout_seed']['fine_rollout_terminal_qvel'])
                    difference=experiment.predictor.tangent(fine_q,fine_v,coarse_q,coarse_v)
                    detail['native_fine_vs_coarse_terminal_divergence']={
                        'xy_mm':float(1000*np.linalg.norm(difference[:2])),
                        'rotation_deg':float(np.degrees(np.linalg.norm(difference[3:6]))),
                        'xy_velocity_mm_s':float(1000*np.linalg.norm(difference[9:11])),
                        'fine_goal_error_mm':float(1000*np.linalg.norm(fine_q[:2]-goal)),
                        'coarse_goal_error_mm':native_metrics['terminal_goal_error_mm']}
                    record['native_candidate_cost']=nc
                    record['selected_cost_to_native_ratio']=plan['nonlinear_cost']/max(nc,1e-30)
                    record['native_terminal_goal_error_mm']=native_metrics['terminal_goal_error_mm']
                    record['native_fine_coarse_terminal_xy_gap_mm']=detail['native_fine_vs_coarse_terminal_divergence']['xy_mm']
                    assert plan['nonlinear_cost']<=nc+1e-6*max(1,abs(nc))
                for key,value in selected.items():
                    if not isinstance(value,(list,dict)):record[key]=value
                detail['nonlinear_cost_progress']=nonlinear_progress(plan,seed_costs)
                save(case/'trajectory_quality.json',detail)
                record['selected_inputs_sha256']=hashlib.sha256(np.asarray(plan['inputs'],dtype=np.float64).tobytes()).hexdigest()
                record['settings']=asdict(settings)
            else:
                record['failure']=plan.get('failure')
            rows.append(record);save(case/'summary.json',record)
            report['rows']=rows;save(output/'report.json',report)
            print(json.dumps(clean({k:v for k,v in record.items() if k!='settings'})),flush=True)
    report.update(status='completed',source_controller_unchanged=True,
        limits=['Longer rawcost sums are not ranked across horizons.','Private finite/bounded forecasts do not prove closed-loop recovery or known future terrain.','Contact metrics inspect1msphysics-substep solves, not actual contacts or a safety certificate.','Fresh solver cache can differ from old continuously warmed solutions.','No PSDrepair, objective rescaling, newweights or additional hardstateconstraints.'])
    save(output/'report.json',report)
    fields=sorted(set().union(*(r.keys() for r in rows))-{'settings','solver_status_counts'})
    with (output/'metrics.csv').open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=fields);writer.writeheader();writer.writerows({k:r.get(k) for k in fields} for r in rows)
    save(output/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(output)),'bytes':p.stat().st_size,'sha256':digest(p)}
        for p in sorted(output.rglob('*')) if p.is_file() and 'history_workspace' not in p.parts and '__pycache__' not in p.parts]})
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--protocol',type=Path,default=DEFAULT_PROTOCOL)
    parser.add_argument('--anchors',nargs='+',type=float,default=[.4,2.])
    parser.add_argument('--horizons',nargs='+',type=float,default=[.4,.8,1.2])
    parser.add_argument('--refinements',type=int,default=5)
    args=parser.parse_args()
    if args.refinements<1:parser.error('Refinement budget must be positive')
    if any(not np.isfinite(h) or h<=0 or not np.isclose(h/.02,round(h/.02),atol=1e-9,rtol=0) for h in args.horizons):parser.error('Horizons must be positive20ms multiples')
    try:run(args)
    except Exception:
        if args.output.is_dir() and not (args.output/'artifact_manifest.json').exists():
            (args.output/'failure.txt').write_text(traceback.format_exc())
        raise


if __name__=='__main__':main()
