"""Offline-speed MPC experiment with a shared observer and retained nominal goal.

Run in the Gazebo environment. The private predictor receives measured state
and estimated support only. Controller defaults and production source are not
changed. The baseline's LQR fields remain explicitly logged as shadow fields
when direct MPC commands replace its final physical input.
"""
from contextlib import ExitStack
from dataclasses import asdict
from datetime import datetime, timezone
from importlib.metadata import PackageNotFoundError, version
from importlib.util import find_spec
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from time import perf_counter
from unittest.mock import patch

import mujoco
import numpy as np

from spherical_wheel.gazebo.experiments import GazeboWorkspace, run_gazebo
from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import ReferencePath, save_run
from spherical_wheel.predictive_seed import generate_native_rollout_seed
from spherical_wheel.terrain import Terrain


WARM_S, MOTION_S, RAMP_S, DISTANCE_M = .8, .6, .2, .016
PHASE = np.pi/2


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def diagnostic_json(value):
    """Keep failed/nonfinite solver diagnostics writable without fake numbers."""
    if isinstance(value, dict):
        return {key: diagnostic_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [diagnostic_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return diagnostic_json(value.tolist())
    if isinstance(value, np.generic):
        return diagnostic_json(value.item())
    return None if isinstance(value, float) and not np.isfinite(value) else value


def snapshot(directory, arguments):
    files = [Path(__file__).resolve(), REPO_ROOT/'pyproject.toml',
             *sorted((REPO_ROOT/'src/spherical_wheel').rglob('*.py')),
             *sorted((REPO_ROOT/'config').glob('*.yaml'))]
    hashes = {}
    for source in files:
        relative = source.relative_to(REPO_ROOT)
        target = directory/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
        hashes[str(relative)] = digest(source)
    versions = {}
    for package in ('numpy', 'scipy', 'mujoco', 'osqp'):
        try:
            versions[package] = version(package)
        except PackageNotFoundError:
            versions[package] = None
    result = {'captured_before_run_utc': datetime.now(timezone.utc).isoformat(),
              'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
              'arguments': vars(arguments) | {'output': str(arguments.output)},
              'source_sha256': hashes,
              'versions': versions,
              'measurement_boundary': 'Exact simulated qpos/qvel and applied inputs; observer/locator support. No live terrain or solved contact input to MPC.',
              'warm_s': WARM_S, 'motion_s': MOTION_S, 'distance_m': DISTANCE_M,
              'controller': f'Existing LQR baseline; native estimator every5ms, direct {arguments.model_mode} MPC replanned at its stage period after warmup.',
              'limits': 'Hard physical input bounds. No hard tilt, rotor-speed or slew constraints. Prediction approximation and optimizer convergence do not guarantee configuration recovery or real-time execution.'}
    (directory/'provenance.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


class PredictiveExperiment:
    def __init__(self, mirror, path, settings, load_residual_fraction=.002):
        from spherical_wheel.predictive_model import EstimatedPlanePredictor
        from spherical_wheel.predictive_control import AffineMPC
        self.predictor = EstimatedPlanePredictor(mirror)
        self.optimizer = AffineMPC(settings)
        self.settings, self.path = settings, path
        self.load_residual_fraction = float(load_residual_fraction)
        self.next_plan_time = -np.inf
        self.last_command = None
        self.last_status = 'not_started'
        self.plan_rows = []
        self.plane = {}

    def targets(self, controller, now, normal, q0, v0):
        targets = []
        yaw = float(np.arctan2(2*(q0[3]*q0[6]+q0[4]*q0[5]),
                               1-2*(q0[5]**2+q0[6]**2)))
        heading = np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)])
        for stage in range(1, self.settings.horizon_steps+1):
            xy, velocity, acceleration = self.path(now+stage*self.settings.step_s)
            qref = q0.copy()
            qref[:2] = xy
            qref[2] = q0[2]-normal[:2]@(xy-q0[:2])/max(normal[2], .2)
            reference = controller.reference_attitude(normal, acceleration)
            mujoco.mju_mulQuat(qref[3:7], reference, heading)
            vref = np.zeros(9)
            vref[:2] = velocity
            vref[2] = -normal[:2]@velocity/max(normal[2], .2)
            targets.append(self.predictor.tangent(q0, v0, qref, vref))
        return np.asarray(targets)

    def command(self, controller, now, native, info):
        plant = controller.plant
        row = {'mpc_active': True, 'mpc_updated': False, 'mpc_success': self.last_command is not None,
               'mpc_fallback': self.last_command is None, 'mpc_total_wall_s': 0.,
               'mpc_qp_wall_s': 0., 'mpc_linearization_wall_s': 0.,
               'mpc_plan_nonlinear_xy_max_error_m': np.nan,
               'mpc_plan_nonlinear_rotation_max_error_rad': np.nan,
               'mpc_plane_offset_m': self.plane.get('plane_offset_m', np.nan),
               'mpc_load_residual_N': self.plane.get('normal_load_residual_N', np.nan)}
        if now >= self.next_plan_time-1e-9:
            start = perf_counter()
            row['mpc_updated'] = True
            q0, v0 = plant.data.qpos.copy(), plant.data.qvel.copy()
            normal = np.asarray(info['estimate']['normal']).copy()
            point = np.asarray(info['measurement']['point_m']).copy()
            load = float(sum(info['measurement']['loads_N'].values()))
            plan = {'time_s': now, 'qpos': q0.tolist(), 'qvel': v0.tolist(),
                    'normal': normal.tolist(), 'point_m': point.tolist(), 'load_N': load}
            try:
                if load < .05*controller.weight or normal[2] < .2:
                    raise RuntimeError('insufficient_estimated_support')
                self.plane = self.predictor.update(q0, v0, normal, point, load,
                    friction=info['estimate']['friction'], command=plant.last_input)
                plan['calibration'] = self.plane
                allowed_error = max(.02, self.load_residual_fraction*load)
                usable = abs(self.plane['normal_load_residual_N']) <= allowed_error
                if self.load_residual_fraction > .002:
                    usable = usable and self.plane['load_fit_bracketed'] and abs(self.plane['plane_offset_m']) < self.plane['plane_offset_bound_m']-1e-7
                plan.update(load_model_usable=bool(usable), load_residual_fraction=self.load_residual_fraction)
                if not usable:
                    raise RuntimeError('estimated_plane_load_fit_failed')
                # Linearize about the actually applied preceding input, not
                # a potentially saturated shadow LQR request on a new path.
                anchor = plant.last_input.copy()
                A, B, f0 = self.predictor.linearize(q0, v0, anchor, self.settings.step_s)
                # Height/vertical-speed costs use the estimated plane normal,
                # while x/y retain world coordinates. No world-Z hold on slope.
                transform = np.eye(18)
                transform[2, :3] = normal
                transform[11, 9:12] = normal
                inverse = np.linalg.inv(transform)
                target = self.targets(controller, now, normal, q0, v0)
                result = self.optimizer.solve(transform@A@inverse, transform@B,
                    transform@f0, target@transform.T, anchor, plant.last_input, plant.bounds)
                plan.update(calibration=self.plane, solver=result.diagnostics,
                            target_states=target.tolist(), A=A.tolist(), B=B.tolist(), f0=f0.tolist())
                if not result.success:
                    raise RuntimeError('qp_'+result.diagnostics['status'])
                self.last_command = result.command.copy()
                affine_states = result.states@inverse.T
                # Independent nonlinear plan replay exposes failure of the
                # frozen affine model; it does not alter the selected command.
                nq, nv = self.predictor.rollout(q0, v0, result.inputs, self.settings.step_s)
                nonlinear = np.asarray([self.predictor.tangent(q0, v0, q, v)
                                        for q, v in zip(nq[1:], nv[1:])])
                difference = nonlinear-affine_states
                row['mpc_plan_nonlinear_xy_max_error_m'] = float(np.max(np.linalg.norm(difference[:, :2], axis=1)))
                row['mpc_plan_nonlinear_rotation_max_error_rad'] = float(np.max(np.linalg.norm(difference[:, 3:6], axis=1)))
                row['mpc_qp_wall_s'] = result.diagnostics['total_time_s']
                row['mpc_linearization_wall_s'] = self.predictor.diagnostics['linearization_s']
                plan.update(inputs=result.inputs.tolist(), affine_states=affine_states.tolist(),
                            nonlinear_states=nonlinear.tolist(), success=True)
                self.last_status = 'solved'
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                # Failure is explicit. Native LQR is available at every 5ms,
                # and fallback performance is not credited as successful MPC.
                self.last_command = None
                self.last_status = str(error)
                plan.update(success=False, failure=self.last_status)
            self.next_plan_time = now+self.settings.step_s
            row['mpc_total_wall_s'] = perf_counter()-start
            self.plan_rows.append(plan)
        row.update(mpc_success=self.last_command is not None, mpc_fallback=self.last_command is None,
                   mpc_plane_offset_m=self.plane.get('plane_offset_m', np.nan),
                   mpc_load_residual_N=self.plane.get('normal_load_residual_N', np.nan))
        command = np.asarray(native).copy() if self.last_command is None else self.last_command.copy()
        assert np.all(np.abs(command) <= plant.bounds+1e-10)
        return command, row


class TrajectoryPredictiveExperiment(PredictiveExperiment):
    """SQP refinement with nonlinear candidate screening at every update.

    Initial seeds are bounded command sequences, never imposed robot poses.
    Defaults retain one refinement; optional further steps continue from the
    best first-pass candidate. An optional native-policy seed uses predictions.
    Five Jacobian knots approximate twenty stages; nonlinear state evolution
    and candidate cost are evaluated at every stage. This is an offline
    successive-linearization prototype, not a hybrid-control guarantee.
    """

    def __init__(self, *args, jacobian_stride=5, seed_shaft_torque=.03,
                 refinement_steps=1, native_rollout_seed=False,
                 native_command_function=None, **kwargs):
        super().__init__(*args, **kwargs)
        if (isinstance(refinement_steps, (bool, np.bool_)) or
                not isinstance(refinement_steps, (int, np.integer)) or refinement_steps < 1):
            raise ValueError('refinement_steps must be a positive integer')
        self.sequence = None
        self.jacobian_stride = int(jacobian_stride)
        self.seed_shaft_torque = float(seed_shaft_torque)
        self.refinement_steps = int(refinement_steps)
        self.native_rollout_seed = bool(native_rollout_seed)
        self.native_command_function = native_command_function
        self.active_source_code = 0
        self.active_cost = np.nan

    def objective_parameters(self, controller, now, normal, q0, v0, transform):
        """Legacy objective unless an explicitly selected experiment overrides it."""
        return self.targets(controller, now, normal, q0, v0), {}, {}

    def evaluate(self, q0, v0, inputs, target, transform, bounds, previous,
                 target_inputs=None, terminal_weight=None, terminal_tail_weight=None):
        nq, nv = self.predictor.rollout(q0, v0, inputs, self.settings.step_s)
        states = np.asarray([self.predictor.tangent(q0, v0, q, v)
                             for q, v in zip(nq[1:], nv[1:])])
        residual = ((states-target)@transform.T)/np.asarray(self.settings.state_scales)
        weights = np.asarray(self.settings.state_weights)
        state_cost = float(np.sum(residual[:-1]**2*weights)+
                           self.settings.terminal_multiplier*np.sum(residual[-1]**2*weights))
        physical_residual = (states-target)@transform.T
        if terminal_weight is not None:
            state_cost = float(np.sum(residual[:-1]**2*weights)+
                               physical_residual[-1]@terminal_weight@physical_residual[-1])
        normalized = inputs/bounds
        differences = np.diff(np.vstack((previous/bounds, normalized)), axis=0)
        input_residual = normalized if target_inputs is None else (inputs-target_inputs)/bounds
        input_cost = float(np.sum(input_residual**2*np.asarray(self.settings.input_weight)))
        change_cost = float(np.sum(differences**2*np.asarray(self.settings.change_weight)))
        tail_cost = 0.
        if terminal_tail_weight is not None:
            tail = np.r_[physical_residual[-1], input_residual[-1]]
            tail_cost = float(tail@terminal_tail_weight@tail)
        cost = state_cost+input_cost+change_cost+tail_cost
        if not np.isfinite(cost):
            raise RuntimeError('nonfinite_nonlinear_candidate_cost')
        return states, cost, {'state_cost': state_cost, 'input_cost': input_cost,
                             'change_cost': change_cost, 'tail_cost': tail_cost}

    def command(self, controller, now, native, info):
        plant = controller.plant
        row = dict(mpc_active=True, mpc_updated=False, mpc_success=self.last_command is not None,
                   mpc_fallback=self.last_command is None, mpc_total_wall_s=0., mpc_qp_wall_s=0.,
                   mpc_linearization_wall_s=0., mpc_plan_nonlinear_xy_max_error_m=np.nan,
                   mpc_plan_nonlinear_rotation_max_error_rad=np.nan,
                   mpc_plane_offset_m=self.plane.get('plane_offset_m', np.nan),
                   mpc_load_residual_N=self.plane.get('normal_load_residual_N', np.nan),
                   mpc_nonlinear_cost=self.active_cost, mpc_plan_source_code=self.active_source_code,
                   mpc_qp_solves_this_update=0)
        if now >= self.next_plan_time-1e-9:
            start = perf_counter()
            row['mpc_updated'] = True
            q0, v0 = plant.data.qpos.copy(), plant.data.qvel.copy()
            normal = np.asarray(info['estimate']['normal']).copy()
            point = np.asarray(info['measurement']['point_m']).copy()
            load = float(sum(info['measurement']['loads_N'].values()))
            plan = dict(time_s=now, qpos=q0.tolist(), qvel=v0.tolist(),
                        normal=normal.tolist(), point_m=point.tolist(), load_N=load,
                        method='trajectory_SQP_nonlinear_screening', trials=[])
            try:
                if load < .05*controller.weight or normal[2] < .2:
                    raise RuntimeError('insufficient_estimated_support')
                self.plane = self.predictor.update(q0, v0, normal, point, load,
                    friction=info['estimate']['friction'], command=plant.last_input)
                usable = (abs(self.plane['normal_load_residual_N']) <= max(.02, self.load_residual_fraction*load)
                          and self.plane['load_fit_bracketed']
                          and abs(self.plane['plane_offset_m']) < self.plane['plane_offset_bound_m']-1e-7)
                plan.update(calibration=self.plane, load_model_usable=bool(usable),
                            load_residual_fraction=self.load_residual_fraction)
                if not usable:
                    raise RuntimeError('estimated_plane_load_fit_failed')
                transform = np.eye(18)
                transform[2, :3] = normal
                transform[11, 9:12] = normal
                inverse = np.linalg.inv(transform)
                target, objective, objective_diagnostics = self.objective_parameters(
                    controller, now, normal, q0, v0, transform)
                plan['objective'] = objective_diagnostics
                previous = plant.last_input.copy()
                first = self.sequence is None
                warm = (np.tile(previous, (self.settings.horizon_steps, 1)) if first else
                        np.vstack((self.sequence[1:], self.sequence[-1:])))
                seeds = [('warm_start', warm)]
                if first:
                    pulse_steps = max(1, round(.12/self.settings.step_s))
                    for sign in (-1, 1):
                        seed = warm.copy()
                        seed[:pulse_steps, 0] += sign*self.seed_shaft_torque
                        seeds.append((f'seed_shaft_{sign:+d}', np.clip(seed, -plant.bounds, plant.bounds)))
                if self.native_rollout_seed:
                    try:
                        seed, seed_diagnostics = generate_native_rollout_seed(
                            controller, self.predictor, self.path, now,
                            self.settings.horizon_steps, self.settings.step_s,
                            current_command=np.asarray(native).copy(),
                            command_function=self.native_command_function)
                        plan['native_rollout_seed'] = seed_diagnostics
                        seeds.append(('native_rollout', seed))
                    except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                        plan['native_rollout_seed_failure'] = str(error)
                plan['refinement_steps_requested'] = self.refinement_steps
                candidates = []
                qp_time, linearization_time, solved = 0., 0., 0
                for index in range(len(seeds)+self.refinement_steps-1):
                    if index < len(seeds):
                        label, nominal = seeds[index]
                        source = label
                        source_code = (1 if label == 'warm_start' else
                                       4 if label == 'native_rollout' else 2)
                        refinement = 1
                    else:
                        if not candidates:
                            break
                        previous_best = min(candidates, key=lambda candidate: candidate['cost'])
                        label = f'refine_{index-len(seeds)+2}'
                        nominal = previous_best['inputs'].copy()
                        source, source_code = previous_best['source'], previous_best['source_code']
                        refinement = index-len(seeds)+2
                    trial = {'seed': label, 'refinement_iteration': refinement}
                    try:
                        nominal_states, nominal_cost, costs = self.evaluate(q0, v0, nominal,
                            target, transform, plant.bounds, previous, **objective)
                        item = dict(inputs=nominal, states=nominal_states, cost=nominal_cost,
                                    costs=costs, source=source, source_code=source_code)
                        candidates.append(item)
                        stage_start = perf_counter()
                        A, B, f0, nominal_root_states = self.predictor.linearize_trajectory(
                            q0, v0, nominal, self.settings.step_s,
                            jacobian_stride=self.jacobian_stride)
                        linearization_time += perf_counter()-stage_start
                        item['model'] = (A, B, f0, nominal)
                        result = self.optimizer.solve(transform@A@inverse, transform@B,
                            f0@transform.T, target@transform.T, nominal, previous, plant.bounds,
                            **objective)
                        qp_time += result.diagnostics['total_time_s']
                        solved += int(result.success)
                        trial.update(solver=result.diagnostics, nominal_cost=nominal_cost)
                        if result.success:
                            trial['line_search'] = []
                            for alpha in (1., .5, .25):
                                inputs = nominal+alpha*(result.inputs-nominal)
                                states, cost, costs = self.evaluate(q0, v0, inputs, target,
                                    transform, plant.bounds, previous, **objective)
                                trial['line_search'].append({'alpha': alpha, 'nonlinear_cost': cost})
                                candidates.append(dict(inputs=inputs, states=states, cost=cost,
                                    costs=costs, source=f'SQP_{label}_alpha_{alpha:g}', source_code=3,
                                    model=(A, B, f0, nominal)))
                    except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                        trial['failure'] = str(error)
                    plan['trials'].append(trial)
                if not candidates:
                    raise RuntimeError('no_finite_bounded_nonlinear_plan')
                best = min(candidates, key=lambda candidate: candidate['cost'])
                self.sequence = best['inputs'].copy()
                self.last_command = self.sequence[0].copy()
                self.last_status = best['source']
                self.active_source_code, self.active_cost = best['source_code'], best['cost']
                row.update(mpc_nonlinear_cost=best['cost'], mpc_plan_source_code=best['source_code'],
                           mpc_qp_solves_this_update=solved, mpc_qp_wall_s=qp_time,
                           mpc_linearization_wall_s=linearization_time)
                plan.update(success=True, command_source=best['source'], qp_solved_count=solved,
                    target_states=target.tolist(), inputs=best['inputs'].tolist(),
                    nonlinear_states=best['states'].tolist(), nonlinear_cost=best['cost'],
                    cost_components=best['costs'], jacobian_stride=self.jacobian_stride,
                    seed_shaft_torque_Nm=self.seed_shaft_torque)
                if 'model' in best:
                    A, B, f0, nominal = best['model']
                    state, affine = np.zeros(18), []
                    for a, b, c, u, u0 in zip(A, B, f0, best['inputs'], nominal):
                        state = a@state+b@(u-u0)+c
                        affine.append(state)
                    affine = np.asarray(affine)
                    difference = best['states']-affine
                    row['mpc_plan_nonlinear_xy_max_error_m'] = float(np.max(np.linalg.norm(difference[:, :2], axis=1)))
                    row['mpc_plan_nonlinear_rotation_max_error_rad'] = float(np.max(np.linalg.norm(difference[:, 3:6], axis=1)))
                    plan.update(A=A.tolist(), B=B.tolist(), f0=f0.tolist(),
                                affine_states=affine.tolist(), model_anchor_inputs=nominal.tolist())
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                self.last_command = None
                self.active_source_code, self.active_cost = 0, np.nan
                self.last_status = str(error)
                plan.update(success=False, failure=self.last_status)
            self.next_plan_time = now+self.settings.step_s
            row['mpc_total_wall_s'] = perf_counter()-start
            self.plan_rows.append(plan)
        row.update(mpc_success=self.last_command is not None, mpc_fallback=self.last_command is None,
                   mpc_plane_offset_m=self.plane.get('plane_offset_m', np.nan),
                   mpc_load_residual_N=self.plane.get('normal_load_residual_N', np.nan),
                   mpc_plan_source_code=self.active_source_code, mpc_nonlinear_cost=self.active_cost)
        output = np.asarray(native).copy() if self.last_command is None else self.last_command.copy()
        assert np.all(np.abs(output) <= plant.bounds+1e-10)
        return output, row


def run_case(args):
    directory = args.output/'runs'/args.motion/args.variant
    if directory.exists():
        raise FileExistsError(f'Preserve earlier trials; choose a fresh output: {directory}')
    directory.mkdir(parents=True)
    provenance = snapshot(directory, args)
    workspace = GazeboWorkspace(load_config(), args.output/'_workspaces'/args.motion/args.variant,
        seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz', heightmap_cell_m=.04,
        plant_options={'visual_bounds': (-.6, .6, -.6, .6)})
    terrain = Terrain(name='side_slope_10', features=[{'type': 'plane', 'slope': [0., np.tan(np.deg2rad(10.))]}])
    path = ReferencePath(DISTANCE_M, 0. if args.motion == 'world_x' else 90., MOTION_S,
                         start_s=WARM_S, ramp_s=RAMP_S)
    params = OffroadParams(mode='hardware', rolling_allocation='least_squares', track_contact_point=False)
    original = OffroadController.command
    rows, activation = [], {}
    experiment = None
    component_ids = None

    def forbidden(*args, **kwargs):
        raise AssertionError('Controller attempted to read live simulator contacts')

    def command(controller, xy, velocity, acceleration):
        nonlocal experiment, component_ids
        mirror = controller.plant
        assert mirror.ground is None and mirror.terrain is None and mirror.data.ncon == 0
        ids = (id(controller.observer), id(controller.estimator), id(controller.synchronizer))
        if component_ids is None:
            component_ids = ids
        assert ids == component_ids
        now = float(mirror.data.time)
        if not activation and now >= WARM_S-1e-9:
            path.origin = mirror.data.qpos[:2].copy()
            xy[:] = path(now)[0]
            activation.update(time_s=now, original_goal_origin=path.origin.tolist(),
                              qpos=mirror.data.qpos.tolist(), qvel=mirror.data.qvel.tolist(),
                              physical_state_not_reset=True, live_estimator_preserved=True)
            if args.variant == 'mpc':
                from spherical_wheel.predictive_control import AffineMPCSettings
                settings = AffineMPCSettings(horizon_steps=args.horizon_steps, step_s=args.plan_step_s)
                if args.model_mode == 'trajectory':
                    experiment = TrajectoryPredictiveExperiment(mirror, path, settings,
                        args.load_residual_fraction, jacobian_stride=args.jacobian_stride,
                        seed_shaft_torque=args.seed_shaft_torque,
                        refinement_steps=args.refinement_steps,
                        native_rollout_seed=args.native_rollout_seed,
                        native_command_function=original)
                else:
                    experiment = PredictiveExperiment(mirror, path, settings, args.load_residual_fraction)
                provenance['mpc_settings'] = asdict(settings)
        before_time = float(controller.observer._previous[0])
        with ExitStack() as stack:
            for target in ('spherical_wheel.offroad_control.measure_contact',
                           'spherical_wheel.offroad_control.records_from_mujoco',
                           'spherical_wheel.offroad_control.rolling_feedforward',
                           'spherical_wheel.synchronization.ContactSynchronizer._normal_loads',
                           'mujoco.mj_contactForce'):
                stack.enter_context(patch(target, forbidden))
            native, info = original(controller, xy, velocity, acceleration)
            native_requested = info['requested'].copy()
            row = dict(mpc_active=False, mpc_updated=False, mpc_success=False, mpc_fallback=False,
                       mpc_total_wall_s=0., mpc_qp_wall_s=0., mpc_linearization_wall_s=0.,
                       mpc_plan_nonlinear_xy_max_error_m=np.nan,
                       mpc_plan_nonlinear_rotation_max_error_rad=np.nan,
                       mpc_plane_offset_m=np.nan, mpc_load_residual_N=np.nan,
                       mpc_nonlinear_cost=np.nan, mpc_plan_source_code=0,
                       mpc_qp_solves_this_update=0)
            applied = native.copy()
            if experiment is not None:
                applied, update = experiment.command(controller, now, native, info)
                row.update(update)
                info['requested'] = applied.copy()
                info['actuator_saturated'] = np.abs(applied) > mirror.bounds+1e-12
                info['clipping_command_delta'] = np.zeros(5)
            assert float(controller.observer._previous[0]) == now
            assert now == 0. or before_time < now
            assert mirror.data.ncon == 0
        row.update(mpc_native_requested=native_requested, mpc_native_applied=native.copy(),
                   mpc_command_delta=applied-native_requested,
                   mpc_actual_input_at_limit=np.abs(applied) >= .99*mirror.bounds,
                   mpc_original_goal_xy=np.asarray(xy).copy(),
                   exact_qpos=mirror.data.qpos.copy(), exact_qvel=mirror.data.qvel.copy(),
                   exact_observer_wrench=np.r_[controller.observer.wrench()],
                   exact_applied=applied.copy(), time_s=now)
        rows.append(row)
        if len(rows) % 100 == 0:
            print(json.dumps({'variant': args.variant, 'motion': args.motion, 'time_s': now,
                              'mpc_status': experiment.last_status if experiment else 'baseline'}), flush=True)
        return applied, info

    duration = WARM_S+MOTION_S+args.recovery_s
    try:
        with patch.object(OffroadController, 'command', command):
            log, summary = run_gazebo(workspace, terrain, path, params, phase=PHASE,
                duration_s=duration, record_diagnostics=True)
        extra = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
        log.update({key: value for key, value in extra.items() if key.startswith('mpc_')})
        active = log['time_s'] >= WARM_S-1e-9
        actuated = active.copy()
        actuated[-1] = False
        error = np.linalg.norm(log['qpos'][:, :2]-log['reference_xy'], axis=1)
        goal = path(WARM_S+MOTION_S+10.)[0]
        goals = np.linalg.norm(log['qpos'][:, :2]-goal, axis=1)
        plans = [] if experiment is None else experiment.plan_rows
        summary['mpc_experiment'] = {'variant': args.variant, 'motion': args.motion,
            'activation': activation, 'plans_generated_including_terminal': len(plans),
            'accepted_plans': sum(p.get('success', False) for p in plans),
            'qp_success_definition': 'Affine: solved QP and finite replay. Trajectory: best bounded finite nonlinear sequence among warm/seed/SQP line-search plans; QP counts separately in trials.',
            'fallback_actuated_active_samples': int(np.sum(log['mpc_fallback'][actuated])),
            'guarded_controller_calls': len(rows), 'original_goal_error_final_mm': float(1000*goals[-1]),
            'original_goal_progress_final_mm': float(1000*((log['qpos'][-1, :2]-path.origin)@path.direction)),
            'active_path_error_rms_mm': float(1000*np.sqrt(np.mean(error[active]**2))) if active.any() else None,
            'active_peak_reference_lean_deg': float(np.degrees(log['attitude_error_rad'][active].max())) if active.any() else None,
            'active_lean_error_integral_deg_s': float(np.trapz(np.degrees(log['attitude_error_rad'][active]), log['time_s'][active])) if active.any() else None,
            'active_input_limit_occupancy_fraction': float(np.any(log['mpc_actual_input_at_limit'][actuated], axis=1).mean()) if actuated.any() else None,
            'mpc_update_time_median_s': float(np.median(log['mpc_total_wall_s'][log['mpc_updated']])) if np.any(log['mpc_updated']) else None,
            'scope': ('Successive trajectory linearization with nonlinear candidate screening, estimated private-plane dynamics, and sparse Jacobian knots; no hard-tilt or real-time qualification.' if args.model_mode == 'trajectory' else 'Local affine constrained MPC with estimated private-plane dynamics. Not nonlinear configuration planning, hard-tilt safety, or real-time qualification.'),
            'diagnostic_components': 'Native LQR/FF/sync fields are shadows in MPC mode; mpc_command_delta added to their native requested sum yields actual final input.'}
        save_run(directory, log, summary, terrain)
        np.savez_compressed(directory/'precision.npz', **extra)
        (directory/'plans.json').write_text(json.dumps(diagnostic_json(plans), indent=2, allow_nan=False)+'\n')
        provenance['generated_sha256'] = {p.name: digest(p) for p in
            (directory/'rollout.npz', directory/'precision.npz', directory/'plans.json', directory/'summary.json')}
        provenance['gain_cache_sha256'] = digest(workspace.cache_path)
        provenance['engine'] = summary['engine']
        (directory/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
        print(json.dumps(summary['mpc_experiment']), flush=True)
    finally:
        workspace.plant(terrain).close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', choices=('baseline', 'mpc'), required=True)
    parser.add_argument('--motion', choices=('world_x', 'world_y'), required=True)
    parser.add_argument('--recovery-s', type=float, default=4.)
    parser.add_argument('--plan-step-s', type=float, default=.02)
    parser.add_argument('--horizon-steps', type=int, default=20)
    parser.add_argument('--load-residual-fraction', type=float, default=.002,
                        help='Static load-fit quality gate; .10 is the separately validated force-gap trial')
    parser.add_argument('--model-mode', choices=('affine', 'trajectory'), default='affine')
    parser.add_argument('--jacobian-stride', type=int, default=5)
    parser.add_argument('--seed-shaft-torque', type=float, default=.03)
    parser.add_argument('--refinement-steps', type=int, default=1)
    parser.add_argument('--native-rollout-seed', action='store_true')
    args = parser.parse_args()
    args.output = args.output.resolve()
    if not np.isfinite(args.recovery_s) or args.recovery_s < 0:
        parser.error('Recovery must be finite and nonnegative')
    if not np.isfinite(args.load_residual_fraction) or not 0 < args.load_residual_fraction <= .1:
        parser.error('Load residual fraction must lie in (0, .1]')
    if args.jacobian_stride < 1 or not np.isfinite(args.seed_shaft_torque) or not 0 <= args.seed_shaft_torque <= .1:
        parser.error('Jacobian stride must be positive; seed torque must lie in [0, .1] Nm')
    if args.refinement_steps < 1:
        parser.error('Refinement steps must be positive')
    if args.model_mode != 'trajectory' and (args.native_rollout_seed or args.refinement_steps != 1):
        parser.error('Native rollout seeds and repeated refinements require trajectory model mode')
    if args.horizon_steps < 1 or not np.isfinite(args.plan_step_s) or args.plan_step_s <= 0:
        parser.error('Horizon and plan period must be positive')
    control_ratio = args.plan_step_s/.005
    if not np.isclose(control_ratio, round(control_ratio), atol=1e-10, rtol=0):
        parser.error('Plan period must be an integer multiple of the .005 s sensing/control interval')
    if args.variant == 'mpc' and find_spec('osqp') is None:
        parser.error("MPC requires the optional solver: python -m pip install -e '.[mpc]'")
    run_case(args)


if __name__ == '__main__':
    main()
