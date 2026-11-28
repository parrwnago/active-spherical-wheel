"""I78 result-owned run loop, copied from native run_offroad.

Documented changes only: mirror follows before governed reference selection;
selected references are confirmed after one live native command; terminate at
first reference completion plus ten physical seconds or the fixed budget.
The native branch is required to reproduce every original precision field.
"""
import mujoco
import numpy as np
import spherical_wheel.offroad_experiments as experiments
from spherical_wheel.offroad_experiments import (
    OffroadParams, OffroadController, place_on_ground, measure_for, summarize,
    _LOG_KEYS, _DIAGNOSTIC_KEYS, _UNOBSERVED, SUPPORT_NAMES)

ACTIVE_SESSION = None

def run_offroad(workspace, terrain, path, params=None, duration_s=None, phase=None, pushes=(),
                balanced_lean=True, settle_s=3., yaw_deg=0., record_diagnostics=False):
    """Simulate one run and return ``(log, summary)``.

    ``pushes`` are ``(start_s, duration_s, [Fx, Fy, Fz])`` world forces on the
    fork, as in the lab. ``settle_s`` extends the run past the end of the path.
    The log is aligned as in the other experiments: commands logged at time t
    are held until the next row.

    A ``hardware`` controller is given the workspace's ``StateMirror``, which
    follows the simulated robot's state before every call. Its contact
    estimates stay in its diagnostics; the log's contact columns are the
    solver's in every mode, so they measure what really happened.

    ``record_diagnostics`` adds estimated contacts, measured rolling-consistency
    residuals, and separate command contributions without feeding them back.
    These additional fields currently require ``hardware`` mode.
    """
    params = params or OffroadParams()
    plant = workspace.plant(terrain)
    phase = plant.config['simulation'].get('initial_shaft_phase_rad', .30) if phase is None else phase
    controller = OffroadController(workspace.mirror() if params.mode == 'hardware' else plant,
                                   workspace.design, params, phase, record_diagnostics=record_diagnostics)
    place_on_ground(plant, controller, path.origin, phase, balanced_lean, yaw_deg)
    duration = float(duration_s if duration_s is not None else path.start_s+path.duration_s+settle_s)
    if ACTIVE_SESSION is not None:
        duration = ACTIVE_SESSION.requested_horizon_s
    count = int(round(duration/plant.dt))
    keys = _LOG_KEYS+_DIAGNOSTIC_KEYS if record_diagnostics else _LOG_KEYS
    log = {key: [] for key in keys}
    failure = None
    previous_requested = plant.last_input.copy()
    for index in range(count+1):
        now = float(plant.data.time)
        if ACTIVE_SESSION is None:
            xy, velocity, acceleration = path(now)
            measure_for(controller, plant)
        else:
            measure_for(controller, plant)
            xy, velocity, acceleration = ACTIVE_SESSION.select(
                controller, plant, path, now)
        applied, diagnostics = controller.command(xy, velocity, acceleration)
        if ACTIVE_SESSION is not None:
            ACTIVE_SESSION.confirm(controller, applied, diagnostics, (xy, velocity, acceleration))
        push = sum((np.asarray(force, float) for start, length, force in pushes if start <= now < start+length-1e-9),
                   start=np.zeros(3))
        estimate = diagnostics['estimate']
        measurement = diagnostics['measurement'] if controller.plant is plant else experiments.measure_contact(plant)
        snapshot = plant.snapshot()
        body_up = plant.data.xmat[plant.fork_id].reshape(3, 3)[:, 2]
        reference_matrix = np.empty(9)
        mujoco.mju_quat2Mat(reference_matrix, diagnostics['reference_quat'])
        reference_up = reference_matrix.reshape(3, 3)[:, 2]
        quat = plant.data.qpos[3:7]
        row = {'time_s': now, 'qpos': plant.data.qpos.copy(), 'qvel': plant.data.qvel.copy(),
               'command': diagnostics['requested'], 'input': applied.copy(), 'reference_xy': np.asarray(xy, float).copy(),
               'reference_quat': diagnostics['reference_quat'], 'tilt_rad': snapshot['tilt_rad'],
               'attitude_error_rad': float(np.arccos(np.clip(body_up@reference_up, -1., 1.))), 'push_N': push,
               'ground_normal': plant.ground['normal'].copy(), 'estimated_normal': estimate['normal'],
               'ground_height_m': float(plant.ground['point'][2]), 'ground_friction': plant.ground['friction'],
               'estimated_friction': estimate['friction'], 'contact_force_N': measurement['force_N'],
               'normal_force_N': measurement['normal_force_N'],
               'tangential_force_N': measurement['tangential_force_norm_N'],
               'utilization': measurement['utilization'], 'slip_speed_m_s': measurement['slip_speed_m_s'],
               'support_loads_N': np.array([measurement['loads_N'][name] for name in SUPPORT_NAMES]),
               'effective_phase_rad': diagnostics['effective_phase_rad'], 'drive_fade': diagnostics['drive_fade'],
               'support_torque': diagnostics['support_torque'][:3],
               'heading_rad': float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]), 1-2*(quat[2]**2+quat[3]**2))),
               'fork_load_N': measurement['fork_load_N'],
               # Ground normal seen from the carrier: where on the wheel the support point is.
               'support_direction': plant.data.xmat[controller.carrier_id].reshape(3, 3).T@plant.ground['normal'],
               'contact_point_m': np.asarray(measurement['point_m'], float).copy(),
               # Solved moment about the wheel centre, and the observer's force and
               # moment when the controller uses it (NaN otherwise).
               'contact_moment_Nm': np.asarray(measurement['moment_Nm'], float).copy(),
               'observer_force_N': (diagnostics['observed']['force_N'].copy() if diagnostics.get('observed')
                                    else _UNOBSERVED.copy()),
               'observer_moment_Nm': (diagnostics['observed']['moment_Nm'].copy() if diagnostics.get('observed')
                                      else _UNOBSERVED.copy())}
        if record_diagnostics:
            row.update({key: diagnostics[key] for key in _DIAGNOSTIC_KEYS
                        if key not in ('previous_applied_input', 'previous_actuator_saturated')})
            row['previous_applied_input'] = plant.last_input.copy()
            row['previous_actuator_saturated'] = np.abs(previous_requested) > plant.bounds+1e-12
        for key in keys:
            log[key].append(row[key])
        if snapshot['balance_failed']:
            failure = 'fork or platform touched the ground'
        elif snapshot['tilt_rad'] > np.deg2rad(70):
            failure = 'tilt exceeded 70 degrees'
        elif not np.all(np.isfinite(plant.data.qpos)):
            failure = 'nonfinite state'
        elif np.linalg.norm(plant.data.qpos[:2]-xy) > 1.:
            failure = 'more than 1 m from the reference'
        governed_end = (ACTIVE_SESSION is not None and
                        ACTIVE_SESSION.should_stop(now))
        if failure or index == count or governed_end:
            break
        plant.step(applied, np.r_[push, np.zeros(3)])
        controller.after_step()
        previous_requested = diagnostics['requested'].copy()
    log = {key: np.asarray(value) for key, value in log.items()}
    requested_stop = (duration if ACTIVE_SESSION is None else
                      ACTIVE_SESSION.planned_stop_s)
    summary = summarize(log, plant, path, params, failure, requested_stop)
    return log, summary

