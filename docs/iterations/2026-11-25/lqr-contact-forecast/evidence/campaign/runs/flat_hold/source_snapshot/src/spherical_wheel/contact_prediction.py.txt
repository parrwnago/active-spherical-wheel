"""Diagnostic contact forecasts under the unchanged native feedback policy.

Current observer/controller history is copied into a contact-free mirror. The
copy receives only private predicted states and its own prior applied inputs;
solved private contacts are returned for evaluation, never fed to that policy.
The fitted plane and friction stay fixed while the copied observer and filters
continue updating. Commands are retained at the native 5 ms cadence, without
the coarse averaging used by the separate MPC initialization helper.
"""
from __future__ import annotations

import copy
from time import perf_counter

import mujoco
import numpy as np

from .offroad_control import OffroadController
from .predictive_model import EstimatedPlanePredictor
from .predictive_seed import clone_native_controller


PATCH_LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
_NATIVE_COMMAND = OffroadController.command
# Keep evaluation of PRIVATE contacts separate from guards on live APIs. The
# helper below always supplies the predictor's own model and data to this call.
_PRIVATE_CONTACT_FORCE = mujoco.mj_contactForce


def _contact_snapshot(predictor, instant, threshold, force_offset_s):
    model, data = predictor.model, predictor.data
    loads = dict.fromkeys(PATCH_LABELS, 0.)
    weighted = {name: np.zeros(3) for name in PATCH_LABELS}
    nonwheel = {}
    records = []
    force = np.empty(6)
    for number, contact in enumerate(data.contact):
        if predictor.floor_id not in contact.geom:
            continue
        geom = int(contact.geom[1] if contact.geom[0] == predictor.floor_id else contact.geom[0])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or f'geom_{geom}'
        patch = next((label for label in PATCH_LABELS
                      if name == label or name.startswith(label + '_')), None)
        load = 0.
        if contact.efc_address >= 0:
            _PRIVATE_CONTACT_FORCE(model, data, number, force)
            load = max(0., float(force[0]))
        if not np.isfinite(load):
            raise RuntimeError('Private contact forecast produced a nonfinite force')
        position = np.asarray(contact.pos, float).copy()
        if patch is None:
            nonwheel[name] = nonwheel.get(name, 0.) + load
        else:
            loads[patch] += load
            weighted[patch] += load * position
        records.append({'geom_name': name, 'patch': patch,
                        'position_m': position.tolist(), 'normal_force_N': load,
                        'distance_m': float(contact.dist)})
    points = {name: (weighted[name] / loads[name]).tolist() if loads[name] > 0 else None
              for name in PATCH_LABELS}
    total = float(sum(loads.values()))
    point = sum(weighted.values(), start=np.zeros(3)) / total if total > 0 else None
    loaded = [name for name in PATCH_LABELS if loads[name] > threshold]
    dominant = max(loaded, key=loads.__getitem__) if loaded else 'none'
    return {'time_s': float(instant), 'loads_N': loads, 'points_m': points,
            'point_m': None if point is None else point.tolist(),
            'dominant_patch': dominant, 'loaded_patches': loaded,
            'total_wheel_load_N': total,
            'total_nonwheel_load_N': float(sum(nonwheel.values())),
            'max_nonwheel_geom_load_N': float(max(nonwheel.values(), default=0.)),
            'nonwheel_geoms': nonwheel, 'geometric_contact_count': len(records),
            'contacts': records, 'load_threshold_N': float(threshold),
            'force_sample_time_s': float(instant + force_offset_s),
            'force_sample_offset_from_state_s': float(force_offset_s)}


def forecast_native_contacts(controller, predictor, path, now, horizon_s=.2, *,
                             current_command=None, command_function=None,
                             load_threshold_N=.05):
    """Return fine states, native inputs and private contact evaluation records.

    ``predictor`` must already be fitted using current estimated point, normal,
    load and friction. ``path(t)`` supplies reference position/velocity/
    acceleration, not future measurements. If the live command has already
    updated the observer at ``now``, pass it as ``current_command``; otherwise
    only the clone computes that anchor command. An explicit unbound original
    ``command_function`` bypasses any live experiment monkeypatch.

    State/time arrays have N+1 rows and commands N rows. Contact row zero is a
    fresh private ``mj_forward`` at the anchor under the preceding input.
    Advanced contact rows are the last solver step of each 5 ms transition,
    preceding its returned integrated state by one model physics step. This
    matches the existing predictor's reset-per-transition integration semantics;
    it does not assert exact force/state simultaneity. No future true terrain
    or true contacts are accepted. The source controller is never advanced.
    """
    started = perf_counter()
    if not isinstance(predictor, EstimatedPlanePredictor):
        raise TypeError('A configured private EstimatedPlanePredictor is required')
    if not predictor._ready:
        raise RuntimeError('Configure the estimated prediction plane before forecasting')
    if (not np.isfinite(now) or isinstance(horizon_s, (bool, np.bool_)) or
            not np.isfinite(horizon_s) or horizon_s <= 0):
        raise ValueError('now must be finite and horizon_s finite and positive')
    if not np.isfinite(load_threshold_N) or load_threshold_N <= 0:
        raise ValueError('load_threshold_N must be finite and positive')
    dt = float(controller.plant.dt)
    if not np.isclose(dt, .005, atol=1e-12, rtol=0):
        raise ValueError('The diagnostic forecast requires the native 5 ms feedback interval')
    steps = int(round(horizon_s / dt))
    if steps < 1 or not np.isclose(steps * dt, horizon_s, atol=1e-12, rtol=0):
        raise ValueError('horizon_s must be a positive integer multiple of 5 ms')
    if not np.isclose(controller.plant.data.time, now, atol=1e-8, rtol=0):
        raise ValueError('now must match the measured controller state time')
    if not np.array_equal(predictor.bounds, controller.plant.bounds):
        raise ValueError('Native policy and predictor input bounds must agree')
    if (predictor.model is controller.plant.model or
            predictor.data is controller.plant.data):
        raise ValueError('Prediction model and data must be private')
    native = _NATIVE_COMMAND if command_function is None else command_function
    if not callable(path) or not callable(native):
        raise ValueError('path and command_function must be callable')
    clone = clone_native_controller(controller)
    observer_time = (None if controller.observer._previous is None else
                     float(controller.observer._previous[0]))
    if current_command is None and observer_time is not None and observer_time >= now - 1e-12:
        raise ValueError('Pass current_command when native feedback already observed this time')
    if current_command is not None and (observer_time is None or abs(observer_time - now) > 1e-8):
        raise ValueError('current_command requires native observer history updated at now')
    plant = clone.plant
    bounds = plant.bounds.copy()

    def bounded(value):
        value = np.asarray(value, float)
        if (value.shape != (5,) or not np.isfinite(value).all() or
                np.any(abs(value) > bounds + 1e-10)):
            raise ValueError('Native commands must be five finite bounded physical inputs')
        return value.copy()

    first = None if current_command is None else bounded(current_command)
    anchor_preview = copy.deepcopy(clone._sync) if first is not None else None
    plane_point, normal = predictor._plane_point.copy(), predictor.normal.copy()
    friction = predictor.model.geom_friction[predictor.floor_id].copy()
    qpos, qvel = plant.data.qpos.copy(), plant.data.qvel.copy()
    times = now + np.arange(steps + 1) * dt
    predictor._assign(qpos, qvel, plant.last_input)
    contacts = [_contact_snapshot(predictor, times[0], load_threshold_N, 0.)]
    positions, velocities, commands = [qpos.copy()], [qvel.copy()], []
    physics_dt = float(predictor.model.opt.timestep)
    for index in range(steps):
        if plant.data.ncon or plant.ground is not None or plant.terrain is not None:
            raise RuntimeError('Predicted native feedback must use a contact-free measurement mirror')
        if index == 0 and first is not None:
            command = first
        else:
            xy, velocity, acceleration = path(float(times[index]))
            command, _ = native(clone, np.asarray(xy, float).copy(),
                                np.asarray(velocity, float).copy(),
                                np.asarray(acceleration, float).copy())
            command = bounded(command)
        if index == 0 and anchor_preview is None:
            anchor_preview = copy.deepcopy(clone._sync)
        commands.append(command.copy())
        qpos, qvel = predictor.transition(qpos, qvel, command, dt)
        contacts.append(_contact_snapshot(predictor, times[index + 1],
                                          load_threshold_N, -physics_dt))
        positions.append(qpos.copy())
        velocities.append(qvel.copy())
        plant.set_state(qpos, qvel, float(times[index + 1]), command)
        clone.after_step()
    if (not np.array_equal(predictor._plane_point, plane_point) or
            not np.array_equal(predictor.normal, normal) or
            not np.array_equal(predictor.model.geom_friction[predictor.floor_id], friction)):
        raise RuntimeError('The forecast changed its frozen estimated plane or friction')
    loads = np.array([[row['loads_N'][name] for name in PATCH_LABELS] for row in contacts])
    points = np.full((steps + 1, len(PATCH_LABELS), 3), np.nan)
    for i, row in enumerate(contacts):
        for j, name in enumerate(PATCH_LABELS):
            if row['points_m'][name] is not None:
                points[i, j] = row['points_m'][name]
    return {'time_s': times, 'qpos': np.asarray(positions), 'qvel': np.asarray(velocities),
            'commands': np.asarray(commands), 'contacts': contacts,
            'patch_labels': list(PATCH_LABELS), 'patch_loads_N': loads,
            'patch_points_m': points, 'anchor_kinematic_preview': anchor_preview,
            'metadata': {
                'method': 'cloned_native_5ms_feedback_private_contact_forecast',
                'native_interval_s': dt, 'horizon_s': float(horizon_s), 'fine_steps': steps,
                'first_command_source': ('already_computed_native' if first is not None
                                         else 'computed_on_clone'),
                'observer_history_time_s': observer_time,
                'frozen_plane': {'estimated_point_m': predictor.point.copy(),
                                 'calibrated_point_m': plane_point,
                                 'normal': normal, 'friction': friction,
                                 'friction_coefficient': float(friction[0])},
                'contact_force_timing': 'Anchor mj_forward at current state; future endpoint records are from the final physics solver step before integration.',
                'advanced_contact_force_offset_s': -physics_dt,
                'integration': 'Existing predictor transition resets integration/warm-start fields once per 5 ms feedback interval.',
                'measurement_boundary': 'Current measured state and observer/controller history only. The clone observes private predicted states and its own preceding inputs; solved private contacts are evaluation outputs only. The plane/friction stay frozen, while clone observer/filter estimates update.',
                'command_averaging': False, 'total_wall_s': float(perf_counter() - started)}}
