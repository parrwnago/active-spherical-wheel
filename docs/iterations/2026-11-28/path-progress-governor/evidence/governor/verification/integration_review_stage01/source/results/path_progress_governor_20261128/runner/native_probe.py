"""Current-state native-command trials on an isolated persistent controller.

No prediction or integration occurs. Each command has exactly one full-speed
reference probe beginning from live measured state and numerical history.
Reference-independent stages are checked against the sole live command; all
outputs are checked when governed and probed references are identical.
"""
from __future__ import annotations

import copy
import hashlib
import mujoco
import numpy as np
from unittest.mock import patch

from spherical_wheel.predictive_seed import clone_native_controller


CONTROL_HISTORY = ('nominal_joints', '_vref', '_held', '_sync',
                   'heading_reference', '_slow_normal', '_fall_end',
                   '_yaw_error', '_plane')
OBSERVER_HISTORY = ('generalized', 'interval_mean', '_previous')


def _canonical(value):
    if isinstance(value, dict):
        return tuple((key, _canonical(value[key])) for key in sorted(value))
    if isinstance(value, (tuple, list)):
        return tuple(_canonical(item) for item in value)
    if value is None or isinstance(value, (bool, str, int, float)):
        return value
    value = np.asarray(value)
    return value.dtype.str, value.shape, np.ascontiguousarray(value).tobytes()


def history_fingerprint(controller):
    """Selected complete native numerical history, not a solver-state dump."""
    value = {name: getattr(controller, name) for name in CONTROL_HISTORY}
    value['estimator'] = vars(controller.estimator)
    value['observer'] = {name: getattr(controller.observer, name)
                         for name in OBSERVER_HISTORY}
    value['synchronizer_plane'] = controller.synchronizer._plane
    value['mirror_qpos'] = controller.plant.data.qpos
    value['mirror_qvel'] = controller.plant.data.qvel
    value['mirror_time'] = float(controller.plant.data.time)
    value['mirror_previous_applied'] = controller.plant.last_input
    return hashlib.sha256(repr(_canonical(value)).encode()).hexdigest()


def exact_array(value):
    value = np.asarray(value)
    return value.dtype.str, value.shape, np.ascontiguousarray(value).tobytes()


class NativeCommandProbe:
    """One isolated clone reused for current-state reference alternatives.

    The existing native clone helper supplies independent models, MjData,
    observer and synchronizer. The same read-only design library is shared.
    Native hardware/least_squares is the only permitted control configuration.
    """

    def __init__(self, live, command_function):
        if (live.params.tilt_reference is not None or live.rolling_allocator is not None
                or live.motion_reference is not None or live.params.heading_gain != 0):
            raise ValueError('Probe is restricted to unchanged selected native N')
        self.live = live
        self.shadow = clone_native_controller(live)
        self.command_function = command_function
        self.calls = 0
        self.last = None
        self.last_reference = None
        self._immutable_names = set(vars(live)) - set(CONTROL_HISTORY)

    def refresh(self):
        live, shadow = self.live, self.shadow
        if set(vars(live)) - set(CONTROL_HISTORY) != self._immutable_names:
            raise AssertionError('Native controller state schema changed')
        shadow.plant.set_state(live.plant.data.qpos, live.plant.data.qvel,
                               live.plant.data.time, live.plant.last_input)
        for name in CONTROL_HISTORY:
            setattr(shadow, name, copy.deepcopy(getattr(live, name)))
        shadow.estimator.__dict__.clear()
        shadow.estimator.__dict__.update(copy.deepcopy(vars(live.estimator)))
        for name in OBSERVER_HISTORY:
            setattr(shadow.observer, name, copy.deepcopy(getattr(live.observer, name)))
        shadow.synchronizer._plane = copy.deepcopy(live.synchronizer._plane)
        if history_fingerprint(shadow) != history_fingerprint(live):
            raise AssertionError('Probe refresh does not match live native history')

    def evaluate(self, xy, velocity, acceleration):
        before = history_fingerprint(self.live)
        self.refresh()
        native_update = self.shadow.observer.update
        updates = []
        def updated(*args, **kwargs):
            updates.append(1)
            return native_update(*args, **kwargs)
        with patch.object(self.shadow.observer, 'update', updated):
            applied, info = self.command_function(self.shadow,
                np.asarray(xy, float).copy(), np.asarray(velocity, float).copy(),
                np.asarray(acceleration, float).copy())
        if len(updates) != 1:
            raise AssertionError('Exactly one private observer update per nominal probe')
        if history_fingerprint(self.live) != before:
            raise AssertionError('Reference trial mutated live native history')
        self.calls += 1
        self.last_reference = tuple(np.asarray(value, float).copy()
                                    for value in (xy, velocity, acceleration))
        records = self.shadow.locator.records(self.shadow.plant.data,
                                             *self.shadow.observer.wrench())
        matrices, loads = [], []
        for record in records:
            jacobian = np.zeros((3, self.shadow.plant.model.nv))
            mujoco.mj_jac(self.shadow.plant.model, self.shadow.plant.data,
                jacobian, None, record['pos'], int(record['body']))
            matrix = np.zeros((2, 5))
            matrix[:, :3] = jacobian[:2, 6:9]
            if record['name'].startswith('belt_'):
                side = 0 if record['name'].startswith('belt_A') else 1
                matrix[:, 3+side] = self.shadow.plant.data.geom_xmat[
                    int(record['geom'])].reshape(3, 3)[:2, 1]
            matrices.append(matrix)
            loads.append(max(0., float(record['normal_force'])))
        loads = np.asarray(loads, float)
        shares = loads/loads.sum() if loads.size and loads.sum()>0 else np.zeros_like(loads)
        authority = (np.einsum('i,ijk->jk', shares, np.asarray(matrices))
                     if matrices else np.zeros((2, 5)))
        singular = np.linalg.svd(authority*np.array([5.,5.,5.,.3,.3]),
                                 compute_uv=False)
        full = np.asarray(matrices).reshape(-1,5)
        stack_singular = np.linalg.svd(full*np.array([5.,5.,5.,.3,.3]),
                                      compute_uv=False) if matrices else np.empty(0)
        padded_singular = np.full(5, np.nan)
        padded_singular[:len(stack_singular)] = stack_singular
        rank = int(np.sum(stack_singular>stack_singular[0]*1e-7)) if len(stack_singular) else 0
        native_drive = np.asarray(info['allocation_drive_reference'])
        residual = full@native_drive+np.tile(np.asarray(velocity,float),len(records))
        info['i78_probe'] = dict(authority_matrix=authority,
            authority_scaled_singular_values=singular,
            authority_min_scaled_sigma=float(singular[-1]),
            full_stack_scaled_singular_values=padded_singular,
            full_stack_rank=rank,
            full_stack_condition_resolved=(float(stack_singular[0]/stack_singular[rank-1])
                                           if rank else np.nan),
            full_stack_requested_rolling_residual_norm_m_s=float(np.linalg.norm(residual)),
            nominal_joint_reference_max_abs_rad_s=float(np.max(
                np.abs(info['allocation_drive_reference'][:3]))),
            nominal_total_request_max_fraction=float(np.max(
                np.abs(info['requested'])/self.shadow.plant.bounds)),
            record_count=len(records), probe_control_updates=len(updates),
            probe_dynamic_steps=0)
        self.last = (np.array(applied, copy=True), copy.deepcopy(info),
                     history_fingerprint(self.shadow),
                     copy.deepcopy(vars(self.shadow.estimator)),
                     {name: copy.deepcopy(getattr(self.shadow.observer, name))
                      for name in OBSERVER_HISTORY})
        return np.array(applied, copy=True), copy.deepcopy(info)

    def confirm_live(self, applied, info, reference):
        if self.last is None:
            raise AssertionError('One nominal reference probe is required')
        expected_applied, expected_info, expected_history, estimator, observer = self.last
        if _canonical(vars(self.live.estimator)) != _canonical(estimator):
            raise AssertionError('Live/probe normal-filter history differs')
        for name in OBSERVER_HISTORY:
            if _canonical(getattr(self.live.observer, name)) != _canonical(observer[name]):
                raise AssertionError('Live/probe observer history differs: ' + name)
        for key in ('feedback_gain', 'support_command', 'estimated_support_loads_N',
                    'estimated_contact_point_m', 'effective_phase_rad', 'drive_fade'):
            if exact_array(info[key]) != exact_array(expected_info[key]):
                raise AssertionError('Reference-independent live/probe stage differs: ' + key)
        identical = all(exact_array(a)==exact_array(b)
                        for a,b in zip(reference, self.last_reference))
        if not identical:
            return False
        if exact_array(applied) != exact_array(expected_applied):
            raise AssertionError('Identical-reference applied input differs from probe')
        for key in ('requested', 'reference_qpos', 'reference_qvel',
                    'feedback_reference_qvel', 'feedback_gain', 'error',
                    'equilibrium_command', 'rolling_feedforward', 'support_command',
                    'lqr_command', 'transition_command_delta', 'heading_command',
                    'fade_command_delta', 'clipping_command_delta', 'sync_weights',
                    'estimated_support_loads_N', 'estimated_contact_point_m',
                    'allocation_drive_reference', 'reference_quat',
                    'reference_acceleration_m_s2', 'feedback_reference_xy'):
            if exact_array(info[key]) != exact_array(expected_info[key]):
                raise AssertionError('Live/selected-shadow output differs: ' + key)
        if history_fingerprint(self.live) != expected_history:
            raise AssertionError('Identical-reference live/probe numerical history differs')
        return True
