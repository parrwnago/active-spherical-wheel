"""Shadow audit of already-available observer wrench timing and support routing.

The live controller is never changed.  Interval-mean and filtered wrench use
the same current-pose SupportLocator.  The counterfactual changes only the
support shares supplied to scheduled feedback/holding torque, while retaining
the native normal, reference/error, feedforward and dropout decision.
"""
from __future__ import annotations

import hashlib
import json

import numpy as np

from .contact_sensing import intrinsic_normal


LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
FALLBACK_CODES = {0: 'interval_shares_used', 1: 'native_not_touching',
                  2: 'native_total_at_or_below_0.001N',
                  3: 'interval_total_at_or_below_0.001N', 4: 'invalid_interval_support'}


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def digest(value):
    return hashlib.sha256(json.dumps(_jsonable(value), sort_keys=True).encode()).hexdigest()


def history_fingerprint(controller):
    """Observer/filter/reference/synchronizer history and measured state, no truth."""
    plant = controller.plant
    value = dict(qpos=plant.data.qpos, qvel=plant.data.qvel, time_s=plant.data.time,
                 last_input=plant.last_input, ctrl=plant.data.ctrl,
                 surfacevel=plant.model.geom_surfacevel,
                 controller={name: getattr(controller, name, None) for name in
                    ('nominal_joints', '_held', '_sync', '_vref', '_plane', '_slow_normal',
                     '_fall_end', '_yaw_error', 'heading_reference')})
    observer = controller.observer
    value['observer'] = dict(generalized=observer.generalized, interval_mean=observer.interval_mean,
                             previous=observer._previous, mass=observer._mass,
                             qpos=observer._data.qpos, qvel=observer._data.qvel)
    value['estimator'] = vars(controller.estimator)
    sync = controller.synchronizer
    value['synchronizer'] = dict(plane=sync._plane, qpos=sync._data.qpos,
                                  qvel=sync._data.qvel, time_s=sync._data.time)
    tilt = getattr(controller, '_tilt_reference', None)
    value['tilt_reference'] = None if tilt is None else tilt.state_dict()
    return digest(value)


def support_from_wrench(locator, data, wrench):
    """Locate one world-frame force/moment at the supplied CURRENT robot pose.

    Margins are signed axial distances to the existing metadata bands; they
    diagnose classification sensitivity, not uncertainty probabilities.  The
    returned point is a record-weighted equivalent mean, including shared rims.
    No observer update, contact query, ground normal or terrain is read.
    """
    wrench = np.asarray(wrench, float)
    if wrench.shape != (6,):
        raise ValueError('World force/moment must contain six values')
    finite = bool(np.isfinite(wrench).all())
    normal = intrinsic_normal(wrench[:3], wrench[3:], locator.radius) if finite else None
    records = locator.records(data, wrench[:3], wrench[3:]) if finite else []
    loads = np.zeros(4)
    sums = np.zeros((4, 3))
    for record in records:
        for k, label in enumerate(LABELS):
            if record['name'] == label or record['name'].startswith(label+'_'):
                load = max(0., float(record['normal_force']))
                loads[k] += load
                sums[k] += load*np.asarray(record['pos'], float)
                break
    total = float(loads.sum())
    shares = loads/total if total > 0 else np.zeros(4)
    points = np.full((4, 3), np.nan)
    positive = loads > 0
    points[positive] = sums[positive]/loads[positive, None]
    point = sums.sum(axis=0)/total if total > 0 else np.full(3, np.nan)
    lower, upper, signed = np.full(4, np.nan), np.full(4, np.nan), np.full(4, np.nan)
    if normal is not None:
        support = -locator.radius*normal
        for k, patch in enumerate(locator.patches):
            axis = data.xmat[patch['body']].reshape(3, 3)[:, 0]
            signed[k] = patch['sign']*float(axis@support)
            lower[k] = signed[k]-patch['lower']
            upper[k] = patch['upper']-signed[k]
    finite_margins = np.r_[np.abs(lower), np.abs(upper)]
    finite_margins = finite_margins[np.isfinite(finite_margins)]
    return dict(valid_wrench=finite, intrinsic_normal_valid=normal is not None,
        records_available=bool(records), normal=np.asarray(normal).copy() if normal is not None else np.full(3, np.nan),
        loads_N=loads, shares=shares, total_load_N=total, point_m=point, patch_points_m=points,
        point_valid=total > 0, record_count=len(records),
        signed_band_coordinate_m=signed, lower_band_margin_m=lower, upper_band_margin_m=upper,
        nearest_band_boundary_abs_m=float(finite_margins.min()) if len(finite_margins) else np.nan,
        dominant_patch_index=int(loads.argmax()) if total > 0 else -1,
        qualification='Current-pose ideal-band/rim equivalent support from inferred wrench; not solved contacts.')


def scaled_interval_loads(native_loads, interval_loads, touching, interval_valid=True):
    """Preserve native inferred total and native dropout; change only share ratios."""
    native = np.asarray(native_loads, float)
    interval = np.asarray(interval_loads, float)
    if native.shape != (4,) or interval.shape != (4,) or not np.isfinite(native).all() or np.any(native < 0):
        raise ValueError('Finite nonnegative four-patch native loads required')
    invalid = not interval_valid or not np.isfinite(interval).all() or np.any(interval < 0)
    code = (1 if not touching else 2 if native.sum() <= .001 else
            4 if invalid else 3 if interval.sum() <= .001 else 0)
    used = native.copy() if code else interval/interval.sum()*native.sum()
    return used, code


def feedback_routing_shadow(controller, normal, native_measurement, touching,
                             native_feedback, native_pre_sync_command, feedforward):
    """Gain/holding-only counterfactual, after ONE native feedback call.

    This function never invokes _terrain_feedback, updates the observer, or
    changes held loads, heading selection, references, rolling allocation or
    synchronization.  Results are PRE-synchronization command sensitivities,
    not an executed alternative controller or a full alternative-policy rollout.
    """
    before = history_fingerprint(controller)
    input_before = digest(dict(normal=normal, measurement=native_measurement,
        feedback=native_feedback, command=native_pre_sync_command, feedforward=feedforward))
    observer = controller.observer
    filtered = support_from_wrench(controller.locator, controller.plant.data,
                                   np.r_[observer.wrench()])
    interval = support_from_wrench(controller.locator, controller.plant.data,
                                   observer.interval_mean[:6])
    native_loads = np.array([native_measurement['loads_N'][name] for name in LABELS])
    if not np.array_equal(filtered['loads_N'], native_loads):
        raise AssertionError('Diagnostic filtered locator must reproduce native support loads exactly')
    used, code = scaled_interval_loads(native_loads, interval['loads_N'], touching,
                                      interval['valid_wrench'] and interval['intrinsic_normal_valid'])
    held = getattr(controller, '_held', None)
    effective_native_loads = (np.array([held['loads'][k] for k in LABELS])
        if not touching and held is not None else native_loads.copy())
    native_gain = np.asarray(native_feedback['K'])
    native_support = np.asarray(native_feedback['support_torque'])
    error = np.asarray(native_feedback['error'])
    if code:
        gain, support = native_gain.copy(), native_support.copy()
    else:
        load_dict = dict(zip(LABELS, used))
        gain = controller.feedback._gain(native_feedback['effective_phase_rad'], load_dict)
        gain[0] = native_gain[0]
        support = controller.params.support_feedforward*controller.support_torque(normal, load_dict)
    delta_gain = -(gain-native_gain)@error
    delta_support = support-native_support
    delta = delta_gain+delta_support
    equilibrium = np.asarray(controller.ordinary['equilibrium']['u'])
    reconstructed_native = equilibrium+np.asarray(feedforward)+native_support-native_gain@error
    shadow = equilibrium+np.asarray(feedforward)+support-gain@error
    input_after = digest(dict(normal=normal, measurement=native_measurement,
        feedback=native_feedback, command=native_pre_sync_command, feedforward=feedforward))
    invariants = dict(native_history_unchanged=before == history_fingerprint(controller),
                      native_feedback_inputs_unchanged=input_before == input_after,
                      native_shaft_gain_row_preserved=bool(np.array_equal(gain[0], native_gain[0])))
    if not all(invariants.values()):
        raise AssertionError(invariants)
    return dict(filtered=filtered, interval=interval, used_loads_N=used,
        native_feedback_effective_loads_N=effective_native_loads,
        used_feedback_effective_loads_N=effective_native_loads.copy() if code else used.copy(),
        fallback_code=code, fallback_reason=FALLBACK_CODES[code], native_touching=bool(touching),
        native_feedback_gain=native_gain.copy(), interval_share_feedback_gain=gain,
        native_support_torque=native_support.copy(), interval_share_support_torque=support,
        native_error=error.copy(), native_normal=np.asarray(normal).copy(),
        native_feedforward=np.asarray(feedforward).copy(),
        delta_gain_command=delta_gain, delta_support_command=delta_support,
        delta_pre_sync_command=delta, native_pre_sync_command=np.asarray(native_pre_sync_command).copy(),
        interval_share_pre_sync_command=shadow,
        native_reconstruction_max_abs_gap=float(np.max(np.abs(reconstructed_native-native_pre_sync_command))),
        delta_reconstruction_max_abs_gap=float(np.max(np.abs((shadow-native_pre_sync_command)-delta))),
        invariants=invariants,
        scope='Counterfactual gain+holding routing only, before native synchronization/heading/fade/clipping; never actuated.')


def bridge_contact_truth(rows, centre_m):
    """Evaluator-only admitted wheel contacts, matching native bridge admission.

    Moment is about the centre from the SAME completed physics substep.  Body
    contacts are recorded separately and never mixed into wheel support loads.
    """
    rows, centre = np.asarray(rows, float), np.asarray(centre_m, float)
    if rows.ndim != 2 or rows.shape[1] != 16 or centre.shape != (3,) or not np.isfinite(rows).all() or not np.isfinite(centre).all():
        raise ValueError('Finite native bridge rows16 and matching centre3 required')
    loads, sums, counts = np.zeros(4), np.zeros((4, 3)), np.zeros(4, int)
    force, moment, body_load = np.zeros(3), np.zeros(3), 0.
    body_force, body_moment = np.zeros(3), np.zeros(3)
    for row in rows:
        if row[12] < .5:
            continue
        normal_force = float(row[7:10]@row[4:7])
        if row[10] <= 0 and normal_force <= 0:
            continue
        tag = int(row[0])
        pressure = max(0., normal_force)
        if tag == 6:
            body_load += pressure
            body_force += row[7:10]
            body_moment += np.cross(row[1:4]-centre, row[7:10])
            continue
        if not 0 <= tag <= 5:
            raise ValueError('Unknown native bridge contact tag')
        patch = (0, 1, 2, 3, 0, 1)[tag]
        force += row[7:10]
        moment += np.cross(row[1:4]-centre, row[7:10])
        loads[patch] += pressure
        sums[patch] += pressure*row[1:4]
        counts[patch] += int(pressure > .05)
    points = np.full((4, 3), np.nan)
    loaded = loads > 1e-12
    points[loaded] = sums[loaded]/loads[loaded, None]
    equivalent = sums.sum(axis=0)/loads.sum() if loads.sum() > 1e-12 else np.full(3, np.nan)
    return dict(wrench_world=np.r_[force, moment], loads_N=loads, patch_points_m=points,
                resolved_contact_counts=counts, equivalent_point_m=equivalent, body_load_N=body_load,
                body_wrench_world=np.r_[body_force, body_moment],
                all_external_wrench_world=np.r_[force+body_force, moment+body_moment])


class IntervalTruthSampler:
    """Harness-only five single-step evaluations under one unchanged command.

    Call begin() immediately inside physical.step, let run() replace its
    sim.run, then finish().  Outside a physical control step, run() delegates
    unchanged (resets/ground checks are not sampled).  No mirror is pulled or
    forwarded, and no controller/observer is called during these evaluations.
    """
    def __init__(self, sim, physical_step_s=.001, control_dt_s=.005):
        if physical_step_s <= 0 or control_dt_s <= 0 or not np.isfinite([physical_step_s, control_dt_s]).all():
            raise ValueError('Finite positive physics/control steps required')
        ratio = control_dt_s/physical_step_s
        if abs(ratio-round(ratio)) > 1e-10:
            raise ValueError('Control interval must contain an integer number of physics steps')
        self.sim, self.original_run = sim, sim.run
        self.physics_dt, self.control_dt, self.substeps = float(physical_step_s), float(control_dt_s), int(round(ratio))
        self.active = None
        self.latest = None

    def begin(self, start_time_s):
        if self.active is not None or not np.isfinite(start_time_s):
            raise ValueError('Sampler cannot overlap intervals or accept invalid time')
        self.active = dict(start_time_s=float(start_time_s), samples=[])

    def run(self, iterations):
        if self.active is None:
            return self.original_run(iterations)
        if iterations != self.substeps or self.active['samples']:
            raise AssertionError('Exactly one native control-interval run is expected')
        for k in range(self.substeps):
            self.original_run(1)
            state = np.asarray(self.sim.state(), float).copy()
            contacts = np.asarray(self.sim.contacts(), float).copy()
            truth = bridge_contact_truth(contacts, state[2:5])
            self.active['samples'].append(dict(time_s=self.active['start_time_s']+(k+1)*self.physics_dt,
                bridge_time_s=float(state[0]), bridge_state=state, contact_rows=contacts, centre_m=state[2:5], **truth))

    def finish(self):
        active = self.active
        if active is None or len(active['samples']) != self.substeps:
            raise AssertionError('Every completed control interval needs all physics samples')
        samples = active['samples']
        loads = np.stack([s['loads_N'] for s in samples])
        points = np.stack([s['patch_points_m'] for s in samples])
        weighted = np.nansum(loads[:, :, None]*points, axis=0)
        mean_loads = loads.mean(axis=0)
        mean_points = np.full((4, 3), np.nan)
        mask = loads.sum(axis=0) > 1e-12
        mean_points[mask] = weighted[mask]/loads.sum(axis=0)[mask, None]
        self.latest = dict(start_time_s=active['start_time_s'], end_time_s=active['start_time_s']+self.control_dt,
            mean_wrench_world=np.mean([s['wrench_world'] for s in samples], axis=0),
            mean_all_external_wrench_world=np.mean([s['all_external_wrench_world'] for s in samples], axis=0),
            mean_body_wrench_world=np.mean([s['body_wrench_world'] for s in samples], axis=0),
            mean_body_load_N=float(np.mean([s['body_load_N'] for s in samples])),
            mean_loads_N=mean_loads, mean_patch_points_m=mean_points,
            mean_equivalent_point_m=weighted.sum(axis=0)/loads.sum() if loads.sum() > 1e-12 else np.full(3, np.nan),
            substep_time_s=np.array([s['time_s'] for s in samples]),
            substep_bridge_time_s=np.array([s['bridge_time_s'] for s in samples]),
            substep_bridge_state=np.array([s['bridge_state'] for s in samples]),
            substep_contact_rows=[s['contact_rows'] for s in samples],
            substep_wrench_world=np.array([s['wrench_world'] for s in samples]),
            substep_all_external_wrench_world=np.array([s['all_external_wrench_world'] for s in samples]),
            substep_body_wrench_world=np.array([s['body_wrench_world'] for s in samples]),
            substep_loads_N=loads, substep_patch_points_m=points,
            substep_resolved_contact_counts=np.array([s['resolved_contact_counts'] for s in samples]),
            substep_body_load_N=np.array([s['body_load_N'] for s in samples]),
            timing='Mean of five completed1ms solver samples preceding current5ms observation; moments use each matching moving centre.')
        self.active = None
        return self.latest

    def completed_interval(self, observation_time_s):
        if self.latest is None:
            return None
        if abs(self.latest['end_time_s']-float(observation_time_s)) > 1e-9:
            raise ValueError('Interval truth must align with the current observation endpoint')
        return self.latest
