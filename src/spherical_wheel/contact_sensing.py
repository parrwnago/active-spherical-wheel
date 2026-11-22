"""Ground-reaction measurement and what a controller can infer from it.

``measure_contact`` reads the solved wheel-ground contact: the net force on
the wheel, its moment about the wheel centre, the load on each shell and belt,
the friction coefficient in effect and the sliding velocity of the loaded
surface. It works on the flat plant and on the terrain plant.

``intrinsic_normal`` recovers the ground normal from that force and moment
alone. A point contact on a sphere of radius ``R`` exerts the moment
``M = -R n x F`` about the centre, which fixes ``n`` once ``F`` and ``M`` are
known. A six-axis load cell at the wheel hub measures exactly that pair, so
the slope under the wheel is observable without a terrain map. In simulation
the pair comes from the solver; sensor noise, bias and the hub inertial loads
a real cell would also see are not modelled.

``ContactEstimator`` turns the raw measurement into the slowly varying
quantities feedback needs: a filtered normal, the normal load as a fraction of
weight, and a friction estimate that drops when the surface is seen sliding.
"""
from __future__ import annotations

import mujoco
import numpy as np

SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')


def _support(name):
    for support in SUPPORTS:
        if name == support or name.startswith(support+'_'):
            return support
    return None


def measure_contact(plant):
    """Net wheel-ground reaction in world coordinates.

    Forces act on the wheel. ``moment_Nm`` is about the wheel centre.
    ``slip_velocity_m_s`` is the load-weighted tangential velocity of the
    wheel surface over the ground, including a belt's commanded surface speed.
    ``utilization`` is the largest per-contact ratio of tangential force to
    its friction limit: 1 means that contact is at its limit.
    """
    model, data = plant.model, plant.data
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    centre = data.xpos[plant.fork_id]
    force, moment = np.zeros(3), np.zeros(3)
    normal_sum, point_sum, slip_sum = np.zeros(3), np.zeros(3), np.zeros(3)
    loads = {name: 0. for name in SUPPORTS}
    normal_load = friction_sum = utilization = 0.
    fork_load = 0.
    wrench, jacobian = np.zeros(6), np.zeros((3, model.nv))
    count = 0
    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.efc_address < 0 or floor not in contact.geom:
            continue
        # MuJoCo reports the force of geom1 on geom2 in the contact frame,
        # whose first axis points from geom1 to geom2.
        ground_first = int(contact.geom[0]) == floor
        other = int(contact.geom[1] if ground_first else contact.geom[0])
        sign = 1. if ground_first else -1.
        mujoco.mj_contactForce(model, data, index, wrench)
        frame = contact.frame.reshape(3, 3)
        normal = sign*frame[0]
        world = sign*(frame.T@wrench[:3])
        support = _support(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other) or '')
        pressure = max(0., float(wrench[0]))
        if support is None:
            fork_load += pressure
            continue
        count += 1
        loads[support] += pressure
        force += world
        moment += np.cross(contact.pos-centre, world)
        normal_load += pressure
        normal_sum += pressure*normal
        point_sum += pressure*contact.pos
        friction_sum += pressure*float(contact.friction[0])
        mujoco.mj_jac(model, data, jacobian, None, contact.pos, int(model.geom_bodyid[other]))
        velocity = jacobian@data.qvel
        if support.startswith('belt'):
            velocity = velocity+data.geom_xmat[other].reshape(3, 3)@model.geom_surfacevel[other, :3]
        slip_sum += pressure*(velocity-normal*(normal@velocity))
        limit = float(contact.friction[0])*pressure
        if limit > 1e-9:
            utilization = max(utilization, float(np.hypot(wrench[1], wrench[2]))/limit)
    if normal_load > 1e-9:
        normal = normal_sum/np.linalg.norm(normal_sum)
        point, slip, friction = point_sum/normal_load, slip_sum/normal_load, friction_sum/normal_load
    else:
        normal, point, slip, friction = np.array([0., 0., 1.]), centre.copy(), np.zeros(3), float('nan')
    tangential = force-normal*(normal@force)
    return {'in_contact': bool(normal_load > 1e-9), 'contacts': count,
            'force_N': force, 'moment_Nm': moment, 'normal': normal, 'point_m': point,
            'normal_force_N': float(normal_load), 'tangential_force_N': tangential,
            'tangential_force_norm_N': float(np.linalg.norm(tangential)),
            'friction_coefficient': float(friction), 'utilization': float(utilization),
            'slip_velocity_m_s': slip, 'slip_speed_m_s': float(np.linalg.norm(slip)),
            'loads_N': loads, 'fork_load_N': float(fork_load)}


def intrinsic_normal(force, moment, radius):
    """Ground normal from the wheel's net contact force and centre moment.

    Solves ``moment = -radius * n x force`` for the unit vector ``n`` with a
    compressive component along the force. With several simultaneous contacts
    it returns the equivalent single-contact normal. Returns ``None`` when the
    force is too small to define a direction.
    """
    force, moment = np.asarray(force, float), np.asarray(moment, float)
    squared = float(force@force)
    if squared < 1e-12:
        return None
    lateral = -np.cross(force, moment)/(radius*squared)
    excess = float(lateral@lateral)
    if excess >= 1.:
        return lateral/np.sqrt(excess)
    normal = np.sqrt(1.-excess)*force/np.sqrt(squared)+lateral
    return normal/np.linalg.norm(normal)


class ContactEstimator:
    """Filtered ground normal, load fraction and friction estimate.

    ``update(measurement, dt)`` returns the current estimate. The normal is a
    first-order low-pass of the intrinsic estimate and is held while the wheel
    is unloaded. ``friction`` starts at the nominal coefficient; when the
    loaded surface has slid faster than ``slip_threshold_m_s`` for
    ``slip_confirm_s`` it drops to the force ratio measured during that slide,
    and it relaxes back with ``friction_recovery_s`` once sliding stops. It is
    learned from sliding, never a prediction of ground the wheel has not slid on.
    """

    def __init__(self, radius, weight_N, nominal_friction, normal_time_constant_s=.04,
                 load_time_constant_s=.02, slip_threshold_m_s=.02, friction_recovery_s=1.5,
                 minimum_load_fraction=.05, source='intrinsic', rim_time_constant_s=.4,
                 slip_confirm_s=.03):
        if source not in ('intrinsic', 'solver'):
            raise ValueError('Normal source must be intrinsic or solver')
        self.radius, self.weight = float(radius), float(weight_N)
        self.nominal_friction = float(nominal_friction)
        self.normal_time_constant_s = float(normal_time_constant_s)
        self.load_time_constant_s = float(load_time_constant_s)
        self.slip_threshold_m_s = float(slip_threshold_m_s)
        self.friction_recovery_s = float(friction_recovery_s)
        self.minimum_load_fraction = float(minimum_load_fraction)
        self.rim_time_constant_s = float(rim_time_constant_s)
        self.slip_confirm_s = float(slip_confirm_s)
        self.source = source
        self.reset()

    def reset(self, normal=(0., 0., 1.)):
        self.normal = np.asarray(normal, float)/np.linalg.norm(normal)
        self.load_fraction = 1.
        self.friction = self.nominal_friction
        self.sliding = False
        self.loaded = True
        self._sliding_time, self._sliding_ratio = 0., 0.

    def update(self, measurement, dt, rim_axis=None, hold_axis=False):
        """Advance the estimate by one controller interval.

        Pass ``rim_axis`` (the carrier's x axis in world coordinates) while
        the wheel may be riding on a shell rim instead of its spherical
        surface. A rim contact sits up to the half gap away from where a
        sphere would touch, displaced along that axis, so along it the
        force-and-moment normal reports the ground slope plus the rim offset.
        In this model the rims are also 48-sided, so the contact wanders
        across the axis by a few millimetres as a shell turns.

        On the rims the whole estimate therefore follows with
        ``rim_time_constant_s``: slow enough to average a rim-to-rim crossing
        and the facets, fast enough to keep up with ground that changes under
        the wheel meanwhile. Pass ``hold_axis`` when the wheel is resting on
        both rims. Two rims are a stable two-point base whose load shares
        depend on where the body leans, so following them there tips the wheel
        onto whichever rim it leans toward; the axial part is held instead.
        """
        raw_fraction = measurement['normal_force_N']/self.weight
        blend = dt/(self.load_time_constant_s+dt)
        self.load_fraction += blend*(raw_fraction-self.load_fraction)
        self.loaded = raw_fraction > self.minimum_load_fraction
        raw_normal = None
        if self.loaded:
            raw_normal = (measurement['normal'] if self.source == 'solver' else
                          intrinsic_normal(measurement['force_N'], measurement['moment_Nm'], self.radius))
        if raw_normal is not None:
            innovation = raw_normal-self.normal
            blend = dt/(self.normal_time_constant_s+dt)
            if rim_axis is None or self.source == 'solver':
                mixed = self.normal+blend*innovation
            else:
                axis = np.asarray(rim_axis, float)
                along = (innovation@axis)*axis
                slow = dt/(self.rim_time_constant_s+dt)
                mixed = self.normal+slow*(innovation-along)+(0. if hold_axis else slow)*along
            self.normal = mixed/np.linalg.norm(mixed)
        # One sliding sample proves nothing: a handover between surfaces shows
        # a brief surface speed while the force is well inside the cone. Only
        # sustained sliding means the measured force ratio is the friction
        # coefficient, and then the largest ratio seen while sliding is used.
        if self.loaded and measurement['slip_speed_m_s'] > self.slip_threshold_m_s:
            self._sliding_time += dt
            ratio = measurement['tangential_force_norm_N']/max(measurement['normal_force_N'], 1e-9)
            self._sliding_ratio = max(self._sliding_ratio, ratio)
        else:
            self._sliding_time, self._sliding_ratio = 0., 0.
        self.sliding = self._sliding_time >= self.slip_confirm_s
        if self.sliding:
            self.friction = float(np.clip(min(self.friction, self._sliding_ratio), .05, self.nominal_friction))
        else:
            self.friction += dt/(self.friction_recovery_s+dt)*(self.nominal_friction-self.friction)
        return {'normal': self.normal.copy(), 'raw_normal': raw_normal,
                'load_fraction': float(self.load_fraction), 'raw_load_fraction': float(raw_fraction),
                'friction': float(self.friction), 'sliding': bool(self.sliding), 'loaded': bool(self.loaded)}
