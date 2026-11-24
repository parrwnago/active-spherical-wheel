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

``MomentumObserver`` obtains the same force and moment without any force
sensing. It reads only the joint state, the drive torques and the dynamics
model, and recovers the ground wrench from how the machine's momentum changes.

``SupportLocator`` turns that force and moment into contact records: which
shell, belt or rim carries the load, where, and how much, so that nothing
downstream needs the solver's contacts either.

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
    records = getattr(plant, 'contact_records', None)
    if records is not None:
        # A plant simulated elsewhere (Gazebo) supplies its own solved contacts.
        return measure_records(plant, records())
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


def records_from_mujoco(plant):
    """The MuJoCo plant's active ground contacts as contact records.

    Same fields as ``GazeboPlant.contact_records``: ``name``, ``geom``,
    ``body``, ``pos``, unit ``normal`` from the ground into the wheel, world
    ``force`` on the wheel, ``normal_force``, ``tangent_force_norm`` and
    ``friction``. Used to check that the records path of ``measure_contact``
    and friends reproduces the MuJoCo path exactly.
    """
    model, data = plant.model, plant.data
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    wrench = np.zeros(6)
    records = []
    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.efc_address < 0 or floor not in contact.geom:
            continue
        ground_first = int(contact.geom[0]) == floor
        other = int(contact.geom[1] if ground_first else contact.geom[0])
        sign = 1. if ground_first else -1.
        mujoco.mj_contactForce(model, data, index, wrench)
        frame = contact.frame.reshape(3, 3)
        records.append({'name': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other) or '',
                        'geom': other, 'body': int(model.geom_bodyid[other]), 'pos': contact.pos.copy(),
                        'normal': sign*frame[0], 'force': sign*(frame.T@wrench[:3]),
                        'normal_force': float(wrench[0]),
                        'tangent_force_norm': float(np.hypot(wrench[1], wrench[2])),
                        'friction': float(contact.friction[0])})
    return records


def measure_records(plant, records):
    """``measure_contact`` from contact records instead of MuJoCo's contact list.

    The arithmetic is that of ``measure_contact``; only the source of each
    contact's position, normal, force and friction differs. The plant's
    ``model``/``data`` still supply the frames, Jacobians and belt speeds.
    """
    model, data = plant.model, plant.data
    centre = data.xpos[plant.fork_id]
    force, moment = np.zeros(3), np.zeros(3)
    normal_sum, point_sum, slip_sum = np.zeros(3), np.zeros(3), np.zeros(3)
    loads = {name: 0. for name in SUPPORTS}
    normal_load = friction_sum = utilization = 0.
    fork_load = 0.
    jacobian = np.zeros((3, model.nv))
    count = 0
    for record in records:
        normal, world, position = record['normal'], record['force'], record['pos']
        support = _support(record['name'])
        pressure = max(0., float(record['normal_force']))
        if support is None:
            fork_load += pressure
            continue
        count += 1
        loads[support] += pressure
        force += world
        moment += np.cross(position-centre, world)
        normal_load += pressure
        normal_sum += pressure*normal
        point_sum += pressure*position
        friction_sum += pressure*float(record['friction'])
        mujoco.mj_jac(model, data, jacobian, None, position, int(record['body']))
        velocity = jacobian@data.qvel
        if support.startswith('belt'):
            geom = int(record['geom'])
            velocity = velocity+data.geom_xmat[geom].reshape(3, 3)@model.geom_surfacevel[geom, :3]
        slip_sum += pressure*(velocity-normal*(normal@velocity))
        limit = float(record['friction'])*pressure
        if limit > 1e-9:
            utilization = max(utilization, float(record['tangent_force_norm'])/limit)
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


def _rotation(quat):
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, np.asarray(quat, float)/np.linalg.norm(quat))
    return matrix.reshape(3, 3)


class MomentumObserver:
    """Ground force and moment from the machine's motion, without force sensing.

    A generalized momentum observer (De Luca and Mattone, 2003) on the plant's
    nine velocities ``v`` (the fork's free joint, the shaft and both shells).
    With ``M(q)`` the mass matrix and ``p = M v`` the generalized momentum,

        dp/dt = S u + f_passive - c(q, v) + (dM/dt) v + f_ext

    where ``S u`` are the drive torques, ``f_passive`` the joint damping, ``c``
    gravity plus Coriolis and centrifugal forces, and ``f_ext`` everything the
    model does not contain: the ground contact, plus any push. Integrated over
    one control interval this gives the interval's mean ``f_ext`` from two
    consecutive states and the command held between them. The integrals of the
    smooth model terms use the trapezoidal rule. No acceleration and no
    contact force is read. The estimate follows that mean through
    ``dr/dt = K (f_ext - r)`` with ``K = cutoff_rad_s``, discretised exactly
    for a value held over the interval.

    The first six rows belong to the fork's free joint, whose origin is the
    wheel centre. Drive torques and joint damping act between bodies, so they
    cancel there: those rows are the whole machine's momentum balance. They
    give the net ground force (world frame) and its moment about the wheel
    centre (fork frame, rotated to world here), the pair a hub load cell would
    measure and ``intrinsic_normal`` expects. The moment rows are rotated to
    world with the interval's mid rotation before filtering, so a turning body
    does not smear the moment.

    Inputs: ``q`` and ``v`` (on hardware: IMU attitude and rate, joint
    encoders, and an estimate of the base's linear velocity), the applied drive
    torques and the model. In simulation ``q`` and ``v`` are the plant's state.
    A push on the body is indistinguishable from ground force and biases the
    estimate while it lasts.
    """

    def __init__(self, model, fork_id, cutoff_rad_s=100.):
        if not cutoff_rad_s > 0:
            raise ValueError('The observer cutoff must be positive')
        if (model.njnt == 0 or model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE or model.jnt_dofadr[0] != 0
                or model.jnt_bodyid[0] != fork_id):
            raise ValueError('The observer expects the fork to carry the first joint, a free joint')
        self.model, self.fork_id = model, int(fork_id)
        self.cutoff_rad_s = float(cutoff_rad_s)
        self._data = mujoco.MjData(model)   # private: the plant's data are never written
        self._mass = np.zeros((model.nv, model.nv))
        self._drive = np.zeros((model.nv, model.nu))
        for index in range(model.nu):
            if (model.actuator_trntype[index] != mujoco.mjtTrn.mjTRN_JOINT
                    or model.actuator_dyntype[index] != mujoco.mjtDyn.mjDYN_NONE
                    or model.actuator_gaintype[index] != mujoco.mjtGain.mjGAIN_FIXED
                    or model.actuator_biastype[index] != mujoco.mjtBias.mjBIAS_NONE):
                raise ValueError('The observer models only direct joint torque motors')
            dof = model.jnt_dofadr[model.actuator_trnid[index, 0]]
            self._drive[dof, index] = model.actuator_gear[index, 0]
        self.generalized = np.zeros(model.nv)    # filtered f_ext; rows 0-5 in world frame
        self.interval_mean = np.zeros(model.nv)  # last interval's mean f_ext, same frames
        self._previous = None

    def _torques(self, command):
        """Actuator forces for a command, with MuJoCo's control and force limits."""
        model = self.model
        control = np.zeros(model.nu) if command is None else np.array(command[:model.nu], float)
        limited = model.actuator_ctrllimited.astype(bool)
        control[limited] = np.clip(control[limited], *model.actuator_ctrlrange[limited].T)
        force = model.actuator_gainprm[:, 0]*control
        limited = model.actuator_forcelimited.astype(bool)
        force[limited] = np.clip(force[limited], *model.actuator_forcerange[limited].T)
        return self._drive@force

    def _terms(self, qpos, qvel):
        """Mass matrix, bias force ``c(q, v)``, passive force and fork rotation at one state."""
        model, data = self.model, self._data
        data.qpos[:], data.qvel[:] = qpos, qvel
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_crb(model, data)
        mujoco.mj_fullM(model, data, self._mass)
        mujoco.mj_comVel(model, data)
        mujoco.mj_passive(model, data)
        bias = np.empty(model.nv)
        mujoco.mj_rne(model, data, 0, bias)
        return (self._mass.copy(), bias, data.qfrc_passive.copy(),
                data.xmat[self.fork_id].reshape(3, 3).copy(), data.xquat[self.fork_id].copy())

    @staticmethod
    def _to_world(generalized, rotation):
        world = np.array(generalized, float)
        world[3:6] = rotation@world[3:6]
        return world

    def reset(self, qpos, qvel, time, command=None):
        """Start from the given state, assuming it is not accelerating.

        At rest the ground carries whatever the model's own forces leave
        unbalanced: on the free-joint rows, gravity. For a machine standing
        still that places the contact under the centre of mass.
        """
        mass, bias, passive, rotation, quat = self._terms(qpos, qvel)
        self.generalized = self._to_world(bias-passive-self._torques(command), rotation)
        self.interval_mean = self.generalized.copy()
        self._previous = (float(time), mass, np.array(qvel, float), bias, passive, quat)
        return self.wrench()

    def update(self, qpos, qvel, command, time):
        """Advance to a new state; ``command`` is the one held since the previous state.

        A repeated time leaves the estimate unchanged; an earlier time restarts it.
        """
        if self._previous is None:
            return self.reset(qpos, qvel, time, command)
        time0, mass0, qvel0, bias0, passive0, quat0 = self._previous
        dt = float(time)-time0
        if dt < -1e-12:
            return self.reset(qpos, qvel, time, command)
        if dt <= 1e-12:
            return self.wrench()
        mass1, bias1, passive1, _, quat1 = self._terms(qpos, qvel)
        qvel1 = np.array(qvel, float)
        known = (dt*(self._torques(command)+.5*(passive0+passive1)-.5*(bias0+bias1))
                 + (mass1-mass0)@(.5*(qvel0+qvel1)))
        mean = (mass1@qvel1-mass0@qvel0-known)/dt
        middle = quat0+(quat1 if quat0@quat1 >= 0 else -quat1)
        self.interval_mean = self._to_world(mean, _rotation(middle))
        self.generalized += (1.-np.exp(-self.cutoff_rad_s*dt))*(self.interval_mean-self.generalized)
        self._previous = (float(time), mass1, qvel1, bias1, passive1, quat1)
        return self.wrench()

    def wrench(self):
        """``(force, moment)``: ground force on the wheel and its moment about the wheel centre, world frame."""
        return self.generalized[:3].copy(), self.generalized[3:6].copy()

    def measurement(self, radius):
        """The fields of ``measure_contact`` that ``ContactEstimator`` reads, from the observer.

        The normal load is the force along the ground normal implied by the
        same force and moment. Sliding cannot be seen without contact sensing:
        ``slip_speed_m_s`` is reported as zero, so the friction estimate stays
        at its nominal value.
        """
        force, moment = self.wrench()
        normal = intrinsic_normal(force, moment, radius)
        if normal is None:
            normal, pressure = np.array([0., 0., 1.]), 0.
        else:
            pressure = max(0., float(normal@force))
        tangential = force-normal*(normal@force)
        return {'in_contact': bool(pressure > 1e-9), 'force_N': force, 'moment_Nm': moment, 'normal': normal,
                'normal_force_N': pressure, 'tangential_force_N': tangential,
                'tangential_force_norm_N': float(np.linalg.norm(tangential)), 'slip_velocity_m_s': np.zeros(3),
                'slip_speed_m_s': 0., 'source': 'momentum_observer'}


class SupportLocator:
    """Which wheel surface carries the ground force, and where, from that force and its moment.

    The force and its moment about the wheel centre fix the force's line of
    action. The wheel's sphere of radius ``R`` meets that line at
    ``c - R n``, with ``n`` from ``intrinsic_normal``. Every shell and belt
    patch is a band of that sphere between two planes normal to its body's x
    axis (the ``geom_user`` metadata the clipped contact model uses), so the
    patch whose band holds the point is the one carrying the load: the same
    rule the contact model applies to the ground plane.

    The bands leave a gap between the two shells, where the wheel stands on
    the shell rims instead. A load on one rim acts through that rim, which
    lies on the sphere, so the point found above is the rim itself. Shared by
    both rims, the resultant passes between them at the load-weighted axial
    position, and the lever rule gives each rim its share.

    ``records(data, force, moment)`` returns contact records in the format of
    ``records_from_mujoco``, one per loaded surface, so ``measure_records``,
    ``rolling_feedforward_records`` and ``ContactSynchronizer`` use them
    unchanged. Nothing here reads a contact, the ground or the terrain: only
    the force and moment and the wheel's own pose. With no compressive load
    above ``minimum_load_N`` there are no records.
    """

    SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')

    def __init__(self, model, centre_body, radius, nominal_friction, minimum_load_N):
        if model.nuser_geom < 5:
            raise ValueError('The support locator needs the generated spherical patch metadata')
        self.centre_body = int(centre_body)
        self.radius = float(radius)
        self.friction = float(nominal_friction)
        self.minimum_load_N = float(minimum_load_N)
        self.patches = []
        for support in self.SUPPORTS:
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, support+'_smooth')
            if geom < 0:
                raise ValueError(f'The model lacks {support}_smooth')
            sign, lower, upper, _, patch_radius = model.geom_user[geom, :5]
            if not np.isclose(patch_radius, self.radius):
                raise ValueError(f'{support} is not on the sphere of radius {self.radius}')
            self.patches.append({'name': support+'_smooth', 'support': support, 'geom': int(geom),
                                 'body': int(model.geom_bodyid[geom]), 'sign': float(sign),
                                 'lower': float(lower), 'upper': float(upper)})
        self.shells = [patch for patch in self.patches if patch['support'].startswith('shell')]

    def _axis(self, data, patch):
        return data.xmat[patch['body']].reshape(3, 3)[:, 0]

    def locate(self, data, support):
        """``[(patch, share, point)]`` for a support vector ``support`` from the wheel centre."""
        centre = data.xpos[self.centre_body]
        for patch in self.patches:
            signed = patch['sign']*float(self._axis(data, patch)@support)
            if patch['lower']-1e-12 <= signed <= patch['upper']+1e-12:
                return [(patch, 1., centre+support)]
        # In the gap between the shells: the rims. Axial positions are along
        # the first shell's axis; the shells turn about the same carrier axis.
        axis = self._axis(data, self.shells[0])
        axial = float(axis@support)
        radial = support-axial*axis
        size = float(np.linalg.norm(radial))
        direction = radial/size if size > 1e-12 else radial
        rims = [(patch, patch['sign']*patch['lower']*float(self._axis(data, patch)@axis)) for patch in self.shells]
        (first, x_first), (second, x_second) = rims
        share = float(np.clip((axial-x_second)/(x_first-x_second), 0., 1.))
        return [(patch, weight, centre+x*axis+np.sqrt(max(self.radius**2-x*x, 0.))*direction)
                for (patch, x), weight in zip(rims, (share, 1.-share)) if weight > 0.]

    def records(self, data, force, moment):
        force, moment = np.asarray(force, float), np.asarray(moment, float)
        normal = intrinsic_normal(force, moment, self.radius)
        if normal is None:
            return []
        load = float(normal@force)
        if load < self.minimum_load_N:
            return []
        tangential = float(np.linalg.norm(force-normal*load))
        return [{'name': patch['name'], 'geom': patch['geom'], 'body': patch['body'], 'pos': point,
                 'normal': normal.copy(), 'force': share*force, 'normal_force': share*load,
                 'tangent_force_norm': share*tangential, 'friction': self.friction}
                for patch, share, point in self.locate(data, -self.radius*normal)]


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
