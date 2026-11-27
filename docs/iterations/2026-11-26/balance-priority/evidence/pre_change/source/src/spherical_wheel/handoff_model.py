"""Short-horizon velocity authority from measured state and supplied contacts.

This model evaluates rigid-body terms on a private MuJoCo copy. It does not
query contacts, terrain or solver forces and never integrates a plant. The
supplied estimated contacts give a fixed soft rolling-velocity penalty.
Consequently its responses are a local control-allocation approximation, not
contact prediction or a certificate of friction, unilateral support or balance.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass

import mujoco
import numpy as np


BODY_NAMES = ('fork', 'carrier', 'shell_A', 'shell_B')
SUPPORT_NAMES = ('shell_A', 'shell_B', 'belt_A', 'belt_B')


def _readonly(value):
    result = np.array(value, dtype=float, copy=True)
    result.setflags(write=False)
    return result


def _cross_matrix(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


@dataclass(frozen=True)
class ContactVelocityResponse:
    """Frozen-geometry affine response ``v_next = offset + response @ u``.

    ``v`` has the native nine generalized-velocity coordinates: free-base
    translation in world axes, free-base angular velocity in fork axes, then
    three relative hinge rates. The five inputs are three motor commands in
    N m and two prescribed belt speeds in m/s. Belt commands enter directly
    as velocity constraints, not as torques or belt-rotor dynamics.

    Contact rows are stacked WORLD xyz rows, one triplet per contact. For
    ``C = contact_jacobian`` and ``E = belt_surface_matrix`` the residual is
    ``C @ v_next + E @ u[3:]``. The impulse maps contain world force impulses
    on each supplied body, in N s; their normal/tangent feasibility is not
    enforced. Dependent and incompatible contacts retain residuals. The contact
    penalty has units kg, so velocity residual times penalty is an impulse.
    ``contact_penalty_kg`` gives the unattenuated isotropic row masses;
    ``contact_penalty_matrix_kg`` is the actual normal/tangent penalty.

    Body angular-momentum maps use world axes about the total-system COM and
    include spin plus orbital terms. They map velocity at this fixed pose;
    using them at the next pose would require another evaluation.
    """
    horizon_s: float
    offset: np.ndarray
    response: np.ndarray
    free_offset: np.ndarray
    free_response: np.ndarray
    mass_matrix: np.ndarray
    bias_force: np.ndarray
    passive_force: np.ndarray
    drive_matrix: np.ndarray
    contact_names: tuple[str, ...]
    contact_jacobian: np.ndarray
    belt_surface_matrix: np.ndarray
    contact_positions_world_m: np.ndarray
    contact_normals_world: np.ndarray
    contact_loads_N: np.ndarray
    contact_friction: np.ndarray
    contact_penalty_kg: np.ndarray
    contact_penalty_matrix_kg: np.ndarray
    tangent_confidence: float
    contact_impulse_offset: np.ndarray
    contact_impulse_response: np.ndarray
    constraint_offset: np.ndarray
    constraint_response: np.ndarray
    constraint_singular_values: np.ndarray
    constraint_rank: int
    contact_strength: float
    fork_rotation_world: np.ndarray
    shaft_axis_world: np.ndarray
    shell_axis_world: np.ndarray
    body_angular_velocity_maps_world: np.ndarray
    body_spin_momentum_maps_world: np.ndarray
    body_momentum_maps_world: np.ndarray
    total_momentum_map_world: np.ndarray
    system_COM_world_m: np.ndarray
    total_mass_kg: float
    shell_axial_inertias_kg_m2: np.ndarray

    def predict(self, command):
        """Evaluate the affine map; no limits or feasibility are implied."""
        command = np.asarray(command, float)
        if command.shape != (5,) or not np.isfinite(command).all():
            raise ValueError('Expected five finite torque/belt commands')
        return self.offset + self.response @ command


class MeasuredContactVelocityModel:
    """Soft kinetic projection using only measured q/v and contact estimates.

    First freeze ``M``, bias and passive force at the measured state, and form
    ``v_free = v + h M^-1 (S tau + passive - bias)``. Then minimize
    ``||v_next-v_free||_M^2 + ||C v_next+E belt_speed||_Lambda^2``, where each
    contact has penalty
    ``Lambda_i = contact_strength * estimated_normal_force / |gravity| *
    (n n.T + c (I-n n.T))``, where supplied ``c`` is tangential confidence.
    Its default is one; zero confidence retains only the normal penalty. This gives
    ``(M+C.T Lambda C) v_next = M v_free-C.T Lambda E belt_speed``.

    Positive estimated normal loads therefore soften contact authority rather
    than enforcing a hard zero-slip jump. They are not measured impulses.
    Neither normal-impulse sign nor a friction cone is constrained; a large
    measured slip can still imply unrealistic modeled forces. Callers should
    use the differential response with declared model uncertainty, rather than
    claiming the offset is an accurate contact-dynamics forecast.
    """

    def __init__(self, model, *, contact_strength=10., relative_cutoff=1e-10,
                 minimum_load_N=0.):
        settings = np.asarray([relative_cutoff, contact_strength, minimum_load_N], float)
        if (not np.isfinite(settings).all() or not 0 < relative_cutoff < 1
                or contact_strength <= 0 or minimum_load_N < 0):
            raise ValueError('Finite cutoff in (0,1), positive strength and nonnegative load required')
        if (model.nq, model.nv, model.nu) != (10, 9, 3):
            raise ValueError('The handoff model requires the wheel with nq=10, nv=9 and three motors')
        if (model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE
                or model.jnt_dofadr[0] != 0):
            raise ValueError('The first joint must be the fork free joint')
        if np.any(model.dof_armature != 0):
            raise ValueError('Body momentum accounting requires the declared zero-armature model')
        self.model = copy.copy(model)
        self._data = mujoco.MjData(self.model)
        self.relative_cutoff = float(relative_cutoff)
        self.contact_strength = float(contact_strength)
        self.gravity_m_s2 = float(np.linalg.norm(self.model.opt.gravity))
        if self.gravity_m_s2 <= 0:
            raise ValueError('The soft contact penalty requires nonzero modeled gravity')
        self.minimum_load_N = float(minimum_load_N)
        self.body_ids = tuple(int(mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, name)) for name in BODY_NAMES)
        if min(self.body_ids) <= 0:
            raise ValueError('Required fork/carrier/shell bodies are missing')
        if int(self.model.jnt_bodyid[0]) != self.body_ids[0]:
            raise ValueError('The first free joint must belong to fork')
        self._drive = np.zeros((9, 3))
        self._joint_ids = []
        for i in range(3):
            if (self.model.actuator_trntype[i] != mujoco.mjtTrn.mjTRN_JOINT
                    or self.model.actuator_dyntype[i] != mujoco.mjtDyn.mjDYN_NONE
                    or self.model.actuator_gaintype[i] != mujoco.mjtGain.mjGAIN_FIXED
                    or self.model.actuator_biastype[i] != mujoco.mjtBias.mjBIAS_NONE):
                raise ValueError('Only direct fixed-gain joint torque motors are supported')
            joint = int(self.model.actuator_trnid[i, 0])
            self._joint_ids.append(joint)
            dof = int(self.model.jnt_dofadr[joint])
            if dof != 6+i:
                raise ValueError('Motor order must be shaft, shell A, shell B')
            self._drive[dof, i] = (self.model.actuator_gainprm[i, 0]
                                   * self.model.actuator_gear[i, 0])
        # The generated robot's nonzero masses must all enter its COM budget.
        if not np.isclose(self.model.body_mass[list(self.body_ids)].sum(),
                          self.model.body_mass.sum(), rtol=0., atol=1e-12):
            raise ValueError('Unexpected massive bodies outside the wheel body group')

    def _contacts(self, records):
        positions, normals, loads, friction, names, bodies = [], [], [], [], [], []
        expected = {'shell_A': self.body_ids[2], 'shell_B': self.body_ids[3],
                    'belt_A': self.body_ids[1], 'belt_B': self.body_ids[1]}
        for record in records:
            name = str(record.get('name', ''))
            support = next((n for n in SUPPORT_NAMES
                            if name == n or name.startswith(n+'_')), None)
            if support is None:
                raise ValueError('Only supplied estimated shell/belt supports are accepted')
            load = float(record['normal_force'])
            if not np.isfinite(load) or load < 0:
                raise ValueError('Estimated normal load must be finite and nonnegative')
            if load <= self.minimum_load_N:
                continue
            point = np.asarray(record['pos'], float)
            normal = np.asarray(record['normal'], float)
            if (point.shape != (3,) or normal.shape != (3,)
                    or not np.isfinite(point).all() or not np.isfinite(normal).all()
                    or np.linalg.norm(normal) <= 1e-12):
                raise ValueError('A loaded support needs a finite point and nonzero normal')
            body = int(record['body'])
            if body != expected[support]:
                raise ValueError('Estimated support name/body do not match the wheel mechanism')
            mu = float(record.get('friction', np.nan))
            if not np.isnan(mu) and (not np.isfinite(mu) or mu < 0):
                raise ValueError('Known contact friction must be finite and nonnegative')
            positions.append(point.copy())
            normals.append(normal/np.linalg.norm(normal))
            loads.append(load)
            friction.append(mu)
            names.append(support)
            bodies.append(body)
        return (tuple(names), np.asarray(positions, float).reshape(-1, 3),
                np.asarray(normals, float).reshape(-1, 3), np.asarray(loads, float),
                np.asarray(friction, float), bodies)

    def evaluate(self, qpos, qvel, records, *, horizon_s=.02, tangent_confidence=1.):
        """Build an affine response without modifying caller state.

        ``tangent_confidence`` is a caller-supplied model-confidence factor,
        not a measured friction coefficient or an estimate computed here.
        """
        qpos, qvel = np.asarray(qpos, float), np.asarray(qvel, float)
        if (qpos.shape != (10,) or qvel.shape != (9,)
                or not np.isfinite(qpos).all() or not np.isfinite(qvel).all()
                or abs(np.linalg.norm(qpos[3:7])-1.) > 1e-6):
            raise ValueError('Finite wheel qpos/qvel and a unit free quaternion required')
        h = float(horizon_s)
        if not np.isfinite(h) or h <= 0:
            raise ValueError('A finite positive local prediction horizon is required')
        confidence = float(tangent_confidence)
        if not np.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('Tangential contact confidence must lie in [0,1]')
        names, points, normals, loads, friction, bodies = self._contacts(records)
        model, data = self.model, self._data
        data.qpos[:] = qpos
        data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
        data.qvel[:] = qvel
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_crb(model, data)
        mass = np.empty((9, 9))
        mujoco.mj_fullM(model, data, mass)
        mujoco.mj_comVel(model, data)
        mujoco.mj_passive(model, data)
        bias = np.empty(9)
        mujoco.mj_rne(model, data, 0, bias)
        passive = data.qfrc_passive.copy()
        lower = np.linalg.cholesky(mass)
        free_offset = qvel+h*np.linalg.solve(mass, passive-bias)
        free_response = np.zeros((9, 5))
        free_response[:, :3] = h*np.linalg.solve(mass, self._drive)
        carrier_rotation = data.xmat[self.body_ids[1]].reshape(3, 3)
        fork_rotation = data.xmat[self.body_ids[0]].reshape(3, 3)
        contact = np.zeros((3*len(names), 9))
        surface = np.zeros((len(contact), 2))
        for i, (name, point, normal, body) in enumerate(zip(names, points, normals, bodies)):
            rows = slice(3*i, 3*i+3)
            jac = np.empty((3, 9))
            mujoco.mj_jac(model, data, jac, None, point, body)
            contact[rows] = jac
            if name.startswith('belt_'):
                # Preserve the projected direction's length, as the bridge does.
                along = carrier_rotation[:, 1]
                surface[rows, 0 if name == 'belt_A' else 1] = along-normal*(normal@along)
        penalty = np.repeat(self.contact_strength*loads/self.gravity_m_s2, 3)
        penalty_matrix = np.zeros((len(contact), len(contact)))
        weighted = np.zeros_like(contact)
        for i, normal in enumerate(normals):
            rows = slice(3*i, 3*i+3)
            normal_projector = np.outer(normal, normal)
            penalty_matrix[rows, rows] = penalty[3*i]*(
                confidence*np.eye(3)+(1.-confidence)*normal_projector)
            # Analytic symmetric square root of the normal/tangent penalty.
            root = np.sqrt(confidence)*np.eye(3)+(1.-np.sqrt(confidence))*normal_projector
            weighted[rows] = np.sqrt(penalty[3*i])*root@contact[rows]
        singular, rank = np.empty(0), 0
        if len(contact):
            whitened = np.linalg.solve(lower, weighted.T).T
            singular = np.linalg.svd(whitened, compute_uv=False)
            rank = int(np.sum(singular > singular[0]*self.relative_cutoff))
        augmented_mass = mass+contact.T@penalty_matrix@contact
        offset = np.linalg.solve(augmented_mass, mass@qvel+h*(passive-bias))
        response = np.zeros((9, 5))
        response[:, :3] = h*np.linalg.solve(augmented_mass, self._drive)
        # E is tangential by construction. This expression also gives exact
        # zero belt authority at confidence=0 without floating normal leakage.
        surface_penalty = confidence*penalty[:, None]*surface
        response[:, 3:] = -np.linalg.solve(augmented_mass, contact.T@surface_penalty)
        constraint_offset = contact@offset
        constraint_response = contact@response
        constraint_response[:, 3:] += surface
        impulse_offset = -penalty_matrix@constraint_offset
        impulse_response = -penalty_matrix@(contact@response)
        impulse_response[:, 3:] -= surface_penalty

        body_jv, body_jw, spin_maps = [], [], []
        for body in self.body_ids:
            jp, jr = np.zeros((3, 9)), np.zeros((3, 9))
            mujoco.mj_jacBodyCom(model, data, jp, jr, body)
            rotation = data.ximat[body].reshape(3, 3)
            inertia = rotation@np.diag(model.body_inertia[body])@rotation.T
            body_jv.append(jp)
            body_jw.append(jr)
            spin_maps.append(inertia@jr)
        body_jv, body_jw, spin_maps = map(np.asarray, (body_jv, body_jw, spin_maps))
        body_mass = model.body_mass[list(self.body_ids)]
        positions = data.xipos[list(self.body_ids)]
        total_mass = float(body_mass.sum())
        centre = (body_mass[:, None]*positions).sum(axis=0)/total_mass
        centre_jv = (body_mass[:, None, None]*body_jv).sum(axis=0)/total_mass
        momentum = np.array([spin_maps[i]+body_mass[i]*_cross_matrix(positions[i]-centre)
                             @(body_jv[i]-centre_jv) for i in range(4)])
        axial_inertias = []
        for body, joint in zip(self.body_ids[2:], self._joint_ids[1:]):
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, model.body_iquat[body])
            rotation = rotation.reshape(3, 3)
            inertia = rotation@np.diag(model.body_inertia[body])@rotation.T
            arm = model.body_ipos[body]-model.jnt_pos[joint]
            inertia = inertia+model.body_mass[body]*(arm@arm*np.eye(3)-np.outer(arm, arm))
            axis = model.jnt_axis[joint]
            axial_inertias.append(float(axis@inertia@axis))
        return ContactVelocityResponse(
            h, _readonly(offset), _readonly(response), _readonly(free_offset),
            _readonly(free_response), _readonly(mass), _readonly(bias),
            _readonly(passive), _readonly(self._drive), names, _readonly(contact),
            _readonly(surface), _readonly(points), _readonly(normals), _readonly(loads),
            _readonly(friction), _readonly(penalty), _readonly(penalty_matrix), confidence,
            _readonly(impulse_offset),
            _readonly(impulse_response), _readonly(constraint_offset),
            _readonly(constraint_response), _readonly(singular), rank, self.contact_strength,
            _readonly(fork_rotation), _readonly(fork_rotation[:, 1]),
            _readonly(carrier_rotation[:, 0]), _readonly(body_jw),
            _readonly(spin_maps), _readonly(momentum), _readonly(momentum.sum(axis=0)),
            _readonly(centre), total_mass, _readonly(axial_inertias))
