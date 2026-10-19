"""Preview velocity matching for an incoming shell or polar belt.

Only commands are returned. The plant's state, model, and applied controls are
never changed. Preview kinematics use a private ``MjData``; the caller must
serialize access to its live plant, as it does for a normal controller call.
The preview is constant-velocity and is not a dynamic contact prediction.
"""
from __future__ import annotations

import math
import mujoco
import numpy as np


class ContactSynchronizer:
    """Prepare an unloaded incoming drive without replacing loaded-drive feedback.

    ``preview(primary_command)`` returns five desired ``commands`` and five
    ``blend_weights``. Integrate as ``(1-w)*primary + w*commands`` and apply via
    ``Plant.step``. Weights are zero for a drive carrying at least 5% of the
    current normal load. Unloaded shell preparation is capped at a 0.35 blend
    because its real reaction torque acts on the carrier and upper body.

    The shell speed ceiling is a declared numerical target limit, not a measured
    motor limit. The physical command is still a torque within Plant.bounds.
    """

    def __init__(self, plant, lookahead_s=.12, transition_margin_m=.012,
                 shell_speed_limit_rad_s=20., shell_speed_gain_Nms=.025,
                 shell_blend_limit=.35, lever_regularization_m=.004):
        values = [lookahead_s, transition_margin_m, shell_speed_limit_rad_s,
                  shell_speed_gain_Nms, lever_regularization_m]
        if not all(np.isfinite(v) and v > 0 for v in values):
            raise ValueError('Synchronization horizons, gains and limits must be positive and finite')
        if not np.isfinite(shell_blend_limit) or not 0 <= shell_blend_limit <= 1:
            raise ValueError('Shell synchronization blend must lie in [0, 1]')
        self.plant = plant
        self.lookahead_s = float(lookahead_s)
        self.transition_margin_m = float(transition_margin_m)
        self.shell_speed_limit_rad_s = float(shell_speed_limit_rad_s)
        self.shell_speed_gain_Nms = float(shell_speed_gain_Nms)
        self.shell_blend_limit = float(shell_blend_limit)
        self.lever_regularization_m = float(lever_regularization_m)
        self._data = mujoco.MjData(plant.model)
        self._floor = self._id(mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        self._patches = []
        for name, index in [('shell_A', 1), ('shell_B', 2), ('belt_A', 3), ('belt_B', 4)]:
            geom = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, name + '_smooth')
            if geom < 0:
                geom = self._id(mujoco.mjtObj.mjOBJ_GEOM, name if name.startswith('belt') else name + '_sector_000')
            if plant.model.nuser_geom < 5:
                raise ValueError('Synchronization requires generated spherical patch metadata')
            sign, lower, upper, _, radius = plant.model.geom_user[geom, :5]
            dof = None
            if name.startswith('shell'):
                joint = self._id(mujoco.mjtObj.mjOBJ_JOINT, name + '_joint')
                dof = int(plant.model.jnt_dofadr[joint])
            self._patches.append({'name': name, 'index': index, 'geom': int(geom),
                                  'body': int(plant.model.geom_bodyid[geom]),
                                  'sign': float(sign), 'lower': float(lower),
                                  'upper': float(upper), 'radius': float(radius), 'dof': dof})

    def _id(self, kind, name):
        value = mujoco.mj_name2id(self.plant.model, kind, name)
        if value < 0:
            raise ValueError(f'Model is missing {name}')
        return value

    def _kinematics(self, qpos):
        self._data.qpos[:] = qpos
        mujoco.mj_kinematics(self.plant.model, self._data)
        mujoco.mj_comPos(self.plant.model, self._data)

    def _support(self, patch, data):
        normal = data.geom_xmat[self._floor].reshape(3, 3)[:, 2]
        center = data.xpos[patch['body']]
        point = center - patch['radius'] * normal
        axis = data.xmat[patch['body']].reshape(3, 3)[:, 0]
        signed_x = patch['sign'] * float(np.dot(-patch['radius'] * normal, axis))
        # At the pole, radius is a coordinate extremum, not an exposed edge.
        margin = signed_x - patch['lower']
        if patch['name'].startswith('shell'):
            margin = min(margin, patch['upper'] - signed_x)
        gap = float(np.dot(point - data.geom_xpos[self._floor], normal))
        return point.copy(), normal.copy(), float(margin), gap

    def _velocity(self, patch, data, qvel, include_surface=True):
        point, normal, margin, gap = self._support(patch, data)
        jacobian = np.zeros((3, self.plant.model.nv))
        mujoco.mj_jac(self.plant.model, data, jacobian, None, point, patch['body'])
        projection = np.eye(3) - np.outer(normal, normal)
        velocity = projection @ (jacobian @ qvel)
        if include_surface and patch['name'].startswith('belt'):
            local_speed = self.plant.model.geom_surfacevel[patch['geom'], :3]
            velocity += projection @ (data.geom_xmat[patch['geom']].reshape(3, 3) @ local_speed)
        return velocity, projection @ jacobian, projection, margin, gap

    def _normal_loads(self):
        loads = {patch['name']: 0. for patch in self._patches}
        velocity_sums = {name: np.zeros(3) for name in loads}
        force = np.zeros(6)
        normal = self.plant.data.geom_xmat[self._floor].reshape(3, 3)[:, 2]
        projection = np.eye(3) - np.outer(normal, normal)
        for number, contact in enumerate(self.plant.data.contact):
            if contact.efc_address < 0 or self._floor not in contact.geom:
                continue
            mujoco.mj_contactForce(self.plant.model, self.plant.data, number, force)
            for geom in contact.geom:
                name = mujoco.mj_id2name(self.plant.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom)) or ''
                for patch_name in loads:
                    if name == patch_name or name.startswith(patch_name + '_'):
                        load = max(0., float(force[0]))
                        loads[patch_name] += load
                        jacobian = np.zeros((3, self.plant.model.nv))
                        mujoco.mj_jac(self.plant.model, self.plant.data, jacobian, None,
                                      contact.pos, int(self.plant.model.geom_bodyid[geom]))
                        velocity = jacobian @ self.plant.data.qvel
                        if patch_name.startswith('belt'):
                            velocity += self.plant.data.geom_xmat[geom].reshape(3, 3) @ self.plant.model.geom_surfacevel[geom, :3]
                        velocity_sums[patch_name] += load * (projection @ velocity)
        return loads, {name: velocity_sums[name] / max(loads[name], 1e-12) for name in loads}

    def preview(self, primary_command=None):
        """Return bounded incoming-drive targets and actual velocity diagnostics.

        Velocities are tangential world-frame velocities at the virtual spherical
        support point. An inactive patch's support point is a kinematic preview,
        not an added collision or an assertion that it touches the floor.
        """
        plant, model = self.plant, self.plant.model
        primary = np.asarray(plant.last_input if primary_command is None else primary_command, float)
        if primary.shape != (5,) or not np.all(np.isfinite(primary)):
            raise ValueError('Primary command must contain five finite values')
        commands = np.clip(primary, -plant.bounds, plant.bounds).copy()
        weights = np.zeros(5)
        qpos, qvel = plant.data.qpos.copy(), plant.data.qvel.copy()
        if not np.all(np.isfinite(qpos)) or not np.all(np.isfinite(qvel)):
            return {'commands': commands.tolist(), 'blend_weights': weights.tolist(),
                    'targets': [], 'warning': 'Nonfinite plant state; synchronization disabled'}
        loads, contact_velocities = self._normal_loads()
        total_load = sum(loads.values())
        # Resolve rapid carrier motion rather than jumping over a narrow cap.
        angular_scale = max(1., float(np.max(abs(qvel[3:]))))
        count = min(16, max(4, int(math.ceil(angular_scale * self.lookahead_s / .15))))
        previews = []
        for elapsed in np.linspace(0., self.lookahead_s, count + 1):
            predicted = qpos.copy()
            mujoco.mj_integratePos(model, predicted, qvel, float(elapsed))
            self._kinematics(predicted)
            previews.append({'time_s': float(elapsed), 'qpos': predicted,
                             'margins': [self._support(patch, self._data)[2] for patch in self._patches]})
        self._kinematics(qpos)
        actual = {patch['name']: self._velocity(patch, self._data, qvel)[0] for patch in self._patches}
        for name, load in loads.items():
            if load > 1e-8:
                actual[name] = contact_velocities[name]
        outgoing_velocity = sum((actual[name] * load for name, load in loads.items()), start=np.zeros(3)) / max(total_load, 1e-12)
        targets = []
        for patch_number, patch in enumerate(self._patches):
            current_margin = previews[0]['margins'][patch_number]
            future = previews[1:]
            best = max(future, key=lambda item: item['margins'][patch_number])
            approaching = best['margins'][patch_number] > current_margin + 1e-7
            exposed = [item for item in future if item['margins'][patch_number] >= 0]
            chosen = exposed[0] if exposed else best
            self._kinematics(chosen['qpos'])
            body_velocity, jacobian, projection, predicted_margin, support_gap = self._velocity(patch, self._data, qvel, include_surface=False)
            activation = np.clip(1. + best['margins'][patch_number] / self.transition_margin_m, 0., 1.)
            activation = float(activation * activation * (3. - 2. * activation))
            load_fraction = loads[patch['name']] / max(total_load, 1e-12)
            unloaded_fraction = float(np.clip(1. - load_fraction / .05, 0., 1.))
            weight = activation * unloaded_fraction if approaching and support_gap <= .02 else 0.
            item = {'name': patch['name'], 'command_index': patch['index'],
                    'lookahead_s': chosen['time_s'], 'approaching': bool(approaching),
                    'currently_exposed': bool(current_margin >= -1e-12),
                    'predicted_exposed': bool(predicted_margin >= -1e-12),
                    'current_patch_margin_m': current_margin,
                    'predicted_patch_margin_m': predicted_margin,
                    'predicted_support_gap_m': support_gap,
                    'normal_load_N': loads[patch['name']], 'normal_load_fraction': load_fraction,
                    'actual_tangential_velocity_m_s': actual[patch['name']].tolist(),
                    'actual_velocity_point': 'force-weighted actual contacts' if loads[patch['name']] > 1e-8 else 'virtual spherical support point',
                    'actual_mismatch_to_outgoing_m_s': (actual[patch['name']] - outgoing_velocity).tolist()}
            if patch['dof'] is None:
                axis = projection @ self._data.geom_xmat[patch['geom']].reshape(3, 3)[:, 1]
                axis_norm_squared = float(axis @ axis)
                target = -float(axis @ body_velocity) / (axis_norm_squared + 1e-8)
                target = float(np.clip(target, -plant.bounds[patch['index']], plant.bounds[patch['index']]))
                residual = body_velocity + axis * target
                if axis_norm_squared < 1e-6:
                    weight = 0.
                commands[patch['index']] = target
                item['target_surface_speed_m_s'] = target
                item['predicted_control_axis_norm'] = math.sqrt(axis_norm_squared)
            else:
                axis = jacobian[:, patch['dof']]
                other_velocity = body_velocity - axis * qvel[patch['dof']]
                lever_squared = float(axis @ axis)
                target = -float(axis @ other_velocity) / (lever_squared + self.lever_regularization_m**2)
                target = float(np.clip(target, -self.shell_speed_limit_rad_s, self.shell_speed_limit_rad_s))
                torque = self.shell_speed_gain_Nms * (target - qvel[patch['dof']])
                torque = float(np.clip(torque, -plant.bounds[patch['index']], plant.bounds[patch['index']]))
                commands[patch['index']] = torque
                residual = other_velocity + axis * target
                weight *= self.shell_blend_limit
                if lever_squared < 1e-8:
                    weight = 0.
                item.update(target_joint_speed_rad_s=target, current_joint_speed_rad_s=float(qvel[patch['dof']]),
                            target_torque_Nm=torque, predicted_contact_lever_m=math.sqrt(lever_squared))
            weights[patch['index']] = weight
            item['blend_weight'] = float(weight)
            item['predicted_matched_tangential_velocity_m_s'] = residual.tolist()
            item['predicted_matched_speed_m_s'] = float(np.linalg.norm(residual))
            targets.append(item)
        return {'commands': commands.tolist(), 'blend_weights': weights.tolist(),
                'targets': targets, 'outgoing_tangential_velocity_m_s': outgoing_velocity.tolist(),
                'lookahead_s': self.lookahead_s,
                'prediction': 'Constant-velocity spherical-patch kinematics; no dynamic contact forecast',
                'shell_speed_limit_rad_s': self.shell_speed_limit_rad_s}


def preview_synchronization(plant, primary_command=None, **options):
    """Convenience wrapper; reuse ContactSynchronizer for repeated control calls."""
    return ContactSynchronizer(plant, **options).preview(primary_command)
