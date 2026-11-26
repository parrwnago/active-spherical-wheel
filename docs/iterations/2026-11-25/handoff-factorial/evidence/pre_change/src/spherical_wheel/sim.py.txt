"""Nonlinear plant. All routine forces enter through internal drives or contacts."""
from __future__ import annotations

from pathlib import Path
import numpy as np
import mujoco
from .model import generate_model, load_config


class Plant:
    input_names = ('shaft', 'shell_A', 'shell_B', 'belt_A', 'belt_B')
    input_units = ('N m', 'N m', 'N m', 'm/s', 'm/s')

    def __init__(self, config=None, model_path=None):
        self.config = load_config(config) if isinstance(config, (str, Path)) else (config or load_config())
        self.model_path = Path(model_path) if model_path else generate_model(self.config)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        from .contact import install_clipped_floor_contact
        self.clipped_contact = install_clipped_floor_contact(self.model, self.config)
        self.data = mujoco.MjData(self.model)
        act = self.config['actuation']
        self.bounds = np.array([*act['torque_limits_Nm'], act['belt_speed_limit_m_s'], act['belt_speed_limit_m_s']], float)
        self.dt = float(self.config['simulation']['control_interval_s'])
        self.substeps = int(round(self.dt / self.model.opt.timestep))
        if not np.isclose(self.substeps * self.model.opt.timestep, self.dt):
            raise ValueError('Controller interval must be an integer number of physics steps')
        self.belt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'belt_' + side) for side in ('A','B')]
        if min(self.belt_ids) < 0:
            raise ValueError('Model must contain two belt contact geoms')
        self.smooth_belt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'belt_' + side + '_smooth') for side in ('A','B')]
        self.all_belt_ids = self.belt_ids + [g for g in self.smooth_belt_ids if g >= 0]
        if not hasattr(self.model, 'geom_surfacevel'):
            raise RuntimeError('This plant requires MuJoCo moving-surface contact support')
        self.model.flg_surfacevel = 1
        self.fork_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'fork')
        self.last_command = np.zeros(5)
        self.last_input = np.zeros(5)
        self.last_disturbance = np.zeros(6)
        self.state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self.reset()

    def reset(self, tilt=(0., 0.), shaft_phase=None, qpos=None, qvel=None):
        mujoco.mj_resetData(self.model, self.data)
        if qpos is not None:
            self.data.qpos[:] = qpos
        else:
            roll, pitch = tilt
            qx = np.array([np.cos(roll/2), np.sin(roll/2), 0., 0.])
            qy = np.array([np.cos(pitch/2), 0., np.sin(pitch/2), 0.])
            mujoco.mju_mulQuat(self.data.qpos[3:7], qy, qx)
            self.data.qpos[7] = self.config['simulation'].get('initial_shaft_phase_rad', .30) if shaft_phase is None else shaft_phase
        if qvel is not None:
            self.data.qvel[:] = qvel
        self.model.geom_surfacevel[self.all_belt_ids] = 0
        self.last_command[:] = 0
        self.last_input[:] = 0
        self.last_disturbance[:] = 0
        mujoco.mj_forward(self.model, self.data)
        return self.snapshot()

    def apply_input(self, u):
        u = np.asarray(u, float)
        if u.shape != (5,) or not np.all(np.isfinite(u)):
            raise ValueError('Expected five finite commands: three torques and two belt speeds')
        self.last_command[:] = u
        self.last_input[:] = np.clip(u, -self.bounds, self.bounds)
        self.data.ctrl[:] = self.last_input[:3]
        self.model.geom_surfacevel[self.all_belt_ids] = 0
        # Each contact geom's local y follows its carrier-mounted module.
        self.model.geom_surfacevel[self.belt_ids, 1] = self.last_input[3:]
        for side, geom in enumerate(self.smooth_belt_ids):
            if geom >= 0:
                self.model.geom_surfacevel[geom, 1] = self.last_input[3+side]
        self.model.flg_surfacevel = 1

    def step(self, u, disturbance=None):
        self.apply_input(u)
        self.data.xfrc_applied[:] = 0
        self.last_disturbance[:] = 0 if disturbance is None else np.asarray(disturbance, float)
        if self.last_disturbance.shape != (6,):
            raise ValueError('Disturbance must be [Fx,Fy,Fz,Tx,Ty,Tz] in world coordinates')
        self.data.xfrc_applied[self.fork_id] = self.last_disturbance
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        return self.snapshot()

    def capture(self):
        state = np.empty(mujoco.mj_stateSize(self.model, self.state_spec))
        mujoco.mj_getState(self.model, self.data, state, self.state_spec)
        return state

    def restore(self, state):
        mujoco.mj_setState(self.model, self.data, np.asarray(state), self.state_spec)
        mujoco.mj_forward(self.model, self.data)

    def error(self, qpos_ref, qvel_ref=None):
        dq = np.zeros(self.model.nv)
        mujoco.mj_differentiatePos(self.model, dq, 1., np.asarray(qpos_ref), self.data.qpos)
        return np.r_[dq, self.data.qvel - (0 if qvel_ref is None else qvel_ref)]

    def perturb(self, tangent):
        tangent = np.asarray(tangent)
        mujoco.mj_integratePos(self.model, self.data.qpos, tangent[:self.model.nv], 1.)
        self.data.qvel[:] += tangent[self.model.nv:]
        mujoco.mj_forward(self.model, self.data)

    def contact_summary(self):
        result = []
        for index, c in enumerate(self.data.contact):
            if c.efc_address < 0:
                continue
            names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or str(g) for g in c.geom]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            result.append({'geoms': names, 'normal_force_N': float(force[0]), 'tangent_force_N': force[1:3].tolist(), 'position_m': c.pos.copy().tolist()})
        return result

    @property
    def balance_failed(self):
        for c in self.data.contact:
            if c.efc_address < 0:
                continue
            names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or '' for g in c.geom]
            if 'floor' in names or 'ground' in names:
                if any(self.model.geom_bodyid[int(g)] == self.fork_id for g in c.geom):
                    return True
        return False

    def snapshot(self):
        q = self.data.qpos[3:7]
        # Tilt of the fork's up-axis is yaw-invariant.
        mat = np.empty(9)
        mujoco.mju_quat2Mat(mat, q)
        tilt = np.arccos(np.clip(mat.reshape(3,3)[2,2], -1., 1.))
        return {'time_s': float(self.data.time), 'qpos': self.data.qpos.copy(), 'qvel': self.data.qvel.copy(),
                'tilt_rad': float(tilt), 'input': self.last_input.copy(), 'command': self.last_command.copy(),
                'disturbance': self.last_disturbance.copy(), 'balance_failed': self.balance_failed}
