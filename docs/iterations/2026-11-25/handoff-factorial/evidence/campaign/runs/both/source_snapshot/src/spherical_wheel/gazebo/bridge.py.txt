"""Lockstep Gazebo server in this process, driven through the bridge plugin.

``GazeboSim`` loads a world with the gz-sim 8 Python bindings and steps it in
the calling thread. The SphericalWheelBridge plugin applies the command held
in its shared state and publishes the state and admitted contacts after every
iteration; this module reads both through the plugin's C interface. The
library is loaded with ctypes from the same path gz-sim loads it from, so both
see the same shared state.
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path

import numpy as np

from ..model import REPO_ROOT
from .worlds import LEO_WORLDS

PLUGIN_DIR = REPO_ROOT/'build'/'gazebo'
PLUGIN_NAME = 'libSphericalWheelBridge.so'
TAGS = ('shell_A_smooth', 'shell_B_smooth', 'belt_A_smooth', 'belt_B_smooth',
        'shell_A_rim', 'shell_B_rim', 'body')
STATE_FIELDS = ('time', 'iterations', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz',
                'wx', 'wy', 'wz', 'q_shaft', 'q_A', 'q_B', 'dq_shaft', 'dq_A', 'dq_B', 'step_s',
                'resets', 'reset_pending')
RESET_STEP_S = 1e-6


def _prepend(variable, *paths):
    existing = [p for p in os.environ.get(variable, '').split(os.pathsep) if p]
    os.environ[variable] = os.pathsep.join([str(p) for p in paths]+[p for p in existing if p not in map(str, paths)])


def plugin_path(plugin_dir=None):
    path = Path(plugin_dir or PLUGIN_DIR)/PLUGIN_NAME
    if not path.is_file():
        raise FileNotFoundError(f'{path} not found. Build it: cmake -S gazebo/plugin -B build/gazebo && cmake --build build/gazebo')
    return path.resolve()


def environment(world_dir, plugin_dir=None):
    """Point gz-sim at the bridge plugin, the generated model and the Leo models."""
    _prepend('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path(plugin_dir).parent)
    _prepend('GZ_SIM_RESOURCE_PATH', Path(world_dir).resolve()/'models', LEO_WORLDS/'models')


class GazeboSim:
    """One gz-sim server running ``world`` with the wheel model and bridge plugin."""

    _servers = 0

    def __init__(self, world, plugin_dir=None):
        world = Path(world).resolve()
        environment(world.parent, plugin_dir)
        import gz.math7  # noqa: F401  (registers the math types the bindings return)
        from gz.sim8 import Server, ServerConfig
        self.lib = ctypes.CDLL(str(plugin_path(plugin_dir)))
        double_p = ctypes.POINTER(ctypes.c_double)
        self.lib.asw_set_command.argtypes = [double_p, double_p, double_p, ctypes.c_double]
        self.lib.asw_request_reset.argtypes = [double_p]*5
        self.lib.asw_state.argtypes = [double_p]
        self.lib.asw_contacts.argtypes = [double_p, ctypes.c_int]
        self.lib.asw_counters.argtypes = [double_p]
        self.state_size = int(self.lib.asw_state_size())
        self.stride = int(self.lib.asw_contact_stride())
        self._contacts = np.zeros((64, self.stride))
        config = ServerConfig()
        config.set_sdf_file(str(world))
        self.server = Server(config)
        self.world = world
        self.iterations = 0
        self.run(1)  # the plugin initialises in its first update
        if not self.lib.asw_ready():
            raise RuntimeError('The SphericalWheelBridge plugin did not initialise; check the model and plugin paths')

    @staticmethod
    def _ptr(array):
        return np.ascontiguousarray(array, dtype=np.float64).ctypes.data_as(ctypes.POINTER(ctypes.c_double))

    def run(self, iterations):
        # gz-sim never completes a blocking run of paused iterations, so every
        # iteration here is a physics step.
        if not self.server.run(True, int(iterations), False):
            raise RuntimeError('gz-sim server failed to run')
        self.iterations += int(iterations)

    def command(self, torque, belt, wrench=None, friction=.7):
        torque = np.ascontiguousarray(torque, float)
        belt = np.ascontiguousarray(belt, float)
        wrench = np.ascontiguousarray(np.zeros(6) if wrench is None else wrench, float)
        self.lib.asw_set_command(self._ptr(torque), self._ptr(belt), self._ptr(wrench), float(friction))

    def reset(self, pose, joints, speeds=(0., 0., 0.), linear=(0., 0., 0.), angular=(0., 0., 0.)):
        """Set the fork pose (x y z qw qx qy qz), joints and base twist (world).

        The plugin first shrinks the physics step to ``RESET_STEP_S`` and then
        applies the reset during that step, so the returned state differs from
        the request only by a microsecond of motion and the contacts are those
        of the reset state. The nominal step is restored for the next step.
        """
        before = self.state()[STATE_FIELDS.index('resets')]
        arrays = [np.ascontiguousarray(a, float) for a in (pose, joints, speeds, linear, angular)]
        self.lib.asw_request_reset(*[self._ptr(a) for a in arrays])
        for _ in range(6):
            self.run(1)
            state = self.state()
            if state[STATE_FIELDS.index('resets')] > before:
                if state[STATE_FIELDS.index('step_s')] > 10*RESET_STEP_S:
                    raise RuntimeError('Reset was applied during a full-length physics step')
                return state
        raise RuntimeError('The bridge plugin did not apply the reset')

    def state(self):
        out = np.zeros(self.state_size)
        self.lib.asw_state(out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
        return out

    def contacts(self):
        """All contacts, one row each: tag, position, normal, force on wheel, depth, friction, admitted."""
        while True:
            count = self.lib.asw_contacts(self._contacts.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                                          self._contacts.shape[0])
            if count <= self._contacts.shape[0]:
                return self._contacts[:count].copy()
            self._contacts = np.zeros((2*count, self.stride))

    def counters(self):
        out = np.zeros(3)
        self.lib.asw_counters(out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
        return {'admitted': int(out[0]), 'rejected': int(out[1]), 'unmatched': int(out[2])}
