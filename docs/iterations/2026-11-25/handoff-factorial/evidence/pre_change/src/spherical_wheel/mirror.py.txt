"""The controller's view of a robot whose contacts it cannot see.

On the real wheel the controller has the fork's pose and twist from motion
capture, the shaft and shell angles and rates from their encoders, and the
commands it sent. ``StateMirror`` holds exactly that in a MuJoCo model of the
wheel and evaluates frames and Jacobians from it, as the Gazebo plant's mirror
does. It has the plant interface the off-road controller uses (``model``,
``data``, ``dt``, ``bounds``, ``last_input``, ``error`` ...), but collisions are
switched off and there is no terrain or ground plane, so nothing that reads a
contact or the ground can work from it. Whatever the ``hardware`` controller
knows about the ground comes from the momentum observer.

The mirror is never integrated. In simulation ``follow(plant)`` copies a
simulated robot's measurable state into it before every controller call; on
the robot ``set_state`` takes the measurements directly.
"""
from __future__ import annotations

import mujoco
import numpy as np

from .offroad import TerrainPlant
from .terrain import Terrain


class StateMirror(TerrainPlant):
    """Measured state and applied inputs in the wheel model; no contacts, no ground.

    Built from the same generated model as the terrain plant, so body, joint
    and geom numbering match a simulated robot. ``ground`` and ``terrain`` are
    ``None``.
    """

    def __init__(self, config=None, model_dir=None):
        super().__init__(config, Terrain(name='flat'), model_dir)
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.terrain = None
        self.ground = None
        self.set_state(self.data.qpos, self.data.qvel, 0., np.zeros(5))
        self._following = True

    def sync_ground(self):
        """There is no ground to pose."""

    def set_state(self, qpos, qvel, time, applied):
        """Write one measurement and the inputs applied over the interval that ended with it.

        ``qpos``/``qvel`` are MuJoCo's: fork position and quaternion (world),
        shaft and shell angles; fork linear velocity (world), angular velocity
        (fork frame), shaft and shell rates. ``applied`` is the five inputs
        after the actuator bounds: three torques and two belt speeds.
        """
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.time = float(time)
        self.apply_input(applied)
        mujoco.mj_forward(self.model, self.data)

    def follow(self, plant):
        """Measure a simulated robot: its state and the inputs it applied, nothing else."""
        self.set_state(plant.data.qpos, plant.data.qvel, plant.data.time, plant.last_input)

    def reset(self, *args, **kwargs):
        if getattr(self, '_following', False):
            raise RuntimeError('A mirror is not reset; it follows the robot (set_state or follow)')
        return super().reset(*args, **kwargs)

    def step(self, u, disturbance=None):
        raise RuntimeError('A mirror is never integrated; the robot or a simulator advances')
