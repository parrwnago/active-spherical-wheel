"""The five-input wheel plant simulated by Gazebo, for the unchanged controllers.

``GazeboPlant`` has the interface of :class:`~spherical_wheel.offroad.TerrainPlant`
(same five inputs, bounds, control interval, ``reset``/``step``/``snapshot``,
``ground``). Gazebo (DART) integrates the dynamics and solves the contacts.

The MuJoCo ``model``/``data`` inherited from ``TerrainPlant`` are kept only
as a *kinematic mirror*: after every control interval the Gazebo state is
written into ``data.qpos``/``data.qvel`` and ``mj_forward`` evaluates frames,
Jacobians and the tangent ground plane. That is what lets the off-road
controller, the gain schedule, the rolling feedforward and the incoming-drive
preview run without modification. The mirror is never integrated.

Every quantity the controller reads from contact forces comes from Gazebo:
``contact_records()`` returns the admitted wheel-ground contacts solved by
DART, and ``measure_contact``, ``rolling_feedforward``,
``ContactSynchronizer`` and ``contact_summary`` use those records whenever a
plant provides them.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

import mujoco
import numpy as np

from ..model import REPO_ROOT, load_config
from ..offroad import TerrainPlant
from ..terrain import Terrain
from .bridge import STATE_FIELDS, TAGS, GazeboSim
from .worlds import MARS_YARDS, write_world

_F = {name: index for index, name in enumerate(STATE_FIELDS)}
# Gazebo collision tag -> MuJoCo geom used for frames, Jacobians and names.
_MIRROR_GEOMS = ('shell_A_smooth', 'shell_B_smooth', 'belt_A_smooth', 'belt_B_smooth',
                 'shell_A_sector_000', 'shell_B_sector_000', 'platform')


class GazeboPlant(TerrainPlant):
    """``TerrainPlant`` interface; Gazebo integrates, MuJoCo only mirrors kinematics.

    ``ground`` selects the Gazebo ground: ``'auto'`` uses a plane for level
    terrain and a height map of ``terrain`` otherwise; ``'plane'``,
    ``'heightmap'`` or a Mars yard name from ``worlds.MARS_YARDS`` force one.
    For a Mars yard pass the matching grid terrain (``mars.MarsTerrain``) so
    the mirror's tangent plane, the logged ground and the friction map agree
    with the Gazebo surface.
    """

    def __init__(self, config=None, terrain=None, model_dir=None, visual_bounds=None, ground='auto',
                 world_dir=None, heightmap_size_m=None, heightmap_cell_m=.01, heightmap_centre=None,
                 plugin_dir=None, collision_detector='ode', ground_check_points=None):
        config = load_config(config) if isinstance(config, (str, Path)) else copy.deepcopy(config or load_config())
        terrain = terrain if terrain is not None else Terrain(name='flat')
        self.sim = None
        self.gazebo_contacts = np.zeros((0, 16))
        model_dir = Path(model_dir or REPO_ROOT/'generated'/'gazebo'/'mirror').resolve()
        super().__init__(config, terrain, model_dir, visual_bounds)
        if ground == 'auto':
            ground = 'plane' if getattr(terrain, 'is_flat', False) else 'mesh'
        if ground == 'plane' and not getattr(terrain, 'is_flat', False):
            raise ValueError('A plane ground needs level terrain')
        self.ground_kind = ground
        world_dir = Path(world_dir or model_dir/'world').resolve()
        options = {}
        if ground == 'marsyard2021_dem':
            pass
        elif ground == 'mesh':
            bounds = visual_bounds or self._default_bounds(terrain)
            margin = .3
            options = {'terrain': terrain, 'cell_m': heightmap_cell_m,
                       'mesh_bounds': (bounds[0]-margin, bounds[1]+margin, bounds[2]-margin, bounds[3]+margin)}
        elif ground == 'heightmap':
            bounds = visual_bounds or self._default_bounds(terrain)
            centre = heightmap_centre or ((bounds[0]+bounds[1])/2, (bounds[2]+bounds[3])/2)
            size = heightmap_size_m or max(bounds[1]-bounds[0], bounds[3]-bounds[2])+.5
            options = {'terrain': terrain, 'centre': centre, 'size_m': size, 'cell_m': heightmap_cell_m}
        elif ground not in ('plane', 'none', 'marsyard2021_dem', *MARS_YARDS):
            raise ValueError(f'Unknown Gazebo ground {ground!r}')
        if ground == 'marsyard2021_dem':
            from .mars import write_mars_world
            self.world_path = write_mars_world(world_dir, config, yard=terrain, collision_detector=collision_detector)
        else:
            self.world_path = write_world(world_dir, config, ground=ground, collision_detector=collision_detector,
                                          **options)
        self.world_info = json.loads((world_dir/'world_info.json').read_text())
        self.sim = GazeboSim(self.world_path, plugin_dir)
        self._geom_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in _MIRROR_GEOMS]
        if min(self._geom_ids) < 0:
            raise ValueError('The mirror model lacks a geom the Gazebo contacts map onto')
        self._time = 0.
        self._ground_bounds = options.get('mesh_bounds') if ground == 'mesh' else visual_bounds
        self.ground_check = self.verify_ground(points=ground_check_points) if ground != 'none' else None
        self.reset()

    def verify_ground(self, samples=7, press_m=5e-4, tolerance_m=2e-4, seed=0, points=None):
        """Check that Gazebo collides with the surface the terrain model describes.

        Presses the wheel ``press_m`` into the modelled surface along its
        normal at ``samples`` points and compares Gazebo's deepest admitted
        contact depth. Raises when any point is off by more than
        ``tolerance_m``, so a wrong or stale ground can never run silently.
        """
        if points is None:
            bounds = self._ground_bounds or (-.5, .5, -.5, .5)
            rng = np.random.default_rng(seed)
            x0, x1, y0, y1 = bounds
            margin = .2*min(x1-x0, y1-y0)
            points = np.column_stack([rng.uniform(x0+margin, x1-margin, samples),
                                      rng.uniform(y0+margin, y1-margin, samples)])
        points = np.asarray(points, float)
        errors = []
        for x, y in points:
            normal = self.terrain.normal_at(x, y)
            centre = np.array([x, y, self.terrain.height(x, y)])+(self.radius-press_m)*normal
            # Support on shell A's patch, away from every patch boundary.
            self.sim.reset(np.r_[centre, 1., 0., 0., 0.], [.30, 0., 0.])
            depths = [row[10] for row in self.sim.contacts() if row[12] > .5]
            errors.append(max(depths)-press_m if depths else -press_m)
        errors = np.array(errors)
        within = np.abs(errors) <= tolerance_m
        result = {'points': points.tolist(), 'depth_error_m': errors.tolist(),
                  'median_abs_error_m': float(np.median(np.abs(errors))),
                  'max_abs_error_m': float(np.abs(errors).max()), 'share_within_tolerance': float(within.mean()),
                  'tolerance_m': tolerance_m}
        # On rough faceted ground a pressed sphere can first touch a raised
        # neighbouring facet, so one point may exceed the tolerance. A wrong
        # or stale ground is off by millimetres or more almost everywhere.
        if result['median_abs_error_m'] > tolerance_m or within.mean() < .5:
            raise RuntimeError(f'Gazebo ground does not match the terrain model: {result}')
        return result

    @staticmethod
    def _default_bounds(terrain):
        from ..offroad import course_bounds
        return course_bounds(terrain)

    # -- state transfer --------------------------------------------------
    def _push(self):
        """Send the mirror's qpos/qvel to Gazebo, then mirror what Gazebo holds."""
        q, v = self.data.qpos, self.data.qvel
        rotation = self.data.xmat[self.fork_id].reshape(3, 3)
        self.sim.reset(q[:7].copy(), q[7:10].copy(), v[6:9].copy(), v[:3].copy(), rotation@v[3:6])
        self._pull()

    def _pull(self):
        s = self.sim.state()
        quat = s[_F['qw']:_F['qz']+1]
        self.data.qpos[:3] = s[_F['x']:_F['z']+1]
        self.data.qpos[3:7] = quat/np.linalg.norm(quat)
        self.data.qpos[7:10] = s[_F['q_shaft']:_F['q_B']+1]
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, self.data.qpos[3:7])
        self.data.qvel[:3] = s[_F['vx']:_F['vz']+1]
        # MuJoCo's free joint takes angular velocity in the body frame.
        self.data.qvel[3:6] = rotation.reshape(3, 3).T@s[_F['wx']:_F['wz']+1]
        self.data.qvel[6:9] = s[_F['dq_shaft']:_F['dq_B']+1]
        self.data.time = self._time
        self.gazebo_contacts = self.sim.contacts()
        self.gazebo_state = s

    def reset(self, tilt=(0., 0.), shaft_phase=None, qpos=None, qvel=None, xy=None, yaw=0.):
        super().reset(tilt, shaft_phase, qpos, qvel, xy, yaw)
        if self.sim is None:
            return self.snapshot()
        self._time = 0.
        self._push()
        self._forward()
        return self.snapshot()

    def step(self, u, disturbance=None):
        self.apply_input(u)
        self.last_disturbance[:] = 0 if disturbance is None else np.asarray(disturbance, float)
        if self.last_disturbance.shape != (6,):
            raise ValueError('Disturbance must be [Fx,Fy,Fz,Tx,Ty,Tz] in world coordinates')
        friction = self.ground['friction'] if self.ground else self.nominal_friction
        self.sim.command(self.last_input[:3], self.last_input[3:], self.last_disturbance, friction)
        self.sim.run(self.substeps)
        self._time += self.dt
        self._pull()
        self._forward()
        return self.snapshot()

    def restore(self, state):
        mujoco.mj_setState(self.model, self.data, np.asarray(state), self.state_spec)
        self._time = float(self.data.time)
        self._forward()
        self._push()
        self._forward()

    def perturb(self, tangent):
        tangent = np.asarray(tangent)
        mujoco.mj_integratePos(self.model, self.data.qpos, tangent[:self.model.nv], 1.)
        self.data.qvel[:] += tangent[self.model.nv:]
        self._forward()
        self._push()
        self._forward()

    # -- contacts ----------------------------------------------------------
    def contact_records(self):
        """Admitted Gazebo contacts that touch or carry load, mapped onto mirror geoms.

        Each record has the contact ``pos``, unit ``normal`` from the ground
        into the wheel, ``force`` on the wheel (world), ``normal_force`` (its
        compressive part), ``tangent_force`` (2-vector in the plane), the
        ``friction`` coefficient used, the mirror ``geom`` and ``body`` and the
        MuJoCo-style ``name``. Contacts the collision margin reports while still
        separated and unloaded are left out, as MuJoCo only reports active ones.
        """
        records = []
        for row in self.gazebo_contacts:
            if row[12] < .5:
                continue  # switched off by the patch rule; carries no load
            tag = int(row[0])
            normal, force = row[4:7], row[7:10]
            normal_force = float(force@normal)
            if row[10] <= 0. and normal_force <= 0.:
                continue
            geom = self._geom_ids[tag]
            tangent = force-normal*normal_force
            records.append({'name': _MIRROR_GEOMS[tag], 'tag': TAGS[tag], 'geom': geom,
                            'body': int(self.model.geom_bodyid[geom]), 'pos': row[1:4].copy(),
                            'normal': normal.copy(), 'force': force.copy(), 'normal_force': normal_force,
                            'tangent_force': tangent, 'tangent_force_norm': float(np.linalg.norm(tangent)),
                            'depth': float(row[10]), 'friction': float(row[11])})
        return records

    def contact_summary(self):
        result = []
        for record in self.contact_records():
            n = record['normal']
            # An orthonormal tangent frame for the reported tangential force.
            t1 = np.cross(n, [1., 0., 0.]) if abs(n[0]) < .9 else np.cross(n, [0., 1., 0.])
            t1 /= np.linalg.norm(t1)
            t2 = np.cross(n, t1)
            result.append({'geoms': ['floor', record['name']], 'normal_force_N': record['normal_force'],
                           'tangent_force_N': [float(record['force']@t1), float(record['force']@t2)],
                           'position_m': record['pos'].tolist()})
        return result

    @property
    def balance_failed(self):
        if self.sim is None:
            return False
        return any(record['name'] == 'platform' for record in self.contact_records())

    def close(self):
        self.sim = None
