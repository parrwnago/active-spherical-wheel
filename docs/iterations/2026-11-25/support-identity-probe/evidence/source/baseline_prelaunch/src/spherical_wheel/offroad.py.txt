"""The nonlinear plant on smooth uneven ground.

The declared spherical-patch contact (``contact.py``) supports one plane. Here
that plane becomes a mocap body which is re-posed before every physics step as
the tangent plane of the analytic terrain at the surface point closest to the
wheel centre. MuJoCo then solves the same unilateral, finite-friction contact
it solves on the flat floor, with the local normal, contact point, penetration
and friction coefficient of the real ground under the wheel.

What this does and does not represent:

* For a sphere on a smooth surface the closest-point tangent plane reproduces
  the contact point, normal and penetration exactly at each step. Curvature
  enters through the plane moving between steps. ``Terrain.audit`` reports the
  tightest valley; a valley tighter than the wheel needs two contacts and is
  outside this model.
* The fork and platform collide with the same plane. That is adequate for
  detecting a fall beside the wheel and nothing more.
* The displayed height field is visual only and never collides.

Nothing here adds a force. Gravity, the five drive commands, logged external
pushes and solved contact forces remain the only loads.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .contact import _clipped_floor_filter
from .model import REPO_ROOT, generate_model, load_config
from .sim import Plant
from .terrain import Terrain

_PROXIES = ('shell_A_smooth', 'shell_B_smooth', 'belt_A_smooth', 'belt_B_smooth')
_DEFAULT_BOUNDS = (-.8, 4.4, -1.2, 1.2)


def course_bounds(terrain, along=(-.8, 4.4), across=(-1.2, 1.2)):
    """World bounding box of a course rectangle laid out along the terrain heading."""
    c, s = terrain._cos, terrain._sin
    corners = np.array([[u, v] for u in along for v in across])
    x = terrain.origin[0]+c*corners[:, 0]-s*corners[:, 1]
    y = terrain.origin[1]+s*corners[:, 0]+c*corners[:, 1]
    return float(x.min()), float(x.max()), float(y.min()), float(y.max())


def build_offroad_model(config, terrain, output_dir, visual_bounds=None, cell_m=.02):
    """Write the terrain variant of the generated model and return its path.

    The stock model is generated unchanged, then its floor plane is moved onto
    a mocap body and a display-only height field is added. Returns
    ``(path, visual)`` where ``visual`` holds the sampled display heights.
    """
    output_dir = Path(output_dir).resolve()
    base = generate_model(config, output_dir)
    root = ET.parse(base).getroot()
    # The stock size limits reserve about 470 MB per MjData. This plant never
    # has more than a few dozen contacts, and suites run several in parallel.
    size = root.find('size')
    for legacy in ('njmax', 'nconmax'):
        size.attrib.pop(legacy, None)
    size.set('memory', '64M')
    world = root.find('worldbody')
    floor = next(geom for geom in world.findall('geom') if geom.get('name') == 'floor')
    world.remove(floor)
    floor.attrib.pop('material', None)
    # Priority makes the ground own the friction coefficient, so a slippery
    # patch can fall below the wheel's nominal value.
    floor.set('priority', '1')
    floor.set('rgba', '0 0 0 0')
    floor.set('group', '4')
    carrier = ET.Element('body', name='terrain_contact', mocap='true', pos='0 0 0')
    carrier.append(floor)
    world.insert(0, carrier)
    x0, x1, y0, y1 = visual_bounds or course_bounds(terrain)
    ncol, nrow = int(round((x1-x0)/cell_m))+1, int(round((y1-y0)/cell_m))+1
    xs, ys = np.linspace(x0, x1, ncol), np.linspace(y0, y1, nrow)
    heights = terrain.grid(xs, ys)
    low, high = float(heights.min()), float(heights.max())
    span = max(high-low, 1e-4)
    asset = root.find('asset')
    ET.SubElement(asset, 'hfield', name='terrain', nrow=str(nrow), ncol=str(ncol),
                  size=f'{(x1-x0)/2:.9g} {(y1-y0)/2:.9g} {span:.9g} 0.05')
    ET.SubElement(asset, 'texture', name='terrain_tex', type='2d', builtin='checker',
                  rgb1='0.52 0.47 0.38', rgb2='0.58 0.53 0.43', width='256', height='256')
    ET.SubElement(asset, 'material', name='terrain_mat', texture='terrain_tex',
                  texrepeat=f'{(x1-x0)/.2:.4g} {(y1-y0)/.2:.4g}', reflectance='0')  # 0.1 m squares
    ET.SubElement(world, 'geom', name='terrain_visual', type='hfield', hfield='terrain',
                  pos=f'{(x0+x1)/2:.9g} {(y0+y1)/2:.9g} {low:.9g}', material='terrain_mat',
                  contype='0', conaffinity='0')
    path = output_dir/'model_offroad.xml'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode', xml_declaration=True)
    (output_dir/'terrain.json').write_text(json.dumps(terrain.spec(), indent=2)+'\n')
    return path, {'xs': xs, 'ys': ys, 'heights': heights, 'low': low, 'span': span}


def install_terrain_contact(model, config):
    """Enable the clipped spherical patches against the mocap ground plane.

    Same patch rule as the flat floor: the callback already evaluates patch
    membership along the plane's actual normal. Returns whether the smooth
    proxies were activated (``contact.mode: analytic_clipped_floor``).
    """
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    if floor < 0 or model.geom_type[floor] != mujoco.mjtGeom.mjGEOM_PLANE:
        raise ValueError('The terrain plant needs the declared ground plane')
    if model.body_mocapid[model.geom_bodyid[floor]] < 0:
        raise ValueError('The ground plane must ride on a mocap body')
    if config.get('contact', {}).get('mode', 'faceted') != 'analytic_clipped_floor':
        return False
    if model.nuser_geom < 5:
        raise ValueError('Clipped contacts require generated geom_user patch metadata')
    for name in _PROXIES:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom < 0:
            raise ValueError(f'Missing clipped sphere proxy {name}')
        model.geom_contype[geom] = 1
        model.geom_conaffinity[geom] = 2
    model.geom_user[floor, 0] = 1
    mujoco.set_mjcb_contactfilter(_clipped_floor_filter)
    return True


def pose_ground_plane(model, data, terrain, floor_id, mocap_id, centre, guess, nominal_friction):
    """Set the mocap plane to the terrain's tangent plane nearest ``centre``.

    Also writes the local friction coefficient onto the plane. Returns
    ``(point, normal, signed_distance, friction)``; pass ``point[:2]`` back as
    the next ``guess``. Call before every ``mj_step`` or ``mj_forward``.
    """
    point, normal, distance = terrain.closest_point(centre, guess)
    quat = np.array([1.+normal[2], -normal[1], normal[0], 0.])
    data.mocap_pos[mocap_id] = point
    data.mocap_quat[mocap_id] = quat/np.linalg.norm(quat)
    friction = float(terrain.friction(point[0], point[1], nominal_friction))
    model.geom_friction[floor_id, 0] = friction
    return point, normal, distance, friction


class TerrainPlant(Plant):
    """``Plant`` on a :class:`Terrain`, with the same five inputs and bounds.

    ``ground`` always describes the plane used by the most recent physics
    evaluation: contact ``point``, unit ``normal``, ``gap`` between the ideal
    sphere and the surface (negative in penetration) and ``friction``.
    """

    def __init__(self, config=None, terrain=None, model_dir=None, visual_bounds=None):
        config = load_config(config) if isinstance(config, (str, Path)) else copy.deepcopy(config or load_config())
        self.terrain = terrain if terrain is not None else Terrain(name='flat')
        self._ground_ready = False
        path, visual = build_offroad_model(config, self.terrain, model_dir or REPO_ROOT/'generated'/'offroad',
                                           visual_bounds)
        # The flat installer refuses a floor that is not fixed to the world.
        # Load with it skipped, then install the terrain variant below.
        stock = copy.deepcopy(config)
        stock['contact'] = dict(stock['contact'], mode='faceted')
        super().__init__(stock, path)
        self.config = config
        self.clipped_contact = install_terrain_contact(self.model, config)
        self.radius = float(config['geometry']['sphere_radius_m'])
        self.floor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        self._mocap = int(self.model.body_mocapid[self.model.geom_bodyid[self.floor_id]])
        self.nominal_friction = float(config['contact']['friction'][0])
        self.total_mass = float(self.model.body_subtreemass[self.fork_id])
        self.visual = visual
        field = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_HFIELD, 'terrain')
        start = int(self.model.hfield_adr[field])
        normalized = (visual['heights']-visual['low'])/visual['span']
        self.model.hfield_data[start:start+normalized.size] = normalized.ravel()
        self._guess = None
        self.ground = None
        self._ground_ready = True
        self.reset()

    def sync_ground(self):
        """Pose the contact plane under the wheel's current position."""
        if not self._ground_ready:
            return
        point, normal, distance, friction = pose_ground_plane(
            self.model, self.data, self.terrain, self.floor_id, self._mocap,
            self.data.qpos[:3], self._guess, self.nominal_friction)
        self._guess = point[:2]
        self.ground = {'point': point, 'normal': normal, 'gap': distance-self.radius, 'friction': friction}

    def _forward(self):
        self.sync_ground()
        mujoco.mj_forward(self.model, self.data)

    def reset(self, tilt=(0., 0.), shaft_phase=None, qpos=None, qvel=None, xy=None, yaw=0.):
        """Reset as ``Plant.reset``; without ``qpos`` the wheel is set on the ground at ``xy``.

        ``tilt`` and ``yaw`` are world roll, pitch and heading in radians. A
        supplied ``qpos`` is used as given, so a flat-floor equilibrium is
        only a resting state where the ground under it is level at z = 0.
        """
        super().reset(tilt, shaft_phase, qpos, qvel)
        if not self._ground_ready:
            return self.snapshot()
        if qpos is None:
            x, y = (0., 0.) if xy is None else (float(xy[0]), float(xy[1]))
            clearance = float(self.config['simulation']['initial_clearance_m'])
            ground = np.array([x, y, self.terrain.height(x, y)])
            self.data.qpos[:3] = ground+(self.radius+clearance)*self.terrain.normal_at(x, y)
            heading = np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)])
            mujoco.mju_mulQuat(self.data.qpos[3:7], heading, self.data.qpos[3:7].copy())
        self._guess = None
        self._forward()
        return self.snapshot()

    def step(self, u, disturbance=None):
        self.apply_input(u)
        self.data.xfrc_applied[:] = 0
        self.last_disturbance[:] = 0 if disturbance is None else np.asarray(disturbance, float)
        if self.last_disturbance.shape != (6,):
            raise ValueError('Disturbance must be [Fx,Fy,Fz,Tx,Ty,Tz] in world coordinates')
        self.data.xfrc_applied[self.fork_id] = self.last_disturbance
        for _ in range(self.substeps):
            self.sync_ground()
            mujoco.mj_step(self.model, self.data)
        self._forward()
        return self.snapshot()

    def restore(self, state):
        mujoco.mj_setState(self.model, self.data, np.asarray(state), self.state_spec)
        self._forward()

    def perturb(self, tangent):
        tangent = np.asarray(tangent)
        mujoco.mj_integratePos(self.model, self.data.qpos, tangent[:self.model.nv], 1.)
        self.data.qvel[:] += tangent[self.model.nv:]
        self._forward()
