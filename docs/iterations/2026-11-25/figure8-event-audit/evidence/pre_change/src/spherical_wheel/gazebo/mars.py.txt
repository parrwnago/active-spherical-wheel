"""The ERC 2021 Mars yard (Leo Rover's ``marsyard2021``) for the wheel.

The yard is Leo Rover's ``leo_gz_worlds/models/marsyard2021_terrain`` (MIT,
Fictionlab): a 47 m x 47 m drone-survey DEM (500 x 500 samples, 9.4 cm apart,
6.6 m of relief) plus lava and rock meshes. Gazebo is given:

* the DEM itself as a triangle mesh for collision, one vertex per DEM sample,
  each cell split along the same diagonal as :class:`MarsYard` interpolates.
  Gazebo's own DEM height-map loader resamples the grid (measured 4 mm
  median, 17 mm 90th-percentile differences from the samples), so the mesh
  keeps the collision surface and this module's ground model identical;
* Leo's lava and rock meshes unchanged, with the poses of Leo's model, as
  collision and visual;
* Leo's textured height map as the visual ground.

:class:`MarsYard` has the :class:`~spherical_wheel.terrain.Terrain` interface
the plant and runner use. Its height is exact on the collision mesh; rocks and
lava are not part of it, so ``rock_map`` rasterises them separately for route
selection and for detecting contacts on them.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from ..model import load_config
from .model_sdf import MODEL_NAME, write_model
from .worlds import LEO_WORLDS, _fmt, _wheel, _world_skeleton

YARD = LEO_WORLDS/'models'/'marsyard2021_terrain'
DEM = YARD/'dem'/'marsyard_terrain_hm.tif'
YARD_SIZE_M = 47.
# Leo's model: features link yawed 90 degrees; medium rocks lowered 3 cm.
FEATURES = (('lava', 0.), ('rocks_big', 0.), ('rocks_medium', -.03))


def load_dem(path=DEM):
    from PIL import Image
    return np.asarray(Image.open(path), dtype=float)


class MarsYard:
    """Height, slope and nearest-point queries on the triangulated DEM.

    Sample ``(row, col)`` sits at ``x = -S/2 + col*h``, ``y = S/2 - row*h``
    with ``h = S/(N-1)`` (row 0 is the +y edge, as in Leo's north-up DEM) and
    height equal to the DEM value. Each cell is split along the diagonal from
    its (row, col) corner to its (row+1, col+1) corner, exactly as the
    collision mesh written by :func:`write_dem_mesh`.
    """

    def __init__(self, path=DEM, size_m=YARD_SIZE_M, friction=None, name='marsyard2021'):
        self.heights = load_dem(path)
        self.count = self.heights.shape[0]
        if self.heights.shape != (self.count, self.count):
            raise ValueError('The Mars yard DEM must be square')
        self.size = float(size_m)
        self.spacing = self.size/(self.count-1)
        self.base_friction = friction
        self.name, self.description = name, 'ERC 2021 Mars yard DEM (Leo Rover marsyard2021), 9.4 cm grid.'
        self.is_flat = False
        self.yaw_deg, self.origin, self._cos, self._sin = 0., (0., 0.), 1., 0.
        self.features, self.friction_patches = [], []

    # -- geometry ---------------------------------------------------------
    def _cell(self, x, y):
        col = (x+self.size/2)/self.spacing
        row = (self.size/2-y)/self.spacing
        c0 = int(min(max(math.floor(col), 0), self.count-2))
        r0 = int(min(max(math.floor(row), 0), self.count-2))
        return r0, c0, col-c0, row-r0

    def _plane(self, x, y):
        """Height and world gradient of the triangle containing (x, y)."""
        r, c, u, v = self._cell(x, y)
        h = self.heights
        a, b, cc, d = h[r, c], h[r, c+1], h[r+1, c+1], h[r+1, c]
        if u >= v:   # triangle a, b, cc
            dzdu, dzdv = b-a, cc-b
        else:        # triangle a, cc, d
            dzdu, dzdv = cc-d, d-a
        z = a+u*dzdu+v*dzdv
        # u grows with x, v grows with -y; both in units of one cell.
        return z, dzdu/self.spacing, -dzdv/self.spacing

    def evaluate(self, x, y):
        z, gx, gy = self._plane(float(x), float(y))
        return z, gx, gy, 0., 0., 0.

    def height(self, x, y):
        return self._plane(float(x), float(y))[0]

    def normal_at(self, x, y):
        _, gx, gy = self._plane(float(x), float(y))
        return np.array([-gx, -gy, 1.])/math.sqrt(1.+gx*gx+gy*gy)

    def friction(self, x, y, default):
        return default if self.base_friction is None else self.base_friction

    def closest_point(self, center, guess=None, iterations=30):
        """Nearest surface point to ``center`` on the triangulated DEM."""
        cx, cy, cz = (float(v) for v in center)
        x, y = cx, cy
        best = None
        for _ in range(iterations):
            z, gx, gy = self._plane(x, y)
            normal = np.array([-gx, -gy, 1.])/math.sqrt(1.+gx*gx+gy*gy)
            # Foot of the perpendicular from the centre onto this facet's plane.
            distance = float(normal@(np.array([cx, cy, cz])-np.array([x, y, z])))
            nx, ny = cx-distance*normal[0], cy-distance*normal[1]
            if best is None or abs(distance) < abs(best[2]) - 1e-12 or (abs(nx-x) < 1e-10 and abs(ny-y) < 1e-10):
                best = (np.array([nx, ny, z+gx*(nx-x)+gy*(ny-y)]), normal, distance)
            if abs(nx-x) < 1e-10 and abs(ny-y) < 1e-10:
                break
            x, y = nx, ny
        point, normal, distance = best
        point[2] = self.height(point[0], point[1])
        return point, normal, float(normal@(np.array([cx, cy, cz])-point))

    def heights_at(self, x, y):
        """Vectorised height on the triangulated DEM (same diagonal as ``_plane``)."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        col, row = (x+self.size/2)/self.spacing, (self.size/2-y)/self.spacing
        c = np.clip(np.floor(col).astype(int), 0, self.count-2)
        r = np.clip(np.floor(row).astype(int), 0, self.count-2)
        u, v = col-c, row-r
        h = self.heights
        a, b, cc, d = h[r, c], h[r, c+1], h[r+1, c+1], h[r+1, c]
        upper = u >= v
        return np.where(upper, a+u*(b-a)+v*(cc-b), a+u*(cc-d)+v*(d-a))

    def grid(self, xs, ys):
        gx, gy = np.meshgrid(np.asarray(xs, float), np.asarray(ys, float))
        return self.heights_at(gx, gy)

    grid_fast = grid

    def slope_deg(self, x, y):
        n = self.normal_at(x, y)
        return math.degrees(math.acos(min(1., n[2])))

    def spec(self):
        return {'name': self.name, 'description': self.description, 'source': str(DEM),
                'size_m': self.size, 'samples': self.count, 'spacing_m': self.spacing,
                'base_friction': self.base_friction, 'features': [], 'friction_patches': [],
                'yaw_deg': 0., 'origin': [0., 0.]}


def write_dem_mesh(yard, path):
    """Write the DEM as an OBJ triangle mesh: one vertex per sample, MarsYard's diagonal."""
    n, h = yard.count, yard.spacing
    cols, rows = np.meshgrid(np.arange(n), np.arange(n))
    x = -yard.size/2+cols*h
    y = yard.size/2-rows*h
    vertices = np.column_stack([x.ravel(), y.ravel(), yard.heights.ravel()])
    dzdr, dzdc = np.gradient(yard.heights, h)
    normals = np.column_stack([-dzdc.ravel(), dzdr.ravel(), np.ones(n*n)])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    index = np.arange(n*n).reshape(n, n)
    a, b = index[:-1, :-1].ravel(), index[:-1, 1:].ravel()
    c, d = index[1:, 1:].ravel(), index[1:, :-1].ravel()
    # Rows run toward -y, so (a, c, b) and (a, d, c) wind counter-clockwise from above.
    faces = np.concatenate([np.column_stack([a, c, b]), np.column_stack([a, d, c])])+1
    with Path(path).open('w') as out:
        out.write('# ERC 2021 Mars yard DEM (Leo Rover marsyard2021) as a triangle mesh, metres, z up.\n')
        np.savetxt(out, vertices, fmt='v %.6f %.6f %.6f')
        np.savetxt(out, normals, fmt='vn %.6f %.6f %.6f')
        np.savetxt(out, np.repeat(faces, 2, axis=1), fmt='f %d//%d %d//%d %d//%d')
    return path


def _read_obj(path):
    vertices, faces = [], []
    with Path(path).open() as stream:
        for line in stream:
            if line.startswith('v '):
                vertices.append([float(v) for v in line.split()[1:4]])
            elif line.startswith('f '):
                items = [int(token.split('/')[0]) for token in line.split()[1:]]
                for k in range(1, len(items)-1):
                    faces.append((items[0], items[k], items[k+1]))
    vertices = np.asarray(vertices)
    faces = np.asarray(faces)-1
    return vertices, faces


def feature_triangles():
    """World-frame triangles of the lava and rock meshes, as Leo's model places them."""
    out = {}
    for name, dz in FEATURES:
        vertices, faces = _read_obj(YARD/'meshes'/f'{name}.obj')
        # Link pose yaw +90 deg: (x, y) -> (-y, x).
        world = np.column_stack([-vertices[:, 1], vertices[:, 0], vertices[:, 2]+dz])
        out[name] = world[faces]
    return out


def rock_map(yard, cell_m=.05, bounds=None):
    """Highest lava/rock surface above the DEM on a grid; NaN where there is none.

    Returns ``(xs, ys, height_above_ground)`` with heights shaped (len(ys), len(xs)).
    Triangles are rasterised by their vertices and edge midpoints, which is
    adequate at this cell size for planning and reporting, not for physics.
    """
    x0, x1, y0, y1 = bounds or (-yard.size/2, yard.size/2, -yard.size/2, yard.size/2)
    xs = np.arange(x0, x1+cell_m/2, cell_m)
    ys = np.arange(y0, y1+cell_m/2, cell_m)
    top = np.full((len(ys), len(xs)), -np.inf)
    for triangles in feature_triangles().values():
        points = np.concatenate([triangles.reshape(-1, 3),
                                 ((triangles[:, 0]+triangles[:, 1])/2),
                                 ((triangles[:, 1]+triangles[:, 2])/2),
                                 ((triangles[:, 2]+triangles[:, 0])/2),
                                 triangles.mean(axis=1)])
        i = np.round((points[:, 1]-y0)/cell_m).astype(int)
        j = np.round((points[:, 0]-x0)/cell_m).astype(int)
        keep = (i >= 0) & (i < len(ys)) & (j >= 0) & (j < len(xs))
        np.maximum.at(top, (i[keep], j[keep]), points[keep, 2])
    ground = np.array([[yard.height(x, y) for x in xs] for y in ys])
    above = top-ground
    above[~np.isfinite(above)] = np.nan
    return xs, ys, above


def write_mars_world(directory, config=None, yard=None, collision_detector='ode', wheel_pose=(0., 0., 3., 0., 0., 0.),
                     name='asw_marsyard2021', scene_broadcaster=True):
    """World: Leo's ERC 2021 Mars yard (DEM mesh collision, rocks, lava) and the wheel."""
    c = copy.deepcopy(config if isinstance(config, dict) else load_config(config))
    yard = yard or MarsYard()
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    write_model(c, directory/'models')
    mesh = directory/'marsyard2021_dem.obj'
    if not mesh.is_file():
        write_dem_mesh(yard, mesh)
    sdf, world = _world_skeleton(c, name, scene_broadcaster, collision_detector)
    friction = c['contact']['friction'][0]
    model = ET.SubElement(world, 'model', name='marsyard')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='marsyard_terrain')
    collision = ET.SubElement(link, 'collision', name='ground')
    ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh'), 'uri').text = str(mesh)
    _surface(collision, friction)
    visual = ET.SubElement(link, 'visual', name='terrain_visual')
    heightmap = ET.SubElement(ET.SubElement(visual, 'geometry'), 'heightmap')
    ET.SubElement(heightmap, 'use_terrain_paging').text = 'false'
    ET.SubElement(heightmap, 'uri').text = 'model://marsyard2021_terrain/dem/marsyard_terrain_hm.tif'
    ET.SubElement(heightmap, 'pos').text = '0 0 0'
    ET.SubElement(heightmap, 'size').text = _fmt([yard.size, yard.size, yard.heights.max()-yard.heights.min()])
    texture = ET.SubElement(heightmap, 'texture')
    ET.SubElement(texture, 'diffuse').text = 'model://marsyard2021_terrain/dem/marsyard_terrain_texture.png'
    ET.SubElement(texture, 'normal').text = 'model://marsyard2021_terrain/dem/flat_normal.png'
    ET.SubElement(texture, 'size').text = _fmt(yard.size)
    features = ET.SubElement(model, 'link', name='marsyard_features')
    ET.SubElement(features, 'pose').text = '0 0 0 0 0 1.5707963268'
    for feature, dz in FEATURES:
        for kind in ('collision', 'visual'):
            element = ET.SubElement(features, kind, name=f'{feature}_{kind}')
            if dz:
                ET.SubElement(element, 'pose').text = _fmt([0, 0, dz, 0, 0, 0])
            ET.SubElement(ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh'), 'uri').text = \
                f'model://marsyard2021_terrain/meshes/{feature}.obj'
            if kind == 'collision':
                _surface(element, friction)
    _wheel(world, wheel_pose)
    ET.indent(sdf)
    path = directory/f'{name}.sdf'
    ET.ElementTree(sdf).write(path, encoding='unicode', xml_declaration=True)
    info = {'ground': 'marsyard2021', 'collision_detector': collision_detector, 'dem_mesh': str(mesh),
            'source': str(YARD), 'cell_m': yard.spacing, 'features': [f for f, _ in FEATURES]}
    (directory/'world_info.json').write_text(json.dumps(info, indent=2)+'\n')
    return path


def _surface(collision, friction):
    surface = ET.SubElement(collision, 'surface')
    ode = ET.SubElement(ET.SubElement(surface, 'friction'), 'ode')
    ET.SubElement(ode, 'mu').text = _fmt(friction)
    ET.SubElement(ode, 'mu2').text = _fmt(friction)


# ---------------------------------------------------------------------------
# Routes, runs and evaluation.

class SplinePath:
    """A smooth route through waypoints at a cruise speed, at rest at both ends.

    The curve is a natural cubic spline through the waypoints parametrised by
    chord length; time follows ``ReferencePath``'s cruise profile along arc
    length, so velocity and acceleration are analytic. ``__call__(t)``
    returns world ``(xy, velocity, acceleration)``.
    """

    kind = 'spline'

    def __init__(self, waypoints, speed_m_s=.1, start_s=1., ramp_s=1.5, closed=False):
        from scipy.interpolate import CubicSpline
        points = np.asarray(waypoints, float)
        if closed and not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[:1]])
        chord = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        self.curve = CubicSpline(chord, points, bc_type='periodic' if closed else 'natural')
        sigma = np.linspace(0., chord[-1], 40000)
        speed = np.linalg.norm(self.curve(sigma, 1), axis=1)
        arc = np.r_[0., np.cumsum((speed[1:]+speed[:-1])/2*np.diff(sigma))]
        self._sigma, self._arc = sigma, arc
        self.length_m = float(arc[-1])
        self.speed_m_s, self.start_s, self.ramp_s = float(speed_m_s), float(start_s), float(ramp_s)
        self.duration_s = self.length_m/self.speed_m_s+self.ramp_s
        self.waypoints, self.closed = points, bool(closed)
        self.origin = points[0].copy()
        tangent = self.curve(0., 1)
        self.heading_deg = float(np.degrees(np.arctan2(tangent[1], tangent[0])))
        self.direction = tangent/np.linalg.norm(tangent)

    def _profile(self, t):
        elapsed = float(np.clip(t-self.start_s, 0., self.duration_s))
        ramp, total, distance = self.ramp_s, self.duration_s, self.length_m
        speed = distance/(total-ramp)
        if elapsed < ramp:
            r = elapsed/ramp
            return speed*ramp*(r**3-r**4/2), speed*(3*r**2-2*r**3), speed*(6*r-6*r**2)/ramp
        if elapsed <= total-ramp:
            return speed*(elapsed-ramp/2), speed, 0.
        r = (total-elapsed)/ramp
        return distance-speed*ramp*(r**3-r**4/2), speed*(3*r**2-2*r**3), -speed*(6*r-6*r**2)/ramp

    def __call__(self, t):
        s, v, a = self._profile(t)
        sigma = float(np.interp(s, self._arc, self._sigma))
        d1, d2 = self.curve(sigma, 1), self.curve(sigma, 2)
        norm = float(np.linalg.norm(d1))
        dsigma = 1./norm                                   # d sigma / ds
        ddsigma = -float(d1@d2)/norm**4                    # d^2 sigma / ds^2
        velocity = d1*dsigma*v
        acceleration = (d2*dsigma**2+d1*ddsigma)*v*v+d1*dsigma*a
        return self.curve(sigma), velocity, acceleration

    def spec(self):
        return {'kind': 'spline', 'waypoints': self.waypoints.tolist(), 'closed': self.closed,
                'length_m': self.length_m, 'speed_m_s': self.speed_m_s, 'start_s': self.start_s,
                'ramp_s': self.ramp_s, 'duration_s': self.duration_s, 'origin': self.origin.tolist(),
                'initial_heading_deg': self.heading_deg}


def _cruise(elapsed, distance, total, ramp):
    """``ReferencePath``'s cruise profile: arc length, speed and acceleration at ``elapsed``."""
    elapsed = float(np.clip(elapsed, 0., total))
    speed = distance/(total-ramp)
    if elapsed < ramp:
        r = elapsed/ramp
        return speed*ramp*(r**3-r**4/2), speed*(3*r**2-2*r**3), speed*(6*r-6*r**2)/ramp
    if elapsed <= total-ramp:
        return speed*(elapsed-ramp/2), speed, 0.
    r = (total-elapsed)/ramp
    return distance-speed*ramp*(r**3-r**4/2), speed*(3*r**2-2*r**3), -speed*(6*r-6*r**2)/ramp


class CirclePath:
    """One full circle at a constant cruise speed, at rest at both ends.

    Starts at ``start`` moving along ``heading_deg``; the centre is to the left
    (counter-clockwise) or right (``clockwise``). Arc length follows the same
    cruise profile as the straight routes, so velocity and acceleration are exact.
    """

    kind = 'ring'

    def __init__(self, start, radius_m, heading_deg=90., speed_m_s=.1, start_s=1., ramp_s=1.5, clockwise=False):
        self.origin = np.asarray(start, float)
        self.radius_m, self.heading_deg, self.clockwise = float(radius_m), float(heading_deg), bool(clockwise)
        self.sign = -1. if clockwise else 1.
        angle = math.radians(heading_deg)
        self.direction = np.array([math.cos(angle), math.sin(angle)])
        left = np.array([-self.direction[1], self.direction[0]])
        self.centre = self.origin+self.sign*self.radius_m*left
        self.theta0 = math.atan2(*(self.origin-self.centre)[::-1])
        self.length_m = 2*math.pi*self.radius_m
        self.speed_m_s, self.start_s, self.ramp_s = float(speed_m_s), float(start_s), float(ramp_s)
        self.duration_s = self.length_m/self.speed_m_s+self.ramp_s

    def __call__(self, t):
        s, v, a = _cruise(t-self.start_s, self.length_m, self.duration_s, self.ramp_s)
        theta = self.theta0+self.sign*s/self.radius_m
        radial = np.array([math.cos(theta), math.sin(theta)])
        tangent = self.sign*np.array([-radial[1], radial[0]])
        return self.centre+self.radius_m*radial, v*tangent, a*tangent-v*v/self.radius_m*radial

    def spec(self):
        return {'kind': 'ring', 'start': self.origin.tolist(), 'centre': self.centre.tolist(), 'radius_m': self.radius_m,
                'initial_heading_deg': self.heading_deg, 'clockwise': self.clockwise, 'length_m': self.length_m,
                'speed_m_s': self.speed_m_s, 'start_s': self.start_s, 'ramp_s': self.ramp_s,
                'duration_s': self.duration_s, 'origin': self.origin.tolist()}


class PolylinePath:
    """Straight legs through waypoints, coming to rest at every corner.

    Each leg uses the straight routes' cruise profile (``ramp_s`` speed-up and
    slow-down, ``speed_m_s`` between), so a corner is a genuine stop and turn
    rather than a rounded curve. Position, velocity and acceleration are
    continuous; velocity and acceleration are zero at each waypoint.
    """

    kind = 'polyline'

    def __init__(self, waypoints, speed_m_s=.1, start_s=1., ramp_s=1.5):
        points = np.asarray(waypoints, float)
        legs = np.diff(points, axis=0)
        lengths = np.linalg.norm(legs, axis=1)
        if len(points) < 2 or np.any(lengths <= speed_m_s*ramp_s):
            raise ValueError('Every leg must be longer than the distance covered by one speed ramp')
        self.waypoints, self.lengths, self.units = points, lengths, legs/lengths[:, None]
        self.speed_m_s, self.start_s, self.ramp_s = float(speed_m_s), float(start_s), float(ramp_s)
        self.leg_durations = lengths/self.speed_m_s+self.ramp_s
        self.leg_starts = np.r_[0., np.cumsum(self.leg_durations)[:-1]]
        self.duration_s = float(self.leg_durations.sum())
        self.length_m = float(lengths.sum())
        self.origin, self.direction = points[0].copy(), self.units[0].copy()
        self.heading_deg = float(np.degrees(np.arctan2(self.direction[1], self.direction[0])))

    def __call__(self, t):
        elapsed = float(np.clip(t-self.start_s, 0., self.duration_s))
        k = int(np.clip(np.searchsorted(self.leg_starts, elapsed, side='right')-1, 0, len(self.lengths)-1))
        s, v, a = _cruise(elapsed-self.leg_starts[k], self.lengths[k], self.leg_durations[k], self.ramp_s)
        unit = self.units[k]
        return self.waypoints[k]+s*unit, v*unit, a*unit

    def spec(self):
        return {'kind': 'polyline', 'waypoints': self.waypoints.tolist(), 'leg_lengths_m': self.lengths.tolist(),
                'length_m': self.length_m, 'speed_m_s': self.speed_m_s, 'start_s': self.start_s,
                'ramp_s': self.ramp_s, 'duration_s': self.duration_s, 'origin': self.origin.tolist(),
                'initial_heading_deg': self.heading_deg}


def _straight(start, heading_deg, length_m, speed_m_s=.1):
    from ..offroad_experiments import ReferencePath
    return ReferencePath(length_m, heading_deg, length_m/speed_m_s+1.5, start_s=1., origin=start)


def _line_points(start, heading_deg, length_m):
    angle = math.radians(heading_deg)
    return [start, (start[0]+length_m*math.cos(angle), start[1]+length_m*math.sin(angle))]


# Routes chosen from results/gazebo/mars/terrain_survey.npz (see GAZEBO.md):
# slope is the DEM smoothed to wheel scale (sigma = wheel radius); straight
# routes keep 0.3 m from lava/rock except the deliberate lava crossing.
# ``yaw`` is the platform heading at the start relative to the route heading:
# 0 puts shaft-driven travel along the route, 90 shell-driven travel.
ROUTES = {
    'plain_traverse': {'path': ('line', (-1.79, 7.22), 113., 8.), 'yaw': 0.,
                       'about': 'Easiest 8 m of open plain; about 14 mm of height variation below wheel scale.'},
    'plain_traverse_shell': {'path': ('line', (-1.79, 7.22), 113., 8.), 'yaw': 90.,
                             'about': 'The same plain, rolling on a shell (platform turned 90 deg to the route).'},
    'plain_fast': {'path': ('line', (-1.79, 7.22), 113., 8.), 'yaw': 0., 'speed': .2,
                   'about': 'The same plain at twice the cruise speed (0.2 m/s).'},
    'moderate_climb': {'path': ('line', (12.42, -1.78), -76., 6.), 'yaw': 0.,
                       'about': '6 m climbing 0.19 m over moderate slopes.'},
    'crater_rim_descent': {'path': ('line', (-13.94, 13.31), 67., 5.), 'yaw': 0.,
                           'about': '5 m down the left crater rim, 0.72 m drop.'},
    'crater_rim_climb': {'path': ('line', (-11.99, 17.91), 247., 5.), 'yaw': 0.,
                         'about': 'The same rim driven uphill: 0.72 m climb.'},
    'steep_descent': {'path': ('line', (-6.68, 7.91), -173., 5.), 'yaw': 0.,
                      'about': '5 m into a crater, 0.82 m drop, steeper than the 16.7 deg side-slope bound.'},
    'lava_crossing': {'path': ('line', (-4.6, -0.76), 161., 5.), 'yaw': 0.,
                      'about': '5 m with about a third of the path over lava up to 6.3 cm high.'},
    'plain_circle': {'path': ('circle', (12.41, -1.85), .45, 36.), 'yaw': 0.,
                     'about': 'A 0.45 m radius circle on gentle rock-free ground, counter-clockwise.'},
    'plain_figure8': {'path': ('figure8', (-1.6, 9.6), .45, 40.), 'yaw': 0.,
                      'about': 'A 0.9 m figure-eight on the plain: changing curvature and turn direction.'},
    'plain_patrol': {'path': ('spline', [(1.53, 10.6), (-0.42, 12.07), (-2.58, 11.02), (-3.24, 8.61), (0.1, 7.75),
                                         (0.67, 9.48)], True), 'yaw': None,
                     'about': 'A closed 15 m patrol loop on the plain west of the lava field, '
                              'clear of features over 2 cm by 0.25 m.'},
}


def _polygon(start, heading_deg, side_m, sides):
    """Closed regular polygon, counter-clockwise, first side along ``heading_deg``."""
    turns = heading_deg+np.arange(sides)*360./sides
    steps = side_m*np.c_[np.cos(np.radians(turns)), np.sin(np.radians(turns))]
    return [tuple(p) for p in np.asarray(start, float)+np.r_[np.zeros((1, 2)), np.cumsum(steps, axis=0)]]


def _zigzag(start, heading_deg, leg_m, swing_deg, legs):
    """Legs alternating ``+swing_deg`` and ``-swing_deg`` about ``heading_deg``, starting with ``+``."""
    turns = heading_deg+swing_deg*(-1.)**np.arange(legs)
    steps = leg_m*np.c_[np.cos(np.radians(turns)), np.sin(np.radians(turns))]
    return [tuple(p) for p in np.asarray(start, float)+np.r_[np.zeros((1, 2)), np.cumsum(steps, axis=0)]]


def _out_and_back(start, heading_deg, length_m):
    out = _line_points(start, heading_deg, length_m)
    return [out[0], out[1], out[0]]


# Shape routes (python -m spherical_wheel.gazebo mars-shapes), kept apart from
# ROUTES so the existing suites are unchanged. Each placement was the lowest
# worst-case wheel-scale slope (results/gazebo/mars/route_fields.npz) found on a
# 0.2 m grid of starts and 15 deg of headings, with the whole path at least
# 0.35 m from lava/rock standing more than 2 cm above the DEM.
SHAPE_ROUTES = {
    'shape_circle': {'path': ('ring', (-14.8, -6.), 90., 1.), 'yaw': 0.,
                     'about': 'A 1 m radius circle, counter-clockwise, at a constant 0.1 m/s.'},
    'shape_triangle': {'path': ('polyline', _polygon((2.6, 17.4), 30., 2., 3)), 'yaw': 0.,
                       'about': 'An equilateral triangle with 2 m sides, counter-clockwise, stopping at each corner.'},
    'shape_line_and_back': {'path': ('polyline', _out_and_back((-19.2, 3.2), 285., 3.)), 'yaw': 0.,
                            'about': '3 m straight out, stop, and 3 m straight back to the start.'},
    'shape_zigzag': {'path': ('polyline', _zigzag((.6, 16.6), 15., 1.5, 45., 5)), 'yaw': 0.,
                     'about': 'Five 1.5 m legs at +/-45 deg to the route axis (90 deg turns), stopping at each corner.'},
}


def route_item(name):
    return ROUTES[name] if name in ROUTES else SHAPE_ROUTES[name]


def route_closed(name):
    """Whether a route ends where it starts (then the loop-closure gate applies)."""
    spec = route_item(name)['path']
    kind = spec[0]
    if kind == 'polyline':
        return bool(np.allclose(spec[1][0], spec[1][-1]))
    return kind in ('circle', 'figure8', 'ring') or (kind == 'spline' and spec[2])


def build_route(name):
    from ..mobility import MobilityPath
    item = route_item(name)
    kind = item['path'][0]
    speed = item.get('speed', .1)
    if kind == 'line':
        _, start, heading, length = item['path']
        path = _straight(start, heading, length, speed)
        route_heading = heading
    elif kind in ('circle', 'figure8'):
        _, origin, radius, duration = item['path']
        path = MobilityPath(kind, duration, radius, origin=origin)
        route_heading = 90. if kind == 'circle' else 45.
    elif kind == 'ring':
        _, start, heading, radius = item['path']
        path = CirclePath(start, radius, heading, speed)
        route_heading = heading
    elif kind == 'polyline':
        path = PolylinePath(item['path'][1], speed)
        route_heading = path.heading_deg
    else:
        _, waypoints, closed = item['path']
        path = SplinePath(waypoints, speed, closed=closed)
        route_heading = path.heading_deg
    yaw = route_heading if item.get('yaw') is None else route_heading+item['yaw']
    return path, float(yaw)


def route_samples(path, count=400):
    start = path.start_s
    return np.array([path(t)[0] for t in np.linspace(start, start+path.duration_s, count)])


def route_bounds(names, margin=1.5):
    points = np.vstack([route_samples(build_route(name)[0]) for name in names])
    return (float(points[:, 0].min()-margin), float(points[:, 0].max()+margin),
            float(points[:, 1].min()-margin), float(points[:, 1].max()+margin))


def check_points(names, yard, count=9):
    """Rock-free points along the routes for the Gazebo ground check."""
    xs, ys, above = _ROCKS.get('map') or _rock_cache(yard)
    points = []
    for name in names:
        samples = route_samples(build_route(name)[0], 60)
        for x, y in samples[::max(1, len(samples)//count)]:
            i, j = int(np.clip(round((y-ys[0])/(ys[1]-ys[0])), 0, len(ys)-1)), int(np.clip(round((x-xs[0])/(xs[1]-xs[0])), 0, len(xs)-1))
            window = above[max(i-6, 0):i+7, max(j-6, 0):j+7]
            if not np.any(np.nan_to_num(window, nan=0.) > .005):
                points.append((x, y))
    return points[:count]


_ROCKS = {}


def _rock_cache(yard):
    _ROCKS['map'] = rock_map(yard, cell_m=.05)
    return _ROCKS['map']


def rock_contact_metrics(log, yard, threshold_m=.01):
    """Time the wheel spent touching lava/rock: contact point above the DEM by > threshold."""
    above = []
    for point, normal in zip(log['contact_point_m'], log['contact_force_N']):
        if not np.all(np.isfinite(point)):
            above.append(np.nan)
            continue
        above.append(float(point[2]-yard.height(point[0], point[1])))
    above = np.asarray(above)
    # Unloaded samples report the wheel centre as the contact point; skip them.
    loaded = log['normal_force_N'] > .05*1.86*9.81
    above = np.where(loaded, above, np.nan)
    on_rock = loaded & (np.nan_to_num(above, nan=0.) > threshold_m)
    dt = float(np.median(np.diff(log['time_s']))) if len(log['time_s']) > 1 else 0.
    return {'time_on_rock_s': float(np.sum(on_rock)*dt),
            'max_contact_height_above_dem_m': float(np.nanmax(above)) if np.any(np.isfinite(above)) else float('nan')}


def evaluate_route(log, summary, path, yard, closed):
    """Mobility acceptance gates (mobility.TARGETS) plus terrain figures for a Mars route."""
    from ..mobility import TARGETS
    error = np.linalg.norm(log['qpos'][:, :2]-log['reference_xy'], axis=1)
    moving = (log['time_s'] >= path.start_s) & (log['time_s'] <= path.start_s+path.duration_s)
    p95 = float(np.percentile(error[moving], 95)) if moving.any() else float('inf')
    closure = float(np.linalg.norm(log['qpos'][-1, :2]-log['qpos'][0, :2]))
    heading = np.degrees(np.unwrap(log['heading_rad'])-log['heading_rad'][0])
    gates = {'completed': summary['completed'],
             'rms': summary['tracking']['rms_error_while_moving_m'] <= TARGETS['rms_m'],
             'p95': p95 <= TARGETS['p95_m'],
             'max': summary['tracking']['max_error_m'] <= TARGETS['max_m'],
             'final': summary['tracking']['final_error_m'] <= TARGETS['final_m'],
             'attitude': summary['attitude']['max_error_from_reference_deg'] <= TARGETS['attitude_deg'],
             'saturation': summary['actuation']['saturated_time_fraction'] <= TARGETS['saturated_fraction'],
             'settled': summary['settled_at_end']}
    if closed:
        gates['closure'] = closure <= TARGETS['closure_m']
    xy = log['qpos'][:, :2]
    travelled = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    z = log['qpos'][:, 2]
    extra = {'gates': gates, 'passed': bool(all(gates.values())), 'p95_moving_error_m': p95,
             'loop_closure_m': closure, 'distance_travelled_m': travelled,
             'reference_length_m': float(np.sum(np.linalg.norm(np.diff(route_samples(path, 2000), axis=0), axis=1))),
             'elevation_change_m': float(z[-1]-z[0]), 'elevation_range_m': [float(z.min()), float(z.max())],
             'max_heading_excursion_deg': float(np.max(np.abs(heading)))}
    extra.update(rock_contact_metrics(log, yard))
    summary.pop('distance_along_path_m', None)
    summary.pop('final_cross_track_m', None)
    summary['mars'] = extra
    return summary


# Controller settings compared on the yard. The heading hold is the existing
# OffroadParams option (off by default) at the value of the tuning sweep.
CONTROLLERS = {'terrain_aware': {'mode': 'terrain_aware'},
               'flat_baseline': {'mode': 'flat_baseline'},
               # terrain_aware with the ground force and moment from the momentum observer.
               'momentum_observer': {'mode': 'momentum_observer'},
               'terrain_aware_heading_hold': {'mode': 'terrain_aware', 'heading_gain': .2},
               # Turn the shaft-driven direction onto the fall line, steering with the
               # unloaded shells mostly at pole crossings (results/gazebo/mars_yaw_study).
               'terrain_aware_fall_line': {'mode': 'terrain_aware', 'heading_reference': 'fall_line',
                                           'heading_actuation': 'shells', 'heading_gain': .6,
                                           'heading_damping': .1, 'heading_torque_limit_Nm': .3,
                                           'heading_roll_budget_Nm': .05, 'fall_line_slope_deg': (6., 12.)}}
LABELS = {'terrain_aware': 'contact-aware', 'flat_baseline': 'flat baseline',
          'momentum_observer': 'momentum observer (no force sensing)',
          'terrain_aware_heading_hold': 'contact-aware + heading hold',
          'terrain_aware_fall_line': 'contact-aware + fall-line heading'}

_WORKSPACE = None


def _workspace(root, names):
    global _WORKSPACE
    if _WORKSPACE is None:
        import os
        from .experiments import GazeboWorkspace
        from .parity import SEED_CACHE
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        yard = MarsYard()
        _WORKSPACE = (GazeboWorkspace(directory=Path(root)/f'worker_{os.getpid()}',
                                      seed_cache=SEED_CACHE if SEED_CACHE.is_file() else None,
                                      plant_options={'ground': 'marsyard2021_dem', 'visual_bounds': route_bounds(names),
                                                     'ground_check_points': check_points(names, yard)}), yard)
    return _WORKSPACE


def run_route(job):
    """Run one route with one controller in this worker's Gazebo Mars yard."""
    import time
    from ..offroad_control import OffroadParams
    from ..offroad_experiments import save_run
    from .experiments import run_gazebo
    name, mode, root = job['route'], job['controller'], Path(job['output'])
    phase = job.get('phase')
    workspace, yard = _workspace(job['workspace'], job['all_routes'])
    path, yaw = build_route(name)
    start = time.time()
    log, summary = run_gazebo(workspace, yard, path, OffroadParams(**CONTROLLERS[mode]), yaw_deg=yaw, phase=phase)
    summary = evaluate_route(log, summary, path, yard, route_closed(name))
    summary['route'] = {'name': name, 'about': route_item(name)['about'], 'initial_yaw_deg': yaw, 'path': path.spec(),
                        'start_phase_rad': .30 if phase is None else phase}
    directory = save_run(root/name/mode/(f'phase_{phase:+.2f}' if phase is not None else ''), log, summary, yard)
    plot_route(log, summary, path, yard, directory/'plots.png', f'{name}: {mode}')
    return {'route': name, 'controller': mode, 'summary_path': str(directory/'summary.json'), 'wall_s': time.time()-start,
            'completed': summary['completed'], 'failure': summary['failure'], 'passed': summary['mars']['passed']}


def run_routes(output, routes=None, controllers=('terrain_aware', 'flat_baseline'), workers=4, phases=None):
    """Run routes x controllers; ``phases`` adds runs from other wheel starting phases (rad)."""
    from concurrent.futures import ProcessPoolExecutor, as_completed
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    names = list(routes or ROUTES)
    jobs = [{'route': name, 'controller': mode, 'output': str(output), 'workspace': str(output/'_workspaces'),
             'all_routes': list(ROUTES), 'phase': phase}
            for name in names for mode in controllers for phase in (phases or [None])]
    results = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_route, job): job for job in jobs}
        for future in as_completed(futures):
            job = futures[future]
            try:
                result = future.result()
            except Exception as error:
                result = {'route': job['route'], 'controller': job['controller'], 'error': repr(error)}
            results.append(result)
            print(f"{job['route']:>22} {job['controller']:>13} phase {job['phase']}: " +
                  (f"ERROR {result['error']}" if 'error' in result else
                   f"completed={result['completed']} passed={result['passed']} failure={result['failure']} ({result['wall_s']:.0f} s)"),
                  flush=True)
            (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    write_mars_report(output)
    return results


def plot_route(log, summary, path, yard, output, title):
    from matplotlib.colors import LightSource
    from ..offroad_analysis import _pyplot
    plt = _pyplot()
    xy, ref, t = log['qpos'][:, :2], log['reference_xy'], log['time_s']
    route = route_samples(path, 800)
    span = max(np.ptp(route[:, 0]), np.ptp(route[:, 1]), 1.)+1.2
    cx, cy = (route[:, 0].max()+route[:, 0].min())/2, (route[:, 1].max()+route[:, 1].min())/2
    bounds = (cx-span/2, cx+span/2, cy-span/2, cy+span/2)
    xs, ys = np.linspace(bounds[0], bounds[1], 300), np.linspace(bounds[3], bounds[2], 300)
    heights = yard.grid(xs, ys)
    fig = plt.figure(figsize=(15, 8.5), constrained_layout=True)
    grid = fig.add_gridspec(3, 3)
    ax = fig.add_subplot(grid[:, 0])
    shade = LightSource(315, 40).shade(heights, cmap=plt.cm.copper, vert_exag=3, dx=xs[1]-xs[0], dy=xs[1]-xs[0], blend_mode='soft')
    ax.imshow(shade, extent=bounds, origin='upper')
    rx, ry, above = _ROCKS.get('map') or _rock_cache(yard)
    ax.contourf(rx, ry, np.nan_to_num(above, nan=0.), levels=[.02, .12, 10], colors=['#555555', '#111111'], alpha=.7)
    ax.plot(route[:, 0], route[:, 1], color='#ff8c00', lw=1.5, label='reference')
    ax.plot(xy[:, 0], xy[:, 1], color='#00e5ff', lw=1.2, label='wheel centre')
    ax.plot(*route[0], 'o', color='w', ms=5)
    if summary['failure']:
        ax.plot(*xy[-1], 'x', color='red', ms=10, mew=2, label=summary['failure'])
    ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='x (m)', ylabel='y (m)', aspect='equal', title='Route on the DEM (rocks dark)')
    ax.legend(fontsize=7, loc='lower left')
    error = np.linalg.norm(xy-ref, axis=1)*1e3
    panels = [(grid[0, 1], [(error, 'tracking error')], 'mm'),
              (grid[0, 2], [(np.degrees(log['attitude_error_rad']), 'attitude error'), (np.degrees(np.arccos(np.clip(log['ground_normal'][:, 2], -1, 1))), 'DEM slope under wheel')], 'deg'),
              (grid[1, 1], [(log['normal_force_N']/(1.86*9.81), 'normal load / weight')], ''),
              (grid[1, 2], [(log['qpos'][:, 2], 'wheel centre height')], 'm'),
              (grid[2, 1], [(log['input'][:, 0], 'shaft'), (log['input'][:, 1], 'shell A'), (log['input'][:, 2], 'shell B')], 'N m'),
              (grid[2, 2], [(log['support_loads_N'][:, i]/(1.86*9.81), label) for i, label in enumerate(('shell A', 'shell B', 'belt A', 'belt B'))], 'load share')]
    for slot, series, unit in panels:
        axis = fig.add_subplot(slot)
        for values, label in series:
            axis.plot(t, values, lw=.8, label=label)
        axis.set(ylabel=unit, xlabel='time (s)')
        axis.grid(alpha=.25)
        axis.legend(fontsize=7, loc='upper right')
    m = summary['mars']
    fig.suptitle(f"Gazebo Mars yard - {title}: {'passed' if m['passed'] else 'failed gates'}; "
                 f"max error {summary['tracking']['max_error_m']*1e3:.0f} mm, final {summary['tracking']['final_error_m']*1e3:.0f} mm, "
                 f"max attitude error {summary['attitude']['max_error_from_reference_deg']:.1f} deg"
                 + (f"; FAILED: {summary['failure']}" if summary['failure'] else ''))
    fig.savefig(output, dpi=110)
    plt.close(fig)


def write_mars_report(output):
    output = Path(output)
    rows = []
    for name in ROUTES:
        for mode in CONTROLLERS:
            path = output/name/mode/'summary.json'
            if path.is_file():
                rows.append((name, mode, json.loads(path.read_text())))
    label = LABELS
    lines = ['# Gazebo Mars yard runs (ERC 2021, marsyard2021)', '',
             '| Route | Controller | Finished | Passed all gates | Failed gates | RMS / p95 / max error mm | Final mm | '
             'Max attitude err deg | Max slope under wheel deg | Elevation change m | Peak load x W | Time on rock s | Heading excursion deg |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for name, mode, s in rows:
        m, t = s['mars'], s['tracking']
        failed = [k for k, v in m['gates'].items() if not v]
        lines.append(f"| {name} | {label[mode]} | {'yes' if s['completed'] else 'no: '+str(s['failure'])} | "
                     f"{'yes' if m['passed'] else 'no'} | {', '.join(failed) or '-'} | "
                     f"{t['rms_error_while_moving_m']*1e3:.0f} / {m['p95_moving_error_m']*1e3:.0f} / {t['max_error_m']*1e3:.0f} | "
                     f"{t['final_error_m']*1e3:.0f} | {s['attitude']['max_error_from_reference_deg']:.1f} | "
                     f"{s['ground']['max_slope_under_wheel_deg']:.1f} | {m['elevation_change_m']:+.2f} | "
                     f"{s['contact']['normal_load_fraction_max']:.1f} | {m['time_on_rock_s']:.1f} | {m['max_heading_excursion_deg']:.0f} |")
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def plot_overview(output, controllers=('terrain_aware', 'terrain_aware_heading_hold', 'flat_baseline')):
    """All routes on the yard, one panel per controller setting, coloured by outcome."""
    from matplotlib.colors import LightSource
    from ..offroad_analysis import _pyplot
    plt = _pyplot()
    output = Path(output)
    yard = MarsYard()
    bounds = (-16.5, 15.5, -13.5, 19.5)
    xs, ys = np.linspace(bounds[0], bounds[1], 700), np.linspace(bounds[3], bounds[2], 700)
    shade = LightSource(315, 40).shade(yard.grid(xs, ys), cmap=plt.cm.copper, vert_exag=2,
                                       dx=xs[1]-xs[0], dy=xs[1]-xs[0], blend_mode='soft')
    rx, ry, above = _ROCKS.get('map') or _rock_cache(yard)
    present = [c for c in controllers if any((output/r/c/'rollout.npz').is_file() for r in ROUTES)]
    fig, axes = plt.subplots(1, len(present), figsize=(7.2*len(present), 7.4), constrained_layout=True, squeeze=False)
    colours = {'passed': '#2a9d8f', 'completed': '#e9c46a', 'failed': '#e63946'}
    for axis, mode in zip(axes[0], present):
        axis.imshow(shade, extent=bounds, origin='upper')
        axis.contourf(rx, ry, np.nan_to_num(above, nan=0.), levels=[.02, .12, 10], colors=['#555555', '#111111'], alpha=.7)
        counts = {'passed': 0, 'completed': 0, 'failed': 0}
        for name in ROUTES:
            run = output/name/mode
            if not (run/'rollout.npz').is_file():
                continue
            summary = json.loads((run/'summary.json').read_text())
            log = np.load(run/'rollout.npz')
            state = 'passed' if summary['mars']['passed'] else 'completed' if summary['completed'] else 'failed'
            counts[state] += 1
            path, _ = build_route(name)
            route = route_samples(path, 400)
            axis.plot(route[:, 0], route[:, 1], color='w', lw=.8, ls='--', alpha=.8)
            axis.plot(log['qpos'][:, 0], log['qpos'][:, 1], color=colours[state], lw=1.8)
            if state == 'failed':
                axis.plot(*log['qpos'][-1, :2], 'x', color=colours['failed'], ms=9, mew=2.2)
            axis.annotate(name.replace('_', ' '), route[0], fontsize=6.5, color='w', xytext=(3, 3), textcoords='offset points')
        axis.set(xlim=bounds[:2], ylim=bounds[2:], aspect='equal', xlabel='x (m)', ylabel='y (m)',
                 title=f"{LABELS[mode]}: {counts['passed']} passed all gates, {counts['completed']} finished, "
                       f"{counts['failed']} fell or lost the path")
    handles = [plt.Line2D([], [], color=c, lw=2, label={'passed': 'finished, all gates passed',
                                                         'completed': 'finished, some gates failed',
                                                         'failed': 'fell or > 1 m off (x: where)'}[k]) for k, c in colours.items()]
    handles.append(plt.Line2D([], [], color='w', ls='--', label='reference'))
    axes[0][0].legend(handles=handles, fontsize=7, loc='lower left', facecolor='#333333', labelcolor='w')
    fig.suptitle('Gazebo, ERC 2021 Mars yard: routes driven by each controller setting (rocks and lava dark)')
    fig.savefig(output/'overview.png', dpi=120)
    plt.close(fig)


def robustness_table(output, mode='terrain_aware', phases=(.30, .90, 2.20, -1.)):
    """Per route: finishes, gate passes, median and worst errors over the starting phases."""
    output = Path(output)
    lines = [f'| Route | Finished | Passed all gates | Max error mm, median (worst) | Final error mm, median (worst) | '
             'Max attitude error deg, median (worst) |', '| --- | --- | --- | --- | --- | --- |']
    totals = {'runs': 0, 'finished': 0, 'passed': 0}
    for name in ROUTES:
        rows = []
        for phase in phases:
            sub = output/name/mode/('' if abs(phase-.30) < 1e-9 else f'phase_{phase:+.2f}')/'summary.json'
            if sub.is_file():
                rows.append(json.loads(sub.read_text()))
        if not rows:
            continue
        finished = sum(r['completed'] for r in rows)
        passed = sum(r['mars']['passed'] for r in rows)
        totals['runs'] += len(rows); totals['finished'] += finished; totals['passed'] += passed
        mx = np.array([r['tracking']['max_error_m'] for r in rows])*1e3
        fi = np.array([r['tracking']['final_error_m'] for r in rows])*1e3
        at = np.array([r['attitude']['max_error_from_reference_deg'] for r in rows])
        lines.append(f'| {name} | {finished}/{len(rows)} | {passed}/{len(rows)} | {np.median(mx):.0f} ({mx.max():.0f}) | '
                     f'{np.median(fi):.0f} ({fi.max():.0f}) | {np.median(at):.1f} ({at.max():.1f}) |')
    return '\n'.join(lines), totals
