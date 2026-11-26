"""Gazebo worlds for the wheel: a level plane, analytic terrains and Mars yards.

Physics is DART (the only gz-physics engine with per-contact surface motion,
which the ideal-servo belts need) through the exact-state physics system in
gazebo/physics. The default collision detector is ODE: its sphere contacts
roll with vertical noise near 1e-7 m/s where Bullet's (used by Leo Rover's
Mars yard worlds) shows 1e-4 to 1e-2 m/s. The step is the MuJoCo model's.

An analytic :class:`~spherical_wheel.terrain.Terrain` is baked into a square
float32 GeoTIFF height map. Gazebo samples a height map on a regular grid and
triangulates it, so unlike the MuJoCo tangent-plane ground the surface is
faceted at the grid spacing; ``heightmap_cell_m`` sets that spacing.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from ..model import REPO_ROOT, load_config
from .model_sdf import MODEL_NAME, write_model

LEO_WORLDS = REPO_ROOT/'assets'/'gazebo'/'third_party'/'leo_simulator-ros2'/'leo_gz_worlds'
MARS_YARDS = {'marsyard2021': 'marsyard2021_terrain', 'marsyard2022': 'marsyard2022_terrain',
              'marsyard2020': 'marsyard2020_terrain'}


def _fmt(values):
    if np.isscalar(values):
        return f'{float(values):.12g}'
    return ' '.join(f'{float(v):.12g}' for v in values)


def _world_skeleton(config, name, scene_broadcaster=True, collision_detector='ode'):
    sdf = ET.Element('sdf', version='1.10')
    world = ET.SubElement(sdf, 'world', name=name)
    physics = ET.SubElement(world, 'physics', name='mujoco_step', type='dart')
    ET.SubElement(physics, 'max_step_size').text = _fmt(config['simulation']['timestep_s'])
    ET.SubElement(physics, 'real_time_factor').text = '0'
    ET.SubElement(physics, 'real_time_update_rate').text = '0'
    dart = ET.SubElement(physics, 'dart')
    ET.SubElement(dart, 'collision_detector').text = collision_detector
    ET.SubElement(world, 'gravity').text = _fmt(config['simulation']['gravity_m_s2'])
    # gz-sim's physics system with exact per-step link state (gazebo/physics/README.md).
    ET.SubElement(world, 'plugin', filename='AswExactPhysics', name='gz::sim::systems::Physics')
    if scene_broadcaster:
        ET.SubElement(world, 'plugin', filename='gz-sim-scene-broadcaster-system',
                      name='gz::sim::systems::SceneBroadcaster')
        ET.SubElement(world, 'plugin', filename='gz-sim-user-commands-system',
                      name='gz::sim::systems::UserCommands')
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.55 0.5 0.45 1'
    ET.SubElement(scene, 'background').text = '0.85 0.72 0.6 1'
    ET.SubElement(scene, 'grid').text = 'false'
    light = ET.SubElement(world, 'light', type='directional', name='sun')
    ET.SubElement(light, 'cast_shadows').text = 'true'
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.85 0.82 0.78 1'
    ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
    return sdf, world


def _wheel(world, pose=(0., 0., .5, 0., 0., 0.)):
    include = ET.SubElement(world, 'include')
    ET.SubElement(include, 'uri').text = f'model://{MODEL_NAME}'
    ET.SubElement(include, 'name').text = MODEL_NAME
    ET.SubElement(include, 'pose').text = _fmt(pose)


def _ground_surface(collision, friction):
    surface = ET.SubElement(collision, 'surface')
    ode = ET.SubElement(ET.SubElement(surface, 'friction'), 'ode')
    ET.SubElement(ode, 'mu').text = _fmt(friction)
    ET.SubElement(ode, 'mu2').text = _fmt(friction)


def write_heightmap(terrain, centre, size_m, cell_m, path):
    """Sample ``terrain`` on a square ``(2^n+1)`` grid and save a 16-bit PNG height map.

    Returns ``(vertices, size_m, low, high)``. Row 0 is the +y edge, as in a
    north-up image. Gazebo stretches the image over ``size_m`` and maps grey
    level 0..65535 onto ``low``..``high`` (the caller's ``<size>`` z and
    ``<pos>`` z), a vertical resolution of ``(high - low)/65535``. A float
    GeoTIFF is not used: Gazebo's DEM loader needs a spatial reference system
    to place one, and without it the collision is flat at the highest sample.
    """
    from PIL import Image
    count = int(2**math.ceil(math.log2(max(size_m/cell_m, 2.)))+1)
    xs = centre[0]+np.linspace(-size_m/2, size_m/2, count)
    ys = centre[1]+np.linspace(size_m/2, -size_m/2, count)
    heights = _grid(terrain, xs, ys)
    low, high = float(heights.min()), float(heights.max())
    levels = np.round((heights-low)/max(high-low, 1e-9)*65535).astype(np.uint16)
    Image.fromarray(levels, mode='I;16').save(Path(path))
    return count, float(size_m), low, high


def write_terrain_mesh(terrain, bounds, cell_m, path):
    """Triangulate ``terrain`` over ``bounds = (x0, x1, y0, y1)`` into an OBJ mesh.

    Vertices are exact samples of the analytic surface on a ``cell_m`` grid;
    each cell is split into two triangles. Returns ``(nx, ny, cell_x, cell_y,
    low, high)``. Used instead of a Gazebo height map, whose image and DEM
    loaders mis-place generated maps (flat at the top sample or offset).
    """
    x0, x1, y0, y1 = bounds
    nx, ny = int(math.ceil((x1-x0)/cell_m))+1, int(math.ceil((y1-y0)/cell_m))+1
    xs, ys = np.linspace(x0, x1, nx), np.linspace(y0, y1, ny)
    heights = _grid(terrain, xs, ys)  # shape (ny, nx)
    gx, gy = np.meshgrid(xs, ys)
    vertices = np.column_stack([gx.ravel(), gy.ravel(), heights.ravel()])
    # gz-physics' DART mesh shape drops a submesh without one normal per vertex.
    dy, dx = np.gradient(heights, ys, xs)
    normals = np.column_stack([-dx.ravel(), -dy.ravel(), np.ones(dx.size)])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    index = np.arange(nx*ny).reshape(ny, nx)
    a, b, c, d = index[:-1, :-1].ravel(), index[:-1, 1:].ravel(), index[1:, 1:].ravel(), index[1:, :-1].ravel()
    faces = np.concatenate([np.column_stack([a, b, c]), np.column_stack([a, c, d])])+1
    with Path(path).open('w') as out:
        out.write('# Analytic terrain sampled on a regular grid (metres, z up).\n')
        np.savetxt(out, vertices, fmt='v %.7f %.7f %.7f')
        np.savetxt(out, normals, fmt='vn %.6f %.6f %.6f')
        np.savetxt(out, np.repeat(faces, 2, axis=1), fmt='f %d//%d %d//%d %d//%d')
    return nx, ny, (x1-x0)/(nx-1), (y1-y0)/(ny-1), float(heights.min()), float(heights.max())


def _grid(terrain, xs, ys):
    if getattr(terrain, 'is_flat', False):
        return np.zeros((len(ys), len(xs)))
    if hasattr(terrain, 'grid_fast'):
        return terrain.grid_fast(xs, ys)
    return terrain.grid(xs, ys)


def write_world(directory, config=None, ground='plane', terrain=None, centre=(0., 0.), size_m=8.,
                cell_m=.01, wheel_pose=(0., 0., .5, 0., 0., 0.), name='asw', scene_broadcaster=True,
                collision_detector='ode', mesh_bounds=None):
    """Write a world and the wheel model under ``directory``; return the world path.

    ``ground`` is ``'plane'`` (level, z = 0), ``'heightmap'`` (bake
    ``terrain`` over a ``size_m`` square around ``centre``) or a Mars yard
    name from :data:`MARS_YARDS`. The wheel model is written to
    ``directory/models``; add it and the Leo models to the resource path.
    """
    c = copy.deepcopy(config if isinstance(config, dict) else load_config(config))
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    write_model(c, directory/'models')
    sdf, world = _world_skeleton(c, name, scene_broadcaster, collision_detector)
    friction = c['contact']['friction'][0]
    info = {'ground': ground, 'collision_detector': collision_detector}
    if ground == 'plane':
        model = ET.SubElement(world, 'model', name='ground')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='ground')
        collision = ET.SubElement(link, 'collision', name='ground')
        geometry = ET.SubElement(ET.SubElement(collision, 'geometry'), 'plane')
        ET.SubElement(geometry, 'normal').text = '0 0 1'
        ET.SubElement(geometry, 'size').text = '100 100'
        _ground_surface(collision, friction)
        visual = ET.SubElement(link, 'visual', name='ground_visual')
        geometry = ET.SubElement(ET.SubElement(visual, 'geometry'), 'plane')
        ET.SubElement(geometry, 'normal').text = '0 0 1'
        ET.SubElement(geometry, 'size').text = '100 100'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0.6 0.62 0.66 1'
        ET.SubElement(material, 'diffuse').text = '0.6 0.62 0.66 1'
    elif ground == 'heightmap':
        if terrain is None:
            raise ValueError('A height-map world needs a terrain')
        digest = hashlib.sha1(json.dumps([terrain.spec(), cell_m, centre, size_m], sort_keys=True,
                                         default=str).encode()).hexdigest()[:12]
        image = directory/f'terrain_heightmap_{digest}.png'
        count, size, low, high = write_heightmap(terrain, centre, size_m, cell_m, image)
        model = ET.SubElement(world, 'model', name='ground')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='ground')
        span = max(high-low, 1e-6)
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name='ground' if kind == 'collision' else 'ground_visual')
            heightmap = ET.SubElement(ET.SubElement(element, 'geometry'), 'heightmap')
            ET.SubElement(heightmap, 'uri').text = str(image)
            ET.SubElement(heightmap, 'size').text = _fmt([size, size, span])
            ET.SubElement(heightmap, 'pos').text = _fmt([centre[0], centre[1], low])
            if kind == 'collision':
                _ground_surface(element, friction)
            else:
                ET.SubElement(heightmap, 'use_terrain_paging').text = 'false'
        info.update(heightmap=str(image), vertices=count, size_m=size, low_m=low, high_m=high,
                    cell_m=size/(count-1), centre=list(centre), terrain=terrain.spec())
    elif ground == 'mesh':
        if terrain is None:
            raise ValueError('A mesh world needs a terrain')
        # Gazebo's mesh manager caches meshes by path for the whole process, so
        # each terrain needs its own file name or a later world reuses the
        # first terrain's surface.
        digest = hashlib.sha1(json.dumps([terrain.spec(), cell_m, mesh_bounds, centre, size_m],
                                         sort_keys=True, default=str).encode()).hexdigest()[:12]
        mesh = directory/f'terrain_mesh_{digest}.obj'
        bounds = mesh_bounds or (centre[0]-size_m/2, centre[0]+size_m/2, centre[1]-size_m/2, centre[1]+size_m/2)
        if mesh.is_file():
            nx, ny, cx, cy, low, high = json.loads(mesh.with_suffix('.json').read_text())
        else:
            nx, ny, cx, cy, low, high = write_terrain_mesh(terrain, bounds, cell_m, mesh)
            mesh.with_suffix('.json').write_text(json.dumps([nx, ny, cx, cy, low, high]))
        model = ET.SubElement(world, 'model', name='ground')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='ground')
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name='ground' if kind == 'collision' else 'ground_visual')
            ET.SubElement(ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh'), 'uri').text = str(mesh)
            if kind == 'collision':
                _ground_surface(element, friction)
            else:
                material = ET.SubElement(element, 'material')
                ET.SubElement(material, 'ambient').text = '0.55 0.45 0.35 1'
                ET.SubElement(material, 'diffuse').text = '0.62 0.5 0.38 1'
        info.update(mesh=str(mesh), bounds=list(bounds), vertices=[nx, ny], cell_m=max(cx, cy),
                    low_m=low, high_m=high, terrain=terrain.spec())
    elif ground == 'none':
        pass  # free space: dynamics checks without contact
    elif ground in MARS_YARDS:
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = f'model://{MARS_YARDS[ground]}'
        ET.SubElement(include, 'name').text = 'marsyard'
        info.update(source=str(LEO_WORLDS/'models'/MARS_YARDS[ground]))
    else:
        raise ValueError(f'Unknown ground {ground!r}')
    _wheel(world, wheel_pose)
    ET.indent(sdf)
    path = directory/f'{name}.sdf'
    ET.ElementTree(sdf).write(path, encoding='unicode', xml_declaration=True)
    (directory/'world_info.json').write_text(json.dumps(info, indent=2)+'\n')
    return path
