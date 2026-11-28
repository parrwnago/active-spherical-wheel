"""SDFormat model of the wheel, generated from the MuJoCo model's own inputs.

``generate_model`` (model.py) is run first, so masses, centres of mass,
inertia tensors, collision dimensions and the native CAD display meshes are
the same objects the MuJoCo plant uses. Only the collision representation is
specific to Gazebo:

* ``shell_A_smooth``/``shell_B_smooth`` (on the shells) and
  ``belt_A_smooth``/``belt_B_smooth`` (on the carrier) are spheres of the
  wheel radius, the same proxies MuJoCo's clipped contact uses. The bridge
  plugin admits a contact on one of them only on its own patch.
* ``shell_X_rim_NNN`` capsules trace the equatorial rims: the same 48-sided
  outer rim polygon as the MuJoCo collision wedges, rounded by
  ``RIM_CAPSULE_RADIUS``. In MuJoCo the faceted wedge meshes supply this rim.
* Fork arms, bridge and platform are the same boxes.

Link frames coincide with the MuJoCo body frames: every link origin is at the
wheel centre and all joint positions are zero in the generated pose, so Gazebo
joint angles equal MuJoCo ``qpos[7:10]``.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from ..model import _fork_shapes, generate_model, load_config, mass_properties

MODEL_NAME = 'active_spherical_wheel'
RIM_CAPSULE_RADIUS = .0005  # m
# DART penetration it leaves uncorrected; MuJoCo's soft contact rests at 33.5 um.
CONTACT_ERROR_ALLOWANCE = 5e-5  # m


def _fmt(values):
    if np.isscalar(values):
        return f'{float(values):.12g}'
    return ' '.join(f'{float(v):.12g}' for v in values)


def _rpy_from_z(direction):
    """Roll-pitch-yaw that rotates the local z axis onto ``direction``."""
    d = np.asarray(direction, float)/np.linalg.norm(direction)
    pitch = math.atan2(math.hypot(d[0], d[1]), d[2])
    yaw = math.atan2(d[1], d[0])
    return [0., pitch, yaw]


def rim_segments(config):
    """Capsule end points tracing one rim; returns ``(starts, ends)`` for shell A (+x).

    The MuJoCo wedges' outer rim vertices lie at ``x = gap`` on the sphere,
    radius ``sqrt(R^2 - gap^2)`` from the axis. Each capsule's axis runs
    between consecutive vertices pulled in by its radius, so its outermost
    surface passes through the same vertices.
    """
    g = config['geometry']
    radius, gap = g['sphere_radius_m'], g.get('equatorial_gap_half_m', 0.)
    sectors = int(g['collision_azimuth_sectors'])
    rho = math.sqrt(radius**2-gap**2)-RIM_CAPSULE_RADIUS
    angles = 2*math.pi*np.arange(sectors+1)/sectors
    points = np.column_stack([np.full(sectors+1, gap), rho*np.cos(angles), rho*np.sin(angles)])
    return points[:-1], points[1:]


def _inertial(link, props):
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'pose').text = _fmt([*props['com_m'], 0, 0, 0])
    ET.SubElement(inertial, 'mass').text = _fmt(props['mass_kg'])
    tensor = np.asarray(props['inertia_kg_m2'])
    inertia = ET.SubElement(inertial, 'inertia')
    for name, i, j in (('ixx', 0, 0), ('ixy', 0, 1), ('ixz', 0, 2), ('iyy', 1, 1), ('iyz', 1, 2), ('izz', 2, 2)):
        ET.SubElement(inertia, name).text = _fmt(tensor[i, j])


def _surface(collision, friction):
    surface = ET.SubElement(collision, 'surface')
    ode = ET.SubElement(ET.SubElement(surface, 'friction'), 'ode')
    ET.SubElement(ode, 'mu').text = _fmt(friction)
    ET.SubElement(ode, 'mu2').text = _fmt(friction)
    ET.SubElement(ET.SubElement(surface, 'bounce'), 'restitution_coefficient').text = '0'


def _geometry(parent, kind, **values):
    geometry = ET.SubElement(parent, 'geometry')
    shape = ET.SubElement(geometry, kind)
    for key, value in values.items():
        ET.SubElement(shape, key).text = value if isinstance(value, str) else _fmt(value)
    return geometry


def _visual(link, name, kind, rgba, pose=None, **values):
    visual = ET.SubElement(link, 'visual', name=name)
    if pose is not None:
        ET.SubElement(visual, 'pose').text = _fmt(pose)
    _geometry(visual, kind, **values)
    material = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(material, tag).text = _fmt(rgba)
    ET.SubElement(material, 'specular').text = '0.2 0.2 0.2 1'
    return visual


def write_model(config=None, models_dir=None):
    """Write ``<models_dir>/active_spherical_wheel/{model.sdf, model.config, meshes/}``.

    Returns the model directory. Mesh URIs use ``model://``, so add
    ``models_dir`` to ``GZ_SIM_RESOURCE_PATH`` (``bridge.environment`` does).
    """
    c = copy.deepcopy(config if isinstance(config, dict) else load_config(config))
    models_dir = Path(models_dir).resolve()
    directory = models_dir/MODEL_NAME
    source = directory/'mujoco'
    generate_model(c, source)
    meshes = directory/'meshes'
    meshes.mkdir(parents=True, exist_ok=True)
    for name in ('native_shell_positive.obj', 'native_shell_negative.obj', 'native_carrier.obj', 'belt_A.obj', 'belt_B.obj'):
        if (source/'meshes'/name).exists():
            shutil.copy2(source/'meshes'/name, meshes/name)
    props = mass_properties(c)
    g, s, a = c['geometry'], c['simulation'], c['actuation']
    radius = g['sphere_radius_m']
    friction = c['contact']['friction'][0]
    polar_rim = radius*math.cos(math.radians(g['polar_cap_angle_deg']))

    sdf = ET.Element('sdf', version='1.10')
    model = ET.SubElement(sdf, 'model', name=MODEL_NAME)
    ET.SubElement(model, 'self_collide').text = 'false'
    links = {}
    for name in ('fork', 'carrier', 'shell_A', 'shell_B'):
        links[name] = ET.SubElement(model, 'link', name=name)
        ET.SubElement(links[name], 'pose').text = '0 0 0 0 0 0'
        _inertial(links[name], props[name])

    for shape in _fork_shapes(c):
        pose = [*shape['pos'], 0, 0, 0]
        collision = ET.SubElement(links['fork'], 'collision', name=shape['name'])
        ET.SubElement(collision, 'pose').text = _fmt(pose)
        _geometry(collision, 'box', size=shape['size'])
        _surface(collision, friction)
        _visual(links['fork'], shape['name']+'_visual', 'box', [.75, .78, .83, 1], pose, size=shape['size'])

    carrier = links['carrier']
    _visual(carrier, 'shaft_visual', 'cylinder', [.35, .38, .42, 1], [0, 0, 0, math.pi/2, 0, 0],
            radius=g['shaft_radius_m'], length=g['shaft_length_m'])
    if (meshes/'native_carrier.obj').exists():
        _visual(carrier, 'native_carrier_visual', 'mesh', [.63, .61, .55, 1], uri=f'model://{MODEL_NAME}/meshes/native_carrier.obj')
    for side in ('A', 'B'):
        _visual(carrier, f'belt_{side}_visual', 'mesh', [.17, .19, .22, 1], uri=f'model://{MODEL_NAME}/meshes/belt_{side}.obj')
        collision = ET.SubElement(carrier, 'collision', name=f'belt_{side}_smooth')
        _geometry(collision, 'sphere', radius=radius)
        _surface(collision, friction)

    starts, ends = rim_segments(c)
    for name, sign, rgba, native in (('shell_A', 1., [.08, .54, .69, 1], 'native_shell_positive.obj'),
                                     ('shell_B', -1., [.11, .35, .49, 1], 'native_shell_negative.obj')):
        link = links[name]
        collision = ET.SubElement(link, 'collision', name=f'{name}_smooth')
        _geometry(collision, 'sphere', radius=radius)
        _surface(collision, friction)
        for index, (start, end) in enumerate(zip(starts, ends)):
            start, end = start*np.array([sign, 1, 1]), end*np.array([sign, 1, 1])
            collision = ET.SubElement(link, 'collision', name=f'{name}_rim_{index:03d}')
            ET.SubElement(collision, 'pose').text = _fmt([*(start+end)/2, *_rpy_from_z(end-start)])
            _geometry(collision, 'capsule', radius=RIM_CAPSULE_RADIUS, length=np.linalg.norm(end-start))
            _surface(collision, friction)
        if (meshes/native).exists():
            _visual(link, f'native_{name}_visual', 'mesh', rgba, uri=f'model://{MODEL_NAME}/meshes/{native}')
        else:
            _visual(link, f'{name}_visual', 'sphere', rgba, radius=radius)

    for name, parent, child, axis, limit in (('shaft', 'fork', 'carrier', [0, 1, 0], a['torque_limits_Nm'][0]),
                                              ('shell_A_joint', 'carrier', 'shell_A', [1, 0, 0], a['torque_limits_Nm'][1]),
                                              ('shell_B_joint', 'carrier', 'shell_B', [1, 0, 0], a['torque_limits_Nm'][2])):
        joint = ET.SubElement(model, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent').text = parent
        ET.SubElement(joint, 'child').text = child
        axis_element = ET.SubElement(joint, 'axis')
        ET.SubElement(axis_element, 'xyz').text = _fmt(axis)
        limits = ET.SubElement(axis_element, 'limit')
        ET.SubElement(limits, 'lower').text = '-1e16'
        ET.SubElement(limits, 'upper').text = '1e16'
        ET.SubElement(limits, 'effort').text = _fmt(limit)
        ET.SubElement(limits, 'velocity').text = '-1'
        dynamics = ET.SubElement(axis_element, 'dynamics')
        ET.SubElement(dynamics, 'damping').text = _fmt(s['joint_damping_Nms'])
        ET.SubElement(dynamics, 'friction').text = '0'
        ET.SubElement(dynamics, 'spring_stiffness').text = '0'

    plugin = ET.SubElement(model, 'plugin', filename='SphericalWheelBridge', name='asw::SphericalWheelBridge')
    ET.SubElement(plugin, 'sphere_radius').text = _fmt(radius)
    ET.SubElement(plugin, 'equatorial_gap_half').text = _fmt(g.get('equatorial_gap_half_m', 0.))
    ET.SubElement(plugin, 'polar_rim').text = _fmt(polar_rim)
    ET.SubElement(plugin, 'fork_com').text = _fmt(props['fork']['com_m'])
    ET.SubElement(plugin, 'contact_error_allowance').text = _fmt(CONTACT_ERROR_ALLOWANCE)

    ET.indent(sdf)
    directory.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(sdf).write(directory/'model.sdf', encoding='unicode', xml_declaration=True)
    config_xml = ET.Element('model')
    ET.SubElement(config_xml, 'name').text = MODEL_NAME
    ET.SubElement(config_xml, 'version').text = '1.0'
    ET.SubElement(config_xml, 'sdf', version='1.10').text = 'model.sdf'
    ET.SubElement(config_xml, 'description').text = ('Active spherical wheel: fork and platform, carrier with polar belts, '
                                                     'two independently driven shells. Generated from config/model.yaml.')
    ET.indent(config_xml)
    ET.ElementTree(config_xml).write(directory/'model.config', encoding='unicode', xml_declaration=True)
    return directory
