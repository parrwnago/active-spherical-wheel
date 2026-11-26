"""Controlled Gazebo contacts: a supported rim pair and an ambiguous V-groove.

These fixtures examine the existing support locator using the *solved* net
wrench, so observer error cannot explain a discrepancy. They do not claim
that arbitrary multi-contact support can be recovered from one wrench.
"""
from __future__ import annotations

import importlib.util
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from spherical_wheel.contact_sensing import SupportLocator, intrinsic_normal
from spherical_wheel.mirror import StateMirror
from spherical_wheel.model import REPO_ROOT, load_config


_PLUGINS = REPO_ROOT/'build'/'gazebo'
_HAVE_GAZEBO = (
    importlib.util.find_spec('gz') is not None
    and importlib.util.find_spec('gz.sim8') is not None
    and (_PLUGINS/'libSphericalWheelBridge.so').is_file()
    and (_PLUGINS/'libAswExactPhysics.so').is_file()
)
pytestmark = pytest.mark.skipif(not _HAVE_GAZEBO, reason='Gazebo environment or plugins not available')


def _world(directory, config, angle=None):
    from spherical_wheel.gazebo.worlds import write_world

    path = write_world(directory, config, ground='plane' if angle is None else 'none',
                       scene_broadcaster=False)
    if angle is not None:
        tree = ET.parse(path)
        world = tree.getroot().find('world')
        # Free space is above both planes: z >= tan(angle) * abs(x).
        # Their intersection forms a narrow concave valley, not one tangent plane.
        for sign in (-1, 1):
            model = ET.SubElement(world, 'model', name=f'groove_face_{sign:+d}')
            ET.SubElement(model, 'static').text = 'true'
            link = ET.SubElement(model, 'link', name='ground')
            collision = ET.SubElement(link, 'collision', name='ground')
            plane = ET.SubElement(ET.SubElement(collision, 'geometry'), 'plane')
            ET.SubElement(plane, 'normal').text = f'{sign*np.sin(angle):.15g} 0 {np.cos(angle):.15g}'
            ET.SubElement(plane, 'size').text = '2 2'
            ode = ET.SubElement(ET.SubElement(ET.SubElement(collision, 'surface'), 'friction'), 'ode')
            for name in ('mu', 'mu2'):
                ET.SubElement(ode, name).text = str(config['contact']['friction'][0])
        ET.indent(tree)
        tree.write(path, encoding='unicode', xml_declaration=True)
    return path


def _mirror_state(mirror, state):
    import mujoco
    from spherical_wheel.gazebo.bridge import STATE_FIELDS

    indices = {name: index for index, name in enumerate(STATE_FIELDS)}
    qpos = np.r_[state[indices['x']:indices['z']+1],
                 state[indices['qw']:indices['qz']+1],
                 state[indices['q_shaft']:indices['q_B']+1]]
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, qpos[3:7])
    qvel = np.r_[state[indices['vx']:indices['vz']+1],
                 rotation.reshape(3, 3).T@state[indices['wx']:indices['wz']+1],
                 state[indices['dq_shaft']:indices['dq_B']+1]]
    mirror.set_state(qpos, qvel, state[indices['time']], np.zeros(5))


def _loaded_rows(sim, minimum_force):
    rows = sim.contacts()
    normal_loads = np.sum(rows[:, 4:7]*rows[:, 7:10], axis=1)
    return rows[(rows[:, 12] > .5) & (normal_loads > minimum_force)]


def _wrench(rows, centre):
    force = rows[:, 7:10].sum(axis=0)
    moment = np.cross(rows[:, 1:4]-centre, rows[:, 7:10]).sum(axis=0)
    return force, moment


def test_gazebo_two_equatorial_rims_are_loaded_and_located(tmp_path):
    """Both shell rims physically support the wheel on the same level plane."""
    from spherical_wheel.gazebo.bridge import GazeboSim

    config = load_config()
    radius = config['geometry']['sphere_radius_m']
    gap = config['geometry']['equatorial_gap_half_m']
    mirror = StateMirror(config, tmp_path/'mirror')
    weight = float(mirror.model.body_subtreemass[mirror.fork_id])*9.81
    locator = SupportLocator(mirror.model, mirror.fork_id, radius, .7, .05*weight)
    sim = GazeboSim(_world(tmp_path/'world', config))
    try:
        sim.command(np.zeros(3), np.zeros(2))
        sim.reset([0., 0., np.sqrt(radius**2-gap**2)-5e-5, 1., 0., 0., 0.], [0., 0., 0.])
        for _ in range(20):
            sim.run(1)
            rows = _loaded_rows(sim, .05*weight)
            # Bridge tags 4 and 5 are the distinct A/B rim collision groups.
            assert set(rows[:, 0].astype(int)) == {4, 5}
            actual = [rows[rows[:, 0] == tag] for tag in (4, 5)]
            loads = np.array([part[:, 9].sum() for part in actual])
            points = np.array([np.average(part[:, 1:4], axis=0, weights=part[:, 9]) for part in actual])
            assert np.linalg.norm(points[0]-points[1]) > 1.5*gap
            assert np.all(loads > .25*weight)
            assert abs(loads.sum()/weight-1.) < .05
            _mirror_state(mirror, sim.state())
            centre = mirror.data.xpos[mirror.fork_id]
            records = locator.records(mirror.data, *_wrench(rows, centre))
            assert {record['name'] for record in records} == {'shell_A_smooth', 'shell_B_smooth'}
            for index, name in enumerate(('shell_A_smooth', 'shell_B_smooth')):
                record = next(record for record in records if record['name'] == name)
                assert np.linalg.norm(record['pos']-points[index]) < 2e-3
                assert abs(record['normal_force']/sum(r['normal_force'] for r in records)
                           -loads[index]/loads.sum()) < .02
    finally:
        # Release this server before another test uses the process-global bridge.
        sim.server = None


def test_gazebo_v_groove_has_two_contacts_but_one_equivalent_support(tmp_path):
    """A perfect net wrench does not recover two support points on one shell.

    Opposing normal forces cancel horizontally. The inferred vertical normal
    and equivalent point are physically meaningful resultant quantities, but
    neither is either of the two local contact normals/points.
    """
    from spherical_wheel.gazebo.bridge import GazeboSim

    config = load_config()
    radius = config['geometry']['sphere_radius_m']
    angle = np.deg2rad(25.)
    mirror = StateMirror(config, tmp_path/'mirror')
    weight = float(mirror.model.body_subtreemass[mirror.fork_id])*9.81
    locator = SupportLocator(mirror.model, mirror.fork_id, radius, .7, .05*weight)
    sim = GazeboSim(_world(tmp_path/'world', config, angle))
    try:
        sim.command(np.zeros(3), np.zeros(2))
        sim.reset([0., 0., (radius-5e-5)/np.cos(angle), 1., 0., 0., 0.], [.60, 0., 0.])
        for _ in range(20):
            sim.run(1)
            rows = _loaded_rows(sim, .1*weight)
            assert len(rows) == 2
            # Tag 0 is shell A's clipped spherical patch; both supports are on it.
            assert np.all(rows[:, 0] == 0)
            assert np.linalg.norm(rows[0, 1:4]-rows[1, 1:4]) > .08
            assert rows[0, 4]*rows[1, 4] < 0.
            normal_angle = np.degrees(np.arccos(np.clip(rows[0, 4:7]@rows[1, 4:7], -1., 1.)))
            assert normal_angle == pytest.approx(50., abs=.1)
            _mirror_state(mirror, sim.state())
            centre = mirror.data.xpos[mirror.fork_id]
            force, moment = _wrench(rows, centre)
            assert abs(force[2]/weight-1.) < .05
            normal = intrinsic_normal(force, moment, radius)
            assert normal is not None and normal[2] > .99
            records = locator.records(mirror.data, force, moment)
            assert len(records) == 1 and records[0]['name'] == 'shell_A_smooth'
            inferred = records[0]['pos']
            assert np.min(np.linalg.norm(rows[:, 1:4]-inferred, axis=1)) > .04
            local_errors = np.degrees(np.arccos(np.clip(rows[:, 4:7]@normal, -1., 1.)))
            assert np.min(local_errors) > 20.
            # Despite the missing contact distribution, the equivalent point
            # represents this symmetric fixture's resultant force and moment.
            np.testing.assert_allclose(np.cross(inferred-centre, force), moment, atol=2e-3)
    finally:
        sim.server = None
