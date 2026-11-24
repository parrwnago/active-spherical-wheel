"""The Gazebo reconstruction: controller-side equivalence and simulator parity.

The first group needs only MuJoCo. The second needs the Gazebo environment
(gz-sim 8 Python bindings) and the built plugins (build/gazebo); it is
skipped otherwise. See GAZEBO.md.
"""
from __future__ import annotations

import copy
import importlib.util

import mujoco
import numpy as np
import pytest

from spherical_wheel.contact_sensing import measure_contact, measure_records, records_from_mujoco
from spherical_wheel.experiments import rolling_feedforward, rolling_feedforward_records
from spherical_wheel.model import REPO_ROOT, generate_model, load_config
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import Workspace, place_on_ground, scenario
from spherical_wheel.sim import Plant

PLUGINS = REPO_ROOT/'build'/'gazebo'
HAVE_GAZEBO = (importlib.util.find_spec('gz') is not None and importlib.util.find_spec('gz.sim8') is not None
               and (PLUGINS/'libSphericalWheelBridge.so').is_file() and (PLUGINS/'libAswExactPhysics.so').is_file())
gazebo = pytest.mark.skipif(not HAVE_GAZEBO, reason='Gazebo environment or plugins not available')


@pytest.fixture(scope='module')
def workspace():
    return Workspace()


def _compare(a, b):
    if isinstance(a, dict):
        return max((_compare(a[key], b[key]) for key in a), default=0.)
    if isinstance(a, (bool, int)):
        return float(a != b)
    a, b = np.asarray(a, float), np.asarray(b, float)
    if not a.size:
        return 0.
    both_nan = np.isnan(a) & np.isnan(b)
    return float(np.max(np.where(both_nan, 0., np.abs(a-b))))


def test_contact_records_reproduce_the_mujoco_path_exactly(workspace):
    """Gazebo feeds the controller contact records; built from MuJoCo they give identical outputs."""
    terrain, path = scenario('hills')
    plant = workspace.plant(terrain)
    controller = OffroadController(plant, workspace.design, OffroadParams())
    place_on_ground(plant, controller, path.origin, .30)
    worst = 0.
    for _ in range(500):
        xy, velocity, acceleration = path(plant.data.time)
        records = records_from_mujoco(plant)
        worst = max(worst, _compare(measure_contact(plant), measure_records(plant, records)))
        for one, two in zip(rolling_feedforward(plant, np.asarray(velocity)),
                            rolling_feedforward_records(plant, records, np.asarray(velocity))):
            worst = max(worst, _compare(one, two))
        loads, speeds = controller.synchronizer._normal_loads()
        loads2, speeds2 = controller.synchronizer._normal_loads_records(records)
        worst = max(worst, _compare(loads, loads2), _compare(speeds, speeds2))
        applied, _ = controller.command(xy, velocity, acceleration)
        plant.step(applied)
        controller.after_step()
    assert worst == 0.


def test_yaw_placement_turns_heading_and_keeps_the_world_lean(workspace):
    terrain, path = scenario('ramp_forward')
    plant = workspace.plant(terrain)
    controller = OffroadController(plant, workspace.design, OffroadParams())
    ups = {}
    for yaw in (0., 90.):
        place_on_ground(plant, controller, (.6, 0.), .30, yaw_deg=yaw)
        rotation = plant.data.xmat[plant.fork_id].reshape(3, 3)
        ups[yaw] = rotation[:, 2].copy()
        heading = np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))
        assert abs(heading-yaw) < .5
    # On the slope the balancing lean is about world axes, whatever the heading.
    assert np.degrees(np.arccos(np.clip(ups[0.]@ups[90.], -1., 1.))) < .05
    assert np.degrees(np.arccos(ups[0.][2])) > 5.


def test_spline_route_has_exact_derivatives():
    from spherical_wheel.gazebo.mars import SplinePath
    path = SplinePath([(0., 0.), (2., 1.), (3., 3.), (1., 4.)], speed_m_s=.1)
    step = 1e-4
    for t in np.linspace(path.start_s+.5, path.start_s+path.duration_s-.5, 9):
        velocity = (path(t+step)[0]-path(t-step)[0])/(2*step)
        acceleration = (path(t+step)[1]-path(t-step)[1])/(2*step)
        assert np.allclose(velocity, path(t)[1], atol=2e-6)
        assert np.allclose(acceleration, path(t)[2], atol=2e-6)
    assert np.allclose(path(0.)[1], 0.) and np.allclose(path(path.start_s+path.duration_s+1.)[1], 0.)


def test_shape_routes_have_exact_derivatives_and_stop_at_corners():
    from spherical_wheel.gazebo.mars import SHAPE_ROUTES, build_route, route_closed
    step = 1e-4
    for name in SHAPE_ROUTES:
        path, yaw = build_route(name)
        assert np.isclose(yaw, path.heading_deg)
        for t in np.linspace(path.start_s+.3, path.start_s+path.duration_s-.3, 23):
            velocity = (path(t+step)[0]-path(t-step)[0])/(2*step)
            acceleration = (path(t+step)[1]-path(t-step)[1])/(2*step)
            assert np.allclose(velocity, path(t)[1], atol=2e-6), name
            assert np.allclose(acceleration, path(t)[2], atol=2e-5), name
        end = path.start_s+path.duration_s
        assert np.allclose(path(0.)[1], 0.) and np.allclose(path(end+1.)[1], 0.)
        assert np.isclose(np.linalg.norm(path(path.start_s+path.duration_s/2)[1]), .1) or path.kind == 'polyline'
        if path.kind == 'polyline':
            for corner, point in zip(path.start_s+path.leg_starts[1:], path.waypoints[1:-1]):
                assert np.allclose(path(corner)[0], point) and np.allclose(path(corner)[1], 0.)
        assert route_closed(name) == np.allclose(path(end)[0], path(0.)[0]), name


@pytest.mark.skipif(not (REPO_ROOT/'assets'/'gazebo'/'third_party'/'leo_simulator-ros2').is_dir(),
                    reason='Leo Rover Mars yard not fetched (scripts/fetch-marsyard.sh)')
def test_mars_yard_surface_matches_its_samples_and_facets():
    from spherical_wheel.gazebo.mars import MarsYard
    yard = MarsYard()
    rng = np.random.default_rng(0)
    rows, cols = rng.integers(1, yard.count-1, 50), rng.integers(1, yard.count-1, 50)
    x, y = -yard.size/2+cols*yard.spacing, yard.size/2-rows*yard.spacing
    assert np.allclose(yard.heights_at(x, y), yard.heights[rows, cols], atol=1e-9)
    for px, py in rng.uniform(-20, 20, (50, 2)):
        z, gx, gy = yard._plane(px, py)
        assert abs(yard.heights_at(px, py)-z) < 1e-9
        normal = yard.normal_at(px, py)
        assert abs(np.linalg.norm(normal)-1.) < 1e-12 and np.allclose(normal[:2]/normal[2], [-gx, -gy])


# -- Gazebo -------------------------------------------------------------------
def _gazebo_state(state):
    from spherical_wheel.gazebo.bridge import STATE_FIELDS
    f = {name: i for i, name in enumerate(STATE_FIELDS)}
    q = np.r_[state[f['x']:f['z']+1], state[f['qw']:f['qz']+1], state[f['q_shaft']:f['q_B']+1]]
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, q[3:7])
    v = np.r_[state[f['vx']:f['vz']+1], rotation.reshape(3, 3).T@state[f['wx']:f['wz']+1],
              state[f['dq_shaft']:f['dq_B']+1]]
    return q, v


@gazebo
def test_free_space_dynamics_match_mujoco(tmp_path):
    """Same inertias, joints and torque sequence: Gazebo and MuJoCo agree to 1e-6."""
    from spherical_wheel.gazebo.bridge import GazeboSim
    from spherical_wheel.gazebo.worlds import write_world
    config = load_config()
    config['simulation']['gravity_m_s2'] = [0., 0., 0.]
    plant = Plant(copy.deepcopy(config), generate_model(config, tmp_path/'mujoco'))
    plant.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    q0 = np.r_[.1, -.2, 1., np.array([.9, .1, -.2, .3])/np.linalg.norm([.9, .1, -.2, .3]), .4, 1., -2.]
    v0 = np.array([.05, -.02, .03, .4, -.3, .2, .5, 3., -2.])
    plant.reset(qpos=q0, qvel=v0)
    sim = GazeboSim(write_world(tmp_path/'world', config, ground='none'))
    rotation = plant.data.xmat[plant.fork_id].reshape(3, 3)
    q, v = _gazebo_state(sim.reset(q0[:7], q0[7:10], v0[6:9], v0[:3], rotation@v0[3:6]))
    plant.reset(qpos=q, qvel=v)
    rng = np.random.default_rng(3)
    for _ in range(60):
        u = np.r_[rng.uniform(-1, 1, 3)*np.array([.3, .1, .1]), 0., 0.]
        plant.step(u)
        sim.command(u[:3], [0., 0.])
        sim.run(5)
        q, v = _gazebo_state(sim.state())
        dq = np.zeros(9)
        mujoco.mj_differentiatePos(plant.model, dq, 1., plant.data.qpos, q)
        assert np.abs(dq).max() < 1e-6 and np.abs(v-plant.data.qvel).max() < 1e-6


@gazebo
def test_standing_wheel_carries_its_weight_and_never_sleeps(workspace, tmp_path):
    from spherical_wheel.gazebo.plant import GazeboPlant
    from spherical_wheel.terrain import Terrain
    plant = GazeboPlant(workspace.config, Terrain(name='flat'), tmp_path/'mirror', world_dir=tmp_path/'world')
    controller = OffroadController(plant, workspace.design, OffroadParams())
    place_on_ground(plant, controller, (0., 0.), .30)
    weight = plant.total_mass*9.81
    for _ in range(200):   # 1 s: past DART's 0.5 s default sleep time
        applied, diagnostics = controller.command(np.zeros(2), np.zeros(2), np.zeros(2))
        assert abs(diagnostics['measurement']['normal_force_N']/weight-1.) < 2e-3
        plant.step(applied)
        controller.after_step()
    assert abs(plant.data.qvel[2]) < 1e-6


@gazebo
def test_polar_belt_moves_the_wheel_like_mujoco(workspace, tmp_path):
    """Pole down, both drives idle, belt A driven: same direction and similar speed in both simulators."""
    from spherical_wheel.gazebo.plant import GazeboPlant
    from spherical_wheel.offroad import TerrainPlant
    from spherical_wheel.terrain import Terrain
    velocities = {}
    for name, plant in (('mujoco', TerrainPlant(workspace.config, Terrain(name='flat'), tmp_path/'mj')),
                        ('gazebo', GazeboPlant(workspace.config, Terrain(name='flat'), tmp_path/'gzm', world_dir=tmp_path/'gzw'))):
        q = np.r_[0., 0., .12-3.35e-5, 1., 0., 0., 0., np.pi/2, 0., 0.]
        plant.reset(qpos=q, qvel=np.zeros(9))
        for _ in range(20):
            plant.step(np.array([0., 0., 0., .1, 0.]))
        velocities[name] = plant.data.qvel[:2].copy()
    mj, gz = velocities['mujoco'], velocities['gazebo']
    assert np.linalg.norm(mj) > .01
    assert mj@gz/(np.linalg.norm(mj)*np.linalg.norm(gz)) > .99
    assert abs(np.linalg.norm(gz)/np.linalg.norm(mj)-1.) < .1


@gazebo
def test_terrain_mesh_matches_the_analytic_surface(workspace, tmp_path):
    from spherical_wheel.gazebo.plant import GazeboPlant
    for name in ('ramp_forward', 'washboard'):  # also catches Gazebo's mesh cache serving a stale file
        terrain, _ = scenario(name)
        plant = GazeboPlant(workspace.config, terrain, tmp_path/'mirror', world_dir=tmp_path/'world')
        assert plant.ground_check['median_abs_error_m'] < 5e-5
        plant.close()
