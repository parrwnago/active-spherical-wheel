"""The uneven-ground contact model against closed-form mechanics.

A plain solid sphere on the moving tangent plane must reproduce textbook
results before the wheel's behaviour on that ground means anything.
"""
import math

import mujoco
import numpy as np
import pytest

from spherical_wheel.contact_sensing import intrinsic_normal, measure_contact
from spherical_wheel.control import find_equilibrium
from spherical_wheel.model import generate_model, load_config
from spherical_wheel.offroad import TerrainPlant, pose_ground_plane
from spherical_wheel.sim import Plant
from spherical_wheel.terrain import PRESETS, Terrain, make_terrain

RADIUS, MASS, GRAVITY = .12, 1.5, 9.81
_BALL = f"""
<mujoco>
  <option timestep="0.001" gravity="0 0 -{GRAVITY}" integrator="implicitfast" cone="elliptic"/>
  <worldbody>
    <body name="ground" mocap="true">
      <geom name="floor" type="plane" size="5 5 0.1" priority="1" friction="0.7 0.005 0.0001"
            solref="0.005 1" solimp="0.95 0.99 0.001" condim="3"/>
    </body>
    <body name="ball">
      <freejoint/>
      <geom name="ball" type="sphere" size="{RADIUS}" mass="{MASS}" friction="0.7 0.005 0.0001"
            solref="0.005 1" solimp="0.95 0.99 0.001" condim="3"/>
    </body>
  </worldbody>
</mujoco>
"""


class Ball:
    """A uniform solid sphere on a terrain, using the plant's own ground routine."""

    def __init__(self, terrain, xy=(0., 0.)):
        self.terrain = terrain
        self.model = mujoco.MjModel.from_xml_string(_BALL)
        self.data = mujoco.MjData(self.model)
        self.floor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        self.guess = None
        point = np.array([xy[0], xy[1], terrain.height(*xy)])
        self.data.qpos[:3] = point+RADIUS*terrain.normal_at(*xy)
        self.ground = None

    def step(self, count=1):
        for _ in range(count):
            self.ground = pose_ground_plane(self.model, self.data, self.terrain, self.floor, 0,
                                            self.data.qpos[:3], self.guess, .7)
            self.guess = self.ground[0][:2]
            mujoco.mj_step(self.model, self.data)

    def contact_force(self):
        self.ground = pose_ground_plane(self.model, self.data, self.terrain, self.floor, 0,
                                        self.data.qpos[:3], self.guess, .7)
        mujoco.mj_forward(self.model, self.data)
        force = np.zeros(6)
        if self.data.ncon:
            mujoco.mj_contactForce(self.model, self.data, 0, force)
        return force[0], float(np.hypot(force[1], force[2]))


@pytest.mark.parametrize('name', ['ramp', 'bumps', 'rough', 'course'])
@pytest.mark.parametrize('heading', [90., 180., 270., 37.])
def test_course_heading_turns_the_whole_terrain(name, heading):
    plain, turned = make_terrain(name), make_terrain(name, heading_deg=heading)
    angle = np.radians(heading)
    rng = np.random.default_rng(5)
    heights = []
    for _ in range(40):
        u, v = rng.uniform(-.3, 3.8), rng.uniform(-.5, .5)
        x, y = np.cos(angle)*u-np.sin(angle)*v, np.sin(angle)*u+np.cos(angle)*v
        assert turned.height(x, y) == pytest.approx(plain.height(u, v), abs=1e-12)
        assert turned.friction(x, y, .7) == pytest.approx(plain.friction(u, v, .7), abs=1e-12)
        heights.append(plain.height(u, v))
    # The sampled ground is not level, so the comparison above is not vacuous.
    assert np.ptp(heights) > .005


@pytest.mark.parametrize('name', PRESETS)
@pytest.mark.parametrize('heading', [0., 90., 180., 37.])
def test_terrain_gradient_and_hessian_match_finite_differences(name, heading):
    terrain = make_terrain(name, heading_deg=heading)
    rng = np.random.default_rng(3)
    step = 1e-5
    for _ in range(25):
        x, y = rng.uniform(-.4, 3.6), rng.uniform(-.7, .7)
        _, gx, gy, hxx, hxy, hyy = terrain.evaluate(x, y)
        assert gx == pytest.approx((terrain.height(x+step, y)-terrain.height(x-step, y))/(2*step), abs=1e-7)
        assert gy == pytest.approx((terrain.height(x, y+step)-terrain.height(x, y-step))/(2*step), abs=1e-7)
        assert hxx == pytest.approx((terrain.evaluate(x+step, y)[1]-terrain.evaluate(x-step, y)[1])/(2*step), abs=1e-5)
        assert hxy == pytest.approx((terrain.evaluate(x, y+step)[1]-terrain.evaluate(x, y-step)[1])/(2*step), abs=1e-5)
        assert hyy == pytest.approx((terrain.evaluate(x, y+step)[2]-terrain.evaluate(x, y-step)[2])/(2*step), abs=1e-5)


@pytest.mark.parametrize('name', ['hills', 'bumps', 'rough', 'course', 'washboard'])
def test_closest_point_is_the_nearest_surface_point(name):
    terrain = make_terrain(name)
    rng = np.random.default_rng(11)
    for _ in range(12):
        x, y = rng.uniform(.4, 3.), rng.uniform(-.3, .3)
        centre = np.array([x, y, terrain.height(x, y)+RADIUS*rng.uniform(.9, 1.1)])
        point, normal, distance = terrain.closest_point(centre)
        # The centre lies on the surface normal through the returned point.
        np.testing.assert_allclose(centre-point, distance*normal, atol=1e-9)
        assert point[2] == pytest.approx(terrain.height(point[0], point[1]), abs=1e-12)
        offsets = np.linspace(-.06, .06, 61)
        nearest = min(math.dist(centre, (point[0]+dx, point[1]+dy, terrain.height(point[0]+dx, point[1]+dy)))
                      for dx in offsets for dy in offsets)
        assert distance <= nearest+1e-9


@pytest.mark.parametrize('name', PRESETS)
def test_presets_stay_within_the_single_contact_limit(name):
    audit = make_terrain(name).audit((-.5, 4.2, -.6, .6), RADIUS, samples=71)
    assert audit['single_contact_valid'], audit
    assert audit['max_slope_deg'] < 20.


def test_rolling_down_a_plane_matches_the_solid_sphere_result():
    grade = math.radians(10.)
    ball = Ball(Terrain([{'type': 'plane', 'slope': [math.tan(grade), 0.]}]))
    ball.step(100)
    start_time, start = ball.data.time, ball.data.qpos[:3].copy()
    start_speed = np.linalg.norm(ball.data.qvel[:3])
    ball.step(500)
    elapsed = ball.data.time-start_time
    travelled = np.linalg.norm(ball.data.qpos[:3]-start)
    acceleration = 2*(travelled-start_speed*elapsed)/elapsed**2
    assert acceleration == pytest.approx(5/7*GRAVITY*math.sin(grade), rel=.01)
    normal, tangential = ball.contact_force()
    assert normal == pytest.approx(MASS*GRAVITY*math.cos(grade), rel=.01)
    assert tangential == pytest.approx(2/7*MASS*GRAVITY*math.sin(grade), rel=.02)
    # Rolling, not sliding: surface speed at the contact is far below travel speed.
    spin = ball.data.qvel[3:6]
    rotation = ball.data.xmat[1].reshape(3, 3)
    contact_velocity = ball.data.qvel[:3]+np.cross(rotation@spin, -RADIUS*ball.ground[1])
    assert np.linalg.norm(contact_velocity) < .01*np.linalg.norm(ball.data.qvel[:3])


def test_a_slippery_patch_lowers_the_friction_the_solver_uses():
    grade, mu = math.radians(10.), .05
    terrain = Terrain([{'type': 'plane', 'slope': [math.tan(grade), 0.]}],
                      [{'mu': mu, 'x': [-5., 5.], 'edge': .01}])
    ball = Ball(terrain)
    ball.step(100)
    start_time, start = ball.data.time, ball.data.qpos[:3].copy()
    start_speed = np.linalg.norm(ball.data.qvel[:3])
    ball.step(400)
    elapsed = ball.data.time-start_time
    acceleration = 2*(np.linalg.norm(ball.data.qpos[:3]-start)-start_speed*elapsed)/elapsed**2
    # Below (2/7) tan(grade) the sphere must slide: a = g (sin - mu cos).
    assert mu < 2/7*math.tan(grade)
    assert acceleration == pytest.approx(GRAVITY*(math.sin(grade)-mu*math.cos(grade)), rel=.01)
    normal, tangential = ball.contact_force()
    assert tangential == pytest.approx(mu*normal, rel=.01)


def test_rolling_in_a_valley_has_the_period_set_by_its_curvature():
    # h = A (1 - cos k x) has curvature radius 1/(A k^2) at the bottom. A solid
    # sphere rolling there oscillates with T = 2 pi sqrt(7 (rho - R) / (5 g)).
    rho = .5
    k = math.pi
    amplitude = 1./(rho*k*k)
    terrain = Terrain([{'type': 'waves', 'amplitude': [-amplitude], 'kx': [k], 'ky': [0.], 'phase': [math.pi/2]},
                       {'type': 'plane', 'offset': amplitude}])
    assert terrain.height(0., 0.) == pytest.approx(0., abs=1e-12)
    ball = Ball(terrain, xy=(.02, 0.))
    crossings, previous = [], ball.data.qpos[0]
    for _ in range(4000):
        ball.step()
        current = ball.data.qpos[0]
        if previous > 0 >= current:
            crossings.append(ball.data.time-ball.model.opt.timestep*current/(current-previous))
        previous = current
    assert len(crossings) >= 2
    period = float(np.mean(np.diff(crossings)))
    assert period == pytest.approx(2*math.pi*math.sqrt(7*(rho-RADIUS)/(5*GRAVITY)), rel=.01)
    # The ball stayed in the valley and on the ground throughout.
    assert abs(ball.data.qpos[1]) < 1e-6 and ball.ground[2]-RADIUS < 1e-4


@pytest.fixture(scope='module')
def plants(tmp_path_factory):
    directory = tmp_path_factory.mktemp('offroad_ground')
    config = load_config()
    flat = Plant(config, generate_model(config, directory/'flat'))
    return config, flat, directory


def test_level_terrain_reproduces_the_flat_plant_exactly(plants):
    config, flat, directory = plants
    terrain_plant = TerrainPlant(config, make_terrain('flat'), directory/'level')
    equilibrium = find_equilibrium(flat)
    flat.reset(qpos=equilibrium['qpos'])
    terrain_plant.reset(qpos=equilibrium['qpos'])
    tilt = np.zeros(18)
    tilt[3:5] = np.deg2rad([1.5, -1.])
    flat.perturb(tilt)
    terrain_plant.perturb(tilt)
    for index in range(120):
        command = np.array([.05*np.sin(.05*index), .02, -.02, .01, 0.])
        expected, actual = flat.step(command), terrain_plant.step(command)
        np.testing.assert_array_equal(actual['qpos'], expected['qpos'])
        np.testing.assert_array_equal(actual['qvel'], expected['qvel'])


@pytest.mark.parametrize('grade,cross', [(10., 0.), (0., 8.), (12., -6.)])
def test_force_and_moment_alone_recover_the_ground_normal(plants, grade, cross):
    config, _, directory = plants
    plant = TerrainPlant(config, make_terrain('slope', grade_deg=grade, cross_deg=cross), directory/'slope')
    plant.reset(xy=(0., 0.))
    plant.data.qpos[:3] -= .0004*plant.ground['normal']
    plant._forward()
    measurement = measure_contact(plant)
    assert measurement['contacts'] == 1 and measurement['normal_force_N'] > 5.
    truth = plant.ground['normal']
    np.testing.assert_allclose(measurement['normal'], truth, atol=1e-9)
    estimate = intrinsic_normal(measurement['force_N'], measurement['moment_Nm'], plant.radius)
    assert np.degrees(np.arccos(np.clip(estimate@truth, -1., 1.))) < .01
    # No force added: the plant's only external load slot is still empty.
    np.testing.assert_array_equal(plant.data.xfrc_applied, 0.)
    np.testing.assert_array_equal(plant.data.qfrc_applied, 0.)


def test_displayed_height_field_matches_the_analytic_ground(plants):
    config, _, directory = plants
    terrain = make_terrain('course')
    plant = TerrainPlant(config, terrain, directory/'course')
    visual = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, 'terrain_visual')
    assert plant.model.geom_contype[visual] == 0 and plant.model.geom_conaffinity[visual] == 0
    hit = np.zeros(1, dtype=np.int32)
    for x, y in [(.6, .3), (1.5, -.4), (2.3, .1), (2.85, .02), (3.4, -.5)]:
        distance = mujoco.mj_ray(plant.model, plant.data, np.array([x, y, 1.]), np.array([0., 0., -1.]),
                                 None, 1, -1, hit)
        assert int(hit[0]) == visual
        assert 1.-distance == pytest.approx(terrain.height(x, y), abs=5e-4)
