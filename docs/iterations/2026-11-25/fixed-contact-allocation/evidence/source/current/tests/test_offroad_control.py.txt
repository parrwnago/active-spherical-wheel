"""The contact-aware controller on slopes and uneven ground, against the flat baseline.

Every comparison starts both controllers from the same state on the same
plant, with no load other than gravity, the five drives and solved contact.
"""
import mujoco
import numpy as np
import pytest

from spherical_wheel.contact_sensing import ContactEstimator, intrinsic_normal, measure_contact
from spherical_wheel.model import load_config
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import ReferencePath, Workspace, place_on_ground, run_offroad, scenario
from spherical_wheel.offroad_geometry import support_leverage
from spherical_wheel.offroad_suite import Pool, run_cost, scenario_job, slope_limits
from spherical_wheel.terrain import make_terrain


@pytest.fixture(scope='module')
def workspace(tmp_path_factory):
    return Workspace(directory=tmp_path_factory.mktemp('offroad_control'))


def _hold(workspace, terrain, mode, phase=.30, seconds=5.):
    path = ReferencePath(0., 0., seconds-.5, start_s=.5, profile='quintic')
    log, summary = run_offroad(workspace, terrain, path, OffroadParams(mode=mode), duration_s=seconds,
                               phase=phase, settle_s=0.)
    plant = workspace.plant(terrain)
    # The only external load slot is the logged push, and none was requested.
    np.testing.assert_array_equal(plant.data.xfrc_applied, 0.)
    np.testing.assert_array_equal(plant.data.qfrc_applied, 0.)
    assert plant.model.neq == 0
    return log, summary


def test_force_and_moment_normal_inverts_a_point_contact_on_a_sphere():
    rng = np.random.default_rng(5)
    radius = .12
    for _ in range(200):
        normal = rng.normal(size=3)
        normal[2] = abs(normal[2])+.5
        normal /= np.linalg.norm(normal)
        # Any force inside a wide friction cone about the normal.
        tangent = np.cross(normal, rng.normal(size=3))
        force = rng.uniform(1., 40.)*(normal+rng.uniform(0., .9)*tangent/np.linalg.norm(tangent))
        moment = np.cross(-radius*normal, force)
        np.testing.assert_allclose(intrinsic_normal(force, moment, radius), normal, atol=1e-12)
    assert intrinsic_normal(np.zeros(3), np.zeros(3), radius) is None


def test_friction_estimate_needs_sustained_sliding():
    estimator = ContactEstimator(.12, 18., .7, slip_confirm_s=.03, friction_recovery_s=1.)
    dt = .005

    def measurement(slip, ratio):
        return {'normal_force_N': 18., 'tangential_force_norm_N': 18.*ratio, 'slip_speed_m_s': slip,
                'force_N': np.array([0., 18.*ratio, 18.]), 'moment_Nm': np.zeros(3), 'normal': np.array([0., 0., 1.])}

    # One sliding sample at a force well inside the cone is a handover artefact.
    estimator.update(measurement(.2, .2), dt)
    assert estimator.friction == pytest.approx(.7)
    estimator.update(measurement(0., .2), dt)
    for _ in range(4):
        estimator.update(measurement(0., .2), dt)
    assert estimator.friction == pytest.approx(.7)
    # Sustained sliding at a force ratio of 0.25 is a measurement of friction.
    for _ in range(10):
        state = estimator.update(measurement(.2, .25), dt)
    assert state['sliding'] and estimator.friction == pytest.approx(.25)
    # It relaxes back once the surface grips again.
    for _ in range(int(5./dt)):
        estimator.update(measurement(0., .1), dt)
    assert estimator.friction == pytest.approx(.7, abs=.01)


def test_rim_support_slows_the_sensed_normal_and_holds_its_axial_part_at_rest():
    radius, dt = .12, .005
    axis = np.array([1., 0., 0.])
    force = np.array([0., 0., 18.])

    def estimate(tilt_along, tilt_across, seconds, **rim):
        apparent = np.array([np.sin(tilt_along), np.sin(tilt_across), 0.])
        apparent[2] = np.sqrt(1.-apparent@apparent)
        measurement = {'normal_force_N': 18., 'tangential_force_norm_N': 0., 'slip_speed_m_s': 0.,
                       'force_N': force, 'moment_Nm': np.cross(-radius*apparent, force), 'normal': apparent}
        estimator = ContactEstimator(radius, 18., .7, normal_time_constant_s=.04, rim_time_constant_s=.4)
        for _ in range(round(seconds/dt)):
            state = estimator.update(measurement, dt, **rim)
        return np.arcsin(state['normal'][:2])

    # On the sphere the estimate has all but arrived in a tenth of a second.
    np.testing.assert_allclose(estimate(.08, .05, .1), [.08, .05], rtol=.1)
    # A rim crossing of the same length moves it only a little...
    along, across = estimate(.08, .05, .1, rim_axis=axis)
    assert .1*.08 < along < .3*.08 and .1*.05 < across < .3*.05
    # ...but a wheel that stays on a rim is still followed.
    np.testing.assert_allclose(estimate(.08, .05, 3., rim_axis=axis), [.08, .05], rtol=.02)
    # Resting on both rims, an apparent tilt along the carrier axis is the
    # load shifting between rims, not a slope. It is never followed, while a
    # tilt across the axis still is.
    along, across = estimate(.08, .05, 3., rim_axis=axis, hold_axis=True)
    assert abs(along) < 1e-9 and across == pytest.approx(.05, rel=.02)


def test_static_limits_follow_from_the_declared_model():
    config = load_config()
    limits = slope_limits(config)
    weight, radius = limits['weight_N'], config['geometry']['sphere_radius_m']
    assert limits['lean_per_unit_slope'] == pytest.approx(1.86*.12/(1.2*.19109), rel=1e-3)
    assert np.sin(np.radians(limits['shaft_torque_limit_deg']))*weight*radius == pytest.approx(1.5)
    assert np.tan(np.radians(limits['friction_limit_deg'])) == pytest.approx(.7)
    assert limits['belt_cap_side_slope_limit_deg'] == pytest.approx(
        config['geometry']['polar_cap_angle_deg']/(1+limits['lean_per_unit_slope']))
    # The shell bound, not friction or the shaft, limits a side slope.
    assert limits['shell_torque_limit_deg_at_initial_phase'] < limits['friction_limit_deg']


def test_leverage_map_shows_the_belt_covering_only_a_small_cap():
    config = load_config()
    cap = config['geometry']['polar_cap_angle_deg']
    leverage, patch = support_leverage(config, [17.2, 90., 90., 90., 0.], [0., 0., cap-1., cap+4., 0.])
    assert patch[0, 0] == 1 and leverage[0, 0] == pytest.approx(np.cos(np.radians(17.2)), abs=1e-6)
    assert patch[1, 1] == 2 and leverage[1, 1] == pytest.approx(1.)
    assert patch[2, 2] == 2 and leverage[2, 2] > .95
    # Just outside the cap with a pole down, a shell is loaded but spinning it
    # moves the contact the same way the shaft does: nothing drives sideways.
    assert patch[3, 3] == 1 and leverage[3, 3] < 1e-6
    assert patch[4, 4] == 0


@pytest.mark.parametrize('grade,cross,phase', [(10., 0., .30), (0., 8., .30), (8., 5., np.pi/2)])
def test_feedforward_equals_the_torque_that_carries_the_weight(workspace, grade, cross, phase):
    terrain = make_terrain('slope', grade_deg=grade, cross_deg=cross)
    plant = workspace.plant(terrain)
    controller = OffroadController(plant, workspace.design, OffroadParams(), phase)
    place_on_ground(plant, controller, (0., 0.), phase)
    measurement = measure_contact(plant)
    assert measurement['contacts'] == 1
    analytic = controller.support_torque(plant.ground['normal'], measurement['loads_N'])
    # Independent route: joint torque that balances the full weight applied
    # vertically at the solver's contact point, through MuJoCo's own Jacobian.
    name = next(key for key, value in measurement['loads_N'].items() if value > 0)
    geom = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, name+'_smooth')
    jacobian = np.zeros((3, plant.model.nv))
    mujoco.mj_jac(plant.model, plant.data, jacobian, None, measurement['point_m'], int(plant.model.geom_bodyid[geom]))
    expected = -jacobian[:, 6:9].T@np.array([0., 0., controller.weight])
    np.testing.assert_allclose(analytic[:3], expected, atol=2e-3)
    assert abs(analytic[0]) > .2 or abs(analytic[1]) > .2


def test_a_grade_is_held_by_leaning_and_feeding_the_holding_torque_forward(workspace):
    grade = 10.
    terrain = make_terrain('slope', grade_deg=grade)
    log, summary = _hold(workspace, terrain, 'terrain_aware')
    assert summary['completed'] and summary['settled_at_end']
    assert summary['tracking']['final_error_m'] < .005
    assert summary['attitude']['max_error_from_reference_deg'] < .5
    limits = slope_limits()
    tail = log['time_s'] > 3.
    lean = np.degrees(log['tilt_rad'][tail]).mean()
    assert lean == pytest.approx(np.degrees(np.arcsin(limits['lean_per_unit_slope']*np.sin(np.radians(grade)))), abs=.3)
    holding = limits['weight_N']*.12*np.sin(np.radians(grade))
    assert log['input'][tail, 0].mean() == pytest.approx(holding, rel=.04)
    # Sensed from force and moment only, the slope matches the ground.
    assert summary['contact']['normal_estimate_error_rms_deg'] < .1
    # Friction demand on a slope at rest is tan(grade) / mu.
    assert summary['contact']['utilization_p95'] == pytest.approx(np.tan(np.radians(grade))/.7, abs=.03)
    baseline_log, baseline = _hold(workspace, terrain, 'flat_baseline')
    np.testing.assert_array_equal(baseline_log['qpos'][0], log['qpos'][0])
    assert baseline['tracking']['final_error_m'] > .25


def test_a_side_slope_is_held_on_a_shell(workspace):
    terrain = make_terrain('slope', grade_deg=0., cross_deg=8.)
    log, summary = _hold(workspace, terrain, 'terrain_aware')
    assert summary['completed'] and summary['settled_at_end']
    assert summary['tracking']['final_error_m'] < .005
    tail = log['time_s'] > 3.
    expected = slope_limits()['weight_N']*.12*np.cos(.30)*np.sin(np.radians(8.))
    assert abs(log['input'][tail, 1].mean()) == pytest.approx(expected, rel=.06)
    assert np.all(log['support_loads_N'][tail, 0] > 15.)


def test_a_gentle_side_slope_is_held_with_a_pole_down(workspace):
    limits = slope_limits()
    assert 4. < limits['belt_cap_side_slope_limit_deg']
    _, summary = _hold(workspace, make_terrain('slope', grade_deg=0., cross_deg=4.), 'terrain_aware', phase=np.pi/2)
    assert summary['completed'] and summary['settled_at_end']
    assert summary['tracking']['final_error_m'] < .005
    assert summary['contact']['support_time_share']['belt_A'] > .95


def test_level_ground_is_not_made_worse(workspace):
    terrain = make_terrain('flat')
    # 0.45 m rolls across the equatorial gap, where the rims mimic a slope.
    path = ReferencePath(.45, 0., 6.)
    results = {mode: run_offroad(workspace, terrain, path, OffroadParams(mode=mode))[1]
               for mode in ('flat_baseline', 'terrain_aware')}
    aware, baseline = results['terrain_aware'], results['flat_baseline']
    assert aware['completed'] and baseline['completed']
    assert aware['contact']['support_handovers'] >= 1
    assert aware['tracking']['max_error_m'] < 1.1*baseline['tracking']['max_error_m']
    assert aware['attitude']['max_error_from_reference_deg'] < 1.1*baseline['attitude']['max_error_from_reference_deg']+.05
    assert aware['tracking']['final_error_m'] < .003


def test_worker_processes_reproduce_the_single_process_result(tmp_path):
    jobs = [scenario_job(name, {'mode': mode}, distance=.15, duration=3.5, settle_s=.5)
            for name in ('ramp_forward', 'rough') for mode in ('terrain_aware', 'flat_baseline')]
    with Pool(1, tmp_path) as pool:
        serial = pool.run(jobs)
    with Pool(2, tmp_path) as pool:
        parallel = pool.run(jobs)
    for one, other in zip(serial, parallel):
        assert one['controller'] == other['controller'] and one['terrain'] == other['terrain']
        for group in ('tracking', 'attitude', 'contact', 'actuation'):
            assert one[group] == other[group]


def test_climbing_a_ramp_tracks_where_the_flat_controller_sags(workspace):
    terrain, path = scenario('ramp_forward')
    results = {mode: run_offroad(workspace, terrain, path, OffroadParams(mode=mode))[1]
               for mode in ('flat_baseline', 'terrain_aware')}
    aware, baseline = results['terrain_aware'], results['flat_baseline']
    assert aware['completed'] and aware['settled_at_end']
    assert aware['ground']['max_slope_under_wheel_deg'] > 7.5
    assert aware['tracking']['max_error_m'] < .06
    assert aware['tracking']['final_error_m'] < .005
    assert aware['distance_along_path_m'] == pytest.approx(1., abs=.005)
    assert baseline['tracking']['final_error_m'] > .4
    assert run_cost(aware) < .2*run_cost(baseline)
