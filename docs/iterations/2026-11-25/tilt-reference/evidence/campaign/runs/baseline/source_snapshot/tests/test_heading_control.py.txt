"""Heading options of the contact-aware controller: fall-line reference, shell steering, LQR weights.

These check the arithmetic and the declared behaviour of each option on the
MuJoCo plant. The evidence that the fall-line steering helps on rough ground
is the Gazebo Mars-yard study in results/gazebo/mars_yaw_study.
"""
import mujoco
import numpy as np
import pytest

from spherical_wheel.control import design_lqr, find_equilibrium
from spherical_wheel.offroad_control import LQR_WEIGHTS, OffroadController, OffroadParams, design_weighted_lqr
from spherical_wheel.offroad_experiments import Workspace, place_on_ground
from spherical_wheel.terrain import make_terrain

STEERING = dict(mode='terrain_aware', heading_reference='fall_line', heading_actuation='shells',
                heading_gain=.6, heading_damping=.1, heading_torque_limit_Nm=.3, heading_roll_budget_Nm=.05,
                fall_line_slope_deg=(3., 7.))


@pytest.fixture(scope='module')
def workspace(tmp_path_factory):
    return Workspace(directory=tmp_path_factory.mktemp('heading_control'))


def _controller(workspace, params, phase=.30, yaw_deg=0.):
    plant = workspace.plant(make_terrain('flat'))
    controller = OffroadController(plant, workspace.design, params, phase)
    place_on_ground(plant, controller, (0., 0.), phase, yaw_deg=yaw_deg)
    return plant, controller


def test_weighted_design_without_overrides_is_the_library_design(workspace):
    designer = workspace.design.designer
    for phase in (.30, np.pi/2):
        equilibrium = find_equilibrium(designer, phase)
        np.testing.assert_array_equal(design_weighted_lqr(designer, equilibrium)['K'],
                                      design_lqr(designer, equilibrium)['K'])


def test_yaw_aware_weights_take_roll_work_off_the_unloaded_shell(workspace):
    """At the ordinary phase shell A carries the wheel and shell B is free."""
    designer = workspace.design.designer
    equilibrium = find_equilibrium(designer, .30)
    flat = design_lqr(designer, equilibrium)['K']
    yaw = design_weighted_lqr(designer, equilibrium, LQR_WEIGHTS['yaw_aware'])
    assert yaw['full_closed_spectral_radius'] <= 1.0001
    roll, own_speed, yaw_rate = 3, 17, 14
    assert abs(yaw['K'][2, roll]) < .5*abs(flat[2, roll])
    assert abs(yaw['K'][2, own_speed]) > 3*abs(flat[2, own_speed])
    assert abs(yaw['K'][2, yaw_rate]) > 2*abs(flat[2, yaw_rate])


def test_heading_options_are_validated():
    for bad in ({'heading_reference': 'north'}, {'heading_actuation': 'belts'}, {'lqr_weights': 'other'},
                {'fall_line_slope_deg': (6., 2.)}, {'heading_roll_budget_Nm': -.1},
                {'heading_reference': 'fall_line', 'heading_actuation': 'shells', 'heading_gain': 0.}):
        with pytest.raises(ValueError):
            OffroadParams(**bad)
    OffroadParams(heading_reference='fall_line', heading_actuation='lqr')   # the LQR needs no shell gain


def test_fall_line_target_takes_the_nearer_end_and_fades_out_on_level_ground(workspace):
    _, controller = _controller(workspace, OffroadParams(**STEERING, heading_normal_time_constant_s=1e-9))

    def error(slope_deg, downhill_deg, yaw_deg, fresh=True):
        if fresh:
            controller._fall_end = None    # no end chosen yet
        s, d = np.radians(slope_deg), np.radians(downhill_deg)
        normal = np.array([np.sin(s)*np.cos(d), np.sin(s)*np.sin(d), np.cos(s)])
        return np.degrees(controller._update_heading_target(normal, np.radians(yaw_deg)))

    limit = np.degrees(controller.params.heading_error_limit_rad)
    assert error(10., 0., 20.) == pytest.approx(20., abs=1e-6)        # turned 20 deg off the downhill direction
    assert error(10., 0., 170.) == pytest.approx(-10., abs=1e-6)      # nearer to the uphill end: hold that one
    assert error(10., 90., 30.) == pytest.approx(-limit, abs=1e-9)    # 60 deg off: bounded by heading_error_limit_rad
    assert error(0., 0., 40.) == 0.                                    # level: no target, only yaw damping
    assert error(5., 0., 10.) == pytest.approx(5., abs=1e-4)           # half way through the 3-7 deg fade
    # Hysteresis: the uphill end is kept until the downhill one is 20 deg nearer.
    controller._fall_end = np.pi
    assert error(10., 0., 80., fresh=False) == pytest.approx(-limit, abs=1e-9)   # 100 from uphill, 80 from downhill
    assert controller._fall_end == np.pi
    assert error(10., 0., 60., fresh=False) == pytest.approx(limit, abs=1e-9)    # 120 against 60: switch
    assert controller._fall_end == 0.


def test_without_a_reference_the_heading_hold_is_unchanged(workspace):
    """heading_gain alone holds the start heading with the free shell, as before these options."""
    params = OffroadParams(mode='terrain_aware', heading_gain=.2)
    plant, controller = _controller(workspace, params)
    controller._held = {'loads': {'shell_A': 18., 'shell_B': 0., 'belt_A': 0., 'belt_B': 0.}}
    torque, error = controller._heading_torque(controller.heading_reference+.3)
    vertical = plant.data.xmat[controller.carrier_id].reshape(3, 3)[2, 0]
    velocity = np.empty(6)
    mujoco.mj_objectVelocity(plant.model, plant.data, mujoco.mjtObj.mjOBJ_BODY, plant.fork_id, velocity, 0)
    expected = np.clip((.2*.3+params.heading_damping*velocity[2])/vertical, -.15, .15)
    assert error == pytest.approx(.3)
    np.testing.assert_allclose(torque, [0., 0., expected, 0., 0.], atol=1e-15)


def test_steering_is_pure_yaw_at_a_pole_and_bounded_in_roll_on_a_shell(workspace):
    params = OffroadParams(**STEERING)
    # On shell A: only shell B is free, and its horizontal reaction stays inside the roll budget.
    plant, controller = _controller(workspace, params, phase=.30)
    controller._held = {'loads': {'shell_A': 18., 'shell_B': 0., 'belt_A': 0., 'belt_B': 0.}}
    controller._yaw_error = .5
    torque, _ = controller._heading_torque(0.)
    axis = plant.data.xmat[controller.carrier_id].reshape(3, 3)[:, 0]
    assert torque[0] == torque[1] == 0. and torque[2] != 0.
    assert abs(torque[2])*np.hypot(axis[0], axis[1]) == pytest.approx(params.heading_roll_budget_Nm)
    # A pole down: both shells are free with near-vertical axes and share the full torque limit.
    plant, controller = _controller(workspace, params, phase=np.pi/2)
    controller._held = {'loads': {'shell_A': 0., 'shell_B': 0., 'belt_A': 18., 'belt_B': 0.}}
    controller._yaw_error = .5
    torque, _ = controller._heading_torque(0.)
    axis = plant.data.xmat[controller.carrier_id].reshape(3, 3)[:, 0]
    assert abs(axis[2]) > .99
    assert torque[1] == torque[2]
    assert abs(torque[1]+torque[2]) == pytest.approx(params.heading_torque_limit_Nm)
    # The platform is turned back: reaction -t*axis has the opposite sign to the yaw error.
    assert -(torque[1]+torque[2])*axis[2] < 0.
