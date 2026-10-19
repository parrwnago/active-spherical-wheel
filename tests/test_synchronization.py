"""Incoming surface preparation uses rigid-body kinematics and real actuators."""
import json

import mujoco
import numpy as np
import pytest

from spherical_wheel.control import find_equilibrium
from spherical_wheel.model import generate_model
from spherical_wheel.sim import Plant
from spherical_wheel.synchronization import ContactSynchronizer


@pytest.fixture(scope='module')
def plant_and_equilibria(tmp_path_factory):
    plant = Plant(model_path=generate_model(output_dir=tmp_path_factory.mktemp('synchronization_model')))
    equilibria = {phase: find_equilibrium(plant, phase) for phase in [.1, .3, 1.2, 1.8, np.pi/2]}
    return plant, equilibria


def configured_plant(fixture, phase, base_velocity=(0., 0., 0.), shaft_velocity=0., angular_velocity=(0., 0., 0.)):
    plant, equilibria = fixture
    eq = equilibria[phase]
    plant.reset(qpos=eq['qpos'], qvel=eq['qvel'])
    # These are explicit initial velocities for a kinematic test, not corrective
    # velocity assignments made by the controller or a claimed recovery run.
    plant.data.qvel[:3] = base_velocity
    plant.data.qvel[3:6] = angular_velocity
    plant.data.qvel[6] = shaft_velocity
    mujoco.mj_forward(plant.model, plant.data)
    return plant


def target(result, name):
    return next(item for item in result['targets'] if item['name'] == name)


def test_preview_preserves_live_integration_state_model_and_controls(plant_and_equilibria):
    plant = configured_plant(plant_and_equilibria, 1.2, (.12, .09, 0.), 1.)
    plant.apply_input([.1, -.2, .3, .04, -.05])
    before_state = plant.capture().copy()
    before_surface = plant.model.geom_surfacevel.copy()
    before_controls = plant.data.ctrl.copy()
    before_command = plant.last_command.copy()
    before_input = plant.last_input.copy()
    before_external = plant.data.xfrc_applied.copy()
    before_generalized = plant.data.qfrc_applied.copy()

    result = ContactSynchronizer(plant).preview([.1, -.2, .3, .04, -.05])

    np.testing.assert_array_equal(plant.capture(), before_state)
    np.testing.assert_array_equal(plant.model.geom_surfacevel, before_surface)
    np.testing.assert_array_equal(plant.data.ctrl, before_controls)
    np.testing.assert_array_equal(plant.last_command, before_command)
    np.testing.assert_array_equal(plant.last_input, before_input)
    np.testing.assert_array_equal(plant.data.xfrc_applied, before_external)
    np.testing.assert_array_equal(plant.data.qfrc_applied, before_generalized)
    json.dumps(result, allow_nan=False)


def test_belt_is_prepared_before_loading_and_loaded_shell_keeps_authority(plant_and_equilibria):
    plant = configured_plant(plant_and_equilibria, 1.2, (.12, .09, 0.), 1.)
    primary = np.array([.1, .2, -.1, 0., 0.])
    result = ContactSynchronizer(plant).preview(primary)
    belt = target(result, 'belt_A')
    shell = target(result, 'shell_A')

    assert not belt['currently_exposed']
    assert belt['predicted_exposed']
    assert belt['normal_load_N'] == 0.
    assert belt['blend_weight'] > .9
    assert belt['target_surface_speed_m_s'] == pytest.approx(-.09, abs=1e-7)
    assert belt['predicted_matched_speed_m_s'] < 1e-6
    assert shell['normal_load_fraction'] > .99
    assert result['blend_weights'][1] == 0.
    weight = np.asarray(result['blend_weights'])
    applied = (1.-weight)*primary + weight*np.asarray(result['commands'])
    assert applied[0] == primary[0]
    assert applied[1] == primary[1]


def test_incoming_shell_matches_point_velocity_with_bounded_torque(plant_and_equilibria):
    plant = configured_plant(plant_and_equilibria, 1.8, (.12, .09, 0.), 1.)
    result = ContactSynchronizer(plant).preview(np.zeros(5))
    shell = target(result, 'shell_A')
    belt = target(result, 'belt_A')

    assert shell['normal_load_N'] == 0.
    assert not shell['currently_exposed']
    assert shell['predicted_exposed']
    assert 0. < shell['blend_weight'] <= .35
    assert shell['target_joint_speed_rad_s'] > 2.
    assert shell['target_joint_speed_rad_s'] != pytest.approx(plant.data.qvel[6])
    assert 0. < shell['target_torque_Nm'] <= plant.bounds[1]
    assert shell['predicted_matched_speed_m_s'] < .002
    assert belt['normal_load_fraction'] > .99
    assert belt['blend_weight'] == 0.


def test_shell_B_prepares_before_equatorial_transfer(plant_and_equilibria):
    plant = configured_plant(plant_and_equilibria, .1, (-.24, .09, 0.), -2.)
    result = ContactSynchronizer(plant).preview(np.zeros(5))
    incoming = target(result, 'shell_B')
    outgoing = target(result, 'shell_A')

    assert incoming['normal_load_N'] == 0.
    assert not incoming['currently_exposed']
    assert incoming['predicted_exposed']
    assert incoming['approaching']
    assert incoming['blend_weight'] > 0.
    assert incoming['target_joint_speed_rad_s'] < 0.
    assert incoming['target_torque_Nm'] < 0.
    assert outgoing['normal_load_N'] > 1.
    assert outgoing['blend_weight'] == 0.


def test_zero_shell_lever_remains_finite_and_cannot_override_loaded_belt(plant_and_equilibria):
    plant = configured_plant(plant_and_equilibria, np.pi/2, (0., .25, 0.))
    plant.data.qvel[7:9] = [40., -40.]
    mujoco.mj_forward(plant.model, plant.data)
    primary = np.array([.4, .3, -.2, -.25, 0.])
    result = ContactSynchronizer(plant).preview(primary)

    for name in ['shell_A', 'shell_B']:
        shell = target(result, name)
        assert shell['predicted_contact_lever_m'] < 1e-8
        assert abs(shell['target_joint_speed_rad_s']) < 1e-6
        assert shell['blend_weight'] == 0.
    assert np.all(np.isfinite(result['commands']))
    assert np.all(np.abs(result['commands']) <= plant.bounds)
    assert target(result, 'belt_A')['blend_weight'] == 0.
    np.testing.assert_array_equal(result['blend_weights'], np.zeros(5))
    applied = (1.-np.asarray(result['blend_weights']))*primary + np.asarray(result['blend_weights'])*result['commands']
    np.testing.assert_array_equal(applied, primary)


@pytest.mark.parametrize('phase,patch_name', [(.3, 'shell_A'), (np.pi/2, 'belt_A')])
def test_actual_contact_velocity_includes_free_base_common_shaft_and_belt_surface(plant_and_equilibria, phase, patch_name):
    plant = configured_plant(plant_and_equilibria, phase, (.12, .09, 0.), 1.2, (.25, -.15, .4))
    plant.data.qvel[7] = -1.1
    plant.apply_input([0., 0., 0., .12, 0.])
    mujoco.mj_forward(plant.model, plant.data)
    result = ContactSynchronizer(plant).preview()
    actual = target(result, patch_name)
    weighted_velocity = np.zeros(3)
    total_force = 0.
    shell_motion_only = np.zeros(3)
    for index, contact in enumerate(plant.data.contact):
        if contact.efc_address < 0:
            continue
        for geom in contact.geom:
            name = mujoco.mj_id2name(plant.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom)) or ''
            if not name.startswith(patch_name):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(plant.model, plant.data, index, force)
            normal_load = max(0., force[0])
            jacobian = np.zeros((3, plant.model.nv))
            mujoco.mj_jac(plant.model, plant.data, jacobian, None, contact.pos, int(plant.model.geom_bodyid[geom]))
            velocity = jacobian @ plant.data.qvel
            # Independently compose rigid-body angular velocities about the
            # shared wheel center, including the free base and shaft drive.
            fork_rotation = plant.data.xmat[plant.fork_id].reshape(3, 3)
            shaft_joint = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, 'shaft')
            omega = fork_rotation @ plant.data.qvel[3:6] + plant.data.xaxis[shaft_joint]*plant.data.qvel[6]
            if patch_name == 'shell_A':
                shell_joint = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, 'shell_A_joint')
                omega += plant.data.xaxis[shell_joint]*plant.data.qvel[7]
            independently_composed = plant.data.qvel[:3] + np.cross(omega, contact.pos-plant.data.xpos[plant.fork_id])
            np.testing.assert_allclose(velocity, independently_composed, atol=1e-10)
            velocity += plant.data.geom_xmat[geom].reshape(3, 3) @ plant.model.geom_surfacevel[geom, :3]
            velocity[2] = 0.  # The declared floor is horizontal.
            weighted_velocity += normal_load*velocity
            shell_motion_only += normal_load*(jacobian[:, 7]*plant.data.qvel[7])
            total_force += normal_load
    assert total_force > 1.
    expected = weighted_velocity/total_force
    np.testing.assert_allclose(actual['actual_tangential_velocity_m_s'], expected, atol=1e-10)
    assert actual['actual_velocity_point'] == 'force-weighted actual contacts'
    assert np.linalg.norm(expected-shell_motion_only/total_force) > .03


def test_target_commands_respect_mixed_torque_and_speed_bounds(plant_and_equilibria):
    plant = configured_plant(plant_and_equilibria, 1.8, (5., -5., 0.), 2.)
    plant.data.qvel[7:9] = [-100., 100.]
    mujoco.mj_forward(plant.model, plant.data)
    synchronizer = ContactSynchronizer(plant, shell_speed_limit_rad_s=12.)
    result = synchronizer.preview([1e4, -1e4, 1e4, -1e4, 1e4])

    assert np.all(np.isfinite(result['commands']))
    assert np.all(np.abs(result['commands']) <= plant.bounds)
    assert np.all(np.asarray(result['blend_weights']) >= 0.)
    assert np.all(np.asarray(result['blend_weights']) <= 1.)
    for item in result['targets']:
        if 'target_joint_speed_rad_s' in item:
            assert abs(item['target_joint_speed_rad_s']) <= 12.
        if 'target_surface_speed_m_s' in item:
            assert abs(item['target_surface_speed_m_s']) <= plant.bounds[item['command_index']]
