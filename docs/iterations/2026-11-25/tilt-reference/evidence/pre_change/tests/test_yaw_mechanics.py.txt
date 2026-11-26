"""Physical yaw mechanisms in one freely supported spherical wheel assembly."""
import numpy as np
import mujoco
import pytest
from scipy.optimize import brentq

from spherical_wheel.model import generate_model
from spherical_wheel.sim import Plant


BODY_NAMES = ('fork', 'carrier', 'shell_A', 'shell_B')


def angular_momentum(plant):
    """Independent world-frame body sum, about the complete system COM."""
    model, data = plant.model, plant.data
    bodies = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in BODY_NAMES]
    masses = model.body_mass[bodies]
    com = np.sum(masses[:, None]*data.xipos[bodies], axis=0)/sum(masses)
    velocities, omegas, inertias = {}, {}, {}
    for name, body in zip(BODY_NAMES, bodies):
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros_like(jacp)
        mujoco.mj_jacBodyCom(model, data, jacp, jacr, body)
        velocities[name], omegas[name] = jacp@data.qvel, jacr@data.qvel
        rotation = data.ximat[body].reshape(3, 3)
        inertias[name] = rotation@np.diag(model.body_inertia[body])@rotation.T
    vcom = sum((model.body_mass[body]*velocities[name] for name, body in zip(BODY_NAMES, bodies)), start=np.zeros(3))/sum(masses)
    by_body = {name: inertias[name]@omegas[name] + model.body_mass[body]*np.cross(data.xipos[body]-com, velocities[name]-vcom)
               for name, body in zip(BODY_NAMES, bodies)}
    total = sum(by_body.values(), start=np.zeros(3))
    mujoco.mj_subtreeVel(model, data)
    np.testing.assert_allclose(total, data.subtree_angmom[0], atol=1e-12)
    return total, by_body, omegas


def yaw_deg(plant):
    w, x, y, z = plant.data.qpos[3:7]
    return float(np.rad2deg(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))))


def internal_pulse(model_path, opposing=False):
    plant = Plant(model_path=model_path)
    # Only this private test plant changes. Default body inertias, motor torque
    # limits and internal joint damping are retained. No heading constraint.
    plant.model.opt.gravity[:] = 0.
    plant.model.geom_contype[:] = 0
    plant.model.geom_conaffinity[:] = 0
    plant.reset(shaft_phase=np.pi/2)
    initial, _, _ = angular_momentum(plant)
    command = np.array([0., .1, -.1 if opposing else 0., 0., 0.])
    maximum_error = 0.
    for _ in range(round(.1/plant.dt)):
        plant.step(command)
        total, by_body, omegas = angular_momentum(plant)
        maximum_error = max(maximum_error, float(np.linalg.norm(total-initial)))
        np.testing.assert_array_equal(plant.data.xfrc_applied, 0.)
        np.testing.assert_array_equal(plant.data.qfrc_applied, 0.)
        assert plant.data.ncon == 0
    base = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, 'base')
    return {
        'duration_s': float(plant.data.time), 'torques_Nm': command[:3].tolist(),
        'fork_yaw_deg': yaw_deg(plant),
        'fork_angular_velocity_world_rad_s': omegas['fork'].tolist(),
        'body_angular_momentum_world_kg_m2_s': {name: value.tolist() for name, value in by_body.items()},
        'total_angular_momentum_world_kg_m2_s': total.tolist(),
        'maximum_total_angular_momentum_error_kg_m2_s': maximum_error,
        'contacts': int(plant.data.ncon), 'equality_constraints': int(plant.model.neq),
        'free_root_joint': bool(plant.model.jnt_type[base] == mujoco.mjtJoint.mjJNT_FREE),
        'gravity_m_s2': plant.model.opt.gravity.tolist(),
        'joint_damping_Nms': plant.model.dof_damping[6:9].tolist(),
        'external_force_norm': float(np.linalg.norm(plant.data.xfrc_applied)),
        'external_generalized_force_norm': float(np.linalg.norm(plant.data.qfrc_applied)),
    }


def ground_contacts(plant):
    floor = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    net_force, net_moment, contacts = np.zeros(3), np.zeros(3), []
    for number, contact in enumerate(plant.data.contact):
        if contact.efc_address < 0 or floor not in contact.geom:
            continue
        wrench = np.zeros(6)
        mujoco.mj_contactForce(plant.model, plant.data, number, wrench)
        force = (1 if contact.geom[0] == floor else -1)*contact.frame.reshape(3, 3).T@wrench[:3]
        lever = contact.pos-plant.data.xpos[plant.fork_id]
        moment = np.cross(lever, force)
        geom = int(contact.geom[1] if contact.geom[0] == floor else contact.geom[0])
        net_force += force
        net_moment += moment
        contacts.append({'geom': mujoco.mj_id2name(plant.model, mujoco.mjtObj.mjOBJ_GEOM, geom),
                         'position_relative_to_wheel_m': lever.tolist(),
                         'force_world_N': force.tolist(), 'yaw_moment_Nm': float(moment[2]),
                         'contact_dimension': int(contact.dim),
                         'direct_contact_torque': wrench[3:].tolist()})
    return {'net_force_world_N': net_force.tolist(), 'moment_about_wheel_world_Nm': net_moment.tolist(), 'contacts': contacts}


def dual_rim_pulse(model_path):
    plant = Plant(model_path=model_path)
    radius = plant.config['geometry']['sphere_radius_m']
    gap = plant.config['geometry']['equatorial_gap_half_m']
    rim_radius = np.sqrt(radius**2-gap**2)
    weight = float(np.sum(plant.model.body_mass)*abs(plant.model.opt.gravity[2]))

    def assign(height):
        plant.reset(shaft_phase=0.)
        plant.data.qpos[2] = height
        mujoco.mj_forward(plant.model, plant.data)

    def vertical_residual(height):
        assign(height)
        return ground_contacts(plant)['net_force_world_N'][2]-weight

    # Set only the initial height to the symmetric near-static rim support.
    # No position, velocity or heading is imposed during the motor pulse.
    height = brentq(vertical_residual, rim_radius-.002, rim_radius+.001, xtol=1e-13)
    assign(height)
    initial_contacts = ground_contacts(plant)
    initial_acceleration = float(np.linalg.norm(plant.data.qacc))
    command = np.array([0., .1, -.1, 0., 0.])
    for _ in range(round(.1/plant.dt)):
        plant.step(command)
        np.testing.assert_array_equal(plant.data.xfrc_applied, 0.)
        np.testing.assert_array_equal(plant.data.qfrc_applied, 0.)
    return {'duration_s': float(plant.data.time), 'torques_Nm': command[:3].tolist(),
            'initial_height_m': height, 'initial_acceleration_norm': initial_acceleration,
            'initial_ground_contacts': initial_contacts, 'final_ground_contacts': ground_contacts(plant),
            'fork_yaw_deg': yaw_deg(plant), 'equality_constraints': int(plant.model.neq),
            'external_disturbance_norm': float(np.linalg.norm(plant.data.xfrc_applied)),
            'no_slip_small_angle_yaw_gain': float(rim_radius/(2*gap))}


@pytest.fixture(scope='module')
def model_path(tmp_path_factory):
    return generate_model(output_dir=tmp_path_factory.mktemp('yaw_mechanics_model'))


def test_polar_internal_motor_yaws_free_platform_and_conserves_total_momentum(model_path):
    result = internal_pulse(model_path)
    assert result['free_root_joint'] and result['equality_constraints'] == 0
    assert result['contacts'] == 0
    assert result['maximum_total_angular_momentum_error_kg_m2_s'] < 1e-10
    assert 2. < result['fork_yaw_deg'] < 4.
    assert result['fork_angular_velocity_world_rad_s'][2] > .8
    angular = result['body_angular_momentum_world_kg_m2_s']
    assert -.0101 < angular['shell_A'][2] < -.009
    assert .009 < angular['fork'][2]+angular['carrier'][2] < .0101
    np.testing.assert_allclose(np.sum(list(angular.values()), axis=0), 0., atol=1e-10)


def test_opposite_polar_shell_torques_cancel_platform_reaction(model_path):
    result = internal_pulse(model_path, opposing=True)
    angular = result['body_angular_momentum_world_kg_m2_s']
    assert abs(result['fork_yaw_deg']) < 1e-8
    assert np.linalg.norm(result['fork_angular_velocity_world_rad_s']) < 1e-8
    assert abs(angular['shell_A'][2]) > .009
    np.testing.assert_allclose(angular['shell_A'], -np.asarray(angular['shell_B']), atol=1e-10)
    assert result['maximum_total_angular_momentum_error_kg_m2_s'] < 1e-10


def test_dual_equatorial_rims_generate_yaw_couple_without_net_horizontal_force(model_path):
    result = dual_rim_pulse(model_path)
    assert result['initial_acceleration_norm'] < 1e-5
    initial = result['initial_ground_contacts']['contacts']
    assert any(item['geom'].startswith('shell_A_sector_') for item in initial)
    assert any(item['geom'].startswith('shell_B_sector_') for item in initial)
    assert all(item['contact_dimension'] == 3 for item in initial)
    assert result['equality_constraints'] == 0 and result['external_disturbance_norm'] == 0.
    final = result['final_ground_contacts']
    assert np.linalg.norm(final['net_force_world_N'][:2]) < 1e-5
    assert final['moment_about_wheel_world_Nm'][2] < -.005
    assert result['fork_yaw_deg'] < -.1
    for item in final['contacts']:
        np.testing.assert_array_equal(item['direct_contact_torque'], 0.)
    assert result['no_slip_small_angle_yaw_gain'] == pytest.approx(5.97913037155, rel=1e-8)
