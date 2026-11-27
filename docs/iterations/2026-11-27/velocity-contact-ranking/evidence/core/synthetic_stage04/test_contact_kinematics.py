"""Contact consistency uses measured motion, not a desired-speed tracking error."""
from types import SimpleNamespace

import mujoco
import numpy as np

from spherical_wheel.contact_kinematics import rolling_consistency


def candidate_plant():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="fork" pos="0 0 .1"><freejoint/>
        <geom type="sphere" size=".02" mass="1" contype="0" conaffinity="0"/>
        <body name="shell"><joint type="hinge" axis="0 1 0"/>
          <geom name="shell_A_smooth" type="sphere" size=".1" mass="1" contype="0" conaffinity="0"/>
          <geom name="belt_A_smooth" type="sphere" size=".09" mass=".1" contype="0" conaffinity="0"/>
        </body>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return SimpleNamespace(model=model, data=data)


def record(plant, pos=(0., 0., 0.), belt=False):
    geom = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM,
                            'belt_A_smooth' if belt else 'shell_A_smooth')
    return {'name': 'belt_A_smooth' if belt else 'shell_A_smooth',
            'geom': geom, 'body': int(plant.model.geom_bodyid[geom]),
            'pos': np.array(pos), 'normal': np.array([0., 0., 1.]), 'normal_force': 10.}


def test_pure_rolling_predicts_measured_translation_from_joint_motion():
    plant = candidate_plant()
    plant.data.qvel[0], plant.data.qvel[6] = .1, 1.
    mujoco.mj_forward(plant.model, plant.data)
    result = rolling_consistency(plant, [record(plant)])
    assert result['velocity_consistency_valid']
    np.testing.assert_allclose(result['predicted_body_velocity_m_s'], [.1, 0., 0.], atol=1e-12)
    assert result['contact_velocity_residual_rms_m_s'] < 1e-12
    # An independently measured translation inconsistent with these same
    # joint rates must be visible; no desired velocity enters the function.
    plant.data.qvel[0] = .06
    result = rolling_consistency(plant, [record(plant)])
    assert np.isclose(result['contact_velocity_residual_rms_m_s'], .04)
    assert result['kinematic_inconsistency']


def test_belt_uses_current_applied_surface_motion():
    plant = candidate_plant()
    item = record(plant, belt=True)
    plant.model.geom_surfacevel[item['geom'], 1] = .2
    plant.data.qvel[1] = -.2
    mujoco.mj_forward(plant.model, plant.data)
    result = rolling_consistency(plant, [item])
    np.testing.assert_allclose(result['predicted_body_velocity_m_s'], [0., -.2, 0.], atol=1e-12)
    assert result['contact_velocity_residual_rms_m_s'] < 1e-12


def test_opposing_contact_residuals_do_not_cancel_the_rms():
    plant = candidate_plant()
    plant.data.qvel[6] = 1.
    mujoco.mj_forward(plant.model, plant.data)
    result = rolling_consistency(plant, [record(plant), record(plant, pos=(0., 0., .2))])
    np.testing.assert_allclose(result['contact_velocity_residual_m_s'], 0., atol=1e-12)
    assert np.isclose(result['contact_velocity_residual_rms_m_s'], .1)
    assert result['kinematic_inconsistency']


def test_no_support_is_invalid_rather_than_zero_slip():
    result = rolling_consistency(candidate_plant(), [])
    assert not result['velocity_consistency_valid']
    assert np.isnan(result['contact_velocity_residual_rms_m_s'])


def test_damped_allocation_rejects_unbounded_rates_at_a_vanishing_lever():
    from spherical_wheel.experiments import _rolling_allocation
    # One drive can move the contact, but only through a 10 micrometre lever.
    rows = np.array([[1e-5, 0., 0., 0., 0.], [0., 0., 0., 0., 0.]])
    rhs = np.array([.06, 0.])
    ordinary = _rolling_allocation(rows, rhs)
    damped = _rolling_allocation(rows, rhs, .01)
    assert ordinary[0] > 5000.
    assert abs(damped[0]) < .2
    # A regular 12 cm rolling lever still achieves the requested speed closely.
    rows[0, 0] = .12
    damped = _rolling_allocation(rows, rhs, .01)
    assert abs((rows@damped)[0]-.06) < 2e-5


def test_damping_handles_nearly_parallel_drive_directions():
    from spherical_wheel.experiments import _rolling_allocation
    # Both lever arms are substantial, but their velocity directions almost
    # coincide. Matching the orthogonal demand would require opposing huge rates.
    rows = np.array([[.1, .03, 0., 0., 0.], [.03, .009001, 0., 0., 0.]])
    target = np.array([.06, .03])
    ordinary = _rolling_allocation(rows, target)
    damped = _rolling_allocation(rows, target, .01)
    assert np.max(np.abs(ordinary)) > 1000.
    assert np.max(np.abs(damped)) < 2.
    assert np.linalg.norm(rows@damped-target) > 1e-3

