"""Static authority model contracts, without Gazebo or time integration."""
import copy

import mujoco
import numpy as np
import pytest

from spherical_wheel.handoff_model import MeasuredContactVelocityModel


@pytest.fixture
def robot():
    model = mujoco.MjModel.from_xml_string('''
    <mujoco><compiler angle="radian"/>
      <option gravity="0 0 -9.81"/>
      <default><joint damping=".0001" armature="0"/></default>
      <worldbody><body name="fork" pos="0 0 .12"><freejoint/>
        <inertial pos="0 0 .16" mass="1.2" diaginertia=".01 .012 .014"/>
        <body name="carrier"><joint name="shaft" axis="0 1 0"/>
          <inertial pos="0 0 0" mass=".36" diaginertia=".0008 .0009 .001"/>
          <body name="shell_A"><joint name="A" axis="1 0 0"/>
            <inertial pos=".05 0 0" mass=".15" diaginertia=".0004 .0005 .0005"/>
          </body>
          <body name="shell_B"><joint name="B" axis="1 0 0"/>
            <inertial pos="-.05 0 0" mass=".15" diaginertia=".0004 .0005 .0005"/>
          </body>
        </body>
      </body></worldbody>
      <actuator><motor joint="shaft"/><motor joint="A"/><motor joint="B"/></actuator>
    </mujoco>''')
    q = model.qpos0.copy()
    q[3:7] = np.array([.98, .10, -.06, .13])
    q[3:7] /= np.linalg.norm(q[3:7])
    q[7:10] = [-1.1, .2, .8]
    v = np.array([.10, -.05, .02, .3, -.2, .5, .4, -1., 1.7])
    return model, q, v


def record(model, name='shell_A', normal=(0., 0., 1.), load=18.):
    body = 'carrier' if name.startswith('belt') else name
    return dict(name=name+'_smooth', body=int(mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, body)), pos=np.array([.01, -.02, 0.]),
        normal=np.array(normal), normal_force=load, friction=.7)


def _cross(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def test_no_support_matches_free_rigid_body_dynamics_and_no_belt_authority(robot):
    model, q, v = robot
    data = mujoco.MjData(model)
    data.qpos[:], data.qvel[:] = q, v
    mujoco.mj_forward(model, data)
    mass = np.empty((9, 9))
    mujoco.mj_fullM(model, data, mass)
    helper = MeasuredContactVelocityModel(model)
    result = helper.evaluate(q, v, [], horizon_s=.015)
    command = np.array([.1, -.2, .15, .3, -.3])
    expected = v+.015*np.linalg.solve(
        mass, result.drive_matrix@command[:3]+data.qfrc_passive-data.qfrc_bias)
    np.testing.assert_allclose(result.predict(command), expected, atol=1e-13, rtol=0.)
    np.testing.assert_allclose(result.mass_matrix, mass, atol=1e-13, rtol=0.)
    np.testing.assert_allclose(result.bias_force, data.qfrc_bias, atol=1e-13, rtol=0.)
    np.testing.assert_allclose(result.passive_force, data.qfrc_passive, atol=1e-15, rtol=0.)
    np.testing.assert_array_equal(result.response[:, 3:], np.zeros((9, 2)))
    assert result.contact_names == () and result.contact_jacobian.shape == (0, 9)


def test_soft_projection_satisfies_momentum_and_optimality_equations(robot):
    model, q, v = robot
    result = MeasuredContactVelocityModel(model).evaluate(
        q, v, [record(model, 'belt_A', (.3, -.2, 1.))])
    u = np.array([.2, -.1, .15, .18, -.2])
    predicted = result.predict(u)
    free = result.free_offset+result.free_response@u
    impulse = result.contact_impulse_offset+result.contact_impulse_response@u
    residual = result.contact_jacobian@predicted+result.belt_surface_matrix@u[3:]
    np.testing.assert_allclose(impulse, -result.contact_penalty_kg*residual, atol=1e-13)
    np.testing.assert_allclose(result.mass_matrix@(predicted-free),
                               result.contact_jacobian.T@impulse, atol=1e-13)
    # A finite soft contact penalty reduces disagreement rather than asserting
    # an exact zero-slip correction for an arbitrarily large measured mismatch.
    free_residual = result.contact_jacobian@free+result.belt_surface_matrix@u[3:]
    assert 0 < np.linalg.norm(residual) < np.linalg.norm(free_residual)


def test_contact_jacobian_matches_independent_world_joint_chain(robot):
    model, q, v = robot
    helper = MeasuredContactVelocityModel(model)
    for name in ('shell_A', 'shell_B', 'belt_A'):
        item = record(model, name)
        result = helper.evaluate(q, v, [item])
        point = item['pos']
        arm = point-q[:3]
        expected = np.zeros((3, 9))
        expected[:, :3] = np.eye(3)
        expected[:, 3:6] = -_cross(arm)@result.fork_rotation_world
        expected[:, 6] = np.cross(result.shaft_axis_world, arm)
        if name.startswith('shell'):
            expected[:, 7 if name == 'shell_A' else 8] = np.cross(result.shell_axis_world, arm)
        np.testing.assert_allclose(result.contact_jacobian, expected, atol=2e-15, rtol=0.)


def test_belt_direction_is_tangent_with_native_sign_and_projected_length(robot):
    model, q, v = robot
    n = np.array([.3, -.2, 1.]); n /= np.linalg.norm(n)
    result = MeasuredContactVelocityModel(model).evaluate(q, v, [record(model, 'belt_A', n)])
    # The shaft's Y axis is also carrier Y; the shaft rotates about that axis.
    along = result.shaft_axis_world-n*(n@result.shaft_axis_world)
    assert np.linalg.norm(along) < 1.
    np.testing.assert_allclose(result.belt_surface_matrix[:, 0], along, atol=1e-15)
    np.testing.assert_array_equal(result.belt_surface_matrix[:, 1], np.zeros(3))
    np.testing.assert_array_equal(result.response[:, 4], np.zeros(9))
    assert abs(n@result.belt_surface_matrix[:, 0]) < 1e-15
    material_response = result.contact_jacobian@result.response[:, 3]
    assert material_response@along < 0  # body material moves opposite to +belt speed


def test_splitting_the_same_contact_load_does_not_change_authority(robot):
    model, q, v = robot
    helper = MeasuredContactVelocityModel(model)
    single = helper.evaluate(q, v, [record(model, load=18.)])
    split = helper.evaluate(q, v, [record(model, load=9.), record(model, load=9.)])
    np.testing.assert_allclose(split.offset, single.offset, atol=1e-13, rtol=0.)
    np.testing.assert_allclose(split.response, single.response, atol=1e-13, rtol=0.)
    assert split.constraint_rank == single.constraint_rank == 3


def test_vanishing_load_continuously_returns_to_airborne_response(robot):
    model, q, v = robot
    helper = MeasuredContactVelocityModel(model)
    free = helper.evaluate(q, v, [])
    tiny = helper.evaluate(q, v, [record(model, load=1e-12)])
    zero = helper.evaluate(q, v, [record(model, load=0.)])
    np.testing.assert_allclose(tiny.offset, free.offset, atol=1e-8, rtol=0.)
    np.testing.assert_allclose(tiny.response, free.response, atol=1e-8, rtol=0.)
    np.testing.assert_array_equal(zero.offset, free.offset)
    np.testing.assert_array_equal(zero.response, free.response)


def test_conflicting_belts_have_visible_residual_instead_of_unbounded_velocity(robot):
    model, q, v = robot
    helper = MeasuredContactVelocityModel(model)
    result = helper.evaluate(q, v, [record(model, 'belt_A', load=9.),
                                    record(model, 'belt_B', load=9.)])
    u = np.array([0., 0., 0., .3, -.3])
    np.testing.assert_allclose(result.predict(u), result.predict(np.zeros(5)), atol=1e-13)
    disagreement = result.constraint_response@u
    assert np.linalg.norm(disagreement) > .1
    assert result.constraint_rank == 3 and len(result.constraint_singular_values) == 6
    assert np.isfinite(result.response).all()


def test_internal_motor_response_preserves_total_world_angular_momentum(robot):
    model, q, v = robot
    result = MeasuredContactVelocityModel(model).evaluate(q, v, [])
    # Each drive applies an internal equal/opposite couple. Its instantaneous
    # fixed-pose input sensitivity cannot create complete-system momentum.
    np.testing.assert_allclose(result.total_momentum_map_world@result.response[:, :3],
                               np.zeros((3, 3)), atol=1e-14)


def test_contact_impulse_total_momentum_matches_external_world_lever(robot):
    model, q, v = robot
    result = MeasuredContactVelocityModel(model).evaluate(q, v, [record(model)])
    u = np.array([.2, -.1, .3, .2, -.1])
    induced = result.predict(u)-(result.free_offset+result.free_response@u)
    impulse = result.contact_impulse_offset+result.contact_impulse_response@u
    lever = result.contact_positions_world_m[0]-result.system_COM_world_m
    np.testing.assert_allclose(result.total_momentum_map_world@induced,
                               np.cross(lever, impulse), atol=1e-13)


def test_zero_tangent_confidence_retains_normal_impulse_and_no_belt_authority(robot):
    model, q, v = robot
    normal = np.array([.3, -.2, 1.]); normal /= np.linalg.norm(normal)
    helper = MeasuredContactVelocityModel(model)
    result = helper.evaluate(q, v, [record(model, 'belt_A', normal)], tangent_confidence=0.)
    np.testing.assert_array_equal(result.response[:, 3:], np.zeros((9, 2)))
    assert result.constraint_rank == 1
    impulse = result.contact_impulse_offset+result.contact_impulse_response@np.array([.2, .1, -.1, .3, -.2])
    np.testing.assert_allclose(impulse-normal*(normal@impulse), np.zeros(3), atol=1e-13)
    assert abs(normal@impulse) > 0
    # Opposite belt inputs may leave a tangential residual, but cannot obtain
    # artificial body authority from contact when tangential confidence is zero.
    np.testing.assert_array_equal(result.predict([0., 0., 0., .3, -.2]),
                                  result.predict(np.zeros(5)))


def test_anisotropic_contact_impulse_uses_full_world_penalty_matrix(robot):
    model, q, v = robot
    item = record(model, 'belt_A', (.3, -.2, 1.))
    result = MeasuredContactVelocityModel(model).evaluate(q, v, [item], tangent_confidence=.25)
    normal = result.contact_normals_world[0]
    expected = (10.*18./9.81)*(.25*np.eye(3)+.75*np.outer(normal, normal))
    np.testing.assert_allclose(result.contact_penalty_matrix_kg, expected, atol=1e-14)
    u = np.array([.2, -.1, .15, .18, -.2])
    impulse = result.contact_impulse_offset+result.contact_impulse_response@u
    residual = result.contact_jacobian@result.predict(u)+result.belt_surface_matrix@u[3:]
    np.testing.assert_allclose(impulse, -expected@residual, atol=1e-13)


def test_shell_axial_inertias_are_about_the_physical_hinge_axes(robot):
    model, q, v = robot
    helper = MeasuredContactVelocityModel(model)
    first = helper.evaluate(q, v, [])
    np.testing.assert_allclose(first.shell_axial_inertias_kg_m2, [.0004, .0004], atol=1e-16)
    q[7:10] += [.3, 1.2, -2.]
    second = helper.evaluate(q, v, [])
    np.testing.assert_array_equal(first.shell_axial_inertias_kg_m2, second.shell_axial_inertias_kg_m2)


def test_model_reads_no_solver_or_terrain_and_leaves_caller_values_unchanged(robot, monkeypatch):
    model, q, v = robot
    helper = MeasuredContactVelocityModel(model)
    items = [record(model)]
    before = copy.deepcopy(items)
    q0, v0, masses = q.copy(), v.copy(), model.body_mass.copy()
    for name in ('mj_forward', 'mj_step', 'mj_step1', 'mj_step2', 'mj_fwdPosition',
                 'mj_collision', 'mj_contactForce'):
        monkeypatch.setattr(mujoco, name, lambda *args, **kwargs: pytest.fail('Forbidden dynamics/contact API'))
    result = helper.evaluate(q, v, items)
    np.testing.assert_array_equal(q, q0); np.testing.assert_array_equal(v, v0)
    np.testing.assert_array_equal(model.body_mass, masses)
    np.testing.assert_array_equal(items[0]['pos'], before[0]['pos'])
    np.testing.assert_array_equal(items[0]['normal'], before[0]['normal'])
    assert helper.model is not model
    assert not result.response.flags.writeable and not result.mass_matrix.flags.writeable
    model.body_mass[1] *= 2.
    repeated = helper.evaluate(q, v, items)
    np.testing.assert_array_equal(repeated.mass_matrix, result.mass_matrix)


@pytest.mark.parametrize('fault', ['quaternion', 'normal', 'body', 'load', 'horizon'])
def test_invalid_measurements_fail_explicitly(robot, fault):
    model, q, v = robot
    helper = MeasuredContactVelocityModel(model)
    item = record(model); horizon = .02
    if fault == 'quaternion': q[3:7] = 0.
    if fault == 'normal': item['normal'] = np.zeros(3)
    if fault == 'body': item['body'] = helper.body_ids[1]
    if fault == 'load': item['normal_force'] = -1.
    if fault == 'horizon': horizon = 0.
    with pytest.raises(ValueError): helper.evaluate(q, v, [item], horizon_s=horizon)


@pytest.mark.parametrize('confidence', [-.1, 1.1, float('nan')])
def test_invalid_contact_confidence_is_not_silently_clamped(robot, confidence):
    model, q, v = robot
    with pytest.raises(ValueError):
        MeasuredContactVelocityModel(model).evaluate(q, v, [record(model)],
                                                      tangent_confidence=confidence)
