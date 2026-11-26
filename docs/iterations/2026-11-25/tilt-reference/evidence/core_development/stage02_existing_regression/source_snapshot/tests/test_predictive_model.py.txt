"""Physical-input and private-model checks for the estimated-plane predictor."""
from pathlib import Path

import mujoco
import numpy as np
import pytest

from spherical_wheel.mirror import StateMirror
from spherical_wheel.predictive_model import EstimatedPlanePredictor


@pytest.fixture(scope='module')
def mirror(tmp_path_factory):
    return StateMirror(model_dir=tmp_path_factory.mktemp('predictive_model'))


def state(mirror, phase=.3, elevation=0.):
    q = mirror.model.qpos0.copy()
    q[:3] = [0., 0., mirror.radius+elevation]
    q[3:7] = [1., 0., 0., 0.]
    q[7:10] = [phase, 0., 0.]
    return q, np.zeros(9)


def setup(mirror, phase=.3):
    q, v = state(mirror, phase)
    mirror.set_state(q, v, 0., np.zeros(5))
    predictor = EstimatedPlanePredictor(mirror)
    calibration = predictor.update(q, v, [0., 0., 1.], [0., 0., 0.], mirror.total_mass*9.81)
    assert calibration['load_fit_within_tolerance']
    return predictor, q, v


def test_prediction_owns_model_and_preserves_measured_mirror(mirror):
    q, v = state(mirror)
    measured_input = np.array([.03, -.01, .01, .02, -.02])
    mirror.set_state(q, v, 3., measured_input)
    before = {}
    for name in ('qpos', 'qvel', 'ctrl', 'qacc_warmstart', 'mocap_pos', 'mocap_quat'):
        before[name] = np.asarray(getattr(mirror.data, name)).copy()
    flags = mirror.model.opt.disableflags
    surface = mirror.model.geom_surfacevel.copy()
    friction = mirror.model.geom_friction.copy()
    limited = mirror.model.actuator_ctrllimited.copy()
    predictor = EstimatedPlanePredictor(mirror)
    predictor.update(q, v, [0, 0, 1], [0, 0, 0], mirror.total_mass*9.81, friction=.45)
    predictor.transition(q, v, [.2, -.1, .05, .08, -.04], .01)
    assert predictor.model is not mirror.model and predictor.data is not mirror.data
    assert mirror.data.ncon == 0
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(mirror.data, name), value)
    np.testing.assert_array_equal(mirror.last_input, measured_input)
    np.testing.assert_array_equal(mirror.model.geom_surfacevel, surface)
    np.testing.assert_array_equal(mirror.model.geom_friction, friction)
    np.testing.assert_array_equal(mirror.model.actuator_ctrllimited, limited)
    assert mirror.model.opt.disableflags == flags
    assert flags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    assert not predictor.model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)


def test_estimated_load_fits_private_contact_without_external_force(mirror):
    predictor, q, v = setup(mirror)
    result = predictor.diagnostics
    assert abs(result['normal_load_residual_N']) < .02
    assert abs(result['plane_offset_m']) < result['plane_offset_bound_m']
    assert result['predicted_contact_count'] > 0
    np.testing.assert_array_equal(predictor.data.qfrc_applied, 0.)
    np.testing.assert_array_equal(predictor.data.xfrc_applied, 0.)
    qq, vv = predictor.transition(q, v, np.zeros(5), .02)
    assert abs(qq[2]-q[2]) < 1e-5
    assert np.linalg.norm(vv) < 1e-3


def test_unreachable_load_is_reported_and_does_not_create_support(mirror):
    q, v = state(mirror, elevation=.1)
    predictor = EstimatedPlanePredictor(mirror)
    result = predictor.update(q, v, [0, 0, 1], [0, 0, 0], mirror.total_mass*9.81)
    assert not result['load_fit_bracketed']
    assert not result['load_fit_within_tolerance']
    assert result['normal_load_residual_N'] == pytest.approx(-mirror.total_mass*9.81)
    assert abs(result['plane_offset_m']) <= .003
    qq, vv = predictor.transition(q, v, np.zeros(5), .02)
    assert qq[2] < q[2] and vv[2] < -.1
    assert predictor.data.ncon == 0
    np.testing.assert_array_equal(predictor.data.qfrc_applied, 0.)
    np.testing.assert_array_equal(predictor.data.xfrc_applied, 0.)


def test_belt_speed_input_acts_through_loaded_contact_not_motor_torque(mirror):
    predictor, q, v = setup(mirror, np.pi/2)
    q0, v0 = predictor.transition(q, v, np.zeros(5), .02)
    qa, va = predictor.transition(q, v, [0, 0, 0, .05, 0], .02)
    np.testing.assert_array_equal(predictor.data.ctrl, 0.)
    assert va[1] < -.01 and abs(qa[1]-q0[1]) > 1e-4
    qb, vb = predictor.transition(q, v, [0, 0, 0, 0, .05], .02)
    np.testing.assert_allclose(qb, q0, atol=1e-12)
    np.testing.assert_allclose(vb, v0, atol=1e-12)
    qt, vt = predictor.transition(q, v, [.1, 0, 0, 0, 0], .02)
    assert abs(vt[6]-v0[6]) > .01
    assert np.linalg.norm(vt-va) > .01


def test_previous_prediction_does_not_contaminate_next_transition(mirror):
    predictor, q, v = setup(mirror)
    command = [.12, -.03, .02, .01, 0]
    first = predictor.transition(q, v, command, .02)
    displaced, changed = predictor.perturb(q, v, np.r_[np.zeros(9), np.ones(9)*.3])
    predictor.transition(displaced, changed, [-.2, .1, -.1, -.04, .08], .03)
    second = predictor.transition(q, v, command, .02)
    np.testing.assert_array_equal(first[0], second[0])
    np.testing.assert_array_equal(first[1], second[1])


def test_local_affine_model_predicts_independent_mixed_perturbation(mirror):
    predictor, q, v = setup(mirror)
    command = np.zeros(5)
    A, B, f0 = predictor.linearize(q, v, command, .02)
    rng = np.random.default_rng(137)
    dx, du = rng.normal(size=18)*1e-7, rng.normal(size=5)*1e-7
    perturbed_q, perturbed_v = predictor.perturb(q, v, dx)
    next_q, next_v = predictor.transition(perturbed_q, perturbed_v, command+du, .02)
    actual = predictor.tangent(q, v, next_q, next_v)
    predicted = f0+A@dx+B@du
    relative = np.linalg.norm(actual-predicted)/np.linalg.norm(actual-f0)
    assert relative < .002
    assert A.shape == (18, 18) and B.shape == (18, 5) and f0.shape == (18,)
    assert np.isfinite(A).all() and np.isfinite(B).all()


def test_derivative_at_torque_bound_is_not_halved_by_hidden_clipping(mirror):
    predictor, q, v = setup(mirror)
    command = np.array([mirror.bounds[0], 0, 0, 0, 0])
    _, B, _ = predictor.linearize(q, v, command, .01)
    step = 1e-4
    q0, v0 = predictor.transition(q, v, command, .01)
    qm, vm = predictor.transition(q, v, command-np.array([step, 0, 0, 0, 0]), .01)
    one_sided = (predictor.tangent(q, v, q0, v0)-predictor.tangent(q, v, qm, vm))/step
    assert np.linalg.norm(one_sided) > .01
    np.testing.assert_allclose(B[:, 0], one_sided, rtol=2e-3, atol=1e-5)
    assert mirror.model.actuator_ctrllimited.all()
    assert not predictor.model.actuator_ctrllimited.any()


def test_update_uses_latest_applied_input_and_never_reads_ground(mirror):
    class Forbidden:
        def __getattribute__(self, name):
            raise AssertionError('Predictor read live terrain/ground')

    q, v = state(mirror)
    mirror.set_state(q, v, 0., np.zeros(5))
    predictor = EstimatedPlanePredictor(mirror)
    latest = np.array([.1, -.03, .01, .02, -.01])
    mirror.set_state(q, v, .005, latest)
    mirror.ground, mirror.terrain = Forbidden(), Forbidden()
    try:
        predictor.update(q, v, [0, 0, 1], [0, 0, 0], mirror.total_mass*9.81)
        np.testing.assert_array_equal(predictor.data.ctrl, latest[:3])
        np.testing.assert_array_equal(predictor.model.geom_surfacevel[predictor.belt_ids, 1], latest[3:])
        predictor.transition(q, v, latest, .01)
    finally:
        mirror.ground = mirror.terrain = None


@pytest.mark.parametrize('bad', [-.01, 0., .0015, float('nan')])
def test_invalid_prediction_duration_is_rejected(mirror, bad):
    predictor, q, v = setup(mirror)
    with pytest.raises(ValueError):
        predictor.transition(q, v, np.zeros(5), bad)


def test_trajectory_linearization_uses_one_global_state_chart(mirror):
    predictor, q, v = setup(mirror)
    commands = np.array([[.05, .03, -.01, 0, 0], [.08, .02, -.02, 0, 0],
                         [.06, .04, -.01, 0, 0]])
    A, B, c, nominal = predictor.linearize_trajectory(q, v, commands, .01)
    assert A.shape == (3, 18, 18) and B.shape == (3, 18, 5)
    assert c.shape == (3, 18) and nominal.shape == (4, 18)
    # A nonzero node's coordinate perturbation is constructed independently
    # from the root chart, not by perturbing that node in its own local frame.
    rng = np.random.default_rng(218)
    dx, du = rng.normal(size=18)*1e-7, rng.normal(size=5)*1e-7
    node = nominal[2]
    perturbed = q.copy()
    mujoco.mj_integratePos(predictor.model, perturbed, (node+dx)[:9], 1.)
    next_q, next_v = predictor.transition(perturbed, v+(node+dx)[9:], commands[2]+du, .01)
    measured = predictor.tangent(q, v, next_q, next_v)
    predicted = A[2]@(node+dx)+B[2]@du+c[2]
    change = measured-nominal[3]
    assert np.linalg.norm(measured-predicted)/np.linalg.norm(change) < .003
    np.testing.assert_array_equal(mirror.data.qpos, q)
    np.testing.assert_array_equal(mirror.data.qvel, v)


def test_trajectory_first_stage_matches_local_model_and_replays_nominal(mirror):
    predictor, q, v = setup(mirror)
    commands = np.array([[.04, -.02, .01, 0, 0], [.03, .01, .02, 0, 0]])
    A, B, c, nominal = predictor.linearize_trajectory(q, v, commands, .01)
    local_A, local_B, local_f0 = predictor.linearize(q, v, commands[0], .01)
    np.testing.assert_array_equal(A[0], local_A)
    np.testing.assert_array_equal(B[0], local_B)
    np.testing.assert_array_equal(c[0], local_f0)
    poses, velocities = predictor.rollout(q, v, commands, .01)
    observed = np.asarray([predictor.tangent(q, v, qq, vv) for qq, vv in zip(poses, velocities)])
    np.testing.assert_array_equal(nominal, observed)
    affine = np.zeros(18)
    for index in range(2):
        affine = A[index]@affine+c[index]
        np.testing.assert_allclose(affine, observed[index+1], atol=1e-13)


def test_trajectory_rejects_empty_or_invalid_inputs(mirror):
    predictor, q, v = setup(mirror)
    for commands in [np.empty((0, 5)), np.zeros((2, 4)), np.full((2, 5), np.nan)]:
        with pytest.raises(ValueError):
            predictor.linearize_trajectory(q, v, commands, .01)


def test_sparse_trajectory_jacobians_preserve_knots_and_nonlinear_nominal(mirror):
    predictor, q, v = setup(mirror)
    commands = np.array([[.03+.004*i, .01, -.01, 0, 0] for i in range(6)])
    full_A, full_B, _, full_nominal = predictor.linearize_trajectory(q, v, commands, .005)
    A, B, c, nominal = predictor.linearize_trajectory(q, v, commands, .005, jacobian_stride=2)
    nodes = predictor.diagnostics['trajectory_jacobian_evaluation_nodes']
    assert nodes == [0, 2, 4, 5]
    np.testing.assert_array_equal(nominal, full_nominal)
    np.testing.assert_array_equal(A[nodes], full_A[nodes])
    np.testing.assert_array_equal(B[nodes], full_B[nodes])
    np.testing.assert_allclose(A[1], .5*(full_A[0]+full_A[2]), atol=1e-13)
    np.testing.assert_allclose(B[3], .5*(full_B[2]+full_B[4]), atol=1e-13)
    affine = np.zeros(18)
    for index in range(len(commands)):
        affine = A[index]@affine+c[index]
        np.testing.assert_allclose(affine, nominal[index+1], atol=1e-13)
    for bad in (0, -1, 1.5, True):
        with pytest.raises(ValueError):
            predictor.linearize_trajectory(q, v, commands, .005, jacobian_stride=bad)
