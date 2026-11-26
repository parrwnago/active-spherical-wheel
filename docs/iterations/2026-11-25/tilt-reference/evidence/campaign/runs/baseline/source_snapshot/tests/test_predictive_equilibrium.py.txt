"""Independent hold verification and input-boundary checks for goal audits."""
import json

import mujoco
import numpy as np
import pytest

from spherical_wheel.mirror import StateMirror
from spherical_wheel.predictive_equilibrium import (EquilibriumSettings, audit_rest_target,
                                                     solve_goal_equilibrium)
from spherical_wheel.predictive_model import EstimatedPlanePredictor


@pytest.fixture(scope='module')
def mirror(tmp_path_factory):
    return StateMirror(model_dir=tmp_path_factory.mktemp('predictive_equilibrium'))


def setup(mirror, slope_deg=0.):
    angle = np.deg2rad(slope_deg)
    normal = np.array([0., -np.sin(angle), np.cos(angle)])
    qpos = mirror.model.qpos0.copy()
    qpos[:3] = mirror.radius*normal
    qpos[7:10] = [.8 if slope_deg else .3, 0., 0.]
    upper_moment = mirror.model.body_mass[mirror.fork_id]*mirror.model.body_ipos[mirror.fork_id, 2]
    lean_y = -(mirror.total_mass*mirror.radius/upper_moment)*normal[1]
    roll = -np.arcsin(lean_y)
    qpos[3:7] = [np.cos(roll/2), np.sin(roll/2), 0., 0.]
    mirror.set_state(qpos, np.zeros(9), 0., np.zeros(5))
    predictor = EstimatedPlanePredictor(mirror)
    predictor.update(qpos, np.zeros(9), normal, [0, 0, 0], mirror.total_mass*9.81)
    return predictor, qpos


def test_known_flat_rest_passes_forward_hold_and_fixed_goal(mirror):
    predictor, qpos = setup(mirror)
    result = solve_goal_equilibrium(predictor, qpos)
    assert result.verified
    assert result.diagnostics['selected_audit']['hold_verified']
    assert result.diagnostics['selected_audit']['stationary_acceleration_verified']
    np.testing.assert_array_equal(result.qpos[:2], qpos[:2])
    assert np.max(abs(result.inputs)) < 1e-5
    assert len(result.diagnostics['attempts']) == 1
    assert max(abs(np.array(result.diagnostics['selected_audit']['acceleration_residual_all9']))) < .001
    json.dumps(result.diagnostics, allow_nan=False)


def test_slope_best_found_input_is_not_verified_by_solver_success(mirror):
    predictor, qpos = setup(mirror, 10.)
    result = solve_goal_equilibrium(predictor, qpos, allow_pose_adjustment=False)
    assert result.diagnostics['attempts'][0]['optimizer_success']
    assert not result.verified
    assert not result.diagnostics['selected_audit']['hold_verified']
    assert len(result.diagnostics['selected_audit']['acceleration_residual_all9']) == 9
    assert len(result.diagnostics['selected_audit']['inverse_required_unactuated_force_first6']) == 6
    assert np.linalg.norm(result.diagnostics['selected_audit']['inverse_required_unactuated_force_first6'][:3]) > 1.
    json.dumps(result.diagnostics, allow_nan=False)


def test_goal_plane_and_measured_mirror_are_frozen(mirror):
    predictor, qpos = setup(mirror, 10.)
    plane, normal = predictor._plane_point.copy(), predictor.normal.copy()
    measured_q, measured_v = mirror.data.qpos.copy(), mirror.data.qvel.copy()
    flags = mirror.model.opt.disableflags
    result = solve_goal_equilibrium(predictor, qpos, settings=EquilibriumSettings(max_nfev=4))
    np.testing.assert_array_equal(result.qpos[:2], qpos[:2])
    np.testing.assert_array_equal(predictor._plane_point, plane)
    np.testing.assert_array_equal(predictor.normal, normal)
    np.testing.assert_array_equal(mirror.data.qpos, measured_q)
    np.testing.assert_array_equal(mirror.data.qvel, measured_v)
    assert mirror.model.opt.disableflags == flags
    assert mirror.data.ncon == 0
    assert np.all(abs(result.inputs) <= mirror.bounds)
    np.testing.assert_array_equal(predictor.data.qfrc_applied, 0.)
    np.testing.assert_array_equal(predictor.data.xfrc_applied, 0.)


def test_hold_gate_uses_peak_drift_not_only_endpoint(mirror, monkeypatch):
    predictor, qpos = setup(mirror)
    poses = np.tile(qpos, (3, 1))
    poses[1, 0] += .001
    velocities = np.zeros((3, 9))
    monkeypatch.setattr(predictor, 'rollout', lambda *args, **kwargs: (poses, velocities))
    result = audit_rest_target(predictor, qpos, np.zeros(5), EquilibriumSettings(hold_s=.04))
    assert np.array_equal(poses[-1], qpos)
    assert result['hold_peak_metrics']['xy_drift_m'] == pytest.approx(.001)
    assert not result['hold_gates']['xy_drift']
    assert not result['equilibrium_verified']


def test_audit_does_not_read_live_ground_or_solved_force_api(mirror, monkeypatch):
    predictor, qpos = setup(mirror)
    class Forbidden:
        def __getattribute__(self, name):
            raise AssertionError('Audit read live ground or terrain')
    def forbidden(*args, **kwargs):
        raise AssertionError('Audit used solved-force measurement API')
    mirror.ground = mirror.terrain = Forbidden()
    monkeypatch.setattr(mujoco, 'mj_contactForce', forbidden)
    try:
        result = solve_goal_equilibrium(predictor, qpos)
        assert result.verified
    finally:
        mirror.ground = mirror.terrain = None


@pytest.mark.parametrize('field,value', [('height_change_m', 0.), ('tilt_change_rad', -.1),
    ('phase_change_rad', np.nan), ('hold_s', 0.), ('hold_step_s', .03),
    ('max_xy_drift_m', -.1), ('max_body_rate_rad_s', np.inf),
    ('max_acceleration_residual', 0.), ('max_nfev', 0), ('max_nfev', True), ('max_nfev', 1.5)])
def test_invalid_settings_are_rejected(field, value):
    with pytest.raises(ValueError):
        EquilibriumSettings(**{field: value})


def test_invalid_state_inputs_and_bounds_are_rejected(mirror):
    predictor, qpos = setup(mirror)
    for bad_q in [np.zeros(10), np.ones(9), np.full(10, np.nan)]:
        with pytest.raises(ValueError):
            solve_goal_equilibrium(predictor, bad_q)
    for bad_bound in [np.zeros(5), -np.ones(5), mirror.bounds*2, np.ones(4)]:
        with pytest.raises(ValueError):
            solve_goal_equilibrium(predictor, qpos, bounds=bad_bound)
    with pytest.raises(ValueError):
        solve_goal_equilibrium(predictor, qpos, input_seed=np.full(5, np.nan))
    with pytest.raises(ValueError):
        audit_rest_target(predictor, qpos, np.zeros(5, complex)+1j)
