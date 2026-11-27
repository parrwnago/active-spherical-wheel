"""Task priority and hard TOTAL-command constraints, independent toy maps."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import spherical_wheel.handoff_control as module
from spherical_wheel.handoff_control import BoundedHandoffAllocator, HandoffSettings


CAPS = np.array([1.5, .6, .6, .3, .3])


def solve(G=None, *, settings=None, **overrides):
    kwargs = dict(native_requested=np.zeros(5), input_bounds=CAPS,
        previous_applied=np.zeros(5), dt_s=.005, qvel=np.zeros(9),
        response=np.zeros((9, 5)) if G is None else G,
        fork_rotation_world=np.eye(3), shell_axis_world=np.array([0., 0., 1.]),
        shell_axial_inertia=np.array([.0013, .0013]), support_loads_N=np.array([0., 18., 0., 0.]),
        reference_shell_rates=np.zeros(2))
    kwargs.update(overrides)
    return BoundedHandoffAllocator(settings).solve(**kwargs)


def test_balance_response_wins_over_conflicting_yaw_cost():
    G = np.zeros((9, 5)); G[3, 1] = 1.; G[5, 1] = 5.
    v = np.zeros(9); v[5] = 5.
    u = np.array([0., .2, 0., 0., 0.])
    decision = solve(G, native_requested=u, previous_applied=u, qvel=v)
    assert decision.command[1] == pytest.approx(.2, abs=1e-10)
    assert abs(decision.diagnostics['secondary_balance_delta']).max() < 1e-9


def test_yaw_can_improve_in_balance_nullspace():
    G = np.zeros((9, 5)); G[3, 0] = 1.; G[5, 1] = 1.
    v = np.zeros(9); v[5] = 1.
    u = np.array([.2, .3, 0., 0., 0.])
    result = solve(G, native_requested=u, previous_applied=u, qvel=v)
    assert result.command[0] == pytest.approx(.2, abs=1e-9)
    assert result.command[1] < .2
    assert result.diagnostics['proxy_world_yaw_rate_rad_s'] < 1.
    assert result.diagnostics['change_normalization'] == 'input_bounds'


def test_rate_infeasibility_preserves_best_attainable_balance():
    G = np.zeros((9, 5)); G[3, 0] = G[4, 1] = 1.
    result = solve(G, native_requested=np.array([50., 50., 0., 0., 0.]))
    np.testing.assert_allclose(result.command[:2], [.3, .3], atol=1e-10)
    np.testing.assert_allclose(result.diagnostics['first_stage_balance_delta'], [-1.2, -.3], atol=1e-10)
    assert result.diagnostics['secondary_balance_delta'].max() < 1e-9


def test_all_inputs_obey_actual_bounds_and_slew_on_oversized_requests():
    rng = np.random.default_rng(561)
    for _ in range(20):
        G = rng.normal(size=(9, 5))
        previous = rng.uniform(-1., 1., size=5)*CAPS
        result = solve(G, previous_applied=previous, native_requested=rng.normal(size=5)*100.,
                       qvel=rng.normal(size=9))
        assert np.all(abs(result.command) <= CAPS)
        assert np.all(abs(result.command-previous) <= np.array(HandoffSettings().slew_limits)*.005+1e-14)
        assert np.max(abs(result.diagnostics['secondary_balance_delta'])) <= 1e-8
        assert result.diagnostics['secondary_cost'] <= result.diagnostics['primary_point_secondary_cost']+1e-6


def test_previous_physical_input_is_supplied_not_owned_request_history():
    allocator = BoundedHandoffAllocator()
    kwargs = dict(native_requested=np.ones(5)*100., input_bounds=CAPS, dt_s=.005,
        qvel=np.zeros(9), response=np.zeros((9, 5)), fork_rotation_world=np.eye(3),
        shell_axis_world=np.array([0., 0., 1.]), shell_axial_inertia=np.ones(2)*.001,
        support_loads_N=np.array([0., 18., 0., 0.]), reference_shell_rates=np.zeros(2))
    a = allocator.solve(previous_applied=np.zeros(5), **kwargs)
    b = allocator.solve(previous_applied=-CAPS, **kwargs)
    assert np.all(b.command < a.command)
    assert np.all(b.command+CAPS <= np.array(HandoffSettings().slew_limits)*.005+1e-14)


def test_horizontal_axis_has_no_vertical_rotor_momentum_cost():
    G = np.zeros((9, 5)); G[7, 1] = 10.
    v = np.zeros(9); v[7] = 20.
    settings = HandoffSettings(yaw_weight=0.)
    a = solve(G, settings=settings, qvel=v, shell_axis_world=np.array([1., 0., 0.]))
    b = solve(G, settings=replace(settings, momentum_weight=0.), qvel=v,
              shell_axis_world=np.array([1., 0., 0.]))
    np.testing.assert_allclose(a.command, b.command, atol=1e-11)
    np.testing.assert_array_equal(a.diagnostics['relative_rotor_vertical_momentum_Nms'], np.zeros(2))


def test_momentum_normalization_retains_axis_and_physical_inertia_units():
    G = np.zeros((9, 5)); G[7, 1] = 10.
    v = np.zeros(9); v[7] = 20.
    a = solve(G, qvel=v)
    b = solve(G, qvel=v, shell_axial_inertia=np.array([.13, .13]))
    assert a.command[1] < 0.
    np.testing.assert_allclose(a.command, b.command, atol=1e-11)
    np.testing.assert_allclose(b.diagnostics['relative_rotor_vertical_momentum_Nms'],
                               100.*a.diagnostics['relative_rotor_vertical_momentum_Nms'])


def test_incoming_readiness_does_not_penalize_already_matched_pre_spin():
    G = np.zeros((9, 5)); G[7, 1] = 10.
    v = np.zeros(9); v[7] = 10.
    a = solve(G, qvel=v)
    ready = np.array([0., 1., 0., 0., 0.])
    b = solve(G, qvel=v, readiness=ready, preparation_shell_rates=np.array([10., 0.]))
    assert a.command[1] < -.01
    assert b.command[1] == pytest.approx(0., abs=1e-10)
    np.testing.assert_array_equal(b.diagnostics['auxiliary_shell_target_rad_s'], [10., 0.])


def test_incoming_belt_can_reach_preparation_speed_within_existing_preview():
    # The null-authority incoming belt should retain its preparation command,
    # not be suppressed by an accidental dt-dependent change-cost scale.
    previous = np.zeros(5)
    ready = np.array([0., 0., 0., 1., 0.])
    nominal = np.array([0., 0., 0., .3, 0.])
    for _ in range(24):  # Existing preview horizon .12 s, controller step .005.
        previous = solve(native_requested=nominal, previous_applied=previous,
                         readiness=ready).command.copy()
    assert previous[3] >= .99*.3


def test_missing_support_does_not_add_an_aerial_yaw_policy():
    G = np.zeros((9, 5)); G[5, 1] = 10.; G[7, 1] = 10.
    v = np.zeros(9); v[5] = 10.; v[7] = 20.
    result = solve(G, qvel=v, support_loads_N=np.zeros(4))
    np.testing.assert_allclose(result.command, 0., atol=1e-12)
    assert not result.diagnostics['auxiliary_damping_enabled']


def test_solver_failure_recovers_the_box_equality_qp(monkeypatch):
    G = np.zeros((9, 5)); G[3, 0] = 1.; G[5, 1] = 1.
    v = np.zeros(9); v[5] = 1.
    u = np.array([.2, .3, 0., 0., 0.])
    expected = solve(G, native_requested=u, previous_applied=u, qvel=v)
    monkeypatch.setattr(module, 'minimize', lambda *a, **k:
        SimpleNamespace(success=False, x=np.full(5, np.nan), message='injected solver failure'))
    result = solve(G, native_requested=u, previous_applied=u, qvel=v)
    assert result.diagnostics['secondary_recovery_used']
    assert result.diagnostics['recovery_feasible_faces'] > 0
    np.testing.assert_allclose(result.command, expected.command, atol=1e-7)


def test_inputs_and_returned_command_cannot_alias():
    G = np.eye(9, 5); native = np.array([1., -.2, .3, -.3, .2]); previous = np.zeros(5)
    copies = [a.copy() for a in (G, native, previous, CAPS)]
    decision = solve(G, native_requested=native, previous_applied=previous)
    for a, b in zip((G, native, previous, CAPS), copies):np.testing.assert_array_equal(a, b)
    with pytest.raises(ValueError):decision.command[0] = 0.


@pytest.mark.parametrize('overrides', [
    {'dt_s': 0.}, {'response': np.full((9, 5), np.nan)},
    {'previous_applied': np.ones(5)*100.}, {'support_loads_N': -np.ones(4)},
    {'shell_axis_world': np.zeros(3)}, {'fork_rotation_world': np.ones((3, 3))},
    {'readiness': np.ones(5)*1.1}, {'shell_axial_inertia': np.zeros(2)},
])
def test_invalid_problem_is_rejected(overrides):
    with pytest.raises(ValueError):solve(**overrides)
