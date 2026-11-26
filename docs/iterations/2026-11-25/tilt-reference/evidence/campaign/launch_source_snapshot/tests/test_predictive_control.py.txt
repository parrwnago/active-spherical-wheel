"""Affine MPC: independent optima, physical limits, recovery and failures."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.linalg import solve_discrete_are

from spherical_wheel.predictive_control import AffineMPC, AffineMPCSettings


pytest.importorskip('osqp')


def settings(nx=1, **overrides):
    return AffineMPCSettings(state_scales=(1.,)*nx, state_weights=(1.,)*nx,
                             **overrides)


def test_one_step_matches_independent_scalar_optimum_with_affine_input_anchor():
    # x1 = .3 + 2*(u-.4), so physical u=.4+(x1-.3)/2.
    # Cost: 3*x1^2 + .2*(u/2)^2 + .7*((u-.1)/2)^2.
    config = settings(horizon_steps=1, input_weight=.2, change_weight=.7,
                      terminal_multiplier=3., regularization=0.)
    controller = AffineMPC(config)
    out = controller.solve(np.array([[1.1]]), np.array([[2.]]), np.array([.3]),
                           np.zeros((1, 1)), np.array([.4]), np.array([.1]), np.array([2.]))
    expected = (3.*2.*.5+.7*.1/4)/(3.*4.+(.2+.7)/4)
    assert out.success
    assert out.command[0] == pytest.approx(expected, abs=2e-7)
    assert out.states[0, 0] == pytest.approx(.3+2*(out.command[0]-.4), abs=1e-12)
    assert not out.diagnostics['hard_state_constraints']
    assert not out.diagnostics['hard_slew_constraints']


def test_nonzero_input_target_matches_scalar_optimum_and_physical_costs():
    # x1-target = 2*u-.65. Input tracking is centered at .7 Nm, while
    # numerical regularization deliberately remains centered at zero.
    config = settings(horizon_steps=1, input_weight=.2, change_weight=.7,
                      terminal_multiplier=3., regularization=.04)
    out = AffineMPC(config).solve(
        np.array([[1.1]]), np.array([[2.]]), np.array([.3]),
        np.array([[.15]]), np.array([.4]), np.array([.1]), np.array([2.]),
        target_inputs=np.array([.7]))
    expected = (3*2*.65+(.2*.7+.7*.1)/4)/(3*4+(.2+.7+.04)/4)
    assert out.success
    assert out.command[0] == pytest.approx(expected, abs=2e-7)
    u = out.command[0]
    assert out.diagnostics['state_cost'] == pytest.approx(3*(2*u-.65)**2)
    assert out.diagnostics['input_cost'] == pytest.approx(.2*((u-.7)/2)**2)
    assert out.diagnostics['change_cost'] == pytest.approx(.7*((u-.1)/2)**2)
    assert out.diagnostics['objective'] == pytest.approx(
        3*(2*u-.65)**2+.2*((u-.7)/2)**2+.7*((u-.1)/2)**2+.04*(u/2)**2)


def test_nonzero_holding_input_preserves_stationary_equilibrium():
    # Absolute zero is NOT the equilibrium input: x+ = A*x+B*(u-u_eq).
    # State, input and change costs can all vanish at a nonzero holding input.
    horizon = 6
    equilibrium_input, bounds = np.array([.5, -.3]), np.array([.8, .6])
    config = settings(2, horizon_steps=horizon, input_weight=(.3, .7),
                      change_weight=.2, regularization=0., eps_abs=1e-9, eps_rel=1e-9)
    arguments = (np.array([[1.1, .1], [0., .9]]), np.eye(2), np.zeros(2),
                 np.zeros((horizon, 2)), equilibrium_input, equilibrium_input, bounds)
    centered = AffineMPC(config).solve(*arguments, target_inputs=equilibrium_input)
    absolute = AffineMPC(config).solve(*arguments)
    assert centered.success and absolute.success
    np.testing.assert_allclose(centered.inputs, np.tile(equilibrium_input, (horizon, 1)), atol=2e-7)
    np.testing.assert_allclose(centered.states, 0., atol=2e-7)
    assert centered.diagnostics['objective'] < 1e-12
    assert np.max(abs(absolute.states)) > .01


def test_stage_input_targets_remain_physical_references_with_hard_input_bounds():
    # No state dynamics or change cost: independent optimum is projection of
    # each PHYSICAL target onto its channel's box, even with unequal scales.
    horizon, bounds = 3, np.array([.3, 1.5])
    targets = np.array([[.2, -.7], [-.25, .9], [.8, -2.]])
    config = replace(settings(horizon_steps=horizon, input_weight=(.3, .8),
                              change_weight=0., regularization=0.), state_weights=(0.,))
    out = AffineMPC(config).solve(np.ones((1, 1)), np.zeros((1, 2)), np.zeros(1),
        np.zeros((horizon, 1)), np.zeros(2), np.zeros(2), bounds, target_inputs=targets)
    assert out.success
    np.testing.assert_allclose(out.inputs, np.clip(targets, -bounds, bounds), atol=2e-7)
    expected_cost = np.sum(((out.inputs-targets)/bounds)**2*np.array(config.input_weight))
    assert out.diagnostics['input_cost'] == pytest.approx(expected_cost)
    np.testing.assert_array_equal(out.diagnostics['target_inputs'], targets)


def test_omitted_and_explicit_zero_input_targets_have_identical_solutions():
    config = settings(2, horizon_steps=5, input_weight=(.2, .7), change_weight=.1)
    arguments = (np.array([[1.03, .1], [-.02, .95]]), np.array([[.2, 0.], [.1, .3]]),
                 np.array([.01, -.03]), np.tile([.2, -.1], (5, 1)),
                 np.array([.1, -.2]), np.array([.04, .1]), np.array([.6, 1.5]))
    omitted = AffineMPC(config).solve(*arguments)
    for zero in (np.zeros(2), np.zeros((5, 2))):
        explicit = AffineMPC(config).solve(*arguments, target_inputs=zero)
        assert omitted.success and explicit.success
        np.testing.assert_array_equal(omitted.inputs, explicit.inputs)
        np.testing.assert_array_equal(omitted.states, explicit.states)
        for field in ('state_cost', 'input_cost', 'change_cost', 'objective'):
            assert explicit.diagnostics[field] == omitted.diagnostics[field]


def test_updated_input_targets_reuse_solver_with_same_result_as_fresh_problem():
    config = settings(horizon_steps=4, input_weight=(.4, .2), change_weight=.3)
    controller = AffineMPC(config)
    arguments = (np.ones((1, 1)), np.zeros((1, 2)), np.zeros(1), np.zeros((4, 1)),
                 np.zeros(2), np.array([.1, -.2]), np.array([.5, 1.2]))
    first = controller.solve(*arguments, target_inputs=np.array([.2, -.6]))
    target = np.array([[-.1, .3], [-.2, .7], [-.3, .8], [-.4, .9]])
    updated = controller.solve(*arguments, target_inputs=target)
    fresh = AffineMPC(config).solve(*arguments, target_inputs=target)
    assert first.success and updated.success and fresh.success
    assert updated.diagnostics['warm_start_used']
    np.testing.assert_allclose(updated.inputs, fresh.inputs, atol=2e-6)
    assert np.max(abs(first.inputs-updated.inputs)) > .4


@pytest.mark.parametrize('target', [np.nan, np.array([np.inf]), np.array([1j]),
                                  np.zeros(2), np.zeros((3, 1)), np.zeros((4, 1, 1))])
def test_invalid_input_targets_are_rejected(target):
    with pytest.raises(ValueError, match='target_inputs'):
        AffineMPC(settings(horizon_steps=4)).solve(
            np.ones((1, 1)), np.ones((1, 1)), np.zeros(1), np.zeros((4, 1)),
            np.zeros(1), np.zeros(1), np.ones(1), target_inputs=target)


def test_augmented_terminal_tail_matches_independent_cross_coupled_scalar_optimum():
    config = settings(horizon_steps=1, input_weight=.2, change_weight=.7,
                      terminal_multiplier=3., regularization=.04)
    tail = np.array([[4., 1.5], [1.5, 2.]])
    # Final error = a*u+b; this is independently derived from the supplied
    # scalar dynamics and physical input target, rather than QP condensation.
    a, b = np.array([2., .5]), np.array([-.65, -.35])
    expected = -(3*2*(-.65)+.2*.5*(-.35)+.7*.5*(-.05)+a@tail@b) / (
        3*4+(.2+.7+.04)/4+a@tail@a)
    out = AffineMPC(config).solve(np.array([[1.1]]), np.array([[2.]]), np.array([.3]),
        np.array([[.15]]), np.array([.4]), np.array([.1]), np.array([2.]),
        target_inputs=np.array([.7]), terminal_tail_weight=tail)
    assert out.success
    assert out.command[0] == pytest.approx(expected, abs=2e-7)
    error = a*out.command[0]+b
    assert out.diagnostics['terminal_tail_cost'] == pytest.approx(error@tail@error)
    assert out.diagnostics['objective'] == pytest.approx(
        out.diagnostics['state_cost']+out.diagnostics['input_cost']+
        out.diagnostics['change_cost']+out.diagnostics['terminal_tail_cost']+
        config.regularization*np.sum((out.inputs/2)**2))


@pytest.mark.parametrize('tail', [np.zeros((1, 1)), np.array([[1., 2.], [0., 1.]]),
                                np.diag([1., -1.]), np.diag([1., np.inf]),
                                np.diag([1., 1j])])
def test_invalid_augmented_terminal_tail_is_rejected(tail):
    with pytest.raises(ValueError, match='terminal_tail_weight'):
        AffineMPC(settings(horizon_steps=4)).solve(
            np.ones((1, 1)), np.ones((1, 1)), np.zeros(1), np.zeros((4, 1)),
            np.zeros(1), np.zeros(1), np.ones(1), terminal_tail_weight=tail)


def test_receding_horizon_matches_unconstrained_finite_horizon_riccati_solution():
    # The first command of an unconstrained finite horizon LQR has an
    # independent backwards Riccati recursion, unlike the condensed QP.
    A = np.array([[1.12, .08], [0., .94]])
    B = np.array([[.03], [.2]])
    x = np.array([.3, -.1])
    Q, R = np.diag([2., .7]), np.array([[.4]])
    P_terminal = 3*Q
    horizon, limit = 9, 4.
    P = P_terminal.copy()
    for _ in range(horizon):
        K = np.linalg.solve(R+B.T@P@B, B.T@P@A)
        P = Q+A.T@P@(A-B@K)
    expected = -K@x
    config = settings(2, horizon_steps=horizon, input_weight=.4*limit**2,
                      change_weight=0., terminal_multiplier=3., regularization=0.)
    # x is encoded through the current-state anchor's one-step drift A*x;
    # target=-x then represents zero absolute-state error at every step.
    config = replace(config, state_weights=(2., .7))
    out = AffineMPC(config).solve(A, B, A@x-x, np.tile(-x, (horizon, 1)),
                                 np.zeros(1), np.zeros(1), np.array([limit]),
                                 terminal_weight=P_terminal)
    assert out.success
    np.testing.assert_allclose(out.command, expected, atol=2e-6)
    assert np.max(abs(out.inputs)) < limit


def test_input_bounds_are_physical_per_channel_and_drift_is_retained():
    config = settings(2, horizon_steps=7, input_weight=1e-6, change_weight=0.)
    A, B = np.eye(2), np.eye(2)
    bounds = np.array([.3, 1.5])
    drift = np.array([.8, -2.])
    out = AffineMPC(config).solve(A, B, drift, np.zeros((7, 2)),
                                 np.zeros(2), np.zeros(2), bounds)
    assert out.success
    np.testing.assert_allclose(out.command, [-.3, 1.5], atol=1e-7)
    assert np.all(abs(out.inputs) <= bounds)
    # No finite command in this box can cancel the affine drift, and that
    # residual must remain in the predicted state rather than disappear.
    np.testing.assert_allclose(out.states[0], [.5, -.5], atol=1e-7)
    assert out.diagnostics['input_bound_violation'] == 0.


def test_original_goal_is_preserved_and_affine_state_prediction_matches_rollout():
    A = np.array([[1., .1], [0., 1.]])
    B = np.array([[.005], [.1]])
    drift = np.array([.01, -.02])
    target = np.tile([.4, 0.], (12, 1))
    target_copy = target.copy()
    out = AffineMPC(settings(2, horizon_steps=12)).solve(
        A, B, drift, target, np.array([.3]), np.array([.1]), np.array([1.2]))
    assert out.success
    np.testing.assert_array_equal(target, target_copy)
    state, states = np.zeros(2), []
    for u in out.inputs:
        state = A@state+B@(u-np.array([.3]))+drift
        states.append(state.copy())
    np.testing.assert_allclose(out.states, states, atol=1e-13)
    assert out.states[-1, 0] > .1


def test_delayed_control_authority_can_move_before_a_one_step_position_cost_can():
    # Position can change only via velocity at a later step. A finite horizon
    # values the delayed improvement, whereas B's position row is zero.
    A = np.array([[1., .1], [0., 1.]])
    B = np.array([[0.], [.1]])
    conf = settings(2, horizon_steps=15, input_weight=.003, change_weight=.003)
    conf = replace(conf, state_weights=(20., .05))
    out = AffineMPC(conf).solve(A, B, np.zeros(2), np.tile([.2, 0.], (15, 1)),
                               np.zeros(1), np.zeros(1), np.ones(1))
    one = AffineMPC(replace(conf, horizon_steps=1)).solve(
        A, B, np.zeros(2), np.array([[.2, 0.]]), np.zeros(1), np.zeros(1), np.ones(1))
    assert out.success and one.success
    assert out.command[0] > .5
    assert abs(one.command[0]) < 1e-9
    assert abs(out.states[-1, 0]-.2) < .02


def test_balancing_linear_model_recovers_without_freezing_goal_progress():
    # Linearized cart-pole dynamics, with state [position, velocity, tilt,
    # tilt_rate], independently discretized through a matrix exponential.
    from scipy.linalg import expm
    mass_cart, mass_pole, length, gravity, dt = 1., .2, .5, 9.81, .05
    continuous = np.array([[0., 1., 0., 0.], [0., 0., -mass_pole*gravity/mass_cart, 0.],
                           [0., 0., 0., 1.], [0., 0., (mass_cart+mass_pole)*gravity/(mass_cart*length), 0.]])
    input_map = np.array([[0.], [1/mass_cart], [0.], [-1/(mass_cart*length)]])
    augmented = np.zeros((5, 5))
    augmented[:4, :4], augmented[:4, 4:] = continuous, input_map
    discrete = expm(augmented*dt)
    A, B = discrete[:4, :4], discrete[:4, 4:]
    Q, R, bounds = np.diag([8., 1., 80., 3.]), np.array([[.1]]), np.array([10.])
    terminal = solve_discrete_are(A, B, Q, R)
    conf = settings(4, horizon_steps=20, step_s=dt, input_weight=10., change_weight=.02)
    conf = replace(conf, state_weights=(8., 1., 80., 3.))
    control = AffineMPC(conf)
    initial = np.array([0., 0., .03, 0.])
    goal = np.array([.15, 0., 0., 0.])
    state, previous = initial.copy(), np.zeros(1)
    tilt_peak = abs(state[2])
    for _ in range(70):
        result = control.solve(A, B, A@state-state,
                               np.tile(goal-state, (conf.horizon_steps, 1)),
                               np.zeros(1), previous, bounds, terminal_weight=terminal)
        assert result.success
        state = A@state+B@result.command
        previous = result.command
        tilt_peak = max(tilt_peak, abs(state[2]))
    assert abs(state[0]-goal[0]) < .02
    assert abs(state[2]) < .003
    assert tilt_peak < .07
    uncontrolled = np.linalg.matrix_power(A, 70)@initial
    assert abs(uncontrolled[2]) > 10.


def test_updated_model_and_changed_input_bounds_reuse_solver_without_old_constraints():
    conf = settings(horizon_steps=8, change_weight=0.)
    controller = AffineMPC(conf)
    first = controller.solve(np.array([[1.]]), np.array([[.2]]), np.zeros(1),
                             np.ones((8, 1)), np.zeros(1), np.zeros(1), np.array([2.]))
    second = controller.solve(np.array([[1.1]]), np.array([[.15]]), np.array([.02]),
                              -np.ones((8, 1)), np.array([.2]), first.command, np.array([.1]))
    assert first.success and second.success
    assert second.diagnostics['warm_start_used']
    assert np.all(abs(second.inputs) <= .1)
    assert second.command[0] < 0
    reference = AffineMPC(conf).solve(np.array([[1.1]]), np.array([[.15]]), np.array([.02]),
                                     -np.ones((8, 1)), np.array([.2]), first.command, np.array([.1]))
    np.testing.assert_allclose(second.inputs, reference.inputs, atol=2e-6)


def test_unsolved_qp_returns_no_command_or_plan_for_explicit_fallback():
    conf = settings(horizon_steps=30, max_iter=1, eps_abs=1e-12, eps_rel=1e-12,
                    polishing=False)
    result = AffineMPC(conf).solve(np.array([[1.2]]), np.array([[.01]]), np.array([.7]),
                                  np.full((30, 1), -20.), np.array([.2]), np.array([.4]), np.ones(1))
    assert not result.success
    assert result.command is None and result.states is None and result.inputs is None
    assert result.diagnostics['status_val'] != 1
    assert not result.diagnostics['fallback_used']


def test_cached_problem_accepts_a_previously_unavailable_drive_channel():
    # Actual wheel configurations change which input columns have authority.
    # New nonzero Hessian entries must survive the cached sparse-QP update.
    controller = AffineMPC(settings(horizon_steps=7, change_weight=0.))
    A, drift, target = np.ones((1, 1)), np.zeros(1), np.ones((7, 1))
    bounds = np.array([.6, .3])
    first = controller.solve(A, np.array([[1., 0.]]), drift, target,
                             np.zeros(2), np.zeros(2), bounds)
    second = controller.solve(A, np.array([[0., 1.]]), drift, target,
                              np.zeros(2), first.command, bounds)
    assert first.success and second.success
    assert first.command[0] > .1 and abs(first.command[1]) < 1e-7
    assert second.command[1] > .1 and abs(second.command[0]) < 1e-7
    fresh = AffineMPC(controller.settings).solve(A, np.array([[0., 1.]]), drift,
                                                target, np.zeros(2), first.command, bounds)
    np.testing.assert_allclose(second.inputs, fresh.inputs, atol=2e-6)


def test_external_optimality_gate_rejects_a_backend_that_keeps_stale_hessian():
    # Simulate a matrix-update failure hidden by a cached backend: it uses the
    # NEW q but OLD P, then reports its old-objective optimum as "solved".
    conf = settings(horizon_steps=1, terminal_multiplier=1., change_weight=0.,
                    input_weight=.02, regularization=0.)
    controller = AffineMPC(conf)
    first = controller.solve(np.ones((1, 1)), np.ones((1, 1)), np.zeros(1),
                             np.array([[.2]]), np.zeros(1), np.zeros(1), np.ones(1))
    assert first.success

    class StaleMatrixBackend:
        def update(self, **kwargs):
            self.q = kwargs['q'].copy()

        def warm_start(self, **kwargs):
            pass

        def solve(self, **kwargs):
            old_P = 2*(1.+.02)
            return SimpleNamespace(x=np.clip(-self.q/old_P, -1., 1.),
                info=SimpleNamespace(status='solved', status_val=1, iter=1,
                                     prim_res=0., dual_res=0., solve_time=0., obj_val=0.))

    controller._solver = StaleMatrixBackend()
    out = controller.solve(np.ones((1, 1)), np.array([[.1]]), np.zeros(1),
                           np.array([[.2]]), np.zeros(1), first.command, np.ones(1))
    assert not out.success and out.command is None and out.inputs is None
    assert out.diagnostics['backend_status'] == 'solved'
    assert out.diagnostics['status'] == 'external_box_kkt_rejected'
    assert out.diagnostics['normalized_box_kkt_residual'] > .03
    assert not out.diagnostics['external_box_kkt_passed']
    assert out.diagnostics['solver_cache_discarded']
    assert controller._solver is None


@pytest.mark.parametrize('field,value', [
    ('horizon_steps', 0), ('horizon_steps', 2.5), ('step_s', 0.), ('max_iter', 0),
    ('eps_abs', np.nan), ('state_scales', (0.,)), ('state_weights', (-1.,)*18),
    ('input_weight', -1.), ('change_weight', np.inf), ('terminal_multiplier', -1.),
    ('kkt_tolerance', 0.), ('kkt_tolerance', np.nan)])
def test_invalid_settings_are_rejected(field, value):
    with pytest.raises(ValueError):
        AffineMPCSettings(**{field: value})


@pytest.mark.parametrize('name,value', [
    ('A', np.array([[np.nan]])), ('B', np.ones((2, 1))), ('f0', np.zeros(2)),
    ('target_states', np.ones((3, 1))), ('anchor_u', np.array([np.inf])),
    ('previous_u', np.zeros(2)), ('bounds', np.zeros(1)),
    ('terminal_weight', np.array([[-1.]]))])
def test_invalid_dynamic_models_or_reference_inputs_are_rejected(name, value):
    inputs = dict(A=np.ones((1, 1)), B=np.ones((1, 1)), f0=np.zeros(1),
                  target_states=np.zeros((4, 1)), anchor_u=np.zeros(1),
                  previous_u=np.zeros(1), bounds=np.ones(1))
    inputs[name] = value
    with pytest.raises(ValueError):
        AffineMPC(settings(horizon_steps=4)).solve(**inputs)


def test_nonfinite_finite_horizon_prediction_has_no_actuable_solution():
    conf = settings(horizon_steps=20)
    out = AffineMPC(conf).solve(np.array([[1e100]]), np.ones((1, 1)), np.ones(1),
                               np.zeros((20, 1)), np.zeros(1), np.zeros(1), np.ones(1))
    assert not out.success and out.command is None
    assert out.diagnostics['status'] == 'nonfinite_condensed_prediction'


def test_robot_defaults_do_not_penalize_material_configuration_or_heading_angle():
    config = AffineMPCSettings()
    assert len(config.state_scales) == len(config.state_weights) == 18
    np.testing.assert_array_equal(config.state_weights[5:9], 0.)
    assert config.horizon_steps*config.step_s == pytest.approx(.4)


@pytest.mark.parametrize('with_terminal_tail', [False, True])
def test_stage_varying_model_matches_independent_uncondensed_kkt_optimum(with_terminal_tail):
    # Independent variables here are ALL predicted states and physical inputs,
    # with explicit dynamics equality constraints. No condensed convolution.
    horizon, nx, nu = 4, 2, 2
    A = np.array([[[1., .10], [0., .91]], [[.97, .03], [-.04, .98]],
                  [[1.02, -.05], [.02, .95]], [[.94, .08], [0., 1.01]]])
    B = np.array([[[.08, 0.], [.03, .12]], [[.04, -.02], [.01, .15]],
                  [[0., .06], [.12, .02]], [[.07, .01], [-.02, .10]]])
    f0 = np.array([[.02, -.01], [.01, .03], [-.02, .01], [.03, 0.]])
    anchors = np.array([[.1, -.1], [.05, .08], [-.03, .02], [.04, -.05]])
    target = np.array([[.02, .01], [.04, -.02], [.06, 0.], [.1, -.04]])
    previous, bounds = np.array([.05, -.02]), np.array([2., 3.])
    input_target = (np.array([[.1, -.3], [-.2, .1], [.3, .2], [-.1, -.2]])
                    if with_terminal_tail else np.zeros((horizon, nu)))
    config = settings(nx, horizon_steps=horizon, input_weight=(.2, .4),
                      change_weight=(.3, .1), regularization=0.)
    config = replace(config, state_scales=(.5, 2.), state_weights=(1.3, .7))
    terminal = np.array([[2., .2], [.2, .8]])
    Q = np.diag(np.array(config.state_weights)/np.array(config.state_scales)**2)
    R = np.diag(np.array(config.input_weight)/bounds**2)
    D = np.diag(np.array(config.change_weight)/bounds**2)
    count = horizon*(nx+nu)
    hessian, linear = np.zeros((count, count)), np.zeros(count)
    equalities, rhs = np.zeros((horizon*nx, count)), np.zeros(horizon*nx)
    u_offset = horizon*nx
    for stage in range(horizon):
        x_slice = slice(stage*nx, (stage+1)*nx)
        u_slice = slice(u_offset+stage*nu, u_offset+(stage+1)*nu)
        weight = terminal if stage == horizon-1 else Q
        hessian[x_slice, x_slice] += 2*weight
        linear[x_slice] -= 2*weight@target[stage]
        hessian[u_slice, u_slice] += 2*(R+D)
        linear[u_slice] -= 2*R@input_target[stage]
        if stage == 0:
            linear[u_slice] -= 2*D@previous
        else:
            u_previous = slice(u_offset+(stage-1)*nu, u_offset+stage*nu)
            hessian[u_previous, u_previous] += 2*D
            hessian[u_slice, u_previous] -= 2*D
            hessian[u_previous, u_slice] -= 2*D
        equalities[x_slice, x_slice] = np.eye(nx)
        equalities[x_slice, u_slice] = -B[stage]
        if stage:
            equalities[x_slice, (stage-1)*nx:stage*nx] = -A[stage]
        rhs[x_slice] = f0[stage]-B[stage]@anchors[stage]
    tail = None
    if with_terminal_tail:
        root = np.array([[1., .2, -.3, .1], [.3, 1., .1, -.2],
                         [.2, -.1, 1., .4], [-.2, .3, .2, 1.]])
        tail = root.T@root
        selector = np.zeros((nx+nu, count))
        selector[:nx, (horizon-1)*nx:horizon*nx] = np.eye(nx)
        selector[nx:, -nu:] = np.diag(1/bounds)
        reference = np.r_[target[-1], input_target[-1]/bounds]
        hessian += 2*selector.T@tail@selector
        linear -= 2*selector.T@tail@reference
    kkt = np.block([[hessian, equalities.T],
                    [equalities, np.zeros((horizon*nx, horizon*nx))]])
    expected = np.linalg.solve(kkt, np.r_[-linear, rhs])[:count]
    out = AffineMPC(config).solve(A, B, f0, target, anchors, previous, bounds,
                                 terminal_weight=terminal,
                                 target_inputs=input_target if with_terminal_tail else None,
                                 terminal_tail_weight=tail)
    assert out.success
    assert out.diagnostics['model_kind'] == 'linear_time_varying'
    assert np.max(abs(expected[u_offset:].reshape(horizon, nu))/bounds) < .8
    np.testing.assert_allclose(out.inputs, expected[u_offset:].reshape(horizon, nu), atol=2e-6)
    np.testing.assert_allclose(out.states, expected[:u_offset].reshape(horizon, nx), atol=2e-6)
    assert not out.diagnostics['hessian_psd_projected']
    assert not out.diagnostics['objective_rescaled']
    assert out.diagnostics['hessian_min_eigenvalue'] > 0.
    if with_terminal_tail:
        error = np.r_[out.states[-1]-target[-1], (out.inputs[-1]-input_target[-1])/bounds]
        assert out.diagnostics['terminal_tail_cost'] == pytest.approx(error@tail@error)


def test_stage_varying_authority_anticipates_losing_an_input_direction():
    # Only the first input can create motion before the authority handoff.
    # A constant copy of the final B would miss that early opportunity.
    horizon = 8
    A = np.tile(np.array([[1., .1], [0., 1.]]), (horizon, 1, 1))
    B = np.zeros((horizon, 2, 2))
    B[:3, 1, 0] = .2
    B[3:, 1, 1] = .03
    conf = settings(2, horizon_steps=horizon, input_weight=.001, change_weight=0.)
    conf = replace(conf, state_weights=(20., .05))
    target = np.tile([.12, 0.], (horizon, 1))
    out = AffineMPC(conf).solve(A, B, np.zeros((horizon, 2)), target,
                               np.zeros((horizon, 2)), np.zeros(2), np.ones(2))
    assert out.success and out.command[0] > .5
    np.testing.assert_allclose(out.inputs[:3, 1], 0., atol=1e-7)
    np.testing.assert_allclose(out.inputs[3:, 0], 0., atol=1e-7)
    assert abs(out.states[-1, 0]-.12) < .025
    current = np.zeros(2)
    replay = []
    for stage, command in enumerate(out.inputs):
        current = A[stage]@current+B[stage]@command
        replay.append(current.copy())
    np.testing.assert_allclose(out.states, replay, atol=1e-13)


def test_constant_arrays_broadcast_with_varying_drift_and_input_anchor():
    horizon = 5
    A, B = np.array([[1.03]]), np.array([[.4]])
    f0 = np.array([[.03], [-.01], [.02], [0.], [-.02]])
    anchor = np.array([[.1], [.3], [-.2], [.05], [.0]])
    conf = settings(horizon_steps=horizon, change_weight=.03)
    out = AffineMPC(conf).solve(A, B, f0, np.zeros((horizon, 1)), anchor,
                               np.array([.2]), np.ones(1))
    explicit = AffineMPC(conf).solve(np.tile(A, (horizon, 1, 1)),
                                    np.tile(B, (horizon, 1, 1)), f0,
                                    np.zeros((horizon, 1)), anchor, np.array([.2]), np.ones(1))
    assert out.success and explicit.success
    np.testing.assert_array_equal(out.inputs, explicit.inputs)
    current = np.zeros(1)
    for stage, command in enumerate(out.inputs):
        current = A@current+B@(command-anchor[stage])+f0[stage]
        np.testing.assert_allclose(out.states[stage], current, atol=1e-13)


@pytest.mark.parametrize('field,value', [
    ('A', np.ones((3, 2, 2))), ('B', np.ones((3, 2, 1))),
    ('f0', np.ones((3, 2))), ('anchor_u', np.ones((3, 1))),
    ('A', np.ones((4, 2, 3))), ('B', np.ones((4, 3, 1))),
    ('f0', np.ones((4, 2, 1))), ('anchor_u', np.ones((4, 1, 1)))])
def test_varying_model_dimensions_must_match_horizon_and_fixed_chart(field, value):
    inputs = dict(A=np.tile(np.eye(2), (4, 1, 1)), B=np.ones((4, 2, 1)),
                  f0=np.zeros((4, 2)), target_states=np.zeros((4, 2)),
                  anchor_u=np.zeros((4, 1)), previous_u=np.zeros(1), bounds=np.ones(1))
    inputs[field] = value
    with pytest.raises(ValueError):
        AffineMPC(settings(2, horizon_steps=4)).solve(**inputs)
