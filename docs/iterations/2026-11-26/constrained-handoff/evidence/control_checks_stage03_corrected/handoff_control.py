"""Bounded differential command allocation after native balance feedback.

The native controller remains the policy. This optional layer preserves its
clipped balance response in a supplied local sensitivity model before spending
remaining authority on travel, yaw and unloaded-rotor momentum. Hard bounds
apply to the TOTAL applied command. A local response match is not a stability,
contact/friction-feasibility, or trajectory-completion guarantee.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import product
import warnings

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, lsq_linear, minimize


@dataclass(frozen=True)
class HandoffSettings:
    horizon_s: float = .02
    slew_limits: tuple = (60., 60., 60., 6., 6.)
    travel_scale_m_s: float = .1
    yaw_scale_rad_s: float = 1.
    rotor_reference_limit_rad_s: float = 20.
    travel_weight: float = 1.
    yaw_weight: float = 1.
    momentum_weight: float = 1.
    native_weight: float = .2
    change_weight: float = .2
    effort_weight: float = .01
    readiness_multiplier: float = 4.
    unloaded_fraction: float = .05
    equality_tolerance: float = 1e-8
    rank_relative_cutoff: float = 1e-10

    def __post_init__(self):
        rates = np.asarray(self.slew_limits, float)
        if rates.shape != (5,) or not np.all(np.isfinite(rates) & (rates > 0)):
            raise ValueError('Five positive finite final-command slew limits required')
        object.__setattr__(self, 'slew_limits', tuple(float(x) for x in rates))
        for name in ('horizon_s', 'travel_scale_m_s', 'yaw_scale_rad_s',
                     'rotor_reference_limit_rad_s', 'equality_tolerance'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive and finite')
        for name in ('travel_weight', 'yaw_weight', 'momentum_weight', 'native_weight',
                     'change_weight', 'effort_weight', 'readiness_multiplier'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if self.native_weight + self.effort_weight <= 0:
            raise ValueError('A positive native or effort term must regularize all five inputs')
        if not 0 < self.unloaded_fraction <= 1 or not 0 < self.rank_relative_cutoff < 1:
            raise ValueError('Invalid unloaded fraction or numerical rank cutoff')


@dataclass(frozen=True)
class HandoffDecision:
    command: np.ndarray
    diagnostics: dict


def _array(value, shape, name):
    result = np.asarray(value, float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{name} must be finite with shape {shape}')
    return result


def _enumerated_equality_lsq(A, b, E, f, lower, upper, seed, tolerance):
    """Small convex QP recovery: enumerate bound faces, never an unsafe output.

    Five inputs give at most 243 faces. The optimum appears as an equality-
    constrained stationary point on one of them. This is used only when the
    regular solver does not return a verified solution.
    """
    H, g = A.T @ A, -A.T @ b
    best, best_cost = seed.copy(), float(np.linalg.norm(A @ seed-b)**2)
    feasible = 0
    for face in product((0, -1, 1), repeat=5):
        fixed = np.flatnonzero(np.asarray(face) != 0)
        free = np.flatnonzero(np.asarray(face) == 0)
        x = np.zeros(5)
        if len(fixed):
            x[fixed] = [lower[i] if face[i] == -1 else upper[i] for i in fixed]
        rhs = f-E[:, fixed] @ x[fixed]
        if len(free):
            ef = E[:, free]
            kkt = np.block([[H[np.ix_(free, free)], ef.T],
                            [ef, np.zeros((len(E), len(E)))]])
            target = np.r_[-g[free]-H[np.ix_(free, fixed)] @ x[fixed], rhs]
            solution = np.linalg.lstsq(kkt, target, rcond=1e-12)[0]
            x[free] = solution[:len(free)]
        if (np.max(np.abs(E @ x-f), initial=0.) > tolerance or
                np.any(x < lower-tolerance) or np.any(x > upper+tolerance)):
            continue
        x = np.clip(x, lower, upper)
        if np.max(np.abs(E @ x-f), initial=0.) > tolerance:
            continue
        feasible += 1
        cost = float(np.linalg.norm(A @ x-b)**2)
        if cost < best_cost:
            best, best_cost = x, cost
    return best, feasible


class BoundedHandoffAllocator:
    """Lexicographic projection with a local differential, not forecast, model.

    Stage one minimizes body transverse angular-response disagreement with
    the BOX-CLIPPED native total command inside amplitude and slew bounds.
    Stage two fixes that attained response and minimizes secondary costs.
    The damping proxy is v_current + G (u-u_native_clipped), NOT a prediction
    that the unmodified native controller holds the current velocity constant.
    Actual previous applied input, supplied on every call, centers the slew box;
    there is no optimizer-owned command history or startup slew exemption.
    """

    def __init__(self, settings=None):
        self.settings = settings or HandoffSettings()

    def solve(self, native_requested, input_bounds, previous_applied, dt_s,
              qvel, response, fork_rotation_world, shell_axis_world,
              shell_axial_inertia, support_loads_N, reference_shell_rates,
              *, readiness=None, preparation_shell_rates=None):
        s = self.settings
        native = _array(native_requested, (5,), 'Native total request')
        caps = _array(input_bounds, (5,), 'Input bounds')
        previous = _array(previous_applied, (5,), 'Actual previous input')
        v = _array(qvel, (9,), 'Measured generalized velocity')
        G = _array(response, (9, 5), 'Local differential response')
        R = _array(fork_rotation_world, (3, 3), 'Fork rotation')
        axis = _array(shell_axis_world, (3,), 'Shell axis')
        inertia = _array(shell_axial_inertia, (2,), 'Shell axial inertia')
        loads = _array(support_loads_N, (4,), 'Estimated support loads')
        rates = _array(reference_shell_rates, (2,), 'Native shell-rate reference')
        ready = np.zeros(5) if readiness is None else _array(readiness, (5,), 'Readiness')
        prep = rates.copy() if preparation_shell_rates is None else _array(
            preparation_shell_rates, (2,), 'Preparation shell-rate reference')
        if (np.any(caps <= 0) or np.any(inertia <= 0) or np.any(loads < 0)
                or np.any(ready < 0) or np.any(ready > 1)
                or not np.isfinite(dt_s) or dt_s <= 0):
            raise ValueError('Positive bounds/inertia/dt, nonnegative loads and readiness in [0,1] required')
        if (np.max(abs(R.T @ R-np.eye(3))) > 1e-6 or
                abs(np.linalg.det(R)-1.) > 1e-6 or abs(np.linalg.norm(axis)-1.) > 1e-6):
            raise ValueError('Expected a proper rotation and unit shell axis')
        if np.any(abs(previous) > caps+1e-10):
            raise ValueError('Actual previous input lies outside current actuator bounds')
        previous = np.clip(previous, -caps, caps)
        step = np.asarray(s.slew_limits)*float(dt_s)
        lower_u, upper_u = np.maximum(-caps, previous-step), np.minimum(caps, previous+step)
        lower, upper = lower_u/caps, upper_u/caps
        nominal = np.clip(native, -caps, caps)
        zn, zp = nominal/caps, previous/caps
        A_balance = G[3:5]*caps
        singular = np.linalg.svd(A_balance, compute_uv=False)
        rank = int(np.sum(singular > max(singular[0]*s.rank_relative_cutoff, 1e-12)))
        if rank:
            primary = lsq_linear(A_balance, A_balance @ zn, bounds=(lower, upper),
                                 method='bvls', tol=1e-12, max_iter=100)
            if not primary.success or not np.isfinite(primary.x).all():
                raise RuntimeError(f'Primary balance allocation failed: {primary.message}')
            z1 = np.clip(primary.x, lower, upper)
            _, _, vt = np.linalg.svd(A_balance, full_matrices=False)
            E = vt[:rank]
        else:
            z1 = np.clip(zn, lower, upper)
            E = np.zeros((0, 5))
        f = E @ z1
        rows, targets = [], []
        def add(A, b, weight):
            if weight > 0:
                rows.append(np.sqrt(weight)*A)
                targets.append(np.sqrt(weight)*b)
        travel = G[:2]*caps/s.travel_scale_m_s
        add(travel, travel @ zn, s.travel_weight)
        yaw = (R[2] @ G[3:6])*caps/s.yaw_scale_rad_s
        current_yaw = float(R[2] @ v[3:6])
        total = float(loads.sum())
        # With no estimated support, retain native fade/continuity rather than
        # adding a new aerial yaw/momentum policy based on an absent contact.
        auxiliary_enabled = total > 1e-9
        add(yaw[None, :], np.array([yaw @ zn-current_yaw/s.yaw_scale_rad_s]),
            s.yaw_weight if auxiliary_enabled else 0.)
        unloaded = (np.clip(1.-loads[:2]/(s.unloaded_fraction*total), 0., 1.)
                    if total > 1e-9 else np.zeros(2))
        auxiliary_target = np.clip((1.-ready[1:3])*rates+ready[1:3]*prep,
                                   -s.rotor_reference_limit_rad_s, s.rotor_reference_limit_rad_s)
        # Relative hinge momentum projected on world vertical; normalization
        # uses I_axis*20, NOT I_axis*axis_z*20. Horizontal rotors retain their
        # useful balance reaction without a spurious vertical-momentum cost.
        momentum = axis[2]*G[7:9]*caps/s.rotor_reference_limit_rad_s
        momentum_error = axis[2]*(v[7:9]-auxiliary_target)/s.rotor_reference_limit_rad_s
        w = np.sqrt(unloaded)[:, None]
        add(w*momentum, np.sqrt(unloaded)*(momentum @ zn-momentum_error), s.momentum_weight)
        native_rows = np.diag(np.sqrt(1.+s.readiness_multiplier*ready))
        add(native_rows, native_rows @ zn, s.native_weight)
        # z already uses physical input caps. Normalizing this SOFT cost by
        # the tiny allowed step would penalize belt preparation a second time
        # and prevent reaching the existing preview target. Hard slew limits
        # remain in the actual-dt box independently of this cost.
        change = np.eye(5)
        add(change, change @ zp, s.change_weight)
        add(np.eye(5), np.zeros(5), s.effort_weight)
        A, b = np.vstack(rows), np.concatenate(targets)
        if not rank:
            secondary = lsq_linear(A, b, bounds=(lower, upper), method='bvls',
                                   tol=1e-12, max_iter=100)
            candidate = secondary.x
            messages = []
            success = bool(secondary.success)
        else:
            H, g = A.T @ A, -A.T @ b
            objective = lambda z: float(np.linalg.norm(A @ z-b)**2)
            gradient = lambda z: 2.*(H @ z+g)
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter('always')
                secondary = minimize(objective, z1, jac=gradient, method='SLSQP',
                    bounds=Bounds(lower, upper), constraints=LinearConstraint(E, f, f),
                    options={'ftol': 1e-11, 'maxiter': 80})
            messages = [str(warning.message) for warning in caught]
            candidate, success = secondary.x, bool(secondary.success)
        feasible = (np.isfinite(candidate).all() and
                    np.all(candidate >= lower-s.equality_tolerance) and
                    np.all(candidate <= upper+s.equality_tolerance) and
                    np.max(abs(E @ candidate-f), initial=0.) <= s.equality_tolerance)
        if feasible:
            candidate = np.clip(candidate, lower, upper)
            feasible = (np.max(abs(E @ candidate-f), initial=0.) <= s.equality_tolerance and
                        np.max(abs(A_balance @ (candidate-z1)), initial=0.) <= s.equality_tolerance)
        recovery = not (success and feasible)
        enumerated_faces = 0
        if recovery:
            candidate, enumerated_faces = _enumerated_equality_lsq(
                A, b, E, f, lower, upper, z1, s.equality_tolerance)
        candidate = np.clip(candidate, lower, upper)
        primary_restore = False
        if (np.max(abs(E @ candidate-f), initial=0.) > s.equality_tolerance or
                np.max(abs(A_balance @ (candidate-z1)), initial=0.) > s.equality_tolerance):
            candidate = z1.copy()
            primary_restore = True
        command = np.clip(caps*candidate, lower_u, upper_u)
        delta = command-nominal
        proxy = v+G @ delta
        diagnostics = dict(
            method='lexicographic_total_command_differential_projection',
            model_scope='Local frozen-contact differential proxy; not absolute contact/motion forecast or stability/friction guarantee',
            native_requested=native.copy(), native_clipped=nominal.copy(),
            previous_applied=previous.copy(), lower_command=lower_u, upper_command=upper_u,
            native_amplitude_clipping=np.abs(native) > caps+1e-12,
            native_slew_violation=(nominal < lower_u-1e-12) | (nominal > upper_u+1e-12),
            active_amplitude=np.isclose(abs(command), caps, atol=1e-9, rtol=0.),
            active_slew=np.isclose(abs(command-previous), step, atol=1e-9, rtol=0.),
            balance_rank=rank, balance_singular_values=singular,
            first_stage_balance_delta=A_balance @ (z1-zn),
            secondary_balance_delta=A_balance @ (command/caps-z1),
            equality_residual_max=float(np.max(abs(E @ (command/caps)-f), initial=0.)),
            secondary_recovery_used=recovery, recovery_feasible_faces=enumerated_faces,
            secondary_primary_restore_used=primary_restore,
            solver_message=str(secondary.message), solver_warnings=messages,
            secondary_cost=float(np.linalg.norm(A @ (command/caps)-b)**2),
            primary_point_secondary_cost=float(np.linalg.norm(A @ z1-b)**2),
            current_world_yaw_rate_rad_s=current_yaw,
            proxy_world_yaw_rate_rad_s=float(R[2] @ proxy[3:6]),
            relative_rotor_vertical_momentum_Nms=inertia*axis[2]*(v[7:9]-auxiliary_target),
            proxy_relative_rotor_vertical_momentum_Nms=inertia*axis[2]*(proxy[7:9]-auxiliary_target),
            unloaded_weight=unloaded, readiness=ready.copy(), auxiliary_shell_target_rad_s=auxiliary_target,
            auxiliary_damping_enabled=auxiliary_enabled,
            travel_response_delta_m_s=G[:2] @ delta, input_delta=delta,
            horizon_s=s.horizon_s, actual_control_interval_s=float(dt_s))
        command.setflags(write=False)
        return HandoffDecision(command, diagnostics)
