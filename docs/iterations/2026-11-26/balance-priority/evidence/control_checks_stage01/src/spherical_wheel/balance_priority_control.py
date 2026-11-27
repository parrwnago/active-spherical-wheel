"""Opt-in I58 allocation: preserve native balance before soft smoothing.

Hard constraints apply to the total command amplitude and the supplied local
native transverse response. The original rate thresholds are diagnostics only.
This local response equality is not a stability or contact-feasibility proof.
"""
from __future__ import annotations

import warnings

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, lsq_linear, minimize

from .handoff_control import HandoffDecision, HandoffSettings, _array


class BalancePriorityAllocator:
    """I56A secondary objective on an amplitude-only native-balance equality.

    ``G[3:5] @ (u-clip(native)) == 0`` preserves the native local body-x/y
    angular response. The clipped native command is always a feasible seed
    and is the verified fallback after any failed/unverified solve. Remaining
    authority minimizes exactly I56A's travel, readiness/native, soft change
    and effort terms, with the same input-cap normalization. Yaw and momentum
    weights must be zero. The supplied actual previous command centers only
    the soft change term; no command history is owned by this allocator.
    """

    def __init__(self, settings=None):
        self.settings = settings or HandoffSettings(yaw_weight=0., momentum_weight=0.)
        if self.settings.yaw_weight != 0 or self.settings.momentum_weight != 0:
            raise ValueError('Balance-priority allocation requires zero yaw and momentum weights')

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
        if (np.max(abs(R.T@R-np.eye(3))) > 1e-6
                or abs(np.linalg.det(R)-1.) > 1e-6
                or abs(np.linalg.norm(axis)-1.) > 1e-6):
            raise ValueError('Expected a proper rotation and unit shell axis')
        if np.any(abs(previous) > caps+1e-10):
            raise ValueError('Actual previous input lies outside current actuator bounds')
        previous = np.clip(previous, -caps, caps)
        nominal = np.clip(native, -caps, caps)
        zn, zp = nominal/caps, previous/caps
        # Row scaling changes only numerical representation of the equality.
        # Validate the original physical G equality again after final clipping.
        A_balance = G[3:5]*caps
        row_scale = np.max(abs(A_balance), axis=1)
        normalized = np.divide(A_balance, row_scale[:, None],
                               out=np.zeros_like(A_balance), where=row_scale[:, None] > 0)
        _, singular, vt = np.linalg.svd(normalized, full_matrices=False)
        rank = int(np.sum(singular > singular[0]*s.rank_relative_cutoff)) if singular[0] > 0 else 0
        E = vt[:rank]
        f = E@zn
        lower, upper = -np.ones(5), np.ones(5)
        rows, targets = [], []

        def add(matrix, target, weight):
            if weight > 0:
                rows.append(np.sqrt(weight)*matrix)
                targets.append(np.sqrt(weight)*target)

        travel = G[:2]*caps/s.travel_scale_m_s
        add(travel, travel@zn, s.travel_weight)
        native_rows = np.diag(np.sqrt(1.+s.readiness_multiplier*ready))
        add(native_rows, native_rows@zn, s.native_weight)
        add(np.eye(5), zp, s.change_weight)
        add(np.eye(5), np.zeros(5), s.effort_weight)
        A, b = np.vstack(rows), np.concatenate(targets)
        if not np.isfinite(A).all() or not np.isfinite(b).all():
            raise ValueError('Finite normalized secondary objective required')
        nominal_cost = float(np.linalg.norm(A@zn-b)**2)
        candidate, success, messages = zn.copy(), False, []
        solver_message = ''
        try:
            if rank:
                H, g = A.T@A, -A.T@b
                with warnings.catch_warnings(record=True) as caught:
                    warnings.simplefilter('always')
                    secondary = minimize(lambda z: float(np.linalg.norm(A@z-b)**2),
                        zn, jac=lambda z: 2.*(H@z+g), method='SLSQP',
                        bounds=Bounds(lower, upper), constraints=LinearConstraint(E, f, f),
                        options={'ftol': 1e-11, 'maxiter': 80})
                messages = [str(item.message) for item in caught]
            else:
                secondary = lsq_linear(A, b, bounds=(lower, upper),
                    method='bvls', tol=1e-12, max_iter=100)
            candidate = np.asarray(secondary.x, dtype=float)
            success, solver_message = bool(secondary.success), str(secondary.message)
        except Exception as error:
            solver_message = f'{type(error).__name__}: {error}'

        def verified(z):
            return (z.shape == (5,) and np.isfinite(z).all()
                and np.all(z >= lower-s.equality_tolerance)
                and np.all(z <= upper+s.equality_tolerance)
                and np.max(abs(E@z-f), initial=0.) <= s.equality_tolerance
                and np.max(abs(A_balance@(z-zn)), initial=0.) <= s.equality_tolerance)

        fallback = not (success and verified(candidate))
        if fallback:
            command = nominal.copy()
        else:
            command = np.clip(caps*np.clip(candidate, lower, upper), -caps, caps)
            if (not verified(command/caps)
                    or np.max(abs(G[3:5]@(command-nominal)), initial=0.) > s.equality_tolerance
                    or float(np.linalg.norm(A@(command/caps)-b)**2) > nominal_cost+1e-7):
                fallback = True
                command = nominal.copy()
        # Fallback uses the original clipped vector, avoiding normalization
        # round-trip errors. Verify actual final amplitude and physical equality.
        delta = command-nominal
        balance_delta = G[3:5]@delta
        if (not np.isfinite(command).all() or np.any(abs(command) > caps)
                or np.max(abs(balance_delta), initial=0.) > s.equality_tolerance):
            raise RuntimeError('Verified native-balance fallback unexpectedly failed')
        diagnostic_step = np.asarray(s.slew_limits)*float(dt_s)
        change = command-previous
        current_yaw = float(R[2]@v[3:6])
        proxy = v+G@delta
        target = np.clip((1.-ready[1:3])*rates+ready[1:3]*prep,
                         -s.rotor_reference_limit_rad_s, s.rotor_reference_limit_rad_s)
        diagnostics = dict(
            method='amplitude_bounded_native_balance_priority_soft_smoothing',
            change_normalization='input_bounds',
            model_scope='Frozen local transverse-response equality; no absolute forecast, stability or friction guarantee',
            native_requested=native.copy(), native_clipped=nominal.copy(), previous_applied=previous.copy(),
            lower_command=-caps.copy(), upper_command=caps.copy(),
            hard_slew_enabled=False, rate_threshold_scope='diagnostic_only',
            diagnostic_rate_thresholds=np.asarray(s.slew_limits).copy(),
            diagnostic_rate_threshold_exceeded=abs(change) > diagnostic_step+1e-12,
            command_change_per_s=change/float(dt_s),
            native_amplitude_clipping=abs(native) > caps+1e-12,
            native_slew_violation=abs(nominal-previous) > diagnostic_step+1e-12,
            active_amplitude=np.isclose(abs(command), caps, atol=1e-9, rtol=0.),
            active_slew=np.zeros(5, dtype=bool),
            balance_rank=rank, balance_singular_values=singular,
            balance_rank_basis='row-normalized cap-scaled equality; physical G equality verified after clipping',
            first_stage_balance_delta=np.zeros(2), secondary_balance_delta=balance_delta.copy(),
            equality_residual_max=float(np.max(abs(E@(command/caps)-f), initial=0.)),
            physical_balance_residual_max=float(np.max(abs(balance_delta), initial=0.)),
            secondary_recovery_used=fallback, recovery_feasible_faces=0,
            secondary_primary_restore_used=fallback, native_fallback_used=fallback,
            solver_message=solver_message, solver_warnings=messages,
            secondary_cost=float(np.linalg.norm(A@(command/caps)-b)**2),
            primary_point_secondary_cost=nominal_cost,
            current_world_yaw_rate_rad_s=current_yaw, proxy_world_yaw_rate_rad_s=float(R[2]@proxy[3:6]),
            relative_rotor_vertical_momentum_Nms=inertia*axis[2]*(v[7:9]-target),
            proxy_relative_rotor_vertical_momentum_Nms=inertia*axis[2]*(proxy[7:9]-target),
            readiness=ready.copy(), auxiliary_shell_target_rad_s=target,
            auxiliary_damping_enabled=False, yaw_weight=0., momentum_weight=0.,
            travel_response_delta_m_s=G[:2]@delta, input_delta=delta.copy(),
            horizon_s=s.horizon_s, actual_control_interval_s=float(dt_s))
        command.setflags(write=False)
        return HandoffDecision(command, diagnostics)
