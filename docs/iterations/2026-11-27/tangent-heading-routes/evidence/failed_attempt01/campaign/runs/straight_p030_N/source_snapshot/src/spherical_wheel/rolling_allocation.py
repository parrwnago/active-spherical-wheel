"""Bounded rolling-speed references; balance torques remain the LQR's job.

The joint limits below are declared simulation assumptions, not motor specs.
The 20 rad/s ceiling agrees with the existing incoming-shell preview ceiling.
The belt speed ceiling is the plant's ideal surface-speed command bound.
"""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear


@dataclass(frozen=True)
class AllocationSettings:
    speed_limits: tuple = (20., 20., 20., .3, .3)
    acceleration_limits: tuple = (100., 100., 100., 3., 3.)
    effort_m_s: float = .001
    change_m_s: float = .001
    weak_effort_m_s: float | None = None
    authority_m_s: float = .02

    def __post_init__(self):
        for name in ('speed_limits', 'acceleration_limits'):
            value = np.asarray(getattr(self, name), float)
            if value.shape != (5,) or not np.all(np.isfinite(value) & (value > 0)):
                raise ValueError(f'{name} must contain five finite positive values')
        for name in ('effort_m_s', 'change_m_s'):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not np.isfinite(self.authority_m_s) or self.authority_m_s <= 0:
            raise ValueError('authority_m_s must be finite and positive')
        if self.weak_effort_m_s is not None and (
                not np.isfinite(self.weak_effort_m_s) or self.weak_effort_m_s < self.effort_m_s):
            raise ValueError('Weak-direction effort must be finite and at least the regular effort')


class BoundedRollingAllocator:
    """Minimize tracking error + normalized speed cost + reference-change cost.

    For drive rates d = S z, solve
      ||J S z - b||² + effort² ||z||² + change² ||z-z_previous||²
    subject to reference speed and acceleration bounds. S retains the old
    allocator's normalization (rad/s for joints, m/s for belt surfaces).
    Rows may be square-root load weighted by the caller. This is a kinematic
    speed-demand cost, not a measurement of motor torque, power or energy.
    """
    scale = np.array([5., 5., 5., .3, .3])

    def __init__(self, settings=None):
        self.settings = settings or AllocationSettings()
        self.reset()

    def reset(self):
        self.previous = np.zeros(5)
        self.diagnostics = {}

    def _effort_matrix(self, authority_matrix, effort):
        return effort*np.eye(5)

    def hold(self):
        """Keep current references through an unloaded interval, not old contact data."""
        drive = self.previous.copy()
        self.diagnostics.update(
            allocation_drive_reference=drive.copy(),
            allocation_speed_limits=np.asarray(self.settings.speed_limits, float).copy(),
            allocation_acceleration_limits=np.asarray(self.settings.acceleration_limits, float).copy(),
            allocation_effort_weight_m_s=self.diagnostics.get('allocation_effort_weight_m_s', self.settings.effort_m_s),
            allocation_change_weight_m_s=self.settings.change_m_s,
            allocation_authority_m_s=np.nan,
            allocation_effort_blend=np.nan,
            allocation_active_bounds=np.isclose(np.abs(drive), self.settings.speed_limits,
                                                 atol=1e-8, rtol=0.),
            allocation_residual_rms_m_s=np.nan,
            allocation_velocity_error_m_s=np.full(2, np.nan))
        return drive

    def solve(self, rows, rhs, dt, authority_matrix=None):
        matrix = np.asarray(rows, float)
        if matrix.shape == (0,):
            matrix = matrix.reshape(0, 5)
        if matrix.ndim != 2 or matrix.shape[1] != 5:
            raise ValueError('Rolling matrix must have five columns')
        target = np.asarray(rhs, float)
        if target.shape != (len(matrix),) or not np.isfinite(matrix).all() or not np.isfinite(target).all():
            raise ValueError('Rolling matrix and target must be finite and have matching rows')
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Allocation interval must be finite and positive')
        settings = self.settings
        authority_matrix = matrix if authority_matrix is None else np.asarray(authority_matrix, float)
        if authority_matrix.ndim != 2 or authority_matrix.shape[1] != 5 or not np.isfinite(authority_matrix).all():
            raise ValueError('Authority matrix must be finite with five columns')
        singular = np.linalg.svd(authority_matrix*self.scale, compute_uv=False)
        authority = float(singular[1]) if len(singular) >= 2 else 0.
        blend = settings.authority_m_s**2/(authority**2+settings.authority_m_s**2)
        effort = settings.effort_m_s
        if settings.weak_effort_m_s is not None:
            effort += blend*(settings.weak_effort_m_s-effort)
        limits = np.asarray(settings.speed_limits)
        acceleration = np.asarray(settings.acceleration_limits)
        lower = np.maximum(-limits, self.previous-dt*acceleration)
        upper = np.minimum(limits, self.previous+dt*acceleration)
        previous = self.previous/self.scale
        augmented = np.vstack((matrix*self.scale, self._effort_matrix(authority_matrix, effort),
                               settings.change_m_s*np.eye(5)))
        desired = np.r_[target, np.zeros(5), settings.change_m_s*previous]
        solution = lsq_linear(augmented, desired, bounds=(lower/self.scale, upper/self.scale),
                              method='bvls', tol=1e-10, max_iter=100)
        if not solution.success:
            raise RuntimeError(f'Rolling allocation failed: {solution.message}')
        drive = np.clip(solution.x*self.scale, lower, upper)
        self.diagnostics = {
            'allocation_drive_reference': drive.copy(),
            'allocation_speed_limits': limits.copy(),
            'allocation_acceleration_limits': acceleration.copy(),
            'allocation_active_bounds': (np.isclose(drive, lower, atol=1e-8, rtol=0.) |
                                          np.isclose(drive, upper, atol=1e-8, rtol=0.)),
            'allocation_effort_weight_m_s': effort,
            'allocation_change_weight_m_s': settings.change_m_s,
            'allocation_authority_m_s': authority,
            'allocation_effort_blend': blend if settings.weak_effort_m_s is not None else 0.,
            'allocation_residual_rms_m_s': float(np.linalg.norm(matrix@drive-target)),
        }
        self.previous = drive.copy()
        return drive


class FeasibleRollingAllocator(BoundedRollingAllocator):
    """Directional demand penalty and a body reference matching allocated rates.

    The accepted velocity is exact for the supplied single-contact kinematics.
    With several records it is their load-weighted mean, not proof of no slip
    at every contact or of dynamically achievable motion.
    """

    def __init__(self, settings=None):
        super().__init__(settings or AllocationSettings(weak_effort_m_s=.005))

    def _profile(self, authority_matrix):
        authority_matrix = np.asarray(authority_matrix, float)
        if authority_matrix.shape != (2, 5) or not np.isfinite(authority_matrix).all():
            raise ValueError('Feasible allocation authority must be finite and 2 by 5')
        left, singular, right = np.linalg.svd(authority_matrix*self.scale, full_matrices=True)
        settings = self.settings
        weak = settings.weak_effort_m_s or settings.effort_m_s
        modal = settings.effort_m_s+(weak-settings.effort_m_s)*(
            settings.authority_m_s**2/(singular**2+settings.authority_m_s**2))
        # At rank loss the disappearing mode approaches the nullspace penalty,
        # so the quadratic penalty is independent of arbitrary nullspace bases.
        penalties = np.full(5, weak)
        penalties[:len(modal)] = modal
        factors = singular**2/(singular**2+modal**2+settings.change_m_s**2)
        return penalties[:, None]*right, modal, (left*factors)@left.T

    def velocity_response(self, authority_matrix):
        return self._profile(authority_matrix)[2]

    def _effort_matrix(self, authority_matrix, effort):
        penalty, self._modal_effort, self._velocity_response = self._profile(authority_matrix)
        return penalty

    def solve(self, rows, rhs, dt, authority_matrix=None):
        matrix = np.asarray(rows, float)
        if authority_matrix is None:
            if matrix.shape != (2, 5):
                raise ValueError('Feasible allocation needs a two-row common-translation authority matrix')
            authority_matrix = matrix
        authority_matrix = np.asarray(authority_matrix, float)
        if authority_matrix.shape != (2, 5):
            raise ValueError('Feasible allocation authority matrix must be 2 by 5')
        drive = super().solve(rows, rhs, dt, authority_matrix)
        valid = bool(len(matrix))
        self.diagnostics.update(
            allocation_accepted_velocity_m_s=-authority_matrix@drive,
            allocation_velocity_response=self._velocity_response.copy(),
            allocation_modal_effort_m_s=self._modal_effort.copy(),
            allocation_accepted_velocity_valid=valid)
        return drive

    def hold(self):
        drive = super().hold()
        self.diagnostics.update(
            allocation_accepted_velocity_m_s=np.zeros(2),
            allocation_velocity_response=np.zeros((2, 2)),
            allocation_modal_effort_m_s=np.full(2, np.nan),
            allocation_accepted_velocity_valid=False,
            allocation_reference_residual_rms_m_s=np.nan)
        return drive
