"""I51 isolated joint-rate-reference box guard; no plant/controller imports.

The native rolling least-squares solve, requested XY/body velocity, belt
feedforward and position target remain unchanged. This guard changes only the
three joint-rate references supplied to the native LQR. A bounded reference is
not a guarantee of bounded requested torque, achievable motion or balance.
"""
from dataclasses import dataclass
import copy

import numpy as np


@dataclass(frozen=True)
class ReferenceLimits:
    """Existing simulation reference assumptions, not measured motor limits."""
    speed_limits_rad_s: tuple = (20., 20., 20.)
    acceleration_limits_rad_s2: tuple = (100., 100., 100.)

    def __post_init__(self):
        for name in ('speed_limits_rad_s', 'acceleration_limits_rad_s2'):
            value = np.asarray(getattr(self, name), float)
            if value.shape != (3,) or not np.all(np.isfinite(value) & (value > 0)):
                raise ValueError(f'{name} must contain three positive finite values')
            object.__setattr__(self, name, tuple(float(item) for item in value))


class JointReferenceBoxGuard:
    """Nearest componentwise reference inside speed and previous-output slew box.

    No geometry/condition threshold or effort weights are introduced. A native
    target inside the current box is returned bit-for-bit. Targets can remain
    limited during recovery from a prior violation even at good geometry.
    First emission starts from zero with the declared controller dt; later
    emissions use elapsed measured time. Same-time/same-target calls are
    idempotent; changed targets at the same time and backwards clocks fail.
    """
    def __init__(self, limits=None):
        self.limits = limits or ReferenceLimits()
        self.reset()

    def reset(self):
        self.previous = np.zeros(3)
        self.time_s = None
        self.update_count = 0
        self._last_target = None
        self._last_diagnostic = None

    def state_dict(self):
        return dict(previous_reference_rad_s=self.previous.copy(), time_s=self.time_s,
            update_count=self.update_count,
            last_native_reference_rad_s=None if self._last_target is None else self._last_target.copy())

    def update(self, native_joint_refs, time_s, dt_s):
        target = np.asarray(native_joint_refs, float)
        now, dt = float(time_s), float(dt_s)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError('Expected three finite native joint-rate references')
        if not np.isfinite(now) or not np.isfinite(dt) or dt <= 0:
            raise ValueError('Reference timestamp and positive interval must be finite')
        if self.time_s is not None and now < self.time_s:
            raise ValueError('Reference timestamps cannot go backwards')
        if self.time_s is not None and now == self.time_s:
            if not np.array_equal(target, self._last_target):
                raise ValueError('A repeated timestamp cannot carry another native reference')
            diagnostic = copy.deepcopy(self._last_diagnostic)
            diagnostic['repeated_timestamp'] = True
            diagnostic['initialized_now'] = False
            return self.previous.copy(), diagnostic
        initial = self.time_s is None
        elapsed = dt if initial else now-self.time_s
        speed = np.asarray(self.limits.speed_limits_rad_s, float)
        acceleration = np.asarray(self.limits.acceleration_limits_rad_s2, float)
        previous = self.previous.copy()
        slew_lower, slew_upper = previous-acceleration*elapsed, previous+acceleration*elapsed
        lower, upper = np.maximum(-speed, slew_lower), np.minimum(speed, slew_upper)
        guarded = np.clip(target, lower, upper)
        changed = guarded != target
        diagnostic = dict(native_joint_reference_rad_s=target.copy(), guarded_joint_reference_rad_s=guarded.copy(),
            previous_joint_reference_rad_s=previous, reference_delta_rad_s=guarded-target,
            speed_limits_rad_s=speed.copy(), acceleration_limits_rad_s2=acceleration.copy(),
            lower_reference_rad_s=lower, upper_reference_rad_s=upper,
            speed_clip_mask=np.abs(target)>speed,
            slew_clip_mask=(target<slew_lower)|(target>slew_upper),
            modified_mask=changed, modified=bool(changed.any()), unmodified=not bool(changed.any()),
            initialized_now=initial, repeated_timestamp=False, time_s=now, elapsed_s=elapsed,
            update_count=self.update_count+1)
        self.previous = guarded.copy()
        self.time_s = now
        self.update_count += 1
        self._last_target = target.copy()
        self._last_diagnostic = copy.deepcopy(diagnostic)
        return guarded.copy(), diagnostic


def reset(controller):
    """Reset only I51-owned state, from the harness's native reset wrapper."""
    guard = getattr(controller, '_i51_joint_reference_guard', None)
    if guard is not None:
        if not isinstance(guard, JointReferenceBoxGuard):
            raise TypeError('Unexpected owner of I51 reference-guard history')
        guard.reset()


def apply(controller, vref):
    """Mutate ONLY vref[6:9], immediately before the one original feedback call.

    The harness owns the opt-in branch and original method invocation. Its
    reset wrapper calls reset(controller); baseline must never call apply.
    Native command then logs/stores this same vref and after_step integrates
    it. Dropout-held and zero-motion references pass through the same guard.
    """
    if (not isinstance(vref, np.ndarray) or vref.shape != (9,) or
            vref.dtype.kind != 'f' or not vref.flags.writeable or not np.isfinite(vref).all()):
        raise ValueError('Expected a writable nine-element floating native velocity reference')
    guard = getattr(controller, '_i51_joint_reference_guard', None)
    new_guard = guard is None
    if guard is None:
        guard = JointReferenceBoxGuard()
    elif not isinstance(guard, JointReferenceBoxGuard):
        raise TypeError('Unexpected owner of I51 reference-guard history')
    unchanged = vref[:6].copy()
    guarded, diagnostic = guard.update(vref[6:9], controller.plant.data.time, controller.plant.dt)
    if new_guard:
        controller._i51_joint_reference_guard = guard
    vref[6:9] = guarded
    np.testing.assert_array_equal(vref[:6], unchanged)
    return diagnostic


def rolling_residual(matrix, rhs, native_drive, guarded_joint_refs):
    """Descriptive kinematic residual only; no allocation or dynamics execution."""
    matrix, rhs = np.asarray(matrix, float), np.asarray(rhs, float)
    native, joints = np.asarray(native_drive, float), np.asarray(guarded_joint_refs, float)
    if (matrix.ndim != 2 or matrix.shape[1] != 5 or rhs.shape != (len(matrix),) or
            native.shape != (5,) or joints.shape != (3,) or
            not all(np.isfinite(a).all() for a in (matrix, rhs, native, joints))):
        raise ValueError('Expected finite rolling matrix[N,5], rhs[N], drive[5] and guarded joints[3]')
    guarded = native.copy()
    guarded[:3] = joints
    original_error, guarded_error = matrix@native-rhs, matrix@guarded-rhs
    return dict(native_drive=native.copy(), guarded_drive=guarded,
        native_residual_m_s=original_error, guarded_residual_m_s=guarded_error,
        native_residual_norm_m_s=float(np.linalg.norm(original_error)),
        guarded_residual_norm_m_s=float(np.linalg.norm(guarded_error)),
        belt_feedforward_unchanged=bool(np.array_equal(native[3:], guarded[3:])),
        interpretation='Kinematic reference mismatch; not predicted actual tracking, contact slip, torque feasibility or closed-loop performance.')

# deliberate wrong-hash fixture
