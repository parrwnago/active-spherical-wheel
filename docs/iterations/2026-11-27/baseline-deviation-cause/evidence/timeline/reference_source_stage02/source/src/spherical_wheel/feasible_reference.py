"""Project travel references while retaining the original destination.

This changes feedback setpoints, not the nominal trajectory clock or balance
gains. The nominal error is retained for capped catch-up when authority returns.
No state is advanced, reset, or supplied from simulator contacts here.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FeasibleReferenceSettings:
    catchup_gain_s_inv: float = 1.
    catchup_speed_m_s: float = .06
    position_error_limit_m: float = .05

    def __post_init__(self):
        if not np.isfinite(self.catchup_gain_s_inv) or self.catchup_gain_s_inv < 0:
            raise ValueError('catchup_gain_s_inv must be finite and nonnegative')
        for name in ('catchup_speed_m_s', 'position_error_limit_m'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')


def _vector(value):
    result = np.asarray(value, float)
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError('Travel references must be finite two-dimensional vectors')
    return result


def _cap(vector, limit):
    return vector*min(1., limit/max(float(np.linalg.norm(vector)), 1e-15))


def balance_travel_priority(lean_error_rad, lean_rate_rad_s, scale_rad, horizon_s=.1):
    """Smooth travel priority from measured reference error and lean rate.

    This schedules travel setpoints only. It is not a stability certificate.
    """
    values = (lean_error_rad, lean_rate_rad_s, scale_rad, horizon_s)
    if not np.isfinite(values).all() or lean_error_rad < 0 or lean_rate_rad_s < 0 or scale_rad <= 0 or horizon_s < 0:
        raise ValueError('Balance priority needs finite nonnegative errors/horizon and positive scale')
    metric = lean_error_rad+horizon_s*lean_rate_rad_s
    ratio = min(metric/scale_rad, 1e6)
    return 1./(1.+ratio**4), metric


def _priority(value):
    if not np.isfinite(value) or not 0 <= value <= 1:
        raise ValueError('Travel priority must lie between zero and one')
    return float(value)


class FeasibleMotionReference:
    def __init__(self, settings=None):
        self.settings = settings or FeasibleReferenceSettings()

    def request(self, nominal_xy, nominal_velocity, measured_xy, response=None, priority=1.):
        error = _vector(nominal_xy)-_vector(measured_xy)
        if response is not None:
            response = np.asarray(response, float)
            if response.shape != (2, 2) or not np.isfinite(response).all():
                raise ValueError('Velocity response must be a finite 2 by 2 matrix')
            error = response@error
        correction = _cap(self.settings.catchup_gain_s_inv*error, self.settings.catchup_speed_m_s)
        requested = _vector(nominal_velocity)+correction
        return requested if _priority(priority) == 1. else priority*requested

    def feedback(self, nominal_xy, measured_xy, nominal_acceleration, accepted_velocity,
                 response, valid=True, priority=1.):
        nominal, measured = _vector(nominal_xy), _vector(measured_xy)
        velocity, acceleration = _vector(accepted_velocity), _vector(nominal_acceleration)
        response = np.asarray(response, float)
        if response.shape != (2, 2) or not np.isfinite(response).all():
            raise ValueError('Velocity response must be a finite 2 by 2 matrix')
        if valid:
            # Roundoff protection: travel-error relaxation must not reverse or
            # amplify an error. The response is geometric, not a stability proof.
            values, vectors = np.linalg.eigh(.5*(response+response.T))
            projection = (vectors*np.clip(values, 0., 1.))@vectors.T
        else:
            projection = np.zeros((2, 2))
            velocity = np.zeros(2)
        accepted_error = _cap(projection@(nominal-measured), self.settings.position_error_limit_m)
        priority = _priority(priority)
        if priority != 1.:
            accepted_error = priority*accepted_error
        return {'xy': measured+accepted_error, 'velocity': velocity,
                # Nominal lean feedforward is relaxed in unavailable directions.
                # This is not the exact derivative of velocity, which also
                # depends on geometry, saturation, and the catch-up request.
                'acceleration': priority*(projection@acceleration),
                'relaxed_position_error_m': nominal-measured-accepted_error,
                'projection': projection}
