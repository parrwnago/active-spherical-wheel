"""I76 analytic planned heading/rate, without measured-state or terrain input."""
import numpy as np


class PlannedTangentReference:
    def __init__(self, max_rate_rad_s, max_acceleration_rad_s2, path=None):
        if path is None:
            raise ValueError('An explicit frozen planned route is required')
        self.path = path
        self.max_rate_rad_s = float(max_rate_rad_s)
        self.max_acceleration_rad_s2 = float(max_acceleration_rad_s2)
        caps = [self.max_rate_rad_s, self.max_acceleration_rad_s2]
        if not np.isfinite(caps).all() or min(caps) <= 0:
            raise ValueError('Registered reference guards must be finite and positive')
        certificate = path.derivative_certificate()
        first = certificate['maximum_heading_parameter_derivative']
        second = certificate['maximum_heading_parameter_second_derivative']
        t = path.duration_s
        self.certified_max_rate_rad_s = first*1.875/t
        self.certified_max_acceleration_rad_s2 = second*(1.875/t)**2 + first*10/(np.sqrt(3)*t*t)
        if self.certified_max_rate_rad_s > self.max_rate_rad_s+1e-12:
            raise ValueError('Heading-rate guard does not cover this route certificate')
        if self.certified_max_acceleration_rad_s2 > self.max_acceleration_rad_s2+1e-12:
            raise ValueError('Heading-acceleration guard does not cover this route certificate')

    def sample(self, time_s):
        u, du, ddu, moving = self.path.progress(time_s)
        _, first, second, third = self.path.shape_derivatives(u)
        den = float(first @ first)
        if den <= 0.0 or not np.isfinite(den):
            raise ArithmeticError('Geometric tangent is invalid')
        cross12 = float(first[0]*second[1] - first[1]*second[0])
        cross13 = float(first[0]*third[1] - first[1]*third[0])
        first_angle = cross12/den
        second_angle = (cross13*den - cross12*2*float(first @ second))/(den*den)
        angle = self.path.tangent_heading(u)
        rate = float(first_angle*du)
        acceleration = float(second_angle*du*du + first_angle*ddu)
        if not moving:
            rate = acceleration = 0.0
        if abs(rate) > self.max_rate_rad_s+1e-12 or abs(acceleration) > self.max_acceleration_rad_s2+1e-12:
            raise ArithmeticError('Planned reference violates its registered guard')
        return dict(raw_heading_rad=angle, raw_heading_rate_rad_s=rate,
                    raw_heading_acceleration_rad_s2=acceleration,
                    command_heading_rad=angle, command_heading_rate_rad_s=rate,
                    command_heading_acceleration_rad_s2=acceleration,
                    rate_limited=False, acceleration_limited=False,
                    rate_guard_passed=True, acceleration_guard_passed=True,
                    parameter_u=float(u), motion_active=bool(moving))

    def spec(self):
        return dict(kind='i76_analytic_route_tangent', path=self.path.spec(),
                    formula='psi_u=cross(c1,c2)/|c1|^2; psi_uu=(cross(c1,c3)*|c1|^2-2*cross(c1,c2)*(c1.c2))/|c1|^4',
                    derivative_meaning='Euler heading and its time derivatives, not physical world-z angular velocity.',
                    max_rate_rad_s=self.max_rate_rad_s,
                    max_acceleration_rad_s2=self.max_acceleration_rad_s2,
                    geometric_certificate=self.path.derivative_certificate(),
                    certified_max_rate_rad_s=self.certified_max_rate_rad_s,
                    certified_max_acceleration_rad_s2=self.certified_max_acceleration_rad_s2,
                    raw_equals_command=True, active_governor=False,
                    angle_unwrap='Analytic planned-parameter branch; no temporal sample history or posthoc unwrap.',
                    endpoint_policy='Geometric initial/final tangent at rest; zero heading rate/acceleration in both holds.',
                    reference_inputs='Frozen planned route and time only; no measured state, contact, terrain or outcome.')
