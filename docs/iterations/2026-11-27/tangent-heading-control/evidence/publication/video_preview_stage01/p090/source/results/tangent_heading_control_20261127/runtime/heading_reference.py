"""I75 planned S-path tangent, with analytic Euler-heading derivatives.

Only the frozen geometric path and clock enter this reference.  The path's
quintic progress makes heading/rate/acceleration continuous at both stops.
The registered limits are verified guards, not a lagging target governor:
raw and commanded references are identical. A path outside those guards fails
rather than silently clipping a derivative while retaining a different angle.
"""
from pathlib import Path
import importlib.util
import numpy as np


def _default_path():
    source=Path(__file__).with_name('route.py')
    spec=importlib.util.spec_from_file_location('_i75_heading_path',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.SmoothSPath()


class PlannedTangentReference:
    def __init__(self,max_rate_rad_s,max_acceleration_rad_s2,path=None):
        self.path=_default_path() if path is None else path
        self.max_rate_rad_s=float(max_rate_rad_s)
        self.max_acceleration_rad_s2=float(max_acceleration_rad_s2)
        if not np.isfinite([self.max_rate_rad_s,self.max_acceleration_rad_s2]).all() or min(self.max_rate_rad_s,self.max_acceleration_rad_s2)<=0:
            raise ValueError('Registered reference guards must be finite and positive')
        if self.path.kind!='i73_smooth_s':raise ValueError('Only the frozen I73 S geometry is supported')
        self.start_s=float(self.path.start_s);self.duration_s=float(self.path.duration_s)
        self.forward_m=float(self.path.forward_m);self.lateral_m=float(self.path.lateral_m)
        self.geometry_heading_rad=float(np.deg2rad(self.path.geometry_heading_deg))
        if self.forward_m<=0 or self.duration_s<=0:raise ValueError('Positive forward geometry and duration required')
        self.k=2.*np.pi;self.B=self.k*self.lateral_m
        # Conservative global analytic certificate.  |dpsi/du|<=B*k/F;
        # |d2psi/du2|<=B*k²/F+2*B³*k²/F³, |du/dt|<=1.875/T,
        # and |d2u/dt2|<=10/(sqrt(3)*T²) for the frozen quintic.
        a=abs(self.B);f=self.forward_m;k=self.k;t=self.duration_s
        derivative_bound=a*k/f
        second_bound=a*k*k/f+2.*a**3*k*k/f**3
        self.certified_max_rate_rad_s=derivative_bound*1.875/t
        self.certified_max_acceleration_rad_s2=(second_bound*(1.875/t)**2+
                                                derivative_bound*10./(np.sqrt(3.)*t*t))
        if self.certified_max_rate_rad_s>self.max_rate_rad_s+1e-12:
            raise ValueError('Heading-rate guard does not cover this path certificate')
        if self.certified_max_acceleration_rad_s2>self.max_acceleration_rad_s2+1e-12:
            raise ValueError('Heading-acceleration guard does not cover this path certificate')

    def _progress(self,time_s):
        elapsed=float(time_s)-self.start_s
        if not np.isfinite(elapsed):raise ValueError('Reference clock must be finite')
        if elapsed<=0.:return 0.,0.,0.,False
        if elapsed>=self.duration_s:return 1.,0.,0.,False
        r=elapsed/self.duration_s;t=self.duration_s
        return (10.*r**3-15.*r**4+6.*r**5,
                (30.*r*r-60.*r**3+30.*r**4)/t,
                (60.*r-180.*r*r+120.*r**3)/(t*t),True)

    def sample(self,time_s):
        u,du,ddu,moving=self._progress(time_s)
        theta=self.k*u;y=self.B*np.cos(theta)
        den=self.forward_m**2+y*y
        angle=self.geometry_heading_rad+float(np.arctan2(y,self.forward_m))
        first=-self.forward_m*self.B*self.k*np.sin(theta)/den
        second=(-self.forward_m*self.B*self.k**2*np.cos(theta)/den-
                2.*self.forward_m*self.B**3*self.k**2*np.cos(theta)*np.sin(theta)**2/den**2)
        rate=float(first*du)
        acceleration=float(second*du*du+first*ddu)
        if not moving:rate=acceleration=0.
        if abs(rate)>self.max_rate_rad_s+1e-12 or abs(acceleration)>self.max_acceleration_rad_s2+1e-12:
            raise ArithmeticError('Planned reference violates its registered guard')
        return dict(raw_heading_rad=angle,raw_heading_rate_rad_s=rate,
                    raw_heading_acceleration_rad_s2=acceleration,
                    command_heading_rad=angle,command_heading_rate_rad_s=rate,
                    command_heading_acceleration_rad_s2=acceleration,
                    rate_limited=False,acceleration_limited=False,
                    rate_guard_passed=True,acceleration_guard_passed=True,
                    parameter_u=float(u),motion_active=bool(moving))

    def spec(self):
        return dict(kind='i75_analytic_S_tangent',path=self.path.spec(),
                    formula='psi =113deg+atan2(2*pi*.14*cos(2*pi*u),1.2)',
                    derivative_meaning='Euler heading and its time derivatives, not physical world-z angular velocity.',
                    max_rate_rad_s=self.max_rate_rad_s,
                    max_acceleration_rad_s2=self.max_acceleration_rad_s2,
                    certified_max_rate_rad_s=self.certified_max_rate_rad_s,
                    certified_max_acceleration_rad_s2=self.certified_max_acceleration_rad_s2,
                    raw_equals_command=True,active_governor=False,
                    endpoint_policy='Known geometric tangent at the initial stop; final geometric tangent held after motion, zero heading rate/acceleration.',
                    reference_inputs='Frozen planned path and time only; no measured state, contact, terrain or outcome.')
