"""I78 result-owned, reactive path-progress experiment.

The nominal geometric path remains unchanged. A causal scalar cascade changes
its virtual clock. This is not a hard closed-loop feasibility certificate:
existing demand can remain excessive while the clock decelerates.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class GovernorSettings:
    time_constant_s: float = .10
    maximum_stage_rate_s_inv: float = 4.
    maximum_substep_s: float = .001
    authority_scale_m_s: float = .15
    joint_reference_scale_rad_s: float = 5.
    command_margin_fraction: float = .85

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Governor settings must be finite and positive: '+name)
            object.__setattr__(self, name, value)
        if self.command_margin_fraction > 1.:
            raise ValueError('Command margin cannot exceed the actual command box')
        if self.maximum_substep_s > self.time_constant_s:
            raise ValueError('SSP step must not exceed the cascade time constant')


def bounded_rate(difference, settings):
    """Scalar tanh flow, Lipschitz <=1/time_constant and |flow|<=stage rate."""
    limit = settings.maximum_stage_rate_s_inv
    return limit*math.tanh(float(difference)/(limit*settings.time_constant_s))


def cascade_rhs(z, rate, target, settings):
    return (bounded_rate(target-z, settings),
            bounded_rate(z-rate, settings))


def cascade_step(z, rate, virtual_time_s, target, step_s, settings):
    """One SSP-Heun step, including d(virtual time)/dt=rate.

    The Euler stage is a convex movement toward its input for h<=tau, so both
    scalar cascade states remain in [0,1] without a speed floor or clipping.
    """
    h = float(step_s)
    if not 0 < h <= settings.time_constant_s:
        raise ValueError('Positive Heun step no larger than the time constant required')
    dz0, dr0 = cascade_rhs(z, rate, target, settings)
    z1, r1 = z+h*dz0, rate+h*dr0
    dz1, dr1 = cascade_rhs(z1, r1, target, settings)
    result = (.5*z+.5*(z1+h*dz1), .5*rate+.5*(r1+h*dr1),
              virtual_time_s+.5*h*(rate+r1))
    if not all(math.isfinite(x) for x in result):
        raise ValueError('Nonfinite cascade state')
    if not (-1e-14 <= result[0] <= 1.+1e-14 and -1e-14 <= result[1] <= 1.+1e-14):
        raise ArithmeticError('Scalar-cascade invariant lost')
    return result


def demand_target(info, settings):
    """One fixed heuristic from a full-speed private native-command probe.

    Authority is the second singular value of a load-weighted mean 2x5 rolling
    map using the existing column scales. It is not a per-contact no-slip or
    dynamic-authority guarantee. The 5 rad/s scale is not a motor rating.
    """
    probe = info['i78_probe']
    sigma = float(probe['authority_min_scaled_sigma'])
    drive = np.asarray(info['allocation_drive_reference'], float)
    requested = np.asarray(info['requested'], float)
    bounds = np.asarray(info['input_bounds'], float)
    if drive.shape != (5,) or requested.shape != (5,) or bounds.shape != (5,):
        raise ValueError('Five allocation references, command requests and bounds required')
    if not np.isfinite(np.r_[sigma,drive,requested,bounds]).all() or sigma < 0 or np.any(bounds<=0):
        raise ValueError('Finite nonnegative authority and positive command bounds required')
    maximum_joint = float(np.max(np.abs(drive[:3])))
    ratio = float(np.max(np.abs(requested)/bounds))
    authority_factor = min(1., sigma/settings.authority_scale_m_s)
    joint_factor = (1. if maximum_joint <= settings.joint_reference_scale_rad_s else
                    settings.joint_reference_scale_rad_s/maximum_joint)
    command_factor = (1. if ratio <= settings.command_margin_fraction else
                      settings.command_margin_fraction/ratio)
    target = min(1., authority_factor, joint_factor, command_factor)
    return target, dict(authority_factor=authority_factor,
        joint_reference_factor=joint_factor, command_factor=command_factor,
        authority_sigma_min_m_s=sigma, maximum_nominal_joint_reference_rad_s=maximum_joint,
        nominal_command_ratio=ratio, raw_target_rate=target)


def retimed_reference(route, virtual_time_s, clock_rate, clock_acceleration_s_inv):
    """Exact chain rule applied to the original nominal clock parameter."""
    position, virtual_velocity, virtual_acceleration = route(virtual_time_s)
    rate, acceleration = float(clock_rate), float(clock_acceleration_s_inv)
    if rate == 1. and acceleration == 0.:
        return position,virtual_velocity,virtual_acceleration
    return (position,virtual_velocity*rate,
            virtual_acceleration*rate*rate+virtual_velocity*acceleration)


class ProgressGovernor:
    """One native probe per new time; no contact/model/controller updates here.

    At each time: integrate the previous held target, evaluate the full-speed
    nominal reference at the resulting virtual time, then register its new
    target. Registering target changes neither rate nor acceleration now.
    The cascade gives continuous position, velocity and acceleration in the
    numerical-flow limit. The original quintic endpoint has zero first and
    second derivatives, allowing an explicit terminal reference mode.
    """
    def __init__(self, route, settings=None):
        self.route = route
        self.settings = settings or GovernorSettings()
        self.end_virtual_s = float(route.start_s+route.duration_s)
        if not math.isfinite(self.end_virtual_s) or self.end_virtual_s <= route.start_s:
            raise ValueError('Finite ordered nominal route clock required')
        self.reset()

    def reset(self):
        self._time = None
        self._virtual = None
        self._z = self._rate = self._target = 1.
        self._ever_retimed = False
        self._finished_at = None
        self._first_complete_sample = None
        self._last_selection = None
        self.probe_calls = 0

    def state_dict(self):
        return dict(time_s=self._time, virtual_time_s=self._virtual,
            cascade_state=self._z, clock_rate=self._rate, held_target_rate=self._target,
            ever_retimed=self._ever_retimed, finished_at_s=self._finished_at,
            first_complete_sample_s=self._first_complete_sample, probe_calls=self.probe_calls)

    @property
    def finished_at_s(self): return self._finished_at

    def _finish(self, physical_time_s):
        self._virtual = self.end_virtual_s
        self._finished_at = float(physical_time_s)
        # Reference-generator terminal mode, not a physical pose/history reset.
        self._z = self._rate = self._target = 0.

    def advance_to(self, now_s):
        now = float(now_s)
        if not math.isfinite(now) or now < 0:
            raise ValueError('Finite nonnegative physical clock required')
        if self._time is None:
            self._time = now
            self._virtual = now
            if now >= self.end_virtual_s: self._finish(self.end_virtual_s)
            return
        if now < self._time:
            raise ValueError('Physical clock went backwards; reset explicitly')
        if now == self._time: return
        if self._finished_at is not None:
            self._time = now
            return
        if not self._ever_retimed and self._target == self._z == self._rate == 1.:
            # Preserve exact original reference bytes before the first slowdown.
            self._virtual = now
            if now >= self.end_virtual_s: self._finish(self.end_virtual_s)
            self._time = now
            return
        elapsed = now-self._time
        count = max(1, int(math.ceil(elapsed/self.settings.maximum_substep_s)))
        h = elapsed/count
        physical = self._time
        for _ in range(count):
            z0, r0, t0 = self._z, self._rate, self._virtual
            z, rate, virtual = cascade_step(z0,r0,t0,self._target,h,self.settings)
            if virtual >= self.end_virtual_s:
                # Heun's virtual-time dense extension is t0+r0*s+dr0*s²/2.
                dr0 = cascade_rhs(z0,r0,self._target,self.settings)[1]
                distance = self.end_virtual_s-t0
                discriminant = max(0., r0*r0+2*dr0*distance)
                denominator = r0+math.sqrt(discriminant)
                crossing = (2*distance/denominator if denominator > 0 else h)
                if not -1e-10 <= crossing <= h+1e-10:
                    raise ArithmeticError('Invalid terminal crossing inside Heun step')
                self._finish(physical+min(h,max(0.,crossing)))
                break
            self._z,self._rate,self._virtual = z,rate,virtual
            physical += h
        self._time = now

    def _reference(self):
        if self._finished_at is not None:
            return self.route(self._virtual), 0.
        acceleration = cascade_rhs(self._z,self._rate,self._target,self.settings)[1]
        return retimed_reference(self.route,self._virtual,self._rate,acceleration),acceleration

    def select(self, now_s, evaluate):
        now = float(now_s)
        if (self._last_selection is not None
                and now == self._last_selection['diagnostics']['time_s']):
            return self._last_selection
        self.advance_to(now)
        reference, clock_acceleration = self._reference()
        nominal = self.route(self._virtual)
        nominal_probe = evaluate(*nominal)
        self.probe_calls += 1
        _, info = nominal_probe
        raw_target, pressure = demand_target(info,self.settings)
        previous_target = self._target
        initial_hold = now <= float(self.route.start_s)
        if self._finished_at is not None:
            target = 0.
            if self._first_complete_sample is None: self._first_complete_sample = now
        elif initial_hold:
            target = 1.
        else:
            target = raw_target
            if target < 1.: self._ever_retimed = True
        # Do not advance states with this newly observed target.
        self._target = target
        u, du, ddu, _ = self.route.progress(self._virtual)
        diagnostics = dict(time_s=now,virtual_time_s=self._virtual,
            virtual_delay_s=now-self._virtual,clock_rate=self._rate,
            clock_acceleration_s_inv=clock_acceleration,cascade_state=self._z,
            previous_target_rate=previous_target,new_target_rate=target,
            progress_u=u,progress_rate_s_inv=du*self._rate,
            progress_acceleration_s_inv2=ddu*self._rate*self._rate+du*clock_acceleration,
            reference_complete=self._finished_at is not None,
            finished_at_s=self._finished_at,
            first_complete_sample_s=self._first_complete_sample,
            completion_time_kind='Heun dense-step virtual-clock crossing; reference completion, not robot settling',
            initial_hold_forced_native=initial_hold,nominal_probe_calls=self.probe_calls,
            retiming_ever_active=self._ever_retimed,**pressure)
        selection = dict(reference=reference,nominal_reference=nominal,
                         nominal_probe=nominal_probe,diagnostics=diagnostics)
        self._last_selection = selection
        return selection
