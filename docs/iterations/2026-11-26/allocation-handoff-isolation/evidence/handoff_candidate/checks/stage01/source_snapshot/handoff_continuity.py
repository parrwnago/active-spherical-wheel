"""Opt-in, results-owned belt command continuity at observer load transitions.

This is a transfer-specific simulation experiment, not an actuator model or a
repository default. The native controller is called exactly once by the owner;
this pure hook receives its already box-clipped output. It never accesses a
plant, observer, prediction, contact solver, or controller. Inputs0:3 are motor
torques and pass through unchanged. Inputs3:5 are ideal belt speeds (m/s).
"""
from __future__ import annotations

import numpy as np


class BeltHandoffContinuity:
    """Limit a transfer episode around the previous ACTUALLY APPLIED command.

    A belt becomes loaded at >=5% of the current estimated total support load.
    The first call only initializes this state. Either subsequent crossing
    starts an episode, including an exit from load and contact-label chatter.
    Each active step projects the current native box-clipped target into the
    intersection of its operating cap and previous-applied +/- a*dt. There is
    no integrated/smoothed raw target. The episode ends immediately when the
    current native target is reached; no dwell, time-window gating or reference
    relaxation is added. A stable-mode command after completion passes through.

    The fixed campaign assumption a=3m/s^2 means 0->.30m/s in100ms. It is an
    experimental transfer envelope, not a measured motor acceleration limit.
    An estimated-load dropout can trigger an episode even if native terrain
    feedback holds its previous loaded gain. That behavior is explicit: this
    hook does not alter estimation or copy the controller's held-load logic.
    """

    def __init__(self, acceleration_m_s2=3., load_share_boundary=.05):
        if not np.isfinite(acceleration_m_s2) or acceleration_m_s2<=0:
            raise ValueError('Acceleration assumption must be finite and positive')
        if not np.isfinite(load_share_boundary) or not 0<load_share_boundary<1:
            raise ValueError('Load-share boundary must lie between zero and one')
        self.acceleration_m_s2=float(acceleration_m_s2)
        self.load_share_boundary=float(load_share_boundary)
        self.reset()

    def reset(self):
        self._previous_loaded=None
        self._episode_active=np.zeros(2,dtype=bool)
        self._last_time_s=None

    def state_dict(self):
        if self._previous_loaded is None:return None
        return dict(previous_loaded=self._previous_loaded.copy(),
                    episode_active=self._episode_active.copy(),last_time_s=self._last_time_s)

    def apply(self,native_applied,previous_applied,estimated_loads,dt,cap,time):
        """Return (final_applied, diagnostic); all supplied arrays stay untouched.

        `native_applied` and `previous_applied` must each contain five finite
        commands, with belts already inside `cap`. `estimated_loads` is the
        four-vector shell_A,shell_B,belt_A,belt_B inN, all nonnegative. `cap` is
        a positive scalar or two belt limits. `dt` is the physical next command
        interval; time is the strictly increasing current control timestamp.
        """
        native=np.asarray(native_applied,dtype=float)
        previous=np.asarray(previous_applied,dtype=float)
        loads=np.asarray(estimated_loads,dtype=float)
        caps=np.asarray(cap,dtype=float)
        if caps.ndim==0:caps=np.repeat(caps,2)
        if native.shape!=(5,) or previous.shape!=(5,) or not np.all(np.isfinite(native)) or not np.all(np.isfinite(previous)):
            raise ValueError('Native and previous applied commands need five finite entries')
        if loads.shape!=(4,) or not np.all(np.isfinite(loads)) or np.any(loads<0):
            raise ValueError('Four finite nonnegative estimated patch loads are required')
        if caps.shape!=(2,) or not np.all(np.isfinite(caps)) or np.any(caps<=0):
            raise ValueError('Operating caps must be a positive scalar or two-vector')
        if np.any(abs(native[3:])>caps) or np.any(abs(previous[3:])>caps):
            raise ValueError('Native/previous belt inputs must already obey operating caps')
        if not np.isfinite(dt) or dt<=0 or not np.isfinite(time):
            raise ValueError('Finite time and positive physical control interval are required')
        if self._last_time_s is not None and time<=self._last_time_s:
            raise ValueError('Control timestamps must increase strictly; reset for a new run')
        total=float(loads.sum())
        if not np.isfinite(total):raise ValueError('Total estimated support load must be finite')
        shares=loads[2:]/total if total>0 else np.zeros(2)
        loaded=(loads[2:]>=self.load_share_boundary*total)&(total>0)
        initialized=self._previous_loaded is None
        before=self._episode_active.copy()
        prior=None if initialized else self._previous_loaded.copy()
        triggered=np.zeros(2,dtype=bool)if initialized else loaded!=self._previous_loaded
        active=before|triggered
        aperture=self.acceleration_m_s2*float(dt)
        lower=np.maximum(-caps,previous[3:]-aperture)
        upper=np.minimum(caps,previous[3:]+aperture)
        final=native.copy()
        final[3:][active]=np.clip(native[3:][active],lower[active],upper[active])
        limited=final[3:]!=native[3:]
        after=active&limited
        completed=active&~limited
        self._previous_loaded=loaded.copy()
        self._episode_active=after.copy()
        self._last_time_s=float(time)
        diagnostic=dict(time_s=float(time),dt_s=float(dt),initialized_now=bool(initialized),
            acceleration_assumption_m_s2=self.acceleration_m_s2,load_share_boundary=self.load_share_boundary,
            total_estimated_load_N=total,estimated_belt_load_shares=shares.copy(),previous_loaded=prior,
            loaded=loaded.copy(),triggered=triggered.copy(),episode_before=before,
            episode_active=active.copy(),episode_after=after.copy(),completed=completed.copy(),limited=limited.copy(),
            native_applied=native.copy(),previous_actually_applied=previous.copy(),operating_caps_m_s=caps.copy(),
            slew_bounds_m_s=np.stack((lower,upper),axis=1),final_applied=final.copy(),
            continuity_delta=final-native,state_after=self.state_dict(),
            scope='Post-controller belt command only; observer load-share transition, not solved contacts or a motor model.')
        return final,diagnostic
