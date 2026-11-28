"""I75 result-owned path-facing reference and bounded heading helper.

N has no command patches. T changes only heading target and heading helper:
the desired Euler heading rate accompanies the path tangent, with exact Euler
heading-rate feedback instead of the original helper's world-z rate. The native
observer, LQR, contacts, reference lean, support torque, preparation and clipping
remain in their original command path.
"""
from contextlib import contextmanager
from unittest.mock import patch
import numpy as np
import mujoco

PROJECTED_AXIS_NORM2_MIN=1e-8
REFERENCE_KEYS=('raw_heading_rad','raw_heading_rate_rad_s','raw_heading_acceleration_rad_s2',
                'command_heading_rad','command_heading_rate_rad_s','command_heading_acceleration_rad_s2')


def wrap(angle):
    return float((angle+np.pi)%(2*np.pi)-np.pi)


def euler_heading_rate(rotation,omega_world,minimum=PROJECTED_AXIS_NORM2_MIN):
    """Derivative of atan2(R[1,0],R[0,0]), not the world-z angular component."""
    h=np.asarray(rotation,float).reshape(3,3)[:,0]
    denominator=float(h[0]*h[0]+h[1]*h[1])
    if not np.isfinite(denominator) or denominator<=minimum:
        return np.nan,False,denominator
    derivative=np.cross(np.asarray(omega_world,float),h)
    value=float((h[0]*derivative[1]-h[1]*derivative[0])/denominator)
    return value,bool(np.isfinite(value)),denominator


def helper_solution(controller,yaw,target,desired_rate,use_euler_rate):
    """Existing load/lever/budget policy with an explicit rate reference."""
    plant,params=controller.plant,controller.params
    raw_error=wrap(yaw-target);error=raw_error
    command=np.zeros(5);free=np.zeros(2,dtype=bool)
    axis=np.asarray(plant.data.xmat[controller.carrier_id]).reshape(3,3)[:,0].copy()
    vertical=float(axis[2]);horizontal=float(np.sqrt(max(1.-vertical*vertical,0.)))
    velocity=np.empty(6)
    mujoco.mj_objectVelocity(plant.model,plant.data,mujoco.mjtObj.mjOBJ_BODY,plant.fork_id,velocity,0)
    rotation=np.asarray(plant.data.xmat[plant.fork_id]).reshape(3,3)
    heading_rate,rate_valid,projected_norm2=euler_heading_rate(rotation,velocity[:3])
    measured_rate=heading_rate if use_euler_rate else float(velocity[2])
    held=controller._held;enabled=params.heading_gain>0 and held is not None
    steering=params.heading_actuation=='shells' and controller._yaw_error is not None
    if enabled:
        if steering:error=float(controller._yaw_error)
        elif params.heading_reference!='none':enabled=False
    if held is not None:
        loads=held['loads'];total=sum(loads.values())
        free=np.array([loads[name]<=.05*total for name in ('shell_A','shell_B')],bool)
    limit=float(params.heading_torque_limit_Nm)
    if params.heading_roll_budget_Nm>0:
        limit=min(limit,params.heading_roll_budget_Nm/max(horizontal,1e-6))
    eligible=bool(enabled and free.any() and abs(vertical)>=params.heading_min_lever
                  and (rate_valid or not use_euler_rate))
    wanted=0.
    if eligible:
        wanted=-(params.heading_gain*error+params.heading_damping*(measured_rate-desired_rate))
        each=float(np.clip(-wanted/vertical,-limit,limit))
        command[1:3][free]=each/int(free.sum())
    diagnostics=dict(actual_helper_target_rad=float(target),actual_helper_rate_reference_rad_s=float(desired_rate),
        helper_target_error_rad=raw_error,error_used_rad=float(error),world_z_rate_rad_s=float(velocity[2]),
        euler_heading_rate_rad_s=heading_rate,rate_valid=rate_valid,
        projected_forward_axis_norm2=projected_norm2,
        actual_helper_rate_measurement_rad_s=measured_rate,
        rate_error_rad_s=measured_rate-desired_rate,
        axis_world=axis,vertical_lever=vertical,horizontal_lever=horizontal,
        free_shell_mask=free,eligible=eligible,effective_motor_limit_Nm=limit,
        wanted_world_yaw_torque_Nm=float(wanted),expected_command=command.copy())
    return command,error,diagnostics


class TangentHeadingAdapter:
    def __init__(self,controller,policy,reference):
        if policy not in ('N','T'):raise ValueError('I75 policies are N and T')
        self.controller=controller;self.policy=policy;self.reference=reference
        self.initial_heading=float(controller.heading_reference)
        self.native_update=controller._update_heading_target
        self.native_helper=controller._heading_torque
        self._active=False;self._sample=None

    @contextmanager
    def command_context(self):
        if self._active:raise RuntimeError('Nested tangent helper context')
        self._sample=self.reference.sample(float(self.controller.plant.data.time))
        if not all(key in self._sample for key in REFERENCE_KEYS):raise ValueError('Missing planned heading reference fields')
        if not np.isfinite([self._sample[key] for key in REFERENCE_KEYS]).all():
            raise ArithmeticError('Nonfinite planned heading reference')
        self._active=True
        try:
            if self.policy=='N':yield self
            else:
                with patch.object(self.controller,'_update_heading_target',self._update_target),\
                     patch.object(self.controller,'_heading_torque',self._heading_torque):
                    yield self
        finally:self._active=False

    def _update_target(self,normal,yaw):
        # Preserve the original slow-normal/filter/history update exactly once.
        self.native_update(normal,yaw)
        limit=self.controller.params.heading_error_limit_rad
        return float(np.clip(wrap(yaw-self._sample['command_heading_rad']),-limit,limit))

    def _heading_torque(self,yaw):
        command,error,_=helper_solution(self.controller,yaw,
            self._sample['command_heading_rad'],self._sample['command_heading_rate_rad_s'],True)
        return command,error

    def diagnostics(self,info):
        if self._active or self._sample is None:raise RuntimeError('Diagnostics require completed command')
        controller=self.controller
        if float(controller.heading_reference)!=self.initial_heading:
            raise AssertionError('Native initial heading latch was mutated')
        target=(self.initial_heading if self.policy=='N' else self._sample['command_heading_rad'])
        rate=(0. if self.policy=='N' else self._sample['command_heading_rate_rad_s'])
        expected,error,row=helper_solution(controller,float(info['yaw_rad']),target,rate,self.policy=='T')
        np.testing.assert_allclose(info['heading_command'],expected,atol=1e-12,rtol=1e-12)
        np.testing.assert_allclose(float(info['heading_error_rad']),error,atol=1e-12,rtol=1e-12)
        row.update({key:float(self._sample[key]) for key in REFERENCE_KEYS})
        row.update(command_error_rad=wrap(float(info['yaw_rad'])-self._sample['command_heading_rad']),
            raw_error_rad=wrap(float(info['yaw_rad'])-self._sample['raw_heading_rad']),
            initial_heading_latched_rad=self.initial_heading,
            command_matches_registered_helper=True,
            tangent_policy_enabled=self.policy=='T',
            rate_limited=bool(self._sample.get('rate_limited',False)),
            acceleration_limited=bool(self._sample.get('acceleration_limited',False)))
        return {key:np.asarray(value).copy() for key,value in row.items()}
