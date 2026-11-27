"""Read-only I74 telemetry for the existing bounded heading helper.

This module supplies no command. It verifies the logged native heading term
against the existing helper's geometry, load eligibility and declared limits.
"""
import numpy as np
import mujoco


def heading_diagnostics(controller, info):
    plant, params = controller.plant, controller.params
    yaw = float(info['yaw_rad'])
    target = float(controller.heading_reference)
    raw_error = float((yaw-target+np.pi) % (2*np.pi)-np.pi)
    expected = np.zeros(5)
    free_mask = np.zeros(2, dtype=bool)
    axis = np.asarray(plant.data.xmat[controller.carrier_id]).reshape(3,3)[:,0].copy()
    vertical = float(axis[2])
    horizontal = float(np.sqrt(max(1.-vertical*vertical,0.)))
    velocity = np.empty(6)
    mujoco.mj_objectVelocity(plant.model,plant.data,mujoco.mjtObj.mjOBJ_BODY,
                            plant.fork_id,velocity,0)
    world_z_rate = float(velocity[2])
    error = raw_error
    eligible = False
    limit = float(params.heading_torque_limit_Nm)
    wanted = 0.
    held = controller._held
    helper_enabled = params.heading_gain > 0 and held is not None
    steering = params.heading_actuation == 'shells' and controller._yaw_error is not None
    if helper_enabled:
        if steering:
            error = float(controller._yaw_error)
        elif params.heading_reference != 'none':
            helper_enabled = False
    if held is not None:
        loads = held['loads']
        total = sum(loads.values())
        free_mask = np.asarray([loads[name] <= .05*total for name in ('shell_A','shell_B')],bool)
    if params.heading_roll_budget_Nm > 0:
        limit = min(limit,params.heading_roll_budget_Nm/max(horizontal,1e-6))
    eligible = bool(helper_enabled and free_mask.any() and abs(vertical)>=params.heading_min_lever)
    if eligible:
        wanted = -(params.heading_gain*error+params.heading_damping*world_z_rate)
        each = float(np.clip(-wanted/vertical,-limit,limit))
        expected[1:3][free_mask] = each/int(free_mask.sum())
    actual = np.asarray(info['heading_command'],float)
    np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=1e-12,
        err_msg='Logged heading delta differs from existing native helper')
    np.testing.assert_allclose(float(info['heading_error_rad']),error,atol=1e-12,rtol=1e-12)
    return dict(target_rad=target,raw_error_rad=raw_error,error_used_rad=error,
        world_z_rate_rad_s=world_z_rate,axis_world=axis,
        vertical_lever=vertical,horizontal_lever=horizontal,
        free_shell_mask=free_mask,eligible=eligible,
        effective_motor_limit_Nm=limit,wanted_world_yaw_torque_Nm=wanted,
        expected_command=expected,command_matches_native=True)
