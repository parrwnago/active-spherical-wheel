"""Independent force/lever/axis generalized-load reference, no point Jacobian.

Current-state known geometry only. The caller owns private kinematic data and
performs no physics. Rows 0:6 are world force/moment; rows 6:9 are hinge scalars.
"""
import numpy as np


def point_load(model, data, point_world, family, force_world):
    import mujoco
    point, force = np.asarray(point_world, float), np.asarray(force_world, float)
    result = np.full(9, np.nan)
    if point.shape != (3,) or force.shape != (3,):
        raise ValueError('World point and force must each have three components')
    if family not in (0, 1, 2, 3) or not np.isfinite(point).all() or not np.isfinite(force).all():
        return result
    fork = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'fork')
    result[:3] = force
    result[3:6] = np.cross(point-data.xpos[fork], force)
    result[6:9] = 0.
    for index, joint_name in enumerate(('shaft', 'shell_A_joint', 'shell_B_joint')):
        if index == 1 and family != 0:
            continue
        if index == 2 and family != 1:
            continue
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint < 0 or model.jnt_dofadr[joint] != 6+index:
            raise ValueError('Unexpected hinge definition/order')
        result[6+index] = data.xaxis[joint]@np.cross(point-data.xanchor[joint], force)
    return result


def joint_gate(prediction, observed_generalized, force_norm, radius,
               *, absolute_tolerance_Nm=.03, relative_tolerance=.10):
    """Independent componentwise check without using the production ratio test."""
    predicted, observed = np.asarray(prediction, float), np.asarray(observed_generalized, float)
    tolerance = absolute_tolerance_Nm+relative_tolerance*radius*force_norm
    valid = (predicted.shape == (9,) and observed.shape == (9,)
             and np.isfinite(predicted).all() and np.isfinite(observed).all()
             and np.isfinite(tolerance) and radius > 0 and force_norm >= 1.)
    return bool(valid and all(abs(observed[j]-predicted[j]) <= tolerance for j in (6,7,8)))
