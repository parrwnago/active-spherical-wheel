"""Rolling consistency from measured motion and candidate contacts only.

No contacts or terrain are queried here. A residual is evidence that the
candidate rolling constraint does not explain the motion; it cannot alone
distinguish slip, an incorrect contact, or measurement/model error.
"""
from __future__ import annotations

import mujoco
import numpy as np

SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')


def rolling_consistency(plant, records, minimum_load_N=.1):
    """Compare tangential body velocity with the no-slip prediction.

    ``records`` are supplied by the estimator, not fetched from the plant.
    Joint/body rates come from the current measured state. Belt surface
    velocity is the last applied input in this ideal-belt model, i.e. the
    interval ending at this measurement, not the next requested command.
    Both body-velocity outputs are tangential, world-frame quantities.
    RMS/max residuals preserve opposing slip at separate contacts even when
    their weighted vector average cancels.
    """
    model, data = plant.model, plant.data
    predicted, measured, residual = np.zeros(3), np.zeros(3), np.zeros(3)
    squared, maximum, total = 0., 0., 0.
    points, loads = np.zeros((4, 3)), np.zeros(4)
    for record in records:
        name = record['name']
        support = next((item for item in SUPPORTS if name.startswith(item)), None)
        if support is None:
            continue
        load = max(0., float(record['normal_force']))
        normal = np.asarray(record['normal'], float)
        length = float(np.linalg.norm(normal))
        if load <= 0 or not np.isfinite(load) or not np.isfinite(length) or length < 1e-12:
            continue
        normal = normal/length
        projection = np.eye(3)-np.outer(normal, normal)
        jacobian = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jacobian, None, record['pos'], int(record['body']))
        # Free-base translation is world-frame; remaining columns include
        # measured base rotation and every articulated joint's motion.
        other = jacobian[:, 3:]@data.qvel[3:]
        if support.startswith('belt'):
            geom = int(record['geom'])
            other += data.geom_xmat[geom].reshape(3, 3)@model.geom_surfacevel[geom, :3]
        expected = -projection@other
        actual = projection@data.qvel[:3]
        mismatch = actual-expected
        if not np.all(np.isfinite(mismatch)):
            continue
        predicted += load*expected
        measured += load*actual
        residual += load*mismatch
        magnitude = float(np.linalg.norm(mismatch))
        squared += load*magnitude**2
        maximum = max(maximum, magnitude)
        total += load
        index = SUPPORTS.index(support)
        points[index] += load*np.asarray(record['pos'])
        loads[index] += load
    positions = np.full((4, 3), np.nan)
    active = loads > 0
    positions[active] = points[active]/loads[active, None]
    valid = bool(total >= minimum_load_N and total > 0)
    missing = np.full(3, np.nan)
    return {
        'predicted_body_velocity_m_s': predicted/total if valid else missing.copy(),
        'measured_body_velocity_m_s': measured/total if valid else missing.copy(),
        'contact_velocity_residual_m_s': residual/total if valid else missing.copy(),
        'contact_velocity_residual_rms_m_s': float(np.sqrt(squared/total)) if valid else float('nan'),
        'contact_velocity_residual_max_m_s': maximum if valid else float('nan'),
        'velocity_consistency_valid': valid,
        'kinematic_inconsistency': bool(valid and np.sqrt(squared/total) > .02),
        'estimated_contact_points_m': positions,
    }
