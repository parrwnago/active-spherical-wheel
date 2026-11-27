"""Truth-free I-70 frame features from frozen measured-state primitives.

NumPy only. Geometry is the original 102-slot bank. The force map uses endpoint
geometry, with native angular generalized loads rotated by the observer's
interval-midpoint rotation. It is not a substep contact-history measurement.
"""
from __future__ import annotations

import numpy as np

from bank_points import candidate_layout, reconstruct_bank

FAMILY_ANCESTRY = np.array([[1, 1, 0], [1, 0, 1], [1, 0, 0], [1, 0, 0]], float)


def rotation(q):
    q = np.asarray(q, float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) <= 1e-12:
        raise ValueError('finite nonzero wxyz quaternion required')
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def cross_matrices(v):
    v = np.asarray(v, float)
    out = np.zeros(v.shape[:-1]+(3, 3))
    out[..., 0, 1] = -v[..., 2]; out[..., 0, 2] = v[..., 1]
    out[..., 1, 0] = v[..., 2]; out[..., 1, 2] = -v[..., 0]
    out[..., 2, 0] = -v[..., 1]; out[..., 2, 1] = v[..., 0]
    return out


def interval_rotation(primitives, index):
    q = primitives['qpos'][index]
    valid = (index > 0 and abs(float(primitives['time_s'][index]-primitives['time_s'][index-1])-.005) <= 1e-9
             and np.isfinite(primitives['qpos'][index-1]).all()
             and np.isfinite(q).all() and np.isfinite(primitives['observer_interval_mean'][index]).all())
    if valid:
        a, b = primitives['qpos'][index-1, 3:7], q[3:7]
        return rotation(a+(b if a @ b >= 0 else -b)), True
    return rotation(q[3:7]), False


def force_maps(primitives, index, points):
    """Map world force at each candidate into the observer-aligned nine rows."""
    q = primitives['qpos'][index]; centre = primitives['centre_m'][index]
    r = rotation(q[3:7]); middle, interval_valid = interval_rotation(primitives, index)
    family = candidate_layout()['family_index']
    arm_cross = cross_matrices(np.asarray(points)-centre)
    # Cached shell-A patch axis is carrier x. Shaft axis is fork y.
    shell_axis = primitives['patch_axis_world'][index, 0]
    hinge_axes = np.array([r[:, 1], shell_axis, shell_axis])
    out = np.zeros((len(points), 9, 3))
    out[:, :3] = np.eye(3)
    out[:, 3:6] = np.einsum('ij,cjk->cik', middle @ r.T, arm_cross)
    out[:, 6:] = (np.einsum('ji,cik->cjk', hinge_axes, arm_cross)
                   * FAMILY_ANCESTRY[family, :, None])
    return out, interval_valid


def material_velocities(primitives, index, points, normals):
    q, v = primitives['qpos'][index], primitives['qvel'][index]
    r = rotation(q[3:7]); family = candidate_layout()['family_index']
    shell_axis = primitives['patch_axis_world'][index, 0]
    axes = np.array([r[:, 1], shell_axis, shell_axis])
    omega = r @ v[3:6] + (FAMILY_ANCESTRY[family]*v[6:]) @ axes
    velocity = v[:3]+np.cross(omega, points-primitives['centre_m'][index])
    # Actual prescribed belt surface speed is tangent to the candidate plane.
    for f in (2, 3):
        mask = family == f; n = normals[mask]; y = r[:, 1]
        velocity[mask] += primitives['previous_applied'][index, f+1]*(y-(n @ y)[:, None]*n)
    tangent = velocity-normals*np.sum(normals*velocity, axis=1)[:, None]
    return velocity, tangent


def geometry(primitives, index):
    pose = dict(centre_m=primitives['centre_m'][index], patch_axis_world=primitives['patch_axis_world'][index],
                patch_lower_m=primitives['patch_lower_m'], patch_upper_m=primitives['patch_upper_m'],
                radius_m=primitives['radius_m'], valid=primitives['pose_valid'][index])
    return reconstruct_bank(pose, primitives['intrinsic_normal_hint'][index],
             observer_force_N=primitives['observer_generalized'][index, :3],
             original_frame_valid=primitives['original_frame_valid'][index])


def frame_features(primitives, index, cached=None, bank=None):
    """Method API. Only current and previous measured states can enter a frame."""
    layout = candidate_layout()
    if cached is None:
        bank = geometry(primitives, index) if bank is None else bank
        points = bank['points_m']
        normals = -(points-primitives['centre_m'][index])/float(primitives['radius_m'])
        material, tangent = material_velocities(primitives, index, points, normals)
        normal_angle = bank['normal_angle_rad']
    else:
        points, normals, material, tangent, normal_angle = (cached[key][index] for key in
             ('points_m', 'force_normals', 'material_velocity_m_s', 'tangent_velocity_m_s', 'normal_angle_rad'))
    maps, interval_valid = force_maps(primitives, index, points)
    finite = (np.isfinite(points).all(axis=1) & np.isfinite(normals).all(axis=1)
              & np.isfinite(material).all(axis=1) & np.isfinite(maps).all(axis=(1, 2)))
    valid = primitives['original_valid'][index] & finite & interval_valid
    return dict(observation9=np.array(primitives['observer_interval_mean'][index], copy=True),
                filtered_observation9=np.array(primitives['observer_generalized'][index], copy=True),
                points_m=points, force_normals=normals, material_velocity_m_s=material,
                tangent_velocity_m_s=tangent, force_maps=maps,
                family_index=layout['family_index'], candidate_index=layout['candidate_index'],
                candidate_kind=layout['kind'], valid=valid,
                normal_hint=np.array(primitives['intrinsic_normal_hint'][index], copy=True),
                normal_angle_rad=normal_angle, interval_valid=bool(interval_valid),
                original_valid=np.array(primitives['original_valid'][index], copy=True),
                time_s=float(primitives['time_s'][index]))
