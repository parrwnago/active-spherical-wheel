"""Portable I-68 force-map and slip arithmetic; NumPy only, no controller code.

The analytical map is restricted to this repository's coincident-origin
fork/shaft/shell topology. Callers must verify the frozen model metadata.
"""
from __future__ import annotations

import numpy as np


TAG_FAMILY = np.array([0, 1, 2, 3, 0, 1, 4], dtype=np.int64)
ANCESTRY = np.array([[1, 1, 0], [1, 0, 1], [1, 0, 0],
                     [1, 0, 0], [1, 1, 0], [1, 0, 1],
                     [0, 0, 0]], dtype=float)


def rotation(quaternion):
    """Normalized wxyz quaternion to world-from-fork rotation."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) == 0:
        raise ValueError('finite nonzero wxyz quaternion required')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def middle_rotation(qpos0, qpos1):
    """The observer's hemisphere-corrected normalized quaternion sum."""
    a, b = np.asarray(qpos0)[3:7], np.asarray(qpos1)[3:7]
    return rotation(a + (b if a @ b >= 0 else -b))


def world_rows(generalized, world_from_fork):
    out = np.array(generalized, dtype=float, copy=True)
    out[..., 3:6] = np.einsum('ij,...j->...i', world_from_fork, out[..., 3:6])
    return out


def axes(qpos):
    q = np.asarray(qpos, dtype=float)
    if q.shape != (10,) or not np.isfinite(q).all():
        raise ValueError('finite qpos10 required')
    r = rotation(q[3:7])
    # Both shell hinge axes are carrier x; shell's own x rotation preserves x.
    shell = r @ np.array([np.cos(q[7]), 0., -np.sin(q[7])])
    return r, np.array([r[:, 1], shell, shell])


def point_force_generalized(qpos, points, forces, tags):
    """Per-contact native generalized load. Unknown or nonfinite rows are NaN."""
    q = np.asarray(qpos, dtype=float)
    p, f, raw_tags = np.asarray(points, float), np.asarray(forces, float), np.asarray(tags)
    if p.shape != f.shape or p.ndim != 2 or p.shape[1] != 3 or raw_tags.shape != (len(p),):
        raise ValueError('points/forces Nx3 and tags N required')
    r, hinge_axes = axes(q)
    valid_tag = np.isfinite(raw_tags) & (raw_tags == np.floor(raw_tags)) & (raw_tags >= 0) & (raw_tags < 7)
    valid = valid_tag & np.isfinite(p).all(axis=1) & np.isfinite(f).all(axis=1)
    out = np.full((len(p), 9), np.nan)
    if valid.any():
        t = raw_tags[valid].astype(int)
        m = np.cross(p[valid] - q[:3], f[valid])
        out[valid, :3] = f[valid]
        out[valid, 3:6] = m @ r
        out[valid, 6:] = (m @ hinge_axes.T) * ANCESTRY[t]
    return out, valid


def point_velocity(qpos, qvel, points, tags):
    """World rigid-body point velocity, with no imposed belt surface motion."""
    q, v = np.asarray(qpos, float), np.asarray(qvel, float)
    p, t = np.asarray(points, float), np.asarray(tags)
    if v.shape != (9,) or not np.isfinite(v).all():
        raise ValueError('finite qvel9 required')
    r, hinge_axes = axes(q)
    valid = (np.isfinite(t) & (t == np.floor(t)) & (t >= 0) & (t < 7)
             & np.isfinite(p).all(axis=1))
    out = np.full_like(p, np.nan)
    if valid.any():
        ti, arm = t[valid].astype(int), p[valid]-q[:3]
        omega = r @ v[3:6]
        angular = omega + (ANCESTRY[ti] * v[6:]) @ hinge_axes
        out[valid] = v[:3] + np.cross(angular, arm)
    return out, valid


def contact_frame(qpos, qvel, rows, held_input):
    """Map all reported force rows; slip uses positive normal-load rows only.

    Admitted=False rows are retained in the primary force map. Their net load
    and impulse are reported separately, rather than presumed to be zero.
    """
    rows, held = np.asarray(rows, float), np.asarray(held_input, float)
    if rows.ndim != 2 or rows.shape[1] != 16 or held.shape != (5,):
        raise ValueError('rows Nx16 and actual held_input5 required')
    loads, row_valid = point_force_generalized(qpos, rows[:, 1:4], rows[:, 7:10], rows[:, 0])
    velocity, velocity_valid = point_velocity(qpos, qvel, rows[:, 1:4], rows[:, 0])
    nlength = np.linalg.norm(rows[:, 4:7], axis=1)
    nvalid = np.isfinite(rows[:, 4:7]).all(axis=1) & (nlength > 0)
    normals = np.full((len(rows), 3), np.nan)
    normals[nvalid] = rows[nvalid, 4:7] / nlength[nvalid, None]
    pressure = np.full(len(rows), np.nan)
    pressure[nvalid] = np.maximum(0., np.sum(rows[nvalid, 7:10]*normals[nvalid], axis=1))
    valid_slip = row_valid & velocity_valid & nvalid & np.isfinite(held).all()
    for tag, channel in ((2, 3), (3, 4)):
        selected = valid_slip & (rows[:, 0] == tag)
        if selected.any():
            y = rotation(np.asarray(qpos)[3:7])[:, 1]
            n = normals[selected]
            velocity[selected] += held[channel] * (y - (n @ y)[:, None]*n)
    tangent = np.full_like(velocity, np.nan)
    tangent[valid_slip] = (velocity[valid_slip]
                          - normals[valid_slip]*np.sum(normals[valid_slip]*velocity[valid_slip], axis=1)[:, None])
    speeds = np.linalg.norm(tangent, axis=1)
    positive = valid_slip & (pressure > 0)
    weight = float(np.sum(pressure[positive]))
    family_load = np.zeros((5, 9))
    family_pressure = np.zeros(5)
    for tag in range(7):
        selected = row_valid & (rows[:, 0] == tag)
        family_load[TAG_FAMILY[tag]] += np.sum(loads[selected], axis=0)
        selected_pressure = selected & nvalid
        family_pressure[TAG_FAMILY[tag]] += np.sum(pressure[selected_pressure])
    admitted = np.isfinite(rows[:, 12]) & (rows[:, 12] >= .5)
    force_valid = bool(row_valid.all())
    out = dict(generalized=np.sum(loads, axis=0) if force_valid else np.full(9, np.nan),
               generalized_by_family=family_load, force_map_valid=force_valid,
               row_count=len(rows), invalid_force_row_count=int((~row_valid).sum()),
               rejected_row_count=int((~admitted).sum()),
               rejected_generalized=np.sum(loads[row_valid & ~admitted], axis=0),
               pressure_N=weight, pressure_by_family_N=family_pressure,
               row_generalized=loads, row_valid=row_valid, row_pressure_N=pressure,
               row_slip_speed_m_s=speeds, row_slip_velocity_m_s=tangent,
               positive_load_contact_count=int(positive.sum()),
               normal_length_max_error=float(np.max(np.abs(nlength[nvalid]-1))) if nvalid.any() else None,
               slip_valid=bool(weight > 0 and force_valid and nvalid.all()
                               and np.isfinite(held).all()))
    if weight > 0:
        w, vt = pressure[positive], tangent[positive]
        mean = np.sum(w[:, None]*vt, axis=0)/weight
        nsum = np.sum(w[:, None]*normals[positive], axis=0)/weight
        resultant = np.linalg.norm(nsum)
        out.update(slip_mean_vector_m_s=mean, slip_mean_norm_m_s=float(np.linalg.norm(mean)),
                   slip_rms_m_s=float(np.sqrt(np.sum(w*speeds[positive]**2)/weight)),
                   slip_max_m_s=float(np.max(speeds[positive])),
                   normal_resultant_length=float(resultant),
                   mean_normal=nsum/resultant if resultant > 0 else np.full(3, np.nan))
    else:
        out.update(slip_mean_vector_m_s=np.full(3, np.nan), slip_mean_norm_m_s=np.nan,
                   slip_rms_m_s=np.nan, slip_max_m_s=np.nan,
                   normal_resultant_length=np.nan, mean_normal=np.full(3, np.nan))
    return out


def reduce_interval(native_loads, step_dt_s, qpos0, qpos1):
    """Duration-weighted native loads followed by the observer frame rotation."""
    a, dt = np.asarray(native_loads, float), np.asarray(step_dt_s, float)
    if a.ndim < 2 or a.shape[0] != len(dt) or a.shape[-1] != 9:
        raise ValueError('first axis must match step durations; last axis must be9')
    if not np.isfinite(a).all() or not np.isfinite(dt).all() or np.any(dt <= 0):
        raise ValueError('finite native loads and positive durations required')
    shape = (len(dt),) + (1,)*(a.ndim-1)
    mean = np.sum(a*dt.reshape(shape), axis=0)/np.sum(dt)
    return world_rows(mean, middle_rotation(qpos0, qpos1))


def vector_stats(values, mask=None):
    """Unit-agnostic component/vector summaries, excluding only nonfinite rows."""
    a = np.asarray(values, float)
    if a.ndim != 2:
        raise ValueError('NxD vectors required')
    m = np.isfinite(a).all(axis=1)
    if mask is not None:
        m &= np.asarray(mask, bool)
    if not m.any():
        return dict(count=0, vector_rms=None, vector_p95=None, vector_max=None,
                    component_rms=None, component_mean=None)
    x, norm = a[m], np.linalg.norm(a[m], axis=1)
    return dict(count=int(m.sum()), vector_rms=float(np.sqrt(np.mean(norm**2))),
                vector_p95=float(np.percentile(norm, 95)), vector_max=float(norm.max()),
                component_rms=np.sqrt(np.mean(x*x, axis=0)).tolist(),
                component_mean=np.mean(x, axis=0).tolist())


def decomposition(observer, interval_proxy, endpoint_model):
    """Exact vector identity with cross-cancellation retained explicitly."""
    mu, contact, endpoint = (np.asarray(x, float) for x in (observer, interval_proxy, endpoint_model))
    a, b = mu-contact, contact-endpoint
    total = mu-endpoint
    return dict(observer_minus_interval=a, interval_minus_endpoint=b,
                observer_minus_endpoint=total,
                identity_error=total-a-b,
                norm_squared_cross_term=2*np.sum(a*b, axis=-1))


def intrinsic_normal(force, moment, radius):
    """The repository's exact equivalent-contact normal construction."""
    force, moment = np.asarray(force, float), np.asarray(moment, float)
    squared = float(force @ force)
    if not np.isfinite(force).all() or not np.isfinite(moment).all() or squared < 1e-12:
        return np.full(3, np.nan)
    lateral = -np.cross(force, moment)/(radius*squared)
    excess = float(lateral @ lateral)
    if excess >= 1.:
        return lateral/np.sqrt(excess)
    normal = np.sqrt(1.-excess)*force/np.sqrt(squared)+lateral
    return normal/np.linalg.norm(normal)


def angle_deg(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    norm = np.linalg.norm(a)*np.linalg.norm(b)
    return float(np.degrees(np.arccos(np.clip((a @ b)/norm, -1., 1.)))) if norm > 0 else np.nan
