"""Pure saved-data event definitions for predeclared I41 diagnostics."""
from __future__ import annotations

import numpy as np


def true_runs(times, mask):
    times, mask = np.asarray(times, float), np.asarray(mask, bool)
    starts = np.flatnonzero(mask & ~np.r_[False, mask[:-1]])
    ends = np.flatnonzero(mask & ~np.r_[mask[1:], False])
    return [{'start_index': int(a), 'end_index': int(b), 'start_s': float(times[a]),
             'end_s': float(times[b]), 'duration_s': float(times[b]-times[a]),
             'samples': int(b-a+1)} for a, b in zip(starts, ends)]


def first_sustained(times, values, threshold, duration_s=.1, after_s=1.):
    mask = (np.asarray(times) >= after_s-1e-9) & (np.asarray(values) > threshold)
    runs = true_runs(times, mask)
    candidates = [r for r in runs if r['duration_s'] >= duration_s-1e-10]
    if not candidates:
        return None
    first = dict(candidates[0])
    first['threshold'] = float(threshold)
    first['confirmed_at_s'] = first['start_s']+duration_s
    return first


def local_projection(reference_times, reference_xy, position, nominal_time, half_window_s=2.):
    """Exact projection on a time-constrained sampled polyline, including clipped segments."""
    rt, ref = np.asarray(reference_times, float), np.asarray(reference_xy, float)
    nominal = float(np.clip(nominal_time, rt[0], rt[-1]))
    lo, hi = max(float(rt[0]), nominal-half_window_s), min(float(rt[-1]), nominal+half_window_s)
    first = max(0, int(np.searchsorted(rt, lo, side='right'))-1)
    stop = min(len(rt)-1, int(np.searchsorted(rt, hi, side='left')))
    indices = np.arange(first, stop)
    p0, p1 = ref[indices], ref[indices+1]
    t0, t1 = rt[indices], rt[indices+1]
    edge = p1-p0; length2 = np.sum(edge*edge, axis=1)
    raw = np.zeros(len(indices)); valid = length2 > 1e-24
    raw[valid] = np.sum((np.asarray(position)-p0[valid])*edge[valid], axis=1)/length2[valid]
    raw[~valid] = (nominal-t0[~valid])/(t1[~valid]-t0[~valid])
    lower = np.clip((lo-t0)/(t1-t0), 0., 1.)
    upper = np.clip((hi-t0)/(t1-t0), 0., 1.)
    ratio = np.minimum(np.maximum(raw, lower), upper)
    points = p0+ratio[:, None]*edge
    distances2 = np.sum((points-np.asarray(position))**2, axis=1)
    projected_times = t0+ratio*(t1-t0)
    ties = np.flatnonzero(distances2 <= distances2.min()+1e-20)
    chosen = int(ties[np.argmin(abs(projected_times[ties]-nominal))])
    at = float(projected_times[chosen])
    assert lo-1e-10 <= at <= hi+1e-10, (lo, at, hi)
    # The physical route endpoint is different from hitting an artificial ±2s edge.
    lower_artificial = lo > rt[0]+1e-9 and abs(at-lo) <= 1e-7
    upper_artificial = hi < rt[-1]-1e-9 and abs(at-hi) <= 1e-7
    return {'closest_time_s': at, 'closest_point_m': points[chosen],
            'distance_m': float(np.sqrt(distances2[chosen])),
            'lag_s_positive_is_behind': nominal-at,
            'artificial_time_window_boundary': bool(lower_artificial or upper_artificial),
            'at_route_endpoint': bool(abs(at-rt[0]) <= 1e-7 or abs(at-rt[-1]) <= 1e-7),
            'window_start_s': lo, 'window_end_s': hi}


def component_reversals(times, velocity, minimum_vector_speed_m_s=.005):
    """Reference component zero crossings with nonzero vector speed."""
    t, v = np.asarray(times, float), np.asarray(velocity, float)
    events = []
    for axis in range(2):
        sign = np.sign(v[:, axis])
        valid = np.flatnonzero(sign != 0)
        for old, new in zip(valid[:-1], valid[1:]):
            if sign[old] == sign[new]:
                continue
            fraction = abs(v[old, axis])/(abs(v[old, axis])+abs(v[new, axis]))
            at = float(t[old]+fraction*(t[new]-t[old]))
            vector = v[old]+fraction*(v[new]-v[old])
            if np.linalg.norm(vector) < minimum_vector_speed_m_s:
                continue
            events.append({'axis': ('x', 'y')[axis], 'axis_index': axis, 'time_s': at,
                'incoming_sign': int(sign[old]), 'outgoing_sign': int(sign[new]),
                'reference_vector_speed_m_s': float(np.linalg.norm(vector)),
                'kind': 'component_maximum' if sign[old] > 0 else 'component_minimum'})
    return sorted(events, key=lambda item: item['time_s'])


def actual_reversal_candidates(times, component_velocity, incoming_sign, outgoing_sign,
                               window_start_s, window_end_s, dwell_s=.1):
    t, v = np.asarray(times, float), np.asarray(component_velocity, float)
    runs = true_runs(t, np.sign(v) == outgoing_sign)
    result = []
    for run in runs:
        i = run['start_index']
        if not (window_start_s-1e-9 <= run['start_s'] <= window_end_s+1e-9):
            continue
        previous = i-1
        while previous >= 0 and v[previous] == 0:
            previous -= 1
        if previous < 0 or np.sign(v[previous]) != incoming_sign or run['duration_s'] < dwell_s-1e-10:
            continue
        if run['start_s']+dwell_s > window_end_s+1e-9:
            continue
        fraction = abs(v[i-1])/(abs(v[i-1])+abs(v[i]))
        at = float(t[i-1]+fraction*(t[i]-t[i-1]))
        result.append({**run, 'interpolated_zero_time_s': at,
                       'incoming_sign': int(incoming_sign), 'outgoing_sign': int(outgoing_sign),
                       'zero_velocity_plateau_s': [float(t[previous+1]), float(t[i-1])] if i-previous > 1 else None})
    return result


def arithmetic_checks():
    t = np.arange(5)*.005
    assert first_sustained(t[:3], np.ones(3), .5, .015, 0.) is None
    assert first_sustained(t[:4], np.ones(4), .5, .015, 0.) is not None
    # Global geometry could select the old horizontal branch, local time selects the current diagonal.
    rt = np.arange(7, dtype=float)
    ref = np.array([[-1, 0], [0, 0], [1, 0], [1, 1], [0, 0], [-1, -1], [-2, -2]], float)
    p = local_projection(rt, ref, [.05, 0.], 4., .7)
    assert 3.3 <= p['closest_time_s'] <= 4.7
    assert p['distance_m'] > .03
    straight = local_projection([0., 1., 2.], [[0., 0.], [1., 0.], [2., 0.]], [.5, .3], 1., 2.)
    assert abs(straight['distance_m']-.3) < 1e-12
    assert abs(straight['closest_time_s']-.5) < 1e-12
    # The adversarial future point must not select a segment beyond the upper time bound.
    future = local_projection([0, 1, 2, 3, 4], [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]], [2, 0], 1., .7)
    assert abs(future['closest_time_s']-1.7) < 1e-12 and future['artificial_time_window_boundary']
    # Artificial boundary is labeled rather than called a pure cross-track projection.
    p = local_projection(rt[:3], ref[:3], [-.9, 0.], 2., .2)
    assert p['artificial_time_window_boundary']
    # A component can reverse while total speed remains nonzero.
    times = np.array([0., 1., 2.]); velocities = np.array([[1., 1.], [0., 1.], [-1., 1.]])
    reversals = component_reversals(times, velocities)
    assert len(reversals) == 1 and reversals[0]['reference_vector_speed_m_s'] == 1.
    t = np.arange(21)*.02; velocity = np.r_[np.ones(5), -np.ones(16)]
    candidates = actual_reversal_candidates(t, velocity, 1, -1, 0., .4, .1)
    assert len(candidates) == 1 and abs(candidates[0]['interpolated_zero_time_s']-.09) < 1e-12
    assert not actual_reversal_candidates(t[:8], velocity[:8], 1, -1, 0., .4, .1)
    zero_times = np.arange(0., .31, .01)
    zero_velocity = np.r_[np.ones(10), 0., -np.ones(20)]
    through_zero = actual_reversal_candidates(zero_times, zero_velocity, 1, -1, 0., .3, .1)
    assert len(through_zero) == 1 and abs(through_zero[0]['interpolated_zero_time_s']-.1) < 1e-12
    return {'passed': True, 'scope': 'Duration spans, branch-constrained projection, boundary labeling, component reversal semantics; no physics.'}
