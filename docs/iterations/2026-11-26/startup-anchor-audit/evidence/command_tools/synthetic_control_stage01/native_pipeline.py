"""Pure NumPy I59 native pipeline; no controller, model, observer or lookup."""
import numpy as np


def _vector(value, length, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(name+' must be a finite vector of length '+str(length))
    return result


def native_stages(row, *, atol=1e-10):
    """Recompose one saved native pipeline, including its own clipping fields."""
    get = lambda name: _vector(row['exact_'+name], 5, name)
    gain = np.asarray(row['exact_feedback_gain'], dtype=float)
    error = np.asarray(row['exact_error'], dtype=float)
    if gain.ndim != 2 or gain.shape[0] != 5 or error.shape != (gain.shape[1],):
        raise ValueError('Expected gain[5,n] and error[n]')
    if not np.isfinite(gain).all() or not np.isfinite(error).all():
        raise ValueError('Finite saved gain and error required')
    equilibrium, rolling, support, lqr = (get(name) for name in
        ('equilibrium_command', 'rolling_feedforward', 'support_command', 'lqr_command'))
    transition, heading, fade_delta = (get(name) for name in
        ('transition_command_delta', 'heading_command', 'fade_command_delta'))
    weights, caps = get('sync_weights'), get('input_bounds')
    if np.any(weights < 0) or np.any(weights > 1) or np.any(caps <= 0):
        raise ValueError('Valid sync weights and positive amplitude bounds required')
    if weights[0] != 0:
        raise ValueError('Declared synchronizer never blends shaft index zero')
    pre_sync = equilibrium+rolling+support+lqr
    post_sync = pre_sync+transition
    pre_fade = post_sync+heading
    requested_reconstructed = pre_fade+fade_delta
    requested, applied, clipping = (get(name) for name in
        ('native_requested', 'native_applied', 'native_clipping_command_delta'))
    residuals = dict(lqr=float(np.max(abs(lqr+gain@error))),
        requested=float(np.max(abs(requested_reconstructed-requested))),
        applied=float(np.max(abs(np.clip(requested, -caps, caps)-applied))),
        clipping=float(np.max(abs((applied-requested)-clipping))),
        shaft_sync_delta=abs(float(transition[0])),
        belt_fade_delta=float(np.max(abs(fade_delta[3:]))))
    if max(residuals.values()) > atol:
        raise ValueError('Native pipeline closure failed: '+str(residuals))
    return dict(pre_sync=pre_sync, post_sync=post_sync, pre_fade=pre_fade,
        lqr_recorded=lqr.copy(), lqr_reconstructed=-gain@error,
        transition_delta=transition.copy(), heading=heading.copy(), fade_delta=fade_delta.copy(),
        weighted_sync_target=transition+weights*pre_sync, weights=weights.copy(),
        native_requested=requested.copy(), native_applied=applied.copy(),
        native_requested_reconstructed=requested_reconstructed, bounds=caps.copy(), residuals=residuals)


def infer_fade(stages, qvel, reference_qvel, airborne_damping_Nms,
               recorded_fade_float32, *, closure_atol=1e-10):
    """Recover a point and conservative factor interval, never rounded-exact.

    ``identifiable`` marks a source slope above closure tolerance; it does not
    assert exact scalar recovery. All counterfactuals must retain lower/upper.
    """
    qvel = _vector(qvel, 9, 'qvel')
    reference_qvel = _vector(reference_qvel, 9, 'reference_qvel')
    if not np.isfinite(airborne_damping_Nms) or airborne_damping_Nms < 0:
        raise ValueError('Finite nonnegative damping required')
    recorded = np.float32(recorded_fade_float32)
    if not np.isfinite(recorded) or not 0 <= recorded <= 1:
        raise ValueError('Finite float32 fade in [0,1] required')
    prior = np.nextafter(recorded, np.float32(-np.inf), dtype=np.float32)
    following = np.nextafter(recorded, np.float32(np.inf), dtype=np.float32)
    float32lo = max(0., .5*(float(prior)+float(recorded)))
    float32hi = min(1., .5*(float(recorded)+float(following)))
    lower, upper = float32lo, float32hi
    damping = -float(airborne_damping_Nms)*(qvel[6:9]-reference_qvel[6:9])
    a = damping-stages['pre_fade'][:3]
    b = stages['fade_delta'][:3]
    index = int(np.argmax(abs(a)))
    for slope, output in zip(a, b):
        if slope == 0:
            if abs(output) > closure_atol:
                raise ValueError('Zero fade slope with nonzero recorded delta')
            continue
        center = 1.-output/slope
        radius = closure_atol/abs(slope)
        lower, upper = max(lower, center-radius), min(upper, center+radius)
    if lower > upper:
        raise ValueError('Exact stage closure and recorded float32 fade have disjoint intervals')
    identifiable = bool(abs(a[index]) > closure_atol)
    raw_estimate = 1.-b[index]/a[index] if a[index] != 0 else float(recorded)
    estimate = float(np.clip(raw_estimate, lower, upper))
    if np.float32(estimate) != recorded:
        estimate = .5*(lower+upper)
    closure = float(np.max(abs((1.-estimate)*a-b)))
    if closure > closure_atol*1.001:
        raise ValueError('Recovered fade fails componentwise closure')
    return dict(estimate=estimate, identifiable=identifiable,
        lower=float(lower), upper=float(upper), float32lo=float32lo, float32hi=float32hi,
        interval_width=float(upper-lower), estimate_error_bound=max(estimate-lower, upper-estimate),
        closure_residual=closure, damping=damping, damping_minus_prefade=a,
        recorded_float32=float(recorded), factor_precision='bounded interval; recorded float32 is never declared exact')


def propagate_fixed_state(stages, delta_pre, fade):
    """Hold targets/weights/state/fade, update dependent deltas, then reclip."""
    delta_pre = _vector(delta_pre, 5, 'Pre-sync perturbation')
    delta_sync = (1.-stages['weights'])*delta_pre
    attenuation = np.array([fade['estimate']]*3+[1., 1.])
    attenuation_lo = np.array([fade['lower']]*3+[1., 1.])
    attenuation_hi = np.array([fade['upper']]*3+[1., 1.])
    delta_request = attenuation*delta_sync
    end_a, end_b = attenuation_lo*delta_sync, attenuation_hi*delta_sync
    delta_lower, delta_upper = np.minimum(end_a, end_b), np.maximum(end_a, end_b)
    base, caps = stages['native_requested'], stages['bounds']
    def shifted(change):
        result = base+change
        result[change == 0] = base[change == 0]
        return result
    requested, requested_lo, requested_hi = (shifted(value) for value in (delta_request, delta_lower, delta_upper))
    applied = np.clip(requested, -caps, caps)
    applied_lo, applied_hi = np.clip(requested_lo, -caps, caps), np.clip(requested_hi, -caps, caps)
    if np.all(delta_pre == 0):
        requested = base.copy()
        applied = stages['native_applied'].copy()
        requested_lo, requested_hi = base.copy(), base.copy()
        applied_lo, applied_hi = applied.copy(), applied.copy()
    return dict(native_request=requested, native_clipped=applied,
        request_lower=requested_lo, request_upper=requested_hi,
        clipped_lower=applied_lo, clipped_upper=applied_hi,
        delta_request=delta_request, delta_request_lower=delta_lower, delta_request_upper=delta_upper,
        delta_clipped=applied-stages['native_applied'],
        delta_clipped_lower=applied_lo-stages['native_applied'],
        delta_clipped_upper=applied_hi-stages['native_applied'],
        pre_sync=stages['pre_sync']+delta_pre,
        post_sync=stages['post_sync']+delta_sync, pre_fade=stages['pre_fade']+delta_sync,
        transition_delta=stages['transition_delta']-stages['weights']*delta_pre,
        fade_delta=stages['fade_delta']+(attenuation-1.)*delta_sync,
        fade_effect_independent=bool(not np.any(delta_sync[:3])),
        boundary='Native pipeline only; no P reoptimization')
