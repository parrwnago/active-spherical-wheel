"""I58 saved-array metrics. No controller, estimator, model or physics imports."""
from copy import deepcopy
import numpy as np


def command_continuity(raw):
    # Each emitted command except the final one is actually held. Include the
    # first emission relative to its actual prior input, not a fictitious zero.
    delta = (raw['exact_applied']-raw['exact_previous_applied'])[:-1]
    caps = raw['exact_input_bounds'][:-1]
    norms = np.linalg.norm(delta/caps, axis=1)
    return dict(held_samples=len(norms),
        normalized_step_norm_RMS=float(np.sqrt(np.mean(norms**2))) if len(norms) else None,
        normalized_step_norm_p99=float(np.quantile(norms, .99)) if len(norms) else None,
        normalized_step_norm_max=float(np.max(norms)) if len(norms) else None,
        definition='Caps-normalized five-input applied step L2 norm; first actual prior included, final unheld emission excluded. Secondary diagnostic, not a selectable benefit gate.')


def priority_metrics(governor, candidate):
    if not candidate:
        return dict(available=False, reason='Native clipped command is its own nominal anchor; no optional model evaluation.')
    first = np.asarray([r['diagnostics']['first_stage_balance_delta'] for r in governor], float)
    secondary = np.asarray([r['diagnostics']['secondary_balance_delta'] for r in governor], float)
    total = first+secondary
    held_norm = np.linalg.norm(total[:-1], axis=1)
    return dict(available=True, all_finite=bool(np.isfinite(total).all()),
        total_response_max_abs=float(np.max(abs(total), initial=0.)),
        total_response_peak_norm=float(np.max(np.linalg.norm(total, axis=1), initial=0.)),
        total_response_held_RMS_norm=float(np.sqrt(np.mean(held_norm**2))) if len(held_norm) else None,
        scope='G20[3:5]*(final_applied-clipped_native_total). First-stage and secondary deltas SUMMED. Max includes all emissions; RMS excludes terminal unheld command. Local estimated response, not stability proof.')


def case_metrics(raw, summary, provenance, governor, protocol, program, events,
                 shared, legacy, event_helper):
    # The immutable I56 metric library distinguishes only native/governor. This
    # copy adapts that label for metric dispatch; original case identity and
    # provenance are retained unchanged in the I58 report/portable metadata.
    compatible_provenance = deepcopy(provenance)
    candidate = provenance['case']['variant'] != 'native'
    compatible_provenance['case']['variant'] = 'handoff' if candidate else 'native'
    compatible_protocol = deepcopy(protocol)
    compatible_protocol['candidate_settings'] = protocol['adapters']['balance_priority']['settings']
    row, signals, geometry = shared.case_metrics(raw, {}, summary,
        compatible_provenance, governor, compatible_protocol, program, events, legacy, event_helper)
    row['priority'] = priority_metrics(governor, candidate)
    row['continuity'] = command_continuity(raw)
    row['commands']['scope'] = ('Amplitude checks apply to every emitted total command. Rate comparisons use the old 60/6 diagnostic thresholds; they are hard constraints only for A, optional diagnostics for P and B. Held exposure excludes terminal command.')
    row['metric_adapter'] = 'Only a copied native/governor label and metric settings were supplied to frozen I56 pure math; actual I58 identity/metadata were not mutated.'
    return row, signals, geometry


def gate_comparison(candidate, baseline, limits, hard_slew=None):
    if not candidate.get('available') or not baseline.get('available'):
        return dict(available=False, passes_all_conditions=False, reason='Required sealed case unavailable')
    c, b = candidate, baseline
    checks = dict(
        full44_no_failure=bool(c['completed'] and c['failure'] is None and c['full44_coverage']),
        baseline_full44_no_failure=bool(b['completed'] and b['failure'] is None and b['full44_coverage']),
        baseline_historical_exact=bool((b.get('historical_parity') or {}).get('all_shared_fields_exact')),
        moving_and_late_coverage=bool(c['moving_coverage'] and b['moving_coverage'] and c['motion']['late']['complete'] and b['motion']['late']['complete']),
        late_heading_valid=bool(c['motion']['late']['all_heading_samples_valid'] and b['motion']['late']['all_heading_samples_valid']),
        same_frozen_bank=c['fixed_bank_numeric_sha256'] == b['fixed_bank_numeric_sha256'],
        finite_actual_history=bool(c['commands']['all_finite'] and c['commands']['actual_previous_input_matches']),
        hard_total_amplitude=bool(c['commands']['applied_box_pass'] and c['commands']['requested_box_pass']),
        exact_nominal_transverse_response=bool(c['priority'].get('available') and c['priority']['all_finite'] and c['priority']['total_response_max_abs'] <= limits['balance_response_tolerance']),
        secondary_equality=bool(c['optimizer']['equality_all_finite'] and c['optimizer']['secondary_balance_extra_all_finite'] and c['optimizer']['equality_residual_max'] <= limits['balance_response_tolerance'] and c['optimizer']['secondary_balance_extra_max'] <= limits['balance_response_tolerance']))

    def le(name, value, bound):
        checks[name] = bool(value is not None and bound is not None and np.isfinite(value) and np.isfinite(bound) and value <= bound+1e-12)

    ct, bt = c['timed_tracking'], b['timed_tracking']
    ca, ba = c['attitude'], b['attitude']
    cy, by = c['motion']['late'], b['motion']['late']
    le('moving_timed_XY_RMS', ct['moving_rms_m'], limits['timed_xy_rms_ratio_max']*bt['moving_rms_m']+limits['timed_xy_rms_add_m'])
    le('full_timed_XY_peak', ct['full_peak_at44_m'], None if bt['full_peak_at44_m'] is None else bt['full_peak_at44_m']+limits['timed_xy_peak_add_m'])
    le('final_timed_XY', ct['final_at44_m'], None if bt['final_at44_m'] is None else bt['final_at44_m']+limits['final_xy_add_m'])
    le('raw_lean_peak', ca['peak_raw_reference_error_deg'], ba['peak_raw_reference_error_deg']+limits['relative_lean_peak_add_deg'])
    le('raw_lean_RMS', ca['rms_raw_reference_error_deg'], limits['relative_lean_rms_ratio_max']*ba['rms_raw_reference_error_deg']+limits['relative_lean_rms_add_deg'])
    le('absolute_tilt_peak', ca['peak_absolute_deg'], ba['peak_absolute_deg']+limits['absolute_tilt_peak_add_deg'])
    le('late_yaw_peak', cy['world_z_peak_abs_rad_s'], limits['yaw_peak_abs_ratio_max']*by['world_z_peak_abs_rad_s']+limits['yaw_peak_abs_add_rad_s'])
    le('late_absolute_heading_travel', cy['heading_absolute_travel_rad'], limits['heading_absolute_travel_ratio_max']*by['heading_absolute_travel_rad']+limits['heading_absolute_travel_add_rad'])
    le('full_planar_speed_peak', c['motion']['full']['planar_speed_peak_m_s'], limits['planar_speed_peak_ratio_max']*b['motion']['full']['planar_speed_peak_m_s']+limits['planar_speed_peak_add_m_s'])
    le('full_50ms_linear_acceleration_peak', c['motion']['full']['linear_acceleration50ms']['peak_norm'], limits['linear_acceleration50ms_peak_ratio_max']*b['motion']['full']['linear_acceleration50ms']['peak_norm']+limits['linear_acceleration50ms_peak_add_m_s2'])
    turn_changes = []
    for x, y in zip(c['turns'], b['turns']):
        assert x['event_id'] == y['event_id']
        u, v = x['geometric_component_overshoot_m'], y['geometric_component_overshoot_m']
        passed = bool(x['window_complete'] and y['window_complete'] and u is not None and v is not None and u <= v+limits['each_turn_overshoot_add_m']+1e-12)
        turn_changes.append(dict(event_id=x['event_id'], passed=passed,
            geometric_overshoot_change_m=None if u is None or v is None else u-v,
            candidate_reversal_lag_s=x['actual_reversal_lag_s'], baseline_reversal_lag_s=y['actual_reversal_lag_s']))
    checks['all_six_turn_overshoots'] = len(turn_changes) == 6 and all(x['passed'] for x in turn_changes)
    checks['conditional_original_settling'] = bool(not b['original_settling']['passed'] or c['original_settling']['passed'])
    if hard_slew is not None:
        a = hard_slew
        checks['hard_slew_comparator_available_and_exact'] = bool(a.get('available') and a['completed'] and a['failure'] is None and a['full44_coverage'] and (a.get('historical_parity') or {}).get('all_shared_fields_exact'))
        if a.get('available'):
            checks['hard_slew_comparator_rate_box'] = bool(a['commands']['actual_slew_pass'])
            checks['nontrivial_old_priority_loss'] = bool(a['priority']['all_finite'] and a['priority']['total_response_peak_norm'] >= limits['old_A_priority_peak_min_rad_s'])
            le('primary_priority_restoration_vs_A', c['priority']['total_response_held_RMS_norm'], limits['priority_RMS_ratio_vs_A_max']*a['priority']['total_response_held_RMS_norm'])
        else:
            checks['primary_priority_restoration_vs_A'] = False
    return dict(available=True, passes_all_conditions=all(checks.values()), checks=checks,
        failed_checks=[k for k,v in checks.items() if not v], turn_changes=turn_changes,
        continuity_is_secondary_only=True,
        scope='Prespecified deterministic screen. Primary benefit is local response priority restoration versus A, not physical stability proof. Motion nonregression is against B; no automatic default adoption.')
