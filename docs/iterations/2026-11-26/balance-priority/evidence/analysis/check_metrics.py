"""Pure I58 metric contracts. No controller/model/physics or policy outcomes."""
from pathlib import Path
from importlib.util import spec_from_file_location,module_from_spec
from copy import deepcopy
import numpy as np
import pytest
p=Path(__file__).with_name('metrics.py');spec=spec_from_file_location('_i58_metric_checks',p);m=module_from_spec(spec);spec.loader.exec_module(m)


def test_continuity_uses_actual_first_prior_and_omits_terminal():
 u=np.zeros((3,5));previous=u.copy();u[:,0]=[.5,.6,.9];previous[:,0]=[.4,.5,.6]
 row=m.command_continuity(dict(exact_applied=u,exact_previous_applied=previous,exact_input_bounds=np.ones((3,5))))
 assert row['held_samples']==2
 assert row['normalized_step_norm_RMS']==pytest.approx(.1)
 assert row['normalized_step_norm_max']==pytest.approx(.1)


def test_priority_sums_first_and_secondary_including_cancellation():
 rows=[dict(diagnostics=dict(first_stage_balance_delta=[.3,.4],secondary_balance_delta=[-.3,-.4]))for _ in range(3)]
 row=m.priority_metrics(rows,True)
 assert row['total_response_max_abs']==0 and row['total_response_held_RMS_norm']==0
 rows[0]['diagnostics']=dict(first_stage_balance_delta=[2.,0.],secondary_balance_delta=[-1.,0.])
 row=m.priority_metrics(rows,True)
 assert row['total_response_max_abs']==1 and row['total_response_held_RMS_norm']==pytest.approx(np.sqrt(.5))


def test_priority_terminal_affects_max_but_not_held_RMS():
 rows=[dict(diagnostics=dict(first_stage_balance_delta=[0.,0.],secondary_balance_delta=[0.,0.])),
       dict(diagnostics=dict(first_stage_balance_delta=[10.,0.],secondary_balance_delta=[0.,0.]))]
 row=m.priority_metrics(rows,True)
 assert row['total_response_max_abs']==10 and row['total_response_held_RMS_norm']==0


def limits():
 keys=('timed_xy_rms_ratio_max','relative_lean_rms_ratio_max','yaw_peak_abs_ratio_max','heading_absolute_travel_ratio_max',
 'planar_speed_peak_ratio_max','linear_acceleration50ms_peak_ratio_max','priority_RMS_ratio_vs_A_max')
 out={k:1. for k in keys}
 out.update({k:0. for k in('timed_xy_rms_add_m','timed_xy_peak_add_m','final_xy_add_m','relative_lean_peak_add_deg',
 'relative_lean_rms_add_deg','absolute_tilt_peak_add_deg','yaw_peak_abs_add_rad_s','heading_absolute_travel_add_rad',
 'planar_speed_peak_add_m_s','linear_acceleration50ms_peak_add_m_s2','each_turn_overshoot_add_m')})
 out.update(balance_response_tolerance=1e-8,old_A_priority_peak_min_rad_s=.1)
 return out


def full():
 return dict(available=True,completed=True,failure=None,full44_coverage=True,moving_coverage=True,
  historical_parity=dict(all_shared_fields_exact=True),fixed_bank_numeric_sha256='fixed',
  motion=dict(late=dict(complete=True,all_heading_samples_valid=True,world_z_peak_abs_rad_s=.2,heading_absolute_travel_rad=.2),
   full=dict(planar_speed_peak_m_s=.2,linear_acceleration50ms=dict(peak_norm=.2))),
  commands=dict(all_finite=True,actual_previous_input_matches=True,applied_box_pass=True,requested_box_pass=True,actual_slew_pass=False),
  priority=dict(available=True,all_finite=True,total_response_max_abs=0.,total_response_peak_norm=0.,total_response_held_RMS_norm=0.),
  optimizer=dict(equality_all_finite=True,secondary_balance_extra_all_finite=True,equality_residual_max=0.,secondary_balance_extra_max=0.),
  timed_tracking=dict(moving_rms_m=.1,full_peak_at44_m=.2,final_at44_m=.1),
  attitude=dict(peak_raw_reference_error_deg=1.,rms_raw_reference_error_deg=.1,peak_absolute_deg=2.),
  turns=[dict(event_id=str(i),window_complete=True,geometric_component_overshoot_m=0.,actual_reversal_lag_s=0.)for i in range(6)],
  original_settling=dict(passed=False))


def test_short_failed_trace_produces_failed_screen_not_exception():
 c=full();c.update(completed=False,failure='fell',full44_coverage=False,moving_coverage=False)
 c['motion']['late']=dict(available=False,complete=False)
 c['motion']['full']['linear_acceleration50ms']=dict(available=False)
 c['timed_tracking']=dict(moving_rms_m=None,full_peak_at44_m=None,final_at44_m=None)
 r=m.gate_comparison(c,full(),limits())
 assert not r['passes_all_conditions']
 assert 'full44_no_failure'in r['failed_checks'] and 'late_heading_valid'in r['failed_checks']
 assert not r['checks']['full_50ms_linear_acceleration_peak']


def test_short_baseline_missing_RMS_and_acceleration_is_unavailable_not_exception():
 b=full();b['timed_tracking']['moving_rms_m']=None;b['motion']['full']['linear_acceleration50ms']=dict(available=False)
 r=m.gate_comparison(full(),b,limits())
 assert not r['passes_all_conditions']and not r['checks']['moving_timed_XY_RMS']


def test_P_old_rate_exceedance_is_not_a_hard_gate():
 c,b=full(),full();r=m.gate_comparison(c,b,limits())
 assert not c['commands']['actual_slew_pass']
 assert r['passes_all_conditions']


def test_secondary_equality_alone_cannot_hide_primary_priority_loss():
 c=full();c['priority']['total_response_max_abs']=.1
 r=m.gate_comparison(c,full(),limits())
 assert r['checks']['secondary_equality']and not r['checks']['exact_nominal_transverse_response']


def test_continuity_is_not_a_selectable_benefit_gate():
 c=full();c['continuity']=dict(normalized_step_norm_RMS=999.)
 r=m.gate_comparison(c,full(),limits())
 assert r['passes_all_conditions']and r['continuity_is_secondary_only']
