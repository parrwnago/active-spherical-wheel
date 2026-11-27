"""Synthetic contracts only. Never reads I56 outcomes."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

p=Path(__file__).with_name('reduce_handoff.py')
spec=importlib.util.spec_from_file_location('_metric_check',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def raw(n=31):
    t=np.arange(n)*.005;q=np.zeros((n,10));q[:,3]=1
    u=np.zeros((n,5))
    return dict(time_s=t,exact_qpos=q,exact_qvel=np.zeros((n,9)),exact_applied=u.copy(),
        exact_previous_applied=u.copy(),exact_input_bounds=np.tile(m.BOUNDS,(n,1)),
        exact_requested=u.copy(),exact_native_requested=u.copy())


def test_world_axis_and_heading_are_distinct_under_tilt():
    r=raw();a=np.pi/3;r['exact_qpos'][:,3]=np.cos(a/2);r['exact_qpos'][:,4]=np.sin(a/2)
    r['exact_qvel'][:,4]=1
    sig=m.signals(r)
    assert np.allclose(sig['world_omega_rad_s'][:,2],np.sin(a))
    assert np.allclose(sig['heading_rate_rad_s'],np.sin(a))


def test_heading_unwrap_and_absolute_travel():
    r=raw();a=np.linspace(3,3.3,len(r['time_s']))
    r['exact_qpos'][:,3]=np.cos(a/2);r['exact_qpos'][:,6]=np.sin(a/2)
    sig=m.signals(r);window=m.motion_window(r,sig,[0,r['time_s'][-1]])
    assert window['heading_absolute_travel_rad']==pytest.approx(.3)
    assert window['heading_net_travel_rad']==pytest.approx(.3)


def test_no_heading_bridge_through_vertical_x_axis():
    r=raw(3);r['exact_qpos'][1,3]=np.sqrt(.5);r['exact_qpos'][1,5]=np.sqrt(.5)
    r['exact_qpos'][2,3]=np.cos(.5);r['exact_qpos'][2,6]=np.sin(.5)
    sig=m.signals(r);window=m.motion_window(r,sig,[0,.01])
    assert not window['all_heading_samples_valid']
    assert window['heading_absolute_travel_rad']==0


def test_interval_acceleration_is_not_instantaneous():
    r=raw();r['exact_qvel'][:,0]=3*r['time_s']
    sig=m.signals(r)
    assert np.isnan(sig['linear_acceleration5ms_m_s2'][0]).all()
    assert np.allclose(sig['linear_acceleration5ms_m_s2'][1:,0],3)
    assert np.allclose(sig['linear_acceleration50ms_m_s2'][10:,0],3)


def test_first_and_terminal_slew_checked_and_terminal_exposure_excluded():
    r=raw(2);r['exact_applied'][0,0]=.31;r['exact_previous_applied'][1]=r['exact_applied'][0]
    r['exact_applied'][1,0]=.62;r['exact_requested']=r['exact_applied'].copy()
    row=m.command_constraints(r,[60,60,60,6,6])
    assert row['slew_max_violation']==pytest.approx(.01)
    assert not row['actual_slew_pass']
    assert row['held_samples']==1
    assert row['maximum_emitted_step_by_input'][0]==pytest.approx(.31)


def test_native_infeasible_demand_distinct_from_final_request():
    r=raw(2);r['exact_native_requested'][0,0]=10
    row=m.command_constraints(r,[60,60,60,6,6])
    assert row['native_infeasible_held_fraction']==1
    assert row['requested_box_pass'] and row['applied_box_pass']
    assert row['final_requested_outside_held_fraction']==0


def test_original_raw_target_attitude_does_not_use_absolute_tilt():
    legacy=m.pure_module(m.LEGACY,'_test_legacy');r=raw()
    a=.2;r['exact_qpos'][:,3]=np.cos(a/2);r['exact_qpos'][:,4]=np.sin(a/2)
    r['exact_reference_qpos']=r['exact_qpos'].copy()
    absolute,relative=legacy.attitude(r)
    assert np.allclose(absolute,a)
    assert np.allclose(relative,0)


def test_partial_availability_cannot_pass_comparison():
    row=m.gate_comparison(dict(available=False),dict(available=False),{})
    assert not row['available'] and not row['passes_all_conditions']
