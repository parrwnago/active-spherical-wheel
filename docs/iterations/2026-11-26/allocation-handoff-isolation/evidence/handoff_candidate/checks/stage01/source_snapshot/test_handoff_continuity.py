"""Targeted pure contracts; no plant, native controller, or dynamics is run."""
import copy
import importlib.util
from pathlib import Path

import numpy as np
import pytest

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('handoff_candidate',HERE/'handoff_continuity.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
BeltHandoffContinuity=module.BeltHandoffContinuity
SHELL=np.array([0.,20.,0.,0.]);BELT=np.array([0.,0.,0.,20.])

def command(belt_B=0.,belt_A=0.):return np.array([.4,-.2,.5,belt_A,belt_B])

def test_first_call_initializes_without_synthetic_transfer_and_copies_inputs():
    h=BeltHandoffContinuity();native=command(.6);previous=command(-.6);n0=native.copy();p0=previous.copy();loads=BELT.copy()
    out,d=h.apply(native,previous,loads,.005,.6,0.)
    np.testing.assert_array_equal(out,native);assert not d['triggered'].any()and not d['episode_active'].any()
    np.testing.assert_array_equal(native,n0);np.testing.assert_array_equal(previous,p0);np.testing.assert_array_equal(loads,BELT)
    out[:]=0;d['state_after']['previous_loaded'][:]=False
    assert h.state_dict()['previous_loaded'][1]

def test_first_real_entry_limits_total_belt_command_and_preserves_torques():
    h=BeltHandoffContinuity();h.apply(command(),command(),SHELL,.005,.6,0.)
    native=command(-.3033613807195354);previous=command(-.012112105614942088)
    out,d=h.apply(native,previous,BELT,.005,.6,.005)
    assert d['triggered'].tolist()==[False,True]and d['limited'].tolist()==[False,True]
    assert abs(out[4]-(-.027112105614942088))<1e-15
    np.testing.assert_array_equal(out[:3],native[:3]);assert abs(out[4]-previous[4])<=.015+1e-15

def test_exit_and_signed_reversal_use_previous_actually_applied():
    h=BeltHandoffContinuity();h.apply(command(.3),command(.3),BELT,.005,.3,0.)
    native=command(-.3);out,d=h.apply(native,command(.3),SHELL,.005,.3,.005)
    assert d['triggered'][1]and d['limited'][1];assert out[4]==.285
    np.testing.assert_array_equal(out[:3],native[:3]);assert out[4]>0

def test_operating_cap_intersection_and_both_belts_are_independent():
    h=BeltHandoffContinuity();h.apply(command(),command(),SHELL,.005,[.6,.3],0.)
    native=command(.3,-.6);previous=command(.295,-.595)
    loads=np.array([0.,0.,10.,10.]);out,d=h.apply(native,previous,loads,.005,[.6,.3],.005)
    np.testing.assert_array_equal(out,native);assert d['triggered'].tolist()==[True,True]
    assert d['completed'].all()and not d['episode_after'].any()
    assert np.all(abs(out[3:])<=[.6,.3])

def test_external_applied_history_prevents_target_integrator_windup():
    h=BeltHandoffContinuity();h.apply(command(),command(),SHELL,.005,.3,0.)
    for k in range(1,5):
        # Another permitted owner overrides the prior output to0; anchor every
        # new envelope at that measured/applied0, never at an internal target.
        out,d=h.apply(command(.3),command(0.),BELT,.005,.3,k*.005)
        assert out[4]==.015 and d['limited'][1]

def test_catchup_ends_immediately_and_stable_mode_commands_pass_unchanged():
    h=BeltHandoffContinuity();h.apply(command(),command(),SHELL,.005,.3,0.)
    first,d=h.apply(command(.03),command(),BELT,.005,.3,.005);assert first[4]==.015 and d['episode_after'][1]
    second,d=h.apply(command(.03),first,BELT,.005,.3,.01);assert second[4]==.03 and d['completed'][1]
    native=command(-.3);third,d=h.apply(native,second,BELT,.005,.3,.015)
    np.testing.assert_array_equal(third,native);assert not d['episode_active'].any()
    # No minimum dwell, changed trajectory, held filtered target or permanent
    # actuator model is introduced after this transfer episode ends.

def test_exact_five_percent_boundary_chatter_and_reset():
    h=BeltHandoffContinuity();below=np.array([0.,.951,0.,.049]);at=np.array([0.,.95,0.,.05])
    h.apply(command(),command(),below,.005,.3,0.)
    out,d=h.apply(command(.3),command(),at,.005,.3,.005);assert d['loaded'][1]and d['triggered'][1]
    out,d=h.apply(command(),out,below,.005,.3,.01);assert not d['loaded'][1]and d['triggered'][1]
    h.reset();assert h.state_dict()is None
    out,d=h.apply(command(.3),command(),at,.005,.3,0.)
    assert out[4]==.3 and d['initialized_now']and not d['triggered'].any()

def test_invalid_input_or_clock_rejected_without_state_mutation():
    h=BeltHandoffContinuity();h.apply(command(),command(),SHELL,.005,.3,1.)
    before=copy.deepcopy(h.state_dict())
    invalid=[dict(native_applied=command(.301)),dict(previous_applied=command(-.301)),dict(estimated_loads=[0,-1,0,1]),dict(dt=0.),dict(cap=0.),dict(time=1.)]
    for replacement in invalid:
        args=dict(native_applied=command(),previous_applied=command(),estimated_loads=SHELL,dt=.005,cap=.3,time=1.005);args.update(replacement)
        with pytest.raises(ValueError):h.apply(**args)
        after=h.state_dict();assert after['last_time_s']==before['last_time_s']
        np.testing.assert_array_equal(after['previous_loaded'],before['previous_loaded']);np.testing.assert_array_equal(after['episode_active'],before['episode_active'])
