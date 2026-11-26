"""Independent pure mathematics and causal-state checks for the I43 tilt reference.

No robot plant or contact solver is constructed. Rotation derivatives use an
independent Rodrigues matrix calculation; the ODE comparison uses DOP853.
"""
from __future__ import annotations

import copy
import numpy as np
import pytest
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

from spherical_wheel import tilt_reference as tilt


TAU=.08
RATE=np.radians(60.)
LIMIT=np.radians(28.)


def _generator(**overrides):
    settings=tilt.TiltReferenceSettings(time_constant_s=TAU,max_rate_deg_s=60.,
        activation_time_s=4.,use_rate_reference=True,**overrides)
    return tilt.TiltReference(settings,max_tilt_rad=LIMIT)


def _rotation(vector):
    vector=np.asarray(vector,float);angle=np.linalg.norm(vector)
    x,y,z=vector
    skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    if angle<1e-8:
        return np.eye(3)+(1-angle**2/6)*skew+(.5-angle**2/24)*(skew@skew)
    return np.eye(3)+np.sin(angle)/angle*skew+(1-np.cos(angle))/angle**2*(skew@skew)


def _quat(matrix):
    x,y,z,w=Rotation.from_matrix(matrix).as_quat()
    return np.array([w,x,y,z])


def _matrix(quat):
    w,x,y,z=quat
    return Rotation.from_quat([x,y,z,w]).as_matrix()


def _lift(theta,equilibrium,yaw):
    return equilibrium@_rotation(np.r_[theta,0.])@_rotation([0.,0.,yaw])


def _reference_velocity(theta,rate,equilibrium,actual):
    # The module returns equilibrium-frame angular velocity. The controller's
    # actual-body transport is independently reviewed/integration-tested.
    return actual.T@equilibrium@tilt.tilt_angular_velocity(theta,rate)


def _independent_flow(vector):
    vector=np.asarray(vector,float);norm=np.linalg.norm(vector)
    return vector/TAU if norm<1e-14 else RATE*np.tanh(norm/(RATE*TAU))*vector/norm


@pytest.mark.parametrize('theta',[[0.,0.],[1e-10,-3e-10],[.2,-.15],[.4,.2]])
@pytest.mark.parametrize('yaw',[0.,1.2,-2.9])
def test_lift_and_body_rate_match_independent_rotation_derivative(theta,yaw):
    theta=np.asarray(theta);rate=np.array([.37,-.22])
    equilibrium=_rotation([.08,-.03,.16]);actual=_rotation([-.13,.2,-.9])
    epsilon=1e-6
    center=_lift(theta,equilibrium,yaw)
    derivative=(_lift(theta+epsilon*rate,equilibrium,yaw)-_lift(theta-epsilon*rate,equilibrium,yaw))/(2*epsilon)
    skew=derivative@center.T
    world=np.array([skew[2,1]-skew[1,2],skew[0,2]-skew[2,0],skew[1,0]-skew[0,1]])/2
    body=_reference_velocity(theta,rate,equilibrium,actual)
    np.testing.assert_allclose(actual@body,world,atol=2e-9,rtol=0.)
    generated=equilibrium@_matrix(tilt.tilt_quaternion(theta))
    np.testing.assert_allclose(generated,equilibrium@_rotation(np.r_[theta,0.]),atol=2e-14,rtol=0.)


def test_changing_tilt_direction_keeps_geometric_angular_z():
    theta=np.array([.3,0.]);rate=np.array([0.,.4])
    body=_reference_velocity(theta,rate,np.eye(3),np.eye(3))
    assert body[2]>.05
    assert np.linalg.norm(body)<=np.linalg.norm(rate)+1e-14


def test_measured_yaw_change_does_not_add_a_desired_yaw_rate():
    theta=np.array([.2,-.1]);equilibrium=_rotation([.03,-.04,.1])
    for yaw in (-3.,-1.,0.,2.8):
        actual=_rotation([0.,0.,yaw])
        np.testing.assert_array_equal(_reference_velocity(theta,np.zeros(2),equilibrium,actual),np.zeros(3))


def test_previous_held_target_and_same_timestamp_are_causal():
    generator=_generator()
    target=np.array([.3,-.1])
    initial=generator.update(np.zeros(2),4.)
    now=generator.update(target,4.005)
    np.testing.assert_array_equal(now['theta_rad'],initial['theta_rad'])
    np.testing.assert_array_equal(now['theta_dot_rad_s'],initial['theta_dot_rad_s'])
    assert np.linalg.norm(now['theta_ddot_rad_s2'])>0.
    next_row=generator.update(target,4.010)
    assert np.linalg.norm(next_row['theta_rad'])>0.
    before=copy.deepcopy(generator)
    duplicate=generator.update(-target,4.010)
    np.testing.assert_array_equal(duplicate['theta_rad'],next_row['theta_rad'])
    np.testing.assert_array_equal(duplicate['theta_dot_rad_s'],next_row['theta_dot_rad_s'])
    continued=generator.update(target,4.015)
    expected=before.update(target,4.015)
    for key in ('theta_rad','theta_dot_rad_s','theta_ddot_rad_s2','z_rad','held_raw_theta_rad'):
        np.testing.assert_array_equal(continued[key],expected[key])


def test_cascade_remains_in_disk_and_satisfies_chart_bounds():
    generator=_generator();time=4.
    generator.update(np.zeros(2),time)
    previous=None
    for index in range(300):
        dt=(.005,.0017,.0063)[index%3];time+=dt
        phase=index*1.73
        raw=LIMIT*np.array([np.cos(phase),np.sin(phase)])
        row=generator.update(raw,time)
        assert np.linalg.norm(row['theta_rad'])<=LIMIT+2e-14
        assert np.linalg.norm(row['z_rad'])<=LIMIT+2e-14
        assert np.linalg.norm(row['theta_dot_rad_s'])<=RATE+2e-14
        assert np.linalg.norm(row['theta_ddot_rad_s2'])<=2*RATE/TAU+2e-12
        if previous is not None:
            # Step bounds concern the generated reference, not measured-frame rotation.
            assert np.linalg.norm(row['theta_rad']-previous['theta_rad'])<=RATE*dt+2e-13
            assert np.linalg.norm(row['theta_dot_rad_s']-previous['theta_dot_rad_s'])<=2*RATE/TAU*dt+2e-12
        previous=row


def test_ssp_solution_matches_independent_high_accuracy_ode():
    generator=_generator();generator.update(np.zeros(2),4.)
    target=np.array([.3,-.2]);generator.update(target,4.005)
    def derivative(_,state):
        z,theta=state[:2],state[2:]
        return np.r_[_independent_flow(target-z),_independent_flow(z-theta)]
    times=np.array([4.045,4.085,4.205,4.805])
    exact=solve_ivp(derivative,[4.005,times[-1]],np.zeros(4),method='DOP853',
        rtol=1e-12,atol=1e-14,t_eval=times)
    assert exact.success
    for i,time in enumerate(times):
        row=generator.update(target,float(time))
        np.testing.assert_allclose(np.r_[row['z_rad'],row['theta_rad']],exact.y[:,i],atol=5e-6,rtol=0.)
    final=generator.update(target,7.)
    np.testing.assert_allclose(final['theta_rad'],target,atol=1e-10,rtol=0.)
    assert np.linalg.norm(final['theta_dot_rad_s'])<1e-9


def test_activation_reset_and_returned_arrays_do_not_alias_state():
    generator=_generator();raw=np.array([.2,.1])
    inactive=generator.update(raw,3.995)
    assert not inactive['active']
    first=generator.update(raw,4.)
    np.testing.assert_array_equal(first['theta_rad'],raw)
    np.testing.assert_array_equal(first['theta_dot_rad_s'],np.zeros(2))
    first['theta_rad'][:]=99.
    intact=generator.update(raw,4.)
    np.testing.assert_array_equal(intact['theta_rad'],raw)
    clone=copy.deepcopy(generator);clone.update(-raw,4.005);changed=clone.update(-raw,4.010)
    original=generator.update(raw,4.010)
    assert np.linalg.norm(changed['theta_rad']-original['theta_rad'])>0.
    generator.reset()
    restarted=generator.update(-raw,4.)
    np.testing.assert_array_equal(restarted['theta_rad'],-raw)
    np.testing.assert_array_equal(restarted['theta_dot_rad_s'],np.zeros(2))


def test_nonfinite_or_backwards_time_is_rejected():
    generator=_generator();generator.update(np.zeros(2),4.)
    for time in (float('nan'),float('inf'),3.9):
        with pytest.raises(ValueError):generator.update(np.zeros(2),time)


def test_backwards_time_before_activation_is_rejected_and_reset_clears_clock():
    generator=_generator()
    assert not generator.update(np.zeros(2),3.)['active']
    with pytest.raises(ValueError):generator.update(np.zeros(2),2.9)
    assert not generator.update(np.zeros(2),3.1)['active']
    generator.reset()
    assert not generator.update(np.zeros(2),2.)['active']
