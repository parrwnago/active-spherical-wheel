"""Focused geometry, helper-policy and adapter-boundary contracts; no physics."""
from types import SimpleNamespace as NS
from unittest.mock import patch
import unittest
import numpy as np
from spherical_wheel.offroad_control import OffroadController,OffroadParams
from tangent_heading_adapter import (TangentHeadingAdapter,euler_heading_rate,helper_solution,wrap)


def Rxyz(roll,pitch,yaw):
    cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def fixture(omega=(0.,0.,.05),rotation=None,vertical=1.,loads=None):
    body=np.eye(3) if rotation is None else rotation
    # A rotation with prescribed shell-axis world vertical component.
    a=np.arcsin(vertical);c,s=np.cos(a),np.sin(a)
    carrier=np.array([[c,0.,-s],[0.,1.,0.],[s,0.,c]])
    plant=NS(model=object(),data=NS(xmat=np.array([body.ravel(),carrier.ravel()]),time=1.),fork_id=0)
    params=OffroadParams(mode='hardware',heading_reference='start',heading_actuation='shells',
        heading_gain=.2,heading_damping=.06,heading_torque_limit_Nm=.15,heading_roll_budget_Nm=.05)
    ctrl=NS(plant=plant,params=params,carrier_id=1,heading_reference=0.,_yaw_error=.1,
            _held=dict(loads=loads or dict(shell_A=0.,shell_B=0.,belt_A=10.,belt_B=0.)))
    def velocity(model,data,kind,body_id,out,local):out[:]=np.r_[omega,np.zeros(3)]
    return ctrl,velocity


class ConstantReference:
    def __init__(self,target=.2,rate=.03):self.target,self.rate=target,rate
    def sample(self,t):
        return dict(raw_heading_rad=self.target,raw_heading_rate_rad_s=self.rate,raw_heading_acceleration_rad_s2=0.,
            command_heading_rad=self.target,command_heading_rate_rad_s=self.rate,command_heading_acceleration_rad_s2=0.,
            rate_limited=False,acceleration_limited=False)


class TangentHelperContracts(unittest.TestCase):
    def test_euler_rate_matches_independent_centered_rotation_derivative(self):
        for angles,omega in [((.3,.4,-.7),(.2,-.4,.3)),((-.5,.2,1.3),(-.3,.15,-.2))]:
            R=Rxyz(*angles);w=np.array(omega);h=1e-6
            cross=np.array([[0.,-w[2],w[1]],[w[2],0.,-w[0]],[-w[1],w[0],0.]])
            rate,valid,_=euler_heading_rate(R,w)
            # Rodrigues exponentials rotate the frame without integrating dynamics.
            norm=np.linalg.norm(w)
            def shifted(sign):
                angle=sign*h*norm
                exponential=np.eye(3)+np.sin(angle)/norm*cross+(1-np.cos(angle))/norm**2*(cross@cross)
                result=exponential@R
                return np.arctan2(result[1,0],result[0,0])
            finite=wrap(shifted(1)-shifted(-1))/(2*h)
            self.assertTrue(valid);self.assertAlmostEqual(rate,finite,places=8)

    def test_heading_rate_equals_world_z_only_for_the_matching_geometry(self):
        rate,valid,_=euler_heading_rate(np.eye(3),[.2,-.3,.4])
        self.assertTrue(valid);self.assertEqual(rate,.4)
        tilted,valid,_=euler_heading_rate(Rxyz(.3,.5,.2),[.2,-.3,.4])
        self.assertTrue(valid);self.assertGreater(abs(tilted-.4),.01)

    def test_singular_projection_disables_only_new_helper(self):
        ctrl,velocity=fixture(rotation=Rxyz(0.,np.pi/2,0.))
        with patch('mujoco.mj_objectVelocity',velocity):command,error,row=helper_solution(ctrl,.1,0.,.03,True)
        self.assertFalse(row['rate_valid']);self.assertFalse(row['eligible'])
        np.testing.assert_array_equal(command,np.zeros(5));self.assertTrue(np.isnan(row['euler_heading_rate_rad_s']))

    def test_desired_rate_changes_the_damping_error_not_gains_or_bounds(self):
        ctrl,velocity=fixture()
        with patch('mujoco.mj_objectVelocity',velocity):
            _,_,zero=helper_solution(ctrl,.1,0.,0.,True)
            command,_,tracking=helper_solution(ctrl,.1,0.,.03,True)
        self.assertAlmostEqual(tracking['wanted_world_yaw_torque_Nm']-zero['wanted_world_yaw_torque_Nm'],.06*.03)
        self.assertEqual(tracking['effective_motor_limit_Nm'],zero['effective_motor_limit_Nm'])
        self.assertLessEqual(abs(command[1:3]).sum(),.15)

    def test_constant_target_zero_rate_matches_original_helper_on_level_pure_yaw(self):
        ctrl,velocity=fixture(omega=(0.,0.,.05))
        with patch('mujoco.mj_objectVelocity',velocity):
            expected,error=OffroadController._heading_torque(ctrl,.1)
            command,actual_error,_=helper_solution(ctrl,.1,0.,0.,True)
        np.testing.assert_array_equal(command,expected);self.assertEqual(actual_error,error)

    def test_original_load_lever_and_reaction_budget_conditions_retained(self):
        for vertical,loads in [(1.,dict(shell_A=5.,shell_B=5.,belt_A=0.,belt_B=0.)),(.05,None)]:
            ctrl,velocity=fixture(vertical=vertical,loads=loads)
            with patch('mujoco.mj_objectVelocity',velocity):command,_,row=helper_solution(ctrl,.1,0.,.03,True)
            self.assertFalse(row['eligible']);np.testing.assert_array_equal(command,np.zeros(5))
        ctrl,velocity=fixture(vertical=.5);ctrl._yaw_error=.5
        with patch('mujoco.mj_objectVelocity',velocity):command,_,row=helper_solution(ctrl,.5,0.,-.5,True)
        self.assertTrue(row['eligible'])
        self.assertLessEqual(abs(command[1:3]).sum()*row['horizontal_lever'],.05+1e-15)

    def test_adapter_preserves_native_target_filter_and_latch(self):
        calls=[]
        ctrl=NS(heading_reference=.1,plant=NS(data=NS(time=0.)),params=NS(heading_error_limit_rad=.5),
            _update_heading_target=lambda normal,yaw:calls.append((normal,yaw)),_heading_torque=lambda yaw:(np.zeros(5),0.))
        original_update=ctrl._update_heading_target;original_helper=ctrl._heading_torque
        adapter=TangentHeadingAdapter(ctrl,'T',ConstantReference(target=1.2,rate=.03))
        with adapter.command_context():
            self.assertEqual(ctrl._update_heading_target([0.,0.,1.],0.),-.5)
        self.assertEqual(len(calls),1);self.assertEqual(ctrl.heading_reference,.1)
        self.assertIs(ctrl._update_heading_target,original_update);self.assertIs(ctrl._heading_torque,original_helper)

    def test_N_context_is_unpatched_and_nested_or_nonfinite_stream_rejected(self):
        ctrl=NS(heading_reference=.1,plant=NS(data=NS(time=0.)),
            _update_heading_target=lambda n,y:None,_heading_torque=lambda y:(np.zeros(5),0.))
        original=ctrl._heading_torque;adapter=TangentHeadingAdapter(ctrl,'N',ConstantReference())
        with adapter.command_context():
            self.assertIs(ctrl._heading_torque,original)
            with self.assertRaises(RuntimeError):
                with adapter.command_context():pass
        invalid=TangentHeadingAdapter(ctrl,'T',ConstantReference(target=np.nan))
        with self.assertRaises(ArithmeticError):
            with invalid.command_context():pass


if __name__=='__main__':unittest.main()
