"""No-physics contracts against the real existing native heading helper."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest.mock import patch
import unittest
import numpy as np

from heading_telemetry import heading_diagnostics
from spherical_wheel.offroad_control import OffroadController,OffroadParams


def fixture(vertical=.8,loads=(0.,10.,0.,0.),gain=.2,reference='start',yaw=.7,target=.2,rate=.4):
    horizontal=np.sqrt(1.-vertical**2)
    rotation=np.array([[horizontal,0.,-vertical],[0.,1.,0.],[vertical,0.,horizontal]])
    params=OffroadParams(mode='hardware',heading_reference=reference,heading_gain=gain,
        heading_damping=.06,heading_torque_limit_Nm=.15,heading_roll_budget_Nm=.05)
    c=NS(params=params,heading_reference=target,
        _yaw_error=None if reference=='none' else np.clip((yaw-target+np.pi)%(2*np.pi)-np.pi,-.5,.5),
        _held=dict(loads=dict(zip(('shell_A','shell_B','belt_A','belt_B'),loads))),
        plant=NS(data=NS(xmat=np.stack([np.eye(3).ravel(),rotation.ravel()])),
                 model=NS(),fork_id=0),carrier_id=1)
    def velocity(model,data,obj,body,out,local):
        if local!=0:raise AssertionError('Heading helper must request world-frame angular rate')
        out[:]=[0.,0.,rate,0.,0.,0.]
    return c,yaw,velocity


def native_info(controller,yaw):
    command,error=OffroadController._heading_torque(controller,yaw)
    return dict(yaw_rad=yaw,heading_command=command,heading_error_rad=error)


class HeadingTelemetryContracts(unittest.TestCase):
    def test_matches_real_helper_and_combined_roll_torque_bound(self):
        c,yaw,velocity=fixture()
        with patch('mujoco.mj_objectVelocity',velocity):
            info=native_info(c,yaw);r=heading_diagnostics(c,info)
        self.assertTrue(r['eligible']);self.assertTrue(r['command_matches_native'])
        self.assertAlmostEqual(sum(info['heading_command'][1:3])*r['horizontal_lever'],.05)
        self.assertLessEqual(abs(info['heading_command']).max(),.15)

    def test_native_disabled_heading_helper_remains_zero(self):
        c,yaw,velocity=fixture(gain=0.,reference='none',rate=3.)
        with patch('mujoco.mj_objectVelocity',velocity):
            r=heading_diagnostics(c,native_info(c,yaw))
        self.assertFalse(r['eligible']);np.testing.assert_array_equal(r['expected_command'],np.zeros(5))
        self.assertEqual(r['world_z_rate_rad_s'],3.)

    def test_insufficient_vertical_axis_skips_heading_correction(self):
        c,yaw,velocity=fixture(vertical=.05)
        with patch('mujoco.mj_objectVelocity',velocity):r=heading_diagnostics(c,native_info(c,yaw))
        self.assertFalse(r['eligible']);np.testing.assert_array_equal(r['expected_command'],np.zeros(5))

    def test_no_unloaded_shell_skips_heading_correction(self):
        c,yaw,velocity=fixture(loads=(5.,5.,0.,0.))
        with patch('mujoco.mj_objectVelocity',velocity):r=heading_diagnostics(c,native_info(c,yaw))
        self.assertFalse(r['eligible']);self.assertFalse(r['free_shell_mask'].any())

    def test_both_unloaded_shells_share_one_bounded_reaction(self):
        c,yaw,velocity=fixture(loads=(0.,0.,10.,0.))
        with patch('mujoco.mj_objectVelocity',velocity):r=heading_diagnostics(c,native_info(c,yaw))
        self.assertTrue(r['free_shell_mask'].all())
        self.assertAlmostEqual(r['expected_command'][1],r['expected_command'][2])
        self.assertLessEqual(abs(r['expected_command'][1:3].sum())*r['horizontal_lever'],.05+1e-15)

    def test_target_error_wraps_and_native_used_error_is_clipped(self):
        for yaw,target in ((-3.1,3.1),(2.,.2)):
            with self.subTest(yaw=yaw,target=target):
                c,yaw,velocity=fixture(yaw=yaw,target=target)
                with patch('mujoco.mj_objectVelocity',velocity):r=heading_diagnostics(c,native_info(c,yaw))
                self.assertLessEqual(abs(r['raw_error_rad']),np.pi)
                self.assertLessEqual(abs(r['error_used_rad']),.5)
                self.assertEqual(r['target_rad'],target)

    def test_rejects_changed_logged_helper_command(self):
        c,yaw,velocity=fixture()
        with patch('mujoco.mj_objectVelocity',velocity):
            info=native_info(c,yaw);info['heading_command'][1]+=.001
            with self.assertRaises(AssertionError):heading_diagnostics(c,info)

    def test_diagnostics_do_not_change_controller_state_or_logged_command(self):
        c,yaw,velocity=fixture()
        with patch('mujoco.mj_objectVelocity',velocity):
            info=native_info(c,yaw);held=deepcopy(c._held);matrix=c.plant.data.xmat.copy();command=info['heading_command'].copy()
            heading_diagnostics(c,info)
        self.assertEqual(c._held,held);np.testing.assert_array_equal(c.plant.data.xmat,matrix)
        np.testing.assert_array_equal(info['heading_command'],command)


if __name__=='__main__':unittest.main()
