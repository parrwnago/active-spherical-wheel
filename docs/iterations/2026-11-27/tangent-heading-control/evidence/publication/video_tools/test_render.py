import unittest
import numpy as np
from render import orientation_signals,nearest


class OrientationContracts(unittest.TestCase):
    def test_yaw_is_distinct_from_tilt(self):
        q=np.zeros((1,10));q[0,3:7]=[np.cos(np.pi/8),0,0,np.sin(np.pi/8)]
        value=orientation_signals(q,np.zeros((1,9)),np.array([[1.,0,0,0]]))
        self.assertAlmostEqual(value['yaw_deg'][0],45.)
        self.assertAlmostEqual(value['tilt_deg'][0],0.)
        self.assertAlmostEqual(value['reference_lean_deg'][0],0.)

    def test_world_z_rate_rotates_body_angular_velocity(self):
        q=np.zeros((1,10));q[0,3:7]=[np.sqrt(.5),np.sqrt(.5),0,0]
        v=np.zeros((1,9));v[0,4]=2.
        value=orientation_signals(q,v,q[:,3:7])
        self.assertAlmostEqual(value['tilt_deg'][0],90.)
        self.assertAlmostEqual(value['reference_lean_deg'][0],90.)
        self.assertAlmostEqual(value['omega_world_z_deg_s'][0],np.rad2deg(2.))

    def test_nearest_pose_never_invents_a_future_end_state(self):
        times=np.array([0.,.005,.01])
        self.assertEqual(nearest(times,45.),2)
        self.assertEqual(nearest(times,.005),1)
        self.assertEqual(nearest(times,-1.),0)

    def test_reference_up_error_is_not_difference_of_tilt_magnitudes(self):
        q=np.zeros((1,10));q[0,3:7]=[np.cos(np.pi/12),np.sin(np.pi/12),0,0]
        ref=np.array([[np.cos(np.pi/12),-np.sin(np.pi/12),0,0]])
        value=orientation_signals(q,np.zeros((1,9)),ref)
        self.assertAlmostEqual(value['tilt_deg'][0],30.)
        self.assertAlmostEqual(value['reference_lean_deg'][0],30.)
        self.assertAlmostEqual(value['reference_up_error_deg'][0],60.)

    def test_heading_target_wraps_across_pi_while_display_yaw_unwraps(self):
        yaw=np.deg2rad(np.array([179.,-179.,-175.]))
        q=np.zeros((3,10));q[:,3]=np.cos(yaw/2);q[:,6]=np.sin(yaw/2)
        target=np.full(3,np.deg2rad(179.))
        value=orientation_signals(q,np.zeros((3,9)),q[:,3:7],target)
        np.testing.assert_allclose(value['yaw_deg'],[179.,181.,185.],atol=1e-12)
        np.testing.assert_allclose(value['heading_error_deg'],[0.,2.,6.],atol=1e-12)
        np.testing.assert_allclose(value['heading_target_deg'],179.,atol=1e-12)

    def test_moving_raw_target_is_distinct_from_actual_heading_and_initial_latch(self):
        yaw=np.deg2rad(np.array([30.,40.,50.]))
        q=np.zeros((3,10));q[:,3]=np.cos(yaw/2);q[:,6]=np.sin(yaw/2)
        target=np.deg2rad(np.array([30.,35.,45.]))
        value=orientation_signals(q,np.zeros((3,9)),q[:,3:7],target)
        np.testing.assert_allclose(value['heading_error_deg'],[0.,5.,5.],atol=1e-12)
        np.testing.assert_allclose(value['heading_target_deg'],[30.,35.,45.],atol=1e-12)

    def test_euler_heading_rate_is_not_world_z_rate_on_a_tilted_body(self):
        angle=np.deg2rad(30.)
        q=np.zeros((1,10));q[0,3:7]=[np.cos(angle/2),0,np.sin(angle/2),0]
        v=np.zeros((1,9));v[0,3]=1.
        value=orientation_signals(q,v,q[:,3:7])
        self.assertTrue(value['heading_rate_valid'][0])
        self.assertAlmostEqual(value['euler_heading_rate_deg_s'][0],0.,places=12)
        self.assertAlmostEqual(value['omega_world_z_deg_s'][0],np.rad2deg(-.5),places=12)


if __name__=='__main__':unittest.main()
