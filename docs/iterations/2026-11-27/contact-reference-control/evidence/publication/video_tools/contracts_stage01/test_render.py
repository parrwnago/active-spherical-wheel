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


if __name__=='__main__':unittest.main()
