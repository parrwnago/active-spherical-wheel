import unittest
import numpy as np
from heading_reference import PlannedTangentReference


class HeadingReferenceContracts(unittest.TestCase):
    def setUp(self):self.ref=PlannedTangentReference(.35,.5)

    def test_reference_faces_geometric_path_tangent_while_moving(self):
        for time in (3.5,8.,15.,19.,26.,34.5):
            velocity=self.ref.path(time)[1]
            expected=np.arctan2(velocity[1],velocity[0])
            actual=self.ref.sample(time)['raw_heading_rad']
            self.assertLess(abs(np.arctan2(np.sin(actual-expected),np.cos(actual-expected))),1e-12)

    def test_known_endpoint_direction_has_exact_zero_rates_and_no_atan2_zero(self):
        expected=self.ref.path.heading_deg
        for time in (-1.,0.,3.,35.,40.,45.,100.):
            row=self.ref.sample(time)
            self.assertAlmostEqual(np.rad2deg(row['raw_heading_rad']),expected,places=12)
            self.assertEqual(row['raw_heading_rate_rad_s'],0.)
            self.assertEqual(row['raw_heading_acceleration_rad_s2'],0.)

    def test_analytic_rate_and_acceleration_match_independent_finite_differences(self):
        step=1e-4
        for time in (4.,8.,13.,19.,25.,31.,34.):
            before,center,after=(self.ref.sample(t) for t in (time-step,time,time+step))
            rate=(after['raw_heading_rad']-before['raw_heading_rad'])/(2*step)
            accel=(after['raw_heading_rate_rad_s']-before['raw_heading_rate_rad_s'])/(2*step)
            self.assertAlmostEqual(center['raw_heading_rate_rad_s'],rate,places=8)
            self.assertAlmostEqual(center['raw_heading_acceleration_rad_s2'],accel,places=8)

    def test_raw_and_command_fields_are_exact_and_clock_order_independent(self):
        for time in (35.,3.,20.,0.,45.,10.,20.):
            row=self.ref.sample(time)
            for suffix in ('heading_rad','heading_rate_rad_s','heading_acceleration_rad_s2'):
                self.assertEqual(row['raw_'+suffix],row['command_'+suffix])
            self.assertFalse(row['rate_limited']);self.assertFalse(row['acceleration_limited'])
            self.assertEqual(row,self.ref.sample(time))

    def test_limits_have_analytic_certificate_and_inadequate_or_nonfinite_caps_fail(self):
        self.assertLess(self.ref.certified_max_rate_rad_s,.35)
        self.assertLess(self.ref.certified_max_acceleration_rad_s2,.5)
        for rate,acceleration in ((0.,.5),(.35,0.),(.1,.5),(.35,.01),(np.nan,.5),(.35,np.inf)):
            with self.assertRaises(ValueError):PlannedTangentReference(rate,acceleration)
        with self.assertRaises(ValueError):self.ref.sample(np.nan)


if __name__=='__main__':unittest.main()
