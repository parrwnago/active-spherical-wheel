import unittest
import numpy as np

from route import ROUTE_IDS, make_route, samples, world_bounds
from heading_reference import PlannedTangentReference


class AnalyticReferenceContracts(unittest.TestCase):
    def test_frozen_suite_specs_initialization_and_clock(self):
        self.assertEqual(ROUTE_IDS, ('straight', 'arc', 'eight'))
        for route_id, duration, heading in (('straight', 32., 113.), ('arc', 32., 113.), ('eight', 120., 158.)):
            path = make_route(route_id)
            self.assertEqual(path.route_id, route_id)
            np.testing.assert_array_equal(path.anchor_origin, [-2.4, 10.])
            expected_offset = [.5, 0.] if route_id == 'eight' else [0., 0.]
            np.testing.assert_array_equal(path.planned_local_offset, expected_offset)
            angle = np.deg2rad(113.)
            expected_origin = np.array([-2.4, 10.])+np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])@expected_offset
            np.testing.assert_array_equal(path.origin, expected_origin)
            self.assertEqual(path.spec()['anchor_origin'], [-2.4, 10.])
            self.assertEqual(path.spec()['planned_local_offset'], expected_offset)
            self.assertEqual((path.start_s, path.duration_s, path.goal_hold_s), (3., duration, 10.))
            self.assertEqual(path.total_duration_s, duration+13.)
            self.assertAlmostEqual(path.heading_deg, heading, places=12)
            self.assertEqual(path.spec()['initial_heading_deg'], path.heading_deg)
            np.testing.assert_allclose(path.direction, [np.cos(np.deg2rad(heading)), np.sin(np.deg2rad(heading))], atol=1e-15)
        with self.assertRaises(ValueError):
            make_route('unknown')

    def test_geometric_derivatives_match_independent_centered_differences(self):
        step = 1e-6
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            for u in (.1, .25, .375, .5, .625, .9):
                before, center, after = (path.shape_derivatives(v) for v in (u-step, u, u+step))
                for order in range(3):
                    numerical = (after[order]-before[order])/(2*step)
                    np.testing.assert_allclose(center[order+1], numerical, rtol=2e-8, atol=3e-8)

    def test_world_motion_derivatives_rotation_and_fixed_goal(self):
        step = 1e-4
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            for fraction in (.15, .4, .5, .65, .85):
                t = path.start_s + fraction*path.duration_s
                before, center, after = (path(v) for v in (t-step, t, t+step))
                np.testing.assert_allclose(center[1], (after[0]-before[0])/(2*step), rtol=2e-7, atol=3e-10)
                np.testing.assert_allclose(center[2], (after[1]-before[1])/(2*step), rtol=2e-7, atol=3e-10)
                u = path.progress(t)[0]
                np.testing.assert_allclose(center[0], path.origin+path.rotation@path.shape_derivatives(u)[0], atol=1e-15)
            goal = path(path.total_duration_s)[0]
            np.testing.assert_array_equal(goal, path(path.total_duration_s+20)[0])
            if route_id == 'eight':
                np.testing.assert_array_equal(goal, path.origin)

    def test_geometric_tangent_remains_regular_and_analytic_branch_continuous(self):
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            angles = []
            for u in np.linspace(0., 1., 2001):
                first = path.shape_derivatives(u)[1]
                self.assertGreater(np.linalg.norm(first), 0.)
                world = path.rotation@first
                angle = path.tangent_heading(u)
                wrapped = np.arctan2(np.sin(angle-np.arctan2(world[1], world[0])), np.cos(angle-np.arctan2(world[1], world[0])))
                self.assertLess(abs(wrapped), 2e-15)
                angles.append(angle)
            self.assertLess(float(abs(np.diff(angles)).max()), .02)
        path = make_route('eight')
        for branch_u in (.375, .625):
            values = [path.tangent_heading(branch_u+offset) for offset in (-1e-8, 0., 1e-8)]
            self.assertLess(max(values)-min(values), 1e-6)
        self.assertAlmostEqual(path.tangent_heading(.5), np.deg2rad(-112.), places=12)

    def test_endpoint_holds_use_geometric_angles_and_exact_zero_temporal_derivatives(self):
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            ref = PlannedTangentReference(.35, .5, path)
            for t, u in ((-20., 0.), (0., 0.), (3., 0.), (3.+path.duration_s, 1.), (path.total_duration_s, 1.), (path.total_duration_s+20, 1.)):
                row = ref.sample(t)
                self.assertEqual(row['raw_heading_rad'], path.tangent_heading(u))
                self.assertEqual(row['raw_heading_rate_rad_s'], 0.)
                self.assertEqual(row['raw_heading_acceleration_rad_s2'], 0.)
                np.testing.assert_array_equal(path(t)[1], np.zeros(2))
                np.testing.assert_array_equal(path(t)[2], np.zeros(2))

    def test_heading_chain_rule_matches_independent_time_finite_differences(self):
        step = 1e-4
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            ref = PlannedTangentReference(.35, .5, path)
            for fraction in (.1, .3, .4, .5, .6, .7, .9):
                t = path.start_s+fraction*path.duration_s
                before, center, after = (ref.sample(v) for v in (t-step, t, t+step))
                numerical_rate = (after['raw_heading_rad']-before['raw_heading_rad'])/(2*step)
                numerical_accel = (after['raw_heading_rate_rad_s']-before['raw_heading_rate_rad_s'])/(2*step)
                self.assertAlmostEqual(center['raw_heading_rate_rad_s'], numerical_rate, places=8)
                self.assertAlmostEqual(center['raw_heading_acceleration_rad_s2'], numerical_accel, places=8)
                velocity = path(t)[1]
                self.assertLess(abs(np.arctan2(np.sin(center['raw_heading_rad']-np.arctan2(velocity[1], velocity[0])), np.cos(center['raw_heading_rad']-np.arctan2(velocity[1], velocity[0])))), 2e-15)

    def test_raw_command_identity_stateless_sampling_and_finite_input_guards(self):
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            ref = PlannedTangentReference(.35, .5, path)
            for t in (path.total_duration_s, 5., 0., 20., 3., 20.):
                row = ref.sample(t)
                self.assertEqual(row, ref.sample(t))
                for suffix in ('heading_rad', 'heading_rate_rad_s', 'heading_acceleration_rad_s2'):
                    self.assertEqual(row['raw_'+suffix], row['command_'+suffix])
                self.assertFalse(row['rate_limited'])
                self.assertFalse(row['acceleration_limited'])
            with self.assertRaises(ValueError):
                ref.sample(np.nan)
            with self.assertRaises(ValueError):
                path.shape_derivatives(1.1)
        for caps in ((0., .5), (.35, 0.), (.01, .5), (.35, .0001), (np.nan, .5), (.35, np.inf)):
            with self.assertRaises(ValueError):
                PlannedTangentReference(*caps, path=make_route('arc'))
        with self.assertRaises(ValueError):
            PlannedTangentReference(.35, .5)

    def test_global_certificates_cover_geometry_and_registered_temporal_bounds(self):
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            certificate = path.derivative_certificate()
            ref = PlannedTangentReference(.35, .5, path)
            self.assertLessEqual(ref.certified_max_rate_rad_s, .35)
            self.assertLessEqual(ref.certified_max_acceleration_rad_s2, .5)
            for u in np.linspace(0., 1., 1001):
                _, d1, d2, d3 = path.shape_derivatives(u)
                self.assertGreaterEqual(np.linalg.norm(d1)+1e-12, certificate['minimum_tangent_norm'])
                self.assertLessEqual(np.linalg.norm(d2), certificate['maximum_second_norm']+1e-12)
                self.assertLessEqual(np.linalg.norm(d3), certificate['maximum_third_norm']+1e-12)

    def test_map_fixture_sampling_schema_and_union_world_bounds(self):
        union = world_bounds()
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            values = samples(route_id, count=21, corridor_radius_m=.15, corridor_count=25)
            self.assertEqual(values['xy_m'].shape, (21, 2))
            self.assertEqual(values['corridor_xy_m'].shape, (21*26, 2))
            np.testing.assert_array_equal(values['time_s'][[0, -1]], [path.start_s, path.start_s+path.duration_s])
            bounds = world_bounds(route_id)
            self.assertLessEqual(union[0], bounds[0])
            self.assertGreaterEqual(union[1], bounds[1])
            self.assertLessEqual(union[2], bounds[2])
            self.assertGreaterEqual(union[3], bounds[3])

    def test_ulp_and_accumulated_endpoint_clocks_keep_geometry_domain_valid(self):
        # The failed attempt's last saved TIME ONLY was122.99499999996063.
        # Its next nominal5ms clock is below123 but the expanded quintic
        # rounds four ULP above1, which formerly rejected the planned route.
        accumulated_endpoint = 122.99499999996063+.005
        r = (accumulated_endpoint-3.)/120.
        self.assertLess(r, 1.)
        self.assertGreater(10*r**3-15*r**4+6*r**5, 1.)
        for route_id in ROUTE_IDS:
            path = make_route(route_id)
            ref = PlannedTangentReference(.35, .5, path)
            end = path.start_s+path.duration_s
            clocks = [np.nextafter(path.start_s, -np.inf), path.start_s,
                      np.nextafter(path.start_s, np.inf),
                      np.nextafter(end, -np.inf), end, np.nextafter(end, np.inf)]
            # Carry the observed fractional progress to each route clock;
            # the endpoint regression must not depend on a neat .005 grid.
            clocks += [path.start_s+path.duration_s*r]
            if route_id == 'eight':
                clocks.append(accumulated_endpoint)
            for clock in clocks:
                u, du, ddu, moving = path.progress(clock)
                self.assertTrue(0. <= u <= 1.)
                self.assertTrue(np.isfinite([u, du, ddu]).all())
                position, velocity, acceleration = path(clock)
                self.assertTrue(np.isfinite(np.r_[position, velocity, acceleration]).all())
                row = ref.sample(clock)
                self.assertTrue(np.isfinite([row['raw_heading_rad'], row['raw_heading_rate_rad_s'],
                                             row['raw_heading_acceleration_rad_s2']]).all())
                if moving:
                    q = (float(clock)-path.start_s)/path.duration_s
                    self.assertEqual(du, (30*q*q-60*q**3+30*q**4)/path.duration_s)
                    self.assertEqual(ddu, (60*q-180*q*q+120*q**3)/path.duration_s**2)


if __name__ == '__main__':
    unittest.main()
