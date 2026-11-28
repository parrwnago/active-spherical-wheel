"""I76 route pairing, initialization, clocks and unchanged controller boundary."""
import copy
import unittest
import run_case


def protocol():
    return dict(iteration='I-76',parameters=dict(run_case.BASELINE_SETTINGS),
        heading_parameters=dict(run_case.HEADING_SETTINGS),heading_reference_parameters=dict(run_case.REFERENCE_SETTINGS),
        cases=run_case.registered_cases())


class RunnerContracts(unittest.TestCase):
    def test_exact_route_phase_pairs_with_route_aligned_yaw_and_clocks(self):
        p=protocol();self.assertEqual(len(p['cases']),18)
        for job in p['cases']:self.assertEqual(run_case.validate_protocol(p,job['case_id']),job)
        for mutate in ('yaw_deg','duration_s','route_id','policy'):
            q=copy.deepcopy(p)
            q['cases'][0][mutate]={'yaw_deg':45.,'duration_s':133.,'route_id':'eight','policy':'T'}[mutate]
            with self.assertRaises(ValueError):run_case.validate_protocol(q,p['cases'][0]['case_id'])
        q=copy.deepcopy(p);q['cases'].pop()
        with self.assertRaises(ValueError):run_case.validate_protocol(q,p['cases'][0]['case_id'])

    def test_only_T_enables_heading_and_reference_guards_are_fixed(self):
        p=protocol()
        self.assertEqual(run_case.parameters_for(p,p['cases'][0]),run_case.BASELINE_SETTINGS)
        self.assertEqual(run_case.parameters_for(p,p['cases'][1]),{**run_case.BASELINE_SETTINGS,**run_case.HEADING_SETTINGS})
        p['heading_reference_parameters']['max_rate_rad_s']=.4
        with self.assertRaises(ValueError):run_case.validate_protocol(p,'straight_m100_T')

    def test_rejects_new_gain_or_allocation_or_unregistered_heading_cost(self):
        for key,value in [('lqr_weights','yaw_aware'),('rolling_allocation','damped'),('track_contact_point',True)]:
            p=protocol();p['parameters'][key]=value
            with self.assertRaises(ValueError):run_case.validate_protocol(p,'straight_m100_T')
        p=protocol();p['heading_parameters']['heading_gain']=.3
        with self.assertRaises(ValueError):run_case.validate_protocol(p,'straight_m100_T')


if __name__=='__main__':unittest.main()
