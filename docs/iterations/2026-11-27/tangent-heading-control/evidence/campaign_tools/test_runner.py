"""I75 protocol boundaries: three phases, same aligned yaw and fixed options."""
import unittest
import run_case


def protocol():
    return dict(iteration='I-75',parameters=dict(run_case.BASELINE_SETTINGS),
        heading_parameters=dict(run_case.HEADING_SETTINGS),heading_reference_parameters=dict(run_case.REFERENCE_SETTINGS),
        initial_yaw_deg=run_case.INITIAL_YAW_DEG,
        cases=[dict(case_id=f'{name}_{policy}',policy=policy,phase_rad=phase,yaw_deg=run_case.INITIAL_YAW_DEG,duration_s=45.)
               for name,phase in [('m100',-1.),('p030',.3),('p090',.9)] for policy in ('N','T')])


class RunnerContracts(unittest.TestCase):
    def test_exact_paired_phases_and_aligned_initial_yaw(self):
        p=protocol();self.assertEqual(run_case.validate_protocol(p,'p030_T')['policy'],'T')
        p['cases'][0]['yaw_deg']=45.
        with self.assertRaises(ValueError):run_case.validate_protocol(p,'p030_T')
        p=protocol();p['cases'][0]['policy']='T'
        with self.assertRaises(ValueError):run_case.validate_protocol(p,'p030_T')

    def test_only_T_enables_heading_and_reference_guard_settings_are_fixed(self):
        p=protocol()
        self.assertEqual(run_case.parameters_for(p,p['cases'][0]),run_case.BASELINE_SETTINGS)
        self.assertEqual(run_case.parameters_for(p,p['cases'][1]),{**run_case.BASELINE_SETTINGS,**run_case.HEADING_SETTINGS})
        p['heading_reference_parameters']['max_rate_rad_s']=.4
        with self.assertRaises(ValueError):run_case.validate_protocol(p,'m100_T')

    def test_rejects_new_gain_or_allocation_or_unregistered_heading_cost(self):
        for key,value in [('lqr_weights','yaw_aware'),('rolling_allocation','damped'),('track_contact_point',True)]:
            p=protocol();p['parameters'][key]=value
            with self.assertRaises(ValueError):run_case.validate_protocol(p,'m100_T')
        p=protocol();p['heading_parameters']['heading_gain']=.3
        with self.assertRaises(ValueError):run_case.validate_protocol(p,'m100_T')


if __name__=='__main__':unittest.main()
