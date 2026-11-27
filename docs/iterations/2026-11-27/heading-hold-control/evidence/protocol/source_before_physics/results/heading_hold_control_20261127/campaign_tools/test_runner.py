"""I74 registration contracts; no plant construction or physical steps."""
from copy import deepcopy
import unittest
import run_case
from spherical_wheel.offroad_control import OffroadParams


def protocol():
    return dict(iteration='I-74',parameters=dict(run_case.BASELINE_SETTINGS),
        heading_parameters=dict(run_case.HEADING_SETTINGS),
        cases=[dict(case_id=f'{name}_{policy}',policy=policy,phase_rad=phase,yaw_deg=45.,duration_s=45.)
        for name,phase in [('m100',-1.),('p030',.3),('p090',.9)] for policy in ('N','H')])


class RunnerContracts(unittest.TestCase):
    def test_exact_six_registered_cases_and_fixed_initial_conditions(self):
        p=protocol();self.assertEqual(run_case.validate_protocol(p,'p030_H')['policy'],'H')
        for mutate in (lambda p:p['cases'].append(deepcopy(p['cases'][0])),
                       lambda p:p['cases'][0].update(policy='A'),
                       lambda p:p['cases'][0].update(yaw_deg=0.),
                       lambda p:p['cases'][0].update(duration_s=44.)):
            p=protocol();mutate(p)
            with self.assertRaises(ValueError):run_case.validate_protocol(p,'p030_H')

    def test_resolved_parameters_change_only_fixed_heading_fields(self):
        p=protocol();n=run_case.parameters_for(p,p['cases'][0]);h=run_case.parameters_for(p,p['cases'][1])
        before=OffroadParams(**n).as_dict();after=OffroadParams(**h).as_dict()
        changed={key for key in before if before[key]!=after[key]}
        self.assertEqual(changed,{'heading_reference','heading_gain','heading_roll_budget_Nm'})
        self.assertEqual(before['heading_gain'],0.);self.assertEqual(after['heading_reference'],'start')
        self.assertEqual(after['heading_actuation'],'shells');self.assertEqual(after['lqr_weights'],'flat')
        self.assertEqual(n,run_case.BASELINE_SETTINGS)

    def test_rejects_unregistered_tuning_and_other_controller_changes(self):
        for field,value in [('heading_gain',.3),('heading_damping',.1),('heading_torque_limit_Nm',.2),('heading_roll_budget_Nm',.1)]:
            p=protocol();p['heading_parameters'][field]=value
            with self.subTest(field=field):
                with self.assertRaises(ValueError):run_case.validate_protocol(p,'p030_H')
        p=protocol();p['parameters']['lqr_weights']='yaw_aware'
        with self.assertRaises(ValueError):run_case.validate_protocol(p,'p030_H')


if __name__=='__main__':unittest.main()
