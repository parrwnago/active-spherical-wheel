"""Synthetic capture adapter/association contracts, no native captures read."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from analyse_capture import analyse, match_contacts


def fixture():
    a = np.zeros((3, 4)); a[:, :3] = [[1, 1.5, 0], [1.5, 4, 0], [0, 0, 2]]
    solver = dict(kind=0, id=1, solver='Dantzig', control_time_s=1., substep=1,
        n=3, stride=4, nub=0, early=True, success=True, before_A=a,
        before_b=np.array([1., 4., 0.]), before_lo=np.array([0., -.7, -.7]),
        before_hi=np.array([np.inf, .7, .7]), before_findex=np.array([-1, 0, 0]),
        after_x=np.array([0., .7, 0.]))
    contact = dict(kind=2, id=2, parent_solver_id=1, control_time_s=1., substep=1,
        offset=0, dim=3, buffer=0, dt_s=.001, mu1=.7, mu2=.7,
        **{'lambda': np.array([0., .7, 0.])}, point=np.array([1., 2., 3.]),
        normal=np.array([0., 0., 1.]), force=np.array([700., 0., 0.]), depth=np.array([.001]),
        basis=np.array([1., 0., 0., 0., 1., 0.]), motion=np.zeros(3))
    row = np.r_[1., contact['point'], -contact['normal'], -contact['force'], .001, .7, 1., 0., 0., 0.]
    trace = dict(control_start_time_s=np.array([1.]), control_end_time_s=np.array([1.005]),
                 substep_index=np.array([1]), contact_rows=row[None], contact_offsets=np.array([0, 1]))
    return [solver, contact], trace


class AdapterContracts(unittest.TestCase):
    def test_applied_lambda_and_force_reconstruction(self):
        p, _ = fixture(); report, _ = analyse(p)
        self.assertEqual(report['checks']['applied_contact_with_final_bound_violation_count'], 1)
        self.assertEqual(report['checks']['maximum_impulse_to_force_difference_N'], 0)
        p[1]['lambda'][1] = .2
        with self.assertRaisesRegex(ValueError, 'differs from parent'):
            analyse(p)

    def test_body_order_sign_and_ambiguous_nonzero_family(self):
        p, trace = fixture(); report, _ = analyse(p)
        m = match_contacts(report, trace)
        self.assertEqual(m['nonzero_association_failure_count'], 0)
        self.assertEqual(m['matches'][0]['candidate_force_sign'], [-1.])
        duplicate = trace['contact_rows'].copy(); duplicate[:, 0] = 3
        trace['contact_rows'] = np.r_[trace['contact_rows'], duplicate]
        trace['contact_offsets'][1] = 2
        self.assertEqual(match_contacts(report, trace)['nonzero_association_failure_count'], 1)

    def test_missing_force_match_is_reported_not_selected_away(self):
        p, trace = fixture(); report, _ = analyse(p)
        trace['contact_rows'][:, 7] += 1
        m = match_contacts(report, trace)
        self.assertEqual(m['nonzero_contact_count'], 1)
        self.assertEqual(m['nonzero_association_failure_count'], 1)


if __name__ == '__main__': unittest.main()
