"""Focused runner boundary contracts with synthetic states only."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_case


def protocol():
    return dict(iteration='I-73',cases=[dict(case_id=f'{prefix}_{policy}',policy=policy,
        phase_rad=phase,yaw_deg=45.,duration_s=45.) for prefix,phase in
        [('m100',-1.),('p030',.3),('p090',.9)] for policy in ('N','A','C')],
        parameters=dict(mode='hardware',rolling_allocation='least_squares',track_contact_point=False))


class RunnerContracts(unittest.TestCase):
    def test_fixed_nine_arm_protocol_and_unmodified_parameters(self):
        value=protocol()
        self.assertEqual(run_case.validate_protocol(value,'p030_C')['policy'],'C')
        value['cases'][0]['policy']='C'
        with self.assertRaises(ValueError):run_case.validate_protocol(value,'p030_C')
        value=protocol();value['parameters']['track_contact_point']=True
        with self.assertRaises(ValueError):run_case.validate_protocol(value,'p030_C')

    def test_truth_centroid_preserves_both_native_families_without_selector_input(self):
        records=[dict(name='shell_A_smooth',normal_force=2.,pos=np.array([1.,0.,0.])),
                 dict(name='shell_A_rim',normal_force=1.,pos=np.array([4.,0.,0.])),
                 dict(name='belt_B_smooth',normal_force=3.,pos=np.array([0.,2.,0.]))]
        plant=NS(data=NS(time=.005),contact_records=lambda:records)
        measurement=dict(loads_N=dict(shell_A=3.,shell_B=0.,belt_A=0.,belt_B=3.),point_m=np.array([1.,1.,0.]))
        row=run_case.truth_row(plant,measurement)
        np.testing.assert_array_equal(row['truth_patch_points_m'][0],[2.,0.,0.])
        np.testing.assert_array_equal(row['truth_patch_points_m'][3],[0.,2.,0.])
        self.assertTrue(np.isnan(row['truth_patch_points_m'][1:3]).all())

    def test_feedback_boundary_rejects_changed_gain_support_or_raw_reference(self):
        gain=np.arange(90,dtype=float).reshape(5,18)
        controller=NS(_held=dict(loads={'shell_A':10.},gain=gain.copy()),
            ordinary=dict(design=dict(K=gain.copy())),params=NS(support_feedforward=.6),
            support_torque=lambda normal,loads:np.array([1.,2.,3.,0.,0.]),
            reference_attitude=lambda normal,acceleration:np.array([1.,0.,0.,0.]))
        info=dict(estimate=dict(normal=np.array([0.,0.,1.])),measurement=dict(loads_N={'shell_A':10.}),
            feedback_gain=gain.copy(),support_command=np.array([.6,1.2,1.8,0.,0.]),
            reference_quat=np.array([1.,0.,0.,0.]),reference_qpos=np.r_[.2,.3,np.zeros(8)])
        # Use the actual multiplication to avoid decimal-literal rounding in this exact guard.
        info['support_command']=.6*np.array([1.,2.,3.,0.,0.])
        run_case.assert_native_feedback(controller,info,np.array([.2,.3]),np.zeros(2))
        for field,index in [('feedback_gain',(1,3)),('support_command',0),('reference_quat',1),('reference_qpos',0)]:
            original=info[field].copy();info[field][index]+=.1
            with self.subTest(field=field):
                with self.assertRaises(AssertionError):run_case.assert_native_feedback(controller,info,np.array([.2,.3]),np.zeros(2))
            info[field]=original

    def test_fingerprint_retains_nonzero_native_gain_and_load_state(self):
        before=run_case.fingerprint(dict(gain=np.arange(90).reshape(5,18),loads=np.array([2.,0.,0.,1.])))
        after=run_case.fingerprint(dict(gain=np.arange(90).reshape(5,18),loads=np.array([1.,0.,0.,2.])))
        self.assertNotEqual(before,after)


if __name__=='__main__':unittest.main()
