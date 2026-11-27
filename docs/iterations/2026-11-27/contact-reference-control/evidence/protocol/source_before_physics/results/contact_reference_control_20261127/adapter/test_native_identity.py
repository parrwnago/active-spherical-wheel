"""Bounded full native-controller identity at fixed states; never steps physics.

The real gain matrices are copied from the frozen flat bank. Two independent
StateMirrors receive identical synthetic q/v and prior held inputs. No terrain,
Gazebo, new gain design, contact truth or historical accuracy scoring is used.
"""
from contextlib import ExitStack
import hashlib
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import unittest

import mujoco
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
from contact_reference_adapter import create, _snapshot
from spherical_wheel.mirror import StateMirror
from spherical_wheel.model import load_config, REPO_ROOT
from spherical_wheel.offroad_control import OffroadController, OffroadParams


CACHE_SHA = '45ab9990aa3961d1202533acf51d5d8a9294eefe5a4c7e6cfde6aa0b3ac462c1'


def frozen_design(path):
    if hashlib.sha256(path.read_bytes()).hexdigest()!=CACHE_SHA:
        raise AssertionError('Original flat gain bank changed')
    with np.load(path,allow_pickle=False) as source:
        values={key:source[key].copy() for key in source.files}
    points=[]
    for i,phase in enumerate(values['phase_rad']):
        points.append(dict(phase=float(phase),K=values['K'][i].copy(),
            regime='belt' if values['regime_code'][i] else 'shell',
            side='B' if values['side_code'][i] else 'A',
            eq=dict(qpos=values['qpos'][i].copy(),qvel=values['qvel'][i].copy(),u=values['equilibrium_u'][i].copy())))
    selected=points[int(np.argmin(np.abs(values['phase_rad']-.3)))]
    ordinary=dict(equilibrium=selected['eq'],design=dict(K=selected['K'].copy()))
    return SimpleNamespace(weights='flat',points=points,local=lambda phase:ordinary),ordinary


def history(controller):
    return _snapshot(dict(nominal_joints=controller.nominal_joints,held=controller._held,
        vref=controller._vref,sync=controller._sync,plane=controller._plane,
        estimator=vars(controller.estimator),observer_generalized=controller.observer.generalized,
        observer_interval_mean=controller.observer.interval_mean,observer_previous=controller.observer._previous,
        slow_normal=controller._slow_normal,fall_end=controller._fall_end,yaw_error=controller._yaw_error))


class NativeIdentityContract(unittest.TestCase):
    def test_full_controller_native_outputs_and_history_exact_on_motion_and_stop(self):
        cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz'
        design,ordinary=frozen_design(cache)
        with tempfile.TemporaryDirectory(prefix='i73_native_identity_') as folder:
            mirrors=[StateMirror(load_config(),Path(folder)/name) for name in ('original','wrapped')]
            q=ordinary['equilibrium']['qpos'].copy();q[:2]=[.21,-.17]
            v=np.array([.025,-.01,0.,.02,-.015,.01,.03,.01,-.02])
            previous=np.array([.1,-.03,.02,.025,-.015])
            for mirror in mirrors:mirror.set_state(q,v,.5,previous)
            controllers=[OffroadController(mirror,design,OffroadParams(mode='hardware',rolling_allocation='least_squares',
                track_contact_point=False),.3,record_diagnostics=True) for mirror in mirrors]
            adapter=create(controllers[1],'N')
            original_feature_evaluate=adapter.features.evaluate
            def forbidden(*args,**kwargs):raise AssertionError('No stepping, contacts or N feature evaluation allowed')
            with ExitStack() as guards:
                for name in ('mj_step','mj_step1','mj_step2','mj_collision','mj_contactForce'):
                    guards.enter_context(patch.object(mujoco,name,forbidden))
                guards.enter_context(patch.object(adapter.features,'evaluate',forbidden))
                for velocity in (np.array([.04,.015]),np.zeros(2)):
                    with self.subTest(moving=bool(np.linalg.norm(velocity)>0)):
                        args=(q[:2]+[.01,-.02],velocity,np.array([.005,-.003]))
                        expected,expected_info=controllers[0].command(*args)
                        with adapter.command_context(controllers[1],velocity):
                            actual,actual_info=controllers[1].command(*args)
                        diagnostics=adapter.diagnostics(actual_info)
                        self.assertEqual(_snapshot(actual),_snapshot(expected))
                        self.assertEqual(_snapshot(actual_info),_snapshot(expected_info))
                        self.assertEqual(history(controllers[1]),history(controllers[0]))
                        self.assertFalse(diagnostics['reference_override'])
                        np.testing.assert_array_equal(diagnostics['delta_vref'],np.zeros(9))
                        np.testing.assert_array_equal(diagnostics['delta_feedforward'],np.zeros(5))
                        np.testing.assert_array_equal(mirrors[0].data.qpos,mirrors[1].data.qpos)
                        np.testing.assert_array_equal(mirrors[0].data.qvel,mirrors[1].data.qvel)
            self.assertTrue(callable(original_feature_evaluate))


if __name__=='__main__':unittest.main()
