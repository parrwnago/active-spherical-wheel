"""Real N/H native command plus read-only telemetry, without integration."""
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
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'adapter'))
from heading_telemetry import heading_diagnostics
import run_case
from spherical_wheel.mirror import StateMirror
from spherical_wheel.model import load_config,REPO_ROOT
from spherical_wheel.offroad_control import OffroadController,OffroadParams
CACHE_SHA='45ab9990aa3961d1202533acf51d5d8a9294eefe5a4c7e6cfde6aa0b3ac462c1'


def _snapshot(value):
    if isinstance(value,dict):return tuple((k,_snapshot(value[k])) for k in sorted(value))
    if isinstance(value,(list,tuple)):return tuple(_snapshot(x) for x in value)
    if value is None or isinstance(value,str):return value
    a=np.asarray(value)
    if a.dtype.hasobject:raise TypeError('Unsupported mutable boundary')
    return a.dtype.str,a.shape,a.tobytes()


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


class NativeTelemetryIdentity(unittest.TestCase):
    def test_real_native_N_and_H_command_and_history_unchanged_by_instrumentation(self):
        design,ordinary=frozen_design(REPO_ROOT/'generated/offroad/flat/control_schedule.npz')
        with tempfile.TemporaryDirectory(prefix='i74_native_identity_') as folder:
            for policy in ('N','H'):
                parameters=dict(run_case.BASELINE_SETTINGS)
                if policy=='H':parameters.update(run_case.HEADING_SETTINGS)
                mirrors=[StateMirror(load_config(),Path(folder)/(policy+name)) for name in ('original','telemetry')]
                q=ordinary['equilibrium']['qpos'].copy();q[:2]=[.21,-.17]
                v=np.array([.025,-.01,0.,.02,-.015,.01,.03,.01,-.02])
                previous=np.array([.1,-.03,.02,.025,-.015])
                for mirror in mirrors:mirror.set_state(q,v,.5,previous)
                controllers=[OffroadController(mirror,design,OffroadParams(**parameters),.3,record_diagnostics=True) for mirror in mirrors]
                def forbidden(*args,**kwargs):raise AssertionError('No physical stepping/contact force/collision allowed')
                with ExitStack() as guards:
                    for name in ('mj_step','mj_step1','mj_step2','mj_collision','mj_contactForce'):
                        guards.enter_context(patch.object(mujoco,name,forbidden))
                    for velocity in (np.array([.04,.015]),np.zeros(2)):
                        with self.subTest(policy=policy,moving=bool(np.linalg.norm(velocity)>0)):
                            args=(q[:2]+[.01,-.02],velocity,np.array([.005,-.003]))
                            expected,expected_info=controllers[0].command(*args)
                            actual,info=controllers[1].command(*args)
                            heading_diagnostics(controllers[1],info)
                            run_case.assert_native_feedback(controllers[1],info,args[0],args[2])
                            self.assertEqual(_snapshot(expected),_snapshot(actual))
                            self.assertEqual(_snapshot(expected_info),_snapshot(info))
                            self.assertEqual(history(controllers[0]),history(controllers[1]))
                            np.testing.assert_array_equal(mirrors[0].data.qpos,mirrors[1].data.qpos)
                            np.testing.assert_array_equal(mirrors[0].data.qvel,mirrors[1].data.qvel)
                            if policy=='N':np.testing.assert_array_equal(info['heading_command'],np.zeros(5))


if __name__=='__main__':unittest.main()
