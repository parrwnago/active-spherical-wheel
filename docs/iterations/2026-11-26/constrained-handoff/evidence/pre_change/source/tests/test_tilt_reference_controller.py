"""Opt-in tilt integration and private-history contracts; no Gazebo execution."""
import copy
from contextlib import ExitStack
from dataclasses import asdict
from unittest.mock import patch

import mujoco
import numpy as np
import pytest

from spherical_wheel.contact_prediction import forecast_native_contacts
from spherical_wheel.model import REPO_ROOT
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import Workspace
from spherical_wheel.predictive_model import EstimatedPlanePredictor
from spherical_wheel.predictive_seed import clone_native_controller, generate_native_rollout_seed
from spherical_wheel.tilt_reference import TiltReferenceSettings


@pytest.fixture(scope='module')
def factory(tmp_path_factory):
    workspace=Workspace(directory=tmp_path_factory.mktemp('tilt_integration'),
        seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz')
    def make(settings=None):
        mirror=copy.deepcopy(workspace.mirror())
        params=OffroadParams(mode='hardware',rolling_allocation='least_squares',tilt_reference=settings)
        controller=OffroadController(mirror,workspace.design,params,phase=.3,record_diagnostics=True)
        equilibrium=controller.ordinary['equilibrium']
        mirror.set_state(equilibrium['qpos'],np.zeros(9),0.,equilibrium['u'])
        controller.reset(.3)
        controller.nominal_joints=mirror.data.qpos[7:10].copy()
        return controller
    return make


def same(a,b):
    if isinstance(a,dict):
        assert a.keys()==b.keys()
        for key in a:same(a[key],b[key])
    elif isinstance(a,(tuple,list)):
        assert len(a)==len(b)
        for x,y in zip(a,b):same(x,y)
    elif isinstance(a,np.ndarray):
        np.testing.assert_array_equal(a,b)
    else:
        assert a==b


def command_at(controller,time,acceleration=(0.,0.),velocity=(0.,0.),previous=None):
    plant=controller.plant
    plant.set_state(plant.data.qpos.copy(),plant.data.qvel.copy(),time,
                    plant.last_input.copy() if previous is None else previous)
    return controller.command(plant.data.qpos[:2].copy(),np.asarray(velocity),np.asarray(acceleration))


def prime(controller):
    controller.lean_vector=lambda normal,acceleration: np.asarray(acceleration,float)
    for time,acceleration in ((4.,(0.,0.)),(4.005,(.1,.04)),(4.010,(.1,.04))):
        current,info=command_at(controller,time,acceleration)
    return current,info


def test_legacy_parameter_dictionary_omits_disabled_feature():
    params=OffroadParams(mode='hardware')
    expected=asdict(params);expected.pop('tilt_reference')
    assert params.as_dict()==expected
    assert 'tilt_reference' not in params.as_dict()
    assert OffroadParams(**params.as_dict()).as_dict()==params.as_dict()


def test_enabled_settings_serialize_and_round_trip():
    settings=TiltReferenceSettings(activation_time_s=4.)
    params=OffroadParams(mode='hardware',tilt_reference=settings)
    assert params.as_dict()['tilt_reference']==asdict(settings)
    restored=OffroadParams(**params.as_dict())
    assert restored.tilt_reference==settings
    assert restored.as_dict()==params.as_dict()


@pytest.mark.parametrize('settings',[True,{'time_constant_s':0.},{'use_rate_reference':'yes'}])
def test_invalid_opt_in_settings_are_rejected(settings):
    with pytest.raises(ValueError):OffroadParams(mode='hardware',tilt_reference=settings)


def test_disabled_and_waiting_paths_preserve_native_commands_bitwise(factory):
    baseline=factory();candidate=factory(TiltReferenceSettings(activation_time_s=4.))
    previous=baseline.plant.last_input.copy()
    for time in (0.,.005,.010,1.,3.995,4.):
        a,ia=command_at(baseline,time,(.04,-.02),previous=previous)
        b,ib=command_at(candidate,time,(.04,-.02),previous=previous)
        np.testing.assert_array_equal(a,b)
        for key in ('requested','reference_quat','reference_qvel','feedback_reference_qvel','feedback_gain'):
            np.testing.assert_array_equal(ia[key],ib[key])
        if time<4:
            assert candidate._tilt_reference.state_dict() is None
            assert not ib['tilt_reference']['active']
        else:
            assert ib['tilt_reference']['initialized_now']
            np.testing.assert_array_equal(ib['tilt_reference']['raw_reference_quat'],ib['reference_quat'])
            assert ib['reference_qvel'][3:6].tobytes()==np.zeros(3).tobytes()
        previous=a
        baseline.after_step();candidate.after_step()
    assert baseline._tilt_reference is None


def test_raw_accessor_and_after_step_never_advance_generator(factory):
    controller=factory(TiltReferenceSettings(activation_time_s=4.));prime(controller)
    before=copy.deepcopy(controller._tilt_reference.state_dict())
    for normal in ([0.,0.,1.],[.1,-.1,.98994949366]):
        controller.reference_attitude(normal,np.array([.1,.2]))
        controller.reference_attitude(normal,np.array([-.1,.2]))
    controller.after_step()
    same(before,controller._tilt_reference.state_dict())
    controller.reset(.3)
    assert controller._tilt_reference.state_dict() is None


def test_transported_rate_is_applied_before_feedback_and_logged_consistently(factory):
    controller=factory(TiltReferenceSettings(activation_time_s=4.));_,info=prime(controller)
    tilt=info['tilt_reference']
    assert np.linalg.norm(tilt['theta_dot_rad_s'])>0
    rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,controller.ordinary['equilibrium']['qpos'][3:7])
    expected_world=rotation.reshape(3,3)@tilt['angular_velocity_equilibrium_rad_s']
    actual_rotation=controller.plant.data.xmat[controller.plant.fork_id].reshape(3,3)
    expected_body=actual_rotation.T@expected_world
    np.testing.assert_allclose(tilt['angular_velocity_world_rad_s'],expected_world,atol=1e-15)
    np.testing.assert_allclose(info['reference_qvel'][3:6],expected_body,atol=1e-15)
    np.testing.assert_array_equal(info['reference_qvel'][3:6],info['feedback_reference_qvel'][3:6])
    np.testing.assert_array_equal(tilt['used_angular_velocity_body_rad_s'],info['reference_qvel'][3:6])


def test_angle_only_and_rate_modes_shape_identical_saved_inputs(factory):
    angle=factory(TiltReferenceSettings(activation_time_s=4.,use_rate_reference=False))
    rate=factory(TiltReferenceSettings(activation_time_s=4.,use_rate_reference=True))
    common_previous=angle.plant.last_input.copy()
    for c in (angle,rate):c.lean_vector=lambda n,a:np.asarray(a,float)
    for time,acceleration in ((4.,(0.,0.)),(4.005,(.1,.04)),(4.010,(.1,.04)),(4.015,(-.05,.08))):
        _,a=command_at(angle,time,acceleration,previous=common_previous)
        _,b=command_at(rate,time,acceleration,previous=common_previous)
        same(angle._tilt_reference.state_dict(),rate._tilt_reference.state_dict())
        np.testing.assert_array_equal(a['reference_quat'],b['reference_quat'])
        np.testing.assert_array_equal(a['reference_qvel'][3:6],np.zeros(3))
    assert np.linalg.norm(b['reference_qvel'][3:6])>0


def test_new_rate_overwrites_stale_dropout_rolling_reference(factory,monkeypatch):
    controller=factory(TiltReferenceSettings(activation_time_s=4.));prime(controller)
    assert controller._held is not None
    controller._held['vref'][3:6]=[7.,8.,9.]
    monkeypatch.setattr(controller.locator,'records',lambda *args:[])
    _,info=command_at(controller,4.015,(.1,.04),velocity=(.02,0.))
    assert not info['measurement']['in_contact']
    np.testing.assert_array_equal(info['reference_qvel'][3:6],info['tilt_reference']['used_angular_velocity_body_rad_s'])
    np.testing.assert_array_equal(info['feedback_reference_qvel'][3:6],info['reference_qvel'][3:6])
    assert np.linalg.norm(info['reference_qvel'][3:6])>0
    assert not np.array_equal(info['reference_qvel'][3:6],[7.,8.,9.])


def test_enabled_generator_is_independent_in_native_clone(factory):
    controller=factory(TiltReferenceSettings(activation_time_s=4.));prime(controller)
    before=copy.deepcopy(controller._tilt_reference.state_dict())
    clone=clone_native_controller(controller)
    assert clone._tilt_reference is not controller._tilt_reference
    assert not np.shares_memory(clone._tilt_reference._theta,controller._tilt_reference._theta)
    assert not np.shares_memory(clone._tilt_reference._z,controller._tilt_reference._z)
    clone._tilt_reference.update(np.array([.02,-.01]),4.015)
    clone._tilt_reference._theta+=.01
    same(before,controller._tilt_reference.state_dict())
    clone.reset(.3)
    same(before,controller._tilt_reference.state_dict())


def test_enabled_seed_and_fine_forecast_keep_live_reference_history_and_anchor_cadence(factory):
    controller=factory(TiltReferenceSettings(activation_time_s=0.))
    goal=controller.plant.data.qpos[:2].copy()
    path=lambda t:(goal.copy(),np.array([.005,0.]),np.array([.03,.02]) if t>=.005 else np.zeros(2))
    current=None
    for time in (0.,.005,.010):
        controller.plant.set_state(controller.plant.data.qpos.copy(),controller.plant.data.qvel.copy(),time,
            controller.plant.last_input.copy() if current is None else current)
        current,info=controller.command(*path(time))
    assert np.linalg.norm(info['tilt_reference']['theta_dot_rad_s'])>0
    predictor=EstimatedPlanePredictor(controller.plant)
    predictor.update(controller.plant.data.qpos,controller.plant.data.qvel,info['estimate']['normal'],
        info['measurement']['point_m'],sum(info['measurement']['loads_N'].values()),
        friction=info['estimate']['friction'],command=controller.plant.last_input)
    before=copy.deepcopy(controller._tilt_reference.state_dict())
    original=OffroadController.command;seen=[]
    def callback(clone,*args):
        assert clone._tilt_reference is not controller._tilt_reference
        seen.append(clone.plant.data.time)
        result=original(clone,*args)
        assert clone._tilt_reference.state_dict()['time_s']==clone.plant.data.time
        return result
    def forbidden(*args,**kwargs):raise AssertionError('Solved contact reached native clone')
    with ExitStack() as stack:
        for target in ('spherical_wheel.offroad_control.measure_contact','spherical_wheel.offroad_control.records_from_mujoco',
                       'spherical_wheel.offroad_control.rolling_feedforward','spherical_wheel.synchronization.ContactSynchronizer._normal_loads','mujoco.mj_contactForce'):
            stack.enter_context(patch(target,forbidden))
        seed,_=generate_native_rollout_seed(controller,predictor,path,.010,4,.005,current_command=current,command_function=callback)
        first_times=list(seen);seen.clear()
        forecast=forecast_native_contacts(controller,predictor,path,.010,.020,current_command=current,command_function=callback)
    np.testing.assert_allclose(first_times,[.015,.020,.025],atol=1e-14)
    np.testing.assert_allclose(seen,first_times,atol=0)
    np.testing.assert_array_equal(seed,forecast['commands'])
    np.testing.assert_array_equal(seed[0],current)
    same(before,controller._tilt_reference.state_dict())
