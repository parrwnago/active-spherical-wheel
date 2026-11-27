"""Selected-load/frame tests with synthetic robot geometry only."""
from copy import deepcopy
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np
import pytest

from spherical_wheel import contact_hypotheses
from spherical_wheel.contact_load_consistency import ContactLoadFeatures, load_compatibility


def model_and_case(family=0, yaw=.6, force_local=(0., 5., 10.)):
    model = mujoco.MjModel.from_xml_string('''<mujoco><size nuser_geom="5"/>
      <default><geom contype="0" conaffinity="0"/></default><worldbody>
      <body name="fork" pos="0 0 .1"><freejoint/><geom type="sphere" size=".01" mass="1"/>
       <body name="carrier"><joint name="shaft" type="hinge" axis="0 1 0"/>
        <geom name="belt_A_smooth" type="sphere" size=".1" mass=".01" user="1 .09 .1 4 .1"/>
        <geom name="belt_B_smooth" type="sphere" size=".1" mass=".01" user="-1 .09 .1 4 .1"/>
        <body name="shell_A"><joint name="A" type="hinge" axis="1 0 0"/>
         <geom name="shell_A_smooth" type="sphere" size=".1" mass=".1" user="1 .01 .09 2 .1"/>
        </body>
        <body name="shell_B"><joint name="B" type="hinge" axis="1 0 0"/>
         <geom name="shell_B_smooth" type="sphere" size=".1" mass=".1" user="-1 .01 .09 2 .1"/>
        </body>
       </body>
      </body></worldbody></mujoco>''')
    q = model.qpos0.copy();q[3:7] = [np.cos(.5*yaw), 0., 0., np.sin(.5*yaw)]
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    sign = -1. if family in (1, 3) else 1.
    offset = rotation@np.array([sign*.09, 0., -np.sqrt(.1**2-.09**2)])
    force = rotation@np.asarray(force_local);moment = np.cross(offset, force)
    joints = np.zeros(3);joints[0] = rotation[:, 1]@moment
    if family < 2:
        joints[1+family] = rotation[:, 0]@moment
    wrench = np.r_[force, moment];generalized = np.r_[wrench, joints]
    return model, q, q[:3]+offset, wrench, generalized, rotation


@pytest.mark.parametrize('family', [0, 1, 2, 3])
def test_rotated_base_predicts_world_wrench_and_correct_material_joint(family):
    model, q, point, wrench, generalized, _ = model_and_case(family)
    features = ContactLoadFeatures(model).evaluate(q, point, family, wrench, generalized)
    assert features['valid']
    np.testing.assert_allclose(features['prediction'], generalized, atol=1e-13)
    np.testing.assert_allclose(features['residual'], 0., atol=1e-13)
    assert load_compatibility(features)['passes']


def test_belt_load_has_zero_material_joint_rows_and_rejects_wrong_shell_load():
    model, q, point, wrench, generalized, _ = model_and_case(2)
    generator = ContactLoadFeatures(model)
    correct = generator.evaluate(q, point, 2, wrench, generalized)
    wrong = generator.evaluate(q, point, 0, wrench, generalized)
    assert correct['valid'] and wrong['valid']  # shared spherical-band boundary
    np.testing.assert_allclose(correct['prediction'][7:9], 0., atol=1e-14)
    assert abs(wrong['prediction'][7]) > .2
    assert load_compatibility(correct)['passes']
    assert load_compatibility(wrong)['vetoed']


def test_virtual_work_uses_world_base_angular_velocity_after_rotation():
    model, q, point, wrench, generalized, rotation = model_and_case(0)
    generator = ContactLoadFeatures(model)
    features = generator.evaluate(q, point, 0, wrench, generalized)
    velocity = np.array([.03, -.02, .01, .7, -.4, .5, -.8, 1.2, .3])
    jacobian = np.zeros((3, model.nv))
    mujoco.mj_jac(model, generator.data, jacobian, None, point, generator.geometry.patches[0]['body'])
    physical_power = wrench[:3]@(jacobian@velocity)
    mixed_velocity = velocity.copy();mixed_velocity[3:6] = rotation@velocity[3:6]
    np.testing.assert_allclose(features['prediction']@mixed_velocity, physical_power, atol=1e-13)


def test_affine_joint_maps_match_direct_point_jacobian_even_off_sphere():
    model, q, point, wrench, generalized, _ = model_and_case(0)
    generator = ContactLoadFeatures(model);maps = generator.joint_maps(q, wrench)
    assert maps['valid']
    # An oracle load-weighted mean can lie off the material sphere. The feature
    # maps do not clamp or reject it; it enters only this later diagnostic.
    off_surface = point+np.array([.003, -.007, .005])
    for family, patch in enumerate(generator.geometry.patches):
        jacobian = np.zeros((3, model.nv))
        mujoco.mj_jac(model, generator.data, jacobian, None, off_surface, patch['body'])
        affine = maps['joint_at_centre'][family]+maps['joint_moment_map'][family]@np.cross(
            off_surface-maps['centre_m'], wrench[:3])
        np.testing.assert_allclose(affine, (jacobian.T@wrench[:3])[6:9], atol=1e-13)
    assert not generator.evaluate(q, off_surface, 0, wrench, generalized)['valid']


def test_generalized_point_load_is_linear_in_force():
    first = model_and_case(0, force_local=(0., 2., 10.))
    second = model_and_case(0, force_local=(0., 3., 7.))
    model, q, point = first[:3];generator = ContactLoadFeatures(model)
    a = generator.evaluate(q, point, 0, first[3], first[4])
    b = generator.evaluate(q, point, 0, second[3], second[4])
    total = generator.evaluate(q, point, 0, first[3]+second[3], first[4]+second[4])
    np.testing.assert_allclose(total['prediction'], a['prediction']+b['prediction'], atol=1e-13)


def test_shaft_load_does_not_distinguish_shared_point_belt_from_shell():
    model, q, point, wrench, generalized, _ = model_and_case(2)
    generator = ContactLoadFeatures(model)
    belt = generator.evaluate(q, point, 2, wrench, generalized)
    shell = generator.evaluate(q, point, 0, wrench, generalized)
    np.testing.assert_allclose(belt['prediction'][:7], shell['prediction'][:7], atol=1e-13)
    assert not np.allclose(belt['prediction'][7:], shell['prediction'][7:])


def test_axis_aligned_contact_force_can_leave_body_identity_unobservable():
    model, q, point, wrench, generalized, _ = model_and_case(2, force_local=(-10., 0., 0.))
    generator = ContactLoadFeatures(model)
    belt = generator.evaluate(q, point, 2, wrench, generalized)
    shell = generator.evaluate(q, point, 0, wrench, generalized)
    np.testing.assert_allclose(belt['prediction'], shell['prediction'], atol=1e-13)
    assert load_compatibility(belt)['passes'] and load_compatibility(shell)['passes']


def test_single_point_gate_can_pass_a_multicontact_distribution():
    model, q, _, _, _, _ = model_and_case(0, yaw=0.)
    generator = ContactLoadFeatures(model)
    # Two equal radial normal forces on opposing shell bands cancel moments
    # and material-joint torques. One rim hypothesis remains within the declared
    # tolerance; compatibility cannot prove one physical contact.
    total_force = np.array([0., 0., 20.*np.sqrt(1.-.9**2)])
    wrench = np.r_[total_force, np.zeros(3)]
    generalized = np.r_[wrench, np.zeros(3)]
    rim = q[:3]+np.array([.01, 0., -np.sqrt(.1**2-.01**2)])
    features = generator.evaluate(q, rim, 0, wrench, generalized)
    assert features['valid']
    assert load_compatibility(features)['passes']


def test_reconstructor_matches_frozen_i60_points_without_scoring():
    model, q, _, _, _, _ = model_and_case(0, yaw=0.)
    generator = ContactLoadFeatures(model)
    normal = np.array([0., 0., 1.])
    rich = generator.geometry.evaluate(q, np.zeros(9), np.zeros(5), normal,
                                       [0., 0., 10., 0., 0., 0.])
    for index in np.flatnonzero(rich['valid'])[::4]:
        np.testing.assert_array_equal(generator.reconstruct_point(q, normal, index), rich['point_m'][index])


def test_feature_generation_does_not_read_contacts_step_or_modify_inputs(monkeypatch):
    model, q, point, wrench, generalized, _ = model_and_case(0)
    generator = ContactLoadFeatures(model)
    live = mujoco.MjData(model)
    before = [x.copy() for x in (q, point, wrench, generalized, model.geom_surfacevel, live.qpos, live.qvel)]
    def forbidden(*args, **kwargs):
        raise AssertionError('No solver or dynamics call allowed')
    for name in ('mj_forward', 'mj_step', 'mj_contactForce', 'mj_collision'):
        monkeypatch.setattr(mujoco, name, forbidden)
    assert generator.evaluate(q, point, 0, wrench, generalized)['valid']
    assert generator.joint_maps(q, wrench)['valid']
    for value, expected in zip((q, point, wrench, generalized, model.geom_surfacevel, live.qpos, live.qvel), before):
        np.testing.assert_array_equal(value, expected)


def gate_features(residual=None, valid=True):
    return dict(valid=valid, residual=np.zeros(9) if residual is None else np.asarray(residual, float),
                radius_m=.1, force_norm_N=10.)


def test_gate_uses_only_extra_joint_rows_and_the_declared_additive_scale():
    residual = np.r_[np.full(6, 1e6), [.12, -.12, .01]]
    passed = load_compatibility(gate_features(residual))
    np.testing.assert_allclose(passed['threshold_Nm'], .13, atol=1e-15)
    assert passed['passes']
    residual[-1] = .14
    failed = load_compatibility(gate_features(residual))
    assert failed['vetoed'] and not failed['passes']


def test_gate_batch_matches_scalar_without_mutation_and_invalid_is_unknown():
    inputs = gate_features(np.array([np.zeros(9), np.r_[np.zeros(8), .2], np.full(9, np.nan)]))
    inputs['valid'] = np.array([True, True, False]);before = deepcopy(inputs)
    batch = load_compatibility(inputs)
    assert batch['passes'].tolist() == [True, False, False]
    assert batch['vetoed'].tolist() == [False, True, False]
    assert batch['unknown'].tolist() == [False, False, True]
    for key, value in inputs.items():
        np.testing.assert_array_equal(value, before[key])
    scalar = load_compatibility(gate_features())
    np.testing.assert_array_equal(batch['normalized_joint_residual'][0], scalar['normalized_joint_residual'])


def test_mismatched_wrench_or_missing_point_is_unknown_not_compatible():
    model, q, point, wrench, generalized, _ = model_and_case(0)
    generator = ContactLoadFeatures(model)
    changed = wrench.copy();changed[0] += .1
    wrong = generator.evaluate(q, point, 0, changed, generalized)
    missing = generator.evaluate(q, np.full(3, np.nan), -1, wrench, generalized)
    for feature in (wrong, missing):
        assert not feature['valid']
        assert load_compatibility(feature)['unknown']


def test_numpy_gate_imports_without_mujoco_or_repository_package():
    path = Path(__file__).resolve().parents[1]/'src/spherical_wheel/contact_load_consistency.py'
    script = '''import builtins,importlib.util,numpy as np,sys
old=builtins.__import__
def guard(name,*args,**kwargs):
 if name=='mujoco' or name.startswith('spherical_wheel'):
  raise ImportError('intentionally unavailable')
 return old(name,*args,**kwargs)
builtins.__import__=guard
spec=importlib.util.spec_from_file_location('load_gate',sys.argv[1]);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.load_compatibility(dict(valid=True,residual=np.zeros(9),radius_m=.1,force_norm_N=10.))['passes']
'''
    subprocess.run([sys.executable, '-c', script, str(path)], check=True)
