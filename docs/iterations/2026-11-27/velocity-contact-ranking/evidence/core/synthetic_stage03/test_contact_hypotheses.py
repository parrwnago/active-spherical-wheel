"""Synthetic information/geometry tests; no historical controller trials."""
from copy import deepcopy
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np
import pytest

from spherical_wheel.contact_hypotheses import (
    ContactHypothesisFeatures, MODES, normal_seeds, project_features, rank_hypotheses)


def synthetic_model():
    return mujoco.MjModel.from_xml_string('''<mujoco><size nuser_geom="5"/>
    <default><geom contype="0" conaffinity="0"/></default><worldbody>
      <body name="fork" pos="0 0 .1"><freejoint/>
        <geom type="sphere" size=".01" mass="1"/>
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


def state(model):
    return model.qpos0.copy(), np.zeros(model.nv), np.zeros(5)


def scalar_features(vectors=((0., 0., 0.), (.04, 0., 0.)), *, labels=(0, 1),
                    angles=None, moments=None, excitation=.1, valid=None):
    vectors = np.asarray(vectors, float)
    count = len(vectors)
    return dict(family_index=np.asarray(labels), valid=np.ones(count, bool) if valid is None else np.asarray(valid),
        frame_valid=True, excitation_m_s=excitation,
        velocity_residual_m_s=vectors, velocity_residual_norm_m_s=np.linalg.norm(vectors, axis=-1),
        normal_angle_rad=np.zeros(count) if angles is None else np.asarray(angles),
        moment_residual_norm=np.zeros(count) if moments is None else np.asarray(moments))


def test_seed_cone_contains_fixed_centre_and_two_rings():
    normal = np.array([.3, -.4, .5]);normal /= np.linalg.norm(normal)
    seeds = normal_seeds(normal)
    assert seeds.shape == (17, 3)
    np.testing.assert_allclose(np.linalg.norm(seeds, axis=1), 1., atol=1e-14)
    np.testing.assert_allclose(seeds[0], normal)
    angle = np.arccos(np.clip(seeds@normal, -1., 1.))
    np.testing.assert_allclose(angle[1:9], np.deg2rad(5), atol=1e-14)
    np.testing.assert_allclose(angle[9:], np.deg2rad(10), atol=1e-14)


def test_candidate_points_belong_to_their_own_bands_and_rims_without_contacts(monkeypatch):
    model = synthetic_model();generator = ContactHypothesisFeatures(model)
    q, v, previous = state(model)
    def forbidden(*args, **kwargs):
        raise AssertionError('No contact/dynamics operation is allowed')
    for name in ('mj_forward', 'mj_step', 'mj_contactForce', 'mj_collision'):
        monkeypatch.setattr(mujoco, name, forbidden)
    result = generator.evaluate(q, v, previous, [0., 0., 1.], [0., 0., 10., 0., 0., 0.])
    assert result['valid'].shape == (102,)
    assert result['valid'].any()
    for i in np.flatnonzero(result['valid']):
        patch = generator.patches[int(result['family_index'][i])]
        offset = result['point_m'][i]-generator.data.xpos[generator.centre_body]
        axis = patch['sign']*generator.data.xmat[patch['body']].reshape(3, 3)[:, 0]
        axial = axis@offset
        assert patch['lower']-1e-12 <= axial <= patch['upper']+1e-12
        if result['kind'][i] == 1:
            assert np.isclose(axial, patch['lower'], atol=1e-12)
        assert np.isclose(np.linalg.norm(offset), generator.radius)
        assert result['normal_angle_rad'][i] <= np.deg2rad(10)+1e-12
        assert result['force_normal_N'][i] >= 1.


def test_same_body_motion_different_measured_joint_rates_change_contact_evidence():
    model = synthetic_model();generator = ContactHypothesisFeatures(model)
    q, v, previous = state(model)
    v[1] = -np.sqrt(.1**2-.01**2);v[7] = 1.
    first = project_features(generator.evaluate(q, v, previous, [0., 0., 1.], [0., 0., 10., 0., -.1, 0.]))
    ranking = rank_hypotheses(first)
    assert ranking['velocity']['chosen_family'] == 0
    assert ranking['velocity']['accepted_family'] == 0
    assert ranking['velocity']['best_velocity_residual_m_s'] < 1e-12
    # Preserve the same body motion (and therefore any same desired-vs-body error),
    # but move the measured rolling speed from shell A to shell B.
    v[7], v[8] = 0., 1.
    second = project_features(generator.evaluate(q, v, previous, [0., 0., 1.], [0., 0., 10., 0., .1, 0.]))
    ranking = rank_hypotheses(second)
    assert ranking['velocity']['chosen_family'] == 1
    assert ranking['velocity']['accepted_family'] == 1


def test_belt_uses_prior_held_speed_and_ignores_model_surface_field_and_torque_commands():
    model = synthetic_model();generator = ContactHypothesisFeatures(model)
    q, v, previous = state(model)
    yaw = .4
    q[3:7] = [np.cos(.5*yaw), 0., 0., np.sin(.5*yaw)]
    v[:3] = [.2*np.sin(yaw), -.2*np.cos(yaw), 0.];previous[3] = .2
    normal = [-np.cos(yaw), -np.sin(yaw), 0.]
    wrench = np.r_[10.*np.asarray(normal), np.zeros(3)]
    model.geom_surfacevel[:] = 123.
    first = generator.evaluate(q, v, previous, normal, wrench)
    belt = first['valid']&(first['family_index'] == 2)
    assert belt.any()
    assert np.max(np.linalg.norm(first['velocity_residual_m_s'][belt], axis=1)) < 1e-12
    previous[:3] = [1e4, -1e4, 3e4]
    second = generator.evaluate(q, v, previous, normal, wrench)
    np.testing.assert_array_equal(first['velocity_residual_m_s'], second['velocity_residual_m_s'])
    previous[3] = 0.
    third = generator.evaluate(q, v, previous, normal, wrench)
    assert np.max(np.linalg.norm(third['velocity_residual_m_s'][belt], axis=1)) > .19
    np.testing.assert_array_equal(model.geom_surfacevel, 123.)


def test_generator_preserves_inputs_live_state_and_model():
    model = synthetic_model();generator = ContactHypothesisFeatures(model)
    q, v, previous = state(model)
    live = mujoco.MjData(model);live.qpos[:] = q;live.qvel[:] = v
    before = [x.copy() for x in (q, v, previous, live.qpos, live.qvel, model.geom_surfacevel)]
    generator.evaluate(q, v, previous, [0., 0., 1.], [0., 0., 10., 0., 0., 0.])
    for actual, expected in zip((q, v, previous, live.qpos, live.qvel, model.geom_surfacevel), before):
        np.testing.assert_array_equal(actual, expected)


def test_reference_point_rigid_velocity_matches_direct_point_jacobians():
    model = synthetic_model();generator = ContactHypothesisFeatures(model)
    q, v, previous = state(model)
    q[3:7] = [np.cos(.3), 0., np.sin(.3), 0.]
    q[7:] = [.2, .4, -.3]
    v[:] = [.04, -.02, .01, -.3, .5, -.2, .7, -.9, .4]
    previous[3:] = [.12, -.08]
    normal = np.array([.1, .2, .97]);normal /= np.linalg.norm(normal)
    result = generator.evaluate(q, v, previous, normal, np.r_[10.*normal, np.zeros(3)])
    assert result['valid'].any()
    for index in np.flatnonzero(result['valid']):
        patch = generator.patches[int(result['family_index'][index])]
        jacobian = np.zeros((3, model.nv))
        mujoco.mj_jac(model, generator.data, jacobian, None, result['point_m'][index], patch['body'])
        actual = jacobian@v
        if patch['family'] >= 2:
            actual += generator.data.geom_xmat[patch['geom']].reshape(3, 3)[:, 1]*previous[patch['family']+1]
        n = result['normal'][index]
        actual -= n*(n@actual)
        np.testing.assert_allclose(result['velocity_residual_m_s'][index], actual, atol=1e-13, rtol=1e-12)


@pytest.mark.parametrize('force', [[0., 0., 0.], [0., 0., .9], [0., 0., -10.]])
def test_airborne_weak_and_tensile_loads_abstain(force):
    model = synthetic_model();generator = ContactHypothesisFeatures(model)
    q, v, previous = state(model)
    raw = generator.evaluate(q, v, previous, [0., 0., 1.], force+[0., 0., 0.])
    assert not raw['frame_valid']
    for score in rank_hypotheses(project_features(raw)).values():
        assert score['chosen_family'] == -1
        assert score['accepted_family'] == -1


def test_nonfinite_state_is_unknown_not_a_contact_vote():
    model = synthetic_model();generator = ContactHypothesisFeatures(model)
    q, v, previous = state(model);v[0] = np.nan
    raw = generator.evaluate(q, v, previous, [0., 0., 1.], [0., 0., 10., 0., 0., 0.])
    assert not raw['frame_valid']
    assert not raw['valid'].any()


def test_projected_and_rich_rankings_agree_and_projection_zeros_only_invalid_values():
    model = synthetic_model();generator = ContactHypothesisFeatures(model)
    q, v, previous = state(model);v[1], v[7] = -.1, 1.
    raw = generator.evaluate(q, v, previous, [0., 0., 1.], [0., 0., 10., 0., 0., 0.])
    projected = project_features(raw)
    assert not projected['velocity_residual_m_s'][~projected['valid']].any()
    left, right = rank_hypotheses(raw), rank_hypotheses(projected)
    for mode in MODES:
        for key in left[mode]:
            np.testing.assert_array_equal(left[mode][key], right[mode][key])


def test_family_minimum_uses_joint_candidate_score_not_independent_component_minima():
    features = scalar_features(vectors=((.04, 0., 0.), (0., 0., 0.), (.02, 0., 0.)),
        labels=(0, 0, 1), angles=(0., np.deg2rad(20), np.deg2rad(10)))
    result = rank_hypotheses(features)['velocity']
    assert result['chosen_family'] == 1
    np.testing.assert_allclose(result['family_scores'][:2], [4., 2.])


@pytest.mark.parametrize('changes,reason', [
    ({'excitation_m_s': 0.}, 'low excitation'),
    ({'velocity_residual_m_s': np.array([[.03, 0., 0.], [.06, 0., 0.]]),
      'velocity_residual_norm_m_s': np.array([.03, .06])}, 'all candidates slide'),
    ({'velocity_residual_m_s': np.zeros((2, 3)), 'velocity_residual_norm_m_s': np.zeros(2)}, 'indistinguishable'),
    ({'valid': np.array([True, False])}, 'geometry forced'),
])
def test_abstention_is_explicit_when_motion_does_not_identify_contact(changes, reason):
    features = scalar_features();features.update(changes)
    result = rank_hypotheses(features)['velocity']
    assert result['accepted_family'] == -1, reason
    assert not result['confident'], reason
    if reason == 'geometry forced':
        assert result['geometry_forced']
        assert result['valid_family_count'] == 1


def test_duplicate_candidates_cannot_create_family_confidence():
    features = scalar_features(vectors=((0., 0., 0.), (.04, 0., 0.), (.08, 0., 0.)), labels=(0, 0, 0))
    result = rank_hypotheses(features)['velocity']
    assert result['chosen_family'] == 0
    assert result['geometry_forced']
    assert result['accepted_family'] == -1


def test_exact_family_ties_are_flagged_and_never_accepted():
    result = rank_hypotheses(scalar_features())['geometry']
    assert result['score_tied']
    assert result['score_margin'] == 0
    assert result['chosen_family'] == 0  # deterministic descriptive ordering only
    assert result['accepted_family'] == -1


def test_batched_rank_matches_single_frames_and_does_not_mutate_arrays():
    features = scalar_features()
    batch = {key: value if key == 'family_index' else np.stack([value, value])
             for key, value in features.items()}
    before = deepcopy(batch)
    batched, single = rank_hypotheses(batch), rank_hypotheses(features)
    for mode in MODES:
        for key in single[mode]:
            np.testing.assert_array_equal(batched[mode][key][0], single[mode][key])
    for key in batch:
        np.testing.assert_array_equal(batch[key], before[key])


def test_tampered_scalar_velocity_norm_is_rejected():
    features = scalar_features();features['velocity_residual_norm_m_s'][0] = .5
    with pytest.raises(ValueError, match='disagrees'):
        rank_hypotheses(features)


def test_pure_rank_module_imports_and_runs_when_mujoco_import_is_forbidden():
    source = Path(__file__).resolve().parents[1]/'src/spherical_wheel/contact_hypotheses.py'
    script = '''import builtins, importlib.util, numpy as np, sys
original = builtins.__import__
def guarded(name, *args, **kwargs):
 if name == 'mujoco' or name.startswith('mujoco.'):
  raise ImportError('MuJoCo intentionally unavailable')
 return original(name, *args, **kwargs)
builtins.__import__ = guarded
spec=importlib.util.spec_from_file_location('portable_contact_rank',sys.argv[1])
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
features=dict(family_index=np.array([0,1]),valid=np.ones(2,bool),frame_valid=True,excitation_m_s=.1,
 normal_angle_rad=np.zeros(2),moment_residual_norm=np.zeros(2),
 velocity_residual_m_s=np.array([[0.,0.,0.],[.04,0.,0.]]),velocity_residual_norm_m_s=np.array([0.,.04]))
assert module.rank_hypotheses(features)['velocity']['accepted_family']==0
'''
    subprocess.run([sys.executable, '-c', script, str(source)], check=True)
