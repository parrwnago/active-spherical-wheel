"""Synthetic integration tests, never experiment/truth logs."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from input_features import frame_features, rotation, interval_rotation
from prepare import primitives_from, MEASUREMENT_KEYS, POSE_KEYS, FEATURE_KEYS


def fixture(phase=0.):
    import mujoco
    from spherical_wheel.contact_hypotheses import ContactHypothesisFeatures
    radius, seam = .12, .11565465836
    xml = f'''<mujoco><size nuser_geom="5"/><worldbody><body name="fork"><freejoint/>
      <geom size=".03" mass="1"/>
      <body name="carrier"><joint axis="0 1 0"/>
      <geom name="belt_A_smooth" size=".12" mass=".1" user="1 {seam} .12 4 .12"/>
      <geom name="belt_B_smooth" size=".12" mass=".1" user="-1 {seam} .12 4 .12"/>
      <body name="shell_A"><joint axis="1 0 0"/>
      <geom name="shell_A_smooth" size=".12" mass=".1" user="1 .01 {seam} 2 .12"/></body>
      <body name="shell_B"><joint axis="1 0 0"/>
      <geom name="shell_B_smooth" size=".12" mass=".1" user="-1 .01 {seam} 2 .12"/></body>
      </body></body></worldbody></mujoco>'''
    model = mujoco.MjModel.from_xml_string(xml); data = mujoco.MjData(model)
    q = np.r_[.1, -.2, .3, 1., 0., 0., 0., phase, .4, -.6]
    v = np.array([.03, -.02, .01, .05, -.07, .02, .4, -.6, .3])
    prior = np.array([.1, -.1, .1, .08, -.09]); hint = np.array([0., 0., 1.])
    filtered = np.array([0., 0., 20., .02, -.01, 0., 0., 0., 0.])
    old = ContactHypothesisFeatures(model).evaluate(q, v, prior, hint, filtered[:6])
    data.qpos[:] = q; data.qvel[:] = v; mujoco.mj_forward(model, data)
    axes = []
    for name, sign in (('shell_A', 1), ('shell_B', -1), ('carrier', 1), ('carrier', -1)):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        axes.append(sign*data.xmat[body].reshape(3, 3)[:, 0])
    p = dict(time_s=np.array([0., .005]), qpos=np.tile(q, (2, 1)), qvel=np.tile(v, (2, 1)),
        previous_applied=np.tile(prior, (2, 1)), observer_interval_mean=np.tile(filtered, (2, 1)),
        observer_generalized=np.tile(filtered, (2, 1)), intrinsic_normal_hint=np.tile(hint, (2, 1)),
        centre_m=np.tile(q[:3], (2, 1)), patch_axis_world=np.tile(axes, (2, 1, 1)),
        pose_valid=np.ones(2, bool), radius_m=np.array(radius),
        patch_lower_m=np.array([.01, .01, seam, seam]), patch_upper_m=np.array([seam, seam, radius, radius]),
        original_valid=np.tile(old['valid'], (2, 1)), original_frame_valid=np.ones(2, bool),
        original_excitation_m_s=np.ones(2))
    return p, old, model, data


class InputContracts(unittest.TestCase):
    def test_original_geometry_valid_points_normals_and_rolling_vectors(self):
        for phase in (0., 1.15, -1.15):
            p, old, _, _ = fixture(phase); new = frame_features(p, 1)
            self.assertTrue(new['interval_valid'])
            np.testing.assert_equal(new['valid'], old['valid'])
            active = old['valid']; self.assertTrue(active.any())
            np.testing.assert_allclose(new['points_m'][active], old['point_m'][active], atol=1e-15, rtol=0)
            np.testing.assert_allclose(new['force_normals'][active], old['normal'][active], atol=2e-15, rtol=0)
            np.testing.assert_allclose(new['tangent_velocity_m_s'][active], old['velocity_residual_m_s'][active], atol=2e-15, rtol=0)

    def test_force_maps_match_private_mujoco_with_observer_mid_frame(self):
        import mujoco
        p, _, model, data = fixture(1.15)
        p['qpos'][0, 3:7] = [np.cos(.2), 0, 0, np.sin(.2)]
        new = frame_features(p, 1); middle, _ = interval_rotation(p, 1)
        jac = np.zeros((3, 9)); force = np.array([2., -3., 5.])
        for i in np.flatnonzero(new['valid']):
            body = ('shell_A', 'shell_B', 'carrier', 'carrier')[new['family_index'][i]]
            mujoco.mj_jac(model, data, jac, None, new['points_m'][i], mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body))
            expected = jac.T @ force; expected[3:6] = middle @ expected[3:6]
            np.testing.assert_allclose(new['force_maps'][i] @ force, expected, atol=2e-15, rtol=0)

    def test_first_frame_and_time_gap_do_not_fabricate_interval(self):
        p, _, _, _ = fixture()
        self.assertFalse(frame_features(p, 0)['valid'].any())
        p['time_s'][1] = .010
        self.assertFalse(frame_features(p, 1)['interval_valid'])
        self.assertFalse(frame_features(p, 1)['valid'].any())

    def test_future_states_cannot_affect_current_features(self):
        p, _, _, _ = fixture(); before = frame_features(p, 1)
        extended = {}
        for key, value in p.items():
            extended[key] = np.concatenate([value, value[-1:]], axis=0) if np.ndim(value) and value.shape[0] == 2 else value.copy()
        extended['qpos'][2] += .3; extended['previous_applied'][2] += 9.
        after = frame_features(extended, 1)
        for key in ('points_m', 'force_maps', 'material_velocity_m_s', 'valid'):
            np.testing.assert_equal(before[key], after[key])

    def test_truth_and_native_labels_are_not_serialized(self):
        p, _, _, _ = fixture()
        measurement = {src: p[dst] for dst, src in MEASUREMENT_KEYS.items()}
        pose = {src: p[dst] for dst, src in POSE_KEYS.items()}; pose['time_s'] = p['time_s']
        projected = {src: p[dst] for dst, src in FEATURE_KEYS.items()}
        projected['family_index'] = np.tile([0, 1, 2, 3, 0, 1], 17)
        measurement.update(truth_family=np.ones(2), exact_estimated_support_loads_N=np.ones((2, 4)), terrain=np.ones(2))
        result = primitives_from(measurement, pose, projected)
        self.assertEqual(set(result), set(MEASUREMENT_KEYS) | set(POSE_KEYS) | set(FEATURE_KEYS))
        self.assertFalse(any('truth' in key or 'estimated' in key or 'terrain' in key for key in result))

    def test_cached_features_and_original_invalid_mask_are_retained(self):
        p, _, _, _ = fixture(); frame = frame_features(p, 1)
        cached = {key: np.stack([frame[key], frame[key]]) for key in
                   ('points_m', 'force_normals', 'material_velocity_m_s', 'tangent_velocity_m_s', 'normal_angle_rad')}
        via_cache = frame_features(p, 1, cached)
        for key in ('points_m', 'force_maps', 'material_velocity_m_s', 'valid'):
            np.testing.assert_equal(frame[key], via_cache[key])
        p['original_valid'][1] = False
        self.assertFalse(frame_features(p, 1, cached)['valid'].any())


if __name__ == '__main__': unittest.main()
