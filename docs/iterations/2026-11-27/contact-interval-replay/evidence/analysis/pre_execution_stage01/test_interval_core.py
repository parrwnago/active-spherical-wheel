"""Synthetic contracts only: no experiment recordings loaded."""
import importlib.util
from pathlib import Path
import unittest
import sys

import numpy as np

_SPEC = importlib.util.spec_from_file_location('interval_core', Path(__file__).with_name('interval_core.py'))
c = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(c)
sys.path.insert(0, str(Path(__file__).parent))
from analyse_interval import analyse


def qpose(phase=0., quat=(1., 0., 0., 0.)):
    return np.r_[.1, -.2, .3, quat, phase, .4, -.6]


def row(tag=0, position=(.1, -.2, .2), force=(1., 0., 10.), normal=(0., 0., 1.), admitted=1.):
    return np.r_[tag, position, normal, force, .001, .7, admitted, 0., 0., 0.]


class CoreContracts(unittest.TestCase):
    def test_joint_ancestry_and_force_moment_sign(self):
        rows = np.array([row(t) for t in range(7)])
        loads, valid = c.point_force_generalized(qpose(), rows[:, 1:4], rows[:, 7:10], rows[:, 0])
        self.assertTrue(valid.all())
        np.testing.assert_allclose(loads[:, :3], np.tile([1, 0, 10], (7, 1)))
        np.testing.assert_allclose(loads[:, 3:6], np.tile([0, -.1, 0], (7, 1)))
        np.testing.assert_allclose(loads[:, 6:], c.ANCESTRY*np.array([-.1, 0, 0]))

    def test_virtual_work_identity(self):
        q, v, p, f = qpose(.7, (.9, .2, -.1, .3)), np.arange(9)*.17, np.array([[.4, .1, -.2]]), np.array([[2., -3., 5.]])
        for tag in range(7):
            load, valid = c.point_force_generalized(q, p, f, np.array([tag]))
            velocity, _ = c.point_velocity(q, v, p, np.array([tag]))
            self.assertTrue(valid[0])
            self.assertAlmostEqual(float(load[0] @ v), float(velocity[0] @ f[0]), places=13)

    def test_native_average_then_mid_rotation_is_not_world_average(self):
        q0, q1 = qpose(), qpose(quat=(np.sqrt(.5), 0., 0., np.sqrt(.5)))
        loads = np.zeros((2, 9)); loads[:, 3] = 1.
        mean = c.reduce_interval(loads, [.001, .003], q0, q1)
        np.testing.assert_allclose(mean[3:6], [np.sqrt(.5), np.sqrt(.5), 0], atol=1e-15)
        np.testing.assert_allclose(c.middle_rotation(q0, q1), c.middle_rotation(q0, np.r_[q1[:3], -q1[3:7], q1[7:]]), atol=1e-15)

    def test_nonuniform_steps_are_duration_weighted(self):
        x = np.zeros((2, 9)); x[:, 0] = [2, 10]
        self.assertEqual(c.reduce_interval(x, [.001, .003], qpose(), qpose())[0], 8.)
        with self.assertRaises(ValueError):
            c.reduce_interval(x, [0, .001], qpose(), qpose())

    def test_rejected_force_is_retained_and_reported(self):
        out = c.contact_frame(qpose(), np.zeros(9), np.array([row(admitted=0)]), np.zeros(5))
        np.testing.assert_equal(out['generalized'], out['rejected_generalized'])
        self.assertEqual(out['rejected_row_count'], 1)

    def test_opposing_slips_do_not_cancel_in_rms(self):
        q, v = qpose(), np.zeros(9); v[3:6] = [0, 0, 1]
        rows = np.array([row(position=(.2, -.2, .3)), row(position=(0., -.2, .3))])
        out = c.contact_frame(q, v, rows, np.zeros(5))
        self.assertAlmostEqual(out['slip_mean_norm_m_s'], 0., places=15)
        self.assertAlmostEqual(out['slip_rms_m_s'], .1, places=15)
        self.assertAlmostEqual(out['slip_max_m_s'], .1, places=15)

    def test_belt_speed_keeps_tangent_projection_magnitude(self):
        n = np.array([0., np.sqrt(.5), np.sqrt(.5)])
        out = c.contact_frame(qpose(), np.zeros(9), np.array([row(tag=2, normal=n)]), np.r_[0, 0, 0, .2, 0])
        self.assertAlmostEqual(out['slip_rms_m_s'], .2*np.sqrt(.5), places=15)

    def test_zero_load_and_unknown_tags_are_not_zero_slip(self):
        out = c.contact_frame(qpose(), np.zeros(9), np.empty((0, 16)), np.zeros(5))
        self.assertTrue(out['force_map_valid']); self.assertFalse(out['slip_valid'])
        self.assertTrue(np.isnan(out['slip_rms_m_s']))
        out = c.contact_frame(qpose(), np.zeros(9), np.array([row(tag=9)]), np.zeros(5))
        self.assertFalse(out['force_map_valid']); self.assertFalse(out['slip_valid'])
        self.assertTrue(np.isnan(out['generalized']).all())

    def test_decomposition_keeps_cancellation(self):
        d = c.decomposition([[1., 0., 0.]], [[3., 0., 0.]], [[0., 0., 0.]])
        np.testing.assert_equal(d['identity_error'], 0)
        self.assertEqual(d['norm_squared_cross_term'][0], -12.)

    def test_intrinsic_normal_and_force_parallel_null(self):
        np.testing.assert_allclose(c.intrinsic_normal([0, 0, 10], [0, 0, 0], .12), [0, 0, 1])
        q, f, p = qpose(.3), np.array([[1., 2., 3.]]), np.array([[.5, .2, -.1]])
        a, _ = c.point_force_generalized(q, p, f, [0])
        b, _ = c.point_force_generalized(q, p+.4*f, f, [0])
        np.testing.assert_allclose(a, b, atol=2e-15)

    def test_mu_jacobian_synthetic_topology(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest('MuJoCo unavailable; pure NumPy contracts still run')
        xml = '''<mujoco><worldbody><body name="fork"><freejoint/>
        <geom type="sphere" size=".03" mass="1"/>
        <body name="carrier"><joint axis="0 1 0"/><geom size=".02" mass="1"/>
        <body name="A"><joint axis="1 0 0"/><geom size=".01" mass="1"/></body>
        <body name="B"><joint axis="1 0 0"/><geom size=".01" mass="1"/></body>
        </body></body></worldbody></mujoco>'''
        model = mujoco.MjModel.from_xml_string(xml); data = mujoco.MjData(model)
        data.qpos[:] = qpose(.7, (.9, .2, -.1, .3))
        data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7]); mujoco.mj_forward(model, data)
        p, f = np.array([[.4, .1, -.2]]), np.array([[2., -3., 5.]])
        jac = np.zeros((3, 9))
        for tag, body in enumerate(('A', 'B', 'carrier', 'carrier', 'A', 'B', 'fork')):
            mujoco.mj_jac(model, data, jac, None, p[0], mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body))
            load, _ = c.point_force_generalized(data.qpos, p, f, [tag])
            np.testing.assert_allclose(load[0], jac.T @ f[0], atol=2e-15)

    def test_wrapper_interval_alignment_and_full_denominators(self):
        trace, precision = wrapper_fixture()
        a, _, report = analyse(trace, precision)
        self.assertEqual(report['intervals'], 1)
        self.assertTrue(a['valid'][0])
        np.testing.assert_allclose(a['interval_before'][0], [0, 0, 1, 0, 0, 0, 0, 0, 0])
        self.assertEqual(report['groups']['full_primary_time_1_to_41']['total_rows'], 0)
        self.assertTrue(a['endpoint_truth_eligible'][0])

    def test_wrapper_rejects_wrong_command_and_missing_substep(self):
        trace, precision = wrapper_fixture()
        precision['exact_previous_applied'][1, 0] = .1
        with self.assertRaisesRegex(ValueError, 'held command'):
            analyse(trace, precision)
        trace, precision = wrapper_fixture(); trace['substep_index'][2] = 9
        with self.assertRaisesRegex(ValueError, 'exactly five'):
            analyse(trace, precision)


def wrapper_fixture():
    q = qpose(); r = row(force=(0, 0, 1))
    trace = dict(time_s=np.arange(1, 6)*.001, step_dt_s=np.full(5, .001),
                 bridge_time_before_s=np.arange(5)*.001, bridge_time_after_s=np.arange(1, 6)*.001,
                 control_start_time_s=np.zeros(5), control_end_time_s=np.full(5, .005),
                 control_end_index=np.ones(5, int), substep_index=np.arange(1, 6),
                 hold_completed=np.ones(5, bool), held_input=np.zeros((5, 5)), disturbance_world=np.zeros((5, 6)),
                 qpos_before=np.tile(q, (5, 1)), qpos_after=np.tile(q, (5, 1)),
                 qvel_before=np.zeros((5, 9)), qvel_after=np.zeros((5, 9)),
                 contact_rows=np.tile(r, (5, 1)), contact_offsets=np.arange(6),
                 tag_mirror_body_names=np.array(['shell_A', 'shell_B', 'carrier', 'carrier', 'shell_A', 'shell_B', 'fork']))
    mu = np.zeros((2, 9)); mu[:, 2] = 1
    precision = dict(time_s=np.array([0., .005]), exact_observer_interval_mean=mu,
                     exact_observer_generalized=mu.copy(), exact_previous_applied=np.zeros((2, 5)),
                     exact_qpos=np.tile(q, (2, 1)), exact_qvel=np.zeros((2, 9)),
                     exact_estimated_normal=np.tile([0., 0., 1.], (2, 1)),
                     exact_estimated_support_loads_N=np.tile([1., 0, 0, 0], (2, 1)),
                     exact_estimated_contact_point_m=np.tile([.1, -.2, .2], (2, 1)))
    return trace, precision


if __name__ == '__main__':
    unittest.main()
