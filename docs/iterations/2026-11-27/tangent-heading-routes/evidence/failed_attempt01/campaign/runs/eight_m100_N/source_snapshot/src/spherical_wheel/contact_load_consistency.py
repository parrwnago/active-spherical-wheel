"""Opt-in selected-contact load diagnostics; no live control or new ranking.

The observer vector has world-frame force/moment in rows 0:6 and joint scalars
in rows 6:9. A MuJoCo point Jacobian uses body-frame free-base angular rates,
so its transpose-force product must rotate angular rows before comparison.
The single-point model assigns the whole observed force to the selected body;
this is a testable hypothesis, not a multi-contact force decomposition.
"""
from __future__ import annotations

import numpy as np


class ContactLoadFeatures:
    """Private kinematics for a frozen selected point and its material family."""

    def __init__(self, model, *, geometry_module=None, centre_body='fork', minimum_force_N=1.):
        import mujoco
        if geometry_module is None:
            from . import contact_hypotheses as geometry_module
        self._mj, self.model, self.geometry_module = mujoco, model, geometry_module
        self.geometry = geometry_module.ContactHypothesisFeatures(model, centre_body,
            minimum_force_N=minimum_force_N)
        if (model.njnt == 0 or model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE
                or model.jnt_dofadr[0] != 0 or model.jnt_bodyid[0] != self.geometry.centre_body):
            raise ValueError('Mixed-frame load conversion requires the fork to carry the first free joint')
        self.data = self.geometry.data
        self.radius_m, self.minimum_force_N = self.geometry.radius, float(minimum_force_N)

    def _set_pose(self, qpos):
        qpos = np.asarray(qpos, float)
        if qpos.shape != (self.model.nq,):
            raise ValueError('Pose dimension differs from the known model')
        if not np.isfinite(qpos).all() or np.linalg.norm(qpos[3:7]) < 1e-12:
            return False
        self.data.qpos[:] = qpos
        self.data.qpos[3:7] /= np.linalg.norm(self.data.qpos[3:7])
        self._mj.mj_kinematics(self.model, self.data)
        self._mj.mj_comPos(self.model, self.data)
        return True

    def reconstruct_point(self, qpos, normal_hint, original_candidate_index):
        """Reconstruct one ORIGINAL I-60 point; no velocity/gain reevaluation."""
        index = int(original_candidate_index)
        if index < 0:
            return np.full(3, np.nan)
        if index >= self.geometry.candidate_count:
            raise ValueError('Candidate index is outside the frozen I-60 bank')
        if not self._set_pose(qpos):
            return np.full(3, np.nan)
        if np.asarray(normal_hint).shape != (3,):
            raise ValueError('Normal hint must have three components')
        core = self.geometry_module;normal = core._unit(normal_hint)
        if normal is None:
            return np.full(3, np.nan)
        seed, kind = divmod(index, 6)
        label = int(self.geometry.family_index[index]);patch = self.geometry.patches[label]
        normal_seed = core.normal_seeds(normal, self.geometry.cone_rad)[seed]
        axis = patch['sign']*self.data.xmat[patch['body']].reshape(3, 3)[:, 0]
        upper = patch['lower'] if kind >= 4 else patch['upper']
        offset = core._band_projection(-self.radius_m*normal_seed, axis,
                                      self.radius_m, patch['lower'], upper)
        return self.data.xpos[self.geometry.centre_body].copy()+offset

    def evaluate(self, qpos, point_m, family_index, observer_wrench, observer_generalized):
        """Predict J_point.T F, rotate base moment rows, retain all residuals.

        No qdot, desired motion, commanded torque, ground or solved contact is
        needed. The selected point/family must already belong to the frozen I-60
        bank. The compatibility gate below uses only the extra joint rows.
        """
        point = np.asarray(point_m, float)
        wrench, generalized = np.asarray(observer_wrench, float), np.asarray(observer_generalized, float)
        if point.shape != (3,) or wrench.shape != (6,) or generalized.shape != (9,):
            raise ValueError('Expected point[3], wrench[6] and generalized force[9]')
        result = dict(valid=False, prediction=np.full(9, np.nan), residual=np.full(9, np.nan),
            extra_joint_residual_Nm=np.full(3, np.nan), force_norm_N=float(np.linalg.norm(wrench[:3])),
            radius_m=self.radius_m, point_geometry_valid=False, wrench_matches_generalized=False)
        label = int(family_index)
        if not 0 <= label < 4 or not all(np.isfinite(x).all() for x in (point, wrench, generalized)):
            return result
        if (not np.isfinite(result['force_norm_N']) or result['force_norm_N'] < self.minimum_force_N
                or not self._set_pose(qpos)):
            return result
        result['wrench_matches_generalized'] = bool(np.array_equal(wrench, generalized[:6]))
        if not result['wrench_matches_generalized']:
            return result
        patch = self.geometry.patches[label]
        centre = self.data.xpos[self.geometry.centre_body]
        offset = point-centre
        axis = patch['sign']*self.data.xmat[patch['body']].reshape(3, 3)[:, 0]
        axial = float(axis@offset)
        result['point_geometry_valid'] = bool(abs(np.linalg.norm(offset)-self.radius_m) <= 1e-9
            and patch['lower']-1e-9 <= axial <= patch['upper']+1e-9)
        if not result['point_geometry_valid']:
            return result
        jacobian = np.zeros((3, self.model.nv))
        self._mj.mj_jac(self.model, self.data, jacobian, None, point, patch['body'])
        prediction = jacobian.T@wrench[:3]
        rotation = self.data.xmat[self.geometry.centre_body].reshape(3, 3)
        prediction[3:6] = rotation@prediction[3:6]
        residual = generalized-prediction
        result.update(valid=True, prediction=prediction, residual=residual,
                      extra_joint_residual_Nm=residual[6:9].copy())
        return result

    def joint_maps(self, qpos, observer_wrench):
        """Truth-blind affine point-to-joint-load maps for later evaluation.

        For any separately supplied evaluation point p on family k, joint load
        equals ``joint_at_centre[k] + joint_moment_map[k] @ cross(p-centre,F)``.
        No oracle point or label is accepted by this feature-generation API.
        """
        wrench = np.asarray(observer_wrench, float)
        if wrench.shape != (6,):
            raise ValueError('Expected a six-component observer wrench')
        result = dict(valid=False, centre_m=np.full(3, np.nan),
            joint_at_centre=np.full((4, 3), np.nan),
            joint_moment_map=np.full((4, 3, 3), np.nan),
            force_norm_N=float(np.linalg.norm(wrench[:3])), radius_m=self.radius_m)
        if (not np.isfinite(wrench).all() or not np.isfinite(result['force_norm_N'])
                or result['force_norm_N'] < self.minimum_force_N
                or not self._set_pose(qpos)):
            return result
        centre = self.data.xpos[self.geometry.centre_body].copy()
        base, maps = np.empty((4, 3)), np.empty((4, 3, 3))
        cache = {}
        for family, patch in enumerate(self.geometry.patches):
            body = patch['body']
            if body not in cache:
                linear, angular = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
                self._mj.mj_jac(self.model, self.data, linear, angular, centre, body)
                cache[body] = (linear[:, 6:9].T@wrench[:3], angular[:, 6:9].T.copy())
            base[family], maps[family] = cache[body]
        result.update(valid=True, centre_m=centre, joint_at_centre=base, joint_moment_map=maps)
        return result


def load_compatibility(features, *, absolute_tolerance_Nm=.03, relative_tolerance=.10):
    """Pure NumPy veto using ONLY joint residual rows 6:9, with fixed scales.

    Tolerance per joint is abs + rel*R*norm(F), in N m. Invalid features are
    unknown, not a positive compatibility result. These are engineering bounds,
    not calibrated likelihoods. Passing this gate does not prove the hypothesis.
    """
    if (not np.isfinite([absolute_tolerance_Nm, relative_tolerance]).all()
            or absolute_tolerance_Nm <= 0 or relative_tolerance < 0):
        raise ValueError('Positive absolute and nonnegative relative tolerances required')
    residual = np.asarray(features['residual'], float)
    if residual.ndim < 1 or residual.shape[-1] != 9:
        raise ValueError('Expected a nine-component generalized residual')
    shape = residual.shape[:-1]
    force = np.broadcast_to(np.asarray(features['force_norm_N'], float), shape)
    radius = np.broadcast_to(np.asarray(features['radius_m'], float), shape)
    valid = np.broadcast_to(np.asarray(features['valid'], bool), shape).copy()
    valid &= np.isfinite(residual).all(axis=-1)&np.isfinite(force)&np.isfinite(radius)&(force >= 0)&(radius > 0)
    threshold = absolute_tolerance_Nm+relative_tolerance*radius*force
    joints = residual[..., 6:9].copy()
    ratio = joints/threshold[..., None]
    maximum = np.max(abs(ratio), axis=-1)
    passes = valid&(maximum <= 1.)
    return dict(valid=valid, threshold_Nm=threshold, extra_joint_residual_Nm=joints,
        normalized_joint_residual=ratio, max_normalized_joint_residual=maximum,
        passes=passes, vetoed=valid&~passes, unknown=~valid)
