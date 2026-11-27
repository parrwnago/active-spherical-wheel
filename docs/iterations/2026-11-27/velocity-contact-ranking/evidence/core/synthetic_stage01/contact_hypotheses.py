"""Offline contact hypotheses from measured motion, geometry and observer wrench.

This opt-in analysis module never reads solver contacts, terrain or references.
It does not run dynamics or alter a live controller. ``rank_hypotheses`` and
``project_features`` need only NumPy; MuJoCo is imported lazily by the private
kinematics feature generator. Scores and margins are uncalibrated, not posterior
probabilities. Single-contact candidates cannot identify arbitrary multi-contact.
"""
from __future__ import annotations

import math
import numpy as np

FAMILIES = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
MODES = ('geometry', 'wrench', 'velocity', 'fused')
DEFAULT_CONE_RAD = math.radians(10.)


def _unit(value):
    value = np.asarray(value, float)
    size = float(np.linalg.norm(value))
    return value/size if np.isfinite(size) and size > 1e-12 else None


def _tangent_basis(normal):
    axis = np.eye(3)[int(np.argmin(abs(normal)))]
    first = np.cross(normal, axis)
    first /= np.linalg.norm(first)
    return first, np.cross(normal, first)


def normal_seeds(normal, cone_rad=DEFAULT_CONE_RAD):
    """Centre, half-cone and cone rings at eight fixed tangent directions."""
    normal = _unit(normal)
    if normal is None:
        raise ValueError('A finite nonzero normal is required')
    if not np.isfinite(cone_rad) or not 0 < cone_rad < .5*np.pi:
        raise ValueError('Normal cone must lie between zero and pi/2')
    first, second = _tangent_basis(normal)
    seeds = [normal]
    for angle in (.5*cone_rad, cone_rad):
        for azimuth in np.arange(8)*(.25*np.pi):
            tangent = np.cos(azimuth)*first+np.sin(azimuth)*second
            seeds.append(np.cos(angle)*normal+np.sin(angle)*tangent)
    return np.asarray(seeds)


def _band_projection(offset, axis, radius, lower, upper):
    """Nearest spherical-band point, preserving azimuth about its signed axis."""
    axial = float(np.clip(axis@offset, lower, upper))
    radial = offset-axis*float(axis@offset)
    size = float(np.linalg.norm(radial))
    if size <= 1e-12:
        radial = _tangent_basis(axis)[0]
    else:
        radial /= size
    return axial*axis+math.sqrt(max(radius*radius-axial*axial, 0.))*radial


class ContactHypothesisFeatures:
    """Private kinematics for 17 normal seeds × six surface/rim kinds.

    Four family labels are used. The two exposed equatorial rim kinds retain
    their shell's family, so duplicate candidates cannot manufacture a family
    runner-up. Rim points lie on known rim geometry; their candidate radial
    normal remains a local single-contact model assumption.
    """

    def __init__(self, model, centre_body='fork', *, cone_rad=DEFAULT_CONE_RAD,
                 minimum_force_N=1.):
        import mujoco
        if not np.isfinite(minimum_force_N) or minimum_force_N <= 0:
            raise ValueError('Minimum force must be positive and finite')
        normal_seeds([0., 0., 1.], cone_rad)
        if model.nuser_geom < 5 or model.nv != 9:
            raise ValueError('Expected the nine-velocity wheel and five patch metadata fields')
        self._mj, self.model = mujoco, model
        self.data = mujoco.MjData(model)
        self.cone_rad, self.minimum_force_N = float(cone_rad), float(minimum_force_N)
        self.centre_body = (int(centre_body) if isinstance(centre_body, (int, np.integer))
                            else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, centre_body))
        if not 0 <= self.centre_body < model.nbody:
            raise ValueError('Unknown wheel-centre body')
        self.patches = []
        for index, name in enumerate(FAMILIES):
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name+'_smooth')
            if geom < 0:
                raise ValueError('Missing spherical patch '+name)
            sign, lower, upper, _, radius = map(float, model.geom_user[geom, :5])
            if (sign not in (-1., 1.) or not 0 <= lower < upper <= radius
                    or not np.isfinite([sign, lower, upper, radius]).all()):
                raise ValueError('Invalid spherical-band metadata for '+name)
            self.patches.append(dict(name=name, family=index, geom=int(geom),
                body=int(model.geom_bodyid[geom]), sign=sign, lower=lower,
                upper=upper, radius=radius))
        self.radius = self.patches[0]['radius']
        if not all(np.isclose(p['radius'], self.radius, atol=1e-12, rtol=0.) for p in self.patches):
            raise ValueError('All contact patches must share one sphere radius')
        # Seed-major layout: the four surface patches, then the two shell rims.
        self.family_index = np.tile(np.array([0, 1, 2, 3, 0, 1], np.int8), 17)
        self.kind = np.tile(np.array([0, 0, 0, 0, 1, 1], np.int8), 17)
        self.candidate_count = len(self.family_index)

    def _empty(self):
        count = self.candidate_count
        return dict(family_index=self.family_index.copy(), kind=self.kind.copy(),
            valid=np.zeros(count, bool), frame_valid=False, excitation_m_s=0.,
            normal=np.full((count, 3), np.nan), point_m=np.full((count, 3), np.nan),
            velocity_residual_m_s=np.full((count, 3), np.nan),
            moment_residual_normalized=np.full((count, 3), np.nan),
            normal_angle_rad=np.full(count, np.nan), force_normal_N=np.full(count, np.nan))

    def evaluate(self, qpos, qvel, previous_applied, normal_hint, observer_wrench):
        """Evaluate one timestamp; no desired speed or next command is accepted.

        ``normal_hint`` is supplied inference, normally the intrinsic normal of
        the observed wrench. ``previous_applied[3:5]`` are achieved surface
        speeds only under the existing ideal prescribed-belt simulation model.
        Joint torques are never interpreted as achieved joint rates.
        """
        result = self._empty()
        qpos, qvel, previous_applied, observer_wrench = (
            np.asarray(x, float) for x in (qpos, qvel, previous_applied, observer_wrench))
        if (qpos.shape != (self.model.nq,) or qvel.shape != (self.model.nv,)
                or previous_applied.shape != (5,) or observer_wrench.shape != (6,)):
            raise ValueError('Incorrect state, prior-input or observer-wrench shape')
        if np.asarray(normal_hint).shape != (3,):
            raise ValueError('Normal hint must have three components')
        normal = _unit(normal_hint)
        finite = all(np.isfinite(x).all() for x in (qpos, qvel, previous_applied, observer_wrench))
        if not finite or normal is None or np.linalg.norm(qpos[3:7]) < 1e-12:
            return result
        force, moment = observer_wrench[:3], observer_wrench[3:]
        force_size = float(np.linalg.norm(force))
        if force_size < self.minimum_force_N or float(force@normal) < self.minimum_force_N:
            return result
        mj, model, data = self._mj, self.model, self.data
        data.qpos[:] = qpos
        data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
        data.qvel[:] = qvel
        mj.mj_kinematics(model, data)
        mj.mj_comPos(model, data)
        centre = data.xpos[self.centre_body].copy()
        # The velocity at every candidate point follows from one reference-point
        # Jacobian per rigid body. No per-candidate dynamics/contact solve occurs.
        motions = {}
        for patch in self.patches:
            body = patch['body']
            if body not in motions:
                linear, angular = np.zeros((3, model.nv)), np.zeros((3, model.nv))
                mj.mj_jac(model, data, linear, angular, centre, body)
                motions[body] = (linear@qvel, angular@qvel)
        kinds = self.patches+[self.patches[0], self.patches[1]]
        for seed_index, seed in enumerate(normal_seeds(normal, self.cone_rad)):
            for kind_index, patch in enumerate(kinds):
                index = 6*seed_index+kind_index
                axis = patch['sign']*data.xmat[patch['body']].reshape(3, 3)[:, 0]
                lower = patch['lower']
                upper = lower if kind_index >= 4 else patch['upper']
                offset = _band_projection(-self.radius*seed, axis, self.radius, lower, upper)
                candidate_normal = -offset/self.radius
                angle = float(np.arctan2(np.linalg.norm(np.cross(normal, candidate_normal)),
                                        np.clip(normal@candidate_normal, -1., 1.)))
                normal_force = float(force@candidate_normal)
                axial = float(axis@offset)
                geometry_valid = lower-1e-12 <= axial <= upper+1e-12
                valid = geometry_valid and angle <= self.cone_rad+1e-12 and normal_force >= self.minimum_force_N
                if not valid:
                    continue
                linear, angular = motions[patch['body']]
                material_velocity = linear+np.cross(angular, offset)
                if patch['family'] >= 2:
                    belt_axis = data.geom_xmat[patch['geom']].reshape(3, 3)[:, 1]
                    material_velocity = material_velocity+belt_axis*previous_applied[patch['family']+1]
                residual = material_velocity-candidate_normal*float(candidate_normal@material_velocity)
                result['valid'][index] = True
                result['normal'][index] = candidate_normal
                result['point_m'][index] = centre+offset
                result['velocity_residual_m_s'][index] = residual
                result['moment_residual_normalized'][index] = (moment-np.cross(offset, force))/(self.radius*force_size)
                result['normal_angle_rad'][index] = angle
                result['force_normal_N'][index] = normal_force
        result['frame_valid'] = True
        result['excitation_m_s'] = float(np.linalg.norm(qvel[:3])
            +self.radius*np.linalg.norm(qvel[3:6])+self.radius*np.linalg.norm(qvel[6:9])
            +np.linalg.norm(previous_applied[3:]))
        return result


def project_features(features):
    """Drop points/normals and retain exactly the NumPy ranker's inputs."""
    valid = np.asarray(features['valid'], bool)
    velocity = np.where(valid[..., None], np.asarray(features['velocity_residual_m_s'], float), 0.)
    angle = np.where(valid, np.asarray(features['normal_angle_rad'], float), 0.)
    moment = np.where(valid, np.linalg.norm(features['moment_residual_normalized'], axis=-1), 0.)
    return dict(family_index=np.array(features['family_index'], copy=True),
        valid=valid.copy(),
        frame_valid=np.array(features['frame_valid'], copy=True),
        excitation_m_s=np.array(features['excitation_m_s'], copy=True),
        normal_angle_rad=angle,
        velocity_residual_m_s=velocity.copy(),
        velocity_residual_norm_m_s=np.linalg.norm(velocity, axis=-1),
        moment_residual_norm=moment)


def rank_hypotheses(features, *, velocity_scale_m_s=.02,
                    normal_scale_rad=DEFAULT_CONE_RAD, minimum_excitation_m_s=.02,
                    minimum_margin=.25, maximum_residual_m_s=.02,
                    minimum_separation_m_s=.01):
    """Rank families with shared geometry prior; abstain under fixed gates.

    Supports one frame or leading batch dimensions. Candidate vectors are only
    needed for best/runner-up residual contrast, a limited identifiability check.
    Per-family minima use each candidate's total score, not separately minimized
    terms. ``chosen_family`` is descriptive; ``accepted_family`` is -1 unless
    all declared gates pass. Neither field is a calibrated probability.
    """
    settings = [velocity_scale_m_s, normal_scale_rad, minimum_excitation_m_s,
                minimum_margin, maximum_residual_m_s, minimum_separation_m_s]
    if not all(np.isfinite(v) and v > 0 for v in settings):
        raise ValueError('All rank scales and gates must be positive and finite')
    raw = project_features(features) if 'moment_residual_norm' not in features else features
    valid = np.asarray(raw['valid'], bool)
    if valid.ndim < 1:
        raise ValueError('Expected at least a candidate dimension')
    shape, count = valid.shape[:-1], valid.shape[-1]
    if count == 0:
        raise ValueError('At least one candidate slot is required')
    family = np.asarray(raw['family_index'], int)
    if family.shape != (count,) or not np.isin(family, np.arange(4)).all():
        raise ValueError('Candidate families must be a fixed vector of four known labels')
    angle, speed, wrench = (np.asarray(raw[k], float) for k in
        ('normal_angle_rad', 'velocity_residual_norm_m_s', 'moment_residual_norm'))
    vectors = np.asarray(raw['velocity_residual_m_s'], float)
    if any(x.shape != valid.shape for x in (angle, speed, wrench)) or vectors.shape != valid.shape+(3,):
        raise ValueError('Candidate feature shapes disagree')
    frame_valid = np.broadcast_to(np.asarray(raw['frame_valid'], bool), shape)
    excitation = np.broadcast_to(np.asarray(raw['excitation_m_s'], float), shape)
    finite = np.isfinite(angle)&np.isfinite(speed)&np.isfinite(wrench)&np.isfinite(vectors).all(axis=-1)
    finite &= (angle >= 0)&(speed >= 0)&(wrench >= 0)
    active = valid&finite&frame_valid[..., None]&np.isfinite(excitation)[..., None]
    vector_speed = np.linalg.norm(vectors, axis=-1)
    if np.any(active & ~np.isclose(vector_speed, speed, atol=1e-12, rtol=1e-10)):
        raise ValueError('Stored velocity norm disagrees with residual vector')
    normal_cost, velocity_cost, wrench_cost = (angle/normal_scale_rad)**2, (speed/velocity_scale_m_s)**2, wrench**2
    costs = (normal_cost, normal_cost+wrench_cost,
             normal_cost+velocity_cost, normal_cost+wrench_cost+velocity_cost)
    result = {}
    for mode, cost in zip(MODES, costs):
        scores = np.full(shape+(4,), np.inf)
        indices = np.full(shape+(4,), -1, int)
        for label in range(4):
            options = np.where(active&(family == label), cost, np.inf)
            best_index = np.argmin(options, axis=-1)
            best_score = np.take_along_axis(options, best_index[..., None], axis=-1)[..., 0]
            scores[..., label] = best_score
            indices[..., label] = np.where(np.isfinite(best_score), best_index, -1)
        order = np.argsort(scores, axis=-1, kind='stable')
        first, second = order[..., 0], order[..., 1]
        first_score = np.take_along_axis(scores, first[..., None], axis=-1)[..., 0]
        second_score = np.take_along_axis(scores, second[..., None], axis=-1)[..., 0]
        number = np.isfinite(scores).sum(axis=-1)
        with np.errstate(invalid='ignore'):
            margin = second_score-first_score
        margin = np.where(number >= 2, margin, np.nan)
        best_index = np.take_along_axis(indices, first[..., None], axis=-1)[..., 0]
        runner_index = np.take_along_axis(indices, second[..., None], axis=-1)[..., 0]
        safe_best, safe_runner = np.maximum(best_index, 0), np.maximum(runner_index, 0)
        best_vector = np.take_along_axis(vectors, safe_best[..., None, None], axis=-2)[..., 0, :]
        runner_vector = np.take_along_axis(vectors, safe_runner[..., None, None], axis=-2)[..., 0, :]
        best_speed = np.where(number > 0, np.linalg.norm(best_vector, axis=-1), np.nan)
        separation = np.where(number >= 2, np.linalg.norm(best_vector-runner_vector, axis=-1), np.nan)
        confident = (frame_valid&(number >= 2)&(excitation >= minimum_excitation_m_s)
                     &(margin >= minimum_margin)&(best_speed <= maximum_residual_m_s)
                     &(separation >= minimum_separation_m_s))
        chosen = np.where(number > 0, first, -1)
        result[mode] = dict(family_scores=scores, candidate_index=indices,
            chosen_family=chosen, accepted_family=np.where(confident, chosen, -1),
            score_margin=margin, valid_family_count=number, confident=confident,
            geometry_forced=number == 1, best_velocity_residual_m_s=best_speed,
            residual_separation_m_s=separation, excitation_m_s=excitation.copy())
    return result
