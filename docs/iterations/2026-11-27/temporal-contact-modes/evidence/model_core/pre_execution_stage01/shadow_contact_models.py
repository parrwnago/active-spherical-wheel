"""Opt-in contact profile potentials; no live controller or simulator imports.

Twenty known modes encode four singletons and six pairs, each rolling/slipping;
mode20 is unknown. The numbers are engineering profile potentials, not exact
marginal likelihoods or calibrated contact probabilities. Contact forces are
conditionally optimized; a short, observation-dependent point shortlist is
also profiled/averaged. No function in this module updates a live command.

Force normals point into the robot. Maps transform world point force to the
same9 generalized coordinates as observation9. Material velocities include any
belt surface motion; this module projects them into each candidate tangent.
The caller owns interval timing, map frames and temporal inference.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations

import numpy as np
from scipy.optimize import nnls


SUPPORT_SETS = tuple((i,) for i in range(4)) + tuple(combinations(range(4), 2))
PAIR_SETS = tuple(combinations(range(4), 2))
UNKNOWN_INDEX = 20
MODE_FAMILIES = np.full((21, 2), -1, dtype=np.int64)
for _i, _families in enumerate(SUPPORT_SETS):
    MODE_FAMILIES[2*_i:2*_i+2, :len(_families)] = _families
MODE_MOTION = np.r_[np.tile([0, 1], 10), -1]  #0 rolling,1 slip,−1 unknown
MODE_FAMILIES.setflags(write=False)
MODE_MOTION.setflags(write=False)


@dataclass(frozen=True)
class ModelConfig:
    force_sigma_N: float = .5
    moment_sigma_Nm: float = .03
    joint_sigma_Nm: float = .03
    rolling_sigma_m_s: float = .02
    slip_sigma_m_s: float = .15
    normal_sigma_rad: float = np.deg2rad(10.)
    friction_coefficient: float = .7
    minimum_pair_normal_share: float = .1
    minimum_observed_force_N: float = 1.
    unknown_cost: float = 9.
    nnls_maxiter: int = 256
    point_dedup_tolerance_m: float = 1e-12

    def __post_init__(self):
        positive = (self.force_sigma_N, self.moment_sigma_Nm, self.joint_sigma_Nm,
                    self.rolling_sigma_m_s, self.slip_sigma_m_s, self.normal_sigma_rad)
        nonnegative = (self.friction_coefficient, self.minimum_observed_force_N,
                       self.unknown_cost, self.point_dedup_tolerance_m)
        if (not np.isfinite(positive+nonnegative).all() or min(positive) <= 0
                or min(nonnegative) < 0 or self.slip_sigma_m_s < self.rolling_sigma_m_s
                or not 0 < self.minimum_pair_normal_share < .5
                or int(self.nnls_maxiter) != self.nnls_maxiter or self.nnls_maxiter < 1):
            raise ValueError('Invalid fixed contact-model assumptions')

    @property
    def residual_sigma(self):
        return np.array([self.force_sigma_N]*3+[self.moment_sigma_Nm]*3+[self.joint_sigma_Nm]*3)


def friction_rays(normal, coefficient=.7):
    """Four inscribed square-pyramid rays, each with unit normal component."""
    normal = np.asarray(normal, float)
    coefficient = float(coefficient)
    if (normal.shape != (3,) or not np.isfinite(normal).all()
            or np.linalg.norm(normal) < 1e-12 or not np.isfinite(coefficient) or coefficient < 0):
        raise ValueError('Finite nonzero inward normal and nonnegative friction required')
    n = normal/np.linalg.norm(normal)
    seed = np.eye(3)[int(np.argmin(np.abs(n)))]
    tangent1 = np.cross(n, seed)
    tangent1 /= np.linalg.norm(tangent1)
    tangent2 = np.cross(n, tangent1)
    signs = np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]])
    return n[:, None]+coefficient/np.sqrt(2.)*(tangent1[:, None]*signs[:, 0]+tangent2[:, None]*signs[:, 1])


def fit_contact_forces(observation9, force_maps, force_normals, config=None):
    """NNLS for one/two forces, with both pair normal shares at least.1.

    Pair columns combine all16 ray pairs at the two extreme admissible load
    shares. Nonnegative combinations preserve the share bounds and both cones.
    There is no additional force-effort term or pair-complexity penalty here.
    """
    config = config or ModelConfig()
    y, maps, normals = (np.asarray(x, float) for x in (observation9, force_maps, force_normals))
    k = len(maps)
    if y.shape != (9,) or maps.shape != (k, 9, 3) or normals.shape != (k, 3) or k not in (1, 2):
        raise ValueError('Expected observation9 and one/two9x3 maps and normals')
    if not all(np.isfinite(x).all() for x in (y, maps, normals)):
        raise ValueError('Finite force-fitting inputs required')
    rays = [friction_rays(n, config.friction_coefficient) for n in normals]
    if k == 1:
        generators = rays[0][None, :, :]
    else:
        m = config.minimum_pair_normal_share
        columns = []
        for share in (m, 1.-m):
            for i in range(4):
                for j in range(4):
                    columns.append(np.stack((share*rays[0][:, i], (1.-share)*rays[1][:, j])))
        generators = np.moveaxis(np.stack(columns), 0, -1)  #contacts,xyz,32
    design = np.einsum('kij,kjl->il', maps, generators)
    sigma = config.residual_sigma
    whitened = design/sigma[:, None]
    result = dict(valid=False, forces_N=np.full((k, 3), np.nan), predicted9=np.full(9, np.nan),
        residual9=np.full(9, np.nan), normal_share=np.full(k, np.nan), cost=np.inf,
        rank=-1, active_rays=0, solve_failed=False)
    try:
        weights, _ = nnls(whitened, y/sigma, maxiter=config.nnls_maxiter)
    except (RuntimeError, np.linalg.LinAlgError):
        result['solve_failed'] = True
        return result
    forces = np.einsum('kij,j->ki', generators, weights)
    n = normals/np.linalg.norm(normals, axis=1)[:, None]
    normal_load = np.sum(forces*n, axis=1)
    total_load = float(normal_load.sum())
    if not np.isfinite(forces).all() or total_load <= 1e-9:
        return result  #Do not manufacture a zero-force support/ghost pair.
    share = normal_load/total_load
    if k == 2 and np.min(share) < config.minimum_pair_normal_share-1e-10:
        raise ArithmeticError('Pair force construction violated its normal-share constraint')
    prediction = np.einsum('kij,kj->i', maps, forces)
    residual = y-prediction
    force_map = np.concatenate(maps, axis=1)/sigma[:, None]
    result.update(valid=True, forces_N=forces, predicted9=prediction, residual9=residual,
        normal_share=share, cost=.5*float(np.sum((residual/sigma)**2)),
        rank=int(np.linalg.matrix_rank(force_map)), active_rays=int(np.count_nonzero(weights > 1e-10)))
    return result


def _storage(shape):
    def floating(tail=()):return np.full(shape+tail, np.nan)
    return dict(valid=np.zeros(shape, bool), selected_candidate_index=np.full(shape+(2,), -1, np.int64),
        selected_point_m=floating((2, 3)), estimated_force_N=floating((2, 3)),
        predicted_generalized9=floating((9,)), residual9=floating((9,)),
        normal_share=floating((2,)), force_fit_cost=floating(), geometry_cost=floating(),
        velocity_cost=floating(), map_complete_cost=floating(),
        conditional_point_mean_m=floating((2, 3)), conditional_point_covariance_m2=floating((2, 3, 3)),
        shortlist_map_weight=floating(), fit_rank=np.full(shape, -1, np.int64),
        active_rays=np.zeros(shape, np.int64), alternative_count=np.zeros(shape, np.int64),
        finite_alternative_count=np.zeros(shape, np.int64))


def _aggregate(alternatives, costs, velocity_costs, geometry_costs, points, ids, storage, destination):
    """Mean exponential of complete costs; MAP output stays one coupled fit."""
    storage['alternative_count'][destination] = len(alternatives)
    costs = np.asarray(costs, float)
    finite = np.isfinite(costs)
    storage['finite_alternative_count'][destination] = int(finite.sum())
    if not finite.any():return np.inf
    best = int(np.argmin(costs))
    low = float(costs[best])
    mass = np.zeros(len(costs));mass[finite] = np.exp(-(costs[finite]-low))
    potential_cost = low-np.log(np.sum(mass)/len(costs))
    conditional = mass/mass.sum()
    rows, fit = alternatives[best]
    k = len(rows)
    storage['valid'][destination] = True
    storage['selected_candidate_index'][destination][:k] = ids[list(rows)]
    storage['selected_point_m'][destination][:k] = points[list(rows)]
    storage['estimated_force_N'][destination][:k] = fit['forces_N']
    storage['predicted_generalized9'][destination] = fit['predicted9']
    storage['residual9'][destination] = fit['residual9']
    storage['normal_share'][destination][:k] = fit['normal_share']
    for key, value in (('force_fit_cost', fit['cost']), ('geometry_cost', geometry_costs[best]),
                       ('velocity_cost', velocity_costs[best]), ('map_complete_cost', low),
                       ('shortlist_map_weight', conditional[best]), ('fit_rank', fit['rank']),
                       ('active_rays', fit['active_rays'])):
        storage[key][destination] = value
    for slot in range(k):
        cloud = np.stack([points[item[0][slot]] for item in alternatives])
        mean = np.sum(conditional[:, None]*cloud, axis=0)
        delta = cloud-mean
        covariance = np.einsum('i,ij,ik->jk', conditional, delta, delta)
        storage['conditional_point_mean_m'][destination][slot] = mean
        storage['conditional_point_covariance_m2'][destination][slot] = covariance
    return float(potential_cost)


def evaluate_frame(observation9, points_m, force_normals, material_velocity_m_s,
                   force_maps, family_index, valid, normal_hint, candidate_index=None,
                   *, dt_s=.005, observation_valid=True, config=None):
    """Return node_cost21, edge_cost6x2x2 and complete-fit diagnostics.

    Node order is singleton0..3 then lexicographic pairs, rolling/slip each,
    followed by unknown20. An edge[pair,endpoint_role,motion] fits both interval
    forces but scores velocity only at its endpoint family. Both geometry
    priors remain present. No temporal transition prior or dt tempering occurs.

    Covariances/weights describe only the conditional point shortlist, not
    calibrated uncertainty, the whole continuous surface or identifiable forces.
    """
    config = config or ModelConfig()
    y = np.asarray(observation9, float)
    points, normals, velocity, maps = (np.asarray(x, float) for x in
        (points_m, force_normals, material_velocity_m_s, force_maps))
    family, admitted = np.asarray(family_index), np.asarray(valid, bool)
    hint = np.asarray(normal_hint, float)
    count = len(points)
    ids = np.arange(count, dtype=np.int64) if candidate_index is None else np.asarray(candidate_index)
    if (y.shape != (9,) or points.shape != (count, 3) or normals.shape != (count, 3)
            or velocity.shape != (count, 3) or maps.shape != (count, 9, 3)
            or family.shape != (count,) or admitted.shape != (count,) or ids.shape != (count,)
            or hint.shape != (3,) or not np.isfinite(dt_s) or dt_s <= 0):
        raise ValueError('Invalid contact-model array shapes or interval duration')
    if (not np.isfinite(family).all() or not np.equal(family, np.floor(family)).all()
            or np.any((family < 0) | (family >= 4)) or not np.isfinite(ids).all()
            or not np.equal(ids, np.floor(ids)).all() or np.any(ids < 0) or len(np.unique(ids)) != count):
        raise ValueError('Four known families and distinct nonnegative candidate IDs required')
    family, ids = family.astype(np.int64), ids.astype(np.int64)
    output = dict(node_cost=np.full(21, np.inf), edge_cost=np.full((6, 2, 2), np.inf),
        node=_storage((21,)), edge=_storage((6, 2, 2)),
        shortlist_candidate_index=np.full((4, 2), -1, np.int64), shortlist_count=np.zeros(4, np.int64),
        force_fit_count=0, failed_force_fit_count=0, observation_valid=False,
        potential_scope='Uncalibrated conditional profile potentials; no temporal prior or normalized posterior.')
    output['node_cost'][UNKNOWN_INDEX] = config.unknown_cost
    if (not observation_valid or not np.isfinite(y).all() or np.linalg.norm(y[:3]) < config.minimum_observed_force_N
            or not np.isfinite(hint).all() or np.linalg.norm(hint) < 1e-12):
        return output
    output['observation_valid'] = True
    eligible = (admitted & np.isfinite(points).all(axis=1) & np.isfinite(normals).all(axis=1)
                & np.isfinite(velocity).all(axis=1) & np.isfinite(maps).all(axis=(1, 2)))
    lengths = np.linalg.norm(normals, axis=1)
    eligible &= lengths > 1e-12
    unit = np.zeros_like(normals)
    unit[eligible] = normals[eligible]/lengths[eligible, None]
    hint = hint/np.linalg.norm(hint)
    angle = np.arccos(np.clip(unit@hint, -1., 1.))
    geometry = .5*(angle/config.normal_sigma_rad)**2
    tangent = velocity-unit*np.sum(unit*velocity, axis=1)[:, None]
    squared_speed = np.sum(tangent*tangent, axis=1)
    velocity_cost = np.stack([.5*squared_speed/sigma**2+2.*np.log(sigma/config.rolling_sigma_m_s)
        for sigma in (config.rolling_sigma_m_s, config.slip_sigma_m_s)])
    shortlist = []
    for label in range(4):
        rows = np.flatnonzero(eligible & (family == label))
        selected = []
        if len(rows):
            for score in (geometry, geometry+velocity_cost[0]):
                row = min(rows, key=lambda item: (score[item], ids[item]))
                if not any(np.linalg.norm(points[row]-points[old]) <= config.point_dedup_tolerance_m for old in selected):
                    selected.append(int(row))
        shortlist.append(selected)
        output['shortlist_count'][label] = len(selected)
        output['shortlist_candidate_index'][label, :len(selected)] = ids[selected]
    for support_index, labels in enumerate(SUPPORT_SETS):
        if len(labels) == 1:
            options = [(row,) for row in shortlist[labels[0]]]
        else:
            options = [(a, b) for a in shortlist[labels[0]] for b in shortlist[labels[1]]]
        alternatives = []
        for rows in options:
            fit = fit_contact_forces(y, maps[list(rows)], unit[list(rows)], config)
            output['force_fit_count'] += 1
            output['failed_force_fit_count'] += int(fit['solve_failed'])
            alternatives.append((rows, fit))
        geometric = [float(np.sum(geometry[list(rows)])) for rows, _ in alternatives]
        for motion in range(2):
            velocities = [float(np.sum(velocity_cost[motion, list(rows)])) for rows, _ in alternatives]
            costs = [fit['cost']+g+v if fit['valid'] else np.inf
                for (_, fit), g, v in zip(alternatives, geometric, velocities)]
            mode = 2*support_index+motion
            output['node_cost'][mode] = _aggregate(alternatives, costs, velocities, geometric, points, ids, output['node'], mode)
        if len(labels) == 2:
            pair = support_index-4
            for endpoint in range(2):
                for motion in range(2):
                    velocities = [float(velocity_cost[motion, rows[endpoint]]) for rows, _ in alternatives]
                    costs = [fit['cost']+g+v if fit['valid'] else np.inf
                        for (_, fit), g, v in zip(alternatives, geometric, velocities)]
                    destination = (pair, endpoint, motion)
                    output['edge_cost'][destination] = _aggregate(alternatives, costs, velocities, geometric, points, ids, output['edge'], destination)
    return output
