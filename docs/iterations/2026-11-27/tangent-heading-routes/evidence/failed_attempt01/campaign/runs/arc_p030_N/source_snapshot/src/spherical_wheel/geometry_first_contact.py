"""Opt-in geometry-first reduction before the unchanged frozen I60 ranker.

One original active point per family is selected by squared world distance to
the native observer's equivalent point. The anchor may be inside the sphere;
it is not projected or replaced by another family's physical candidate.
Velocity/moment values enter only the ORIGINAL rank eligibility validation and
the subsequent unchanged ranker, never the point-distance objective.

The caller supplies the provenance-bound frozen rank_hypotheses callable and
owns native fallback. This module imports no live estimator/controller, model,
simulator, truth source, temporal filter or optimizer.
"""
from __future__ import annotations

import numpy as np


PROJECTED_FIELDS=('family_index','valid','frame_valid','excitation_m_s',
    'normal_angle_rad','velocity_residual_m_s','velocity_residual_norm_m_s','moment_residual_norm')


def legacy_active_mask(projected_features):
    """The original I60 active predicate, including its norm consistency check.

    The minimum excitation threshold belongs to rank acceptance, not active
    candidate eligibility. Low but finite excitation preserves raw choices.
    """
    missing=set(PROJECTED_FIELDS)-set(projected_features)
    if missing:raise ValueError('Missing projected rank fields: '+','.join(sorted(missing)))
    valid=np.asarray(projected_features['valid'],bool)
    if valid.ndim<1 or valid.shape[-1]==0:raise ValueError('At least one candidate slot required')
    shape,count=valid.shape[:-1],valid.shape[-1]
    family=np.asarray(projected_features['family_index'])
    if (family.shape!=(count,) or not np.isfinite(family).all()
            or not np.equal(family,np.floor(family)).all() or not np.isin(family,np.arange(4)).all()):
        raise ValueError('Fixed original family labels0..3 required')
    angle,speed,moment=(np.asarray(projected_features[key],float) for key in
        ('normal_angle_rad','velocity_residual_norm_m_s','moment_residual_norm'))
    vectors=np.asarray(projected_features['velocity_residual_m_s'],float)
    if any(value.shape!=valid.shape for value in (angle,speed,moment)) or vectors.shape!=valid.shape+(3,):
        raise ValueError('Projected candidate feature shapes differ')
    try:
        frame=np.broadcast_to(np.asarray(projected_features['frame_valid'],bool),shape)
        excitation=np.broadcast_to(np.asarray(projected_features['excitation_m_s'],float),shape)
    except ValueError as error:raise ValueError('Frame/excitation shape differs') from error
    finite=np.isfinite(angle)&np.isfinite(speed)&np.isfinite(moment)&np.isfinite(vectors).all(axis=-1)
    finite&=(angle>=0)&(speed>=0)&(moment>=0)
    active=valid&finite&frame[...,None]&np.isfinite(excitation)[...,None]
    vector_speed=np.linalg.norm(vectors,axis=-1)
    if np.any(active&~np.isclose(vector_speed,speed,atol=1e-12,rtol=1e-10)):
        raise ValueError('Stored velocity norm disagrees with residual vector')
    return active


def select_geometry_points(projected_features,world_points,native_anchor_point_m,candidate_ids):
    """Geometry-only family representatives with exact ties broken by ID.

    Leading batch dimensions are independent frames; no time/history is read.
    Inactive point coordinates may be NaN. A missing active coordinate is an
    input error, not a new silent geometry gate. Missing/nonfinite anchors are
    explicit and leave every candidate unselected.
    """
    active=legacy_active_mask(projected_features)
    shape,count=active.shape[:-1],active.shape[-1]
    points=np.asarray(world_points,float)
    ids=np.asarray(candidate_ids)
    if points.shape!=active.shape+(3,):raise ValueError('World points must match original candidate slots')
    if (ids.shape!=(count,) or not np.isfinite(ids).all() or not np.equal(ids,np.floor(ids)).all()
            or np.any(ids<0) or np.any(ids>np.iinfo(np.int64).max) or len(np.unique(ids))!=count):
        raise ValueError('Distinct nonnegative original candidate IDs required')
    ids=ids.astype(np.int64)
    if np.any(ids<0):raise ValueError('Candidate IDs exceed the integer representation')
    if np.any(active&~np.isfinite(points).all(axis=-1)):
        raise ValueError('Original active candidate is missing a physical coordinate')
    if native_anchor_point_m is None:anchor=np.full(shape+(3,),np.nan)
    else:
        anchor=np.asarray(native_anchor_point_m,float)
        if anchor.shape!=shape+(3,):raise ValueError('One native equivalent point per frame required')
    available=np.isfinite(anchor).all(axis=-1)
    with np.errstate(invalid='ignore',over='ignore'):
        squared=np.sum((points-anchor[...,None,:])**2,axis=-1)
    eligible=active&available[...,None]
    if np.any(eligible&~np.isfinite(squared)):
        raise ValueError('Squared physical point distance is not numerically finite')
    family=np.asarray(projected_features['family_index'],int)
    selected=np.zeros(active.shape,bool)
    selected_id=np.full(shape+(4,),-1,np.int64)
    selected_row=np.full(shape+(4,),-1,np.int64)
    distances=np.full(shape+(4,),np.nan)
    for label in range(4):
        options=np.where(eligible&(family==label),squared,np.inf)
        distance=np.min(options,axis=-1)
        has=np.isfinite(distance)
        ties=eligible&(family==label)&(squared==distance[...,None])
        best_id=np.min(np.where(ties,ids,np.iinfo(np.int64).max),axis=-1)
        chosen=ties&(ids==best_id[...,None])
        selected|=chosen
        selected_id[...,label]=np.where(has,best_id,-1)
        selected_row[...,label]=np.where(has,np.argmax(chosen,axis=-1),-1)
        distances[...,label]=np.where(has,distance,np.nan)
    return dict(selected_candidate_id=selected_id,selected_row_index=selected_row,
        selected_mask=selected,family_distance_squared_m2=distances,
        anchor_available=np.array(available,copy=True),legacy_active_mask=active)


def geometry_first_rank(projected_features,world_points,native_anchor_point_m,candidate_ids,
                        *,rank_hypotheses,rank_options=None):
    """Reduce by geometry, then delegate all scores/gates to frozen I60 code.

    Ranking candidate_index values retain the frozen ranker's INPUT ROW-index
    meaning; selected_candidate_id separately records original bank IDs.
    No native family, fallback decision or evaluation label is accepted here.
    """
    if not callable(rank_hypotheses):raise TypeError('Explicit frozen I60 rank_hypotheses callable required')
    result=select_geometry_points(projected_features,world_points,native_anchor_point_m,candidate_ids)
    features={key:np.array(projected_features[key],copy=True) for key in PROJECTED_FIELDS}
    features['valid']=result['selected_mask'].copy()
    result['ranking']=rank_hypotheses(features,**({} if rank_options is None else dict(rank_options)))
    return result
