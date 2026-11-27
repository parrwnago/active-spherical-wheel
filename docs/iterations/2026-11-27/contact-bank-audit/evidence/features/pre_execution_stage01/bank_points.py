"""I67 reconstruction of the ORIGINAL 102 I60 geometry slots; NumPy only.

This does not choose candidates, change validity or rerank a bank. The scalar
seed/projection operation order below is copied from frozen I60 geometry so
frozen selected points can be compared byte for byte. Raw projected slots are
not automatically usable: original active masks remain authoritative.
"""
import math
import numpy as np

DEFAULT_CONE_RAD=math.radians(10.)


def _unit(value):
    value=np.asarray(value,float)
    size=float(np.linalg.norm(value))
    return value/size if np.isfinite(size) and size>1e-12 else None


def _tangent_basis(normal):
    axis=np.eye(3)[int(np.argmin(abs(normal)))]
    first=np.cross(normal,axis)
    first/=np.linalg.norm(first)
    return first,np.cross(normal,first)


def normal_seeds(normal,cone_rad=DEFAULT_CONE_RAD):
    if np.asarray(normal).shape!=(3,):raise ValueError('Normal must have three components')
    normal=_unit(normal)
    if normal is None:raise ValueError('A finite nonzero normal is required')
    if not np.isfinite(cone_rad) or not 0<cone_rad<.5*np.pi:raise ValueError('Normal cone must lie between zero and pi/2')
    first,second=_tangent_basis(normal);seeds=[normal]
    for angle in (.5*cone_rad,cone_rad):
        for azimuth in np.arange(8)*(.25*np.pi):
            tangent=np.cos(azimuth)*first+np.sin(azimuth)*second
            seeds.append(np.cos(angle)*normal+np.sin(angle)*tangent)
    return np.asarray(seeds)


def _band_projection(offset,axis,radius,lower,upper):
    axial=float(np.clip(axis@offset,lower,upper))
    radial=offset-axis*float(axis@offset)
    size=float(np.linalg.norm(radial))
    if size<=1e-12:radial=_tangent_basis(axis)[0]
    else:radial/=size
    return axial*axis+math.sqrt(max(radius*radius-axial*axial,0.))*radial


def candidate_layout():
    return dict(candidate_index=np.arange(102,dtype=np.int64),seed_index=np.repeat(np.arange(17,dtype=np.int64),6),
        slot=np.tile(np.arange(6,dtype=np.int8),17),family_index=np.tile(np.array([0,1,2,3,0,1],np.int8),17),
        kind=np.tile(np.array([0,0,0,0,1,1],np.int8),17))


def _frame(pose,normal_hint,cone_rad):
    centre=np.asarray(pose['centre_m'],float);axes=np.asarray(pose['patch_axis_world'],float)
    lower=np.asarray(pose['patch_lower_m'],float);upper=np.asarray(pose['patch_upper_m'],float);radius=float(np.asarray(pose['radius_m']))
    if centre.shape!=(3,) or axes.shape!=(4,3) or lower.shape!=(4,) or upper.shape!=(4,):raise ValueError('Expected one cached four-family pose')
    if not np.isfinite(radius) or radius<=0 or not np.isfinite(np.r_[lower,upper]).all() or np.any(lower<0) or np.any(lower>=upper) or np.any(upper>radius):raise ValueError('Invalid original spherical-band metadata')
    if np.asarray(normal_hint).shape!=(3,):raise ValueError('Normal hint must have three components')
    normal=_unit(normal_hint)
    known=bool(pose['valid']) and np.isfinite(centre).all() and np.isfinite(axes).all() and normal is not None
    # The outer unit conversion and the inner one in normal_seeds are BOTH
    # intentional: frozen I60/I61 did the same two normalizations.
    seeds=normal_seeds(normal,cone_rad) if known else None
    return centre,axes,lower,upper,radius,normal,seeds


def reconstruct_selected(pose,normal_hint,candidate_ids,*,normal_cone_rad=DEFAULT_CONE_RAD):
    """Reconstruct supplied ORIGINAL IDs only; negative IDs remain unknown."""
    ids=np.asarray(candidate_ids)
    if not np.issubdtype(ids.dtype,np.integer):raise ValueError('Original candidate IDs must be integers')
    if np.any(ids>=102):raise ValueError('Candidate ID outside original bank')
    centre,axes,lower,upper,radius,normal,seeds=_frame(pose,normal_hint,normal_cone_rad)
    result=np.full(ids.shape+(3,),np.nan)
    if seeds is None:return result
    families=np.array([0,1,2,3,0,1])
    for output,index in zip(result.reshape(-1,3),ids.reshape(-1)):
        if index<0:continue
        seed,slot=divmod(int(index),6);family=int(families[slot])
        bound=lower[family] if slot>=4 else upper[family]
        offset=_band_projection(-radius*seeds[seed],axes[family],radius,lower[family],bound)
        output[:]=centre+offset
    return result


def reconstruct_bank(pose,normal_hint,*,observer_force_N=None,original_frame_valid=True,
                     candidate_mask=None,normal_cone_rad=DEFAULT_CONE_RAD):
    """Return original slots and geometry-only old admission diagnostics.

    With no mask all pre-admission projected slots are reconstructed. A supplied
    mask only skips requested work; it is never inferred or changed here.
    original_geometry_valid applies the old cone/compression checks and given
    old frame flag. It is not the complete original scoring active mask, which
    additionally uses cached finite motion/features/excitation.
    """
    layout=candidate_layout();requested=np.ones(102,bool) if candidate_mask is None else np.asarray(candidate_mask,bool)
    if requested.shape!=(102,):raise ValueError('Expected the original 102-slot mask')
    ids=layout['candidate_index'][requested]
    points=np.full((102,3),np.nan);points[requested]=reconstruct_selected(pose,normal_hint,ids,normal_cone_rad=normal_cone_rad)
    centre,axes,lower,upper,radius,normal,seeds=_frame(pose,normal_hint,normal_cone_rad)
    defined=np.isfinite(points).all(axis=1);angles=np.full(102,np.nan);compression=np.full(102,np.nan);axial=np.full(102,np.nan)
    band=np.zeros(102,bool);admitted=np.zeros(102,bool)
    force=None if observer_force_N is None else np.asarray(observer_force_N,float)
    if force is not None and force.shape!=(3,):raise ValueError('Expected world observer force with three components')
    frame_force=(force is not None and np.isfinite(force).all() and normal is not None
        and float(np.linalg.norm(force))>=1. and float(force@normal)>=1.)
    for index in np.flatnonzero(defined):
        family=int(layout['family_index'][index]);slot=int(layout['slot'][index]);offset=points[index]-centre
        candidate_normal=-offset/radius
        angles[index]=float(np.arctan2(np.linalg.norm(np.cross(normal,candidate_normal)),np.clip(normal@candidate_normal,-1.,1.)))
        axial[index]=float(axes[family]@offset);bound=lower[family] if slot>=4 else upper[family]
        band[index]=lower[family]-1e-12<=axial[index]<=bound+1e-12
        if force is not None:compression[index]=float(force@candidate_normal)
        admitted[index]=bool(original_frame_valid and frame_force and band[index] and angles[index]<=normal_cone_rad+1e-12 and compression[index]>=1.)
    return dict(**layout,points_m=points,reconstructed=defined,requested=requested.copy(),normal_angle_rad=angles,
        normal_force_N=compression,axial_coordinate_m=axial,within_band=band,original_geometry_valid=admitted)
