"""Observed-only terrain memory and geometric contact preview.

Inputs are calibrated axial-depth frames, capture transforms and timestamps.
This module has no scene, DEM, collision, contact-force or controller access.
The map is a static 2.5D local-plane approximation, not a full multi-contact
posterior. Circular equatorial rims are idealized rather than segmented.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import math

import numpy as np
from scipy.spatial import ConvexHull, QhullError


@dataclass(frozen=True)
class MemorySettings:
    cell_size_m: float = .01
    normal_radius_m: float = .04
    min_support_cells: int = 6
    max_plane_rms_m: float = .003
    min_xy_spread_m: float = .003
    max_slope_deg: float = 60.
    max_cell_height_spread_m: float = .02
    max_age_s: float | None = None

    def __post_init__(self):
        for name in ('cell_size_m','normal_radius_m','max_plane_rms_m',
                     'min_xy_spread_m','max_cell_height_spread_m'):
            if not np.isfinite(getattr(self,name)) or getattr(self,name)<=0:
                raise ValueError(f'{name} must be finite and positive')
        if isinstance(self.min_support_cells,(bool,np.bool_)) or not isinstance(self.min_support_cells,(int,np.integer)) or self.min_support_cells<3:
            raise ValueError('At least three distinct support cells are required')
        if not np.isfinite(self.max_slope_deg) or not 0<self.max_slope_deg<90:
            raise ValueError('max_slope_deg must lie between0 and90')
        if self.max_age_s is not None and (not np.isfinite(self.max_age_s) or self.max_age_s<0):
            raise ValueError('max_age_s must be nonnegative or None')


@dataclass
class _Cell:
    count: int
    point_sum: np.ndarray
    min_z: float
    max_z: float
    first_capture_s: float
    last_capture_s: float
    last_delivery_s: float

    @property
    def point(self): return self.point_sum/self.count

    @property
    def height_spread(self): return self.max_z-self.min_z


def _time(value,name):
    value=float(value)
    if not np.isfinite(value) or value<0:
        raise ValueError(f'{name} must be finite and nonnegative')
    return value


def _frame_points(frame):
    """Deproject observations only. Optical x right, y down, z forward."""
    depth=np.asarray(frame.depth_m,float);valid=np.asarray(frame.valid_mask,bool)
    intrinsic=np.asarray(frame.intrinsics,float);transform=np.asarray(frame.world_from_camera,float)
    if depth.ndim!=2 or valid.shape!=depth.shape:
        raise ValueError('Depth and validity must be matching2D arrays')
    if intrinsic.shape!=(3,3) or not np.isfinite(intrinsic).all() or intrinsic[0,0]<=0 or intrinsic[1,1]<=0 or abs(np.linalg.det(intrinsic))<1e-12:
        raise ValueError('Intrinsics must be a finite nonsingular pinhole matrix')
    if not np.allclose(intrinsic[2],[0.,0.,1.],atol=1e-12,rtol=0.):
        raise ValueError('Pinhole intrinsics must have bottom row[0,0,1]')
    if transform.shape!=(4,4) or not np.isfinite(transform).all() or not np.allclose(transform[3],[0,0,0,1],atol=1e-12,rtol=0.):
        raise ValueError('Capture transform must be a finite rigid4x4 matrix')
    rotation=transform[:3,:3]
    if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-9,rtol=0.) or not np.isclose(np.linalg.det(rotation),1.,atol=1e-9,rtol=0.):
        raise ValueError('Capture rotation must be proper orthonormal')
    if np.any(valid&(~np.isfinite(depth)|(depth<=0))):
        raise ValueError('Valid pixels must have finite positive axial depth')
    row,column=np.nonzero(valid)
    if not len(row): return np.empty((0,3)),depth,valid,intrinsic,transform
    try:
        rays=np.column_stack((column,row,np.ones(len(row))))@np.linalg.inv(intrinsic).T
    except np.linalg.LinAlgError as error:
        raise ValueError('Intrinsics must be nonsingular') from error
    if np.any(rays[:,2]<=0): raise ValueError('Optical rays must point forward')
    camera=rays*(depth[row,column]/rays[:,2])[:,None]
    world=camera@rotation.T+transform[:3,3]
    return world,depth,valid,intrinsic,transform


class TerrainMemory:
    """Monotone, observed-cell-only static elevation memory.

    ingest rejects observations not delivered by available_at_s. Querying an
    earlier anchor after later ingestion is forbidden; retain snapshot() for
    historical predictions. Plane fits use equal-weight distinct cell means,
    and require an observed query cell plus surrounding nondegenerate support.
    """
    def __init__(self,settings=None):
        self.settings=settings or MemorySettings()
        if not isinstance(self.settings,MemorySettings): raise ValueError('settings must be MemorySettings')
        self._cells={};self._frames={};self._available_at_s=0.

    def _key(self,xy): return tuple(np.floor(np.asarray(xy)/self.settings.cell_size_m).astype(np.int64))

    def _check_time(self,at_time_s):
        at_time_s=_time(at_time_s,'Query time')
        if at_time_s<self._available_at_s-1e-10:
            raise ValueError('A future-updated map cannot answer a past anchor; use a saved snapshot')
        return at_time_s

    def ingest(self,frames,available_at_s):
        """Atomically validate a batch before adding any of its depth points."""
        available_at_s=self._check_time(available_at_s)
        prepared=[];batch={};received_count=0
        for frame in frames:
            received_count+=1
            capture=_time(frame.capture_time_s,'Capture time');delivery=_time(frame.delivered_at_s,'Delivery time')
            if delivery<capture-1e-10 or capture>available_at_s+1e-10 or delivery>available_at_s+1e-10:
                raise ValueError('Only captured and delivered observations available at the anchor may enter memory')
            name=str(frame.camera_name)
            if not name: raise ValueError('A camera name is required')
            points,depth,valid,intrinsic,transform=_frame_points(frame)
            digest=hashlib.sha256()
            for value in (depth,valid,intrinsic,transform):digest.update(np.ascontiguousarray(value).tobytes())
            digest.update(repr((delivery,depth.shape)).encode())
            key=(name,capture);fingerprint=digest.hexdigest()
            previous=batch.get(key,self._frames.get(key))
            if previous is not None and previous!=fingerprint:
                raise ValueError('Conflicting observations share a camera/capture timestamp')
            if previous is None:prepared.append((key,fingerprint,points,capture,delivery))
            batch[key]=fingerprint
        points_added=0
        for frame_key,fingerprint,points,capture,delivery in prepared:
            self._frames[frame_key]=fingerprint;points_added+=len(points)
            if not len(points):continue
            indices=np.floor(points[:,:2]/self.settings.cell_size_m).astype(np.int64)
            unique,inverse=np.unique(indices,axis=0,return_inverse=True)
            counts=np.bincount(inverse);sums=np.column_stack([np.bincount(inverse,weights=points[:,axis]) for axis in range(3)])
            minimum=np.full(len(unique),np.inf);maximum=np.full(len(unique),-np.inf)
            np.minimum.at(minimum,inverse,points[:,2]);np.maximum.at(maximum,inverse,points[:,2])
            for i,index in enumerate(unique):
                key=tuple(index);cell=self._cells.get(key)
                if cell is None:
                    self._cells[key]=_Cell(int(counts[i]),sums[i].copy(),float(minimum[i]),float(maximum[i]),capture,capture,delivery)
                else:
                    cell.count+=int(counts[i]);cell.point_sum+=sums[i]
                    cell.min_z=min(cell.min_z,float(minimum[i]));cell.max_z=max(cell.max_z,float(maximum[i]))
                    cell.first_capture_s=min(cell.first_capture_s,capture);cell.last_capture_s=max(cell.last_capture_s,capture)
                    cell.last_delivery_s=max(cell.last_delivery_s,delivery)
        self._available_at_s=available_at_s
        return {'frames_added':len(prepared),'duplicate_frames':received_count-len(prepared),
                'points_added':points_added,'observed_cells':len(self._cells),'available_at_s':available_at_s}

    def snapshot(self,at_time_s):
        now=self._check_time(at_time_s);result=copy.deepcopy(self);result._available_at_s=now
        return result

    def observed_cells(self,at_time_s):
        """Copied arrays for visualization; means/counts/times include ambiguous cells."""
        now=self._check_time(at_time_s);cells=list(self._cells.values())
        return {'centers_m':np.array([c.point for c in cells]).reshape(-1,3),
            'count':np.array([c.count for c in cells],dtype=np.int64),
            'first_capture_s':np.array([c.first_capture_s for c in cells]),
            'last_capture_s':np.array([c.last_capture_s for c in cells]),
            'age_s':np.array([now-c.last_capture_s for c in cells]),
            'height_spread_m':np.array([c.height_spread for c in cells]),
            'ambiguous_height':np.array([c.height_spread>self.settings.max_cell_height_spread_m for c in cells],dtype=bool)}

    def query(self,xy,at_time_s):
        now=self._check_time(at_time_s);xy=np.asarray(xy,float)
        if xy.shape!=(2,) or not np.isfinite(xy).all():raise ValueError('Query xy must contain two finite values')
        result={'supported':False,'reason':'unobserved_cell','query_xy_m':xy.copy(),'point_m':None,'normal':None,
            'height_m':None,'plane_rms_m':None,'support_cells':0,'first_capture_s':None,'last_capture_s':None,
            'age_s':None,'at_time_s':now,'confidence':'unknown','assumption':'static2.5D local plane'}
        center=self._cells.get(self._key(xy))
        if center is None:return result
        result.update(first_capture_s=center.first_capture_s,last_capture_s=center.last_capture_s,age_s=now-center.last_capture_s,
                      query_cell_height_spread_m=center.height_spread)
        if center.height_spread>self.settings.max_cell_height_spread_m:
            result['reason']='multiple_heights_in_query_cell';return result
        if self.settings.max_age_s is not None and now-center.last_capture_s>self.settings.max_age_s+1e-10:
            result['reason']='stale_query_cell';return result
        radius=self.settings.normal_radius_m;extent=int(math.ceil(radius/self.settings.cell_size_m))+1;key=self._key(xy)
        cells=[];ambiguous=0
        for dx in range(-extent,extent+1):
            for dy in range(-extent,extent+1):
                cell=self._cells.get((key[0]+dx,key[1]+dy))
                if cell is None or np.linalg.norm(cell.point[:2]-xy)>radius+1e-12:continue
                if cell.height_spread>self.settings.max_cell_height_spread_m:
                    ambiguous+=1;continue
                if self.settings.max_age_s is not None and now-cell.last_capture_s>self.settings.max_age_s+1e-10:continue
                cells.append(cell)
        result.update(support_cells=len(cells),ambiguous_neighbor_cells=ambiguous)
        if len(cells)<self.settings.min_support_cells:
            result['reason']='insufficient_observed_support';return result
        points=np.array([cell.point for cell in cells]);relative=points[:,:2]-xy
        covariance=np.cov(relative,rowvar=False,bias=True)
        spread=float(np.sqrt(max(0.,np.linalg.eigvalsh(covariance)[0])))
        result['minor_xy_spread_m']=spread
        if spread<self.settings.min_xy_spread_m:
            result['reason']='degenerate_xy_support';return result
        try:hull=ConvexHull(points[:,:2])
        except QhullError:
            result['reason']='degenerate_xy_support';return result
        if np.any(hull.equations[:,:2]@xy+hull.equations[:,2]>1e-10):
            result['reason']='outside_observed_support_hull';return result
        design=np.column_stack((relative,np.ones(len(points))))
        coefficients=np.linalg.lstsq(design,points[:,2],rcond=None)[0]
        residual=points[:,2]-design@coefficients;rms=float(np.sqrt(np.mean(residual**2)))
        result['plane_rms_m']=rms
        if rms>self.settings.max_plane_rms_m:
            result['reason']='nonplanar_observed_support';return result
        normal=np.r_[-coefficients[:2],1.];normal/=np.linalg.norm(normal)
        slope=float(np.degrees(np.arctan2(np.linalg.norm(normal[:2]),normal[2])))
        result['slope_deg']=slope
        if slope>self.settings.max_slope_deg:
            result['reason']='surface_not_supported_as_heightfield';return result
        height=float(coefficients[2])
        result.update(supported=True,reason='local_observed_plane',height_m=height,point_m=np.r_[xy,height],normal=normal,
            confidence='geometric_fit_only_not_probability',support_hull_xy_m=points[hull.vertices,:2].copy(),
            support_oldest_last_capture_s=min(c.last_capture_s for c in cells),
            support_newest_last_capture_s=max(c.last_capture_s for c in cells))
        return result


@dataclass(frozen=True)
class WheelGeometry:
    radius_m: float
    polar_cap_angle_deg: float
    equatorial_gap_half_m: float

    def __post_init__(self):
        if not np.isfinite(self.radius_m) or self.radius_m<=0:raise ValueError('Wheel radius must be positive')
        if not np.isfinite(self.polar_cap_angle_deg) or not 0<self.polar_cap_angle_deg<90:raise ValueError('Cap angle must lie between0 and90')
        if not np.isfinite(self.equatorial_gap_half_m) or not 0<=self.equatorial_gap_half_m<self.radius_m*math.cos(math.radians(self.polar_cap_angle_deg)):
            raise ValueError('Equatorial gap must lie inside the shell band')

    @classmethod
    def from_config(cls,config):
        geometry=config['geometry']
        return cls(float(geometry['sphere_radius_m']),float(geometry['polar_cap_angle_deg']),float(geometry.get('equatorial_gap_half_m',0.)))


def _rotation(quaternion):
    q=np.asarray(quaternion,float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:raise ValueError('A finite nonzero wxyz quaternion is required')
    w,x,y,z=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
        [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
        [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def propagate_cv(qpos,qvel,horizon_s):
    """World translation, body-angular quaternion integration and joint rates."""
    q=np.asarray(qpos,float);v=np.asarray(qvel,float);horizon_s=_time(horizon_s,'Horizon')
    if q.shape!=(10,) or v.shape!=(9,) or not np.isfinite(q).all() or not np.isfinite(v).all():
        raise ValueError('CV preview requires finite qpos10 and qvel9')
    _rotation(q[3:7]);quaternion=q[3:7]/np.linalg.norm(q[3:7])
    vector=v[3:6]*horizon_s;angle=float(np.linalg.norm(vector));scale=.5 if angle<1e-12 else math.sin(angle/2)/angle
    delta=np.r_[math.cos(angle/2),scale*vector]
    w1,x1,y1,z1=quaternion;w2,x2,y2,z2=delta
    product=np.array([w1*w2-x1*x2-y1*y2-z1*z2,w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2,w1*z2+x1*y2-y1*x2+z1*w2])
    result=q.copy();result[:3]+=horizon_s*v[:3];result[3:7]=product/np.linalg.norm(product);result[7:]+=horizon_s*v[6:]
    return result


def carrier_axis(qpos):
    q=np.asarray(qpos,float)
    if q.shape!=(10,) or not np.isfinite(q).all():raise ValueError('Finite qpos10 required')
    phase=float(q[7]);axis=_rotation(q[3:7])@np.array([math.cos(phase),0.,-math.sin(phase)])
    return axis/np.linalg.norm(axis)


def contact_candidates(surface,qpos,geometry,band_tolerance_m=.001,max_gap_m=.005):
    """Possible geometric engagements against ONE estimated local plane.

    Signed clearance<=max_gap includes predicted penetration, explicitly logged.
    No normal loads, friction, actual contact decisions or support shares exist.
    """
    if not isinstance(geometry,WheelGeometry):raise ValueError('Known WheelGeometry required')
    if not np.isfinite(band_tolerance_m) or band_tolerance_m<0 or not np.isfinite(max_gap_m) or max_gap_m<0:
        raise ValueError('Geometry margins must be finite and nonnegative')
    if not surface.get('supported',False):return {'candidates':[],'all_candidates':[],'reason':surface.get('reason','unknown_surface')}
    normal=np.asarray(surface['normal'],float);point=np.asarray(surface['point_m'],float)
    if normal.shape!=(3,) or point.shape!=(3,) or not np.isfinite(normal).all() or not np.isfinite(point).all() or np.linalg.norm(normal)<1e-12:
        raise ValueError('A supported surface must supply finite point/normal')
    normal=normal/np.linalg.norm(normal)
    center=np.asarray(qpos,float)[:3];axis=carrier_axis(qpos);radius=geometry.radius_m
    support=-radius*normal;axial=float(axis@support);gap=geometry.equatorial_gap_half_m
    cap=radius*math.cos(math.radians(geometry.polar_cap_angle_deg));all_candidates=[]
    def append(patch,feature,vector):
        position=center+vector;clearance=float(normal@(position-point))
        row={'patch':patch,'feature':feature,'point_m':position,'normal':normal.copy(),
            'clearance_m':clearance,'predicted_penetration_m':max(0.,-clearance),'rim_idealized':feature=='rim'}
        if not any(old['patch']==patch and old['feature']==feature and np.linalg.norm(old['point_m']-position)<1e-10 for old in all_candidates):
            all_candidates.append(row)
    def boundary_vector(sign,value):
        direction_axis=sign*axis;x0=float(direction_axis@support);radial=support-x0*direction_axis
        radial_radius=math.sqrt(max(0.,radius*radius-value*value))
        if radial_radius==0.:
            # At a polar extremum the ring collapses to one point. Axial
            # roundoff must not require an undefined tangent direction.
            return value*direction_axis
        if np.linalg.norm(radial)<1e-12:
            return None
        return value*direction_axis+radial_radius*radial/np.linalg.norm(radial)
    for sign,side in ((1.,'A'),(-1.,'B')):
        x=sign*axial
        if gap-band_tolerance_m<=x<=cap+band_tolerance_m:
            clipped=float(np.clip(x,gap,cap));vector=support if clipped==x else boundary_vector(sign,clipped)
            if vector is not None:
                feature='rim' if abs(clipped-gap)<1e-10 else 'polar_edge' if abs(clipped-cap)<1e-10 else 'shell'
                append('shell_'+side,feature,vector)
        if cap-band_tolerance_m<=x<=radius+band_tolerance_m:
            clipped=float(np.clip(x,cap,radius));vector=support if clipped==x else boundary_vector(sign,clipped)
            if vector is not None:append('belt_'+side,'polar_edge' if abs(clipped-cap)<1e-10 else 'belt',vector)
        if gap>0 and abs(axial)<=gap+band_tolerance_m:
            vector=boundary_vector(sign,gap)
            if vector is not None:append('shell_'+side,'rim',vector)
    candidates=[row for row in all_candidates if row['clearance_m']<=max_gap_m+1e-12]
    return {'candidates':candidates,'all_candidates':all_candidates,
        'reason':'possible_geometric_contact' if candidates else 'no_near_contact',
        'assumption':'single local plane; ideal spherical bands and circular equatorial rims; no force prediction'}


def preview_contacts(provider,geometry,qpos,qvel,anchor_time_s,horizons_s=(.05,.1,.2),band_tolerance_m=.001,max_gap_m=.005):
    """Same CV endpoints for any provider.query(xy,at_time_s) implementation."""
    anchor=_time(anchor_time_s,'Anchor time');rows=[]
    for horizon in horizons_s:
        horizon=_time(horizon,'Horizon');predicted=propagate_cv(qpos,qvel,horizon)
        surface=provider.query(predicted[:2],at_time_s=anchor)
        row={'horizon_s':horizon,'time_s':anchor+horizon,'predicted_qpos':predicted,
            'surface':surface,'supported_surface':bool(surface.get('supported',False)),
            'supported':False,'contact_prediction_supported':False,'candidates':[],'ambiguous':False,
            'known_no_near_contact':False,'reason':surface.get('reason','unknown_surface'),
            'assumption':'CV motion and one local fitted heightfield plane; geometry only'}
        if not row['supported_surface']:
            rows.append(row);continue
        geometric=contact_candidates(surface,predicted,geometry,band_tolerance_m,max_gap_m)
        confirmed=[];unsupported=[]
        for candidate in geometric['candidates']:
            foot=provider.query(candidate['point_m'][:2],at_time_s=anchor)
            if not foot.get('supported',False):
                unsupported.append({'patch':candidate['patch'],'reason':foot.get('reason','unknown_contact_region')});continue
            normal=np.asarray(surface['normal']);point=np.asarray(surface['point_m']);xy=candidate['point_m'][:2]
            expected_height=float(point[2]-normal[:2]@(xy-point[:2])/normal[2])
            tolerance=getattr(getattr(provider,'settings',None),'max_plane_rms_m',.003)
            inconsistency=abs(float(foot['height_m'])-expected_height)
            if inconsistency>tolerance+1e-12:
                unsupported.append({'patch':candidate['patch'],'reason':'nonlocal_surface_disagreement','height_difference_m':inconsistency});continue
            confirmed.append({**candidate,'contact_surface_query':foot,'local_surface_height_difference_m':inconsistency})
        complete=not unsupported
        row.update(supported=complete,contact_prediction_supported=complete,candidates=confirmed,
            ambiguous=len({x['patch'] for x in confirmed})>1 or bool(unsupported),
            unsupported_contact_queries=unsupported,known_no_near_contact=complete and not confirmed,
            reason=('partial_contact_region_unknown' if unsupported else geometric['reason']),
            all_geometric_candidates=geometric['all_candidates'])
        rows.append(row)
    return {'anchor_time_s':anchor,'rows':rows,'measurement_boundary':'Only provider observations available by the anchor; no future executed state or force input'}
