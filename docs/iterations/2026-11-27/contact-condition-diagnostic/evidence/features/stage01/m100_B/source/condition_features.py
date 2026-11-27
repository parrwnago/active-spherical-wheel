"""Causal, candidate-independent observer/known-patch diagnostics (I-63).

The pure NumPy projection accepts cached kinematic axes and measured/inferred
signals. It does not accept candidate scores, native support shares or truth.
MuJoCo is imported only by the optional private pose-to-axis projection.
"""
import numpy as np

PATCH_NAMES=('shell_A','shell_B','belt_A','belt_B')


def backward_rate(time_s,value,valid,sample_period_s=.005,time_tolerance_s=1e-9):
    """One adjacent backward interval; invalid samples never get bridged."""
    t=np.asarray(time_s,float);x=np.asarray(value,float);valid=np.asarray(valid,bool)
    if t.ndim!=1 or x.shape!=t.shape or valid.shape!=t.shape:raise ValueError('Expected equal one-dimensional histories')
    if (not np.isfinite([sample_period_s,time_tolerance_s]).all() or sample_period_s<=0 or time_tolerance_s<0):raise ValueError('Invalid logged sample period')
    if not np.isfinite(t).all() or np.any(np.diff(t)<=0):raise ValueError('Strictly increasing finite times required')
    rate=np.full(len(t),np.nan);ok=np.zeros(len(t),bool)
    if len(t)>1:
        adjacent=np.abs(np.diff(t)-sample_period_s)<=time_tolerance_s
        ok[1:]=valid[1:]&valid[:-1]&np.isfinite(x[1:])&np.isfinite(x[:-1])&adjacent
        differences=np.diff(x)/np.diff(t)
        rate[1:]=np.where(ok[1:],differences,np.nan)
    return rate,ok


def derive_features(primitives,minimum_force_N=1.,sample_period_s=.005,time_tolerance_s=1e-9):
    """All outputs at index i use inputs at i and, for rates, i-1 only.

    ``interval_mean[i]`` is the observer's model-derived load for the interval
    ending at ``time[i]``. Its difference from the current filtered observer is
    a filter innovation, not a new force sensor or an independent true load.
    Positive seam approach means the nearest raw-normal seam distance is
    decreasing. This minimum can switch boundaries; it is not predicted contact.
    """
    if not np.isfinite(minimum_force_N) or minimum_force_N<=0:raise ValueError('Positive force floor required')
    t=np.asarray(primitives['time_s'],float);n=len(t)
    gamma=np.asarray(primitives['observer_generalized'],float)
    interval=np.asarray(primitives['observer_interval_mean'],float)
    hint=np.asarray(primitives['intrinsic_normal_hint'],float)
    axes=np.asarray(primitives['patch_axis_world'],float)
    lower=np.asarray(primitives['patch_lower_m'],float);upper=np.asarray(primitives['patch_upper_m'],float)
    radius=float(np.asarray(primitives['radius_m']))
    if gamma.shape!=(n,9) or interval.shape!=(n,9) or hint.shape!=(n,3) or axes.shape!=(n,4,3):raise ValueError('Incorrect full-rate primitive shapes')
    if lower.shape!=(4,) or upper.shape!=(4,) or not np.isfinite(radius) or radius<=0:raise ValueError('Incorrect fixed patch metadata')
    if not np.isfinite(np.r_[lower,upper]).all() or np.any(lower<0) or np.any(lower>=upper) or np.any(upper>radius):raise ValueError('Invalid known spherical bands')
    # Also validates the timebase when no approach interval happens to be valid.
    backward_rate(t,np.zeros(n),np.ones(n,bool),sample_period_s,time_tolerance_s)
    delta=interval-gamma
    force=np.linalg.norm(gamma[:,:3],axis=1)
    joint=np.linalg.norm(delta[:,6:9],axis=1);force_delta=np.linalg.norm(delta[:,:3],axis=1);moment=np.linalg.norm(delta[:,3:6],axis=1)
    signal_valid=np.isfinite(gamma).all(axis=1)&np.isfinite(interval).all(axis=1)&np.isfinite(delta).all(axis=1)&np.isfinite(force)&(force>=minimum_force_N)
    normalized=[]
    for numerator,denominator in ((joint,radius*force),(force_delta,force),(moment,radius*force)):
        value=np.full(n,np.nan);np.divide(numerator,denominator,out=value,where=signal_valid);normalized.append(value)
    size=np.linalg.norm(hint,axis=1)
    axes_size=np.linalg.norm(axes,axis=2)
    geometry_valid=np.isfinite(hint).all(axis=1)&np.isfinite(size)&(size>1e-12)&np.isfinite(axes).all(axis=(1,2))
    geometry_valid&=np.all(np.abs(axes_size-1.)<=1e-10,axis=1)
    normal=np.full((n,3),np.nan);np.divide(hint,size[:,None],out=normal,where=geometry_valid[:,None])
    offset=-radius*normal
    axial=np.einsum('nkj,nj->nk',axes,offset)
    axial=np.where(geometry_valid[:,None],axial,np.nan)
    seam=np.r_[upper[:2],lower[2:]]
    axial_signed=axial-seam[None,:]
    # Signed shortest meridional surface distance to the known seam circle.
    # Positive is equatorward, negative poleward. Axial offsets alone compress
    # near a pole and would put an entire cap inside a 12-mm axial window.
    signed=radius*(np.arccos(np.clip(axial/radius,-1.,1.))-np.arccos(seam/radius)[None,:])
    distance=np.abs(signed)
    nearest=np.min(distance,axis=1)
    approach,approach_valid=backward_rate(t,nearest,geometry_valid,sample_period_s,time_tolerance_s)
    seam_derivatives=[backward_rate(t,signed[:,k],geometry_valid,sample_period_s,time_tolerance_s) for k in range(4)]
    signed_rate=np.stack([pair[0] for pair in seam_derivatives],axis=1)
    axial_rate=np.stack([backward_rate(t,axial_signed[:,k],geometry_valid,sample_period_s,time_tolerance_s)[0] for k in range(4)],axis=1)
    axial_nearest=np.min(np.abs(axial_signed),axis=1)
    axial_approach,axial_approach_valid=backward_rate(t,axial_nearest,geometry_valid,sample_period_s,time_tolerance_s)
    seam_derivative_valid=np.logical_and.reduce([pair[1] for pair in seam_derivatives])
    # Frozen engineering scales, not calibrated uncertainty or probabilities.
    joint_maxabs=np.max(np.abs(delta[:,6:9]),axis=1)
    innovation_scale=.03+(.10*radius)*force
    u=np.where(signal_valid,np.clip(joint_maxabs/innovation_scale,0.,1.),np.nan)
    seam_width=.012+.12*np.abs(signed_rate)
    per_seam_risk=np.clip(1.-np.abs(signed)/seam_width,0.,1.)
    s=np.where(seam_derivative_valid,np.max(per_seam_risk,axis=1),np.nan)
    risk_valid=signal_valid&seam_derivative_valid
    risk=np.where(risk_valid,np.maximum(u,s),np.nan)
    result=dict(time_s=t.copy(),force_norm_N=force,observer_innovation=delta,
        joint_innovation_norm_Nm=joint,force_innovation_norm_N=force_delta,moment_innovation_norm_Nm=moment,
        joint_innovation_normalized=normalized[0],force_innovation_normalized=normalized[1],moment_innovation_normalized=normalized[2],
        observer_signal_valid=signal_valid,raw_spherical_offset_m=offset,axial_coordinate_m=axial,
        lower_signed_margin_m=axial-lower[None,:],upper_signed_margin_m=upper[None,:]-axial,
        seam_axial_signed_offset_m=axial_signed,seam_axial_signed_rate_m_s=axial_rate,
        seam_axial_distance_m=np.abs(axial_signed),nearest_axial_seam_distance_m=axial_nearest,
        seam_axial_approach_rate_m_s=-axial_approach,seam_axial_approach_valid=axial_approach_valid,
        seam_signed_offset_m=signed,seam_distance_m=distance,nearest_seam_distance_m=nearest,
        seam_approach_rate_m_s=-approach,seam_approach_valid=approach_valid,geometry_valid=geometry_valid,
        seam_signed_rate_m_s=signed_rate,seam_derivative_valid=seam_derivative_valid,
        joint_innovation_maxabs_Nm=joint_maxabs,joint_innovation_scale_Nm=innovation_scale,
        joint_load_risk_u=u,seam_risk_width_m=seam_width,per_seam_risk=per_seam_risk,
        seam_risk_s=s,combined_risk_r=risk,risk_valid=risk_valid)
    if 'qvel' in primitives:
        velocity=np.asarray(primitives['qvel'],float)
        if velocity.shape!=(n,9):raise ValueError('Expected nine measured velocity components')
        result.update(base_speed_m_s=np.linalg.norm(velocity[:,:3],axis=1),
            base_angular_speed_rad_s=np.linalg.norm(velocity[:,3:6],axis=1),
            joint_speed_norm_rad_s=np.linalg.norm(velocity[:,6:9],axis=1))
    return result


class PatchAxisKinematics:
    """Known patch axes from a private model state; kinematics only."""
    def __init__(self,model):
        import mujoco
        self._mj=mujoco;self.model=model;self.data=mujoco.MjData(model)
        if model.nv!=9 or model.nuser_geom<5:raise ValueError('Expected original nine-velocity patch model')
        self.body=[];self.sign=[];self.lower=[];self.upper=[];radii=[]
        for name in PATCH_NAMES:
            geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name+'_smooth')
            if geom<0:raise ValueError('Missing known material patch '+name)
            sign,lower,upper,_,radius=map(float,model.geom_user[geom,:5])
            if sign not in (-1.,1.) or not np.isfinite([sign,lower,upper,radius]).all() or not 0<=lower<upper<=radius:raise ValueError('Invalid patch metadata')
            self.body.append(int(model.geom_bodyid[geom]));self.sign.append(sign);self.lower.append(lower);self.upper.append(upper);radii.append(radius)
        if not np.all(np.asarray(radii)==radii[0]):raise ValueError('Patch radii differ')
        self.radius_m=float(radii[0]);self.lower=np.asarray(self.lower);self.upper=np.asarray(self.upper);self.sign=np.asarray(self.sign)

    def axes(self,qpos):
        q=np.asarray(qpos,float)
        if q.shape!=(self.model.nq,):raise ValueError('Pose shape differs from known model')
        if not np.isfinite(q).all() or np.linalg.norm(q[3:7])<1e-12:return np.full((4,3),np.nan)
        self.data.qpos[:]=q;self.data.qpos[3:7]/=np.linalg.norm(q[3:7])
        self._mj.mj_kinematics(self.model,self.data)
        return np.asarray([sign*self.data.xmat[body].reshape(3,3)[:,0] for body,sign in zip(self.body,self.sign)])
