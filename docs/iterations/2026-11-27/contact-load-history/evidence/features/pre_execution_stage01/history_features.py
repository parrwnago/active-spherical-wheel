"""I-64 endpoint contact-load maps and causal NumPy observer diagnostics.

World force maps into the three scalar generalized joint torques. Only the
private endpoint-map generator imports MuJoCo; portable replay uses cached H.
"""
import numpy as np


class FrozenEndpointMaps:
    """Compute private point Jacobians at frozen I-61 inference points."""
    def __init__(self,model,load_core_module,geometry_module):
        self.core=load_core_module.ContactLoadFeatures(model,geometry_module=geometry_module)
        self.model=model;self.data=self.core.data;self._mj=self.core._mj
        self.radius_m=self.core.radius_m

    def point_map(self,qpos,point_m,family):
        """Assigned-body map, with physical-band validity reported separately.

        Native equivalent points are not silently projected to a material
        patch. Their finite assigned-body Jacobian is a virtual-point model,
        which can be geometrically invalid for that material family.
        """
        result=dict(H=np.full((3,3),np.nan),map_valid=False,material_point_valid=False)
        point=np.asarray(point_m,float);family=int(family)
        if point.shape!=(3,):raise ValueError('Expected a world point with three components')
        if family<0 or family>=4 or not np.isfinite(point).all() or not self.core._set_pose(qpos):return result
        patch=self.core.geometry.patches[family];offset=point-self.data.xpos[self.core.geometry.centre_body]
        axis=patch['sign']*self.data.xmat[patch['body']].reshape(3,3)[:,0]
        axial=float(axis@offset)
        material=bool(abs(np.linalg.norm(offset)-self.radius_m)<=1e-9 and patch['lower']-1e-9<=axial<=patch['upper']+1e-9)
        jacobian=np.zeros((3,self.model.nv))
        self._mj.mj_jac(self.model,self.data,jacobian,None,point,patch['body'])
        H=jacobian[:,6:9].T.copy()
        result.update(H=H,map_valid=bool(np.isfinite(H).all()),material_point_valid=material)
        return result


def _step(previous,value,dt,cutoff_rad_s):
    return previous+(1.-np.exp(-cutoff_rad_s*dt))*(value-previous)


def _history(time_s,gamma_joint,raw_force,anchor_H,anchor_valid,candidate_maps,candidate_valid,
             observation_valid,cutoff_rad_s,warmup_s,sample_period_s,time_tolerance_s):
    n=len(time_s);prior=np.full((n,3),np.nan);after=prior.copy();age=np.full(n,np.nan)
    seeded=np.zeros(n,bool);reset=np.zeros(n,bool);warm=np.zeros(n,bool);chain=np.zeros(n,bool)
    predictions={key:np.full((n,3),np.nan) for key in candidate_maps}
    scores_valid={key:np.zeros(n,bool) for key in candidate_maps}
    active=False;state=None;elapsed=0.
    for i in range(n):
        dt=float(time_s[i]-time_s[i-1]) if i else np.nan
        adjacent=bool(i and abs(dt-sample_period_s)<=time_tolerance_s)
        valid=bool(observation_valid[i] and anchor_valid[i])
        if not valid or (i and not adjacent):
            reset[i]=active;active=False;state=None;elapsed=0.;continue
        if not active:
            state=gamma_joint[i].copy();active=True;elapsed=0.;seeded[i]=True
            after[i]=state;age[i]=0.;chain[i]=True;continue
        prior[i]=state
        elapsed+=dt;age[i]=elapsed;warm[i]=elapsed>=warmup_s-time_tolerance_s;chain[i]=True
        for key,H in candidate_maps.items():
            if candidate_valid[key][i]:
                predictions[key][i]=_step(state,H[i]@raw_force[i],dt,cutoff_rad_s)
                scores_valid[key][i]=warm[i]
        state=_step(state,anchor_H[i]@raw_force[i],dt,cutoff_rad_s)
        after[i]=state
    output=dict(anchor_prior_Nm=prior,anchor_after_Nm=after,history_age_s=age,seeded=seeded,
        reset=reset,anchor_valid=warm,chain_active=chain,seed_weight=np.exp(-cutoff_rad_s*age))
    for key in candidate_maps:
        output[key+'_prediction_Nm']=predictions[key]
        output[key+'_residual_Nm']=gamma_joint-predictions[key]
        output[key+'_valid']=scores_valid[key]
    return output


def derive_history(primitives,cutoff_rad_s=100.,warmup_s=.2,sample_period_s=.005,time_tolerance_s=1e-9):
    """Frozen-A/N shadow histories; current alternatives cannot rewrite them.

    The endpoint map approximates an interval's contact geometry. Force enters
    once as the raw interval mean. Each history is observed-seeded, skips its
    seed interval, and requires 200 ms contiguous valid inputs before scoring.
    Unknown inputs/time gaps break the history; the next adjacent valid row
    starts a new measured seed. This is a model diagnostic, not actual routing.
    """
    if (not np.isfinite([cutoff_rad_s,warmup_s,sample_period_s,time_tolerance_s]).all()
            or cutoff_rad_s<=0 or warmup_s<0 or sample_period_s<=0 or time_tolerance_s<0):raise ValueError('Invalid fixed clock constants')
    t=np.asarray(primitives['time_s'],float);gamma=np.asarray(primitives['observer_generalized'],float)
    mean=np.asarray(primitives['observer_interval_mean'],float);n=len(t)
    if t.ndim!=1 or gamma.shape!=(n,9) or mean.shape!=(n,9):raise ValueError('Expected full-rate generalized observer histories')
    if not np.isfinite(t).all() or np.any(np.diff(t)<=0):raise ValueError('Strictly increasing finite timestamps required')
    radius=float(np.asarray(primitives['radius_m']))
    if not np.isfinite(radius) or radius<=0:raise ValueError('Positive known radius required')
    matrices={key:np.asarray(primitives['H_'+key],float) for key in ('F','Nalt','Neq')}
    valid={key:np.asarray(primitives['H_'+key+'_valid'],bool).copy() for key in matrices}
    for key,H in matrices.items():
        if H.shape!=(n,3,3) or valid[key].shape!=(n,):raise ValueError('Expected point-to-joint maps for each row')
        valid[key]&=np.isfinite(H).all(axis=(1,2))
    accepted=np.asarray(primitives['i60_accepted'],bool)
    if accepted.shape!=(n,):raise ValueError('Frozen acceptance history shape differs')
    filtered_force=gamma[:,:3];raw_force=mean[:,:3];target=gamma[:,6:9];raw_target=mean[:,6:9]
    force_norm=np.linalg.norm(filtered_force,axis=1)
    observed=np.isfinite(gamma).all(axis=1)&np.isfinite(mean).all(axis=1)&np.isfinite(force_norm)&(force_norm>=1.)
    adjacent=np.zeros(n,bool)
    if n>1:adjacent[1:]=np.abs(np.diff(t)-sample_period_s)<=time_tolerance_s
    recurrence=np.full((n,9),np.nan);reconstructed=np.full((n,9),np.nan)
    if n and np.isfinite(gamma[0]).all():reconstructed[0]=gamma[0]
    if n>1:
        for i in range(1,n):
            dt=float(t[i]-t[i-1])
            if adjacent[i]:recurrence[i]=gamma[i]-_step(gamma[i-1],mean[i],dt,cutoff_rad_s)
            if np.isfinite(reconstructed[i-1]).all() and np.isfinite(mean[i]).all():
                reconstructed[i]=_step(reconstructed[i-1],mean[i],dt,cutoff_rad_s)
    result=dict(time_s=t.copy(),filtered_force_norm_N=force_norm,radius_m=np.full(n,radius),
        observation_valid=observed,time_adjacent=adjacent,observer_recurrence_residual9=recurrence,
        observer_reconstructed9=reconstructed,observer_full_recurrence_error9=gamma-reconstructed)
    for key in ('F','Nalt'):
        H=matrices[key]
        filtered_prediction=np.einsum('nij,nj->ni',H,filtered_force)
        raw_prediction=np.einsum('nij,nj->ni',H,raw_force)
        result['M0_'+key+'_prediction_Nm']=filtered_prediction
        result['M0_'+key+'_residual_Nm']=target-filtered_prediction
        result['M0_'+key+'_valid']=valid[key]&observed
        result['M1_'+key+'_prediction_Nm']=raw_prediction
        result['M1_'+key+'_residual_Nm']=raw_target-raw_prediction
        result['M1_'+key+'_valid']=valid[key]&observed
        one_step=np.full((n,3),np.nan);scaled_raw=one_step.copy()
        for i in range(1,n):
            if adjacent[i] and observed[i-1] and observed[i] and valid[key][i]:
                dt=float(t[i]-t[i-1]);one_step[i]=target[i]-_step(target[i-1],raw_prediction[i],dt,cutoff_rad_s)
                scaled_raw[i]=(1.-np.exp(-cutoff_rad_s*dt))*(raw_target[i]-raw_prediction[i])
        result['one_step_'+key+'_residual_Nm']=one_step
        result['one_step_'+key+'_scaled_raw_residual_Nm']=scaled_raw
        result['one_step_'+key+'_identity_error_Nm']=one_step-scaled_raw
    HA=np.where(accepted[:,None,None],matrices['F'],matrices['Neq'])
    validA=np.where(accepted,valid['F'],valid['Neq'])
    for model,Hanchor,anchor_valid in [('M2',HA,validA),('M3',matrices['Neq'],valid['Neq'])]:
        history=_history(t,target,raw_force,Hanchor,anchor_valid,
            {key:matrices[key] for key in ('F','Nalt')},{key:valid[key] for key in ('F','Nalt')},
            observed,cutoff_rad_s,warmup_s,sample_period_s,time_tolerance_s)
        result.update({model+'_'+key:value for key,value in history.items()})
        result[model+'_committed_residual_Nm']=target-history['anchor_after_Nm']
        inherited=np.full((n,3),np.nan)
        alpha=np.full(n,np.nan)
        if n>1:
            dt=np.diff(t);lam=np.exp(-cutoff_rad_s*dt);alpha[1:]=1.-lam
            inherited[1:]=lam[:,None]*(target[:-1]-history['anchor_prior_Nm'][1:])
        result[model+'_inherited_Nm']=inherited
        inherited_norm=np.linalg.norm(inherited,axis=1)
        for key in ('F','Nalt'):
            current=alpha[:,None]*result['M1_'+key+'_residual_Nm']
            current_norm=np.linalg.norm(current,axis=1);total_norm=inherited_norm+current_norm
            inherited_share=np.full(n,np.nan);current_share=np.full(n,np.nan)
            np.divide(inherited_norm,total_norm,out=inherited_share,where=np.isfinite(total_norm)&(total_norm>0))
            np.divide(current_norm,total_norm,out=current_share,where=np.isfinite(total_norm)&(total_norm>0))
            result[model+'_'+key+'_current_increment_Nm']=current
            result[model+'_'+key+'_decomposition_error_Nm']=result[model+'_'+key+'_residual_Nm']-inherited-current
            result[model+'_'+key+'_inherited_norm_share']=inherited_share
            result[model+'_'+key+'_current_norm_share']=current_share
    return result
