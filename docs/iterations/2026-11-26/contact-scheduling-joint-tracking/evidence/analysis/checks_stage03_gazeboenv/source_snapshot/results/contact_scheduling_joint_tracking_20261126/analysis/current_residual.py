"""I52 introduced rolling-reference residual, private CURRENT-pose geometry.

Only mj_kinematics/mj_comPos/mj_jac are allowed. No integration, forward solve,
observer, allocation/controller call, contact solver query or forecast occurs.
The inputs are saved ESTIMATED support points, never true contact locations.
"""
from contextlib import ExitStack
from unittest.mock import patch
import numpy as np

LABELS=('shell_A','shell_B','belt_A','belt_B')


def residual_vectors(matrix,rhs,native_drive,used_joint_refs):
    matrix=np.asarray(matrix,float);rhs=np.asarray(rhs,float);native=np.asarray(native_drive,float)
    used=native.copy();used[:3]=used_joint_refs
    return matrix@native-rhs,matrix@used-rhs


def summarize(output,case,raw,rollout,model_path):
    if model_path is None:
        return dict(available=False,reason='Own executed mirror XML/model assets unavailable or ambiguous; no substitute geometry or actual-motion consistency residual is used.')
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model)
    geometries=[]
    for label in LABELS:
        geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,label+'_smooth')
        if geom<0:return dict(available=False,reason='Missing known spherical-patch metadata geometry: '+label)
        geometries.append((geom,int(model.geom_bodyid[geom])))
    t=raw['time_s'];modified=np.any(raw['exact_reference_guard_delta']!=0,axis=1)
    quality=np.zeros(len(t),np.int8);native_norm=np.full(len(t),np.nan);used_norm=native_norm.copy()
    def forbidden(*args,**kwargs):raise AssertionError('Only current kinematics allowed in rolling-reference residual')
    with ExitStack()as stack:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_integratePos','mj_contactForce'):
            stack.enter_context(patch.object(mujoco,name,forbidden))
        for i in np.flatnonzero(modified):
            quality[i]=-1;loaded=np.flatnonzero(raw['exact_estimated_support_loads_N'][i]>0)
            if not len(loaded):continue
            data.qpos[:]=raw['exact_qpos'][i];mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
            rows=[]
            for j in loaded:
                geom,body=geometries[j]
                point=raw['exact_estimated_contact_point_m'][i]if len(loaded)==1 else rollout['estimated_contact_points_m'][i,j].astype(float)
                if not np.isfinite(point).all():break
                jac=np.zeros((3,model.nv));mujoco.mj_jac(model,data,jac,None,point,body)
                row=np.zeros((2,5));row[:,:3]=jac[:2,6:9]
                if j>=2:row[:,j+1]=data.geom_xmat[geom].reshape(3,3)[:2,1]
                rows.extend(row)
            else:
                matrix=np.asarray(rows);rhs=np.tile(-raw['exact_reference_velocity_m_s'][i],len(loaded))
                native=np.r_[raw['exact_native_joint_reference'][i],raw['exact_rolling_feedforward'][i,3:]]
                a,b=residual_vectors(matrix,rhs,native,raw['exact_guarded_joint_reference'][i])
                native_norm[i]=np.linalg.norm(a);used_norm[i]=np.linalg.norm(b);quality[i]=1 if len(loaded)==1 else 2
    held=np.arange(len(t))<len(t)-1;eligible=modified&held&(quality>0)
    np.savez_compressed(output/(case+'_introduced_reference_residual.npz'),time_s=t,reference_modified=modified,
        native_residual_norm_m_s=native_norm,used_residual_norm_m_s=used_norm,point_quality=quality)
    return dict(available=True,scope=__doc__,model_path=str(model_path),
        point_quality_codes={'0':'reference unchanged; not evaluated','-1':'modified but current estimated geometry unavailable','1':'modified, exact single-estimated-patch global point','2':'modified, plural estimated per-patch points encodedfloat32'},
        modified_all_emission_samples=int(modified.sum()),modified_held_samples=int(np.sum(modified&held)),
        usable_modified_held_samples=int(eligible.sum()),unavailable_modified_held_samples=int(np.sum(modified&held&(quality<0))),
        exact_single_modified_held_samples=int(np.sum(held&(quality==1))),plural_float32_modified_held_samples=int(np.sum(held&(quality==2))),
        rms_native_on_same_modified_held_m_s=float(np.sqrt(np.mean(native_norm[eligible]**2)))if eligible.any()else None,
        rms_used_on_same_modified_held_m_s=float(np.sqrt(np.mean(used_norm[eligible]**2)))if eligible.any()else None,
        max_used_on_modified_held_m_s=float(used_norm[eligible].max())if eligible.any()else None,
        interpretation='Unweighted native XY no-slip row convention, unchanged belt feedforward, current estimated geometry. This is introduced REFERENCE kinematic mismatch, not actual motion consistency, predicted slip, torque feasibility or closed-loop tracking.')


def arithmetic_checks():
    matrix=np.array([[1.,0.,0.,0.,0.],[0.,0.,1.,0.,1.]])
    a,b=residual_vectors(matrix,[2.,2.],[2.,0.,3.,0.,-1.],[1.,0.,2.])
    assert np.array_equal(a,[0.,0.])and np.array_equal(b,[-1.,-1.])
    return dict(passed=True,scope='One independent vector fixture checks used jointreferences and unchanged belt feedforward; actual motion is not an input.')
