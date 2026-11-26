"""Static I55 momentum/force accounting. No dynamics or controller evaluation."""
from contextlib import ExitStack
from unittest.mock import patch
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

NAMES = ('fork', 'carrier', 'shell_A', 'shell_B')
TAG_BODY = np.array([2, 3, 1, 1, 2, 3, 0], dtype=int)
STATE_FIELDS = ('time', 'iterations', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'q_shaft', 'q_A', 'q_B', 'dq_shaft', 'dq_A', 'dq_B', 'step_s', 'resets', 'reset_pending')


def rotation(q):
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w,x,y,z = np.moveaxis(q, -1, 0)
    r = np.empty(q.shape[:-1]+(3,3))
    r[...,0,0]=1-2*(y*y+z*z);r[...,0,1]=2*(x*y-z*w);r[...,0,2]=2*(x*z+y*w)
    r[...,1,0]=2*(x*y+z*w);r[...,1,1]=1-2*(x*x+z*z);r[...,1,2]=2*(y*z-x*w)
    r[...,2,0]=2*(x*z-y*w);r[...,2,1]=2*(y*z+x*w);r[...,2,2]=1-2*(x*x+y*y)
    return r


def state_qv(states):
    s = np.asarray(states, float)
    q = np.column_stack([s[:,2:5], s[:,5:9]/np.linalg.norm(s[:,5:9],axis=1)[:,None], s[:,15:18]])
    r = rotation(q[:,3:7])
    v = np.column_stack([s[:,9:12], np.einsum('nji,nj->ni',r,s[:,12:15]), s[:,18:21]])
    return q,v


def forbidden(*args,**kwargs):
    raise AssertionError('No dynamics, integration, forward, or contact API allowed')


def check_inertias(model, sdf_path):
    links={n.attrib['name']:n for n in ET.parse(sdf_path).getroot().findall('./model/link')}
    checks={}
    for name in NAMES:
        b=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
        assert b>0
        node=links[name].find('inertial');pose=np.fromstring(node.findtext('pose'),sep=' ')
        assert pose.shape==(6,) and np.array_equal(pose[3:],np.zeros(3))
        inertia=node.find('inertia')
        sdfI=np.array([[float(inertia.findtext('ixx')),float(inertia.findtext('ixy')),float(inertia.findtext('ixz'))],[float(inertia.findtext('ixy')),float(inertia.findtext('iyy')),float(inertia.findtext('iyz'))],[float(inertia.findtext('ixz')),float(inertia.findtext('iyz')),float(inertia.findtext('izz'))]])
        r=rotation(model.body_iquat[b]);modelI=r@np.diag(model.body_inertia[b])@r.T
        gaps={'mass_kg':abs(float(node.findtext('mass'))-model.body_mass[b]),'COM_m':np.max(abs(pose[:3]-model.body_ipos[b])),'inertia_kg_m2':np.max(abs(sdfI-modelI))}
        assert max(gaps.values())<1e-10
        checks[name]={'mass_kg':model.body_mass[b],'COM_m':model.body_ipos[b].copy(),'body_inertia_kg_m2':modelI,'max_differences':gaps}
    return checks


def momenta(model_path, sdf_path, states):
    """Copy of the verified I54 rigid-body accounting, with explicit frames."""
    model=mujoco.MjModel.from_xml_path(str(Path(model_path)))
    data=mujoco.MjData(model)
    inertia_checks=check_inertias(model,sdf_path)
    ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n)for n in NAMES]
    assert model.nq==10 and model.nv==9
    masses=model.body_mass[ids].copy();mass=masses.sum()
    q,v=state_qv(states);n=len(q)
    positions=np.empty((n,4,3));velocities=np.empty_like(positions);omega=np.empty_like(positions);spin=np.empty_like(positions);shaftaxis=np.empty((n,3));shellaxis=np.empty((n,3));max_chain_gap=0.
    with ExitStack()as stack:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_fwdPosition','mj_fwdVelocity','mj_fwdAcceleration','mj_integratePos','mj_contactForce'):
            stack.enter_context(patch.object(mujoco,name,forbidden))
        for i in range(n):
            data.qpos[:]=q[i];data.qvel[:]=v[i]
            mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
            for j,b in enumerate(ids):
                jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBodyCom(model,data,jp,jr,b)
                positions[i,j]=data.xipos[b];velocities[i,j]=jp@v[i];omega[i,j]=jr@v[i]
                r=data.ximat[b].reshape(3,3)
                spin[i,j]=(r@np.diag(model.body_inertia[b])@r.T)@omega[i,j]
            rf=rotation(q[i,3:7]);shaftaxis[i]=rf[:,1];shellaxis[i]=data.xmat[ids[1]].reshape(3,3)[:,0]
            direct=np.empty((4,3));direct[0]=rf@v[i,3:6];direct[1]=direct[0]+v[i,6]*shaftaxis[i];direct[2]=direct[1]+v[i,7]*shellaxis[i];direct[3]=direct[1]+v[i,8]*shellaxis[i]
            max_chain_gap=max(max_chain_gap,float(np.max(abs(direct-omega[i]))))
    assert max_chain_gap<1e-10
    com=np.einsum('j,njk->nk',masses,positions)/mass
    vc=np.einsum('j,njk->nk',masses,velocities)/mass
    orbital=masses[None,:,None]*np.cross(positions-com[:,None,:],velocities-vc[:,None,:])
    H=spin+orbital
    return {'qpos':q,'qvel':v,'H_by_body_world':H,'H_total_world':H.sum(axis=1),'spin_H_by_body_world':spin,'orbital_H_by_body_world':orbital,'systemCOM_world_m':com,'systemCOM_velocity_world_m_s':vc,'bodyCOM_world_m':positions,'bodyCOM_velocity_world_m_s':velocities,'body_masses_kg':masses,'gravity_world_m_s2':model.opt.gravity.copy(),'omega_by_body_world_rad_s':omega,'shaft_axis_world':shaftaxis,'shell_axis_world':shellaxis,'mass_kg':mass,'inertia_checks':inertia_checks,'max_chain_omega_gap_rad_s':max_chain_gap}


def contact_moments(contact_rows, offsets, centers):
    """All exported solver force rows, no admission/positive-load exclusion."""
    c=np.asarray(contact_rows,float);o=np.asarray(offsets,np.int64);centers=np.asarray(centers,float)
    assert c.ndim==2 and c.shape[1]==16 and len(o)==len(centers)+1
    assert o[0]==0 and o[-1]==len(c) and np.all(np.diff(o)>=0)
    assert np.isfinite(c).all() and np.isfinite(centers).all()
    tag=c[:,0].astype(int);assert np.array_equal(tag,c[:,0]) and np.all((tag>=0)&(tag<=6))
    row_step=np.repeat(np.arange(len(centers)),np.diff(o))
    torque=np.cross(c[:,1:4]-centers[row_step],c[:,7:10])
    out=np.zeros((len(centers),4,3));np.add.at(out,(row_step,TAG_BODY[tag]),torque)
    force=np.zeros((len(centers),4,3));np.add.at(force,(row_step,TAG_BODY[tag]),c[:,7:10])
    admitted=c[:,12]>=.5
    admitted_torque=np.zeros_like(out);np.add.at(admitted_torque,(row_step[admitted],TAG_BODY[tag[admitted]]),torque[admitted])
    normal_force=np.einsum('ij,ij->i',c[:,7:10],c[:,4:7])
    a_load=np.zeros(len(centers));a_force=np.zeros(len(centers));body_force=np.zeros(len(centers));rejected_force=np.zeros(len(centers))
    a=np.isin(tag,[0,4]);np.add.at(a_load,row_step[a],np.maximum(normal_force[a],0));np.add.at(a_force,row_step[a],np.linalg.norm(c[a,7:10],axis=1))
    b=tag==6;np.add.at(body_force,row_step[b],np.linalg.norm(c[b,7:10],axis=1));np.add.at(rejected_force,row_step[~admitted],np.linalg.norm(c[~admitted,7:10],axis=1))
    return {'torque_by_body_world_Nm':out,'force_by_body_world_N':force,'admitted_torque_by_body_world_Nm':admitted_torque,'shellA_positive_normal_load_N':a_load,'shellA_sum_force_norm_N':a_force,'body_sum_force_norm_N':body_force,'rejected_sum_force_norm_N':rejected_force,'all_rows_count':len(c),'rejected_rows_count':int((~admitted).sum())}


def closure(delta_hz, impulse):
    residual=float(impulse-delta_hz)
    tolerance=max(1e-4,.1*abs(float(delta_hz)))
    return {'state_delta_Hz_kg_m2_s':float(delta_hz),'contact_yaw_impulse_Nm_s':float(impulse),'contact_minus_delta_Hz_Nm_s':residual,'desired_abs_residual_tolerance_Nm_s':tolerance,'comparison_roundoff_allowance_Nm_s':2e-15,'passes_desired_closure':abs(residual)<=tolerance+2e-15}


def heading(qpos,worldomega):
    r=rotation(np.asarray(qpos)[:,3:7]);x=r[:,:,0];h2=np.sum(x[:,:2]**2,axis=1);valid=h2>.04
    angle=np.full(len(x),np.nan);ids=np.flatnonzero(valid)
    for ii in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
        if len(ii):angle[ii]=np.unwrap(np.arctan2(x[ii,1],x[ii,0]))
    xd=np.cross(worldomega,x);rate=(x[:,0]*xd[:,1]-x[:,1]*xd[:,0])/np.maximum(h2,1e-30);rate[~valid]=np.nan
    return angle,rate,valid
