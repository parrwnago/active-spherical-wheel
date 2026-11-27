"""Load-free private pose maps for I-66 geometry/model diagnosis.

All maps act on world force/moment and produce scalar joint torques. Raw
sampled family mean points may be virtual; material flags never project them.
"""
import numpy as np


def skew(vector):
    value=np.asarray(vector,float)
    if value.ndim<1 or value.shape[-1]!=3:raise ValueError('Expected vectors ending in three components')
    out=np.zeros(value.shape[:-1]+(3,3),float)
    out[...,0,1]=-value[...,2];out[...,0,2]=value[...,1]
    out[...,1,0]=value[...,2];out[...,1,2]=-value[...,0]
    out[...,2,0]=-value[...,1];out[...,2,1]=value[...,0]
    return out


def family_point_maps(pose,points_m):
    """Map each point on its own family, accepting finite raw mean points.

    H=L+A*skew(p-centre). Material validity is descriptive only: weighted
    means can lie inside the sphere. Missing points remain individually unknown.
    """
    centre=np.asarray(pose['centre_m'],float);points=np.asarray(points_m,float)
    L=np.asarray(pose['linear_at_centre'],float);A=np.asarray(pose['angular_world'],float)
    if centre.shape[-1:]!=(3,) or points.shape!=centre.shape[:-1]+(4,3) or L.shape!=centre.shape[:-1]+(4,3,3) or A.shape!=L.shape:raise ValueError('Expected matched pose and four-family point shapes')
    offset=points-centre[...,None,:];H=L+np.matmul(A,skew(offset))
    valid=np.asarray(pose['valid'],bool)[...,None]&np.isfinite(points).all(axis=-1)&np.isfinite(H).all(axis=(-2,-1))
    axis=np.asarray(pose['patch_axis_world'],float)
    lower=np.asarray(pose['patch_lower_m'],float);upper=np.asarray(pose['patch_upper_m'],float)
    radius=np.asarray(pose['radius_m'],float)
    radial=np.linalg.norm(offset,axis=-1);axial=np.sum(axis*offset,axis=-1)
    if radius.ndim:radius=radius[...,None]
    material=valid&(np.abs(radial-radius)<=1e-9)&(axial>=lower-1e-9)&(axial<=upper+1e-9)
    return dict(H=np.where(valid[...,None,None],H,np.nan),valid=valid,material_valid=material,
        radius_error_m=radial-radius,axial_coordinate_m=axial)


def assigned_point_maps(pose,points_m,family_index):
    """Choose the explicitly supplied family; unknown family is never guessed."""
    centre=np.asarray(pose['centre_m'],float);points=np.asarray(points_m,float);family=np.asarray(family_index)
    if points.shape!=centre.shape or family.shape!=centre.shape[:-1] or not np.issubdtype(family.dtype,np.integer):raise ValueError('Expected one explicitly assigned family per point')
    expanded=np.broadcast_to(points[...,None,:],centre.shape[:-1]+(4,3))
    all_maps=family_point_maps(pose,expanded);safe=np.clip(family,0,3)
    output={}
    for key,value in all_maps.items():
        if key=='H':chosen=np.take_along_axis(value,safe[...,None,None,None],axis=-3)[...,0,:,:]
        else:chosen=np.take_along_axis(value,safe[...,None],axis=-1)[...,0]
        output[key]=chosen
    good=(family>=0)&(family<4);output['valid']=output['valid']&good;output['material_valid']=output['material_valid']&good
    output['H']=np.where(output['valid'][...,None,None],output['H'],np.nan)
    return output


class EndpointPoseMaps:
    """Current-pose Jacobians only; no force, contacts or dynamics are inputs."""
    def __init__(self,model,load_core_module,geometry_module):
        self.core=load_core_module.ContactLoadFeatures(model,geometry_module=geometry_module)
        self.model=model;self.data=self.core.data;self._mj=self.core._mj
        self.radius_m=self.core.radius_m;self.patches=self.core.geometry.patches
        self.lower=np.array([p['lower'] for p in self.patches]);self.upper=np.array([p['upper'] for p in self.patches])
        joints=[]
        for dof in range(6,9):
            found=np.flatnonzero(np.asarray(model.jnt_dofadr)==dof)
            if len(found)!=1 or model.jnt_type[found[0]]!=self._mj.mjtJoint.mjJNT_HINGE:raise ValueError('Expected three scalar hinge coordinates')
            joints.append(int(found[0]))
        self.joints=joints;self.active=np.zeros((4,3),bool)
        for family,patch in enumerate(self.patches):
            for column,joint in enumerate(joints):
                body=int(patch['body']);joint_body=int(model.jnt_bodyid[joint])
                while body>0:
                    if body==joint_body:self.active[family,column]=True;break
                    body=int(model.body_parentid[body])

    def evaluate(self,qpos):
        empty=dict(centre_m=np.full(3,np.nan),linear_at_centre=np.full((4,3,3),np.nan),
            angular_world=np.full((4,3,3),np.nan),valid=False,radius_m=np.asarray(self.radius_m),
            patch_axis_world=np.full((4,3),np.nan),patch_lower_m=self.lower.copy(),patch_upper_m=self.upper.copy(),
            hinge_axis_world=np.full((3,3),np.nan),hinge_anchor_m=np.full((3,3),np.nan),ancestor_active=self.active.copy())
        if not self.core._set_pose(qpos):return empty
        centre=self.data.xpos[self.core.geometry.centre_body].copy();L=np.empty((4,3,3));A=np.empty((4,3,3));axes=np.empty((4,3))
        for family,patch in enumerate(self.patches):
            linear=np.zeros((3,self.model.nv));angular=np.zeros((3,self.model.nv))
            self._mj.mj_jac(self.model,self.data,linear,angular,centre,patch['body'])
            L[family]=linear[:,6:9].T;A[family]=angular[:,6:9].T
            axes[family]=patch['sign']*self.data.xmat[patch['body']].reshape(3,3)[:,0]
        empty.update(centre_m=centre,linear_at_centre=L,angular_world=A,patch_axis_world=axes,
            hinge_axis_world=self.data.xaxis[self.joints].copy(),hinge_anchor_m=self.data.xanchor[self.joints].copy(),valid=True)
        return empty


def analytical_family_point_maps(pose,points_m):
    """Independent hinge-row H_j = axis_j cross (p-anchor_j), or zero."""
    points=np.asarray(points_m,float);centre=np.asarray(pose['centre_m'],float)
    if points.shape!=centre.shape[:-1]+(4,3):raise ValueError('Expected one point per known family')
    axis=np.asarray(pose['hinge_axis_world'],float);anchor=np.asarray(pose['hinge_anchor_m'],float)
    active=np.asarray(pose['ancestor_active'],bool)
    H=np.cross(axis[...,None,:,:],points[..., :,None,:]-anchor[...,None,:,:])
    H=np.where(active[...,None],H,0.)
    valid=np.asarray(pose['valid'],bool)[...,None]&np.isfinite(points).all(axis=-1)&np.isfinite(H).all(axis=(-2,-1))
    return dict(H=np.where(valid[...,None,None],H,np.nan),valid=valid)
