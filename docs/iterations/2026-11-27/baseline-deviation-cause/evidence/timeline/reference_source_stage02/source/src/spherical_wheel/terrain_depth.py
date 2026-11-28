"""Ideal four-camera range adapter for offline terrain-memory experiments.

Only this scene adapter and an evaluator may access geometry. A DepthFrame
contains depth, calibration, capture pose and times; no hit IDs or normals.
Environment geometry follows Gazebo collision triangles, while robot occlusion
uses the declared visible CAD/fork/belt model. No dynamics is integrated.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import copy
import hashlib
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def _readonly(value,shape=None,dtype=float):
    result=np.array(value,dtype=dtype,copy=True)
    if shape is not None and result.shape!=shape:
        raise ValueError(f'Expected array shape {shape}, got {result.shape}')
    result.setflags(write=False)
    return result


def _transform(value):
    value=_readonly(value,(4,4))
    if not np.isfinite(value).all() or not np.allclose(value[3],[0,0,0,1],atol=1e-12,rtol=0):
        raise ValueError('A finite homogeneous transform is required')
    if not np.allclose(value[:3,:3].T@value[:3,:3],np.eye(3),atol=1e-10,rtol=0) or np.linalg.det(value[:3,:3])<0:
        raise ValueError('Transform rotation must be proper and orthonormal')
    return value


def _intrinsics(value):
    value=_readonly(value,(3,3))
    if (not np.isfinite(value).all() or value[0,0]<=0 or value[1,1]<=0 or
            not np.allclose(value[[0,1,2,2,2],[1,0,0,1,2]],[0,0,0,0,1],atol=1e-12,rtol=0)):
        raise ValueError('Finite pinhole intrinsics with positive focal lengths are required')
    return value


@dataclass(frozen=True)
class CameraSpec:
    name: str
    width: int
    height: int
    intrinsics: np.ndarray
    body_from_camera: np.ndarray
    min_range_m: float = .08
    max_range_m: float = 2.

    def __post_init__(self):
        if not isinstance(self.name,str) or not self.name:
            raise ValueError('Camera name must be nonempty')
        if any(isinstance(v,bool) or not isinstance(v,(int,np.integer)) or v<1 for v in (self.width,self.height)):
            raise ValueError('Image dimensions must be positive integers')
        if not np.isfinite([self.min_range_m,self.max_range_m]).all() or not 0<=self.min_range_m<self.max_range_m:
            raise ValueError('Euclidean range must satisfy 0 <= min < max')
        object.__setattr__(self,'intrinsics',_intrinsics(self.intrinsics))
        object.__setattr__(self,'body_from_camera',_transform(self.body_from_camera))


@dataclass(frozen=True)
class DepthFrame:
    camera_name: str
    depth_m: np.ndarray
    valid_mask: np.ndarray
    intrinsics: np.ndarray
    body_from_camera: np.ndarray
    world_from_camera: np.ndarray
    capture_time_s: float
    delivered_at_s: float

    def __post_init__(self):
        depth=_readonly(self.depth_m);valid=_readonly(self.valid_mask,dtype=bool)
        if depth.ndim!=2 or depth.shape!=valid.shape:
            raise ValueError('Depth and validity must be matching two-dimensional arrays')
        if np.any(valid & (~np.isfinite(depth) | (depth<=0))) or not np.isnan(depth[~valid]).all():
            raise ValueError('Valid axial depths must be positive; invalid depths must be NaN')
        if not np.isfinite([self.capture_time_s,self.delivered_at_s]).all() or not 0<=self.capture_time_s<=self.delivered_at_s:
            raise ValueError('Capture and delivery times must be finite, nonnegative and causal')
        object.__setattr__(self,'depth_m',depth);object.__setattr__(self,'valid_mask',valid)
        object.__setattr__(self,'intrinsics',_intrinsics(self.intrinsics))
        object.__setattr__(self,'body_from_camera',_transform(self.body_from_camera))
        object.__setattr__(self,'world_from_camera',_transform(self.world_from_camera))


def four_camera_rig(width=64,height=48,horizontal_fov_deg=100.,pitch_down_deg=45.,
                    min_range_m=.08,max_range_m=2.):
    """Four outward/downward fork-mounted pinholes; optical x right,y down,z forward."""
    if not np.isfinite([horizontal_fov_deg,pitch_down_deg]).all() or not 0<horizontal_fov_deg<180 or not 0<=pitch_down_deg<90:
        raise ValueError('Finite FOV and downward pitch within supported bounds are required')
    focal=width/(2*np.tan(np.radians(horizontal_fov_deg)/2))
    k=np.array([[focal,0.,(width-1)/2],[0.,focal,(height-1)/2],[0.,0.,1.]])
    pitch=np.radians(pitch_down_deg);rig=[]
    origins=((.105,0.,.235),(0.,.135,.235),(-.105,0.,.235),(0.,-.135,.235))
    for yaw_deg,origin in zip((0,90,180,270),origins):
        yaw=np.radians(yaw_deg);c,s=np.cos(yaw),np.sin(yaw)
        right=np.array([s,-c,0.]);down=np.array([-np.sin(pitch)*c,-np.sin(pitch)*s,-np.cos(pitch)])
        forward=np.array([np.cos(pitch)*c,np.cos(pitch)*s,-np.sin(pitch)])
        transform=np.eye(4);transform[:3,:3]=np.column_stack([right,down,forward]);transform[:3,3]=origin
        rig.append(CameraSpec(f'camera_{yaw_deg:03d}',width,height,k,transform,min_range_m,max_range_m))
    return tuple(rig)


def _sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _read_obj(path):
    vertices=[];faces=[]
    for line in Path(path).read_text().splitlines():
        if line.startswith('v '):vertices.append([float(v)for v in line.split()[1:4]])
        elif line.startswith('f '):
            indices=[int(v.split('/')[0])for v in line.split()[1:]]
            indices=[i-1 if i>0 else len(vertices)+i for i in indices]
            faces.extend((indices[0],indices[i],indices[i+1])for i in range(1,len(indices)-1))
    return np.asarray(vertices,float),np.asarray(faces,int).reshape(-1,3)


def _crop_mesh(vertices,faces,bounds):
    triangles=vertices[faces];x0,x1,y0,y1=bounds
    low,high=triangles.min(axis=1),triangles.max(axis=1)
    keep=(high[:,0]>=x0)&(low[:,0]<=x1)&(high[:,1]>=y0)&(low[:,1]<=y1)
    selected=faces[keep]
    if not len(selected):return np.empty((0,3)),np.empty((0,3),int)
    used,inverse=np.unique(selected,return_inverse=True)
    return vertices[used].copy(),inverse.reshape(-1,3)


def _write_obj(path,vertices,faces):
    with Path(path).open('w')as stream:
        np.savetxt(stream,vertices,fmt='v %.17g %.17g %.17g')
        np.savetxt(stream,faces+1,fmt='f %d %d %d')


class TerrainRayScene:
    """Private optical/evaluator ray scene. Exposing it to the mapper violates the API."""
    def __init__(self,model,environment_geom_ids,robot_geom_ids=(),metadata=None):
        self.model=model;self.data=mujoco.MjData(model)
        self._environment=np.asarray(environment_geom_ids,int);self._robot=np.asarray(robot_geom_ids,int)
        all_ids=np.r_[self._environment,self._robot]
        if (np.any(all_ids<0)or np.any(all_ids>=model.ngeom)or len(np.unique(all_ids))!=len(all_ids)):
            raise ValueError('Environment/robot geometry IDs must be valid and disjoint')
        self.model.geom_group[:]=5
        self.model.geom_group[self._environment]=0;self.model.geom_group[self._robot]=1
        self.model.geom_rgba[all_ids,3]=1.
        self.model.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self._fork=int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'fork'))
        if self._fork<0:self._fork=0
        self.metadata=copy.deepcopy(metadata or {})
        self.metadata.update(backend='MuJoCo mj_multiRay/mj_ray triangle first-hit; no dynamics',mujoco_version=mujoco.__version__,
            environment_geom_count=len(self._environment),robot_occluder_geom_count=len(self._robot),
            numerical_geometry='Source triangle coordinates preserved in assets; MuJoCo compiled mesh vertices use float32. Face normals, never OBJ smooth vn.')
        self.set_qpos(self.data.qpos.copy())

    @classmethod
    def from_xml_string(cls,xml,environment_geom_names,robot_geom_names=()):
        """Small fixture constructor; names classify private hit IDs, never output frames."""
        model=mujoco.MjModel.from_xml_string(xml)
        ids=lambda names:[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name)for name in names]
        return cls(model,ids(environment_geom_names),ids(robot_geom_names),{'scene':'explicit_fixture'})

    @classmethod
    def from_mars(cls,config,directory,bounds_xy,dem_obj=None):
        """Collision DEM/feature faces plus visible robot CAD; never a rendered heightmap.

        bounds_xy must cover all intended ray endpoints/origins. Callers expand
        the declared robot footprint by max range plus mount margin. Selection
        retains every triangle whose AABB intersects the crop, including triangles
        with no vertices inside it. The same crop is used for every frame.
        """
        from .model import generate_model
        from .gazebo.mars import MarsYard,write_dem_mesh,YARD,FEATURES
        bounds=np.asarray(bounds_xy,float)
        if bounds.shape!=(4,)or not np.isfinite(bounds).all()or not(bounds[0]<bounds[1]and bounds[2]<bounds[3]):
            raise ValueError('Crop requires finite xmin<xmax and ymin<ymax')
        directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=True)
        target=directory/'depth_scene.xml'
        if target.exists():raise FileExistsError('Preserve prior scene assets; choose a fresh directory')
        robot_path=generate_model(config,directory/'robot')
        tree=ET.parse(robot_path);root=tree.getroot();world=root.find('worldbody');asset=root.find('asset')
        for tag in ('actuator','sensor','keyframe'):
            node=root.find(tag)
            if node is not None:root.remove(node)
        for body in world.iter():
            for geom in list(body.findall('geom')):
                group=int(geom.get('group','0'))
                if body is world or group in (3,5):body.remove(geom)
                else:
                    geom.set('group','1');geom.set('contype','0');geom.set('conaffinity','0')
        used_meshes={geom.get('mesh')for geom in world.iter('geom')if geom.get('mesh')}
        for mesh in list(asset.findall('mesh')):
            if mesh.get('name')not in used_meshes:asset.remove(mesh)
            else:mesh.set('file',str((robot_path.parent/mesh.get('file')).resolve()))
        if dem_obj is None:
            dem_obj=directory/'source_dem.obj';write_dem_mesh(MarsYard(),dem_obj)
        dem_obj=Path(dem_obj).resolve()
        source_meshes=[('dem',dem_obj)]+[(name,YARD/'meshes'/f'{name}.obj')for name,_ in FEATURES]
        source_records=[];scene_records=[];yaw=1.5707963268;cosine,sine=np.cos(yaw),np.sin(yaw)
        offsets=dict(FEATURES)
        for name,source in source_meshes:
            vertices,faces=_read_obj(source)
            source_records.append({'name':name,'path':str(source),'sha256':_sha(source),'vertices':len(vertices),'triangles':len(faces)})
            if name!='dem':
                vertices=np.c_[cosine*vertices[:,0]-sine*vertices[:,1],sine*vertices[:,0]+cosine*vertices[:,1],vertices[:,2]+offsets[name]]
            vertices,faces=_crop_mesh(vertices,faces,bounds)
            if not len(faces):continue
            if len(vertices)<4:raise ValueError(f'Crop {name} has fewer than four mesh vertices; preserve this unsupported setup')
            path=directory/f'{name}_collision_crop.obj';_write_obj(path,vertices,faces)
            mesh_name='depth_environment_'+name
            ET.SubElement(asset,'mesh',name=mesh_name,file=str(path),inertia='shell')
            ET.SubElement(world,'geom',name=mesh_name,type='mesh',mesh=mesh_name,group='0',contype='0',conaffinity='0',rgba='.5 .5 .5 1')
            scene_records.append({'name':name,'path':str(path),'sha256':_sha(path),'vertices':len(vertices),'triangles':len(faces)})
        if not scene_records:raise ValueError('No environment triangles intersect the requested crop')
        ET.indent(tree);tree.write(target,encoding='unicode')
        model=mujoco.MjModel.from_xml_path(str(target))
        env=[i for i in range(model.ngeom)if int(model.geom_group[i])==0]
        robot=[i for i in range(model.ngeom)if int(model.geom_group[i])==1]
        metadata={'scene':'Mars collision geometry, ideal optical robot selfmask','bounds_xy_m':bounds.tolist(),
            'crop_rule':'All triangles whose world XY AABB intersects bounds; vertices need not lie inside crop.',
            'source_assets':source_records,'cropped_assets':scene_records,'scene_xml':str(target),'scene_xml_sha256':_sha(target),
            'feature_link_yaw_rad':yaw,'feature_local_z_offsets_m':offsets,
            'environment_policy':'Saved collision DEM triangles and original lava/rock faces; no visual heightmap/resampling or smooth vertex normals.',
            'robot_policy':'Generated visible CAD/fork/shaft/belt geoms; groups3/5 hidden wedges/proxies excluded. Known-model self-first-hits masked, not transparent. Not an exact Gazebo robot optical sensor.',
            'robot_model_xml':str(robot_path),'robot_model_xml_sha256':_sha(robot_path)}
        return cls(model,env,robot,metadata)

    def set_qpos(self,qpos):
        qpos=np.asarray(qpos,float)
        if qpos.shape!=(self.model.nq,)or not np.isfinite(qpos).all():
            raise ValueError('qpos must match the private scene model and be finite')
        self.data.qpos[:]=qpos
        mujoco.mj_kinematics(self.model,self.data)

    def body_pose(self):
        transform=np.eye(4);transform[:3,:3]=self.data.xmat[self._fork].reshape(3,3);transform[:3,3]=self.data.xpos[self._fork]
        return transform

    def _rays(self,origin,directions,environment_only=False,cutoff=np.inf):
        directions=np.asarray(directions,float).reshape(-1,3)
        ids=np.full(len(directions),-1,np.int32);distances=np.full(len(directions),-1.);normals=np.zeros((len(directions),3))
        groups=np.array([1,0 if environment_only else 1,0,0,0,0],np.uint8)
        mujoco.mj_multiRay(self.model,self.data,np.asarray(origin,float),directions.ravel(),groups,True,-1,
            ids,distances,normals.ravel(),len(directions),float(cutoff))
        return distances,ids,normals

    def environment_hit(self,origins,directions):
        """Evaluator-only unrestricted first environment hit and geometric face normal."""
        origins,directions=np.asarray(origins,float),np.asarray(directions,float)
        if origins.shape!=directions.shape or origins.ndim!=2 or origins.shape[1]!=3 or not np.isfinite(origins).all()or not np.isfinite(directions).all():
            raise ValueError('Origins and directions must be finite matching (N,3) arrays')
        lengths=np.linalg.norm(directions,axis=1)
        if np.any(lengths<=0):raise ValueError('Ray directions must be nonzero')
        directions=directions/lengths[:,None];distance=np.full(len(origins),np.nan);normal=np.full(origins.shape,np.nan)
        groups=np.array([1,0,0,0,0,0],np.uint8)
        for i,(origin,direction)in enumerate(zip(origins,directions)):
            n=np.zeros(3);hit=mujoco.mj_ray(self.model,self.data,origin,direction,groups,True,-1,None,n)
            if hit>=0:
                distance[i]=hit
                if n@direction>0:n=-n
                normal[i]=n/np.linalg.norm(n)
        valid=np.isfinite(distance)
        return {'range_m':distance,'point_world':origins+distance[:,None]*directions,'normal_world':normal,'valid':valid}


class FourCameraDepthAdapter:
    """Causal capture adapter; return frames only to the mapper, never this scene."""
    def __init__(self,scene,cameras=None):
        self._scene=scene;self.cameras=tuple(four_camera_rig()if cameras is None else cameras)
        if not self.cameras or len({c.name for c in self.cameras})!=len(self.cameras):
            raise ValueError('At least one uniquely named camera is required')
        self._directions=[]
        for camera in self.cameras:
            u,v=np.meshgrid(np.arange(camera.width),np.arange(camera.height));k=camera.intrinsics
            rays=np.stack(((u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1],np.ones_like(u)),axis=-1).reshape(-1,3)
            rays=rays/np.linalg.norm(rays,axis=1,keepdims=True);self._directions.append(rays)

    def render_batch(self,qpos,capture_time_s,delivered_at_s):
        if not np.isfinite([capture_time_s,delivered_at_s]).all()or not 0<=capture_time_s<=delivered_at_s:
            raise ValueError('Capture/delivery timestamps must be finite and causal')
        self._scene.set_qpos(qpos);body=self._scene.body_pose();frames=[]
        for camera,rays in zip(self.cameras,self._directions):
            pose=body@camera.body_from_camera;world=rays@pose[:3,:3].T
            distance,ids,_=self._scene._rays(pose[:3,3],world,cutoff=camera.max_range_m)
            # A rejected near or self hit remains invalid. Never search for a
            # farther terrain point through a physically occluding first hit.
            valid=(distance>=camera.min_range_m)&(distance<=camera.max_range_m)&np.isin(ids,self._scene._environment)
            depth=np.full(len(rays),np.nan);depth[valid]=distance[valid]*rays[valid,2]
            frames.append(DepthFrame(camera.name,depth.reshape(camera.height,camera.width),valid.reshape(camera.height,camera.width),
                camera.intrinsics,camera.body_from_camera,pose,float(capture_time_s),float(delivered_at_s)))
        return tuple(frames)
