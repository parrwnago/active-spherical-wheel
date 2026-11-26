"""One-pose Mars scene/throughput fixture; zero dynamics, not a trace campaign."""
from pathlib import Path
import hashlib,json,shutil,time,traceback
import numpy as np
from spherical_wheel.model import REPO_ROOT,load_config
from spherical_wheel.terrain_depth import TerrainRayScene,FourCameraDepthAdapter,_read_obj

FAMILY=REPO_ROOT/'results/terrain_memory_20261126'
OUT=FAMILY/'adapter_development/stage02_mars_factory'
OUT.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
paths=[Path(__file__),REPO_ROOT/'src/spherical_wheel/terrain_depth.py',REPO_ROOT/'tests/test_terrain_depth.py',FAMILY/'protocol.json']
records=[]
for source in paths:
    target=OUT/'source_snapshot'/source.relative_to(REPO_ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    records.append({'origin':str(source.relative_to(REPO_ROOT)),'snapshot':str(target.relative_to(OUT)),'sha256':sha(source)})
(OUT/'sources_before_execution.json').write_text(json.dumps(records,indent=2)+'\n')
try:
    baseline=REPO_ROOT/'results/lqr_contact_forecast_20261125/campaign/runs/mars_figure8_p090'
    provenance=json.loads((baseline/'provenance.json').read_text());dem=Path(provenance['engine']['world_info']['dem_mesh'])
    with np.load(baseline/'precision.npz')as z:qpos=z['exact_qpos'].copy()
    bounds=[float(qpos[:,0].min()-2.30),float(qpos[:,0].max()+2.30),float(qpos[:,1].min()-2.30),float(qpos[:,1].max()+2.30)]
    started=time.perf_counter();scene=TerrainRayScene.from_mars(load_config(),OUT/'scene',bounds,dem_obj=dem);compile_s=time.perf_counter()-started
    adapter=FourCameraDepthAdapter(scene);started=time.perf_counter();frames=adapter.render_batch(qpos[0],0.,.05);capture_s=time.perf_counter()-started
    stored={}
    for frame in frames:
        for key in ('depth_m','valid_mask','intrinsics','world_from_camera','body_from_camera'):stored[frame.camera_name+'_'+key]=getattr(frame,key)
    np.savez_compressed(OUT/'first_pose_frames.npz',**stored)
    started=time.perf_counter()
    for index in range(20):adapter.render_batch(qpos[0],index*.05,index*.05+.05)
    benchmark_s=time.perf_counter()-started
    origins=np.array([[-1.6,9.6,10.],[-1.53,9.58,10.],[-1.67,9.63,10.]])
    directions=np.tile([0.,0.,-1.],(len(origins),1));hits=scene.environment_hit(origins,directions)
    vertices,faces=_read_obj(dem);triangles=[vertices[faces]]
    for item in scene.metadata['source_assets'][1:]:
        vertices,faces=_read_obj(item['path']);yaw=scene.metadata['feature_link_yaw_rad'];c,s=np.cos(yaw),np.sin(yaw)
        world=np.c_[c*vertices[:,0]-s*vertices[:,1],s*vertices[:,0]+c*vertices[:,1],vertices[:,2]+scene.metadata['feature_local_z_offsets_m'][item['name']]]
        triangles.append(world[faces])
    triangles=np.concatenate(triangles);e1=triangles[:,1]-triangles[:,0];e2=triangles[:,2]-triangles[:,0]
    checks=[]
    for i,(origin,direction)in enumerate(zip(origins,directions)):
        cross=np.cross(np.broadcast_to(direction,e2.shape),e2);det=(e1*cross).sum(1);valid=abs(det)>1e-12
        inv=np.zeros_like(det);inv[valid]=1/det[valid];relative=origin-triangles[:,0];u=(relative*cross).sum(1)*inv
        q=np.cross(relative,e1);v=(q*direction).sum(1)*inv;distance=(q*e2).sum(1)*inv
        valid&=(u>=0)&(v>=0)&(u+v<=1)&(distance>=0)
        indices=np.flatnonzero(valid);chosen=indices[np.argmin(distance[indices])];expected=distance[chosen]
        normal=np.cross(e1[chosen],e2[chosen]);normal/=np.linalg.norm(normal)
        if normal@direction>0:normal=-normal
        gap=abs(hits['range_m'][i]-expected);normal_gap=np.linalg.norm(hits['normal_world'][i]-normal)
        assert gap<2e-6 and normal_gap<2e-5,(gap,normal_gap)
        checks.append({'origin':origin.tolist(),'range_error_m':float(gap),'normal_vector_error':float(normal_gap)})
    assert len(frames)==4 and all(frame.depth_m.shape==(48,64)for frame in frames)
    report={'passed':True,'scope':'One saved initial pose plus20 repeats for throughput only; no fulltrace sensor replay or physics.',
        'compile_s':compile_s,'single_capture_s':capture_s,'twenty_capture_s':benchmark_s,
        'rays_per_s':20*4*64*48/benchmark_s,'extrapolated_880_capture_s':benchmark_s*44,
        'source_sha256':sha(REPO_ROOT/'src/spherical_wheel/terrain_depth.py'),
        'geometry_parity_checks':checks,'scene':scene.metadata,
        'fixture_valid_pixels':{f.camera_name:int(f.valid_mask.sum())for f in frames}}
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
except Exception:
    (OUT/'execution_failure.json').write_text(json.dumps({'traceback':traceback.format_exc()},indent=2)+'\n');raise
