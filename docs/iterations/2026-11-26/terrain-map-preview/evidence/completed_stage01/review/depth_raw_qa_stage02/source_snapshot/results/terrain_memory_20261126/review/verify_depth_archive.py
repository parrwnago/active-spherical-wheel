"""Independent encoded-depth QA using quaternion math and direct source triangles.

No sensor adapter, MuJoCo raycaster, controller or simulator is executed.
"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,math,shutil
import numpy as np

FAMILY=Path(__file__).resolve().parents[1]
ROOT=FAMILY.parents[1]
OUT=FAMILY/'review/depth_raw_qa_stage02'
STAGE=FAMILY/'campaign/depth_stage01'
SAMPLE_BATCHES=(0,100,136,200,400,600,800,880)
COMPILED_MESH_BASE_ALLOWANCE_M=2e-6


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def load(path):
    with np.load(path)as z:return {k:z[k]for k in z.files}


def matrix(q):
    """Independent normalized wxyz quaternion-to-matrix formula."""
    q=np.asarray(q,float);q=q/np.linalg.norm(q,axis=-1,keepdims=True)
    w,x,y,z=np.moveaxis(q,-1,0)
    return np.stack((1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
        2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
        2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)),axis=-1).reshape(q.shape[:-1]+(3,3))


def obj(path):
    vertices=[];faces=[]
    with Path(path).open()as stream:
        for line in stream:
            if line.startswith('v '):vertices.append([float(v)for v in line.split()[1:4]])
            elif line.startswith('f '):
                face=[int(item.split('/')[0])for item in line.split()[1:]]
                face=[i-1 if i>0 else len(vertices)+i for i in face]
                faces.extend([face[0],face[j],face[j+1]]for j in range(1,len(face)-1))
    return np.asarray(vertices,float),np.asarray(faces,int).reshape(-1,3)


def intersect(triangles,origin,direction):
    edge1=triangles[:,1]-triangles[:,0];edge2=triangles[:,2]-triangles[:,0]
    h=np.cross(direction,edge2);det=np.sum(edge1*h,axis=1);valid=abs(det)>1e-13
    inv=np.zeros_like(det);inv[valid]=1/det[valid];s=origin-triangles[:,0]
    u=np.sum(s*h,axis=1)*inv;q=np.cross(s,edge1);v=np.sum(q*direction,axis=1)*inv
    distance=np.sum(edge2*q,axis=1)*inv
    valid&=(u>=-1e-12)&(v>=-1e-12)&(u+v<=1+1e-12)&(distance>=0)
    found=np.flatnonzero(valid)
    if not len(found):return None
    i=int(found[np.argmin(distance[found])]);normal=np.cross(edge1[i],edge2[i]);normal/=np.linalg.norm(normal)
    return float(distance[i]),normal,float(min(u[i],v[i],1-u[i]-v[i]))


def main():
    OUT.mkdir(exist_ok=False)
    source=Path(__file__);snapshot=OUT/'source_snapshot'/source.relative_to(ROOT)
    snapshot.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,snapshot)
    protocol=read(FAMILY/'protocol.json');metadata=read(STAGE/'scene_metadata.json')
    inputs=[FAMILY/'protocol.json',STAGE/'depth_batches.npz',STAGE/'report.json',STAGE/'source_manifest.json',STAGE/'scene_metadata.json',ROOT/protocol['precision']]
    inputs.extend(Path(r['path'])for r in metadata['source_assets']+metadata['cropped_assets'])
    provenance={'captured_before_calculation_utc':datetime.now(timezone.utc).isoformat(),
        'source_sha256':sha(source),'source_snapshot':str(snapshot.relative_to(OUT)),
        'inputs_sha256':{str(p.relative_to(ROOT)):sha(p)for p in inputs},
        'sample_rule':'Fixed batches0/100/136/200/400/600/800/880; each camera first and second thirds of its flattened valid-pixel list. No score-based selection.',
        'tolerance_rule':'Float32 depth half-ULP projected to range plus2um compiled-mesh allowance/max(|faceNormal.dot(ray)|,.05). Exact capture/calibration pose tolerance2e-14.',
        'scope':'Independent encoded-array/quaternion/source-triangle audit; no adapter or simulator execution.'}
    (OUT/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    original_report=read(STAGE/'report.json')
    for name,digest in original_report['outputs_sha256'].items():assert sha(STAGE/name)==digest
    source_count=0;origin_differences=[]
    allowed_later_mapping_sources={'src/spherical_wheel/terrain_memory.py','tests/test_terrain_memory.py','tests/test_terrain_preview_campaign.py'}
    for record in read(STAGE/'source_manifest.json')['sources']:
        assert sha(STAGE/record['snapshot'])==record['sha256']
        current=sha(ROOT/record['path'])
        if current!=record['sha256']:
            assert record['path']in allowed_later_mapping_sources,record['path']
            origin_differences.append({'path':record['path'],'depth_stage_sha256':record['sha256'],'current_sha256':current,
                'role':'Later mapping/test refinement; not executed by depth rendering. Exact depth-stage source remains preserved.'})
        source_count+=1
    data=load(STAGE/'depth_batches.npz');exact=load(ROOT/protocol['precision'])
    depth,valid=data['depth_m'],data['valid_mask'];assert depth.shape==valid.shape==(881,4,48,64)
    assert depth.dtype==np.float32 and valid.dtype==bool
    assert np.isfinite(depth[valid]).all()and np.all(depth[valid]>0)and np.isnan(depth[~valid]).all()
    np.testing.assert_array_equal(data['source_indices'],np.arange(0,8801,10))
    np.testing.assert_array_equal(data['capture_time_s'],exact['time_s'][data['source_indices']])
    np.testing.assert_array_equal(data['delivered_at_s'],data['capture_time_s']+.05)
    np.testing.assert_allclose(np.diff(data['capture_time_s']),.05,atol=1e-12,rtol=0)
    expected_names=['camera_000','camera_090','camera_180','camera_270'];assert data['camera_names'].tolist()==expected_names
    focal=32/math.tan(math.radians(50));k=np.array([[focal,0,31.5],[0,focal,23.5],[0,0,1.]])
    mounts=[];pitch=math.pi/4
    for yaw_deg,origin in zip((0,90,180,270),protocol['cameras']['origins_body_m']):
        yaw=math.radians(yaw_deg);c,s=math.cos(yaw),math.sin(yaw)
        t=np.eye(4);t[:3,3]=origin
        t[:3,:3]=np.array([[s,-math.sin(pitch)*c,math.cos(pitch)*c],[-c,-math.sin(pitch)*s,math.cos(pitch)*s],[0,-math.cos(pitch),-math.sin(pitch)]])
        mounts.append(t)
    mounts=np.array(mounts)
    np.testing.assert_allclose(data['intrinsics'],np.repeat(k[None],4,axis=0),atol=1e-13,rtol=0)
    np.testing.assert_allclose(data['body_from_camera'],mounts,atol=1e-14,rtol=0)
    qpos=exact['exact_qpos'][data['source_indices']];body=np.repeat(np.eye(4)[None],len(qpos),axis=0)
    body[:,:3,:3]=matrix(qpos[:,3:7]);body[:,:3,3]=qpos[:,:3]
    expected=np.einsum('nij,cjk->ncik',body,mounts);pose_error=float(np.max(abs(expected-data['world_from_camera'])))
    np.testing.assert_allclose(expected,data['world_from_camera'],atol=2e-14,rtol=0)
    u,v=np.meshgrid(np.arange(64),np.arange(48));rays=np.stack(((u-31.5)/focal,(v-23.5)/focal,np.ones_like(u)),axis=-1)
    rays/=np.linalg.norm(rays,axis=-1,keepdims=True)
    camera_stats={}
    for c,name in enumerate(expected_names):
        distances=depth[:,c].astype(float)/rays[None,:,:,2]
        quantum=.51*np.spacing(depth[:,c]).astype(float)/rays[None,:,:,2]
        assert np.all(distances[valid[:,c]]>=.08-quantum[valid[:,c]]-1e-12)
        assert np.all(distances[valid[:,c]]<=2.+quantum[valid[:,c]]+1e-12)
        count=int(valid[:,c].sum());total=valid[:,c].size
        camera_stats[name]={'valid_pixels':count,'invalid_pixels':total-count,'valid_ratio':count/total,
            'invalid_ratio':1-count/total,'minimum_valid_euclidean_range_m':float(distances[valid[:,c]].min()),
            'maximum_valid_euclidean_range_m':float(distances[valid[:,c]].max())}
    # Verify every cropped face directly against the transformed raw-source crop
    # selection, independently of MuJoCo's mesh loader or ray routines.
    all_triangles=[];crop_checks=[];bounds=np.asarray(metadata['bounds_xy_m']);source_by_name={r['name']:r for r in metadata['source_assets']}
    for record in metadata['cropped_assets']:
        vertices,faces=obj(source_by_name[record['name']]['path'])
        if record['name']!='dem':
            yaw=1.5707963268;c,s=math.cos(yaw),math.sin(yaw)
            vertices=np.column_stack([c*vertices[:,0]-s*vertices[:,1],s*vertices[:,0]+c*vertices[:,1],
                vertices[:,2]+(-.03 if record['name']=='rocks_medium' else 0.)])
        tri=vertices[faces];low,high=tri.min(axis=1),tri.max(axis=1)
        keep=(high[:,0]>=bounds[0])&(low[:,0]<=bounds[1])&(high[:,1]>=bounds[2])&(low[:,1]<=bounds[3])
        original=tri[keep];cv,cf=obj(record['path']);cropped=cv[cf]
        assert original.shape==cropped.shape
        np.testing.assert_allclose(original,cropped,atol=2e-14,rtol=0)
        crop_checks.append({'asset':record['name'],'triangles':len(cropped),'maximum_source_vertex_difference_m':float(np.max(abs(original-cropped)))})
        all_triangles.append(original)
    triangles=np.concatenate(all_triangles);samples=[]
    for batch in SAMPLE_BATCHES:
        for c,name in enumerate(expected_names):
            finite=np.flatnonzero(valid[batch,c])
            if not len(finite):continue
            for location in sorted({int(finite[len(finite)//3]),int(finite[2*len(finite)//3])}):
                row,column=divmod(location,64);optical=rays[row,column];pose=expected[batch,c]
                direction=pose[:3,:3]@optical;hit=intersect(triangles,pose[:3,3],direction)
                assert hit is not None,(batch,c,row,column)
                distance,normal,bary_margin=hit;encoded=float(depth[batch,c,row,column]);encoded_range=encoded/optical[2]
                half_ulp=.51*float(np.spacing(depth[batch,c,row,column]))/optical[2]
                incidence=abs(float(normal@direction));allowance=COMPILED_MESH_BASE_ALLOWANCE_M/max(incidence,.05)+half_ulp
                error=abs(encoded_range-distance);assert error<=allowance,(batch,c,row,column,error,allowance)
                samples.append({'batch':batch,'camera':name,'pixel_rc':[row,column],'capture_time_s':float(data['capture_time_s'][batch]),
                    'encoded_axial_depth_m':encoded,'independent_source_range_m':distance,'range_difference_m':error,
                    'axial_difference_m':abs(encoded-distance*optical[2]),'compiled_plus_encoding_allowance_m':allowance,
                    'incidence_abs_cos':incidence,'face_barycentric_edge_margin':bary_margin})
    report={'passed':True,'scope':provenance['scope'],'archive_sha256':sha(STAGE/'depth_batches.npz'),
        'shape':list(depth.shape),'capture_delivery_calibration_verified':True,'capture_pose_maximum_matrix_difference':pose_error,
        'source_snapshots_verified':source_count,'depth_producing_sources_still_match_current':True,
        'explicit_later_mapping_source_differences':origin_differences,
        'all_valid_ranges_respect_euclidean_limits_with_float32_encoding_budget':True,
        'camera_validity':camera_stats,'raw_source_crop_face_checks':crop_checks,
        'sampled_valid_rays':len(samples),'maximum_sampled_range_error_m':max(r['range_difference_m']for r in samples),
        'maximum_sampled_axial_error_m':max(r['axial_difference_m']for r in samples),
        'independent_triangle_samples':'ray_samples.json',
        'limitations':['Samples validate finite environment first hits; they do not independently retrace every masked robot pixel.',
            'Robot selfmask and first-hit no-see-through are separately covered by the preserved adapter visibility fixtures.',
            'Float32 depth encoding and compiled mesh arithmetic are retained; ideal sensing is not infinite numerical precision.',
            'No map utility, localization accuracy or controller benefit is established by this integrity audit.']}
    (OUT/'ray_samples.json').write_text(json.dumps(samples,indent=2)+'\n');(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for rel,digest in provenance['inputs_sha256'].items():assert sha(ROOT/rel)==digest
    files=[{'path':str(p.relative_to(OUT)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(OUT.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']
    (OUT/'artifact_manifest.json').write_text(json.dumps({'files':files},indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
