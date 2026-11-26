"""Native ray backend fixture only: concave triangle first hit, no dynamics."""
from pathlib import Path
import hashlib,json,time
import mujoco
import numpy as np
HERE=Path(__file__).resolve().parent
vertices=np.array([[-1.,-1.,1.],[1.,-1.,1.],[1.,1.,1.],[-1.,1.,1.],[0.,0.,0.]])
faces=np.array([[0,1,4],[1,2,4],[2,3,4],[3,0,4]])
xml='<mujoco><asset><mesh name="bowl" inertia="shell" vertex="'+ ' '.join(map(str,vertices.ravel()))+'" face="'+' '.join(map(str,faces.ravel()))+'"/></asset><worldbody><geom name="concave" type="mesh" mesh="bowl" contype="0" conaffinity="0"/></worldbody></mujoco>'
(HERE/'fixture.xml').write_text(xml)
model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);mujoco.mj_kinematics(model,data)
def independent(origin,direction):
 tri=vertices[faces];e1=tri[:,1]-tri[:,0];e2=tri[:,2]-tri[:,0];p=np.cross(np.broadcast_to(direction,e2.shape),e2);det=(e1*p).sum(1);keep=abs(det)>1e-12
 inv=np.zeros_like(det);inv[keep]=1/det[keep];t=origin-tri[:,0];u=(t*p).sum(1)*inv;q=np.cross(t,e1);v=(q*direction).sum(1)*inv;distance=(e2*q).sum(1)*inv
 keep&=(u>=0)&(v>=0)&(u+v<=1)&(distance>=0)
 return float(distance[keep].min())
checks=[]
for origin in ([.1,.2,2.],[-.3,.15,2.],[.35,-.2,2.]):
 direction=np.array([0.,0.,-1.]);origin=np.asarray(origin);gid=np.zeros(1,dtype=np.int32);normal=np.zeros(3)
 actual=mujoco.mj_ray(model,data,origin,direction,None,True,-1,gid,normal);expected=independent(origin,direction)
 assert abs(actual-expected)<1e-7,(actual,expected)
 checks.append(dict(origin=origin.tolist(),ray=actual,independent=expected,normal=normal.tolist()))
n=3072;origin=np.array([.1,.2,2.]);vectors=np.tile([0.,0.,-1.],(n,1));ids=np.empty(n,np.int32);dist=np.empty(n)
start=time.perf_counter()
for _ in range(100):mujoco.mj_multiRay(model,data,origin,vectors.ravel(),None,True,-1,ids,dist,None,n,3.)
elapsed=time.perf_counter()-start
assert np.allclose(dist,independent(origin,vectors[0]),atol=1e-7)
report={'scope':'Concave first-hit/throughput fixture, zero physics steps.','mujoco':mujoco.__version__,'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'checks':checks,'rays':n*100,'wall_s':elapsed,'rays_per_s':n*100/elapsed,'concavity_preserved_not_convex_hull':True}
(HERE/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
