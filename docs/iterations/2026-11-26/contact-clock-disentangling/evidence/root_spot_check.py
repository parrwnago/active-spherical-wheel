"""Independent saved-array admission geometry check; no model/controller/physics."""
from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path.cwd();OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def axis(states):
 q=states[:,5:9];q=q/np.linalg.norm(q,axis=1)[:,None];theta=states[:,15]
 v=np.column_stack((np.cos(theta),np.zeros(len(theta)),-np.sin(theta)))
 return v+2*np.cross(q[:,1:],np.cross(q[:,1:],v)+q[:,:1]*v)
results={};inputs={}
for name,folder in [('Mars','mars_compact_outback'),('flat','flat_compact_outback_setup_corrected')]:
 p=ROOT/'results/support_routing_timing_20261126/campaign/runs'/folder/'truth_substeps.npz';inputs[str(p.relative_to(ROOT))]=sha(p)
 with np.load(p) as z:s={k:z[k] for k in z.files}
 st=s['bridge_state'];rows=s['contacts'];sample=np.repeat(np.arange(len(st)),np.diff(s['contact_offsets']));fn=np.maximum(np.sum(rows[:,4:7]*rows[:,7:10],axis=1),0)
 eligible=(sample>0)&(rows[:,0]>=0)&(rows[:,0]<=3)&(rows[:,12]>.5)&(fn>.05)
 rows=rows[eligible];fn=fn[eligible];sample=sample[eligible];tags=rows[:,0].astype(int)
 lower=np.where(tags<2,.01,.11565465836);upper=np.where(tags<2,.11565465836,.12);sign=np.where(tags%2==0,1.,-1.)
 postaxis=axis(st);checks={};allmargin={}
 for pose,idx in [('post',sample),('prior_proxy',sample-1)]:
  v=rows[:,1:4]-st[idx,2:5];value0=np.sum(postaxis[idx]*v,axis=1)
  for norm in [False,True]:
   value=sign*value0*(.12/np.linalg.norm(v,axis=1) if norm else 1.)
   margin=np.minimum(value-lower,upper-value);key=pose+('_normalized' if norm else '_raw');allmargin[key]=margin
   checks[key]={'violating_records':int(np.sum(margin < -1e-12)),'minimum_margin_m':float(margin.min()),'fraction_of_eligible_normal_load':float(fn[margin < -1e-12].sum()/fn.sum())}
 worst=np.argsort(allmargin['post_raw'])[:3]
 examples=[{'substep_index':int(sample[k]),'relative_time_s':float(s['time_s'][sample[k]]),'tag':int(tags[k]),'normal_load_N':float(fn[k]),'margins_m':{n:float(m[k]) for n,m in allmargin.items()}} for k in worst]
 results[name]={'eligible_records':len(rows),'substeps':len(st),'time_step_continuity':bool(np.allclose(np.diff(s['time_s']),.001,atol=1e-10,rtol=0)),'iteration_continuity':bool(np.all(np.diff(st[:,1])==1)),'conditions':checks,'worst_post_raw_examples':examples}
assert all(sha(ROOT/rel)==digest for rel,digest in inputs.items())
r={'scope':'Independent admission-band diagnostic on individual admitted loaded contacts. Prior saved state is pre-step proxy; no observer, controller or physics executed. Firstsubstepexcluded. Not an observer-error or trajectory-improvement metric.','inputs_sha256':inputs,'source_sha256':sha(Path(__file__)),'cases':results}
(OUT/'root_spot_check.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
