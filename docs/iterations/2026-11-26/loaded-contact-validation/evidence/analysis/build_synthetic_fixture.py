"""Fabricated schema fixture, not a physical experiment or I57 result."""
from pathlib import Path
import json,numpy as np
ROOT=Path.cwd();HERE=ROOT/'results/loaded_contact_validation_20261126/analysis';F=HERE/'synthetic_fixture_stage01';F.mkdir(exist_ok=False)
protocol=json.loads((HERE.parent/'protocol.json').read_text());event=dict(protocol['events'][0]);event.update(event_id='synthetic',index=0,time_s=0.);jobs=[]
for row in protocol['cases'][:15]:
    job=dict(row);job['event_id']='synthetic';job['case_id']='synthetic_'+row['case_id'].split('_',1)[1]
    if row['role']!='governed_center_column_probe':job['delta_input']=[0.]*5
    jobs.append(job)
q=np.zeros((5,10));q[:,3]=1;v=np.zeros((5,9));t=np.arange(5)*.005;u=np.zeros((5,5));saved=dict(time_s=t,exact_qpos=q,exact_qvel=v,exact_previous_applied=u,exact_applied=u)
np.savez_compressed(F/'saved_source.npz',**saved)
protocol.update(events=[event],cases=jobs,source_data=dict(precision_path=str((F/'saved_source.npz').relative_to(ROOT))))
(F/'protocol.json').write_text(json.dumps(protocol))
model=F/'model/synthetic';model.mkdir(parents=True);mat={};P=np.zeros((12,9));P[:3,:3]=np.eye(3);P[3:6,3:6]=np.eye(3);P[6:9,3:6]=np.eye(3);P[9:,6:]=np.eye(3)
G=np.zeros((9,5));G[0,0]=1;G[3,1]=1;G[4,2]=1;G[5,0]=.3;G[6,0]=2;G[7,3]=1;G[8,4]=1
for h in ('h005ms','h020ms'):mat[h+'_response']=G;mat[h+'_free_response']=G;mat[h+'_physical_map']=P
np.savez_compressed(model/'matrices.npz',**mat);(model/'predictions.json').write_text(json.dumps(dict(saved_index=0)))
for job in jobs:
    d=F/'campaign/runs'/job['case_id'];d.mkdir(parents=True);du=np.array(job['delta_input']);commands=np.tile(du,(4,1));pv=v.copy();pv[1:]=G@du;prev=np.tile(du,(5,1));prev[0]=0
    data=dict(time_s=t,offset_s=t,offset_steps=np.arange(5),exact_qpos=q,exact_qvel=pv,exact_previous_applied=prev,actual_applied=prev,bridge_state=np.zeros((5,24)),command_sequence=commands,contact_rows=np.empty((0,16)),contact_offsets=np.zeros(6,dtype=np.int64))
    np.savez_compressed(d/'endpoints.npz',**data);np.savez_compressed(d/'prefix.npz',**{k:value[:1] for k,value in saved.items() if k!='exact_applied'});(d/'provenance.json').write_text(json.dumps(dict(success=True,first_slew_violation=[0]*5)))
print(F)
