"""Descriptive saved-data followup: terrain/contact geometry versus estimatednormal."""
from pathlib import Path
import datetime,hashlib,json,shutil
import numpy as np
from spherical_wheel.contact_sensing import intrinsic_normal
ROOT=Path.cwd();BASE=Path(__file__).resolve().parents[1]
OUT=BASE/'review/normal_geometry_stage01';OUT.mkdir(exist_ok=False)
CASE=BASE/'campaign/native_substeps';TRACE=CASE/'substeps.npz';PREC=CASE/'campaign/runs/m100_R/precision.npz';RADIUS=.12
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inputs=[TRACE,PREC,ROOT/'src/spherical_wheel/contact_sensing.py',Path(__file__)]
registration={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Post-primary descriptive mechanism followup; no newphysics/controller/estimator execution, no changedcanonicaloutcomes/gates.','window_s':[31.405,31.745],'reason_start':'First complete5ms endpoint in1ms trace; originalI54windowbegins31.400 but precedingcontactsolvewasnotrecorded.','methods':'Atmatched5msendpoints: weightallactualpositiveFn contactpoints/normals byFn, noforce beyond1e-9N; compare contactpoint radialdirection and truewrench intrinsicdirection with filtered estimatednormal. Solvepreposecenterprimary, postposecenter sensitivity; solvernormal isnotassumeduniqueterrainplane. No lagfitting/signflips.','radius_m':RADIUS,'input_sha256':{str(p.relative_to(ROOT)):sha(p) for p in inputs}}
(OUT/'registered_before_arithmetic.json').write_text(json.dumps(registration,indent=2)+'\n');shutil.copyfile(__file__,OUT/'check_normal_geometry.py')
def load(p):
 with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def unit(x):return x/np.linalg.norm(x)
def angle(a,b):return float(np.degrees(np.arccos(np.clip(unit(a)@unit(b),-1,1))))
s=load(TRACE);p=load(PREC);times=p['time_s'];ii=np.flatnonzero((times>=31.405-1e-9)&(times<=31.745+1e-9));rows=[]
for i in ii:
 t=float(times[i]);j=int(np.argmin(abs(s['time_s']-t)));assert abs(s['time_s'][j]-t)<1e-8
 c=s['contact_rows'][s['contact_offsets'][j]:s['contact_offsets'][j+1]]
 normal_force=np.einsum('ij,ij->i',c[:,7:10],c[:,4:7]);real=normal_force>1e-9
 assert real.any();forces=c[:,7:10];pre=s['bridge_state_before'][j,2:5];post=s['bridge_state'][j,2:5]
 point=np.average(c[real,1:4],axis=0,weights=normal_force[real]);normal=unit(np.sum(normal_force[real,None]*c[real,4:7],axis=0));point_pre=unit(pre-point);point_post=unit(post-point)
 F=forces.sum(axis=0);Mpre=np.cross(c[:,1:4]-pre,forces).sum(axis=0);Mpost=np.cross(c[:,1:4]-post,forces).sum(axis=0)
 ipre=intrinsic_normal(F,Mpre,RADIUS);ipost=intrinsic_normal(F,Mpost,RADIUS);est=p['exact_estimated_normal'][i]
 assert ipre is not None and ipost is not None
 rows.append({'time_s':t,'estimated_normal':est.tolist(),'solver_weighted_normal':normal.tolist(),'point_pre_normal':point_pre.tolist(),'point_post_normal':point_post.tolist(),'true_wrench_intrinsic_pre':ipre.tolist(),'true_wrench_intrinsic_post':ipost.tolist(),'estimated_vs_point_pre_deg':angle(est,point_pre),'estimated_vs_wrench_intrinsic_pre_deg':angle(est,ipre),'estimated_vs_solver_normal_deg':angle(est,normal),'point_pre_post_sensitivity_deg':angle(point_pre,point_post),'wrench_pre_post_sensitivity_deg':angle(ipre,ipost),'load_N':float(normal_force[real].sum()),'force_carrying_tags':sorted(set(int(x) for x in c[real,0]))})
fields=['estimated_normal','solver_weighted_normal','point_pre_normal','true_wrench_intrinsic_pre'];summary={}
for field in fields:
 summary[field]={'net_change_deg':angle(rows[0][field],rows[-1][field]),'arc_deg':sum(angle(a[field],b[field]) for a,b in zip(rows[:-1],rows[1:])),'start':rows[0][field],'end':rows[-1][field]}
for field in ['estimated_vs_point_pre_deg','estimated_vs_wrench_intrinsic_pre_deg','estimated_vs_solver_normal_deg','point_pre_post_sensitivity_deg','wrench_pre_post_sensitivity_deg']:
 values=np.array([r[field] for r in rows]);summary[field]={'min_deg':float(values.min()),'max_deg':float(values.max()),'rms_deg':float(np.sqrt(np.mean(values**2))),'end_deg':float(values[-1])}
report={'iteration':'I-55','scope':registration['scope'],'samples':len(rows),'window_s':[rows[0]['time_s'],rows[-1]['time_s']],'metrics':summary,'rows':rows,'limitations':['Instantaneouslast1mssolverwrench/geometry versus currentfilteredobserver-derivednormal; compare observableswithout claiming all delay is computation.','Netcontactnormal/contactpointweightedaverage is descriptive, notuniquebalance-requiredterrainplane undergeneral multicontact.','No interventionchangesnormalestimator, so thisdoesnotestablish its individualcausalcontribution.']}
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
assert all(sha(ROOT/n)==h for n,h in registration['input_sha256'].items())
(OUT/'manifest.json').write_text(json.dumps({'files':[{'path':q.name,'bytes':q.stat().st_size,'sha256':sha(q)} for q in sorted(OUT.iterdir()) if q.is_file()]},indent=2)+'\n')
print(json.dumps({'samples':len(rows),'metrics':summary},indent=2))
