"""Clock-matched descriptive contact-normal followup; no new physics."""
from pathlib import Path
import datetime,hashlib,json,shutil
import numpy as np
from spherical_wheel.contact_sensing import intrinsic_normal
from spherical_wheel.model import load_config
ROOT=Path.cwd();BASE=Path(__file__).resolve().parents[1];OUT=BASE/'review/normal_geometry_stage02';OUT.mkdir(exist_ok=False)
CASE=BASE/'campaign/native_substeps';TRACE=CASE/'substeps.npz';PREC=CASE/'campaign/runs/m100_R/precision.npz';RADIUS=float(load_config()['geometry']['sphere_radius_m'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inputs=[TRACE,PREC,ROOT/'src/spherical_wheel/contact_sensing.py',ROOT/'config/model.yaml',Path(__file__)]
registration={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Descriptivepost-primaryfollowup; canonicaloutcomes/gatesunchanged. No physics/controller/observer update.','window_s':[31.405,31.745],'first_stage_failure':'stage01 assumed every1msendpoint carriedforce; assertionfailed. Preservefailedsource/registration/log. This stage recordsundefinedobservables andadds preceding5ms force/moment summaries toalign observerintervaldefinition. No lagfit, signflip ormissing-value repair.','methods':'Signed netforce/moment useallrows; wheel geometry/normals useactualpositiveFn>1e-9N wheelrows. Primarywrenchmean ismean ofeachsolveforce andmoment aboutits ownPREposewheelcenter, postpose sensitivityalso reported. Point-radial andsolvernormal observableskeptseparate. No claimuniqueterrainplane orindividualobserverlatency.','radius_m':RADIUS,'input_sha256':{str(p.relative_to(ROOT)):sha(p) for p in inputs}}
(OUT/'registered_before_arithmetic.json').write_text(json.dumps(registration,indent=2)+'\n');shutil.copyfile(__file__,OUT/'check_normal_geometry_stage02.py')
def load(p):
 with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def unit(x):return x/np.linalg.norm(x)
def angle(a,b):return float(np.degrees(np.arccos(np.clip(unit(a)@unit(b),-1,1))))
s=load(TRACE);p=load(PREC);times=p['time_s'];ii=np.flatnonzero((times>=31.405-1e-9)&(times<=31.745+1e-9));rows=[]
def measure(js):
 forces=[];mpre=[];mpost=[];radials=[];normals=[];weights=[];tags=[];loadedticks=0;bodyforce=0.;rejectedforce=0.
 for j in js:
  c=s['contact_rows'][s['contact_offsets'][j]:s['contact_offsets'][j+1]];f=c[:,7:10];pre=s['bridge_state_before'][j,2:5];post=s['bridge_state'][j,2:5]
  forces.append(f.sum(axis=0));mpre.append(np.cross(c[:,1:4]-pre,f).sum(axis=0));mpost.append(np.cross(c[:,1:4]-post,f).sum(axis=0))
  fn=np.einsum('ij,ij->i',f,c[:,4:7]);real=(fn>1e-9)&(c[:,0]<6);loadedticks+=int(real.any());bodyforce+=float(np.linalg.norm(f[c[:,0]==6],axis=1).sum());rejectedforce+=float(np.linalg.norm(f[c[:,12]<.5],axis=1).sum())
  if real.any():
   weights.extend(fn[real]);normals.extend(c[real,4:7]);radials.extend(pre-c[real,1:4]);tags.extend(c[real,0].astype(int))
 F=np.mean(forces,axis=0);M=np.mean(mpre,axis=0);Mp=np.mean(mpost,axis=0);ni=intrinsic_normal(F,M,RADIUS);nip=intrinsic_normal(F,Mp,RADIUS);f2=float(F@F)
 result={'loaded_ticks':loadedticks,'ticks':len(js),'positive_wheel_force_weight_N':float(sum(weights)),'force_carrying_tags':sorted(set(int(t) for t in tags)),'body_sum_force_norm_N':bodyforce,'rejected_sum_force_norm_N':rejectedforce,'true_wrench_intrinsic_pre':None if ni is None else ni.tolist(),'true_wrench_intrinsic_post':None if nip is None else nip.tolist(),'intrinsic_lateral_squared':float(np.linalg.norm(np.cross(F,M)/(RADIUS*f2))**2) if f2>=1e-12 else None}
 if weights:
  point=unit(np.average(radials,axis=0,weights=weights));normal=unit(np.average(normals,axis=0,weights=weights));result.update(point_pre_normal=point.tolist(),solver_weighted_normal=normal.tolist())
 else:result.update(point_pre_normal=None,solver_weighted_normal=None)
 return result
for i in ii:
 t=float(times[i]);j=int(np.argmin(abs(s['time_s']-t)));assert abs(s['time_s'][j]-t)<1e-8 and s['substep_index'][j]==5
 js=list(range(j-4,j+1));assert all(s['hold_index'][k]==s['hold_index'][j] for k in js)
 rows.append({'time_s':t,'estimated_normal':p['exact_estimated_normal'][i].tolist(),'endpoint':measure([j]),'interval':measure(js)})
summary={'estimated_normal_net_change_deg':angle(rows[0]['estimated_normal'],rows[-1]['estimated_normal']),'samples':len(rows)}
for clock in ['endpoint','interval']:
 data={}
 for field in ['point_pre_normal','solver_weighted_normal','true_wrench_intrinsic_pre']:
  valid=[r for r in rows if r[clock][field] is not None];errs=[angle(r['estimated_normal'],r[clock][field]) for r in valid]
  data[field]={'defined_samples':len(valid),'first_defined_s':valid[0]['time_s'] if valid else None,'last_defined_s':valid[-1]['time_s'] if valid else None,'net_change_between_defined_endpoints_deg':angle(valid[0][clock][field],valid[-1][clock][field]) if valid else None,'estimated_difference_max_deg':max(errs) if errs else None,'estimated_difference_rms_deg':float(np.sqrt(np.mean(np.square(errs)))) if errs else None,'estimated_difference_at_last_defined_deg':errs[-1] if errs else None}
 data['zero_force_samples']=sum(r[clock]['loaded_ticks']==0 for r in rows)
 data['multi_patch_samples']=sum(len(r[clock]['force_carrying_tags'])>1 for r in rows)
 data['over_radius_intrinsic_samples']=sum(r[clock]['intrinsic_lateral_squared'] is not None and r[clock]['intrinsic_lateral_squared']>=1 for r in rows)
 data['pre_post_intrinsic_max_sensitivity_deg']=max([angle(r[clock]['true_wrench_intrinsic_pre'],r[clock]['true_wrench_intrinsic_post']) for r in rows if r[clock]['true_wrench_intrinsic_pre'] is not None and r[clock]['true_wrench_intrinsic_post'] is not None],default=None)
 summary[clock]=data
report={'iteration':'I-55','scope':registration['scope'],'summary':summary,'rows':rows,'interpretation_limits':['Differentphysicalobservables andfilterclocks; no estimation-error/latencyclaim solelyfromtheseangles.','Allmissingnormal/zero-loadobservationsretained; nointerpfill.','Point-radial andequivalentwrenchdirections neednot equaluniquegroundplane; multi-contactaveragesare descriptive.']}
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n');assert all(sha(ROOT[n])==h for n,h in [])
for n,h in registration['input_sha256'].items():assert sha(ROOT/n)==h,n
(OUT/'manifest.json').write_text(json.dumps({'files':[{'path':q.name,'bytes':q.stat().st_size,'sha256':sha(q)} for q in sorted(OUT.iterdir()) if q.is_file()]},indent=2)+'\n');print(json.dumps(summary,indent=2))
