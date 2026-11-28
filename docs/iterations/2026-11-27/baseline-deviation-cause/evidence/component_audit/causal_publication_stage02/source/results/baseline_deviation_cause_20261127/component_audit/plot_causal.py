"""I77 compact physical causal comparison; source/input seal precedes arrays."""
from pathlib import Path
import hashlib,importlib.util,json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=HERE/'causal_publication_stage02'
RUNS=HERE/'causal_branches_stage01'
ARMS=('native24','one_pulse','cluster_cap5')
CASE='straight_m100_N'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')
def main():
 OUT.mkdir(exist_ok=False)
 source=Path(__file__);route=ROOT/'results/tangent_heading_routes_20261127/runtime/route.py'
 report_path=HERE/'causal_reduction_stage02/report.json'
 precisions={arm:RUNS/arm/'child/campaign/runs'/CASE/'precision.npz'for arm in ARMS}
 inputs=[route,report_path,*precisions.values()]
 for p in (source,route):
  dest=OUT/'source'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 save(OUT/'registered_before_arrays.json',dict(iteration='I-77',source={str(p.relative_to(ROOT)):sha(p)for p in (source,route)},inputs={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in inputs},no_case_arrays_loaded_before_registration=True,design='Two panels: actualXY versus exact straight path, geometric error20.4–24s. Same native state prefix; fixed intervention window20.4–20.65s and singlepulse20.585s. No controller adoption claim.'))
 report=json.loads(report_path.read_text())
 spec=importlib.util.spec_from_file_location('_i77_causal_plot_route',route);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);path=module.make_route('straight')
 origin=np.asarray(path.origin);direction=np.asarray(path.direction)
 fig,(xyax,erax)=plt.subplots(1,2,figsize=(12,4.4),gridspec_kw=dict(width_ratios=[1,1.65]))
 colors=['#315D9F','#D08028','#16836B'];labels=['Native baseline','One-pulse hold','250 ms reference cap'];lines=['-','--','-'];points=[]
 for arm,color,label,line in zip(ARMS,colors,labels,lines):
  with np.load(precisions[arm],allow_pickle=False)as z:
   t=z['time_s'][4080:4801];xy=z['exact_qpos'][4080:4801,:2]
  closest=origin+np.clip((xy-origin)@direction,0.,1.2)[:,None]*direction
  error=np.linalg.norm(xy-closest,axis=1)*1000
  expected=report['arms'][arm]['windows']['full_20p4_to24']['geometric']
  if float(np.max(error)/1000)!=expected['peak_m']:raise AssertionError('Figure peak differs from sealed reduction')
  xyax.plot(xy[:,0],xy[:,1],color=color,label=label,linestyle=line,lw=1.8,zorder=3)
  xyax.scatter(*xy[0],s=18,color=color,zorder=4);xyax.scatter(*xy[-1],s=25,marker='s',color=color,zorder=4)
  erax.plot(t,error,color=color,label=f'{label}: peak {np.max(error):.1f} mm',linestyle=line,lw=1.9)
  j=int(np.argmax(error));erax.scatter(t[j],error[j],s=20,color=color,zorder=4)
  points.append(xy)
 full=np.vstack([origin,origin+1.2*direction]);xyax.plot(full[:,0],full[:,1],color='#555555',ls=':',lw=1.7,label='Planned straight path',zorder=1)
 pts=np.concatenate(points);padding=.025
 xyax.set_xlim(float(pts[:,0].min()-padding),float(pts[:,0].max()+padding));xyax.set_ylim(float(pts[:,1].min()-padding),float(pts[:,1].max()+padding));xyax.set_aspect('equal',adjustable='box')
 xyax.set_xlabel('World X (m)');xyax.set_ylabel('World Y (m)');xyax.set_title('Actual motion during the diagnostic window',fontsize=11)
 xyax.ticklabel_format(useOffset=False,style='plain');xyax.grid(alpha=.2);xyax.text(.03,.025,'Dotted: planned path',transform=xyax.transAxes,fontsize=8.5,color='#444444',bbox=dict(facecolor='white',edgecolor='none',alpha=.85))
 erax.axvspan(20.4,20.65,color='#808080',alpha=.17,zorder=0);erax.axvline(20.585,color=colors[1],ls=':',lw=1.,zorder=1)
 erax.text(20.67,159,'Fixed cap window: 20.400–20.650 s',va='top',fontsize=9,color='#555555')
 erax.set_xlim(20.4,24);erax.set_ylim(0,165);erax.set_xlabel('Simulation time (s)');erax.set_ylabel('Distance to the planned path (mm)');erax.set_title('Suppressing the cluster reduces the later departure',fontsize=11);erax.grid(alpha=.2);erax.legend(loc='upper right',bbox_to_anchor=(1.,.86),framealpha=.95,fontsize=9)
 fig.suptitle('Original LQR + momentum observer: one straight-start causal test',fontsize=13,y=.99)
 fig.text(.02,.02,'Same initial state; only feedback joint-rate references differ. Grey band: fixed 250 ms cap; dotted line: single-row hold.\nXY circles mark 20.400 s, squares 24.000 s; error-panel dots mark peaks. Final command unheld. The ±5 rad/s diagnostic cap is not a motor rating.',fontsize=8.5,color='#444444')
 fig.tight_layout(rect=(0,.115,1,.95))
 fig.savefig(OUT/'causal_comparison.png',dpi=180);fig.savefig(OUT/'causal_comparison.pdf');plt.close(fig)
 save(OUT/'artifact_manifest.json',dict(files={str(p.relative_to(OUT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in sorted(OUT.rglob('*'))if p.is_file()}))
 print(str(OUT/'causal_comparison.png'))
if __name__=='__main__':main()
