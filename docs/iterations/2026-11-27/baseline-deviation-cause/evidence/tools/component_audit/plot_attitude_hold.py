"""I77 one eight physical diagnostic: frozen-input XY/error comparison."""
from pathlib import Path
import hashlib,importlib.util,json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent
OUT=HERE/'eight_attitude_publication_stage01';OLD=ROOT/'results/tangent_heading_routes_20261127';CASE='eight_m100_N'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')
def main():
 OUT.mkdir(exist_ok=False)
 source=Path(__file__);metric=OLD/'evaluation/metrics.py';reportpath=HERE/'eight_attitude_reduction_stage01/report.json'
 inputs={'native':OLD/'campaign/runs'/CASE/'precision.npz','held':HERE/'eight_attitude_hold103_stage01/child/campaign/runs'/CASE/'precision.npz'}
 for p in (source,metric):
  q=OUT/'source'/p.relative_to(ROOT);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
 save(OUT/'registered_before_arrays.json',dict(iteration='I-77',sources={str(p.relative_to(ROOT)):sha(p)for p in (source,metric)},inputs={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in [*inputs.values(),reportpath]},no_case_arrays_loaded_before_registration=True,design='ActualXY and geometricdistance96.4–103, originalnative versusonefixed96.4–96.9attitudehold. Postreleaseeffects explicit; no new candidate/controller acceptance.'))
 spec=importlib.util.spec_from_file_location('_i77_plot_eight_metrics',metric);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);path=m.geometric_polyline('eight',4097);report=json.loads(reportpath.read_text())
 fig,(xyax,erax)=plt.subplots(1,2,figsize=(12,4.5),gridspec_kw=dict(width_ratios=[1,1.65]));points=[]
 for arm,color,label in [('native','#315D9F','Native baseline'),('held','#16836B','Fixed 0.5 s attitude hold')]:
  with np.load(inputs[arm],allow_pickle=False)as z:t=z['time_s'][19280:20601];xy=z['exact_qpos'][19280:20601,:2]
  error=m.polyline_distance(xy,path)*1000;expected=report['arms'][arm]['windows']['full96p4_to103']['geometric']['maximum']
  if abs(float(error.max()/1000)-expected)>1e-15:raise AssertionError('Plotpeakdiffersfromsealedreduction')
  xyax.plot(xy[:,0],xy[:,1],color=color,lw=1.8);xyax.scatter(*xy[0],color=color,s=18,zorder=4);xyax.scatter(*xy[-1],color=color,s=25,marker='s',zorder=4);points.append(xy)
  erax.plot(t,error,color=color,lw=1.9,label=f'{label}: peak {error.max():.1f} mm');j=int(error.argmax());erax.scatter(t[j],error[j],color=color,s=22,zorder=4)
 xyax.plot(path[:,0],path[:,1],color='#555555',ls=':',lw=1.6,zorder=1);pts=np.concatenate(points);pad=.025
 xyax.set_xlim(pts[:,0].min()-pad,pts[:,0].max()+pad);xyax.set_ylim(pts[:,1].min()-pad,pts[:,1].max()+pad);xyax.set_aspect('equal',adjustable='box');xyax.set_xlabel('World X (m)');xyax.set_ylabel('World Y (m)');xyax.set_title('Actual motion during the diagnostic window',fontsize=11);xyax.grid(alpha=.2);xyax.ticklabel_format(useOffset=False,style='plain');xyax.text(.03,.025,'Dotted: planned path',transform=xyax.transAxes,fontsize=8.5,color='#444444',bbox=dict(facecolor='white',edgecolor='none',alpha=.85))
 erax.axvspan(96.4,96.9,color='#808080',alpha=.17,zorder=0);erax.axvline(96.9,color='#666666',ls=':',lw=1.,zorder=1);erax.text(96.98,342,'Attitude released at 96.900 s',va='top',fontsize=9,color='#555555');erax.set_xlim(96.4,103);erax.set_ylim(0,355);erax.set_xlabel('Simulation time (s)');erax.set_ylabel('Distance to the planned path (mm)');erax.set_title('Later departure changes after the brief reference hold',fontsize=11);erax.grid(alpha=.2);erax.legend(loc='upper left',bbox_to_anchor=(.02,.88),fontsize=9,framealpha=.95)
 fig.suptitle('Original LQR + momentum observer: one eight-path causal diagnostic',fontsize=13,y=.99)
 fig.text(.02,.02,'Grey: only feedback attitude reference held (96.400–96.900 s); observer, normal, gains and other algorithms remain native.\nPost-release results include release and a changed physical contact path. Diagnostic only; XY circles/squares mark window start/end.',fontsize=8.5,color='#444444');fig.tight_layout(rect=(0,.115,1,.95));fig.savefig(OUT/'attitude_hold_comparison.png',dpi=180);fig.savefig(OUT/'attitude_hold_comparison.pdf');plt.close(fig)
 save(OUT/'artifact_manifest.json',dict(files={str(p.relative_to(OUT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in sorted(OUT.rglob('*'))if p.is_file()}));print(str(OUT/'attitude_hold_comparison.png'))
if __name__=='__main__':main()
