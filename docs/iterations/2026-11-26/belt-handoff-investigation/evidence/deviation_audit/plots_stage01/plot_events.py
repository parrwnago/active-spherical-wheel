"""I50 publication plots from sealed saved arrays; no simulations."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;ROOT=H.parents[2];OLD=ROOT/'results/belt_capacity_sweep_20261126'
CASES=('mars_cap030','mars_cap060','mars_cap090');NAMES=('0.30 m/s','0.60 m/s','0.90 m/s');COLORS=('#225e99','#d37820','#333333');STYLES=('-','-','--')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2)+'\n')
def main():
 out=H/'plots_stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'plot_events.py')
 sources=[Path(__file__),H/'compact_stage03/summary.json'];files=[]
 for c in CASES:files +=[OLD/'campaign/runs'/c/f for f in('precision.npz','rollout.npz')]+[OLD/'analysis/stage01'/(c+'_geometry.npz')]
 hashes={str(p.relative_to(ROOT)):sha(p)for p in sources+files}
 save(out/'before_plotting.json',dict(utc=datetime.now(timezone.utc).isoformat(),input_sha256=hashes,scope='Savedarrays only; fullroute andfixed31.6–31.7/38–44s asrootrequested.0.60/0.90bodytrajectoriescoincide; allcapsretained.'))
 data=[]
 for c in CASES:
  with np.load(OLD/'campaign/runs'/c/'precision.npz')as z:p={k:z[k].copy()for k in z.files}
  with np.load(OLD/'campaign/runs'/c/'rollout.npz')as z:l={k:z[k].copy()for k in z.files}
  with np.load(OLD/'analysis/stage01'/(c+'_geometry.npz'))as z:g={k:z[k].copy()for k in z.files}
  data.append((p,l,g))
 plt.rcParams.update({'font.size':10,'axes.spines.right':False,'axes.spines.top':False,'savefig.dpi':180,'pdf.fonttype':42})
 def finish(fig,name):
  fig.savefig(out/(name+'.png'),bbox_inches='tight');fig.savefig(out/(name+'.pdf'),bbox_inches='tight');plt.close(fig)
 fig,axes=plt.subplots(2,1,figsize=(10,6.4),sharex=True,layout='constrained')
 for j,(p,l,g)in enumerate(data):
  t=p['time_s'];timed=np.linalg.norm(p['exact_qpos'][:,:2]-p['exact_reference_xy'],axis=1)*1000
  axes[0].plot(t,timed,STYLES[j],color=COLORS[j],label=NAMES[j],lw=1.4)
  axes[1].plot(t,g['local_path_distance_m']*1000,STYLES[j],color=COLORS[j],label=NAMES[j],lw=1.4)
 for ax in axes:
  ax.set_ylabel('Distance (mm)');ax.grid(alpha=.2);ax.set_xlim(0,44)
  for x,label in [(6.85,'Early departure'),(31.65,'Twitch / rate spike'),(38.64,'Late baseline >20 mm')]:ax.axvline(x,color='#888888',lw=.8,ls=':')
 axes[0].set_title('Timed tracking error');axes[1].set_title('Distance to reference branch within ±2 s');axes[1].set_xlabel('Recorded simulation time (s)')
 axes[0].legend(loc='upper left',ncol=3);axes[0].text(.45,.92,'0.60 and 0.90 body trajectories coincide',transform=axes[0].transAxes,fontsize=9)
 for x,label in [(6.85,'6.85 s'),(31.65,'31.65 s'),(38.64,'38.64 s')]:axes[1].text(x+.25,180,label,rotation=90,color='#555555',fontsize=8,va='top')
 fig.suptitle('I50: separate early departure, 32 s twitch, and late drift');finish(fig,'whole_route_errors')
 fig,axes=plt.subplots(3,1,figsize=(10,8.1),sharex=True,layout='constrained');p=data[0][0];t=p['time_s'];m=(t>=31.6-1e-9)&(t<=31.7+1e-9)
 for j,k in enumerate((0,2)):
  color=COLORS[j];label=('Shaft','Shell B')[j]
  axes[0].plot(t[m],p['exact_reference_qvel'][m,6+k],color=color,lw=1.5,label=label+' reference')
  axes[1].plot(t[m],p['exact_native_requested'][m,k],color=color,lw=1.5,label=label+' requested')
  axes[2].plot(t[m],p['exact_applied'][m,k],color=color,lw=1.5,label=label+' applied')
 for ax in axes:ax.grid(alpha=.25);ax.axvline(31.65,color='#777777',ls=':',lw=1);ax.set_xlim(31.6,31.7);ax.legend(loc='upper right',ncol=2,fontsize=9)
 axes[0].set_ylabel('Joint reference (rad/s)');axes[1].set_ylabel('Requested torque (N m)');axes[2].set_ylabel('Applied torque (N m)');axes[2].set_xlabel('Recorded simulation time (s)')
 axes[2].axhline(1.5,color=COLORS[0],ls=':',alpha=.5);axes[2].axhline(-1.5,color=COLORS[0],ls=':',alpha=.5)
 axes[2].axhline(.6,color=COLORS[1],ls=':',alpha=.5);axes[2].axhline(-.6,color=COLORS[1],ls=':',alpha=.5)
 fig.suptitle('0.30 baseline: infeasible rolling-rate references precede torque clipping\n31.650 s: observer and solver both shell B; synchronization delta = 0');finish(fig,'baseline_rate_and_torque_spike')
 fig,axes=plt.subplots(4,1,figsize=(10,10),sharex=True,layout='constrained')
 for j,(p,l,g)in enumerate(data):
  t=p['time_s'];m=t>=38-1e-9
  axes[0].plot(t[m],g['local_path_distance_m'][m]*1000,STYLES[j],color=COLORS[j],label=NAMES[j],lw=1.4)
  axes[1].plot(t[m],np.linalg.norm(p['exact_qvel'][m,:2],axis=1),STYLES[j],color=COLORS[j],label=NAMES[j],lw=1.4)
 p=data[0][0];t=p['time_s'];m=t>=38-1e-9
 axes[1].plot(t[m],np.linalg.norm(p['exact_reference_velocity_m_s'][m],axis=1),'k:',label='Desired speed',lw=1.4)
 for j,label in [(1,'Shell B'),(3,'Belt B')]:
  color=COLORS[0]if j==1 else COLORS[1]
  axes[2].plot(t[m],p['truth_patch_loads_N'][m,j],color=color,label=label+' solver',lw=1.3)
  axes[2].plot(t[m],p['exact_estimated_support_loads_N'][m,j],color=color,ls='--',label=label+' inferred',lw=.9)
 axes[3].plot(t[m],p['exact_native_requested'][m,4],color=COLORS[1],label='Belt B requested')
 axes[3].plot(t[m],p['exact_applied'][m,4],color=COLORS[0],label='Belt B applied')
 for y in [-.3,.3]:axes[3].axhline(y,color='k',ls=':',lw=.8)
 for ax in axes:ax.grid(alpha=.2);ax.set_xlim(38,44);ax.axvline(41,color='#888888',ls=':',lw=1);ax.axvline(42.915,color='#888888',ls='--',lw=1)
 axes[0].set_ylabel('Local error (mm)');axes[1].set_ylabel('Planar speed (m/s)');axes[2].set_ylabel('Baseline load (N)');axes[3].set_ylabel('Baseline belt B (m/s)');axes[3].set_xlabel('Recorded simulation time (s)')
 for ax in axes:ax.legend(loc='upper left',ncol=3,fontsize=8)
 fig.suptitle('Late error grows before belt B handoff\nDotted: reference stops at41 s; dashed: first significant belt B endpoint at42.915 s');finish(fig,'late_drift_context')
 for path,h in hashes.items():assert sha(ROOT/path)==h
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('Plots complete')
if __name__=='__main__':main()
