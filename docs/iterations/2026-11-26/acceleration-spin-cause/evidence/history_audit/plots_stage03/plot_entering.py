"""Render already registered I54 saved entering-history series, no dynamics."""
from pathlib import Path
import hashlib,json,csv,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent;ROOT=H.parents[2]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def up(q):
 q=q/np.linalg.norm(q,axis=1)[:,None];v=np.tile([0.,0.,1.],(len(q),1));c=np.cross(q[:,1:],v);return v+2*q[:,:1]*c+2*np.cross(q[:,1:],c)

def main():
 out=H/'plots_stage03';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'plot_entering.py')
 reg=json.loads((H/'registered_before_arithmetic.json').read_text());raw=ROOT/reg['case_paths']['m100_R']/'precision.npz';csvpath=H/'entering_stage02/all_samples.csv';inputs=[raw,csvpath,H/'entering_stage02/report.json',H/'plots_stage02_readable/artifact_manifest.json',H/'final_artifact_manifest.json']
 (out/'provenance.json').write_text(json.dumps(dict(source_sha256=sha(__file__),input_sha256={str(p.relative_to(ROOT)):sha(p)for p in inputs},scope='Annotation-only stage03: same data/math/layout as stage02; move third-panel annotation below legend with va=top. Parent sealed report/manifests bound; no new scientific arithmetic or physics.'),indent=2)+'\n')
 with raw.open('rb')as f:
  with np.load(f,allow_pickle=False)as z:t=z['time_s'].copy();q=z['exact_qpos'][:,3:7].copy();r=z['exact_reference_quat'].copy();wref=z['exact_feedback_reference_qvel'][:,3:6].copy()
 m=(t>=31.4-1e-9)&(t<=31.745+1e-9);x=t[m];a=up(q)[m];b=up(r)[m]
 rows=[r for r in csv.DictReader(csvpath.open())if r['case']=='m100_R'];assert len(rows)==len(x)
 plt.rcParams.update({'font.size':11,'axes.grid':True,'grid.alpha':.22})
 fig,axes=plt.subplots(3,1,figsize=(10.4,9.0),sharex=True,constrained_layout=True)
 fig.suptitle('I54 | Why relative lean rises before the local reference guard\nSaved m100 R history; recorded 31.400–31.745 s',fontsize=15)
 axes[0].plot(x,[float(r['absolute_tilt_deg'])for r in rows],label='Actual tilt from vertical',color='#146da8',lw=2)
 axes[0].plot(x,[float(r['desired_absolute_tilt_deg'])for r in rows],label='Desired tilt from vertical',color='#e69222',ls='--',lw=2)
 axes[0].plot(x,[float(r['relative_lean_deg'])for r in rows],label='Relative lean error',color='#a63446',lw=1.8)
 axes[0].set_ylabel('Angle (deg)');axes[0].legend(loc='upper left',ncol=1)
 for j,color in enumerate(['#146da8','#47985f']):
  axes[1].plot(x,np.degrees(np.arctan2(a[:,j],a[:,2])),label=f'Actual up {"XY"[j]} inclination',color=color,lw=2)
  axes[1].plot(x,np.degrees(np.arctan2(b[:,j],b[:,2])),label=f'Desired up {"XY"[j]} inclination',color=color,ls='--',lw=2)
 axes[1].set_ylabel('Signed up-vector inclination\natan2(up X/Y,up Z), deg');axes[1].legend(loc='lower left',ncol=2)
 axes[2].plot(x,[float(r['actual_up_rate_5ms_deg_s'])for r in rows],label='Actual up-vector change / 5 ms',color='#146da8',lw=2)
 axes[2].plot(x,[float(r['desired_up_rate_5ms_deg_s'])for r in rows],label='Desired up-vector change / 5 ms',color='#e69222',ls='--',lw=2)
 axes[2].plot(x,np.degrees(np.linalg.norm(wref[m],axis=1)),label='Supplied angular-velocity reference norm',color='#777777',ls=':',lw=2)
 axes[2].set_ylabel('Rate (deg/s)');axes[2].set_xlabel('Recorded time (s)');axes[2].legend(loc='upper left')
 for ax in axes:
  ax.axvline(31.685,color='#a63446',alpha=.5,ls=':');ax.axvline(31.745,color='#5a417d',lw=2);ax.set_xlim(31.4,31.745)
 axes[2].text(.02,.55,'Target moves quickly; supplied\nangular-rate reference stays zero.\nObservation, not unique cause.',transform=axes[2].transAxes,ha='left',va='top',fontsize=10,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
 fig.savefig(out/'actual_vs_desired_up.png',dpi=165);fig.savefig(out/'actual_vs_desired_up.pdf');plt.close(fig)
 (out/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=p.name,bytes=p.stat().st_size,sha256=sha(p))for p in sorted(out.iterdir())if p.is_file()and p.name!='artifact_manifest.json']),indent=2)+'\n')
 print('saved',out/'actual_vs_desired_up.png')
if __name__=='__main__':main()
