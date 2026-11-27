"""I58 scientific plots from full-rate portable metric projections; no simulation."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,gzip,hashlib,json,shutil,types
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def module(path):
 m=types.ModuleType('_i58_plot_math');m.__file__=str(path);exec(compile(path.read_text(),str(path),'exec'),m.__dict__);return m

def draw(stage,output):
 output.mkdir(parents=True,exist_ok=False)
 index=json.loads((stage/'portable_inputs.json').read_text());report=json.loads((stage/'report.json').read_text())
 shared_path=stage/index['sources']['shared'];shared=module(shared_path)
 inputs=[stage/'portable_inputs.json',stage/'report.json',shared_path]
 for case,p in index['cases'].items():inputs.extend(stage/p[k]for k in('projection','diagnostics','metadata'))
 for row in json.loads((stage/'artifact_manifest.json').read_text())['files']:
  if sha(stage/row['path'])!=row['sha256']:raise ValueError('Changed sealed metric stage: '+row['path'])
 shutil.copyfile(Path(__file__),output/'plot_source.py')
 (output/'registered_before_render.json').write_text(json.dumps(dict(captured_utc=datetime.now(timezone.utc).isoformat(),
  source_sha256=sha(Path(__file__)),input_sha256={str(p):sha(p)for p in inputs},
  scope='Full5ms projection lines, no downsampling. Dense PDF lines rasterized only for artifact size. Motion/priority metrics are not altered by rendering.'),indent=2)+'\n')
 colors={'B':'#2166ac','A':'#777777','P':'#b35806'}
 labels={'B':'B: native LQR','A':'A: original hard slew','P':'P: amplitude-only balance priority'}
 for phase in('m100','p030','p090'):
  if phase+'_B'not in index['cases']or phase+'_P'not in index['cases']:continue
  case_ids=[phase+'_'+variant for variant in('B','A','P')if phase+'_'+variant in index['cases']]
  fig,axes=plt.subplots(4,2,figsize=(12,13),layout='constrained')
  motion,motion_axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
  for case in case_ids:
   path=index['cases'][case];raw=shared.load(stage/path['projection']);t=raw['time_s'];sig=shared.signals(raw)
   variant=case[-1];style=dict(color=colors[variant],lw=.9,alpha=.9,rasterized=True)
   q=raw['exact_qpos'];rotation=shared.rotation(q[:,3:7]);up=rotation[:,:,2]
   ref_up=shared.rotation(raw['exact_reference_qpos'][:,3:7])[:,:,2]
   absolute=np.degrees(np.arccos(np.clip(up[:,2],-1,1)))
   relative=np.degrees(np.arccos(np.clip(np.sum(up*ref_up,axis=1),-1,1)))
   axes[0,0].plot(q[:,0],q[:,1],label=labels[variant],**style)
   axes[0,1].plot(t,1000*np.linalg.norm(q[:,:2]-raw['exact_reference_xy'],axis=1),**style)
   axes[1,0].plot(t,absolute,**style);axes[1,1].plot(t,relative,**style)
   axes[2,0].plot(t,sig['world_omega_rad_s'][:,2],**style)
   valid=sig['heading_valid'];increments=np.where(valid[1:]&valid[:-1],abs(np.diff(sig['heading_rad'])),0.)
   axes[2,1].plot(t,np.r_[0,np.cumsum(increments)]/(2*np.pi),**style)
   with gzip.open(stage/path['diagnostics'],'rt',encoding='utf8')as stream:rows=json.load(stream)
   if variant!='B':
    priority=np.array([np.asarray(r['diagnostics']['first_stage_balance_delta'])+np.asarray(r['diagnostics']['secondary_balance_delta'])for r in rows])
    axes[3,0].plot(t,np.linalg.norm(priority,axis=1),label=labels[variant],**style)
   delta=raw['exact_applied']-raw['exact_previous_applied'];caps=raw['exact_input_bounds']
   axes[3,1].plot(t[:-1],np.linalg.norm(delta[:-1]/caps[:-1],axis=1),**style)
   motion_axes[0,0].plot(t,sig['planar_speed_m_s'],**style)
   motion_axes[0,1].plot(t,np.linalg.norm(sig['linear_acceleration50ms_m_s2'],axis=1),**style)
   motion_axes[1,0].plot(t[:-1],np.max(abs(raw['exact_applied'][:-1])/caps[:-1],axis=1),**style)
   dt=np.r_[.005,np.diff(t)]
   motion_axes[1,1].plot(t[:-1],np.max(abs(delta[:-1])/(dt[:-1,None]*np.array([60,60,60,6,6])),axis=1),**style)
   if variant=='B':axes[0,0].plot(raw['exact_reference_xy'][:,0],raw['exact_reference_xy'][:,1],color='black',ls='--',lw=1,label='Timed reference')
  axes[0,0].set(xlabel='World x (m)',ylabel='World y (m)',title='Measured route');axes[0,0].set_aspect('equal',adjustable='datalim');axes[0,0].legend(fontsize=8)
  titles=[('Timed reference error','Timed position error (mm)'),('Absolute attitude','Body tilt from gravity (deg)'),
   ('Native raw-target tracking','Body-up error from raw target (deg)'),('Measured body vertical rotation','World Z angular rate (rad/s)'),
   ('Accumulated platform rotation','Absolute heading travel (turns)'),('TOTAL local transverse-response difference','||first-stage + secondary|| (rad/s)'),
   ('Command continuity (secondary diagnostic)','||actual command step / cap||')]
  for ax,(title,label)in zip(list(axes.flat)[1:],titles):ax.set(title=title,ylabel=label,xlabel='Simulation time (s)',xlim=(0,44))
  axes[3,0].legend(fontsize=8)
  for i,ax in enumerate(axes.flat):
   ax.grid(alpha=.2)
   if i:ax.axvspan(28,44,color='black',alpha=.035)
  phase_rad=report['cases'][phase+'_B']['phase_rad']
  phase_title=f'I-58 | Mars figure-eight | initial phase {phase_rad:+.1f} rad | unchanged LQR, observer, raw target and frozen gain bank'
  fig.suptitle(phase_title+'\nP prioritizes nominal transverse response inside amplitude bounds; old rate values are diagnostic for P',fontsize=12)
  fig.savefig(output/(phase+'_comparison.png'),dpi=170);fig.savefig(output/(phase+'_comparison.pdf'),dpi=170);plt.close(fig)
  for ax,(title,ylabel)in zip(motion_axes.flat,[('Measured planar speed','Planar speed (m/s)'),
   ('50 ms interval-average acceleration','Linear acceleration norm (m/s²)'),('Total actual command amplitude','max |actual input| / amplitude cap'),
   ('Old assumed rate thresholds','max |actual command step| / old rate allowance')]):
   ax.set(title=title,ylabel=ylabel,xlabel='Simulation time (s)',xlim=(0,44));ax.grid(alpha=.2);ax.axvspan(28,44,color='black',alpha=.035)
  motion_axes[1,0].axhline(1,color='black',ls=':',lw=.8);motion_axes[1,1].axhline(1,color='black',ls=':',lw=.8)
  motion.suptitle(phase_title+'\nRate limits are hard only for A. Command bounds do not bound physical acceleration.',fontsize=12)
  motion.savefig(output/(phase+'_motion.png'),dpi=170);motion.savefig(output/(phase+'_motion.pdf'),dpi=170);plt.close(motion)
 (output/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)
  for p in output.iterdir()if p.is_file()and p.name!='artifact_manifest.json']),indent=2)+'\n')

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--data-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 a=p.parse_args();draw(a.data_source.resolve(),a.output.resolve())
