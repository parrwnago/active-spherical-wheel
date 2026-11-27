"""Presentation figures from sealed I68 numeric output; no model refit."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

BASE = Path(__file__).resolve().parents[1]
SOURCE = BASE/'analysis/stage02'
OUT = BASE/'publication/plots_stage01'
EVENTS = [7.875, 32.110, 35.055, 35.205]
COLORS = ['#277da8','#e07b39','#42976d','#9464b4','#6e7379']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    OUT.mkdir(exist_ok=False)
    inputs = [SOURCE/'report.json', SOURCE/'interval_frames.npz', SOURCE/'substep_frames.npz']
    seal = dict(created_utc=datetime.now(timezone.utc).isoformat(), script_sha256=sha(__file__),
        inputs={str(p):sha(p) for p in inputs}, scope='Presentation only. Metrics and fixed events are unchanged from frozen analysis.')
    (OUT/'registered_before_render.json').write_text(json.dumps(seal,indent=2)+'\n')
    report=json.loads(inputs[0].read_text())
    with np.load(inputs[1],allow_pickle=False) as z: a={k:z[k].copy() for k in z.files}
    with np.load(inputs[2],allow_pickle=False) as z: s={k:z[k].copy() for k in z.files}
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.spines.top':False,
        'axes.spines.right':False,'axes.titleweight':'bold','axes.grid':True,'grid.alpha':.18,
        'pdf.fonttype':42,'savefig.facecolor':'white'})
    names=['full_primary_time_1_to_41','broad_32_to_36','event_32.110_plus_minus_100ms','event_35.205_plus_minus_100ms']
    labels=['Primary\n1–41 s','Late region\n32–36 s','Transfer\n32.110 ± .100 s','Zero shell pressure\n35.205 ± .100 s']
    fig, ax=plt.subplots(2,2,figsize=(11.7,8.3))
    fig.subplots_adjust(top=.83,bottom=.17,hspace=.52,wspace=.32)
    fig.suptitle('Contact timing and model consistency — unchanged native LQR',x=.06,ha='left',y=.97,fontsize=17,weight='bold')
    fig.text(.06,.91,'I-68  |  Gazebo / DART Mars figure-eight  |  shaft −1 rad, yaw 45°  |  44 s',fontsize=11)
    fig.text(.06,.87,'All 59 original precision fields reproduce byte-for-byte. Contacts are evaluation only.',fontsize=10)
    x=np.arange(len(names));width=.33
    for axes,unit,title in [(ax[0,0],'force_N','Net-force discrepancy (N)'),
                             (ax[0,1],'base_moment_Nm','Base-moment discrepancy (N m)'),
                             (ax[1,0],'joint_Nm','Joint-load vector discrepancy (N m)')]:
        vals=[]
        for key in ('observer_minus_last','observer_minus_interval'):
            vals.append([report['groups'][name]['comparisons']['before'][key][unit]['vector_rms'] for name in names])
        axes.bar(x-width/2,vals[0],width,color='#9b9c9f',label='Last 1 ms only')
        axes.bar(x+width/2,vals[1],width,color='#277da8',label='All five 1 ms steps')
        axes.set_xticks(x,labels,fontsize=8);axes.set_title(title,loc='left',fontsize=11)
        axes.set_ylabel('RMS vector norm');axes.set_ylim(bottom=0)
    ax[0,0].legend(frameon=False,fontsize=8)
    ax[1,1].axis('off')
    lines=['Fixed intervals (each ends at the indicated time)']
    for event in EVENTS:
        r=report['events'][f'{event:.3f}'];mu=np.asarray(r['observer_interval_mean'])
        i=np.asarray(r['interval_before']);last=np.asarray(r['last_before'])
        lines.append(f'{event:.3f} s: joint error {np.linalg.norm(mu[6:]-last[6:]):.4f} → {np.linalg.norm(mu[6:]-i[6:]):.4f} N m')
    lines += ['', '32.110 s: shell B → belt B within the same 5 ms.',
              '35.205 s: zero shell pressure, but shell force remains.', '',
              'Smaller residual is a consistency result.',
              'No new estimator or tracking improvement is tested.']
    ax[1,1].text(0,1,'\n'.join(lines),va='top',fontsize=9,linespacing=1.5)
    fig.text(.06,.055,'Primary geometry: pre-solve pose. Post-solve sensitivity and all numeric values are retained in the report.\n'
        'Reference maps published point forces into generalized loads; contact couples and total constraint wrench are unavailable.',fontsize=8,color='#444')
    fig.savefig(OUT/'interval_comparison.pdf');fig.savefig(OUT/'overview.png',dpi=155);plt.close(fig)
    with PdfPages(OUT/'event_windows.pdf',metadata={'Title':'I-68 fixed contact intervals','Subject':'Unchanged native LQR, evaluation-only force mapping'}) as pdf:
        for event in EVENTS:
            mask=np.abs(a['time_s']-event)<=.10000001;sub=np.abs(s['time_s']-event)<=.10500001
            t=a['time_s'][mask];ts=s['time_s'][sub]
            fig,axes=plt.subplots(6,1,figsize=(10.2,12),sharex=True)
            fig.subplots_adjust(top=.9,bottom=.09,hspace=.28,left=.11,right=.97)
            fig.suptitle(f'Fixed event: interval ending {event:.3f} s',x=.11,ha='left',fontsize=17,weight='bold',y=.97)
            subtitle='Temporal shell-to-belt transfer' if event==32.110 else ('Zero shell pressure; nonzero reported shell tangential force' if event in (35.055,35.205) else 'Good-case control; temporal shell B to shell A transfer')
            fig.text(.11,.93,subtitle+'  |  all curves from the unchanged baseline',fontsize=10)
            for k,name in enumerate(['Shell A','Shell B','Belt A','Belt B','Fork']):
                axes[0].plot(ts,s['before_pressure_by_family_N'][sub,k],color=COLORS[k],label=name,lw=1)
            axes[0].plot(ts,np.linalg.norm(s['before_family_generalized'][sub,1,:3],axis=1),
                color=COLORS[1],ls='--',lw=.9,label='Shell B force norm')
            axes[0].set_ylabel('Normal load /\nshell force (N)');axes[0].legend(ncol=3,frameon=False,fontsize=8,loc='upper right')
            for j,name in enumerate(['Shaft','Shell A','Shell B']):
                axes[j+1].plot(t,a['observer_interval_mean'][mask,6+j],color='#24282d',label='Raw observer',lw=1.4)
                axes[j+1].plot(t,a['interval_before'][mask,6+j],color='#277da8',label='Five-step map',lw=1.2)
                axes[j+1].plot(t,a['last_before'][mask,6+j],color='#d08b3c',label='Last-step map',lw=.9,ls='--')
                axes[j+1].set_ylabel(name+' load\n(N m)')
            axes[1].legend(ncol=3,frameon=False,fontsize=8,loc='upper right')
            axes[4].plot(t,a['before_slip_rms_m_s'][mask],color='#277da8',label='Pressure RMS: pre-solve',lw=1.2)
            axes[4].plot(t,a['after_slip_rms_m_s'][mask],color='#d08b3c',label='Pressure RMS: post-solve',ls='--',lw=1)
            axes[4].set_ylabel('Material slip\n(m/s)');axes[4].legend(ncol=2,frameon=False,fontsize=8)
            axes[5].plot(t,a['intrinsic_to_interval_normal_deg'][mask],color='#9464b4',label='Intrinsic prior',lw=1.2)
            axes[5].plot(t,a['live_to_interval_normal_deg'][mask],color='#42976d',label='Live filtered estimate',lw=1)
            axes[5].set_ylabel('Normal mismatch\n(degrees)');axes[5].set_xlabel('Simulation time (s)');axes[5].legend(ncol=2,frameon=False,fontsize=8)
            for axes_ in axes:
                axes_.axvline(event,color='#555',ls=':',lw=.8)
                axes_.axvspan(event-.005,event,color='#dce7ee',alpha=.4)
            fig.text(.11,.026,'Shading: the five-step interval for the named observer sample. Slip is a kinematic diagnostic at solver contact points.\n'
                'Force-map reference omits unavailable contact couples. These plots diagnose model consistency, not control gains.',fontsize=8,color='#444')
            pdf.savefig(fig)
            fig.savefig(OUT/f'event_{event:.3f}.png',dpi=115)
            plt.close(fig)
    result=dict(files={p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in OUT.iterdir() if p.is_file()},pages=dict(interval_comparison=1,event_windows=4))
    (OUT/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT),pages=5)))


if __name__=='__main__':
    main()
