"""Descriptive I72 figures; fixed earlier anchors, no parameter/event fitting."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,sys,textwrap
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import ListedColormap

CASES=('m100_B','p030_B','p090_B')
NAMES=('shell A','shell B','belt A','belt B')
COLORS=('#2868a9','#77a4cf','#dc8741','#d5b739')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k] for k in z.files}
def note(fig,x,y,s,size=10,width=130):
    fig.text(x,y,'\n'.join(textwrap.fill(t,width) for t in s.split('\n')),va='top',fontsize=size)
def footer(fig,s):note(fig,.055,.045,s,8,155)
def decorate(ax):
    ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)

def overview(s):
    p=s['pooled'];fig,axs=plt.subplots(2,2,figsize=(11.7,8.3));fig.subplots_adjust(left=.075,right=.965,bottom=.15,top=.81,hspace=.52,wspace=.28)
    fig.suptitle('I-72: geometry-first contact selection — spatial gain, mixed result',x=.055,y=.965,ha='left',fontsize=17,fontweight='bold')
    note(fig,.055,.915,'Configured candidate is not adopted: 4/10 gate rows fail, including two redundant coverage checks.\n'
         'Point-error guards pass; family errors rise by three and acceptance falls by 54 samples.',11,130)
    methods=['N','oldF','A','newF','C','VG_C'];labels=['Native N','Old raw F','Guarded A','New raw F','Primary C','VG fallback']
    v=[p['methods'][m]['singleton']['errors'] for m in methods];b=axs[0,0].bar(range(6),v,color=['#666666','#9aaeb9','#47759a','#e2b974','#b94d42','#aaaaaa']);axs[0,0].bar_label(b,padding=3,fontsize=9)
    axs[0,0].set_xticks(range(6),labels,rotation=15);axs[0,0].set_ylim(0,max(v)*1.23);axs[0,0].set_title('23,423 identical dominant-pressure rows',loc='left',fontsize=12);axs[0,0].set_ylabel('Assignment errors');decorate(axs[0,0])
    ax=axs[0,1];x=np.arange(3);w=.27
    for j,(m,label,color) in enumerate([('A','Guarded A','#47759a'),('C','Primary C','#b94d42')]):
        v=[s['cases'][c]['methods'][m]['singleton']['errors'] for c in CASES];b=ax.bar(x+(j-.5)*w,v,w,label=label,color=color);ax.bar_label(b,padding=3,fontsize=9)
    ax.set_xticks(x,['−1.0 rad','+0.3 rad','+0.9 rad']);ax.set_ylim(0,88);ax.set_title('Three additional errors at +0.3 rad',loc='left',fontsize=12);ax.legend(frameon=False,fontsize=9);decorate(ax)
    ax=axs[1,0];x=np.arange(3);w=.24
    for j,(m,label,color) in enumerate([('N','Native N','#666666'),('A','Guarded A','#47759a'),('C','Primary C','#b94d42')]):
        q=p['points']['methods'][m]['all_finite_primary_mm'];v=[q[k] for k in ['rms','p95','maximum']];b=ax.bar(x+(j-1)*w,v,w,label=label,color=color);ax.bar_label(b,fmt='%.2f',padding=3,fontsize=8)
    ax.set_xticks(x,['RMS','95th percentile','Maximum']);ax.set_ylim(0,38);ax.set_ylabel('Point distance (mm)');ax.set_title('All 23,423 rows; wrong families and fallback retained',loc='left',fontsize=11);ax.legend(frameon=False,fontsize=8,ncol=3);decorate(ax)
    ax=axs[1,1];c=p['acceptance']['old_ambiguous_motion'];v=[100*c['old_A_fraction'],100*c['new_fused_fraction']];b=ax.bar([0,1],v,color=['#47759a','#b94d42']);ax.bar_label(b,labels=[f'{a:.2f}%\n{n:,} accepted' for a,n in zip(v,[c['old_A_accepted'],c['new_fused_accepted']])],padding=4,fontsize=9);ax.set_xticks([0,1],['Guarded A','Geometry-first fused']);ax.set_ylim(0,110);ax.set_ylabel('Accepted (%)');ax.set_title(f"Same {c['denominator']:,} old ambiguous-motion rows",loc='left',fontsize=12);decorate(ax)
    footer(fig,'N is the recorded live estimator; A and C are offline shadows. Every point statistic here uses the same full primary rows and raw family centroid.\n'
        'All 471 mixed-pressure pair rows remain reported; this single-family selector is not a general multi-contact estimator. No new robot motion.')
    return fig

def clouds(z,data):
    fig=plt.figure(figsize=(11.7,8.3));fig.suptitle('Choose physical geometry before testing velocity',x=.055,y=.955,ha='left',fontsize=18,fontweight='bold')
    note(fig,.055,.90,'Fixed −1.0 rad events. Dots: original active candidates; colored X: nearest physical point per family.\n'
         'Coordinates are relative to the native observer point. The unchanged scorer compares those representatives.',11,129)
    for col,t in enumerate([7.875,35.205]):
        i=int(np.flatnonzero(abs(z['time_s']-t)<=1e-9)[0]);anchor=data['native_anchor_point_m'][i]
        ax=fig.add_subplot(1,2,col+1,projection='3d');ax.set_position([.04+col*.49,.25,.45,.49])
        points=data['world_points'][i];active=z['legacy_active_mask'][i] if 'legacy_active_mask'in z else data['projected_features']['valid'][i]
        for f in range(4):
            mask=active&(data['projected_features']['family_index']==f);p=(points[mask]-anchor)*1000
            if len(p):ax.scatter(p[:,0],p[:,1],p[:,2],s=13,color=COLORS[f],alpha=.45)
            row=int(z['selected_row_index'][i,f])
            if row>=0:
                p=(points[row]-anchor)*1000;ax.scatter(*p,s=75,marker='X',color=COLORS[f],edgecolor='black',linewidth=.5)
        for key,color,marker,size in [('oldF__point_m','#bb3333','o',100),('newF__point_m','#222222','s',90),('C__point_m','#8a54a2','P',90),('reference_dominant_point_m','#1a8b4d','*',150)]:
            p=(z[key][i]-anchor)*1000;ax.scatter(*p,color=color,marker=marker,s=size,facecolors='none' if marker in ['o','s'] else color)
        ax.scatter(0,0,0,s=60,color='black',marker='D');ax.set_xlabel('Δ world x (mm)',fontsize=8);ax.set_ylabel('Δ world y (mm)',fontsize=8);ax.set_zlabel('Δ world z (mm)',fontsize=8);ax.tick_params(labelsize=7);ax.view_init(elev=24,azim=-60)
        truth=NAMES[int(z['truth_family'][i])];f=NAMES[int(z['newF__family'][i])] if z['newF__family'][i]>=0 else 'unknown';accepted=bool(z['newF_accepted'][i]);ax.set_title(f'{t:.3f} s: pressure {truth}\nNew raw {f}; accepted {accepted}',fontsize=10)
    from matplotlib.lines import Line2D
    handles=[Line2D([],[],marker='o',ls='',color=c,label=n) for c,n in zip(COLORS,NAMES)]
    handles += [Line2D([],[],marker=m,ls='',color=c,label=n,markerfacecolor='none' if m in ['o','s'] else c) for n,c,m in [('Native anchor N','black','D'),('Old raw F','#bb3333','o'),('New raw F','#222222','s'),('Applied shadow C','#8a54a2','P'),('Pressure centroid','#1a8b4d','*')]]
    fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.115),ncol=5,frameon=False,fontsize=9)
    footer(fig,'The pressure centroid is evaluation-only and need not lie on the material surface. Accepted points retain their own physical family.\n'
        'Fallback C retains native N exactly. The native equivalent anchor can lie between contacts; closeness to N is not contact truth.')
    return fig

def timeline(z):
    t=z['time_s'];mask=(t>=13.27)&(t<=13.67);fig,axs=plt.subplots(3,1,figsize=(11.7,8.3),sharex=True,gridspec_kw={'height_ratios':[1.5,1,1.5]});fig.subplots_adjust(left=.13,right=.965,top=.81,bottom=.14,hspace=.3)
    fig.suptitle('Fixed +0.3 rad window: keep decisions and point error separate',x=.055,y=.955,ha='left',fontsize=17,fontweight='bold');note(fig,.055,.90,'Previously fixed I-67 anchor at 13.470 s. This event is descriptive; no parameter or threshold was chosen from it.',11,132)
    keys=['truth_family','N__family','A__family','newF__family','C__family'];image=np.array([z[k][mask]+1 for k in keys]);axs[0].imshow(image,aspect='auto',interpolation='nearest',origin='upper',extent=(t[mask][0]-.0025,t[mask][-1]+.0025,4.5,-.5),cmap=ListedColormap(['#aaaaaa']+list(COLORS)),vmin=0,vmax=4);axs[0].set_yticks(range(5),['Pressure family','Native N','Guarded A','New raw F','Primary C'])
    axs[1].step(t[mask],z['old_accepted'][mask],where='post',label='Old fused acceptance',color='#47759a');axs[1].step(t[mask],z['newF_accepted'][mask],where='post',label='New fused acceptance',color='#b94d42',ls='--');axs[1].set_yticks([0,1],['Fallback','Accepted']);axs[1].set_ylim(-.1,1.2);axs[1].legend(frameon=False,ncol=2,fontsize=9)
    for m,color in [('N','#666666'),('A','#47759a'),('C','#b94d42')]:axs[2].plot(t[mask],z[m+'__point_distance_mm'][mask],label=m,color=color)
    axs[2].set_ylabel('Point distance (mm)');axs[2].legend(frameon=False,ncol=3,fontsize=9);axs[2].set_xlabel('Saved simulation time (s)')
    for ax in axs:ax.axvline(13.470,color='#555555',ls=':',alpha=.6);decorate(ax);ax.set_xlim(13.27,13.67)
    from matplotlib.patches import Patch
    axs[0].legend(handles=[Patch(color=c,label=n) for c,n in zip(['#aaaaaa']+list(COLORS),['Ambiguous / unknown']+list(NAMES))],ncol=5,loc='upper right',bbox_to_anchor=(1,1.22),frameon=False,fontsize=8)
    footer(fig,'Pressure labels are sampled at the final 1 ms of each recorded 5 ms interval. Observer input covers the preceding interval.\n'
        'Every fallback and wrong-family output remains in the primary comparison; these plots do not demonstrate trajectory improvement.')
    return fig

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--base',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.base=a.base.resolve();a.output.mkdir(parents=True,exist_ok=False)
    files=[a.base/'evaluation/stage01/science.json']+[a.base/f'evaluation/stage01/{c}_frames.npz' for c in ['m100_B','p030_B']]+[p for p in (a.base/'inputs/stage01/m100_B').iterdir() if p.is_file()]+[a.base/'inputs/packet_io.py'];bindings={str(p):sha(p) for p in files};save(a.output/'registered_before_render.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),inputs=bindings,events='Fixedold7.875/35.205m100and13.470p030, no post-result selector fitting.'))
    sys.path.insert(0,str(a.base/'inputs'));from packet_io import load_case
    s=json.loads(files[0].read_text());z=load(files[1]);other=load(files[2]);data=load_case(a.base/'inputs/stage01','m100_B')
    figs=[('overview',overview(s)),('physical_candidates',clouds(z,data)),('fixed_window',timeline(other))]
    with PdfPages(a.output/'geometry_first_contact_report.pdf') as pdf:
        for name,fig in figs:pdf.savefig(fig);fig.savefig(a.output/(name+'.png'),dpi=135);plt.close(fig)
    assert all(sha(Path(p))==v for p,v in bindings.items());save(a.output/'render_receipt.json',dict(pages=3,source_sha256=sha(__file__),inputs_unchanged=True,outputs={p.name:sha(p) for p in a.output.iterdir() if p.is_file()}));print('Rendered3pages')

if __name__=='__main__':main()
