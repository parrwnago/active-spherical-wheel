"""Descriptive I70 report. No fitting, inference changes or event cherry-picking."""
from pathlib import Path
import argparse, hashlib, json, textwrap
from datetime import datetime, timezone
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import ListedColormap

CASES=('m100_B','p030_B','p090_B')
METHODS=('N','F','A','emission','state_hmm','hmm')
LABELS=('Live N','Raw F','Guarded A','Frame','History','History + edge')
BITS=(1,2,4,8,3,5,9,6,10,12,0)
SUPPORT=('shell A','shell B','belt A','belt B','shell A+B','shell A + belt A',
         'shell A + belt B','shell B + belt A','shell B + belt B','belt A+B','unknown')
PALETTE=('#2868a9','#6699cc','#dc8741','#d5b739','#579b84','#a17bb5',
         '#e6a7bb','#9b6b43','#a7b84c','#70baba','#aaaaaa')

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v): Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z: return {k:z[k] for k in z.files}
def text(fig,x,y,s,size=11,width=103,**kw):
    return fig.text(x,y,'\n'.join(textwrap.fill(part,width) for part in s.split('\n')),
                    va='top',fontsize=size,**kw)
def footer(fig,s): text(fig,.055,.043,s,8,150,color='#555555')
def decorate(ax):
    ax.spines[['top','right']].set_visible(False)
    ax.grid(axis='y',alpha=.2)
    ax.set_axisbelow(True)
def bars(ax,values,labels,colors=None):
    b=ax.bar(np.arange(len(values)),values,color=colors or '#3975a5')
    ax.set_xticks(np.arange(len(values)),labels)
    for patch,value in zip(b,values):
        ax.annotate(f'{value:g}',(patch.get_x()+patch.get_width()/2,value),
                    xytext=(0,4),textcoords='offset points',ha='center',fontsize=9)
    ax.margins(y=.18);decorate(ax)

def overview(science):
    p=science['pooled'];fig,axs=plt.subplots(2,2,figsize=(11.7,8.3))
    fig.subplots_adjust(left=.075,right=.965,bottom=.16,top=.82,hspace=.52,wspace=.28)
    fig.suptitle('I-70: temporal contact estimation — keep the live baseline',
                 x=.055,y=.965,ha='left',fontsize=18,fontweight='bold')
    text(fig,.055,.915,'Three saved Mars figure-eight starts; offline shadow only. Six of sixteen adoption gates fail.\n'
         'Unknown and pair outputs remain visible and count as errors on dominant-singleton labels.',11,127)
    ax=axs[0,0];x=np.arange(3);width=.19
    for j,(method,label,color) in enumerate(zip(('A','emission','state_hmm','hmm'),
            ('Guarded A','Frame','History','History + edge'),('#555555','#90b6cf','#ddb16b','#b94d42'))):
        v=[science['cases'][c]['methods'][method]['singleton']['errors'] for c in CASES]
        b=ax.bar(x+(j-1.5)*width,v,width,label=label,color=color)
        ax.bar_label(b,padding=2,fontsize=8)
    ax.set_xticks(x,('−1.0 rad','+0.3 rad','+0.9 rad'));ax.set_ylabel('Incorrect dominant-family decisions')
    ax.set_title('Every starting phase regresses',loc='left',fontsize=12)
    ax.legend(fontsize=8,ncol=2,frameon=False);ax.set_ylim(0,520);decorate(ax)
    ax=axs[0,1];wrong=[];unknown=[];pair=[]
    for m in METHODS:
        d=p['methods'][m]['singleton'];unknown.append(d['unknown']);pair.append(d['pair_output'])
        wrong.append(d['errors']-d['unknown']-d['pair_output'])
    ax.bar(range(6),wrong,color='#b94d42',label='Wrong singleton')
    ax.bar(range(6),unknown,bottom=wrong,color='#b7b7b7',label='Unknown')
    ax.bar(range(6),pair,bottom=np.array(wrong)+unknown,color='#deb36a',label='Pair')
    for j,m in enumerate(METHODS):
        n=p['methods'][m]['singleton']['errors'];ax.text(j,n+12,str(n),ha='center',fontsize=9)
    ax.set_xticks(range(6),('N','F','A','Frame','History','+ edge'))
    ax.set_title('Pooled errors: 23,423 identical rows',loc='left',fontsize=12)
    ax.legend(fontsize=8,ncol=3,frameon=False);ax.set_ylim(0,805);decorate(ax)
    ax=axs[1,0];q=p['paired_hmm_A_points'];x=np.array([0.,1.]);w=.32
    for offset,key,label,color in [(-.5,'A_mm','Guarded A','#666666'),(.5,'hmm_mm','History + edge','#3975a5')]:
        v=[q[key]['rms'],q[key]['maximum']];b=ax.bar(x+offset*w,v,w,label=label,color=color)
        ax.bar_label(b,fmt='%.2f',padding=3,fontsize=9)
    ax.set_xticks(x,('RMS','Maximum'));ax.set_ylabel('Point distance (mm)');ax.set_ylim(0,36)
    ax.set_title('Same 22,999 finite-singleton rows',loc='left',fontsize=12)
    ax.legend(fontsize=8,frameon=False);decorate(ax)
    ax=axs[1,1];m=('A','emission','hmm')
    bars(ax,[1000*p['timing'][k]['absolute_latency_s']['p95'] for k in m],
         ['Guarded A','Frame','History + edge'],['#666666','#90b6cf','#3975a5'])
    ax.set_ylabel('Absolute pressure-transition latency p95 (ms)')
    ax.set_title('All three acquire 71/71 defined transitions',loc='left',fontsize=12)
    ax.set_ylim(0,50)
    footer(fig,'A is an earlier offline guarded estimator, not the live controller. Point coverage: 98.19%; missing outputs are not scored as zero error.\n'
        'Set Brier: A 0.04980 → 0.03790; loaded set errors: 595 → 582 / 23,894. These small gains do not pass the joint adoption criteria.')
    return fig

def model_page():
    fig=plt.figure(figsize=(11.7,8.3));fig.suptitle('What the prototype does',x=.055,y=.95,ha='left',fontsize=19,fontweight='bold')
    text(fig,.055,.89,'Instead of choosing one contact independently at every frame, keep several explanations alive and update them from current evidence and past belief.',13,112)
    boxes=[(.055,.70,'Measured state + past applied command\nMomentum observer: previous 5 ms load'),
           (.355,.70,'Known shell/belt geometry\nMaterial velocity at candidate points'),
           (.655,.70,'Physical force fits\nSingles and two-family pairs')]
    for x,y,label in boxes:
        fig.text(x,y,label,va='center',fontsize=10,bbox=dict(boxstyle='round,pad=.7',facecolor='#eaf0f5',edgecolor='#71859b'))
    text(fig,.055,.60,'21 discrete modes: four singleton and six two-family supports, each with rolling/slipping explanations, plus unknown.\n'
         'Continuous forces are fitted with unilateral and friction constraints. Candidate points stay on a physical shell or belt.\n'
         'A causal hidden Markov filter combines those fits with its previous beliefs. Special transition evidence can explain a previous-interval load on an exiting body without claiming that body is still touching now.',12,123)
    text(fig,.055,.37,'Important assumptions',13,fontweight='bold')
    text(fig,.055,.325,'• Contact normals and the candidate shortlist come from inferred local geometry; no terrain or contact truth enters inference.\n'
         '• Noise scales, slip scale, friction 0.7, dwell times and 20 ms evidence correlation are engineering assumptions, not learned/calibrated sensor statistics.\n'
         '• Beliefs are conditional model scores. Point covariance describes only the shortlisted alternatives, not all physical uncertainty.\n'
         '• Two different families can be represented. General three-contact support or multiple points on one family cannot.\n'
         '• The inference has no future samples. It estimates current support; it does not predict unseen terrain.',11,130)
    footer(fig,'Live control stays gain-scheduled LQR + momentum observer + geometry support + rolling/holding feedforward + 120 ms kinematic preparation.\n'
        'This research prototype is opt-in and does not command the robot. No new Gazebo motion or tracking improvement is claimed.')
    return fig

def longest_pair(frames):
    candidates=[]
    for ci,c in enumerate(CASES):
        z=frames[c];t=z['time_s'];bits=z['hmm__bits'];truth=z['truth_family']
        mask=z['primary_mask']&z['truth_dominant']&np.isin(bits,[3,5,9,6,10,12])
        start=None
        for i in range(len(t)+1):
            active=i<len(t) and mask[i]
            cont=active and start is not None and bits[i]==bits[i-1] and truth[i]==truth[i-1] and abs(t[i]-t[i-1]-.005)<1e-8
            if start is not None and not cont:
                candidates.append((i-start,ci,start,i-1));start=None
            if active and start is None:start=i
    n,ci,a,b=sorted(candidates,key=lambda x:(-x[0],x[1],x[2]))[0]
    return dict(case=CASES[ci],start_index=a,end_index=b,start_s=float(frames[CASES[ci]]['time_s'][a]),
                end_s=float(frames[CASES[ci]]['time_s'][b]),rows=n,selector='Longest constant-support erroneous pair spell on primary dominant rows; ties case order then earliest time. Descriptive post-result selector.')

def event_page(z,event,title,note):
    t=z['time_s'];lo=event['start_s']-.2;hi=event['end_s']+.2;mask=(t>=lo)&(t<=hi)
    fig,axes=plt.subplots(4,1,figsize=(11.7,8.3),sharex=True,gridspec_kw={'height_ratios':[1,1.1,1.25,1]})
    fig.subplots_adjust(left=.14,right=.965,bottom=.14,top=.82,hspace=.24)
    fig.suptitle(title,x=.055,y=.955,ha='left',fontsize=17,fontweight='bold')
    text(fig,.055,.904,note,10.5,136)
    for j,label in enumerate(SUPPORT[:4]):axes[0].plot(t[mask],z['truth_shares'][mask,j],label=label,color=PALETTE[j])
    axes[0].set_ylabel('Truth normal\nload share');axes[0].set_ylim(-.03,1.07)
    axes[0].legend(ncol=4,loc='upper right',frameon=False,fontsize=8)
    rows=[]
    for key in ['truth_support_bits','A__bits','emission__bits','state_hmm__bits','hmm__bits']:
        rows.append(np.array([BITS.index(int(b)) if int(b) in BITS else 10 for b in z[key][mask]]))
    axes[1].imshow(np.array(rows),aspect='auto',interpolation='nearest',origin='upper',
        extent=(t[mask][0]-.0025,t[mask][-1]+.0025,4.5,-.5),cmap=ListedColormap(PALETTE),vmin=0,vmax=10)
    axes[1].set_yticks(range(5),['Truth set','Guarded A','Frame','History','History + edge'])
    present=sorted(set(np.array(rows).ravel()))
    from matplotlib.patches import Patch
    axes[1].legend(handles=[Patch(color=PALETTE[k],label=SUPPORT[k]) for k in present],ncol=3,fontsize=7,
        loc='upper right',bbox_to_anchor=(1,1.45),frameon=True)
    prob=z['hmm__probability'];inclusion=np.zeros((len(t),4))
    for k,b in enumerate(BITS):
        for j in range(4):
            if b&(1<<j):inclusion[:,j]+=prob[:,k]
    for j in range(4):axes[2].plot(t[mask],inclusion[mask,j],label=SUPPORT[j],color=PALETTE[j])
    axes[2].plot(t[mask],prob[mask,10],color='#666666',ls='--',label='unknown')
    axes[2].set_ylim(-.03,1.07);axes[2].set_ylabel('HMM family inclusion\n(not exclusive probabilities)')
    axes[2].legend(ncol=5,fontsize=8,frameon=False,loc='upper right')
    for key,label,color in [('A','Guarded A','#555555'),('hmm','History + edge','#b94d42')]:
        axes[3].plot(t[mask],z[key+'__point_distance_mm'][mask],label=label,color=color)
    axes[3].set_ylabel('Singleton point\ndistance (mm)');axes[3].legend(frameon=False,fontsize=8)
    for ax in axes:
        decorate(ax);ax.set_xlim(lo,hi);ax.axvspan(event['start_s']-.0025,event['end_s']+.0025,color='#777777',alpha=.08)
    axes[3].set_xlabel('Saved simulation time (s)')
    footer(fig,'Truth is sampled positive normal pressure, not every force-bearing body or first physical touch. Blank point traces are unknown/pair outputs.\n'
        'Windows are descriptive views of already evaluated recordings; no parameters were tuned using these events.')
    return fig

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--base',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    files=[a.base/'evaluation/stage01/science.json']+[a.base/f'evaluation/stage01/{c}_frames.npz' for c in CASES]
    registration=dict(registered_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),
        inputs={str(p):sha(p) for p in files},purpose='Descriptive rendering after canonical results; no inferential/tuning claim.',
        events=['Longest erroneous pair spell over all primary dominant rows, tie case/time.',
                'Largest HMM singleton point-distance row, tie case/time; 0.2s padding.'])
    write(a.output/'registered_before_render.json',registration)
    science=json.loads(files[0].read_text());frames={c:load(a.base/f'evaluation/stage01/{c}_frames.npz') for c in CASES}
    pair=longest_pair(frames)
    out=[]
    for ci,c in enumerate(CASES):
        z=frames[c];mask=z['primary_mask']&z['truth_dominant']&z['hmm__point_valid']
        for i in np.flatnonzero(mask):out.append((float(z['hmm__point_distance_mm'][i]),ci,int(i)))
    distance,ci,i=sorted(out,key=lambda x:(-x[0],x[1],x[2]))[0];c=CASES[ci];time=float(frames[c]['time_s'][i])
    point=dict(case=c,start_s=time,end_s=time,index=i,distance_mm=distance,selector=registration['events'][1])
    write(a.output/'selected_descriptive_events.json',dict(longest_pair=pair,worst_point=point))
    figs=[('overview',overview(science)),('model',model_page()),
          ('longest_pair',event_page(frames[pair['case']],pair,'Longest erroneous pair spell: '+pair['case'],
            f"Mechanically selected after evaluation: {pair['rows']} consecutive 5 ms rows, {pair['start_s']:.3f}–{pair['end_s']:.3f} s. Truth and emitted pair remain fixed within the highlighted spell.")),
          ('worst_point',event_page(frames[c],point,'Largest HMM point error: '+c,
            f'Mechanically selected after evaluation: {distance:.3f} mm at {time:.3f} s. The point comparison retains wrong-family singleton outputs.'))]
    with PdfPages(a.output/'temporal_contact_report.pdf') as pdf:
        for name,fig in figs:
            pdf.savefig(fig);fig.savefig(a.output/(name+'.png'),dpi=135);plt.close(fig)
    unchanged={str(p):sha(p)==registration['inputs'][str(p)] for p in files}
    assert all(unchanged.values())
    write(a.output/'render_receipt.json',dict(pages=4,source_sha256=sha(__file__),inputs_unchanged=unchanged,
        outputs={p.name:sha(p) for p in a.output.iterdir() if p.is_file() and p.name!='render_receipt.json'}))
    print(json.dumps(dict(pages=4,pair=pair,point=point)))

if __name__=='__main__':main()
