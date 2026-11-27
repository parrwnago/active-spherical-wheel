"""I71 descriptive figures from frozen evaluation; never selects parameters."""
from pathlib import Path
import argparse, hashlib, json, re
from datetime import datetime, timezone
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import frozen_i70_renderer as style

CASES=('m100_B','p030_B','p090_B')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k] for k in z.files}

def overview(s):
    pooled=s['pooled'];old=s['parent_I70_comparison']['pooled']
    fig,axs=plt.subplots(2,2,figsize=(11.7,8.3));fig.subplots_adjust(left=.075,right=.965,bottom=.15,top=.81,hspace=.5,wspace=.28)
    fig.suptitle('I-71: remove current pair states, keep interval-transfer evidence',x=.055,y=.965,ha='left',fontsize=17,fontweight='bold')
    decision='STOP THIS HMM VERSION' if not s['continuation_all_gates_passed'] else 'CONTINUATION CHECKS PASS; NO ADOPTION'
    passed=sum(g['passed'] for g in s['continuation_gates']);adopt=sum(g['passed'] for g in s['gates'])
    style.text(fig,.055,.915,f'{decision}. Continuation checks {passed}/8; original adoption gates {adopt}/16.\n'
        'Same three saved starts; no new physics, fits, geometry or controller changes.',11,132)
    labels=['Live N','Guarded A','I-70 full','Frame','History','+ edge']
    entries=[pooled['methods'][m]['singleton'] for m in ('N','A')]+[old['hmm']['I70_singleton']]+[pooled['methods'][m]['singleton'] for m in ('emission','state_hmm','hmm')]
    ax=axs[0,0];u=np.array([e['unknown'] for e in entries]);pair=np.array([e['pair_output'] for e in entries]);wrong=np.array([e['errors'] for e in entries])-u-pair
    ax.bar(range(6),wrong,color='#b94d42',label='Wrong singleton');ax.bar(range(6),u,bottom=wrong,color='#aaaaaa',label='Unknown');ax.bar(range(6),pair,bottom=wrong+u,color='#d9ae65',label='Pair')
    maximum=max(e['errors'] for e in entries)
    for i,e in enumerate(entries):ax.text(i,e['errors']+maximum*.025,str(e['errors']),ha='center',fontsize=9)
    ax.set_xticks(range(6),labels,rotation=15);ax.set_ylim(0,maximum*1.38);ax.set_ylabel('Primary assignment errors');ax.set_title('Same 23,423 dominant-pressure rows',loc='left',fontsize=12);ax.legend(fontsize=8,ncol=3,frameon=False);style.decorate(ax)
    ax=axs[0,1];x=np.arange(3);width=.24;maxbar=0
    for j,(method,label,color) in enumerate([('A','Guarded A','#666666'),('emission','Restricted frame','#90b6cf'),('hmm','Restricted history + edge','#b94d42')]):
        v=[s['cases'][c]['methods'][method]['singleton']['errors'] for c in CASES];maxbar=max(maxbar,max(v));b=ax.bar(x+(j-1)*width,v,width,label=label,color=color);ax.bar_label(b,padding=3,fontsize=8)
    ax.set_xticks(x,['−1.0 rad','+0.3 rad','+0.9 rad']);ax.set_ylim(0,maxbar*1.45);ax.set_title('Continuation requires no phase regression',loc='left',fontsize=12);ax.legend(fontsize=8,frameon=False);style.decorate(ax)
    ax=axs[1,0];methods=['A','emission','hmm'];v=[pooled['timing'][m]['absolute_latency_s']['p95'] for m in methods]
    style.bars(ax,[1000*q if q is not None else 0 for q in v],['Guarded A','Frame','+ edge'],['#666666','#90b6cf','#3975a5']);ax.set_ylabel('Absolute transition latency p95 (ms)');ax.set_title('Pressure timing; misses reported in full record',loc='left',fontsize=12)
    ax=axs[1,1];q=pooled['paired_hmm_A_points'];x=np.arange(2);w=.32
    for offset,key,label,color in [(-.5,'A_mm','Guarded A','#666666'),(.5,'hmm_mm','Restricted history + edge','#3975a5')]:
        vals=[q[key]['rms'],q[key]['maximum']];b=ax.bar(x+offset*w,vals,w,label=label,color=color);ax.bar_label(b,fmt='%.2f',padding=3,fontsize=9)
    ax.set_xticks(x,['RMS','Maximum']);ax.set_ylabel('Point distance (mm)');ax.set_ylim(0,max(q['A_mm']['maximum'],q['hmm_mm']['maximum'])*1.3);ax.set_title(f"Same {q['count']:,} finite-singleton rows",loc='left',fontsize=12);ax.legend(fontsize=8,frameon=False);style.decorate(ax)
    h=pooled['methods']['hmm'];style.footer(fig,f"Restricted model cannot represent any of the 471 true pair-pressure rows: all remain errors in the loaded-set denominator.\nPrimary unknowns: {h['singleton']['unknown']}; point coverage: {100*q['hmm_finite_singleton_coverage']:.2f}%. Pressure labels are not complete interval-force truth.")
    return fig

def explanation():
    fig=plt.figure(figsize=(11.7,8.3));fig.suptitle('Separate recent force history from current support',x=.055,y=.95,ha='left',fontsize=19,fontweight='bold')
    style.text(fig,.055,.89,'The observer reports the previous 5 ms. A transfer can leave evidence of two bodies even when the current endpoint has only one support family.',13,112)
    for k,(title,both) in enumerate([('Illustrative transfer: A exits, B enters',False),('Illustrative simultaneous pair: both remain loaded',True)]):
        ax=fig.add_axes([.10+k*.47,.55,.36,.20]);ax.set_title(title,fontsize=11,loc='left')
        ax.hlines(1,0,1 if both else .5,color='#2868a9',lw=12);ax.hlines(0,0 if both else .5,1,color='#dc8741',lw=12)
        ax.axvline(1,color='#333333',ls='--');ax.set_xlim(-.05,1.08);ax.set_ylim(-.7,1.7);ax.set_yticks([0,1],['Family B','Family A']);ax.set_xticks([0,1],['t − 5 ms','t now']);ax.spines[['top','right','left']].set_visible(False);ax.tick_params(axis='y',length=0)
    style.text(fig,.055,.46,'What this diagnostic changes',13,fontweight='bold')
    style.text(fig,.055,.41,'• Current answers: shell A, shell B, belt A, belt B, or unknown; rolling/slip modes remain.\n'
        '• Transfer evidence: keep the original two-family force score for transitions between different singleton families.\n'
        '• Keep holding probabilities exactly; redistribute only allowed jump destinations. Prior odds, noise, cost scales, points and fit results remain fixed.\n'
        '• Compare independent-frame decisions, history without transfer edges, and primary history with transfer edges.\n'
        '• Keep genuine two-family pressure samples in the score: this restricted model necessarily fails them.',12,124)
    style.footer(fig,'The timelines are illustrations, not measured substep reconstructions. Original logs provide sampled final-1-ms pressure labels every 5 ms.\nThis is a diagnostic restriction, not a general multi-contact estimator or an adopted controller change.')
    return fig

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--base',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    files=[a.base/'evaluation/stage01/science.json',a.base/'evaluation/stage01/p030_B_frames.npz',Path(__file__).with_name('frozen_i70_renderer.py')]
    inputs={str(p):sha(p) for p in files};save(a.output/'registered_before_render.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),inputs=inputs,event='Fixed prior I70 longest erroneous-pair window15.725..15.765s in p030; descriptive view, no new event selection or tuning.'))
    s=json.loads(files[0].read_text());z=load(files[1]);event=dict(start_s=15.725,end_s=15.765)
    figs=[('overview',overview(s)),('interval_vs_endpoint',explanation()),('fixed_transfer',style.event_page(z,event,'Previously problematic +0.3 rad transfer window',
        'Same I-70 event at 15.725–15.765 s. Endpoint pair states are deliberately disabled; all force-fit and selected-point inputs remain frozen.'))]
    with PdfPages(a.output/'endpoint_pair_ablation_report.pdf') as pdf:
        for name,fig in figs:pdf.savefig(fig);fig.savefig(a.output/(name+'.png'),dpi=135);plt.close(fig)
    assert all(sha(Path(p))==d for p,d in inputs.items())
    save(a.output/'render_receipt.json',dict(pages=3,inputs_unchanged=True,source_sha256=sha(__file__),outputs={p.name:sha(p) for p in a.output.iterdir() if p.is_file()}));print('Rendered3pages')

if __name__=='__main__':main()
