"""I-65 cached-result figures only: no inference, fitting or decision changes."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,math,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

CASES=('m100_B','p030_B','p090_B')
SCORERS=('maxabs','euclidean','shared_covariance')
METHOD_NAMES=('Max absolute','Euclidean','Shared covariance')
COLORS=('#253E54','#7A8696','#B77533','#11837A')
F_COLOR='#6954A0';N_COLOR='#C08D24'
FAMILIES=('shell A','shell B','belt A','belt B')
PHASE={'m100_B':'−1.0 rad','p030_B':'+0.3 rad','p090_B':'+0.9 rad'}
RANK={-1:'F',1:'Nalt',0:'tie',2:'ambiguous',3:'unknown'}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False) as value:return {key:value[key].copy() for key in value.files}
def style():
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titlesize':12,
        'axes.labelsize':10,'axes.spines.top':False,'axes.spines.right':False,
        'axes.edgecolor':'#B7C0C8','axes.labelcolor':'#253E54','text.color':'#253E54',
        'xtick.color':'#45586A','ytick.color':'#45586A','figure.facecolor':'white',
        'axes.facecolor':'white','savefig.facecolor':'white','pdf.fonttype':42,'ps.fonttype':42})
def grid(ax):ax.grid(axis='y',color='#E5E9ED',linewidth=.7);ax.set_axisbelow(True)
def family(index):return FAMILIES[index] if 0<=index<4 else 'unknown'


def overview(science,protocol,out):
    baseline=science['shadows']['M1']['maxabs']['baseline'];A=baseline['A'];N=baseline['N']
    rows=[A]+[science['shadows']['M1'][name]['metrics'] for name in SCORERS]
    labels=['Original A','Max absolute','Euclidean','Covariance'];x=np.arange(4)
    fig=plt.figure(figsize=(14,8.8));gs=fig.add_gridspec(2,3,left=.065,right=.975,bottom=.15,top=.79,wspace=.34,hspace=.67)
    fig.text(.065,.95,'I-65  |  Shared residual covariance',fontsize=23,weight='bold')
    fig.text(.065,.904,'Improves the relative scoring controls; the original accepted/fallback policy still performs better.',fontsize=13)
    fig.text(.065,.862,'Three previously examined Mars figure-eight episodes • held-out whole-episode scores • no new driving or live changes',fontsize=10,color='#617384')
    for col,key,title,maximum in [(0,'error_count','Family errors  ↓',200),(1,'useful_retained_count','Useful corrections retained  ↑',330),(2,'harmful_count','Harmful overrides remaining  ↓',7.1)]:
        ax=fig.add_subplot(gs[0,col]);values=[row[key] for row in rows]
        ax.bar(x,values,color=COLORS,width=.62)
        for position,value in zip(x,values):ax.text(position,value+.025*maximum,str(value),ha='center',fontsize=12,weight='bold')
        ax.set_xticks(x,labels,rotation=16,ha='right');ax.set_ylim(0,maximum);ax.set_ylabel('5 ms frames');ax.set_title(title,loc='left',weight='bold');grid(ax)
        if key=='useful_retained_count':
            bound=baseline['A_useful_count']*protocol['training_gates']['useful_retention_fraction_min']
            ax.axhline(bound,color='#A34243',ls='--',lw=1);ax.text(3.43,bound+3,'90% required',fontsize=8,ha='right',color='#A34243')
        if key=='harmful_count':
            bound=baseline['A_harmful_count']*(1.-protocol['training_gates']['harmful_removal_fraction_min'])
            ax.axhline(bound,color='#A34243',ls='--',lw=1);ax.text(3.43,bound+.12,'≤3 required',fontsize=8,ha='right',color='#A34243')
    for col,key,title,limits in [(0,'point_RMS_mm','Point error RMS  ↓  (mm)',(2.744,2.785)),(1,'point_maximum_mm','Maximum point error  ↓  (mm)',(25.7,30.9))]:
        ax=fig.add_subplot(gs[1,col]);values=[row[key] for row in rows]
        ax.scatter(x,values,s=80,c=COLORS,zorder=3)
        for position,value in zip(x,values):ax.annotate(f'{value:.3f}',(position,value),xytext=(0,10),textcoords='offset points',ha='center',fontsize=10,weight='bold')
        ax.set_xticks(x,labels,rotation=16,ha='right');ax.set_xlim(-.5,3.5);ax.set_ylim(*limits);ax.set_title(title,loc='left',weight='bold');grid(ax)
        ax.text(.01,.02,'Zoomed vertical scale; dots show absolute values.',transform=ax.transAxes,fontsize=7.5,color='#617384')
        if key=='point_maximum_mm':
            ax.axhline(N[key],color='#A34243',ls='--',lw=1);ax.text(3.4,N[key]+.12,f'Native maximum {N[key]:.3f} mm',ha='right',fontsize=8,color='#A34243')
    ax=fig.add_subplot(gs[1,2]);ax.axis('off')
    text=('Decision: retain the live baseline\n\n'
          'Covariance: 163 errors; A: 124.\n'
          '253 / 294 useful corrections retained.\n'
          '4 / 6 harmful overrides remain.\n'
          'Worst point error is unchanged.\n\n'
          'The primary candidate fails the\n'
          'registered discrimination and\n'
          'shadow-policy gates.\n\n'
          'No physical tracking improvement\n'
          'is demonstrated by this replay.')
    ax.text(0,1,text,va='top',fontsize=11,linespacing=1.45,bbox=dict(boxstyle='round,pad=.65',facecolor='#F0F5F5',edgecolor='#D9E4E4'))
    fig.text(.065,.067,f'Primary comparison: {baseline["eligible_count"]:,} matched dominant-support frames (1–41 s); {baseline["A_proposal_count"]:,} accepted proposals. A = original I-60 accepted proposal or native fallback.',fontsize=9)
    fig.text(.065,.037,'Temporal frames are correlated. Calibration uses training-episode family truth; scores are empirical consistency distances, not contact probabilities.',fontsize=9,color='#617384')
    fig.savefig(out/'covariance_overview.png',dpi=180)
    with PdfPages(out/'covariance_overview.pdf') as pdf:pdf.savefig(fig);pages=pdf.get_pagecount()
    plt.close(fig);return pages


def explanation_page(science):
    fig=plt.figure(figsize=(14,9));fig.text(.06,.95,'What changed, and what the scores still cannot establish',fontsize=21,weight='bold')
    text=('Keep all three normalized joint-load residuals: e = residual / (0.03 + 0.10 R ‖filtered force‖).\n'
          'Max absolute keeps the largest component; Euclidean keeps total squared size. Shared covariance also accounts for bias and correlated error directions.\n'
          'One shared mean and covariance is fit on the other two episodes (equal episode weights; shrinkage ρ = 0.2; floor standard deviation = 0.05).\n'
          'The same fitted model scores F and Nalt. Training truth chooses calibration rows; held-out labels are used only for evaluation.')
    fig.text(.06,.87,text,fontsize=10.5,linespacing=1.65,va='top')
    chosen=[]
    for target in (7.875,35.205):chosen.append(next(row for row in science['events'] if row['case']=='m100_B' and abs(row['time_s']-target)<1e-6))
    for col,event in enumerate(chosen):
        ax=fig.add_axes([.075+.485*col,.33,.39,.28]);scores=event['scores']['M1'];positions=np.arange(3)
        ax.bar(positions-.18,scores['normalized_F_residual'],width=.34,color=F_COLOR,label='F proposal')
        ax.bar(positions+.18,scores['normalized_Nalt_residual'],width=.34,color=N_COLOR,label='Nalt physical alternative')
        ax.axhline(0,color='#617384',lw=.7);ax.set_xticks(positions,['shaft','shell A','shell B']);ax.set_ylabel('Normalized residual (dimensionless)');grid(ax)
        title='Correct F preferred' if col==0 else 'Incorrect F also preferred'
        ax.set_title(f'{event["time_s"]:.3f} s  |  {title}',loc='left',weight='bold');ax.legend(fontsize=8,loc='lower left' if col==0 else 'upper right')
        fig.text(.075+.485*col,.275,f'F: {family(event["F_family"])}  •  Nalt: {family(event["N_family"])}  •  dominant truth: {family(event["truth_family"])}',fontsize=9)
        table_rows=[]
        for branch in ['F','Nalt']:table_rows.append([branch]+[f'{scores[name][branch]:.3g}' for name in SCORERS])
        table_ax=fig.add_axes([.075+.485*col,.13,.39,.105]);table_ax.axis('off')
        table=table_ax.table(cellText=table_rows,colLabels=['Candidate','Max absolute','Euclidean','Covariance'],cellLoc='center',loc='center');table.auto_set_font_size(False);table.set_fontsize(9);table.scale(1,1.4)
        for (row,_),cell in table.get_celld().items():cell.set_edgecolor('#DCE3E8');cell.set_facecolor('#F0F5F5' if row==0 else 'white')
    fig.text(.06,.07,'Compare F against Nalt within each score column. Absolute values across scoring methods are not equivalent evidence scales.',fontsize=10)
    fig.text(.06,.035,'At 35.205 s every method retains the incorrect F proposal. A lower model-error score does not prove the loaded contact or actuator identity.',fontsize=10,weight='bold')
    return fig


def window_page(events,page_index,page_count,science,frames):
    fig=plt.figure(figsize=(14,9));fig.text(.065,.962,f'Recorded event windows  |  {page_index} / {page_count}',fontsize=20,weight='bold')
    fig.text(.065,.925,'Primary M1: raw preceding-interval loads with frozen endpoint geometry. F and Nalt remain the original physical alternatives.',fontsize=10)
    gs=fig.add_gridspec(2,3,left=.065,right=.98,bottom=.11,top=.825,wspace=.34,hspace=1.06)
    component_colors=('#2878A5','#C46F2A','#3A9874')
    for row,event in enumerate(events):
        case=event['case'];data=frames[case];anchor=event['time_s'];index=event['index']
        t=data['time_s'];selected=np.abs(t-anchor)<=.1+1e-9;x=1000.*(t[selected]-anchor)
        header_y=.88 if row==0 else .427
        reason=', '.join(reason.replace('_',' ') for reason in event['reasons'])
        fig.text(.065,header_y,f'{PHASE[case]}  |  {anchor:.3f} s  |  {reason}',fontsize=11.5,weight='bold')
        rank=RANK[int(data['M1__shared_covariance_code'][index])];keep=bool(data['M1__shared_covariance_keep'][index])
        fig.text(.065,header_y-.025,f'F={family(event["F_family"])}; Nalt={family(event["N_family"])}; dominant truth={family(event["truth_family"])}.  Covariance: {rank}; {"retain F" if keep else "native fallback"}.  Evaluation point error: N {event["native_point_mm"]:.2f} / F {event["proposal_point_mm"]:.2f} mm.',fontsize=8.8)
        axes=[fig.add_subplot(gs[row,col]) for col in range(3)]
        for component,color in enumerate(component_colors):
            axes[0].plot(x,data['M1__e_F'][selected,component],color=color,lw=1.3,label=f'{("shaft","shell A","shell B")[component]} F')
            axes[0].plot(x,data['M1__e_Nalt'][selected,component],color=color,lw=1.3,ls='--',label=f'{("shaft","shell A","shell B")[component]} Nalt')
        axes[0].set_ylabel('Normalized residual\n(dimensionless)');axes[0].legend(fontsize=6.2,ncol=2,loc='best',framealpha=.8)
        ranks=[]
        for name,color,display in zip(SCORERS,COLORS[1:],METHOD_NAMES):
            F=data[f'M1__{name}_F_score'][selected];N=data[f'M1__{name}_Nalt_score'][selected]
            denominator=np.maximum(1.,np.maximum(abs(F),abs(N)))
            margin=(N-F)/denominator
            margin=np.where(data[f'M1__{name}_known'][selected],margin,np.nan)
            axes[1].plot(x,margin,color=color,lw=1.5,label=display)
            ranks.append(RANK[int(data[f'M1__{name}_code'][index])])
        axes[1].axhline(0,color='#617384',ls='--',lw=.8);axes[1].set_ylim(-1.06,1.06)
        axes[1].set_ylabel('Scaled score preference\npositive favors F (display only)');axes[1].legend(fontsize=7,loc='best',framealpha=.8)
        axes[1].text(.01,.02,'Recorded ranks: '+' / '.join(ranks),transform=axes[1].transAxes,fontsize=6.8,bbox=dict(facecolor='white',edgecolor='none',alpha=.8))
        for branch,color in [('F',F_COLOR),('Nalt',N_COLOR)]:
            scores=data[f'M1__shared_covariance_{branch}_score'][selected]
            axes[2].semilogy(x,np.where(scores>0,scores,np.nan),color=color,lw=1.6,label=branch)
        fitted=science['models'][case]['M1']['fitted']
        if fitted is not None:axes[2].axhline(fitted['training_weighted_p95_score'],color='#7D8B96',ls=':',lw=1,label='Training 95th (reference)')
        axes[2].set_ylabel('Squared covariance score\nlog scale; not probability');axes[2].legend(fontsize=6.8,loc='best',framealpha=.8)
        for ax in axes:
            ax.axvline(0,color='#253E54',lw=.9,alpha=.8);ax.axvspan(0,100,color='#9DACB8',alpha=.065)
            ax.set_xlim(-100,100);ax.set_xticks([-100,-50,0,50,100]);ax.set_xlabel('Time from recorded event (ms)');grid(ax)
    fig.text(.065,.055,'Scaled preference = (scoreNalt − scoreF) / max(1, scoreF, scoreNalt); descriptive display only. Decisions above are cached original classifier outputs.',fontsize=8.5)
    fig.text(.065,.028,'Shaded later samples are plotting context only; they did not enter earlier inference. Same previously inspected episodes; no live actuation or new driving.',fontsize=8.5,color='#617384')
    return fig


def run(root,previous,output):
    output.mkdir(parents=True,exist_ok=False);style()
    science_path=root/'analysis/stage01/science.json';protocol_path=root/'protocol/protocol.json'
    science=read(science_path);protocol=read(protocol_path)
    frames={case:load(root/'analysis/stage01'/f'{case}_oof.npz') for case in CASES}
    bases={case:load(previous/'portable_inputs/i62_base'/f'{case}.npz') for case in CASES}
    events=science['events'];keys=[(event['case'],event['index']) for event in events]
    if len(set(keys))!=len(keys):raise ValueError('Duplicate report event')
    for case in CASES:
        if not np.array_equal(frames[case]['time_s'],bases[case]['time_s']):raise ValueError('Mismatched cached timebases')
    for event in events:
        case=event['case'];i=event['index'];base=bases[case]
        if abs(frames[case]['time_s'][i]-event['time_s'])>1e-9 or int(base['N_family'][i])!=event['N_family'] or int(base['truth_family'][i])!=event['truth_family']:raise ValueError('Event anchor differs from frozen report/base')
    inputs=[science_path,protocol_path]+[root/'analysis/stage01'/f'{case}_oof.npz' for case in CASES]+[previous/'portable_inputs/i62_base'/f'{case}.npz' for case in CASES]
    bindings=[dict(path=str(path),sha256=sha(path)) for path in inputs]
    write(output/'registered_before_plotting.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),inputs=bindings,event_count=len(events),cached_results_only=True,
        descriptive_transform='Scaled preference uses existing scores only; not a new score threshold or classifier.',new_inference=False,new_fits=False))
    shutil.copy2(Path(__file__),output/'plot_results.py')
    overview_pages=overview(science,protocol,output)
    with PdfPages(output/'covariance_event_windows.pdf') as pdf:
        fig=explanation_page(science);pdf.savefig(fig);plt.close(fig)
        window_pages=math.ceil(len(events)/2)
        for index in range(window_pages):
            fig=window_page(events[2*index:2*index+2],index+1,window_pages,science,frames);pdf.savefig(fig);plt.close(fig)
        event_pages=pdf.get_pagecount()
    for item in bindings:
        if sha(item['path'])!=item['sha256']:raise ValueError('Plot input changed')
    write(output/'plot_manifest.json',dict(iteration='I-65',source_sha256=sha(Path(__file__)),inputs=bindings,
        event_count=len(events),event_anchors=[dict(case=event['case'],index=event['index'],time_s=event['time_s'],reasons=event['reasons']) for event in events],
        overview_pdf_pages=overview_pages,event_pdf_pages=event_pages,events_per_window_page=2,window_halfwidth_s=.1,
        all_report_events_included=True,new_inference=False,new_fits=False,
        files=[dict(path=file.name,sha256=sha(file),bytes=file.stat().st_size) for file in sorted(output.iterdir()) if file.is_file() and file.name!='plot_manifest.json']))
    print(json.dumps(dict(event_count=len(events),overview_pdf_pages=overview_pages,event_pdf_pages=event_pages,output=str(output))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--previous',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();root=a.root.resolve();previous=a.previous.resolve() if a.previous else root.parents[1]/read(root/'protocol/protocol.json')['previous_package']
    run(root,previous,a.output.resolve() if a.output else root/'publication/plots_stage01')
