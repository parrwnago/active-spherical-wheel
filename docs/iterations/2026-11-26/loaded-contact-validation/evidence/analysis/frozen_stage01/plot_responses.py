"""Static scientific plots from I57 report; no model or physics calls."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize


def plot(output):
    output=Path(output);report=json.loads((output/'report.json').read_text());groups=list(report['groups']);events=list(report['events']);
    fig,axes=plt.subplots(max(1,len(events)),2,figsize=(11,4.8*max(1,len(events))),squeeze=False,constrained_layout=True)
    image=None
    for row,event_id in enumerate(events):
        event=report['events'][event_id]
        for column,horizon in enumerate((.005,.02)):
            ax=axes[row,column];pairs=[p for p in event['comparisons'] if abs(p['horizon_s']-horizon)<1e-10];labels=[p['pair_id'] for p in pairs]
            values=np.array([[p['groups'][g]['relative_vector_error'] if p['groups'][g]['status']!='unresolved' else np.nan for g in groups] for p in pairs])
            if not len(pairs):ax.text(.5,.5,'No paired endpoints available',ha='center',va='center');ax.set_axis_off();continue
            cmap=plt.colormaps['magma'].copy();cmap.set_bad('#cfcfcf');image=ax.imshow(values,aspect='auto',cmap=cmap,norm=Normalize(0,2))
            for i,pair in enumerate(pairs):
                for j,g in enumerate(groups):
                    s=pair['groups'][g];label='U' if s['status']=='unresolved' else ('P' if s['passes_screen'] else 'F')
                    if s['transverse_nearzero_absolute_pass'] is not None:label+='n' if s['transverse_nearzero_absolute_pass'] else '!'
                    ax.text(j,i,label,ha='center',va='center',fontsize=8,color='black' if np.isnan(values[i,j]) else 'white')
            ax.set_xticks(range(len(groups)),groups,rotation=35,ha='right');ax.set_yticks(range(len(labels)),labels,fontsize=8)
            valid=event['controls']['valid_for_interpretation'];ax.set_title(f'{event_id}, {horizon*1000:.0f} ms'+('' if valid else ' — CONTROLS INCOMPLETE/FAILED'))
            if any(p['endpoint_support_changed'] for p in pairs):ax.set_ylabel('Includes endpoint support changes')
    if image is not None:fig.colorbar(image,ax=axes.ravel().tolist(),label='||observed − predicted|| / max(||observed||, floor)\nClipped at2; grey U=unresolved. P/F=screen; n=nearzero compatible, !=nearzero failed.')
    fig.suptitle('I57 loaded-contact response: paired fixed-command differences\nFinite perturbations; endpoint contacts do not establish continuity within5 ms',fontsize=12)
    fig.savefig(output/'response_accuracy.png',dpi=180);fig.savefig(output/'response_accuracy.pdf');plt.close(fig)
    fig,axes=plt.subplots(max(1,len(events)),2,figsize=(11,3.8*max(1,len(events))),squeeze=False,constrained_layout=True)
    for row,event_id in enumerate(events):
        event=report['events'][event_id]
        for col,horizon in enumerate((.005,.02)):
            ax=axes[row,col];pair=next((p for p in event['comparisons'] if p['pair_id']=='native_to_governed' and abs(p['horizon_s']-horizon)<1e-10),None)
            if pair is None:ax.text(.5,.5,'Native/governed pair unavailable',ha='center',va='center');ax.set_axis_off();continue
            # Per-component values divided by fixed group floors keep units explicit without mixing raw units.
            ix=[0,1,3,4,9,6,7,8];floors=np.array([.0002,.0002,.002,.002,.002,.02,.02,.02]);labels=['vx/.0002','vy/.0002','wx0/.002','wy0/.002','wzW/.002','shaft/.02','shellA/.02','shellB/.02'];x=np.arange(len(ix));width=.25
            for shift,key,label,color in ((-width,'predicted','Loaded model','#2166ac'),(0,'observed','Observed paired','#b2182b'),(width,'free_predicted','Free model','#888888')):
                ax.bar(x+shift,np.asarray(pair[key])[ix]/floors,width,label=label,color=color)
            ax.axhline(0,color='black',lw=.6);ax.axhspan(-1,1,color='#cccccc',alpha=.25);ax.set_yscale('symlog',linthresh=2);ax.set_xticks(x,labels,rotation=40,ha='right',fontsize=8)
            ax.set_ylabel('Signed response / fixed group floor');ax.set_title(f'{event_id}, {horizon*1000:.0f}ms'+('' if pair['valid_for_interpretation'] else ' — NOT INTERPRETABLE'));ax.legend(fontsize=7)
    fig.suptitle('Native-clipped → governed finite response\nwx0/wy0: initial fork axes; wzW: world vertical. Linear units m/s, angular units rad/s.',fontsize=12)
    fig.savefig(output/'native_governed_response.png',dpi=180);fig.savefig(output/'native_governed_response.pdf');plt.close(fig)
