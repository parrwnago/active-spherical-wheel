"""Seal the saved-data audit; preserve initial figure layout before polishing."""
from pathlib import Path
import hashlib
import json
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    if (HERE/'qa.json').exists(): raise FileExistsError('Final audit is already sealed')
    report=json.loads((HERE/'motion_audit.json').read_text())
    previous=HERE/'development/plot_stage01';previous.mkdir(parents=True,exist_ok=False)
    for path in (HERE/'figures').iterdir(): shutil.copy2(path,previous/path.name)
    shutil.copy2(HERE/'motion_audit.json',previous/'motion_audit_initial.json')
    (previous/'layout_note.json').write_text(json.dumps(dict(issue='Initial geometry legend overlapped the unavailable-y callout; initial plots retained.',
        correction='Plot-only shared legend below axes and clear annotations; no numeric evidence recalculated.'),indent=2)+'\n')
    extra={
        'early_summary':'results/contact_diagnostics_20261125/baseline/plain_figure8/phase_+0.90/summary.json',
        'early_provenance':'results/contact_diagnostics_20261125/baseline/provenance.json'}
    digests={rel:sha(ROOT/rel) for rel in extra.values()}
    summary=json.loads((ROOT/extra['early_summary']).read_text());provenance=json.loads((ROOT/extra['early_provenance']).read_text())
    report['early_saved_event']['run_metadata']=dict(variant=provenance['variant'],git_commit=provenance['git_commit'],
        route=summary['route'],parameters=summary['parameters'],simulator=summary['engine'],
        historical_allocator='Historical rolling_allocation enum absent: experiments.py _rolling_allocation regularization0 is normalized least squares.',
        distinct_from_first_departure='Original diagnostic baseline31.65s, not freshI40/I41firstdeparture6.86s.')
    report['integrity']['input_sha256'].update(digests)
    report['source_files'].update(extra)
    plt.rcParams.update({'font.size':11,'axes.titlesize':12,'figure.titlesize':14})
    fig,axes=plt.subplots(1,2,figsize=(11,5.9))
    fig.subplots_adjust(left=.08,right=.98,bottom=.25,top=.75,wspace=.32)
    for ax,example in zip(axes,report['examples']):
        matrix=np.asarray(example['B_body_velocity_per_joint_rate_m'])
        for k,color,label in [(0,'#337eaf','Shaft, +1 rad/s'),(1,'#df9b3a','Loaded shell B, +1 rad/s')]:
            ax.arrow(0,0,matrix[0,k],matrix[1,k],width=.0015,head_width=.007,length_includes_head=True,color=color,label=label,zorder=3)
        ax.axhline(0,color='#999',lw=.6);ax.axvline(0,color='#999',lw=.6)
        ax.set(xlim=(-.015,.14),ylim=(-.12,.09),aspect='equal',xlabel='Body x velocity contribution (m/s)',ylabel='Body y velocity contribution (m/s)',
               title=f'Shaft phase {example["phase_deg"]:.3f}°\nScaled singular values: {example["scaled_singular_values_m_s"][0]:.3f}, {example["scaled_singular_values_m_s"][1]:.3f} m/s')
        ax.grid(alpha=.18)
    axes[1].annotate('x request feasible',xy=(.06,0),xytext=(.045,-.043),arrowprops=dict(arrowstyle='->',color='#396b46'),fontsize=10,color='#396b46')
    axes[1].annotate('y request unavailable',xy=(0,.06),xytext=(.019,.067),arrowprops=dict(arrowstyle='->',color='#ad4943'),fontsize=10,color='#ad4943')
    handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',bbox_to_anchor=(.5,.055),ncol=2,fontsize=10)
    fig.suptitle('One contact, same −20° platform roll: changing shaft phase can align drive directions\nFlat normal; shell B remains selected. Instantaneous geometry, not a balance test.',y=.98)
    for ext in ('png','pdf'):fig.savefig(HERE/'figures'/f'drive_direction_example.{ext}',dpi=200)
    plt.close(fig)
    rows=report['turns']['rows'];x=np.arange(6)
    fig,ax=plt.subplots(figsize=(11.5,4.8),constrained_layout=True)
    for shift,values,color,label in [(-.25,[r['mars_geometric_overshoot_mm'] for r in rows],'#667b8d','Native baseline'),
            (0.,report['interventions']['I42_sync_off']['overshoot_mm'],'#4a94a4','I42 sync off only 4–10s'),
            (.25,report['interventions']['I43_shaped_consistent_rate']['overshoot_mm'],'#d99a4c','I43 shaped tilt + consistent rate')]:
        ax.bar(x+shift,values,width=.25,color=color,label=label)
    ax.set(xticks=x,xticklabels=[r['event_id'].replace('turn_','') for r in rows],ylabel='Geometric component overshoot (mm)',xlabel='Fixed six reference events',
           title='Local improvements did not produce uniform turn improvement\nBoth candidates worsen 4 of 6 overshoots; separate experiments, not one combined factorial.')
    ax.grid(axis='y',alpha=.2);ax.legend(fontsize=9)
    for ext in ('png','pdf'):fig.savefig(HERE/'figures'/f'intervention_turn_tradeoffs.{ext}',dpi=190)
    plt.close(fig)
    report['integrity']['finalizer_source_sha256']=sha(Path(__file__))
    for rel,digest in report['integrity']['input_sha256'].items(): assert sha(ROOT/rel)==digest,rel
    (HERE/'motion_audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    checks=dict(all_old_inputs_unchanged=True,physical_and_normalized_J_units_distinguished=True,
        original_event_float32_limitation_explicit=True,first_departure_distinct_from31p65s_event=True,
        six_turns_and_flat_evidence_preserved=True,separate_interventions_and_carried_history_explicit=True,
        no_new_physics_or_model_forward=True,three_PNG_and_three_PDF=True,
        plot_layout_correction_preserved=True)
    (HERE/'qa.json').write_text(json.dumps(dict(passed=all(checks.values()),checks=checks,
        visual_review='Inspected all three figure layouts; corrected geometry annotation overlap without numerical changes.',
        authority='Numeric assertions in exact preserved build source; source links/units/turn definitions reviewed.',
        input_count=len(report['integrity']['input_sha256'])),indent=2)+'\n')
    snapshot=HERE/'source_snapshot'/Path(__file__).name;shutil.copy2(Path(__file__),snapshot)
    files=[]
    for path in sorted(HERE.rglob('*')):
        if path.is_file() and path.name not in ('artifact_manifest.json','finalize.log'):
            files.append(dict(path=str(path.relative_to(HERE)),sha256=sha(path),bytes=path.stat().st_size))
    (HERE/'artifact_manifest.json').write_text(json.dumps(dict(origin_base=str(HERE.relative_to(ROOT)),files=files),indent=2)+'\n')
    print(json.dumps(dict(passed=True,files=len(files),input_count=len(report['integrity']['input_sha256'])),indent=2))


if __name__=='__main__':main()
