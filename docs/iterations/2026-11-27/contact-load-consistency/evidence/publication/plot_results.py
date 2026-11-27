"""I-61 all-case load-veto figures from sealed evaluation arrays."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import ListedColormap
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
PHASES = ('−1.0 rad', '+0.3 rad', '+0.9 rad')
POLICIES = ('N', 'A', 'C')
LABELS = ('Native N', 'I-60 accepted/fallback A', 'Joint-veto/fallback C')
COLORS = ('#555555', '#228833', '#4477aa')
EVENTS = (35.205, 38.380, 17.765)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def fraction(number, denominator):
    return float(number/denominator) if denominator else None


def summary(data):
    selected = data['primary_mask'] & data['truth_eligible']
    truth, native = data['truth_family'], data['N_family']
    harmful_A = int(np.sum(selected & (native == truth) & (data['A_family'] != truth)))
    harmful_C = int(np.sum(selected & (native == truth) & (data['C_family'] != truth)))
    useful = selected & (native != truth) & (data['A_family'] == truth)
    retained = int(np.sum(useful & (data['C_family'] == truth)))
    oracle = selected & data['stable_truth_mask']
    points = {}
    finite = selected & np.isfinite(data['truth_equivalent_point_m']).all(axis=1)
    for policy in POLICIES:
        finite &= np.isfinite(data[policy+'_point_m']).all(axis=1)
    for policy in POLICIES:
        distance = 1000*np.linalg.norm(data[policy+'_point_m'][finite]-data['truth_equivalent_point_m'][finite], axis=1)
        points[policy] = dict(RMS_mm=float(np.sqrt(np.mean(distance**2))), maximum_mm=float(distance.max()))
    return dict(eligible_count=int(selected.sum()),
        errors={p:int(np.sum(selected & (data[p+'_family'] != truth))) for p in POLICIES},
        harmful_A=harmful_A, harmful_C=harmful_C, useful_A=int(useful.sum()), useful_retained=retained,
        harmful_removed=fraction(harmful_A-harmful_C, harmful_A),
        useful_retention=fraction(retained, int(useful.sum())),
        oracle_count=int(oracle.sum()), oracle_pass_count=int(np.sum(oracle & data['oracle_geometry_pass'])),
        oracle_fraction=fraction(int(np.sum(oracle & data['oracle_geometry_pass'])), int(oracle.sum())), points=points)


def raster(ax, data, lo, hi):
    time = data['time_s']
    mask = (time >= lo-1e-9) & (time < hi-1e-9)
    truth = np.where(data['truth_eligible'], data['truth_family'], -1)
    rows = [truth]+[data[p+'_family'] for p in POLICIES]
    rows += [np.where(data['A_uses_proposal'], data['A_family'], -1),
             np.where(data['C_uses_proposal'], data['C_family'], -1)]
    ax.imshow(np.asarray(rows)[:, mask]+1, aspect='auto', interpolation='nearest',
        extent=(lo, hi, 5.5, -.5), vmin=-.5, vmax=4.5,
        cmap=ListedColormap(('#dddddd', '#4477aa', '#ee6677', '#228833', '#ccbb44')))
    ax.set(yticks=np.arange(6), yticklabels=('Truth ≥90%', 'Native N', 'I-60 hybrid A',
           'Joint-veto hybrid C', 'A accepted proposal', 'C retained proposal'), xlim=(lo, hi))


def trace(axes, data, lo, hi):
    raster(axes[0], data, lo, hi)
    time = data['time_s']
    axes[0].set_title('Blue: shell A · coral: shell B · green: belt A · yellow: belt B · gray: unknown/no proposal')
    axes[1].semilogy(time, np.maximum(data['proposed_max_normalized_joint_residual'], 1e-6),
                     color='#aa4499', label='Frozen F proposal', linewidth=.8)
    other = np.max(abs(data['native_alternative_joint_residual_Nm'])/data['threshold_Nm'][:, None], axis=1)
    axes[1].semilogy(time, np.maximum(other, 1e-6), color='#777777', label='Valid native-family alternative (diagnostic)', linewidth=.6)
    axes[1].axhline(1., color='black', linestyle='--', label='Fixed joint compatibility limit')
    axes[1].set(xlim=(lo, hi), ylabel='Maximum |joint residual| / tolerance')
    axes[1].legend(fontsize=8, loc='lower left')
    mask = data['truth_eligible']
    for policy, color, label in zip(POLICIES, COLORS, LABELS):
        distance = 1000*np.linalg.norm(data[policy+'_point_m']-data['truth_equivalent_point_m'], axis=1)
        axes[2].plot(time, np.where(mask, distance, np.nan), color=color, label=label, linewidth=.7)
    axes[2].set(xlim=(lo, hi), ylabel='Point discrepancy (mm)', xlabel='Recorded time (s)')
    axes[2].legend(fontsize=8)


def draw(source, output):
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__), output/'plot_source.py')
    files = [source/(case+'_evaluation.npz') for case in CASES]
    save(output/'registered_before_render.json', dict(captured_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)), inputs=[dict(path=str(p), sha256=sha(p)) for p in files],
        scope='All starts plus pooled, complete recorded time and fixed I60 worst-point times; no outcome-selected figures.'))
    datasets = []
    for path in files:
        with np.load(path, allow_pickle=False) as data:
            datasets.append({key:data[key].copy() for key in data.files})
    counts = [summary(d) for d in datasets]
    combined = {key:np.concatenate([d[key] for d in datasets], axis=0) for key in datasets[0]}
    pooled = summary(combined)
    save(output/'independent_plot_counts.json', dict(cases=dict(zip(CASES, counts)), pooled=pooled))
    summaries = counts+[pooled]
    x = np.arange(4)
    with PdfPages(output/'load_veto_comparison.pdf') as pdf:
        fig, axes = plt.subplots(1, 3, figsize=(14, 5), layout='constrained')
        for j, (p, label, color) in enumerate(zip(POLICIES, LABELS, COLORS)):
            values = [100*s['errors'][p]/s['eligible_count'] for s in summaries]
            axes[0].bar(x+(j-1)*.24, values, width=.23, label=label, color=color)
        axes[0].set(ylabel='Wrong/unknown dominant family (%)', title='Same primary states, 1–41 s')
        axes[0].legend(fontsize=7)
        for delta, key, label, color in ((-.18, 'harmful_removed', 'Harmful overrides removed', '#aa4499'),
                                         (.18, 'useful_retention', 'Useful corrections retained', '#4477aa')):
            values = [100*s[key] if s[key] is not None else np.nan for s in summaries]
            axes[1].bar(x+delta, values, width=.34, label=label, color=color)
        axes[1].axhline(50., color='#aa4499', linestyle='--', linewidth=1.)
        axes[1].axhline(90., color='#4477aa', linestyle=':', linewidth=1.)
        axes[1].set(ylim=(0,105), ylabel='Percent', title='Fixed benefit/retention checks')
        axes[1].legend(fontsize=7, loc='lower left')
        values = [100*s['oracle_fraction'] if s['oracle_fraction'] is not None else np.nan for s in summaries]
        axes[2].bar(x, values, color='#ddaa33')
        axes[2].axhline(90., color='black', linestyle='--', label='Fixed 90% per-start validity gate')
        axes[2].set(ylim=(0,105), ylabel='Pass / stable eligible (%)', title='Oracle-geometry compatibility')
        axes[2].legend(fontsize=7, loc='lower left')
        for ax in axes:
            ax.set(xticks=x, xticklabels=PHASES+('Pooled',))
        fig.suptitle('I-61 | Do joint-load residuals reject unsupported contact corrections?\n'
                     'Frozen I60 proposals; native fallback; zero new physics or changed commands')
        fig.supxlabel('Oracle diagnostic uses observed total force + logged dominant-family mean point; it is not independent force truth.', fontsize=9)
        fig.savefig(output/'overview.png', dpi=160)
        pdf.savefig(fig)
        plt.close(fig)
        fig, axes = plt.subplots(1,2,figsize=(11,4.8),layout='constrained')
        for ax, field, title in zip(axes, ('RMS_mm','maximum_mm'), ('Point RMS','Maximum point discrepancy')):
            for j,(p,label,color) in enumerate(zip(POLICIES,LABELS,COLORS)):
                values=[s['points'][p][field] for s in summaries]
                bars=ax.bar(x+(j-1)*.24,values,width=.23,label=label,color=color)
                ax.bar_label(bars,fmt='%.2f',fontsize=7)
            ax.set(xticks=x,xticklabels=PHASES+('Pooled',),ylabel='mm',title=title)
            ax.margins(y=.2)
            ax.legend(fontsize=7)
        fig.suptitle('I-61 | Point policies on identical primary rows\nDistance to logged equivalent-point reference; no interpolated or clipped geometry')
        fig.savefig(output/'point_comparison.png',dpi=160)
        pdf.savefig(fig)
        plt.close(fig)
        for case, phase, event, data in zip(CASES, PHASES, EVENTS, datasets):
            for lo,hi,suffix in ((0.,44.,'full'),(event-.15,event+.15,'fixed_worst')):
                fig,axes=plt.subplots(3,1,figsize=(12,9),layout='constrained',gridspec_kw={'height_ratios':(2,1.2,1.2)})
                trace(axes,data,lo,hi)
                fig.suptitle(f'I-61 | Initial shaft {phase} | {suffix.replace("_"," ")}\n'
                             'Sampled last-1-ms labels, filtered preceding-5-ms observer; passing a gate does not prove contact.')
                if suffix=='fixed_worst':
                    for ax in axes:ax.axvline(event,color='#bb3333',linestyle=':',linewidth=.8)
                fig.savefig(output/(case+'_'+suffix+'.png'),dpi=140)
                pdf.savefig(fig)
                plt.close(fig)
    save(output/'artifact_manifest.json', dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(cases=3,pdf_pages=8,output=str(output))))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();draw(args.data_source.resolve(),args.output.resolve())
