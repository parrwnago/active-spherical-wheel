"""Fixed all-case I-60 scientific figures from sealed evaluation arrays."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import ListedColormap
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
PHASES = ('−1.0 rad', '+0.3 rad', '+0.9 rad')
METHODS = ('native', 'geometry', 'wrench', 'velocity', 'fused')
NAMES = ('Native locator', 'Geometry', 'Geometry + wrench', 'Geometry + velocity', 'All three')
WINDOWS = ((6.4, 7.4), (31.4, 32.6), (35.4, 36.4))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def raw_label(data, method):
    return data['native_family'] if method == 'native' else data[method+'__chosen_family']


def primary(data):
    return data['truth_eligible'] & (data['time_s'] >= 1.-1e-9) & (data['time_s'] < 41.-1e-9)


def summary(data):
    eligible = primary(data)
    n = int(eligible.sum())
    errors = {method: int(np.sum(eligible & (raw_label(data, method) != data['truth_family'])))
              for method in METHODS}
    ambiguous = eligible & (data['valid_family_count'] >= 2) & (data['measured_excitation_m_s'] >= .02)
    accepted = eligible & (data['fused__accepted_family'] >= 0)
    return dict(eligible_count=n, raw_errors=errors,
                ambiguous_motion_count=int(ambiguous.sum()), accepted_count=int(accepted.sum()),
                accepted_errors=int(np.sum(accepted & (data['fused__accepted_family'] != data['truth_family']))),
                native_errors_on_accepted=int(np.sum(accepted & (data['native_family'] != data['truth_family']))))


def raster(ax, data, lo, hi):
    t = data['time_s']
    keep = (t >= lo-1e-9) & (t < hi-1e-9)
    truth = np.where(data['truth_eligible'], data['truth_family'], -1)
    rows = [truth, data['native_family']]
    rows.extend(data[mode+'__chosen_family'] for mode in METHODS[1:])
    rows.append(data['fused__accepted_family'])
    ax.imshow(np.asarray(rows)[:, keep]+1, interpolation='nearest', aspect='auto',
              extent=(lo, hi, 6.5, -.5), vmin=-.5, vmax=4.5,
              cmap=ListedColormap(('#dddddd', '#4477aa', '#ee6677', '#228833', '#ccbb44')))
    ax.set(yticks=np.arange(7), yticklabels=('Truth ≥90% load', 'Native', 'Geometry',
           'Geometry + wrench', 'Geometry + velocity', 'All three', 'All three accepted'),
           xlim=(lo, hi), xlabel='Recorded time (s)')


def draw(source, output):
    output.mkdir(parents=True, exist_ok=False)
    script = Path(__file__).resolve()
    shutil.copy2(script, output/'plot_source.py')
    files = [source/(case+'_evaluation.npz') for case in CASES]
    save(output/'registered_before_render.json', dict(iteration='I-60',
         captured_utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(script),
         files=[dict(path=str(path), sha256=sha(path)) for path in files],
         scope='All three starts and all three fixed event windows; no event selection or score tuning.'))
    datasets = []
    for path in files:
        with np.load(path, allow_pickle=False) as data:
            datasets.append({key: data[key].copy() for key in data.files})
    summaries = [summary(data) for data in datasets]
    pooled = dict(eligible_count=sum(x['eligible_count'] for x in summaries),
        raw_errors={m: sum(x['raw_errors'][m] for x in summaries) for m in METHODS},
        ambiguous_motion_count=sum(x['ambiguous_motion_count'] for x in summaries),
        accepted_count=sum(x['accepted_count'] for x in summaries),
        accepted_errors=sum(x['accepted_errors'] for x in summaries),
        native_errors_on_accepted=sum(x['native_errors_on_accepted'] for x in summaries))
    save(output/'independent_plot_counts.json', dict(cases=dict(zip(CASES, summaries)), pooled=pooled,
         meaning='Correlated saved-state counts; not independent trials or continuous wrong-contact durations.'))
    colors = ('#333333', '#aa4499', '#ddaa33', '#4477aa', '#228833')
    with PdfPages(output/'contact_ranking.pdf') as pdf:
        fig, axes = plt.subplots(1, 2, figsize=(13, 5), layout='constrained')
        all_summaries = summaries+[pooled]
        x = np.arange(4)
        for j, (method, name, color) in enumerate(zip(METHODS, NAMES, colors)):
            values = [100*s['raw_errors'][method]/s['eligible_count'] if s['eligible_count'] else np.nan
                      for s in all_summaries]
            axes[0].bar(x+(j-2)*.15, values, width=.145, label=name, color=color)
        axes[0].set(xticks=x, xticklabels=PHASES+('Pooled',), ylabel='Wrong/unknown raw family (%)',
                    title='All dominant-support eligible moving states (1–41 s)')
        axes[0].legend(fontsize=8)
        coverage = [100*s['accepted_count']/s['ambiguous_motion_count'] if s['ambiguous_motion_count'] else np.nan
                    for s in all_summaries]
        axes[1].bar(x, coverage, color=colors[-1])
        axes[1].axhline(50., color='black', linestyle='--', label='Fixed 50% coverage gate')
        axes[1].set(xticks=x, xticklabels=PHASES+('Pooled',), ylabel='Accepted / ambiguous motion-eligible (%)',
                    ylim=(0, 105), title='All-three confidence coverage; singleton choices excluded')
        for i, s in enumerate(all_summaries):
            annotation = (f"{s['accepted_errors']}/{s['accepted_count']} wrong" if s['accepted_count'] else 'No accepted states')
            axes[1].text(i, 103, annotation, ha='center', va='top', fontsize=8)
        axes[1].legend(fontsize=8)
        fig.suptitle('I-60 | Can measured velocity improve contact-family inference?\n'
                     'Saved native trajectories; observer-conditioned candidate domain; zero new physics')
        fig.savefig(output/'overview.png', dpi=170)
        pdf.savefig(fig)
        plt.close(fig)
        for case, phase, data, counts in zip(CASES, PHASES, datasets, summaries):
            t = data['time_s']
            fig, axes = plt.subplots(3, 1, figsize=(12, 9), layout='constrained',
                                      gridspec_kw={'height_ratios': (2.2, 1.2, 1.2)})
            raster(axes[0], data, 0., 44.)
            axes[0].set_title('Blue: shell A · coral: shell B · green: belt A · yellow: belt B · gray: unknown/ineligible')
            for value, label, color in ((data['fused__best_velocity_residual_m_s'], 'Best fused tangential residual', '#228833'),
                                        (data['measured_excitation_m_s'], 'Measured motion-availability metric', '#777777')):
                axes[1].semilogy(t, np.maximum(value, 1e-7), color=color, label=label, linewidth=.7)
            axes[1].axhline(.02, color='black', linestyle='--', label='0.02 m/s residual / motion thresholds')
            axes[1].set(xlim=(0, 44), ylabel='m/s (log scale)')
            axes[1].legend(fontsize=8)
            eligible = primary(data)
            for method, color, label in zip(METHODS, colors, NAMES):
                wrong = eligible & (raw_label(data, method) != data['truth_family'])
                axes[2].plot(t, .005*np.cumsum(wrong), color=color, label=label, linewidth=1.)
            axes[2].set(xlim=(0, 44), ylabel='Sampled disagreement equivalent (s)', xlabel='Recorded time (s)')
            axes[2].legend(fontsize=8, ncol=3)
            fig.suptitle(f'I-60 | Initial shaft {phase} | {counts["eligible_count"]} primary eligible states\n'
                         'Truth: last completed 1 ms; observer: filtered preceding 5 ms. No fixed latency inference.')
            fig.savefig(output/(case+'_timeline.png'), dpi=150)
            pdf.savefig(fig)
            plt.close(fig)
            fig, axes = plt.subplots(3, 1, figsize=(12, 9), layout='constrained')
            for ax, (lo, hi) in zip(axes, WINDOWS):
                raster(ax, data, lo, hi)
                ax.set_title(f'Prespecified event window {lo:g}–{hi:g} s')
            fig.suptitle(f'I-60 | Initial shaft {phase} | all fixed handoff windows\n'
                         'Labels shown on sampled rows; no commanded motion or trajectory changed.')
            pdf.savefig(fig)
            plt.close(fig)
    save(output/'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(output)),
         sha256=sha(p), bytes=p.stat().st_size) for p in sorted(output.rglob('*'))
         if p.is_file() and p.name != 'artifact_manifest.json']))
    print(json.dumps(dict(cases=3, pdf_pages=7, output=str(output))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    draw(args.data_source.resolve(), args.output.resolve())
