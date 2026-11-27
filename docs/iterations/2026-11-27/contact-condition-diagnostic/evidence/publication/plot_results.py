"""Plot frozen I-63 diagnostics and registered event windows; no fitting."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

CASES = ('m100_B', 'p030_B', 'p090_B')


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() for k in z.files}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.package
    analysis = root / 'analysis/stage01'
    science = json.loads((analysis / 'science.json').read_text())
    witness = json.loads((root / 'portable_inputs/i62_old_witnesses.json').read_text())
    data, old_masks = {}, {}
    for case in CASES:
        data[case] = load(analysis / (case + '_diagnostic_frames.npz'))
        times = {round(row['time_s'] * 1e9) for row in witness['witnesses'] if row['case'] == case}
        old_masks[case] = np.array([round(t * 1e9) in times for t in data[case]['time_s']])
    fields = ('joint_load_risk_u', 'seam_risk_s', 'combined_risk_r')
    values = {key: np.concatenate([data[c][key] for c in CASES]) for key in fields}
    useful = np.concatenate([(data[c]['N_family'] != data[c]['truth_family']) &
                              (data[c]['A_family'] == data[c]['truth_family']) for c in CASES])
    harmful = np.concatenate([(data[c]['N_family'] == data[c]['truth_family']) &
                              (data[c]['A_family'] != data[c]['truth_family']) for c in CASES])
    old = np.concatenate([old_masks[c] for c in CASES])
    assert useful.sum() == 294 and harmful.sum() == 6 and old.sum() == 99
    args.output.mkdir(parents=True, exist_ok=False)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.3), layout='constrained')
    colors = ('#4477aa', '#dd9944', '#228866')
    groups = (useful, harmful, old)
    for index, (field, color) in enumerate(zip(fields, colors)):
        sample = [values[field][mask & np.isfinite(values[field])] for mask in groups]
        boxes = axes[0].boxplot(sample, positions=np.arange(3) + (index - 1) * .23,
            widths=.19, patch_artist=True, manage_ticks=False,
            medianprops={'color': 'black'}, flierprops={'markersize': 2})
        for box in boxes['boxes']:
            box.set(facecolor=color, alpha=.5)
        axes[0].plot([], [], color=color, linewidth=6,
            label=('Observer change u', 'Seam motion s', 'Combined r')[index])
        axes[0].scatter(np.full(harmful.sum(), 1 + (index - 1) * .23),
                         values[field][harmful], color=color, s=20, zorder=4)
    axes[0].set(xticks=[0, 1, 2], xticklabels=['Useful corrections\n294 frames',
        'Harmful overrides\n6 frames', 'Old forced-loss subset\n99 of the useful frames'],
        ylabel='Condition score (heuristic; not a probability)', ylim=(-.03, 1.05),
        title='The condition scores overlap')
    axes[0].legend(loc='lower left', fontsize=8)
    axes[0].grid(axis='y', alpha=.2)
    independent = json.loads((root / 'review/results_stage01/report.json').read_text())
    bounds = independent['bounds']['full']
    numbers = [bounds[key]['continuously_forced_useful_losses'] for key in
               ('constant', 'joint_load_risk_u', 'combined_risk_r')]
    bars = axes[1].bar([0, 1, 2], numbers, color=['#888888', '#4477aa', '#228866'], width=.6)
    axes[1].bar_label(bars, padding=3)
    axes[1].axhline(6, color='#b43333', linestyle='--', label='Only 6 harmful changes can be prevented')
    axes[1].set(xticks=[0, 1, 2], xticklabels=['Shared rule\nI-62 reference',
        'Observer-only\ndescriptive bound', 'Combined dynamic\nprimary rule'],
        ylabel='Useful corrections that must be discarded', ylim=(0, 115),
        title='Necessary loss to remove the worst point\nNo adaptive schedule was fitted')
    axes[1].legend(loc='upper right', fontsize=7.5)
    axes[1].grid(axis='y', alpha=.2)
    fig.suptitle('I-63: causal condition signals do not rescue this load-check rule', fontsize=13)
    fig.savefig(args.output / 'overview.png', dpi=180)
    fig.savefig(args.output / 'condition_diagnostic.pdf')
    plt.close(fig)
    event_count = 0
    with PdfPages(args.output / 'event_windows.pdf') as pdf:
        for case in CASES:
            trace = load(analysis / (case + '_event_traces.npz'))
            base = load(root / 'portable_inputs/i62_base' / (case + '.npz'))
            centers = json.loads((analysis / (case + '_event_centers.json')).read_text())
            events = {}
            for item in centers:
                events.setdefault(round(item['time_s'] * 1e9),
                    dict(time=item['time_s'], reasons=[]))['reasons'].append(item['reason'])
            for event in events.values():
                center = event['time']; selected = abs(trace['time_s'] - center) <= .10 + 1e-9
                t = trace['time_s'][selected]; dt_ms = (t - center) * 1000
                idx = np.searchsorted(base['time_s'], t)
                np.testing.assert_array_equal(base['time_s'][idx], t)
                fig, panels = plt.subplots(4, 1, figsize=(10, 8), sharex=True, layout='constrained')
                for key, label, color, linestyle in [('truth_family', 'Dominant truth', 'black', '-'),
                    ('N_family', 'Native N', '#4477aa', '--'), ('A_family', 'Velocity policy A', '#dd9944', ':')]:
                    panels[0].step(dt_ms, trace[key][selected], where='post',
                                   color=color, linestyle=linestyle, label=label)
                panels[0].set_yticks([-1, 0, 1, 2, 3], labels=['Unknown', 'Shell A', 'Shell B', 'Belt A', 'Belt B'])
                panels[0].set(ylabel='Support family', ylim=(-1.3, 3.3))
                panels[0].legend(loc='upper right', fontsize=7)
                mismatch = trace['joint_max_abs_Nm'][selected]
                original = .03 + (.10 * trace['radius_m'][selected]) * trace['force_norm_N'][selected]
                panels[1].plot(dt_ms, mismatch, label='Candidate joint-load mismatch')
                panels[1].plot(dt_ms, original, '--', label='Original I-61 tolerance')
                panels[1].set_ylabel('N m'); panels[1].legend(loc='upper right', fontsize=7)
                for field, label, color in zip(fields, ('Observer u', 'Seam s', 'Combined r'), colors):
                    panels[2].plot(dt_ms, trace[field][selected], label=label, color=color)
                panels[2].set(ylabel='Condition score', ylim=(-.03, 1.05))
                panels[2].legend(loc='lower right', fontsize=7)
                A_distance = np.where(base['i60_accepted'][idx], base['proposed_distance_m'][idx],
                                      base['native_distance_m'][idx])
                matched = base['point_mask'][idx]
                panels[3].plot(dt_ms, np.where(matched, base['native_distance_m'][idx] * 1000, np.nan), label='Native N')
                panels[3].plot(dt_ms, np.where(matched, A_distance * 1000, np.nan), label='Velocity policy A')
                panels[3].set(ylabel='Point discrepancy\n(mm)', xlabel='Time relative to marked event (ms)', xlim=(-100, 100))
                panels[3].legend(loc='upper right', fontsize=7)
                for panel in panels:
                    panel.axvline(0, color='#b43333', linewidth=.8)
                    panel.grid(alpha=.18)
                fig.suptitle(f'{case}, {center:.3f} s: ' + ', '.join(event['reasons']), fontsize=10)
                pdf.savefig(fig); plt.close(fig); event_count += 1
    (args.output / 'plot_scope.json').write_text(json.dumps(dict(
        event_pages=event_count, context_window_s=.10,
        scope='Registered event centers; displayed later samples are plot context only. Every inferred condition uses current/past samples. Labels are evaluator-only. No fitted adaptive policy or driving result.'), indent=2) + '\n')
    print(json.dumps(dict(event_pages=event_count)))


if __name__ == '__main__':
    main()
