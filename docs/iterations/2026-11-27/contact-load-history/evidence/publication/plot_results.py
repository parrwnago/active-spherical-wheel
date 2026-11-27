"""Plot sealed I-64 model consistency and registered event windows; no fitting."""
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
        return {key: z[key].copy() for key in z.files}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root, out = args.package, args.output
    analysis = root / 'analysis/stage01'
    science = json.loads((analysis / 'science.json').read_text())
    out.mkdir(parents=True, exist_ok=False)
    q1 = science['Q1_model_consistency']; q2 = science['Q2_discrimination']['counts']
    old = [q1['per_case'][c]['M0']['vector_RMS_Nm'] for c in CASES] + [q1['M0']['vector_RMS_Nm']]
    new = [q1['per_case'][c]['M2']['vector_RMS_Nm'] for c in CASES] + [q1['M2']['vector_RMS_Nm']]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), layout='constrained')
    x = np.arange(4)
    for dx, values, label, color in [(-.18, old, 'Original instantaneous model', '#bb6677'),
                                    (.18, new, 'Matched A-history model', '#4477aa')]:
        bars = axes[0].bar(x + dx, values, width=.34, label=label, color=color)
        axes[0].bar_label(bars, labels=[f'{v:.3f}' for v in values], padding=2, fontsize=8)
    axes[0].set(xticks=x, xticklabels=['−1.0 rad', '+0.3 rad', '+0.9 rad', 'Pooled'],
        ylabel='Joint-load vector RMS error (N m)', ylim=(0, .1),
        title=f'Load-model consistency improves\nPooled RMS falls {100*(1-new[-1]/old[-1]):.1f}%')
    axes[0].legend(fontsize=8); axes[0].grid(axis='y', alpha=.18)
    for dx, model, color in [(-.18, 'M0', '#bb6677'), (.18, 'M2', '#4477aa')]:
        counts = [q2[model]['correct_useful'], q2[model]['correct_harmful']]
        denominators = [294, 6]
        percentages = [100*a/b for a, b in zip(counts, denominators)]
        bars = axes[1].bar(np.arange(2) + dx, percentages, width=.34, color=color)
        axes[1].bar_label(bars, labels=[f'{a}/{b}' for a, b in zip(counts, denominators)],
                           padding=2, fontsize=9)
    axes[1].set(xticks=[0, 1], xticklabels=['Useful corrections:\nprefer the correct proposal',
        'Harmful overrides:\nprefer the native-family candidate'],
        ylabel='Correct current-family preference (%)', ylim=(0, 105),
        title='Current contact discrimination still fails\nSame frozen physical alternatives')
    axes[1].grid(axis='y', alpha=.18)
    fig.suptitle('I-64: matching the filter improves consistency, but does not qualify the load veto', fontsize=12)
    fig.savefig(out / 'overview.png', dpi=180)
    fig.savefig(out / 'load_history_comparison.pdf')
    plt.close(fig)
    pages = 0
    with PdfPages(out / 'event_windows.pdf') as pdf:
        for case in CASES:
            trace = load(analysis / (case + '_event_traces.npz'))
            primitive = load(root / 'features/portable_stage01' / case / 'primitives.npz')
            centers = science['events'][case]['centers']
            combined = {}
            for event in centers:
                combined.setdefault(round(event['time_s'] * 1e9),
                    dict(time=event['time_s'], reasons=[]))['reasons'].append(event['reason'])
            for event in combined.values():
                center = event['time']; mask = abs(trace['time_s'] - center) <= .1 + 1e-9
                t = trace['time_s'][mask]; x = (t - center) * 1000
                idx = np.searchsorted(primitive['time_s'], t)
                np.testing.assert_array_equal(primitive['time_s'][idx], t)
                measured = primitive['observer_generalized'][idx, 6:9]
                fig, panels = plt.subplots(2, 3, figsize=(13, 7.5), sharex=True, layout='constrained')
                for joint, name in enumerate(('Shaft', 'Hemisphere A', 'Hemisphere B')):
                    panel = panels[0, joint]
                    panel.plot(x, measured[:, joint], color='black', label='Filtered observer')
                    panel.plot(x, trace['M0_F_prediction_Nm'][mask, joint], '--',
                               color='#bb6677', label='Instantaneous F')
                    panel.plot(x, trace['M2_F_prediction_Nm'][mask, joint],
                               color='#4477aa', label='Matched F')
                    panel.plot(x, trace['M2_Nalt_prediction_Nm'][mask, joint], ':',
                               color='#228866', label='Matched native-family candidate')
                    panel.set(title=name, ylabel='Generalized load (N m)')
                    if joint == 0: panel.legend(fontsize=6.5)
                panel = panels[1, 0]
                for key, name, style, color in [('truth_family', 'Dominant truth', '-', 'black'),
                    ('N_family', 'Native N', '--', '#4477aa'), ('A_family', 'Velocity policy A', ':', '#dd9944')]:
                    panel.step(x, trace[key][mask], where='post', label=name, linestyle=style, color=color)
                panel.set_yticks([-1, 0, 1, 2, 3], labels=['Unknown', 'Shell A', 'Shell B', 'Belt A', 'Belt B'])
                panel.set(title='Current family', ylabel='Support')
                panel.legend(fontsize=6.5)
                panel = panels[1, 1]
                for model, color, text in [('M0', '#bb6677', 'Instantaneous'), ('M2', '#4477aa', 'Matched')]:
                    panel.plot(x, trace[model + '_F_score'][mask], color=color, label=text + ' F')
                    panel.plot(x, trace[model + '_Nalt_score'][mask], '--', color=color, label=text + ' native')
                panel.axhline(1, color='gray', linewidth=.7)
                panel.set(title='Candidate scores / old fixed tolerance', ylabel='Max normalized mismatch')
                panel.legend(fontsize=6.5)
                panel = panels[1, 2]
                panel.plot(x, trace['M2_F_inherited_norm_share'][mask], label='Inherited history', color='#228866')
                panel.plot(x, trace['M2_F_current_norm_share'][mask], label='Current interval', color='#4477aa')
                panel.set(title='Residual norm-sum shares (auxiliary)', ylabel='Fraction', ylim=(-.03, 1.03))
                panel.legend(fontsize=6.5)
                for panel in panels.flat:
                    panel.axvline(0, color='#b43333', linewidth=.7)
                    panel.set_xlim(-100, 100); panel.grid(alpha=.17)
                for panel in panels[1]: panel.set_xlabel('Time relative to event (ms)')
                accepted_index = int(np.argmin(abs(t - center)))
                accepted = bool(trace['i60_accepted'][mask][accepted_index])
                fig.suptitle(f'{case}, {center:.3f} s; F accepted by I-60: {accepted}\n' + ', '.join(event['reasons']), fontsize=10)
                pdf.savefig(fig); plt.close(fig); pages += 1
    (out / 'plot_scope.json').write_text(json.dumps(dict(event_pages=pages, window_s=.1,
        scope='Registered event windows; F is a hypothetical raw proposal when not accepted. Later observations are plot context only. No diagnostic changes any history or command.'), indent=2) + '\n')
    print(json.dumps(dict(event_pages=pages)))


if __name__ == '__main__':
    main()
