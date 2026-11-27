"""Plot the sealed I-62 grid; no fitting or parameter selection occurs here."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import SymLogNorm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    science = json.loads((args.analysis / 'science.json').read_text())
    with np.load(args.analysis / 'all_grid_metrics.npz', allow_pickle=False) as archive:
        grid = {key: archive[key].copy() for key in archive.files}
    base = science['full_data_fit']['baseline']
    delta = grid['all__error_count'] - base['A']['error_count']
    removed = base['A_harmful_count'] - grid['all__harmful_count']
    lost = base['A_useful_count'] - grid['all__useful_retained_count']
    maximum = grid['all__point_maximum_mm']
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.4), layout='constrained')
    norm = SymLogNorm(linthresh=1, vmin=min(-1, int(delta.min())),
                     vmax=max(1, int(delta.max())), base=10)
    heat = axes[0].imshow(delta.reshape(61, 61), origin='lower',
        extent=(-.0025, .3025, -.0025, .3025), aspect='auto',
        cmap='RdBu_r', norm=norm, interpolation='nearest')
    axes[0].plot(.10, .03, marker='*', markersize=12, color='#202020',
                 markeredgecolor='white', label='I-61 original check')
    axes[0].set(xlabel='Relative tolerance b', ylabel='Absolute tolerance a (N m)',
                title='Label-error change versus I-60 A\nAll 3,721 settings, all three starts')
    axes[0].legend(loc='upper right', fontsize=8)
    fig.colorbar(heat, ax=axes[0], label='Extra wrong labels (negative is better)',
                 shrink=.8)
    scatter = axes[1].scatter(removed, lost, c=maximum, s=16, alpha=.7,
                              cmap='viridis', rasterized=True)
    bound = np.linspace(0, base['A_harmful_count'], 100)
    axes[1].fill_between(bound, 0, bound, color='#3e9b65', alpha=.13,
                         label='No increase in total label errors')
    axes[1].plot(bound, bound, color='#3e9b65', linewidth=1)
    original = int(np.flatnonzero((grid['a_index'] == 6) & (grid['b_index'] == 20))[0])
    axes[1].plot(removed[original], lost[original], marker='D', color='#cc3a31',
                 markersize=7, label='I-61 original check')
    axes[1].plot(0, 0, marker='x', color='black', markersize=9,
                 markeredgewidth=2, label='No added check (A)')
    axes[1].set_yscale('symlog', linthresh=3)
    axes[1].set(xlabel='Harmful overrides prevented (out of 6)',
                ylabel='Useful corrections discarded (out of 294)',
                title='Safety benefit versus lost corrections\nColor shows the remaining point-error peak',
                xlim=(-.25, base['A_harmful_count']+.25),
                ylim=(-.2, base['A_useful_count']*1.1))
    axes[1].set_xticks(range(base['A_harmful_count']+1))
    axes[1].set_yticks([0, 1, 3, 6, 10, 30, 100, 294],
                       labels=['0', '1', '3', '6', '10', '30', '100', '294'])
    axes[1].grid(alpha=.18)
    axes[1].legend(loc='upper left', fontsize=7.5)
    fig.colorbar(scatter, ax=axes[1], label='Maximum point discrepancy (mm)', shrink=.8)
    fig.suptitle('I-62: retrospective tolerance tuning; live controller unchanged', fontsize=12)
    args.output.mkdir(parents=True, exist_ok=False)
    fig.savefig(args.output / 'overview.png', dpi=180)
    fig.savefig(args.output / 'tolerance_tradeoffs.pdf')
    plt.close(fig)


if __name__ == '__main__':
    main()
