"""Plot the sealed post-primary exploratory baseline audit; no new reduction."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if (HERE/'baseline_yaw_comparison.png').exists():
        raise FileExistsError('Preserve earlier scientific figure')
    report_path = HERE/'report.json'
    initial_hash = sha(report_path)
    (HERE/'plot_registered_before_rendering.json').write_text(json.dumps(dict(
        registered_utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)),
        sealed_report_sha256=initial_hash,
        scope='Render all six absolute world-Z yaw values at both existing events and both existing horizons. Post-primary exploratory; no new scores, model fitting or physics.',
        outputs=['baseline_yaw_comparison.png', 'baseline_yaw_comparison.pdf']), indent=2)+'\n')
    report = json.loads(report_path.read_text())
    plt.rcParams.update({'font.size': 11, 'axes.titlesize': 13, 'axes.labelsize': 11,
                         'pdf.fonttype': 42, 'ps.fonttype': 42})
    fig, axes = plt.subplots(2, 2, figsize=(14.8, 9.4))
    fig.subplots_adjust(left=.075, right=.98, top=.83, bottom=.21, hspace=.42, wspace=.22)
    fig.suptitle('I57: a good differential response can still have the wrong absolute-velocity baseline',
                 fontsize=17, weight='bold', y=.975)
    fig.text(.5, .928, 'POST-PRIMARY EXPLORATORY · Selected after differential G agreement was known · Two saved held-input states',
             ha='center', fontsize=11, color='#813b42')
    keys = ('current_proxy_native', 'actual_native', 'actual_governed',
            'current_proxy_governed', 'frozen_affine_native', 'frozen_affine_governed')
    labels = ('Current', 'Actual\nnative', 'Actual\ngoverned', 'Current\n+ Gδu', 'Affine\nnative', 'Affine\ngoverned')
    colors = ('#89949c', '#d99737', '#348d72', '#a665ad', '#dbb58b', '#5f85b3')
    for row, (event, description) in enumerate((('early_shellB', 'Early shell B · saved t = 4.855 s'),
                                               ('late_beltB', 'Late belt B · saved t = 35.925 s'))):
        values_all = [report['events'][event]['horizons'][h]['vectors'][key][9]
                      for h in ('h005ms', 'h020ms') for key in keys]
        low, high = min(0., min(values_all)), max(0., max(values_all))
        span = high-low
        low, high = low-.13*span, high+.21*span
        for column, (horizon, duration) in enumerate((('h005ms', '5 ms'), ('h020ms', '20 ms'))):
            ax = axes[row, column]
            data = report['events'][event]['horizons'][horizon]['vectors']
            values = np.array([data[key][9] for key in keys])
            ax.bar(np.arange(6), values, width=.74, color=colors, edgecolor='white', linewidth=.7, zorder=3)
            ax.axhline(0, color='#313942', linewidth=.8)
            ax.set_xticks(np.arange(6), labels)
            ax.set_ylim(low, high)
            ax.set_ylabel('Physical world-Z ω [rad/s]')
            ax.set_title(description+' | '+duration, loc='left')
            ax.grid(axis='y', color='#dce1e5', linewidth=.7, zorder=0)
            ax.spines[['top', 'right']].set_visible(False)
            for x, value in enumerate(values):
                offset = (.035 if value >= 0 else -.035)*span
                ax.text(x, value+offset, f'{value:+.5f}', ha='center',
                        va='bottom' if value >= 0 else 'top', fontsize=9)
    late = report['events']['late_beltB']['horizons']['h020ms']['vectors']
    error = late['governed_current_proxy_error'][9]
    native = late['omitted_native_baseline_drift'][9]
    differential = late['paired_G_delta_error'][9]
    fig.text(.075, .118,
        f'Late 20 ms proxy error:  actual governed − (current + Gδu) = {error:+.6f} rad/s\n'
        f'                                 = omitted native future change {native:+.6f} + paired Gδu error {differential:+.6f}',
        fontsize=12, color='#182731', linespacing=1.55,
        bbox=dict(boxstyle='round,pad=.65', facecolor='#eef2f5', edgecolor='#b6c0c8'))
    fig.text(.075, .038,
        'Affine = frozen offset + G·u. Offsets are diagnostic: early-state planar/transverse errors can worsen.\n'
        'No general offset adoption, new acceptance gate, or attribution of the full trajectory deviation is established.',
        fontsize=10, color='#4b5660', linespacing=1.45)
    for suffix in ('png', 'pdf'):
        fig.savefig(HERE/('baseline_yaw_comparison.'+suffix), dpi=200, facecolor='white')
    plt.close(fig)
    assert sha(report_path) == initial_hash
    paths = [Path(__file__), HERE/'plot_registered_before_rendering.json',
             HERE/'baseline_yaw_comparison.png', HERE/'baseline_yaw_comparison.pdf']
    (HERE/'plot_artifact_manifest.json').write_text(json.dumps(dict(
        report_hash_unchanged=True, sealed_report_sha256=initial_hash,
        files=[dict(path=p.name, sha256=sha(p), bytes=p.stat().st_size) for p in paths]), indent=2)+'\n')
    print(json.dumps(dict(report_hash_unchanged=True, files=[p.name for p in paths])))


if __name__ == '__main__':
    main()
