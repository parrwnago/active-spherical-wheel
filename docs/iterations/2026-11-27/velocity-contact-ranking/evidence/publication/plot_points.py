"""Show the I-60 point-average gain and extreme-error regression together."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(report, output):
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__), output/'plot_source.py')
    registration = dict(captured_utc=datetime.now(timezone.utc).isoformat(),
        report_sha256=sha(report), source_sha256=sha(Path(__file__)),
        scope='Descriptive post-primary point check, all starts plus pooled; no threshold or score changes.')
    (output/'registered_before_render.json').write_text(json.dumps(registration, indent=2)+'\n')
    data = json.loads(report.read_text())
    rows = [data['cases'][case]['fused']['equivalent_raw']['all_matched']
            for case in ('m100_B', 'p030_B', 'p090_B')]
    rows.append(data['aggregate']['fused']['equivalent_raw']['all_matched'])
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), layout='constrained')
    x = np.arange(4)
    for ax, field, title in zip(axes, ('RMS_mm', 'maximum_mm'),
                               ('Average distance improves', 'Largest distance worsens')):
        for delta, key, label, color in ((-.18, 'native', 'Native point', '#555555'),
                                         (.18, 'candidate', 'Fused selected point', '#228833')):
            values = [row[key][field] for row in rows]
            bars = ax.bar(x+delta, values, width=.34, color=color, label=label)
            ax.bar_label(bars, fmt='%.2f', fontsize=8)
        ax.set(xticks=x, xticklabels=('−1.0 rad', '+0.3 rad', '+0.9 rad', 'Pooled'),
               ylabel='Distance to logged equivalent-point reference (mm)', title=title)
        ax.margins(y=.17)
        ax.legend(fontsize=8)
    fig.suptitle('I-60 | Better contact-family ranking does not eliminate consequential outliers\n'
                 'Paired dominant-support states, 1–41 s; no new physical trajectory')
    fig.supxlabel('Logged references are last-1-ms load-weighted means; radial-R proxies give similar results.\n'
                  'The enlarged extremes occur in remaining wrong-family selections. No live compensation was tested.', fontsize=9)
    fig.savefig(output/'point_discrepancy.pdf')
    fig.savefig(output/'point_discrepancy.png', dpi=170)
    plt.close(fig)
    manifest = dict(files=[dict(path=p.name, sha256=sha(p), bytes=p.stat().st_size)
                          for p in sorted(output.iterdir()) if p.is_file()])
    (output/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.report.resolve(), args.output.resolve())
