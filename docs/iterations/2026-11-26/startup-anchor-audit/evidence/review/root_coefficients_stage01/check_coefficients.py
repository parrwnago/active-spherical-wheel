"""Independent small coefficient check; input-only, no physics or controller."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
RUNS = Path('results/balance_priority_20261126/campaign/runs')
cases = {}
for case in ('m100_B', 'm100_P', 'p030_B', 'p030_P', 'p090_B', 'p090_P'):
    path = RUNS / case / 'precision.npz'
    with np.load(path, allow_pickle=False) as data:
        row = data['exact_feedback_gain'][:, 0, :]
        trim = data['exact_equilibrium_command']
        cases[case] = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                      'row': row[0].tolist(), 'trim': trim[0].tolist(),
                      'row_constant_exact': bool(np.array_equal(row, np.broadcast_to(row[0], row.shape))),
                      'trim_constant_exact': bool(np.array_equal(trim, np.broadcast_to(trim[0], trim.shape))),
                      'row_constancy_max_abs': float(np.max(abs(row-row[0]))),
                      'trim_constancy_max_abs': float(np.max(abs(trim-trim[0])))}
pairs = []
for left, right in (('m100_B', 'p030_B'), ('m100_B', 'p090_B'), ('p030_B', 'p090_B')):
    a, b = cases[left], cases[right]
    pairs.append({'left': left, 'right': right,
                  'row_max_abs': float(np.max(abs(np.array(a['row'])-np.array(b['row'])))),
                  'trim_max_abs': float(np.max(abs(np.array(a['trim'])-np.array(b['trim']))))})
output = {'cases': cases, 'pairs': pairs, 'boundary': 'Coefficients only; no trajectory attribution'}
(ROOT/'report.json').write_text(json.dumps(output, indent=2)+'\n')
print(json.dumps({'pairs': pairs, 'constant_rows': all(c['row_constant_exact'] for c in cases.values()),
                  'constant_trims': all(c['trim_constant_exact'] for c in cases.values())}))
