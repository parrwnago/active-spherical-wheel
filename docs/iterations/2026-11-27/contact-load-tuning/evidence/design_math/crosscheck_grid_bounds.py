"""Post-release independent crosscheck of canonical grid conditional minima.

This is a verification supplement, not a new selection protocol. Inputs are
the pre-released independent bounds result and canonical grid arrays. It
checks the published descriptive frontier, without changing selection.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--grid', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
bounds = json.loads((HERE/'bounds_audit.json').read_text())
with np.load(args.grid, allow_pickle=False) as archive:
    grid = {k: archive[k].copy() for k in archive.files}
result = dict(source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    grid_sha256=hashlib.sha256(args.grid.read_bytes()).hexdigest(),
    independent_bounds_sha256=hashlib.sha256((HERE/'bounds_audit.json').read_bytes()).hexdigest(),
    groups={})
for subset, b in bounds['subsets'].items():
    group = 'all' if subset=='full' else subset
    errors = grid[group+'__error_count']
    harm = grid[group+'__harmful_count']
    lost = b['useful_A']-grid[group+'__useful_retained_count']
    peak = grid[group+'__point_maximum_mm'] <= b['native_max_mm']+1e-12
    if np.any(errors != b['family_errors_A']+lost-(b['harmful_A']-harm)):
        raise AssertionError('Canonical grid violates count identity: '+group)
    facts = dict(peak_feasible_grid_count=int(peak.sum()),
        grid_minimum_useful_loss_given_peak=int(lost[peak].min()),
        grid_minimum_errors_given_peak=int(errors[peak].min()),
        grid_maximum_harmful_removal_given_peak=int((b['harmful_A']-harm[peak]).max()))
    for key, actual in facts.items():
        if actual != b[key]:
            raise AssertionError('Independent bound mismatch: '+group+' '+key)
    pairs = sorted(set(zip(lost[peak].tolist(),harm[peak].tolist())))
    frontier = []
    smallest = float('inf')
    for n,h in pairs:
        if h < smallest:
            frontier.append(dict(useful_lost=int(n),harmful_remaining=int(h)))
            smallest = h
    if frontier != b['grid_point_peak_label_frontier']:
        raise AssertionError('Independent frontier mismatch: '+group)
    half_harm = peak & ((b['harmful_A']-harm) >= .5*b['harmful_A'])
    facts['minimum_errors_given_peak_and_50pct_harm_removal'] = int(errors[half_harm].min()) if half_harm.any() else None
    facts['frontier_exact_match'] = True
    result['groups'][group] = facts
result['all_checks_passed'] = True
args.output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
