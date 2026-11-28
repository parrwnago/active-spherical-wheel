"""Extract qualified existing I77 evidence; no simulator/controller execution."""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BASE = Path(__file__).resolve().parent
OLD = Path('docs/iterations/2026-11-27/baseline-deviation-cause')
INPUTS = [
    OLD / 'README.md',
    OLD / 'evidence/contact_trace/analysis_stage01/report.json',
    OLD / 'evidence/kinematics/README.md',
    OLD / 'evidence/timeline/findings_stage01/findings.md',
    OLD / 'evidence/timeline/reference_source_stage02/findings.md',
    OLD / 'evidence/component_audit/COMPONENT_AUDIT.md',
    Path('src/spherical_wheel/synchronization.py'),
    Path('src/spherical_wheel/offroad_control.py'),
    Path('src/spherical_wheel/contact_sensing.py'),
    Path('results/tangent_heading_routes_20261127/campaign_tools/run_case.py'),
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = BASE / 'review_stage01'
    out.mkdir(exist_ok=False)
    entries = [{'path': str(p), 'sha256': digest(ROOT / p),
                'bytes': (ROOT / p).stat().st_size} for p in INPUTS]
    source = Path(__file__).read_bytes()
    (out / 'review.py').write_bytes(source)
    receipt = {'iteration': 'I-79', 'scope': 'Retrospective existing-evidence extraction, not independent physics or a blinded test.',
               'source_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
               'source_sha256': hashlib.sha256(source).hexdigest(), 'inputs': entries,
               'no_new_physics': True, 'no_controller_execution': True,
               'no_new_performance_gates': True}
    (out / 'registered_before_extraction.json').write_text(json.dumps(receipt, indent=2) + '\n')
    report = json.loads((ROOT / INPUTS[1]).read_text())
    fields = ['time_s', 'pressure_weighted_solver_normal', 'estimated_normal',
              'estimated_vs_pressure_normal_deg', 'target_up_rate_deg_s',
              'observer_vs_filtered_physical_intrinsic_deg', 'pressure_family_N']
    rows = []
    for when in [96.15, 96.4, 96.5, 96.9, 97.5]:
        row = min(report['rows'], key=lambda x: abs(x['time_s'] - when))
        assert abs(row['time_s'] - when) < 1e-8
        rows.append({k: row[k] for k in fields})
    summary = {'iteration': 'I-79', 'scope': 'Values copied from the prior I77 aligned-contact report, not a new raw-physics reconstruction.',
               'source_iteration': 'I-77', 'source_window_s': [96., 97.5],
               'source_summary': report['summary'], 'selected_existing_rows': rows,
               'actual_terrain_facet_change_verified': False,
               'matched_preview_future_contact_ledger_available': False,
               'wrong_preview_caused_departure_established': False,
               'current_contact_estimate_globally_verified': False,
               'decision': 'Keep native baseline; distinguish preview error, current-estimate timing, and controller response. Instrument a short unchanged replay before causal preview intervention.',
               'no_new_physics': True, 'no_controller_changes': True}
    (out / 'report.json').write_text(json.dumps(summary, indent=2) + '\n')
    assert all(digest(ROOT / x['path']) == x['sha256'] for x in entries)
    files = [{'path': p.name, 'sha256': digest(p), 'bytes': p.stat().st_size}
             for p in sorted(out.iterdir()) if p.is_file()]
    (out / 'manifest.json').write_text(json.dumps({'iteration': 'I-79', 'inputs_unchanged': True, 'files': files}, indent=2) + '\n')
    print(json.dumps({'inputs': len(entries), 'selected_rows': len(rows),
                      'inputs_unchanged': True, 'no_new_physics': True}))


if __name__ == '__main__':
    main()
