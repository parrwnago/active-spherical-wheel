"""Final read-only publication QA, including both archived full pilot prefixes."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import numpy as np

ROOT = Path.cwd()
BASE = ROOT/'results/loaded_contact_validation_20261126'
DOC = ROOT/'docs/iterations/2026-11-26/loaded-contact-validation/evidence'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest = json.loads((DOC/'portable_manifest.json').read_text())
assert all(sha(DOC/x['path']) == x['sha256'] for x in manifest['files'])
protocol = json.loads((BASE/'protocol.json').read_text())
assert all(sha(ROOT/p) == v for p, v in protocol['source_sha256'].items())
assert len(list((BASE/'campaign/runs').iterdir())) == 30
assert not list((BASE/'campaign/runs').rglob('failure.json'))
projected = ROOT/'docs/iterations/2026-11-26/constrained-handoff/evidence/analysis/stage01/portable_inputs/m100_H_precision_projection.npz'
prefix_checks = {}
with np.load(projected, allow_pickle=False) as original:
    for event in protocol['events']:
        case = event['event_id']+'_continue'
        with np.load(DOC/'campaign/runs'/case/'prefix.npz', allow_pickle=False) as prefix:
            rows = event['index']+1
            for field in ('time_s', 'exact_qpos', 'exact_qvel', 'exact_previous_applied'):
                a, b = prefix[field], original[field][:rows]
                assert a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()
            prefix_checks[case] = dict(rows=rows, exact=True)
report = json.loads((DOC/'analysis/stage01/report.json').read_text())
assert report['all_registered_cases_available'] and report['all_event_controls_valid']
counts = Counter(group['status'] for event in report['events'].values()
                 for pair in event['comparisons'] for group in pair['groups'].values())
assert counts == dict(resolved_pass=118, unresolved=165, resolved_fail=2, false_negligible_authority=3)
replay = json.loads((BASE/'published_replay_stage01/report.json').read_text())
assert replay['canonical_numeric_comparison']['passed'] and replay['input_hashes_unchanged']
result = dict(
    all_listed_portable_hashes_pass=True, source_sha256=sha(__file__),
    frozen_sources_unchanged=len(protocol['source_sha256']),
    new_physics_runs=30, physics_failures=0, controller_defaults_changed=False,
    published_pilot_prefixes_recomputed=prefix_checks,
    original_projection_sha256=sha(projected),
    all_cases_and_controls_valid=True, loaded_status_counts=dict(counts),
    portable_metrics_match_canonical=True,
    full_repo_test_suite_rerun=False,
    whitespace_scope='Authored code/docs checked separately; exact CSV CRLF and historical snapshot whitespace retained.',
    excluded_unrelated=['firmware/', 'HAMR_Ordinary_Caster_DIY_Encoder_Codex_Handoff_ESP32C6.md'])
(Path(__file__).parent/'report.json').write_text(json.dumps(result, indent=2)+'\n')
(BASE/'publication_checks.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
