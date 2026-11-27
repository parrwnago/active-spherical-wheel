"""Final I58 archive, frozen-source and scientific-outcome checks."""
from pathlib import Path
import hashlib
import json

ROOT=Path.cwd();BASE=ROOT/'results/balance_priority_20261126'
DOC=ROOT/'docs/iterations/2026-11-26/balance-priority/evidence'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((DOC/'portable_manifest.json').read_text())
assert all(sha(DOC/r['path'])==r['sha256'] for r in manifest['files'])
protocol=json.loads((BASE/'protocol.json').read_text())
assert all(sha(ROOT/p)==h for p,h in protocol['source_sha256'].items())
pre=json.loads((BASE/'pre_change/manifest.json').read_text())
new={'src/spherical_wheel/balance_priority_control.py','tests/test_balance_priority_control.py'}
assert all(sha(ROOT/r['path'])==r['sha256'] for r in pre['prior_source_records']
    if r['path'].startswith(('src/','tests/','config/','gazebo/')) and r['path'] not in new)
report=json.loads((DOC/'analysis/stage02/report.json').read_text())
assert report['all_registered_cases_available'] and len(report['cases'])==7
assert report['broader_campaign_eligible'] and not report['all_three_phase_screens_pass']
assert report['default_adoption'] is False
expected={
    'm100_P_vs_m100_B':[],
    'p030_P_vs_p030_B':['final_timed_XY','all_six_turn_overshoots'],
    'p090_P_vs_p090_B':['late_yaw_peak','full_planar_speed_peak','all_six_turn_overshoots']}
for key,failed in expected.items():
    assert report['comparisons'][key]['failed_checks']==failed
assert all(c['completed'] and c['full44_coverage'] and not c['original_settling']['passed']
           for c in report['cases'].values())
for name in ('m100_P','p030_P','p090_P'):
    c=report['cases'][name]
    assert c['commands']['applied_box_pass'] and c['commands']['requested_box_pass']
    assert c['priority']['total_response_max_abs']<=1e-8
    assert c['optimizer']['primary_restore_samples']==0
for name,fields in [('m100_B',59),('m100_A',59),('p030_B',56),('p090_B',56)]:
    h=report['cases'][name]['historical_parity']
    assert h['all_shared_fields_exact'] and h['exact_fields']==fields
audit=json.loads((DOC/'review/outcome_stage02/audit_result.json').read_text())
assert all(v['independent_core_metrics_agree'] for v in audit['metric_agreement'].values())
assert all(v['all_gate_booleans_agree'] for v in audit['gate_comparisons'].values())
replay=json.loads((BASE/'published_replay_stage02.json').read_text())
assert replay['all_replayed_metrics_exact']
media=json.loads((BASE/'media_publication.json').read_text())
assert len(media)==3 and all(sha(ROOT/r['destination'])==r['sha256'] for r in media)
result=dict(frozen_sources_unchanged=len(protocol['source_sha256']),
    all_listed_archive_hashes_pass=True,published_NumPy_replay_exact=True,
    all_independent_metrics_and_gates_agree=True,all_7_runs_complete=True,
    final_phase_passes={'m100':True,'p030':False,'p090':False},
    initial_extension_decision=True,final_all_phase_acceptance=False,
    selected_tests=72,new_allocator_tests=31,full_repo_suite_rerun=False,
    existing_runtime_config_defaults_unchanged=True,experimental_new_module_only=True,
    actual_settling_passes=0,media_copies_verified=3,
    controller_default_adopted=False,source_sha256=sha(__file__),
    whitespace_scope='Authored code/docs checked; exact CSV CRLF and historical snapshot whitespace retained.',
    excluded_unrelated=['firmware/','HAMR_Ordinary_Caster_DIY_Encoder_Codex_Handoff_ESP32C6.md'])
(Path(__file__).parent/'report.json').write_text(json.dumps(result,indent=2)+'\n')
(BASE/'publication_checks.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
