"""Copy I-56 evidence without raw full rollouts, credentials, or build trees."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'results/constrained_handoff_20261126'
DEST = ROOT / 'docs/iterations/2026-11-26/constrained-handoff/evidence'
EXCLUDE = {'__pycache__', '.pytest_cache', '.git'}


def copy_file(src, relative):
    dst = DEST / relative
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() and dst.read_bytes() != src.read_bytes():
        raise RuntimeError(f'Refuse to overwrite changed evidence: {dst}')
    shutil.copy2(src, dst)


def copy_tree(relative):
    for src in sorted((SOURCE / relative).rglob('*')):
        if (src.is_file() and src.suffix != '.mp4' and
                not EXCLUDE.intersection(src.relative_to(SOURCE).parts)):
            copy_file(src, src.relative_to(SOURCE))


for directory in (
    'pre_change', 'model_checks_stage01', 'model_checks_stage02',
    'control_checks_stage01', 'control_checks_stage02_preparation_failure',
    'control_checks_stage03_corrected', 'control_checks_stage04_final',
    'integration_checks_stage01', 'integration_checks_stage02_final',
    'model_sensitivity_stage01', 'model_sensitivity_stage02',
    'review', 'campaign_tools', 'analysis',
):
    copy_tree(directory)
for name in (
    'protocol.json', 'execution_registration.json', 'preflight.json',
    'ablation/protocol.json', 'ablation/preflight.json',
    'model_final_report.json', 'check_model_sensitivity.py',
    'check_model_sensitivity_stage02.py', 'model_sensitivity_execution.log',
    'model_sensitivity_stage02_execution.log', 'publish_compact.py',
    'media_publication.json',
    'video_tools/README.md', 'video_tools/render_comparison.py',
    'video_tools/presentation_plan.json',
    'video_tools/compact_media_record.tar.gz',
    'video_tools/compact_media_record_manifest.json',
    'video_tools/qa_stage01/report.json',
    'm100_B.log', 'm100_A.log', 'm100_H.log',
):
    src = SOURCE / name
    if src.exists():
        copy_file(src, Path(name))
for case, prefix in [('m100_B', ''), ('m100_A', 'ablation'), ('m100_H', ''),
                     ('p030_B', ''), ('p030_H', ''), ('p090_B', ''), ('p090_H', '')]:
    relative = Path(prefix) / 'campaign/runs' / case
    path = SOURCE / relative
    if not path.exists():
        continue
    for name in ('summary.json', 'execution_result.json', 'provenance_before.json',
                 'provenance.json', 'artifact_manifest.json', 'terrain.json',
                 'historical_precision_comparison.json'):
        if (path / name).exists():
            copy_file(path / name, relative / name)
    if case == 'm100_H':
        copy_tree(relative / 'source_snapshot')

files = []
for p in sorted(DEST.rglob('*')):
    if p.is_file() and p.name != 'portable_manifest.json':
        files.append(dict(path=str(p.relative_to(DEST)), bytes=p.stat().st_size,
                          sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(DEST / 'portable_manifest.json').write_text(json.dumps(dict(
    iteration='I-56', source_root=str(SOURCE.relative_to(ROOT)),
    excludes='Full rollout.npz, precision.npz and governor_diagnostics.json remain local; exact compact analysis inputs are separately identified.',
    files=files), indent=2) + '\n')
print(json.dumps(dict(files=len(files), bytes=sum(x['bytes'] for x in files))))
