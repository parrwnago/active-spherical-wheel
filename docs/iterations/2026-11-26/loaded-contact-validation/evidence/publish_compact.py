"""Preserve I57 diagnostic evidence without duplicating all native workspaces."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path.cwd()
SOURCE = ROOT/'results/loaded_contact_validation_20261126'
DEST = ROOT/'docs/iterations/2026-11-26/loaded-contact-validation/evidence'
EXCLUDE = {'__pycache__', '.pytest_cache', '.git'}


def copy(src, relative):
    dst = DEST/relative
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() and dst.read_bytes() != src.read_bytes():
        raise RuntimeError('Refuse to replace changed evidence: '+str(dst))
    shutil.copy2(src, dst)


def tree(relative):
    folder = SOURCE/relative
    if not folder.exists():
        return
    for p in sorted(folder.rglob('*')):
        if p.is_file() and not EXCLUDE.intersection(p.relative_to(SOURCE).parts):
            copy(p, p.relative_to(SOURCE))


for name in ('pre_change', 'campaign_tools', 'model_tools', 'analysis', 'review',
             'batch_pilots', 'batch_remaining'):
    tree(Path(name))
for name in ('protocol.json', 'register_protocol.py', 'run_batch.py',
             'preflight.json', 'pilots_execution.log', 'remaining_execution.log',
             'publication_checks.json', 'publish_compact.py'):
    p = SOURCE/name
    if p.exists():
        copy(p, Path(name))
case_inventory = []
for folder in sorted((SOURCE/'campaign/runs').iterdir()):
    if not folder.is_dir():
        continue
    row = dict(case_id=folder.name, files={})
    for p in sorted(folder.iterdir()):
        if p.is_file():
            row['files'][p.name] = dict(bytes=p.stat().st_size,
                sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    case_inventory.append(row)
    for name in ('provenance_before.json', 'provenance.json', 'failure.json',
                 'artifact_manifest.json', 'endpoints.npz'):
        if (folder/name).exists():
            copy(folder/name, Path('campaign/runs')/folder.name/name)
    if folder.name.endswith('_continue') and (folder/'prefix.npz').exists():
        copy(folder/'prefix.npz', Path('campaign/runs')/folder.name/'prefix.npz')
    if folder.name == 'early_shellB_governed':
        tree(Path('campaign/runs')/folder.name/'source_snapshot')
(DEST/'case_inventory.json').write_text(json.dumps(dict(
    scope='Full per-case native artifacts remain local. All endpoints and provenance are copied; exact prefix arrays for both pilot controls and one shared executed-source snapshot are included. Remaining prefix arrays/workspaces are identified by hashes.',
    cases=case_inventory), indent=2)+'\n')
files = []
for p in sorted(DEST.rglob('*')):
    if p.is_file() and p.name != 'portable_manifest.json':
        files.append(dict(path=str(p.relative_to(DEST)), bytes=p.stat().st_size,
                          sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(DEST/'portable_manifest.json').write_text(json.dumps(dict(
    iteration='I-57', source_root=str(SOURCE.relative_to(ROOT)),
    full_native_case_count=len(case_inventory), files=files), indent=2)+'\n')
print(json.dumps(dict(files=len(files), bytes=sum(x['bytes'] for x in files), cases=len(case_inventory))))
