"""I52 read-only protection audit, separate from scientific reduction."""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[3]
FAMILY=ROOT/'results/contact_scheduling_joint_tracking_20261126'
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
pre=json.loads((FAMILY/'pre_change/manifest.json').read_text())
allowed={'ITERATION_LOG.md','docs/iterations/2026-11-26/README.md'}
changed=[x['path'] for x in pre['sources'] if x['path'] not in allowed and sha(ROOT/x['path'])!=x['sha256']]
archives=[x['path'] for x in pre['prior_archives'] if sha(ROOT/x['path'])!=x['sha256']]
protocols={}
for rel in ['protocol.json']:
    p=json.loads((FAMILY/rel).read_text())
    source_changed=[path for path,h in p['source_sha256'].items() if sha(ROOT/path)!=h]
    input_changed=[name for name,h in p['input_sha256'].items() if sha(ROOT/p['inputs'][name])!=h]
    protocols[rel]=dict(sha256=sha(FAMILY/rel),source_changes=source_changed,input_changes=input_changed,
                        passed=not source_changed and not input_changed)
out=dict(passed=not changed and not archives and all(p['passed'] for p in protocols.values()),
    protected_origins=len(pre['sources']),authorized_document_updates=sorted(allowed),
    changes_outside_authorized_docs=changed,prior_archive_count=len(pre['prior_archives']),
    changed_archives=archives,protocols=protocols,source_sha256=sha(Path(__file__)))
(FAMILY/'review/protected_inputs.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out))
if not out['passed']:raise SystemExit(1)
