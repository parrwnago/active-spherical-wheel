"""Curated I68 evidence export; excludes full/derivable local rollouts."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
DEST=ROOT/'docs/iterations/2026-11-27/contact-interval-replay/evidence'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    if DEST.exists():raise FileExistsError(DEST)
    chosen=set()
    for folder in ('campaign_tools','instrumentation','analysis','review','publication','launch_stage01'):
        for p in (BASE/folder).rglob('*'):
            if not p.is_file() or '__pycache__' in p.parts:continue
            rel=p.relative_to(BASE)
            if 'qa_replay_producer_stage01' in rel.parts:continue
            if 'stage02' in rel.parts and p.suffix=='.npz':continue
            chosen.add(p)
    chosen.add(BASE/'protocol.json')
    capture=BASE/'campaign/native_substeps'
    for name in ('i68_provenance_before.json','i68_provenance.json',
        'i68_generated_models_before_control.json','i58_precision_comparison.json','i68_artifact_manifest.json'):
        chosen.add(capture/name)
    for folder in ('i68_source_snapshot','i68_models'):
        chosen.update(p for p in (capture/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    native=capture/'campaign/runs/m100_B'
    for p in native.glob('*.json'):
        if p.name=='governor_diagnostics.json':continue
        chosen.add(p)
    plan=[]
    for p in sorted(chosen):
        assert p.is_file(),p
        plan.append(dict(origin=str(p.relative_to(ROOT)),path=str(p.relative_to(BASE)),bytes=p.stat().st_size,sha256=sha(p)))
    seal=dict(iteration='I-68',registered_utc=datetime.now(timezone.utc).isoformat(),
        publisher_sha256=sha(__file__),file_count=len(plan),files=plan,
        omissions=['Full32MB original capture,13MB complete precision, native rollout and generated assets remain local.',
            'The redundant full native governor_diagnostics.json remains local; essential applied commands are included in the portable precision package.',
            'Derived stage02 interval/substep NPZ arrays and producer-replay duplicates omitted; all regenerate from the tracked lossless payload.',
            'Archived stage02 artifact_manifest records those original local files; it is not a claim they are all copied.'],
        scope='Compact evidence export, no numerical changes. Lossless capture payload plus essential precision, source/protocol, reports, tests and figures retained.')
    (BASE/'publication/export_plan_before_copy.json').write_text(json.dumps(seal,indent=2)+'\n')
    DEST.mkdir(parents=True)
    for row in plan:
        source=ROOT/row['origin'];target=DEST/row['path'];target.parent.mkdir(parents=True,exist_ok=True)
        assert sha(source)==row['sha256'];shutil.copyfile(source,target);assert sha(target)==row['sha256']
    shutil.copyfile(BASE/'publication/export_plan_before_copy.json',DEST/'export_plan_before_copy.json')
    manifest=dict(iteration='I-68',files=[dict(path=str(p.relative_to(DEST)),bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(DEST.rglob('*')) if p.is_file()])
    (DEST/'archive_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(files=len(manifest['files']),bytes=sum(x['bytes'] for x in manifest['files']),destination=str(DEST))))


if __name__=='__main__':main()
