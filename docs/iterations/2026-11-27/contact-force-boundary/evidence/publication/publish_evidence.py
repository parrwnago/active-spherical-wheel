"""I69 compact export; inputs are finalized before copying, no self-seal."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib,json,shutil
ROOT=Path(__file__).resolve().parents[3];BASE=Path(__file__).resolve().parents[1]
DEST=ROOT/'docs/iterations/2026-11-27/contact-force-boundary/evidence'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    if DEST.exists():raise FileExistsError(DEST)
    selected=set()
    for folder in ('bench','instrumentation','campaign_tools','source_review','publication','capture_launch_stage01'):
        for p in (BASE/folder).rglob('*'):
            if not p.is_file() or '__pycache__' in p.parts:continue
            rel=p.relative_to(BASE)
            if p.suffix in ('.so','.bin') or p.name=='controlled_probe':continue
            if p.name=='export_plan_before_copy.json':continue
            if any(x in rel.parts for x in ('replay_producer_stage01','replay_system_stage01')):continue
            selected.add(p)
    selected.update((BASE/'protocol.json',BASE/'phase_B_protocol.json'))
    native=BASE/'campaign/native_solver_capture'
    for name in ('capture_receipt.json','i58_precision_comparison.json','i68_trace_comparison.json'):
        selected.add(native/name)
    for name in ('summary.json','provenance.json','provenance_before.json','terrain.json','historical_baseline_comparison.json'):
        p=native/'campaign/runs/m100_B'/name
        if p.is_file():selected.add(p)
    plan=[dict(origin=str(p.relative_to(ROOT)),path=str(p.relative_to(BASE)),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(selected)]
    seal=dict(iteration='I-69',registered_utc=datetime.now(timezone.utc).isoformat(),publisher_sha256=sha(__file__),files=plan,
        omissions=['Full rollout/precision/trace and compiledprivatehook/executable remain local; original hashes and parity receipts retained.',
            'Original local solver binary packet contains uninitialized nonmathematical matrixpadding; compact shared mathematicalpacket deliberately excludespadding.',
            'Only20selectedmatchingcontexts are packed, with original globalindices/unknownholes explicit. This is not a restoredfulltrace.',
            'Derived local producer/system replay directories omitted; regeneratefromtrackedportableinputs.',
            'Original I58 runtime source closure is already tracked with I68; sourceprotocol identifies exactoriginalpaths/hashes.'])
    path=BASE/'publication/export_plan_before_copy.json';path.write_text(json.dumps(seal,indent=2)+'\n')
    DEST.mkdir(parents=True)
    for row in plan:
        source=ROOT/row['origin'];target=DEST/row['path'];target.parent.mkdir(parents=True,exist_ok=True)
        assert sha(source)==row['sha256'],source;shutil.copyfile(source,target);assert sha(target)==row['sha256'],target
    shutil.copyfile(path,DEST/'export_plan_before_copy.json')
    manifest=dict(iteration='I-69',files=[dict(path=str(p.relative_to(DEST)),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(DEST.rglob('*')) if p.is_file()])
    (DEST/'archive_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(files=len(manifest['files']),bytes=sum(x['bytes'] for x in manifest['files']),destination=str(DEST))))


if __name__=='__main__':main()
