"""Preserve I77 compact evidence and a local lossless diagnostic bundle."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import tarfile

ROOT=Path(__file__).resolve().parents[3]
BASE=ROOT/'results/baseline_deviation_cause_20261127'
DOC=ROOT/'docs/iterations/2026-11-27/baseline-deviation-cause'
PUB=BASE/'publication_stage01'
DEST=DOC/'evidence'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()
def write(p,x):
    with Path(p).open('x') as f:json.dump(x,f,indent=2,sort_keys=True);f.write('\n')
def bind(p):return dict(bytes=p.stat().st_size,sha256=sha(p))
def archive(path,files):
    bindings={str(p.relative_to(ROOT)):bind(p) for p in files}
    with tarfile.open(path,'w:gz') as tar:
        for p in files:tar.add(p,arcname=str(p.relative_to(ROOT)),recursive=False)
    count=0
    with tarfile.open(path,'r:gz') as tar:
        members=tar.getmembers()
        if {m.name for m in members}!=set(bindings):raise AssertionError('Archive coverage differs')
        for m in members:
            h=hashlib.sha256();stream=tar.extractfile(m)
            for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
            if m.size!=bindings[m.name]['bytes'] or h.hexdigest()!=bindings[m.name]['sha256']:
                raise AssertionError('Archive member differs: '+m.name)
            count+=1
    return dict(path=str(path.relative_to(ROOT)),**bind(path),verified_members=count,files=bindings)

def main():
    required=[BASE/'component_audit/causal_reduction_stage01/report.json',
              BASE/'contact_trace/native97p5_stage01/parity_receipt.json',
              BASE/'contact_trace/analysis_stage01/artifact_manifest.json',
              BASE/'component_audit/eight_attitude_reduction_stage01/report.json',
              BASE/'kinematics/attitude_precision_verification_stage02/report.json']
    if not all(p.is_file() for p in required):raise RuntimeError('Finish registered diagnostics before curation')
    PUB.mkdir(exist_ok=False);DEST.mkdir(exist_ok=False)
    files=[];excluded=[]
    for p in sorted(BASE.rglob('*')):
        if not p.is_file() or PUB in p.parents or '__pycache__' in p.parts:continue
        if '_workspaces' in p.parts and p.suffix not in ('.xml','.sdf','.yaml','.json'):
            excluded.append(p);continue
        files.append(p)
    original={str(p.relative_to(ROOT)):bind(p) for p in files}
    write(PUB/'registered_before_packaging.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),
        files=original,excluded_rebuildable_workspace_files={str(p.relative_to(ROOT)):bind(p) for p in excluded},
        exclusion='Generated workspace render/mesh/map/cache assets omitted. Exact source/config/model metadata and all new control/contact/raw diagnostic arrays retained. Original I76 input bundle remains a separately identified prerequisite. No control or numeric result changes.'))
    raw=archive(PUB/'I77-baseline-deviation-evidence.tar.gz',files)
    metadata=[p for p in files if p.suffix in ('.py','.json','.jsonl','.md','.csv','.log','.yaml','.xml','.sdf')]
    source=archive(DEST/'source_and_metadata.tar.gz',metadata)
    directories=['timeline/findings_stage01','timeline/reference_source_stage02','kinematics/findings_stage01','kinematics/findings_stage02',
                 'component_audit/causal_publication_stage02','kinematics/causal_precision_verification_stage01',
                 'component_audit/eight_attitude_reduction_stage01','component_audit/eight_attitude_publication_stage01',
                 'kinematics/attitude_precision_verification_stage02']
    selected=set()
    for name in directories:
        folder=BASE/name
        if folder.is_dir():selected.update(p for p in folder.rglob('*') if p.is_file() and p.suffix!='.npz' and '__pycache__'not in p.parts)
    singles=['timeline/late_eight_stage02/fixed_100ms_table.csv','timeline/late_eight_stage02/report.json',
        'kinematics/README.md','kinematics/velocity_decomposition_stage02/report.json','kinematics/velocity_decomposition_stage02/registered_before_calculation.json',
        'component_audit/COMPONENT_AUDIT.md','component_audit/causal_reduction_stage02/report.json',
        'component_audit/causal_branches_stage01/protocol.json','component_audit/runner_contracts_stage03/receipt.json',
        'component_audit/runner_contracts_stage03/unittest.log','contact_trace/analysis_stage01/report.json',
        'component_audit/attitude_hold_protocol.json','component_audit/attitude_hold_prephysics_horizon_amendment.json',
        'component_audit/eight_attitude_hold103_stage01/diagnostic_metadata.json',
        'component_audit/eight_attitude_hold103_stage01/full_precision_prefix_receipt.json',
        'contact_trace/analysis_stage01/aligned_contacts.png','contact_trace/native97p5_stage01/parity_receipt.json',
        'contact_trace/native97p5_stage01/capture_receipt.json','contact_trace/native97p5_stage01/diagnostic_metadata.json']
    for name in singles:
        p=BASE/name
        if p.is_file():selected.add(p)
    for arm in ('native24','one_pulse','cluster_cap5'):
        for name in ('diagnostic_metadata.json','native_full_precision_verification.json'):
            p=BASE/'component_audit/causal_branches_stage01'/arm/name
            if p.is_file():selected.add(p)
    for p in sorted(selected):
        target=DEST/p.relative_to(BASE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
    tools=[p for p in files if p.suffix=='.py' and not any(x in p.parts for x in ('source','source_snapshot','development','child','known_model')) and len(p.relative_to(BASE).parts)<=3]
    for p in tools:
        target=DEST/'tools'/p.relative_to(BASE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
    write(DEST/'raw_bundle_receipt.json',dict(iteration='I-77',path=raw['path'],bytes=raw['bytes'],sha256=raw['sha256'],
        verified_members=raw['verified_members'],inventory='raw_inventory.json',
        prerequisite='I76 original input/source protocol and 149-file output seal. Original full I76 raw bundle is separately retained; it is not duplicated here.',
        scope='New I77 arrays/source/metadata, including preserved failed diagnostic-analysis/setup stages; generated workspace asset exclusion is documented.'))
    write(DEST/'raw_inventory.json',dict(files=raw['files'],excluded_workspace_assets=len(excluded)))
    for relative,expected in original.items():
        if bind(ROOT/relative)!=expected:raise AssertionError('Input changed during packaging: '+relative)
    copied={str(p.relative_to(DEST)):bind(p) for p in DEST.rglob('*') if p.is_file()}
    # Verify every direct copy separately from archive member checks.
    for p in selected:
        if bind(DEST/p.relative_to(BASE))!=bind(p):raise AssertionError('Compact copy differs')
    write(DEST/'archive_verification.json',dict(iteration='I-77',passed=True,
        verified_raw_members=raw['verified_members'],verified_source_metadata_members=source['verified_members'],
        direct_copies_verified=len(selected),source_and_metadata_sha256=source['sha256'],
        selected_inputs_unchanged=True,no_new_physics=True,no_control_or_metric_changes=True,
        source='evidence/tools/review/curate.py',
        scope='Archive bytes and every selected direct copy verified; no automatic live install or claim of isolated baseline re-evaluation.'))
    write(DEST/'manifest.json',dict(iteration='I-77',files={str(p.relative_to(DEST)):bind(p) for p in sorted(DEST.rglob('*')) if p.is_file()}))
    print(json.dumps(dict(passed=True,raw_bytes=raw['bytes'],raw_members=raw['verified_members'],
                         compact_files=len(copied)+1,compact_dir=str(DEST))),flush=True)

if __name__=='__main__':main()
