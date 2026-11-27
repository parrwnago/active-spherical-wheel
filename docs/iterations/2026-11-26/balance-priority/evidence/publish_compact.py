"""Archive I58 exact compact evidence; leave full rollouts/workspaces local."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT=Path.cwd();SOURCE=ROOT/'results/balance_priority_20261126'
DEST=ROOT/'docs/iterations/2026-11-26/balance-priority/evidence'
EXCLUDE={'__pycache__','.pytest_cache','.git'}


def copy(path,relative):
    target=DEST/relative;target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists() and target.read_bytes()!=path.read_bytes():
        raise RuntimeError('Refuse replacement of changed evidence: '+str(target))
    shutil.copy2(path,target)


def tree(relative):
    folder=SOURCE/relative
    if not folder.exists():return
    for path in sorted(folder.rglob('*')):
        if path.is_file() and not EXCLUDE.intersection(path.relative_to(SOURCE).parts):
            copy(path,path.relative_to(SOURCE))


for folder in ('pre_change','control_checks_stage01','control_checks_stage02','control_checks_stage03',
               'campaign_tools','same_state_stage01','analysis','review','batch_initial','batch_conditional'):
    tree(Path(folder))
for name in ('protocol.json','register_protocol.py','run_batch.py','preflight.json',
             'check_same_state.py','same_state_execution.log','initial_execution.log',
             'conditional_execution.log','media_publication.json','publication_checks.json','publish_compact.py'):
    path=SOURCE/name
    if path.exists():copy(path,Path(name))
for name in ('README.md','render_comparison.py','presentation_plan.json',
             'render_broader.py','broader_presentation_plan.json','audit_saved_media.py',
             'prepared_before_outcomes.json','broader_prepared_before_outcomes.json',
             'lead_compact_media_record.tar.gz','lead_compact_media_record_manifest.json',
             'broader_compact_media_record.tar.gz','broader_compact_media_record_manifest.json',
             'lead_final_report.json','broader_final_report.json',
             'broader_qa_stage01/report.json',
             'compact_media_record.tar.gz','compact_media_record_manifest.json','qa_stage01/report.json',
             'conditional/README.md','conditional/render_comparison.py','conditional/presentation_plan.json',
             'conditional/compact_media_record.tar.gz','conditional/compact_media_record_manifest.json',
             'conditional/qa_stage01/report.json'):
    path=SOURCE/'video_tools'/name
    if path.exists():copy(path,Path('video_tools')/name)
inventory=[]
for folder in sorted((SOURCE/'campaign/runs').iterdir()):
    if not folder.is_dir():continue
    inventory.append(dict(case_id=folder.name,files={p.name:dict(bytes=p.stat().st_size,
        sha256=hashlib.sha256(p.read_bytes()).hexdigest())for p in sorted(folder.iterdir())if p.is_file()}))
    for name in ('summary.json','provenance_before.json','provenance.json','execution_result.json',
                 'artifact_manifest.json','historical_precision_comparison.json','failure.json','terrain.json'):
        if (folder/name).exists():copy(folder/name,Path('campaign/runs')/folder.name/name)
    if folder.name=='m100_P':tree(Path('campaign/runs')/folder.name/'source_snapshot')
(DEST/'case_inventory.json').write_text(json.dumps(dict(
    scope='Full originalprecision/rollout/governorJSON/workspaces staylocal, boundbyhash. Exactfullrate metric projections liveinanalysis; allobservedcase metadata andoneexecutedsource snapshotarecopied.',
    cases=inventory),indent=2)+'\n')
files=[]
for path in sorted(DEST.rglob('*')):
    if path.is_file() and path.name!='portable_manifest.json':
        files.append(dict(path=str(path.relative_to(DEST)),bytes=path.stat().st_size,
                          sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(DEST/'portable_manifest.json').write_text(json.dumps(dict(iteration='I-58',
    source_root=str(SOURCE.relative_to(ROOT)),observed_native_cases=len(inventory),files=files),indent=2)+'\n')
print(json.dumps(dict(files=len(files),bytes=sum(x['bytes']for x in files),native_cases=len(inventory))))
