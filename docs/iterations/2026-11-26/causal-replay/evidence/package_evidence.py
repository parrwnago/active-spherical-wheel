"""Copy compact I55 evidence; byte-identical archives, no native execution."""
from pathlib import Path
import datetime,hashlib,json,shutil
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[1]
OUT=ROOT/'docs/iterations/2026-11-26/causal-replay/evidence'
OUT.mkdir(parents=True,exist_ok=True)
files=['protocol.json','protocol_initial_registration.json','native_parity_gate.json','run_diagnostic.py','substep_trace.py','package_data.py','native_batch.log','native_substeps.log','zero_free_shellA.log','hold_attitude.log']
files += ['instrumentation_checks_stage02/report.json','instrumentation_checks_stage02/before_checks.json'] if (BASE/'instrumentation_checks_stage02/before_checks.json').exists() else ['instrumentation_checks_stage02/report.json']
for case in ('native_batch','native_substeps','zero_free_shellA','hold_attitude'):
    files += [f'campaign/{case}/i55_provenance.json',f'campaign/{case}/historical_precision_comparison.json',f'campaign/{case}/i55_artifact_manifest.json',f'campaign/{case}/campaign/runs/m100_R/execution_result.json']
    files += [str(p.relative_to(BASE)) for p in (BASE/'campaign'/case/'i55_models').rglob('*') if p.is_file()]
files += ['analysis/'+p for p in ['final_report.json','artifact_manifest.json','analyze_causal.py','budget_math.py','check_math.py','analyze_available.py','present_causal.py','present_causal_stage02.py','describe_release.py','seal_analysis.py','stage01/report.json','stage01/registered_before_reduction.json','stage01/paired_outcomes.csv','stage01/artifact_manifest.json','checks_stage01/failure.json','checks_stage01/failure.log','checks_stage02_corrected/report.json','checks_stage02_corrected/before_checks.json','checks_stage02_corrected/artifact_manifest.json','release_followup_stage01/postprocess_failure.json','release_followup_stage01/postprocess_stdout_failure.log','release_followup_stage01/describe_release.py','release_followup_stage01/declared_before_readout.json','release_followup_stage01/report.json','release_followup_stage02_stdout_corrected/report.json','release_followup_stage02_stdout_corrected/declared_before_readout.json','release_followup_stage02_stdout_corrected/artifact_manifest.json','seal_stage01/report.json']]
files += [str(p.relative_to(BASE)) for p in (BASE/'analysis/presentation_stage02_three_targets').iterdir() if p.is_file()]
files += ['review/'+p for p in ['budget_source_review_stage01/report.json','source_timing_stage01/report.json','independent_numeric_stage01/self_review.json','independent_numeric_stage02/report.json'] if (BASE/'review'/p).is_file()]
# Preserve executed review sources/registrations and correction notes, not downloaded upstream source trees.
for sub in ['review/independent_numeric_stage01','review/independent_numeric_stage02','instrumentation_checks_stage01','instrumentation_checks_stage02']:
    for p in (BASE/sub).glob('*'):
        if p.is_file() and p.suffix in ('.py','.json','.log') and str(p.relative_to(BASE)) not in files:files.append(str(p.relative_to(BASE)))
entries=[]
for rel in dict.fromkeys(files):
    src=BASE/rel;dst=OUT/rel
    assert src.is_file(),str(src)
    assert not dst.exists(),str(dst)
    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
    entries.append({'path':rel,'origin':str(src.relative_to(ROOT)),'bytes':dst.stat().st_size,'sha256':hashlib.sha256(dst.read_bytes()).hexdigest()})
m={'iteration':'I-55','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Selected source/protocol/reports/plots/QA copies. Originalstage manifests inventory local largerdatasets too; copyingmanifest doesnot imply fullstagecopy. Fullprefixdatasets/workspaces remainlocal. Separate data/manifest.json binds portable raw1ms andexact5ms subsets.','files':entries,'bytes':sum(a['bytes'] for a in entries)}
(OUT/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
print(json.dumps({'files':len(entries),'bytes':m['bytes']},indent=2))
