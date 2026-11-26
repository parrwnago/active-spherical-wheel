"""Post-process I55 source/artifact integrity seal; no outcome recalculation."""
from pathlib import Path
import json,hashlib,shutil
import analyze_causal as ac
H=Path(__file__).resolve().parent;F=H.parent;ROOT=F.parents[1]
def main():
    out=H/'seal_stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/Path(__file__).name)
    stages=['checks_stage02_corrected','early_stage01','stage01','presentation_stage01','presentation_stage02_three_targets','release_followup_stage01','release_followup_stage02_stdout_corrected']
    references=[]
    for name in stages:
        m=H/name/'artifact_manifest.json'
        for r in json.loads(m.read_text())['files']:
            p=m.parent/r['path'];assert p.stat().st_size==r['bytes']and ac.sha(p)==r['sha256'];references.append(str(p.relative_to(H)))
    primary=H/'stage01/report.json';registration=json.loads((H/'stage01/registered_before_reduction.json').read_text())
    for rel,h in registration['input_sha256'].items():assert ac.sha(ROOT/rel)==h,rel
    assert ac.sha(primary)=='4e3cd2e0da6777c9533a9cf0d2da97c5b606fec88af146e7730f88d2d0ae392b'
    assert ac.sha(H/'analyze_causal.py')=='49f3dab456c0715d63d78c608fbf68871beb87e8cae70037607b3765bc155348'
    assert ac.sha(H/'budget_math.py')=='8d80c1c40428d034f0de90dff9bd49d3de30f9989b22fd14a2abf0b88f5d7d19'
    qa=F/'review/independent_numeric_stage02/report.json';proof=F/'review/budget_source_review_stage01/report.json';timing=F/'review/source_timing_stage01/report.json'
    for p in [qa,proof,timing]:assert p.is_file()
    reports={str(p.relative_to(F)):ac.sha(p)for p in [primary,H/'presentation_stage02_three_targets/summary.json',H/'README.md',qa,proof,timing]}
    final={'status':'complete','scope':'Four completed32.500s diagnostic runs; original40s motion program retained. No adopted controller and no completed44s route claim.','canonical_report':'stage01/report.json','canonical_report_sha256':ac.sha(primary),'compact_summary':'presentation_stage02_three_targets/summary.json','readable_report':'README.md','technical_causal_prerequisites_pass':json.loads(primary.read_text())['gates']['technical_causal_prerequisites_pass'],'all_registered_inputs_unchanged':True,'canonical_report_unchanged':True,'verified_stage_file_references':len(references),'math_checks':'Nine unique artifact checks pass in corrected stage02; initial inclusive-boundary floating failure preserved, not an extra count.','independent_QA':'Separate53 saved-array assertions passed in corrected independent_numeric_stage02; earlier metadata-registration correction retained. This is not a combined repository test total.','preserved_failures':['checks_stage01 inclusive floating comparison fixture, corrected using2e-15Nms roundoff allowance without physical tolerance change','release_followup_stage01 console serialization failure after correct savedreport/manifest, corrected stdout only in freshstage02'],'presentation_revision':'stage02 adds three explicitly separated attitude targets; earlierstage01 figuresretained.','secondary_release_followup':'Post-primary descriptive readout motivated by the fixedplot; doesnotmodifyprimarywindow/endpoints/gates.','critical_limits':['First5ms non-Ainputs match; later branch effects include mediatedfeedback.','Holdingtargetreducesprimaryyaw butworsensabsolute tilt andsame-untreated-targeterror; release producesanewtransient.','Native/zeroA55ms yawbudget fails unchanged registered10% tolerance; no sign/clock/center selection applied.'],'bound_report_sha256':reports,'final_figures':['presentation_stage02_three_targets/shell_A_causal_response.png','presentation_stage02_three_targets/shell_A_causal_response.pdf','presentation_stage02_three_targets/attitude_causal_response.png','presentation_stage02_three_targets/attitude_causal_response.pdf']}
    ac.save(H/'final_report.json',final);ac.save(out/'report.json',{'passes':True,'final_report_sha256':ac.sha(H/'final_report.json'),'stage_file_references':references,'canonical_declared_input_count_verified':len(registration['input_sha256'])})
    ac.save(H/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(H)),'bytes':p.stat().st_size,'sha256':ac.sha(p)}for p in sorted(H.rglob('*'))if p.is_file()and'__pycache__'not in p.parts and p!=H/'artifact_manifest.json']})
    print(json.dumps({'final_report_sha256':ac.sha(H/'final_report.json'),'canonical_report_sha256':ac.sha(primary),'summary_sha256':ac.sha(H/'presentation_stage02_three_targets/summary.json'),'README_sha256':ac.sha(H/'README.md'),'manifest_sha256':ac.sha(H/'artifact_manifest.json')}))
if __name__=='__main__':main()
