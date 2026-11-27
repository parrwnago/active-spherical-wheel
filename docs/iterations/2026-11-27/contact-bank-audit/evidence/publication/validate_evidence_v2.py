"""I67 integrity/link/receipt checks, separate from scientific success."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,re,numpy as np


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())


def run(package,repo,record,output):
    m=read(package/'portable_manifest.json');checks={}
    checks['all_archive_hashes_match']=all(sha(package/x['path'])==x['sha256'] for x in m['files'])
    roots=dict(experiment=package,packets=package/'inputs/packets_stage01',banks=package/'features/bank_stage01')
    seal=read(package/'protocol/analysis_input_seal.json')
    checks['10_analysis_sources_inputs_unchanged']=len(seal['files'])==10 and all(sha(roots[x['root']]/x['path'])==x['sha256'] for x in seal['files'])
    count=0
    for name,fields in read(package/'analysis/stage01/array_bindings.json').items():
        with np.load(package/'analysis/stage01'/name,allow_pickle=False) as z:
            if set(z.files)!=set(fields):raise ValueError('Canonical array inventory differs')
            for key,expected in fields.items():
                v=z[key];count+=1
                if v.dtype.str!=expected['dtype'] or list(v.shape)!=expected['shape'] or hashlib.sha256(v.tobytes()).hexdigest()!=expected['sha256']:raise ValueError('Canonical array differs')
    checks['177_canonical_arrays_exact']=count==177
    pages={name:len(re.findall(rb'/Type\s*/Page\b',(package/'publication/plots_stage02'/name).read_bytes())) for name in ('bank_capacity_comparison.pdf','event_windows.pdf')}
    checks['pdf_page_counts_1_and8']=pages=={'bank_capacity_comparison.pdf':1,'event_windows.pdf':8}
    for doc in (record,repo/'docs/CHATGPT_CONTROL_CONTEXT.md'):
        for target in re.findall(r'\]\(([^)]+)\)',doc.read_text()):
            if '://' in target:continue
            if not (doc.parent/target.split('#')[0]).exists():raise ValueError('Broken link '+str(doc)+':'+target)
    checks['record_and_standalone_MD_links_exist']=True
    d=read(package/'publication/context_document_qa.json')
    checks['standalone_MD_matches_reviewed_revision']=sha(repo/d['path'])==d['sha256'] and d['words']>4000
    s=read(package/'analysis/stage01/science.json')
    checks['29of29_and_full_denominators_match']=s['primary_capacity']['supported']==29 and s['primary_capacity']['unsupported']==0 and s['cohorts']['focus']['denominator']==41 and s['cohorts']['useful']['denominator']==294 and s['cohorts']['harmful']['denominator']==6
    independent=read(package/'review/independent_stage01/corrected_scope_stage02/output/comparison.json')
    checks['independent_arithmetic_pass']=independent['all_checks_passed']
    for version in ('producer','numpy121'):
        r=read(package/f'review/replay/{version}_comparison_stage01/comparison.json')
        checks[version+'_replay_pass']=r['all_checks_pass'] and r['array_count']==177
    result=dict(iteration='I-67',utc=datetime.now(timezone.utc).isoformat(),checks=checks,all_checks_pass=all(checks.values()),array_count=count,pdf_pages=pages,
        scope='Publication/source/standalone-context integrity; no new estimator or controller qualification.')
    output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():raise ValueError('Preserve previous QA stage')
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['all_checks_pass']:raise AssertionError('Publication QA failed')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('package','repo','record','output'):p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();run(a.package,a.repo,a.record,a.output)
