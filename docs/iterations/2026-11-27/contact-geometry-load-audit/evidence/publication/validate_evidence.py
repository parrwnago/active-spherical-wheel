"""I66 archive/hash/link/receipt validation, no new scientific models."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,re,numpy as np


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())


def validate(package,repo,record,output):
    manifest=read(package/'portable_manifest.json');checks={}
    checks['all_archived_bytes_match']=all(sha(package/x['path'])==x['sha256'] for x in manifest['files'])
    roots=dict(experiment=package,pose=package/'features/stage01',context=package/'inputs/context_stage01',
               i61=repo/'docs/iterations/2026-11-27/contact-load-consistency/evidence',
               i64=repo/'docs/iterations/2026-11-27/contact-load-history/evidence',
               i65=repo/'docs/iterations/2026-11-27/contact-covariance-ranking/evidence')
    seal=read(package/'protocol/analysis_input_seal.json')
    checks['analysis_source_and24inputs_unchanged']=len(seal['files'])==24 and all(sha(roots[x['root']]/x['path'])==x['sha256'] for x in seal['files'])
    bindings=read(package/'analysis/stage01/array_bindings.json');count=0
    for name,fields in bindings.items():
        with np.load(package/'analysis/stage01'/name,allow_pickle=False) as z:
            if set(z.files)!=set(fields):raise ValueError('Canonical array inventory changed')
            for key,expected in fields.items():
                v=z[key];count+=1
                if v.dtype.str!=expected['dtype'] or list(v.shape)!=expected['shape'] or hashlib.sha256(v.tobytes()).hexdigest()!=expected['sha256']:raise ValueError('Canonical array binding differs')
    checks['177canonical_arrays_exact']=count==177
    pages={}
    for name in ('geometry_load_comparison.pdf','event_windows.pdf'):
        pages[name]=len(re.findall(rb'/Type\s*/Page\b',(package/'publication/plots_stage03'/name).read_bytes()))
    checks['pdf_pages_1_and8']=pages=={'geometry_load_comparison.pdf':1,'event_windows.pdf':8}
    links=[]
    for target in re.findall(r'\]\(([^)]+)\)',record.read_text()):
        if '://' in target:continue
        p=(record.parent/target.split('#')[0]).resolve();links.append(p)
        if not p.exists():raise ValueError('Broken record link: '+str(p))
    checks['all_record_links_exist']=bool(links)
    science=read(package/'analysis/stage01/science.json')
    checks['reported_counts_and_failed_criterion_match']=(science['cohorts']['focus']['denominator']==41 and science['counterfactual']['focus']['replacement_correct']==27 and science['counterfactual']['useful']['replacement_correct']==276 and science['counterfactual']['harmful']['replacement_correct']==4 and not science['geometry_question']['qualified_geometric_improvement'])
    checks['independent_loads_pass']=read(package/'review/independent_loads_stage01/output_stage01/report.json')['all_checks_passed']
    checks['independent_structural_floor_pass']=read(package/'review/independent_loads_stage01/floor_output_stage01/report.json')['passed']
    for version in ('producer','numpy121'):
        r=read(package/f'review/replay/{version}_comparison_stage01/comparison.json')
        checks[version+'_replay_pass']=r['all_checks_pass'] and r['array_count']==177
    result=dict(iteration='I-66',utc=datetime.now(timezone.utc).isoformat(),all_checks_pass=all(checks.values()),checks=checks,
                array_count=count,pdf_page_counts=pages,scope='Publication integrity and existing receipts; no refit, new simulation or extra controller suite.')
    output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():raise ValueError('Preserve previous QA stage')
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['all_checks_pass']:raise AssertionError('Publication QA failed')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('package','repo','record','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();validate(a.package,a.repo,a.record,a.output)
