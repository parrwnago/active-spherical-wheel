"""Read-only I-65 publication/source/binding checks, not scientific retesting."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,re
import numpy as np


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())


def validate(package,previous,record,output,plots):
    manifest=read(package/'portable_manifest.json');checks={}
    checks['archived_files_match_sources']=all(sha(package/x['path'])==x['sha256'] for x in manifest['files'])
    seal=read(package/'protocol/input_source_seal.json')
    checks['original_scientific_source_and_input_seal_exact']=all(
        sha((package if x['root']=='experiment' else previous)/x['path'])==x['sha256'] for x in seal['files'])
    bindings=read(package/'analysis/stage01/array_bindings.json');count=0
    for name,entries in bindings.items():
        with np.load(package/'analysis/stage01'/name,allow_pickle=False) as data:
            if set(data.files)!=set(entries):raise ValueError('Canonical array inventory changed')
            for key,expected in entries.items():
                actual=data[key];count+=1
                if actual.dtype.str!=expected['dtype'] or list(actual.shape)!=expected['shape'] or hashlib.sha256(actual.tobytes()).hexdigest()!=expected['sha256']:
                    raise ValueError('Canonical arrays changed: '+name+'/'+key)
    checks['canonical129arrays_exact']=count==129
    report=read(package/'analysis/stage01/science.json')
    s=report['shadows']['M1']['shared_covariance']
    checks['reported_failed_method_not_promoted']=(not report['primary_qualified'] and not report['discrimination']['M1']['promising'] and not s['all_gates_pass'])
    checks['headline_counts_match']=(s['metrics']['error_count']==163 and s['metrics']['useful_retained_count']==253 and s['metrics']['harmful_count']==4 and s['baseline']['A']['error_count']==124)
    all_paths=[]
    for match in re.finditer(r'\]\(([^)]+)\)',record.read_text()):
        target=match.group(1).split('#')[0]
        if '://' in target or not target:continue
        path=(record.parent/target).resolve();all_paths.append(path)
        if not path.exists():raise ValueError('Broken new record link: '+str(path))
    checks['record_links_exist']=bool(all_paths)
    pages={}
    for name in ('covariance_comparison.pdf','event_windows.pdf'):
        p=package/'publication'/plots/name
        pages[name]=len(re.findall(rb'/Type\s*/Page\b',p.read_bytes()))
    checks['pdf_page_counts_match']=pages=={'covariance_comparison.pdf':1,'event_windows.pdf':8}
    checks['overview_png_exists']=(package/'publication'/plots/'overview.png').is_file()
    independent=read(package/'review/independent_stage01/corrected_stage02/output/comparison.json')
    checks['independent_arithmetic_pass']=independent['all_checks_passed'] and not independent['primary_qualified']
    for version in ('producer','numpy121'):
        value=read(package/f'review/replay_{version}_comparison_stage01/comparison.json')
        checks[version+'_replay_pass']=value['all_checks_pass'] and value['array_count']==129
    value=dict(iteration='I-65',utc=datetime.now(timezone.utc).isoformat(),checks=checks,
               all_checks_pass=all(checks.values()),canonical_array_count=count,pdf_pages=pages,
               scope='Publication/source/evidence integrity only; no new fitting, physics or full controller suite.')
    output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():raise ValueError('Do not replace a QA stage')
    output.write_text(json.dumps(value,indent=2)+'\n')
    print(json.dumps(value,indent=2))
    if not value['all_checks_pass']:raise AssertionError('Publication checks failed')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package',type=Path,required=True);p.add_argument('--previous-package',type=Path,required=True)
    p.add_argument('--record',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plots',default='plots_stage02')
    a=p.parse_args();validate(a.package,a.previous_package,a.record,a.output,a.plots)
