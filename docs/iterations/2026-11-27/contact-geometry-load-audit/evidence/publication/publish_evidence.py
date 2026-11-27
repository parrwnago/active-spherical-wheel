"""Freeze compact I66 evidence without duplicate replay outputs or raw assets."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def run(source,destination):
    files=[];omitted=[]
    for p in sorted(source.rglob('*')):
        if not p.is_file() or '__pycache__' in p.parts:continue
        rel=p.relative_to(source)
        row=dict(path=str(rel),sha256=sha(p),bytes=p.stat().st_size)
        if any(str(rel).startswith('review/replay/'+x+'/') for x in ('producer_stage01','numpy121_stage01')):
            omitted.append({**row,'reason':'Duplicate computed outputs; canonical arrays, replay source binding, logs and comparisons retained.'});continue
        target=destination/rel;target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():
            if sha(target)!=row['sha256']:raise ValueError('Refuse to replace archived bytes: '+str(rel))
        else:shutil.copy2(p,target)
        files.append(row)
    manifest=dict(iteration='I-66',utc=datetime.now(timezone.utc).isoformat(),files=files,local_omissions=omitted,
        tracked_dependencies=['docs/iterations/2026-11-27/contact-load-consistency/evidence','docs/iterations/2026-11-27/contact-load-history/evidence','docs/iterations/2026-11-27/contact-covariance-ranking/evidence'],
        original_assets='Full original I58 logs/model assets stay local, bound by feature input seal. Portable replay uses included force-independent pose maps and endpoint context.',
        new_physics_steps=0,live_source_changes=False)
    (destination/'portable_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(archived_files=len(files),bytes=sum(x['bytes'] for x in files),omitted_duplicate_files=len(omitted))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--destination',type=Path,required=True)
    a=p.parse_args();run(a.source,a.destination)
