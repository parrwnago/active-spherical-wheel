"""Preserve compact I-65 evidence, with old tracked inputs as explicit dependencies."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def publish(source,destination):
    omitted=[];copied=[]
    for path in sorted(source.rglob('*')):
        if not path.is_file() or '__pycache__' in path.parts:continue
        relative=path.relative_to(source)
        if ('method/stages' in str(relative)
            or any(str(relative).startswith('review/'+name+'/') for name in
                   ('replay_producer_stage01','replay_numpy121_stage01'))):
            omitted.append(dict(path=str(relative),sha256=sha(path),bytes=path.stat().st_size,
                                reason='Duplicate replay outputs or method-only snapshots; canonical arrays, original/final source and comparison logs retained.'))
            continue
        target=destination/relative;target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():
            if sha(target)!=sha(path):raise ValueError('Refusing to replace archived evidence: '+str(target))
        else:shutil.copy2(path,target)
        copied.append(dict(path=str(relative),sha256=sha(path),bytes=path.stat().st_size))
    seal=json.loads((source/'protocol/input_source_seal.json').read_text())
    manifest=dict(iteration='I-65',published_utc=datetime.now(timezone.utc).isoformat(),
                  files=copied,local_omissions=omitted,
                  previous_tracked_package='docs/iterations/2026-11-27/contact-load-history/evidence',
                  dependencies=[x for x in seal['files'] if x['root']=='previous'],
                  qualification='No original simulation duplicate, credential, new physics run or controller modification. Numeric cached-map replay uses the tracked I64 package.',
                  new_physics_steps=0)
    (destination/'portable_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(copied_files=len(copied),bytes=sum(x['bytes'] for x in copied),local_omissions=len(omitted))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--destination',type=Path,required=True)
    a=p.parse_args();publish(a.source,a.destination)
