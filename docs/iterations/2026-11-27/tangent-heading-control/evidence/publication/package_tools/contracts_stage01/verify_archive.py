"""Post-pack byte audit and isolated restore of the actual I75 record.

No control/physics/inference/metric scoring is invoked. Full tar members and
plot-only subset bytes are checked, then archived restoration runs in a new
temporary repository fixture with an unrelated sentinel file.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import sys
import tarfile
import tempfile

import numpy as np


def sha(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):result.update(block)
    return result.hexdigest()


def record(path):return dict(bytes=path.stat().st_size,sha256=sha(path))


def binding(value):
    return dict(dtype=value.dtype.str,shape=list(value.shape),
        sha256=hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest())


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as stream:json.dump(value,stream,indent=2,sort_keys=True);stream.write('\n')


def run(evidence,bundle,repo,output):
    evidence=Path(evidence).resolve();bundle=Path(bundle).resolve();repo=Path(repo).resolve();output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    manifest_path=evidence/'compact_record_manifest.json'
    save(output/'registered_before_qa.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),manifest_sha256=sha(manifest_path),bundle=record(bundle),
        scope='Read-only content checks and isolated exact-byte restoration only; no metric/inference/physics changes.'))
    manifest=json.loads(manifest_path.read_text())
    expected=set(manifest['files'])|{'compact_record_manifest.json'}
    actual={str(path.relative_to(evidence)) for path in evidence.rglob('*') if path.is_file()}
    if actual!=expected:raise AssertionError('Compact manifest field-set mismatch')
    for relative,value in manifest['files'].items():
        if record(evidence/relative)!=value:raise AssertionError('Compact artifact byte mismatch: '+relative)
    originals=json.loads((evidence/'publication/package_tools/registered_before_packaging.json').read_text())
    for relative,value in originals['files'].items():
        if record(repo/relative)!=value:raise AssertionError('Original file changed since packaging: '+relative)
    inventory=json.loads((evidence/manifest['bundle_inventory']).read_text())
    if record(bundle)!={key:inventory[key] for key in ('bytes','sha256')}:raise AssertionError('Raw bundle binding mismatch')
    items={item['archive_member']:item for item in inventory['files']}
    with tarfile.open(bundle,'r:gz') as archive:
        members=archive.getmembers()
        if len(members)!=len(items) or {m.name for m in members}!=set(items):raise AssertionError('Tar member layout mismatch')
        for member in members:
            item=items[member.name]
            if not member.isfile() or member.size!=item['bytes']:raise AssertionError('Tar member type/size mismatch')
            digest=hashlib.sha256()
            with archive.extractfile(member) as stream:
                for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
            if digest.hexdigest()!=item['sha256']:raise AssertionError('Tar member byte mismatch: '+member.name)
            if record(repo/item['origin'])!={key:item[key] for key in ('bytes','sha256')}:raise AssertionError('Original raw source changed')
    plotmeta=json.loads((evidence/'publication/plot_samples_50ms/metadata.json').read_text())
    plot_checks={}
    for case,value in plotmeta['cases'].items():
        if not value['available']:plot_checks[case]=dict(available=False);continue
        with np.load(evidence/value['path'],allow_pickle=False) as source:
            arrays={name:source[name].copy() for name in source.files}
        if {key:binding(array) for key,array in arrays.items()}!=value['arrays']:raise AssertionError('Plot array binding mismatch')
        index=arrays['source_frame_index']
        if index[-1]!=value['full_frames']-1:raise AssertionError('Real terminal state missing from plot sample')
        with np.load(repo/value['source_signal'],allow_pickle=False) as signal,np.load(repo/value['source_precision'],allow_pickle=False) as precision:
            for name,array in arrays.items():
                if name=='source_frame_index':continue
                original=precision[name] if name.startswith(('exact_','heading__')) else signal[name]
                if binding(array)!=binding(original[index]):raise AssertionError('Plot-only subset changed source bytes')
        plot_checks[case]=dict(source_frames=value['full_frames'],plot_frames=len(index),all_subset_arrays_byte_exact=True,terminal_retained=True)
    temporary=Path(tempfile.mkdtemp(prefix='i74_actual_restore_',dir='/tmp'))
    checkout=temporary/'checkout';checkout.mkdir();sentinel=checkout/'UNRELATED_SENTINEL.txt';sentinel.write_bytes(b'preserve unrelated files\n')
    command=[sys.executable,'-I',str(evidence/'publication/package_tools/restore_record.py'),
        '--evidence',str(evidence),'--repo-root',str(checkout),'--raw-bundle',str(bundle),'--install-gain-cache']
    env=os.environ.copy()
    for key in ('PYTHONPATH','LD_LIBRARY_PATH','LD_PRELOAD'):env.pop(key,None)
    with (output/'restore.log').open('x') as stream:
        execution=subprocess.run(command,cwd=temporary,env=env,stdout=stream,stderr=subprocess.STDOUT)
    if execution.returncode:raise RuntimeError('Actual archive restoration failed; temporary fixture and log preserved')
    if sentinel.read_bytes()!=b'preserve unrelated files\n':raise AssertionError('Unrelated fixture file changed')
    restored=checkout/'results/tangent_heading_control_20261127'
    for relative,value in manifest['files'].items():
        if record(restored/relative)!=value:raise AssertionError('Restored compact file differs')
    raw_count=0
    for item in inventory['files']:
        if item['restore_kind']=='result_raw':
            if record(checkout/item['origin'])!={key:item[key] for key in ('bytes','sha256')}:raise AssertionError('Restored original raw file differs')
            raw_count+=1
    if (checkout/'src').exists() or (checkout/'build').exists():raise AssertionError('Runtime/live source was installed')
    gain_relative=json.loads((evidence/'protocol/protocol.json').read_text())['gain_cache']['path']
    gain_source=evidence/manifest['frozen_gain_cache_relative']
    if record(checkout/gain_relative)!=record(gain_source):raise AssertionError('Frozen gain-cache bytes changed')
    report=dict(status='PASS',compact_manifest_sha256=sha(manifest_path),compact_files=len(manifest['files']),
        compact_bound_bytes=sum(item['bytes'] for item in manifest['files'].values()),
        compact_all_files_and_field_set_exact=True,original_packaging_inputs_unchanged=True,
        raw_bundle=dict(path=str(bundle),**record(bundle),members=len(items),all_member_bytes_exact=True),
        plot_only_subsets=plot_checks,actual_restore=dict(command=command,returncode=execution.returncode,
            temporary_repository=str(checkout),compact_files_exact=True,raw_files_restored=raw_count,all_raw_files_byte_exact=True,
            frozen_gain_cache_exact=True,unrelated_sentinel_unchanged=True,no_live_sources_or_binaries_installed=True),
        no_new_metric_scoring=True,no_new_inference=True,physics_steps=0)
    save(output/'receipt.json',report)
    print(json.dumps(dict(status=report['status'],compact_files=report['compact_files'],bundle_bytes=report['raw_bundle']['bytes'],
        tar_members=len(items),raw_restored=raw_count,temp_repo=str(checkout)),sort_keys=True))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence','bundle','repo-root','output'):parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args();run(args.evidence,args.bundle,args.repo_root,args.output)
