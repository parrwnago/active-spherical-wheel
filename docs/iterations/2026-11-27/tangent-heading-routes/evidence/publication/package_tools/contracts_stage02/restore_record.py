"""Verify/restore a compact I76 record without overwriting unrelated files.

Only the dedicated result subtree is restored. Optional full raw bundle data
restores original result files, not runtime libraries. External native sources,
assets and binaries are verification-only. No simulator/controller import.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import tarfile


RESULT_RELATIVE=Path('results/tangent_heading_routes_20261127')


def sha(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):result.update(block)
    return result.hexdigest()


def safe_relative(name):
    path=Path(name)
    if path.is_absolute() or '..' in path.parts:raise ValueError('Unsafe archive path: '+str(name))
    return path


def destination(root,relative):
    """Reject existing symlinks so no result write can escape its owned tree."""
    root=Path(root)
    relative=safe_relative(relative)
    current=root
    for part in relative.parts:
        current=current/part
        if current.is_symlink():raise ValueError('Refusing symlink restore destination: '+str(current))
    return current


def verify_file(path,expected):
    if not path.is_file() or path.stat().st_size!=expected['bytes'] or sha(path)!=expected['sha256']:
        raise ValueError('File binding mismatch: '+str(path))


def verify_evidence(evidence):
    manifest=json.loads((evidence/'compact_record_manifest.json').read_text())
    if manifest['result_relative']!=str(RESULT_RELATIVE):raise ValueError('Unexpected restore root')
    for relative,expected in manifest['files'].items():verify_file(evidence/safe_relative(relative),expected)
    return manifest


def no_conflict(path,expected):
    if path.exists():verify_file(path,expected)


def restore(evidence,repo,bundle=None,install_cache=False,check_runtime=False,verify_only=False):
    evidence=Path(evidence).resolve();repo=Path(repo).resolve();target=repo/RESULT_RELATIVE
    manifest=verify_evidence(evidence)
    # Complete conflict preflight before any write; identical existing files are retained.
    target=destination(repo,RESULT_RELATIVE)
    for relative,expected in manifest['files'].items():no_conflict(destination(target,relative),expected)
    no_conflict(destination(target,'compact_record_manifest.json'),dict(bytes=(evidence/'compact_record_manifest.json').stat().st_size,
        sha256=sha(evidence/'compact_record_manifest.json')))
    dependency=json.loads((evidence/'publication/runtime_dependency_bindings.json').read_text())
    cache=dependency['gain_cache'];cache_path=destination(repo,cache['path'])
    cache_source=evidence/safe_relative(manifest['frozen_gain_cache_relative'])
    if install_cache:
        if cache_path!=repo/'generated/offroad/flat/control_schedule.npz':raise ValueError('Unexpected gain-cache destination')
        no_conflict(cache_path,dict(bytes=cache_source.stat().st_size,sha256=cache['sha256']))
    raw=[];binary_count=0
    if bundle is not None:
        bundle=Path(bundle).resolve()
        inventory=json.loads((evidence/safe_relative(manifest['bundle_inventory'])).read_text())
        verify_file(bundle,inventory)
        with tarfile.open(bundle,'r:gz') as archive:
            members={member.name:member for member in archive.getmembers()}
            expected_members={item['archive_member'] for item in inventory['files']}
            if set(members)!=expected_members:raise ValueError('Bundle member set differs')
            for item in inventory['files']:
                member=members[item['archive_member']]
                if not member.isfile() or member.size!=item['bytes']:raise ValueError('Only bound regular bundle files are accepted')
                origin=safe_relative(item['origin'])
                if item['restore_kind']=='result_raw':
                    if not origin.is_relative_to(RESULT_RELATIVE):raise ValueError('Raw destination outside dedicated result root')
                    raw_destination=destination(repo,origin);no_conflict(raw_destination,item);raw.append((item,raw_destination))
                elif item['restore_kind']=='runtime_binary_verify_only':binary_count+=1
                else:raise ValueError('Unknown bundle restore policy')
    if not verify_only:
        for relative,expected in manifest['files'].items():
            out=destination(target,relative)
            if not out.exists():out.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(evidence/relative,out)
        if not (target/'compact_record_manifest.json').exists():
            shutil.copy2(evidence/'compact_record_manifest.json',target/'compact_record_manifest.json')
        if install_cache and not cache_path.exists():
            cache_path.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(cache_source,cache_path)
        if raw:
            with tarfile.open(bundle,'r:gz') as archive:
                for item,out in raw:
                    if not out.exists():
                        out.parent.mkdir(parents=True,exist_ok=True)
                        with archive.extractfile(item['archive_member']) as source,out.open('xb') as output:shutil.copyfileobj(source,output)
                    verify_file(out,item)
        for relative,expected in manifest['files'].items():verify_file(target/relative,expected)
    runtime=[]
    if check_runtime:
        for group in ('sources','inputs'):
            for name,digest in dependency[group].items():
                path=repo/safe_relative(name)
                if not path.is_file() or sha(path)!=digest:runtime.append(name)
        for item in (dependency['bridge_binary'],dependency['exact_physics_binary'],cache):
            path=repo/safe_relative(item['path'])
            if not path.is_file() or sha(path)!=item['sha256']:runtime.append(item['path'])
        if runtime:raise ValueError('Existing runtime dependencies missing/different (not overwritten): '+', '.join(sorted(set(runtime))))
    result=dict(compact_files_verified=len(manifest['files']),restored=not verify_only,
        raw_files_restored=len(raw) if not verify_only else 0,
        runtime_binary_members_not_installed=binary_count,
        destination=str(target),runtime_verified=bool(check_runtime),
        full_raw_metric_files_present=all((target/'campaign/runs'/f'{route}_{phase}_{policy}'/name).is_file()
            for route in ('straight','arc','eight') for phase in ('m100','p030','p090') for policy in ('N','T')
            for name in ('precision.npz','summary.json','provenance.json')),
        scope='Dedicated result subtree only; no live code/plugin overwrite. Plot samples are never substituted for full raw inputs.')
    print(json.dumps(result,sort_keys=True));return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True);parser.add_argument('--repo-root',type=Path,required=True)
    parser.add_argument('--raw-bundle',type=Path);parser.add_argument('--install-gain-cache',action='store_true')
    parser.add_argument('--check-runtime',action='store_true');parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args();restore(args.evidence,args.repo_root,args.raw_bundle,args.install_gain_cache,args.check_runtime,args.verify_only)
