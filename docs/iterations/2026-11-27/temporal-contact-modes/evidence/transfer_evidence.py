"""Hash-planned I70 evidence export. Writes only this evidence subtree.

No experiment is executed. Source files are copied unchanged. Bulky duplicate
arrays and interpreter caches remain local with explicit omission bindings.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def decision(relative):
    parts = relative.parts
    text = relative.as_posix()
    if '__pycache__' in parts or relative.suffix == '.pyc':
        return None, 'Transient interpreter bytecode cache; source and verification receipts are retained.'
    if text.startswith('inputs/stage01/') and relative.name == 'features.npz':
        return None, 'Bulky derived local feature cache; all arrays are reconstructible and bound by portable local_feature_array_bindings.json and exact replay QA.'
    if text.startswith('inputs/stage01/') and relative.name == 'primitives.npz':
        case = parts[2]
        return None, f'Duplicate primitive payload; identical file retained at portable_inputs/{case}/primitives.npz.'
    if text.startswith('inputs/portable_stage01/') and relative.name == 'primitives.npz':
        case = parts[2]
        return None, f'Legacy duplicate primitive payload; unchanged values retained at portable_inputs/{case}/primitives.npz. Legacy metadata/source and rank-binding correction receipts are preserved.'
    if text.startswith('publication/replay_producer_stage01/') and relative.suffix == '.npz':
        return None, 'Recomputed inference duplicate; canonical inference arrays and exact per-array producer replay QA are retained.'
    relocations = (('inputs/portable_stage02/', 'portable_inputs/'),
                   ('publication/portable_tools/', 'portable_tools/'),
                   ('publication/replay_qa/', 'replay_qa/'),
                   ('publication/plot_tools/', 'plot_tools/'),
                   ('publication/plots_stage01/', 'plots_stage01/'),
                   ('publication/plots_stage02/', 'plots_stage02/'))
    for before, after in relocations:
        if text.startswith(before):
            return after+text[len(before):], 'Byte-preserved export; directory relocated for the standalone evidence layout.'
    return text, 'Byte-preserved original evidence, including immutable source, failed checks and successful stages.'


def inventory(source):
    entries = []
    for path in sorted(source.rglob('*')):
        if not path.is_file(): continue
        if not path.resolve().is_relative_to(source.resolve()):
            raise ValueError('Source file resolves outside the allowed I70 base: '+str(path))
        relative = path.relative_to(source)
        if 'credential' in relative.as_posix().lower():
            raise ValueError('Credential-related path excluded from read scope; ask the parent to narrow the inventory.')
        destination, reason = decision(relative)
        entries.append(dict(source=relative.as_posix(), destination=destination,
                            action='copy' if destination else 'omit', reason=reason,
                            bytes=path.stat().st_size, sha256=sha(path)))
    return entries


def manifest_entries(value):
    raw = value.get('files', value)
    if isinstance(raw, list):
        return [(item['path'], item['sha256'], item.get('bytes')) for item in raw
                if isinstance(item, dict) and 'path' in item and 'sha256' in item]
    if isinstance(raw, dict):
        result = []
        for name, info in raw.items():
            if isinstance(info, dict) and 'sha256' in info:
                result.append((name, info['sha256'], info.get('bytes')))
            elif isinstance(info, str) and len(info) == 64:
                result.append((name, info, None))
        return result
    return []


def audit_original_bindings(source):
    checks, preserved_external = [], []
    for manifest in sorted(source.rglob('*manifest.json')):
        if '__pycache__' in manifest.parts: continue
        value = json.loads(manifest.read_text())
        for relative, expected, expected_size in manifest_entries(value):
            candidate = manifest.parent/relative
            prefix = 'results/temporal_contact_modes_20261127/'
            if relative.startswith(prefix): candidate = source/relative[len(prefix):]
            resolution = 'relative original artifact'
            # Portable snapshots intentionally retain the original local-cache
            # manifest even though that large convenience file is omitted.
            if not candidate.exists() and relative == 'features.npz' and 'portable_stage' in manifest.as_posix():
                candidate = source/'inputs/stage01'/manifest.parent.name/'features.npz'
                resolution = 'original local feature cache named by preserved portable provenance'
            if not candidate.resolve().is_relative_to(source.resolve()):
                preserved_external.append(dict(manifest=str(manifest.relative_to(source)), reference=relative,
                                               expected_sha256=expected, status='outside I70 scope; original reference preserved, not opened'))
                continue
            if not candidate.is_file():
                raise ValueError('Missing original artifact binding: '+str(manifest)+' -> '+relative)
            actual = sha(candidate); size = candidate.stat().st_size
            if actual != expected or (expected_size is not None and size != expected_size):
                raise ValueError('Original artifact binding mismatch: '+str(candidate))
            checks.append(dict(manifest=str(manifest.relative_to(source)),
                artifact=str(candidate.relative_to(source)), sha256=actual, bytes=size,
                status='PASS', resolution=resolution))
    # The authoritative inference seal is checked separately from historical
    # receipts whose live paths intentionally moved on after failed versions.
    # External live src/tests are not opened; their exact pre-execution copies
    # inside I70 are the archival source authority.
    seal_path = source/'protocol/inference_source_input_seal.json'
    seal = json.loads(seal_path.read_text())
    for group in ('sources', 'input_files'):
        for relative, expected in seal[group].items():
            prefix = 'results/temporal_contact_modes_20261127/'
            if relative.startswith(prefix):
                candidate = source/relative[len(prefix):]
                resolution = 'authoritative final in-scope inference seal'
            elif relative.startswith(('src/', 'tests/')):
                candidate = source/'pre_execution_source'/relative
                resolution = 'byte-preserved pre-execution copy of external live source; external file not opened'
            else:
                preserved_external.append(dict(manifest='protocol/inference_source_input_seal.json',
                    reference=relative, expected_sha256=expected,
                    status='outside I70 scope; original reference preserved, not opened'))
                continue
            if not candidate.resolve().is_relative_to(source.resolve()) or not candidate.is_file():
                raise ValueError('Missing final sealed source/input in I70: '+relative)
            if sha(candidate) != expected:
                raise ValueError('Authoritative inference seal mismatch: '+relative)
            checks.append(dict(manifest='protocol/inference_source_input_seal.json',
                artifact=str(candidate.relative_to(source)), sha256=expected,
                bytes=candidate.stat().st_size, status='PASS', resolution=resolution))
    return dict(all_in_scope_original_artifact_bindings_pass=True, checked=len(checks),
                checks=checks, external_references_not_opened=preserved_external)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--source', type=Path, required=True)
    args = ap.parse_args(); source = args.source.resolve(); target = Path(__file__).resolve().parent
    if source.name != 'temporal_contact_modes_20261127' or source.parent.name != 'results':
        raise ValueError('Only the requested I70 source base may be read')
    for expected in ('publication/plots_stage02/render_receipt.json', 'review/integration_stage01/receipt.json',
                     'evaluation/posthoc_temporal_errors/stage01/artifact_manifest.json'):
        if not (source/expected).is_file(): raise ValueError('Final source stage is not ready: '+expected)
    for name in ('export_plan.json', 'export_manifest.json', 'transfer_receipt.json'):
        if (target/name).exists(): raise FileExistsError(target/name)
    entries = inventory(source)
    destinations = [row['destination'] for row in entries if row['action'] == 'copy']
    if len(destinations) != len(set(destinations)): raise ValueError('Export destination collision')
    generated = {'transfer_evidence.py', 'export_plan.json', 'export_manifest.json',
                 'transfer_receipt.json', 'artifact_binding_audit.json', 'runtime_provenance.json'}
    if generated.intersection(destinations): raise ValueError('Source collides with generated export records')
    plan = dict(status='SEALED before any source file copy',
        timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        source_base=str(source), scope='Only files within the I70 results base; no credentials or outside files opened.',
        entries=entries, included=sum(x['action']=='copy' for x in entries),
        omitted=sum(x['action']=='omit' for x in entries),
        included_bytes=sum(x['bytes'] for x in entries if x['action']=='copy'),
        omitted_bytes=sum(x['bytes'] for x in entries if x['action']=='omit'))
    write(target/'export_plan.json', plan)
    original_audit = audit_original_bindings(source)
    write(target/'artifact_binding_audit.json', original_audit)
    original_runtime = json.loads((source/'publication/replay_qa/m100_B/registered_before_replay.json').read_text())
    runtime = {key: original_runtime[key] for key in ('python', 'executable', 'numpy', 'scipy', 'imports')}
    runtime.update(status='Summary copied from the original pre-replay registration, not a new environment measurement.',
                   source_registration='replay_qa/m100_B/registered_before_replay.json',
                   source_registration_sha256=sha(source/'publication/replay_qa/m100_B/registered_before_replay.json'))
    write(target/'runtime_provenance.json', runtime)
    copied = []
    for row in entries:
        if row['action'] != 'copy': continue
        before, after = source/row['source'], target/row['destination']
        if sha(before) != row['sha256']: raise ValueError('Source changed after export plan: '+row['source'])
        if after.exists(): raise FileExistsError(after)
        after.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(before, after)
        if sha(after) != row['sha256'] or after.stat().st_size != row['bytes']:
            raise ValueError('Destination copy verification failed: '+row['destination'])
        copied.append(dict(path=row['destination'], source=row['source'], bytes=row['bytes'], sha256=row['sha256']))
    if inventory(source) != entries: raise ValueError('Source inventory changed during export')
    support = {name: dict(bytes=(target/name).stat().st_size, sha256=sha(target/name))
               for name in ('transfer_evidence.py', 'export_plan.json', 'artifact_binding_audit.json', 'runtime_provenance.json')}
    manifest = dict(files=copied, generated_support_files=support,
        omitted_files_record='export_plan.json entries with action=omit; every path/size/hash/reason retained.',
        self_reference_policy='This manifest does not hash itself or its later transfer receipt; the receipt binds this manifest.')
    write(target/'export_manifest.json', manifest)
    receipt = dict(status='PASS: all planned copies and original in-scope artifact bindings verified',
        timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        source_inventory_unchanged=True, copied_files=len(copied), copied_bytes=sum(x['bytes'] for x in copied),
        omitted_files=plan['omitted'], omitted_bytes=plan['omitted_bytes'],
        original_artifact_bindings_checked=original_audit['checked'],
        export_plan_sha256=sha(target/'export_plan.json'), export_manifest_sha256=sha(target/'export_manifest.json'),
        generated_support_files=support, no_experiment_or_test_rerun=True,
        no_frozen_source_modified=True, no_self_recursive_export_plan=True)
    write(target/'transfer_receipt.json', receipt)
    print(json.dumps(receipt))


if __name__ == '__main__': main()
