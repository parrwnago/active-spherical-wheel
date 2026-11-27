"""Byte-preserved I71 evidence export with a pre-copy plan and omissions.

Reads only the requested I71 results base and writes only this evidence tree.
No tests, inference, evaluation or simulations are executed.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def omission(relative):
    if '__pycache__' in relative.parts or relative.suffix == '.pyc':
        return 'Transient interpreter cache; original source, contract results and provenance are retained.'
    if (relative.parts[:2] == ('publication', 'replay_qa') and relative.name == 'predictions.npz'):
        return 'Recomputed prediction duplicate; canonical inference/stage01 predictions and exact replay reports/bindings are retained.'
    return None


def inventory(source):
    rows = []
    for path in sorted(source.rglob('*')):
        if not path.is_file(): continue
        if not path.resolve().is_relative_to(source): raise ValueError('Source path escapes I71 base')
        rel = path.relative_to(source)
        if 'credential' in rel.as_posix().lower():
            raise ValueError('Credential-related path is outside the permitted read scope')
        reason = omission(rel)
        rows.append(dict(source=rel.as_posix(), destination=None if reason else rel.as_posix(),
            action='omit' if reason else 'copy', reason=reason or 'Byte-preserved original file; relative path unchanged.',
            bytes=path.stat().st_size, sha256=digest(path)))
    return rows


def bindings(value):
    entries = value.get('files', value)
    out = []
    if isinstance(entries, list):
        for item in entries:
            if not isinstance(item, dict) or 'sha256' not in item: continue
            name = item.get('path', item.get('copy'))
            if name is not None: out.append((name, item['sha256'], item.get('bytes')))
    elif isinstance(entries, dict):
        for name, entry in entries.items():
            if isinstance(entry, dict) and 'sha256' in entry:
                out.append((name, entry['sha256'], entry.get('bytes')))
            elif isinstance(entry, str) and len(entry) == 64:
                out.append((name, entry, None))
    return out


def audit(source):
    checked, external = [], []
    prefix = 'results/endpoint_pair_ablation_20261127/'
    manifests = set(source.rglob('*manifest.json'))
    manifests.update((source/'inference/all_three_sealed.json', source/'inputs/stage01/all_three_sealed.json'))
    for manifest in sorted(manifests):
        if '__pycache__' in manifest.parts: continue
        for name, expected, size in bindings(json.loads(manifest.read_text())):
            file = source/name[len(prefix):] if name.startswith(prefix) else manifest.parent/name
            if not file.exists() and (source/name).is_file():
                file = source/name  # e.g. all_three_sealed paths are base-relative.
            if not file.resolve().is_relative_to(source):
                external.append(dict(record=str(manifest.relative_to(source)), reference=name,
                    expected_sha256=expected, status='External original reference preserved, not opened.'))
                continue
            if not file.is_file(): raise ValueError('Missing in-scope original binding: '+str(file))
            actual, actual_size = digest(file), file.stat().st_size
            if actual != expected or (size is not None and actual_size != size):
                raise ValueError('Original artifact binding mismatch: '+str(file))
            checked.append(dict(record=str(manifest.relative_to(source)), file=str(file.relative_to(source)),
                                bytes=actual_size, sha256=actual, status='PASS'))
    # Validate the final inference registry. Historical failed verifier seals
    # refer to earlier live filenames, so their exact snapshots/manifests above
    # are preserved instead of asserting that their old live path is unchanged.
    seal_file = source/'protocol/inference_source_input_seal.json'
    seal = json.loads(seal_file.read_text())
    for group in ('sources', 'input_files'):
        for name, expected in seal[group].items():
            if name.startswith(prefix): file = source/name[len(prefix):]
            else:
                external.append(dict(record='protocol/inference_source_input_seal.json', reference=name,
                    expected_sha256=expected, status='External original reference preserved, not opened.'))
                continue
            if not file.resolve().is_relative_to(source) or not file.is_file() or digest(file) != expected:
                raise ValueError('Final in-scope inference binding differs: '+name)
            checked.append(dict(record='protocol/inference_source_input_seal.json',
                file=str(file.relative_to(source)), bytes=file.stat().st_size, sha256=expected, status='PASS'))
    provenance = json.loads((source/'reference_i70/copy_provenance.json').read_text())
    parent_copy = source/'reference_i70/science.json'
    registered = json.loads((source/'protocol/protocol.json').read_text())['source_input_bindings']
    if (digest(parent_copy) != provenance['sha256'] or parent_copy.stat().st_size != provenance['bytes']
            or registered[provenance['original_path']] != provenance['sha256']):
        raise ValueError('Copied parent summary differs from its registered original hash')
    checked.append(dict(record='reference_i70/copy_provenance.json and protocol source_input_bindings',
        file='reference_i70/science.json', bytes=parent_copy.stat().st_size,
        sha256=provenance['sha256'], status='PASS; original external file was not opened'))
    return dict(all_original_in_scope_bindings_pass=True, checked_count=len(checked),
                checks=checked, external_references_not_opened=external)


def main():
    p = argparse.ArgumentParser(); p.add_argument('--source', type=Path, required=True); args = p.parse_args()
    source, target = args.source.resolve(), Path(__file__).resolve().parent
    if source.name != 'endpoint_pair_ablation_20261127' or source.parent.name != 'results':
        raise ValueError('Only the requested I71 results base may be read')
    for name in ('review/combined_contracts_stage01/receipt.json', 'publication/plots_stage01/render_receipt.json',
                 'reference_i70/science.json'):
        if not (source/name).is_file(): raise ValueError('Final stage not yet available: '+name)
    for name in ('export_plan.json', 'export_manifest.json', 'transfer_receipt.json'):
        if (target/name).exists(): raise FileExistsError(target/name)
    rows = inventory(source)
    generated_names = {'transfer_evidence.py', 'export_plan.json', 'export_manifest.json',
                       'transfer_receipt.json', 'artifact_binding_audit.json'}
    if generated_names.intersection(row['source'] for row in rows): raise ValueError('Generated record collision')
    plan = dict(status='SEALED before any source copy', timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        source_base=str(source), path_policy='Every included source relative path is preserved exactly.', entries=rows,
        copied_files=sum(r['action']=='copy' for r in rows), omitted_files=sum(r['action']=='omit' for r in rows),
        copied_bytes=sum(r['bytes'] for r in rows if r['action']=='copy'),
        omitted_bytes=sum(r['bytes'] for r in rows if r['action']=='omit'))
    save(target/'export_plan.json', plan)
    original = audit(source); save(target/'artifact_binding_audit.json', original)
    copied = []
    for row in rows:
        if row['action'] == 'omit': continue
        before, after = source/row['source'], target/row['destination']
        if digest(before) != row['sha256']: raise ValueError('Source changed after plan: '+row['source'])
        if after.exists(): raise FileExistsError(after)
        after.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(before, after)
        if digest(after) != row['sha256'] or after.stat().st_size != row['bytes']:
            raise ValueError('Copy verification failed: '+row['source'])
        copied.append(dict(path=row['destination'], source=row['source'], bytes=row['bytes'], sha256=row['sha256']))
    if inventory(source) != rows: raise ValueError('Source inventory changed while copying')
    support = {name: dict(bytes=(target/name).stat().st_size, sha256=digest(target/name))
               for name in ('transfer_evidence.py', 'export_plan.json', 'artifact_binding_audit.json')}
    save(target/'export_manifest.json', dict(files=copied, generated_support_files=support,
        omissions='Every omitted original path/size/hash/reason is recorded in export_plan.json.',
        self_reference_policy='Manifest excludes itself and its subsequent receipt; receipt binds this manifest.'))
    receipt = dict(status='PASS: planned copies, final in-scope inference seal and original artifact bindings verified',
        timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), source_inventory_unchanged=True,
        copied_files=len(copied), copied_bytes=sum(r['bytes'] for r in copied),
        omitted_files=plan['omitted_files'], omitted_bytes=plan['omitted_bytes'],
        original_bindings_checked=original['checked_count'],
        export_plan_sha256=digest(target/'export_plan.json'), export_manifest_sha256=digest(target/'export_manifest.json'),
        generated_support_files=support, no_source_mutations=True, no_scientific_computation=True,
        no_self_recursive_export_plan=True)
    save(target/'transfer_receipt.json', receipt)
    print(json.dumps(receipt))


if __name__ == '__main__': main()
