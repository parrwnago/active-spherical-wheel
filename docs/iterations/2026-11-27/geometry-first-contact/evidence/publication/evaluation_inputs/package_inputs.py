"""Copy only the frozen I72 projector's exact baseline field whitelists.

No selection, confidence calculation, label construction or scoring is done.
Original sources/files are sealed before any NPZ is opened, then every copied
array is checked by dtype, shape and bytes after the archive is reloaded.
"""
import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_new(path, value):
    with Path(path).open('x') as stream:
        stream.write(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def binding(value):
    return dict(dtype=value.dtype.str, shape=list(value.shape),
                sha256=hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest())


def constants(path):
    wanted = {'CASES', 'BASE_FIELDS', 'OLD_FIELDS'}
    output = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in wanted:
                    output[target.id] = ast.literal_eval(node.value)
    if set(output) != wanted:
        raise ValueError('Frozen projector whitelist constants unavailable')
    return output


def run(repo, package):
    repo = Path(repo).resolve()
    package = Path(package).resolve()
    output = package / 'publication/evaluation_inputs'
    projector = package / 'evaluation/prepare_evaluation.py'
    prior_seal = package / 'evaluation/pre_truth_execution_stage01/source_input_seal.json'
    historical = json.loads(prior_seal.read_text())
    expected = {record['path']: record['sha256']
                for group in ('sources', 'inputs') for record in historical[group]}
    projector_key = str(projector.relative_to(repo))
    if sha(projector) != expected[projector_key]:
        raise ValueError('Frozen projector source changed')
    declared = constants(projector)
    records = []
    for case in declared['CASES']:
        records.append(dict(case=case, group='old_i70', fields=declared['BASE_FIELDS'],
            source=repo / 'docs/iterations/2026-11-27/temporal-contact-modes/evidence/evaluation/packets_stage01' / case / 'evaluation_packet.npz',
            relative='old_i70/' + case + '/evaluation_packet.npz'))
        records.append(dict(case=case, group='old_i60', fields=declared['OLD_FIELDS'],
            source=repo / 'results/velocity_contact_rank_20261127/analysis/stage01' / (case + '_evaluation.npz'),
            relative='old_i60/' + case + '_evaluation.npz'))
    source_files = [Path(__file__).resolve(), projector, prior_seal] + [record['source'] for record in records]
    original_bindings = {str(path.relative_to(repo)): dict(sha256=sha(path), bytes=path.stat().st_size)
                         for path in source_files}
    for record in records:
        key = str(record['source'].relative_to(repo))
        if original_bindings[key]['sha256'] != expected[key]:
            raise ValueError('Baseline differs from actual pre-truth evaluation binding: ' + key)
    save_new(output / 'registered_before_copy.json', dict(iteration='I-72',
        registered_utc=datetime.now(timezone.utc).isoformat(), files=original_bindings,
        authoritative_projector_whitelists=declared,
        operation='Copy-only evaluation inputs; no field values, dtypes, confidence or fixed-domain definitions are recomputed.'))
    copied = []
    for record in records:
        source = record['source']
        destination = output / record['relative']
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            raise FileExistsError(destination)
        with np.load(source, allow_pickle=False) as original:
            arrays = {name: original[name].copy() for name in record['fields']}
        original_arrays = {name: binding(value) for name, value in arrays.items()}
        np.savez_compressed(destination, **arrays)
        with np.load(destination, allow_pickle=False) as archive:
            copied_arrays = {name: binding(archive[name]) for name in archive.files}
        if original_arrays != copied_arrays:
            raise AssertionError('Copy array mismatch: ' + record['relative'])
        copied.append(dict(case=record['case'], group=record['group'], path=record['relative'],
            bytes=destination.stat().st_size, sha256=sha(destination),
            original_path=str(source.relative_to(repo)), original_sha256=sha(source),
            arrays=original_arrays, all_copied_arrays_byte_exact=True))
    for relative, original in original_bindings.items():
        path = repo / relative
        if sha(path) != original['sha256'] or path.stat().st_size != original['bytes']:
            raise AssertionError('Source/original input changed during copy: ' + relative)
    save_new(output / 'metadata.json', dict(iteration='I-72',
        completed_utc=datetime.now(timezone.utc).isoformat(), files=copied,
        packaging_source_sha256=sha(Path(__file__)),
        registration_sha256=sha(output / 'registered_before_copy.json'),
        projector_source_sha256=sha(projector), prior_evaluation_input_seal_sha256=sha(prior_seal),
        all57_arrays_byte_exact=True, source_inputs_unchanged=True,
        preserved='Original labels, acceptance/confidence fields and fixed-domain inputs, copied without recomputation.',
        omitted='All original arrays outside the authoritative BASE_FIELDS and OLD_FIELDS whitelists.',
        fresh_evaluation='Use unchanged evaluation/prepare_evaluation.py with these two baseline roots and freshly replayed predictions.'))
    print(json.dumps(dict(files=len(copied), arrays=sum(len(value['arrays']) for value in copied),
                         bytes=sum(value['bytes'] for value in copied), all_byte_exact=True), sort_keys=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, required=True)
    parser.add_argument('--package', type=Path, required=True)
    args = parser.parse_args()
    run(args.repo_root, args.package)
