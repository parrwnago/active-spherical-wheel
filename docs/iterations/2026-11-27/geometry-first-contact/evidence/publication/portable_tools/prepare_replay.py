"""Bind released I72 canonical arrays and portable sources before replay.

Metadata preparation only: this does not call selection or ranking. Run once
after the all-three canonical inference seal; refuse to overwrite bindings.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np


CASES = ('m100_B', 'p030_B', 'p090_B')
RELEASED_INFERENCE_SEAL = '955f3bd73551de7adc5f9fec51ca49382b450ddf5c8afbf7ff56e12778b93c37'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_new(path, value):
    with Path(path).open('x') as stream:
        stream.write(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def binding(value):
    value = np.asarray(value)
    return dict(dtype=value.dtype.str, shape=list(value.shape),
                sha256=hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest())


def prepare(package):
    package = Path(package).resolve()
    tools = package / 'publication/portable_tools'
    inference_seal = package / 'inference/all_three_sealed.json'
    if sha(inference_seal) != RELEASED_INFERENCE_SEAL:
        raise ValueError('Canonical inference seal is not the released all-three seal')
    released = json.loads(inference_seal.read_text())
    for record in released['files']:
        path = package / record['path']
        if sha(path) != record['sha256'] or path.stat().st_size != record['bytes']:
            raise ValueError('Released canonical artifact changed: ' + record['path'])
    master_path = package / 'protocol/inference_source_input_seal.json'
    if sha(master_path) != released['source_input_seal_sha256']:
        raise ValueError('Canonical scientific source/input seal changed')
    master = json.loads(master_path.read_text())
    scientific = ('inference_tools/geometry_first_contact.py', 'inference_tools/frozen_i60_rank.py',
                  'inference_tools/run_case.py', 'inputs/packet_io.py')
    for relative in scientific:
        key = 'results/geometry_first_contact_20261127/' + relative
        if sha(package / relative) != master['sources'][key]:
            raise ValueError('Canonical scientific source changed: ' + relative)
    cases = {}
    for case in CASES:
        original = package / 'inference/stage01' / case / 'predictions.npz'
        with np.load(original, allow_pickle=False) as packet:
            arrays = {key: binding(packet[key]) for key in packet.files}
        cases[case] = dict(original_file='inference/stage01/' + case + '/predictions.npz',
                           original_file_sha256=sha(original), arrays=arrays)
    references = tools / 'reference_array_bindings.json'
    save_new(references, dict(iteration='I-72', registered_utc=datetime.now(timezone.utc).isoformat(),
        released_inference_seal_sha256=sha(inference_seal), cases=cases,
        scope='All original prediction arrays, not compressed NPZ archive byte identity.'))
    paths = list(scientific) + [
        'publication/portable_tools/prepare_replay.py',
        'publication/portable_tools/replay_inference.py',
        'publication/portable_tools/reference_array_bindings.json',
        'protocol/protocol.json', 'protocol/inference_source_input_seal.json',
        'inputs/stage01/all_three_sealed.json', 'inference/all_three_sealed.json']
    for case in CASES:
        paths += ['inputs/stage01/' + case + '/' + name for name in
                  ('rank_packet.npz', 'native_fallback.npz', 'metadata.json', 'artifact_manifest.json')]
        paths += ['inference/stage01/' + case + '/' + name for name in
                  ('registered_before_inference.json', 'completed.json', 'artifact_manifest.json')]
    files = {relative: sha(package / relative) for relative in sorted(paths)}
    save_new(tools / 'replay_source_input_seal.json', dict(iteration='I-72',
        registered_utc=datetime.now(timezone.utc).isoformat(), files=files,
        original_predictions_bound_by_array_reference=True,
        boundary='Frozen original inputs and exact copied numerical modules only. No truth, model, simulation, or calibration.'))
    print(json.dumps(dict(cases=3, prediction_arrays=sum(len(value['arrays']) for value in cases.values()),
                         files=len(files), reference_sha256=sha(references),
                         replay_seal_sha256=sha(tools / 'replay_source_input_seal.json')), sort_keys=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    prepare(parser.parse_args().package)
