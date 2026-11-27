"""Standalone NumPy I-61 load-veto and N/A/C policy replay."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import sys

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_hash(array):
    return hashlib.sha256(np.asarray(array).tobytes()).hexdigest()


def load_npz(path):
    with np.load(path, allow_pickle=False) as raw:
        return {key: raw[key].copy() for key in raw.files}


def load_core(path):
    spec = importlib.util.spec_from_file_location('_i61_portable_load_gate', path)
    module = importlib.util.module_from_spec(spec);sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def decode_inputs(encoded, recipes):
    decoded = {}
    for key, recipe in recipes.items():
        kind = recipe['encoding']
        if kind == 'plain':
            value = encoded[key].copy()
        elif kind == 'alias':
            value = decoded[recipe['reference']].copy()
        elif kind == 'first_six':
            value = decoded[recipe['reference']][:, :6].copy()
        elif kind == 'constant':
            value = np.full(recipe['shape'], encoded[key].reshape(-1)[0], dtype=np.dtype(recipe['dtype']))
        elif kind == 'xor64':
            reference = np.ascontiguousarray(decoded[recipe['reference']])
            bits = np.bitwise_xor(encoded[key], reference.view(np.uint64))
            value = bits.view(np.dtype(recipe['dtype'])).reshape(recipe['shape']).copy()
        else:
            raise ValueError('Unknown lossless input encoding: '+kind)
        if list(value.shape) != recipe['shape'] or value.dtype.str != recipe['dtype'] or array_hash(value) != recipe['array_sha256']:
            raise ValueError('Decoded input differs byte-for-byte: '+key)
        decoded[key] = value
    return decoded


def derive_packet(inputs, core, gate_options):
    pack = {key: value.copy() for key, value in inputs.items()}
    for prefix in ('proposed', 'native_alternative'):
        valid = pack[prefix+'__valid']
        residual = pack['observer_generalized']-pack[prefix+'__prediction']
        residual[~valid] = np.nan
        pack[prefix+'__residual'] = residual
        pack[prefix+'__extra_joint_residual_Nm'] = residual[:, 6:9].copy()
        feature = {name: pack[prefix+'__'+name] for name in
                   ('valid', 'residual', 'force_norm_N', 'radius_m')}
        for name, value in core.load_compatibility(feature, **gate_options).items():
            pack[prefix+'_gate__'+name] = value
    pack['native_relative_max_ratio_difference'] = (
        pack['proposed_gate__max_normalized_joint_residual']
        -pack['native_alternative_gate__max_normalized_joint_residual'])
    accepted = pack['i60_accepted'];gated = accepted&pack['proposed_gate__passes']
    pack.update(N_family=pack['native_family'].copy(),
        A_family=np.where(accepted, pack['proposed_family'], pack['native_family']),
        C_family=np.where(gated, pack['proposed_family'], pack['native_family']),
        N_point_m=pack['native_point_m'].copy(),
        A_point_m=np.where(accepted[:, None], pack['proposed_point_m'], pack['native_point_m']),
        C_point_m=np.where(gated[:, None], pack['proposed_point_m'], pack['native_point_m']),
        A_uses_proposal=accepted.copy(), C_uses_proposal=gated)
    return pack


def compare_packet(packet, expected):
    if set(packet) != set(expected):
        raise ValueError('Recomputed packet schema differs: '+str(set(packet)^set(expected)))
    fields = {}
    for key, value in packet.items():
        target = expected[key]
        fields[key] = (list(value.shape) == target['shape'] and value.dtype.str == target['dtype']
                       and array_hash(value) == target['array_sha256'])
    if not all(fields.values()):
        raise ValueError('Packet byte mismatch: '+', '.join(k for k, v in fields.items() if not v))
    return fields


def verify_manifest(source):
    manifest = json.loads((source/'load_gate_manifest.json').read_text())
    for item in manifest['files']:
        if sha(source/item['path']) != item['sha256']:
            raise ValueError('Changed portable load-gate artifact: '+item['path'])
    return manifest


def load_and_gate_case(source, case):
    """Return full freshly rederived feature/policy packet and exactness report.

    Predictions/affine maps are supplied geometric feature INPUTS. This replay
    recomputes residuals, gates and policy outputs; it does not claim to rerun
    MuJoCo Jacobians or reconstruct the candidate bank.
    """
    source = Path(source);verify_manifest(source)
    directory = source/case
    metadata = json.loads((directory/'metadata.json').read_text())
    if metadata['case'] != case:
        raise ValueError('Portable case metadata differs')
    core = load_core(source/'source/contact_load_consistency.py')
    inputs = decode_inputs(load_npz(directory/'inputs.npz'), metadata['input_recipes'])
    packet = derive_packet(inputs, core, metadata['gate_options'])
    fields = compare_packet(packet, metadata['expected_packet'])
    report = dict(case=case, rows=len(packet['time_s']), fields=len(fields),
        all_packet_fields_byte_exact=True, fields_byte_exact=fields,
        no_truth_used=True, new_physics_steps=0,
        scope='Fixed load gate and N/A/C arithmetic; geometric predictions and oracle maps are preserved feature inputs.')
    return packet, report


def run(source, output):
    if output.exists():
        raise FileExistsError(output)
    manifest = verify_manifest(source);reports = {}
    for case in manifest['cases']:
        _, reports[case] = load_and_gate_case(source, case)
    output.mkdir(parents=True)
    (output/'load_gate_replay.json').write_text(json.dumps(dict(iteration='I-61', cases=reports,
        all_packet_fields_byte_exact=True, source_manifest_sha256=sha(source/'load_gate_manifest.json'),
        no_truth_used=True, new_physics_steps=0), indent=2)+'\n')
    print(json.dumps(dict(cases=len(reports), all_packet_fields_byte_exact=True, new_physics_steps=0)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args();run(args.data_source.resolve(), args.output.resolve())
