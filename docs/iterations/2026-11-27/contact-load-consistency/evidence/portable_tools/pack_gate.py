"""Losslessly pack sealed I-61 feature inputs; verify every gate/policy field."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import json
import shutil

import numpy as np

from replay_gate import (array_hash, compare_packet, decode_inputs, derive_packet,
                         load_core, load_npz, sha)

CASES = ('m100_B', 'p030_B', 'p090_B')
FEATURES = ('valid', 'prediction', 'force_norm_N', 'radius_m',
            'point_geometry_valid', 'wrench_matches_generalized')
BASE = ('time_s', 'native_family', 'proposed_family', 'i60_accepted',
        'native_point_m', 'proposed_point_m', 'native_alternative_point_m',
        'proposed_candidate_index', 'native_candidate_index',
        'observer_generalized', 'observer_wrench', 'observer_interval_mean')
MAPS = ('valid', 'centre_m', 'joint_at_centre', 'joint_moment_map', 'force_norm_N', 'radius_m')
XOR_REFERENCES = {'proposed_point_m': 'native_point_m',
    'native_alternative_point_m': 'proposed_point_m',
    'native_candidate_index': 'proposed_candidate_index',
    'observer_interval_mean': 'observer_generalized',
    'proposed__prediction': 'observer_generalized',
    'native_alternative__prediction': 'proposed__prediction'}
ALIASES = {'native_alternative__force_norm_N': 'proposed__force_norm_N',
    'native_alternative__radius_m': 'proposed__radius_m',
    'map__force_norm_N': 'proposed__force_norm_N', 'map__radius_m': 'proposed__radius_m'}


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n')


def signature(value):
    return dict(shape=list(value.shape), dtype=value.dtype.str, array_sha256=array_hash(value))


def encode_inputs(packet):
    fields = BASE+tuple(p+'__'+k for p in ('proposed', 'native_alternative') for k in FEATURES)+tuple('map__'+k for k in MAPS)
    encoded, recipes = {}, {}
    for key in fields:
        value = packet[key];recipe = signature(value)
        if key == 'observer_wrench':
            reference = packet['observer_generalized'][:, :6]
            if array_hash(reference) != array_hash(value):
                raise ValueError('Wrench is not the exact first six generalized components')
            recipe.update(encoding='first_six', reference='observer_generalized')
        elif key in ALIASES and signature(packet[ALIASES[key]]) == signature(value):
            recipe.update(encoding='alias', reference=ALIASES[key])
        elif key in XOR_REFERENCES:
            reference = packet[XOR_REFERENCES[key]]
            if value.shape != reference.shape or value.dtype.itemsize != 8 or reference.dtype != value.dtype:
                raise ValueError('Registered lossless XOR reference shape/type differs: '+key)
            encoded[key] = np.bitwise_xor(np.ascontiguousarray(value).view(np.uint64),
                                         np.ascontiguousarray(reference).view(np.uint64))
            recipe.update(encoding='xor64', reference=XOR_REFERENCES[key])
        elif key == 'proposed__radius_m' and array_hash(np.full_like(value, value.reshape(-1)[0])) == array_hash(value):
            encoded[key] = value.reshape(-1)[:1].copy();recipe.update(encoding='constant')
        else:
            encoded[key] = value.copy();recipe.update(encoding='plain')
        recipes[key] = recipe
    return encoded, recipes


def validate_case(directory):
    manifest_path = directory/'artifact_manifest.json'
    if not manifest_path.is_file():
        raise RuntimeError('All three feature cases must be sealed before projection')
    manifest = json.loads(manifest_path.read_text())
    for item in manifest['files']:
        if sha(directory/item['path']) != item['sha256']:
            raise ValueError('Sealed load feature changed: '+item['path'])
    metadata = json.loads((directory/'metadata.json').read_text())
    registration = json.loads((directory/'registered_before_arithmetic.json').read_text())
    if metadata['case'] != directory.name or metadata['case'] not in CASES:
        raise ValueError('Unknown or misnamed case')
    return metadata, registration


def run(directories, output):
    if output.exists():
        raise FileExistsError(output)
    # All seals are checked before any numerical feature packet is opened.
    bindings = [validate_case(path) for path in directories]
    if sorted(x[0]['case'] for x in bindings) != sorted(CASES):
        raise ValueError('Exactly the three registered cases are required')
    core_hashes = {sha(path/'source/contact_load_consistency.py') for path in directories}
    if len(core_hashes) != 1:
        raise ValueError('Cases do not share one frozen load-gate source')
    output.mkdir(parents=True);(output/'source').mkdir()
    shutil.copy2(directories[0]/'source/contact_load_consistency.py', output/'source/contact_load_consistency.py')
    shutil.copy2(Path(__file__), output/'pack_gate.py')
    shutil.copy2(Path(__file__).with_name('replay_gate.py'), output/'replay_gate.py')
    save(output/'registered_before_projection.json', dict(iteration='I-61',
        captured_utc=datetime.now(timezone.utc).isoformat(),
        method='Lossless input aliases/XOR; recompute full residual, gate and N/A/C fields. No weights, thresholds or policies change.',
        inputs=[dict(path=str(path/name), sha256=sha(path/name)) for path in directories
                for name in ('load_features_and_policies.npz', 'artifact_manifest.json',
                             'registered_before_arithmetic.json', 'source/contact_load_consistency.py')],
        packer_sha256=sha(Path(__file__)), replayer_sha256=sha(Path(__file__).with_name('replay_gate.py')),
        no_truth_read=True))
    core = load_core(output/'source/contact_load_consistency.py');reports = {}
    for directory, (metadata, registration) in zip(directories, bindings):
        case = metadata['case'];destination = output/case;destination.mkdir()
        packet = load_npz(directory/'load_features_and_policies.npz')
        encoded, recipes = encode_inputs(packet)
        inputs = decode_inputs(encoded, recipes)
        expected = {key: signature(value) for key, value in packet.items()}
        options = registration['gate_options']
        derived = derive_packet(inputs, core, options);fields = compare_packet(derived, expected)
        np.savez_compressed(destination/'inputs.npz', **encoded)
        roundtrip = derive_packet(decode_inputs(load_npz(destination/'inputs.npz'), recipes), core, options)
        compare_packet(roundtrip, expected)
        save(destination/'metadata.json', dict(iteration='I-61', case=case,
            input_recipes=recipes, expected_packet=expected, gate_options=options,
            original_packet_sha256=sha(directory/'load_features_and_policies.npz'),
            scope='Geometric predictions/validity/affine maps are inputs, not reproduced by the NumPy gate replay. All derived residual/gate/policy fields are recomputed.'))
        shutil.copy2(directory/'artifact_manifest.json', destination/'original_artifact_manifest.json')
        shutil.copy2(directory/'registered_before_arithmetic.json', destination/'original_registration.json')
        reports[case] = dict(rows=len(packet['time_s']), full_packet_fields=len(fields),
            encoded_input_fields=len(encoded), reconstructed_input_fields=len(recipes),
            all_packet_fields_byte_exact=True, fields_byte_exact=fields,
            array_roundtrip_byte_exact=True)
    save(output/'projection_report.json', dict(iteration='I-61', cases=reports,
        all_cases_byte_exact=True, no_truth_read=True, new_physics_steps=0))
    files = [p for p in output.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    save(output/'load_gate_manifest.json', dict(iteration='I-61', cases=list(CASES),
        files=[dict(path=str(p.relative_to(output)), sha256=sha(p), bytes=p.stat().st_size) for p in sorted(files)]))
    print(json.dumps(dict(cases=len(reports), all_packet_fields_byte_exact=True,
        bytes=sum(p.stat().st_size for p in output.rglob('*') if p.is_file()), no_truth_read=True)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case-directories', type=Path, nargs=3, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args();run([p.resolve() for p in args.case_directories], args.output.resolve())
