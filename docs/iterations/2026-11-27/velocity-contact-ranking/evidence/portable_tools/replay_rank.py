"""Standalone NumPy replay of fixed I-60 rankings from minima-preserving inputs."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import sys

import numpy as np

MODES = ('geometry', 'wrench', 'velocity', 'fused')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_npz(path):
    with np.load(path, allow_pickle=False) as data:
        return {name: data[name].copy() for name in data.files}


def exact(left, right):
    left, right = np.asarray(left), np.asarray(right)
    return left.shape == right.shape and left.dtype == right.dtype and left.tobytes() == right.tobytes()


def load_core(path):
    spec = importlib.util.spec_from_file_location('_i60_portable_rank_core', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def flatten_scores(scores):
    return {mode+'__'+name: np.asarray(value) for mode in MODES for name, value in scores[mode].items()}


def original_candidate_indices(scores, source_candidate_index):
    """Map only per-family minima indices; all other outputs stay untouched."""
    result = {key: np.array(value, copy=True) for key, value in scores.items()}
    source = np.asarray(source_candidate_index)
    for mode in MODES:
        key = mode+'__candidate_index'
        index = result[key]
        if index.shape != (len(source), 4):
            raise ValueError('Expected one candidate index per original family')
        selected = np.take_along_axis(source, np.maximum(index, 0), axis=1)
        result[key] = np.where(index >= 0, selected, -1).astype(index.dtype)
    return result


def compare_scores(actual, expected):
    if set(actual) != set(expected):
        raise ValueError('Ranking output schema changed')
    fields = {key: exact(actual[key], expected[key]) for key in sorted(actual)}
    if not all(fields.values()):
        raise ValueError('Ranking byte mismatch: '+', '.join(k for k, v in fields.items() if not v))
    return fields


def verify_manifest(data_source):
    manifest = json.loads((data_source/'rank_manifest.json').read_text())
    for item in manifest['files']:
        path = data_source/item['path']
        if not path.is_file() or sha(path) != item['sha256']:
            raise ValueError('Changed portable ranking artifact: '+str(path))
    return manifest


def load_and_rank_case(data_source, case):
    """Return minimal context, freshly computed ORIGINAL-index scores, parity.

    This is the evaluation replay hook. Truth is absent from this module and
    package; an independent evaluator may combine its own scored truth inputs.
    """
    data_source = Path(data_source)
    verify_manifest(data_source)
    directory = data_source/case
    metadata = json.loads((directory/'metadata.json').read_text())
    if metadata['case'] != case:
        raise ValueError('Portable case metadata differs')
    core = load_core(data_source/'source/contact_hypotheses.py')
    features = load_npz(directory/'projected_features.npz')
    scores = flatten_scores(core.rank_hypotheses(features, **metadata['rank_options']))
    scores = original_candidate_indices(scores, features['source_candidate_index'])
    stored = load_npz(directory/'scores.npz')
    fields = compare_scores(scores, stored)
    context = load_npz(directory/'measurement_context.npz')
    report = dict(case=case, rows=len(features['frame_valid']),
        ranking_fields=len(fields), all_fields_byte_exact=True,
        original_candidate_indices_byte_exact=True, fields_byte_exact=fields,
        compact_candidate_slots=int(len(features['family_index'])),
        original_candidate_slots=int(metadata['original_candidate_slots']),
        no_mujoco_or_robot_module_required=True, truth_used=False, new_physics_steps=0)
    return context, scores, report


def run(data_source, output):
    if output.exists():
        raise FileExistsError(output)
    manifest = verify_manifest(data_source)
    output.mkdir(parents=True)
    reports = {}
    for case in manifest['cases']:
        _, _, report = load_and_rank_case(data_source, case)
        reports[case] = report
    summary = dict(iteration='I-60', cases=reports,
        all_rank_fields_byte_exact=all(r['all_fields_byte_exact'] for r in reports.values()),
        source_manifest_sha256=sha(data_source/'rank_manifest.json'),
        scope='Fixed four-score rank replay only; cannot regenerate the geometric candidate bank or retune scores from the compact minima.',
        new_physics_steps=0, truth_used=False)
    (output/'rank_replay.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(dict(cases=len(reports), all_rank_fields_byte_exact=True, new_physics_steps=0)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.data_source.resolve(), args.output.resolve())
