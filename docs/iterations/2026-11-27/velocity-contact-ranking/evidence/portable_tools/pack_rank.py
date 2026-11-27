"""Pack sealed I-60 features while preserving all fixed-mode family minima."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import json
import shutil

import numpy as np

from replay_rank import (MODES, compare_scores, exact, flatten_scores, load_core,
                         load_npz, original_candidate_indices, sha)

CASES = ('m100_B', 'p030_B', 'p090_B')
SCALARS = ('normal_angle_rad', 'velocity_residual_norm_m_s', 'moment_residual_norm')
CONTEXT = ('time_s', 'exact_estimated_support_loads_N', 'measured_excitation_m_s')


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def project_minima(features, scores):
    """At most four winners per family; preserve stable original-index ties."""
    valid = np.asarray(features['valid'], bool)
    family = np.asarray(features['family_index'])
    if valid.ndim != 2 or family.shape != (valid.shape[1],):
        raise ValueError('Expected fixed candidates over a full-rate frame table')
    rows, count = valid.shape
    choices = np.stack([scores[mode+'__candidate_index'] for mode in MODES], axis=1)
    if choices.shape != (rows, 4, 4):
        raise ValueError('Expected four fixed modes and four family minima')
    source_indices = np.full((rows, 16), -1, np.int16)
    for row in range(rows):
        for label in range(4):
            selected = np.unique(choices[row, :, label])
            selected = selected[selected >= 0]
            if (len(selected) > 4 or np.any(selected >= count)
                    or np.any(family[selected] != label) or not valid[row, selected].all()):
                raise ValueError('A stored minimum does not identify a valid original family candidate')
            source_indices[row, label*4:label*4+len(selected)] = selected
    keep = source_indices >= 0
    safe = np.maximum(source_indices, 0)
    row_index = np.arange(rows)[:, None]
    output = dict(family_index=np.repeat(np.arange(4, dtype=family.dtype), 4),
        valid=keep, frame_valid=np.array(features['frame_valid'], copy=True),
        excitation_m_s=np.array(features['excitation_m_s'], copy=True),
        source_candidate_index=source_indices)
    for key in SCALARS:
        value = np.asarray(features[key])
        if value.shape != valid.shape:
            raise ValueError('Candidate scalar shape differs: '+key)
        output[key] = np.where(keep, value[row_index, safe], 0.)
    velocity = np.asarray(features['velocity_residual_m_s'])
    if velocity.shape != valid.shape+(3,):
        raise ValueError('Candidate vector shape differs')
    output['velocity_residual_m_s'] = np.where(keep[..., None], velocity[row_index, safe], 0.)
    return output


def validate_case(directory):
    manifest_path = directory/'artifact_manifest.json'
    if not manifest_path.is_file():
        raise RuntimeError('Case must be sealed before projection: '+str(directory))
    manifest = json.loads(manifest_path.read_text())
    for item in manifest['files']:
        if sha(directory/item['path']) != item['sha256']:
            raise ValueError('Original sealed artifact changed: '+str(directory/item['path']))
    metadata = json.loads((directory/'metadata.json').read_text())
    registration = json.loads((directory/'registered_before_arithmetic.json').read_text())
    if metadata['case'] != directory.name or metadata['case'] not in CASES:
        raise ValueError('Only the three preregistered cases are accepted')
    required = ('projected_features.npz', 'scores.npz', 'measurement_context.npz',
                'metadata.json', 'registered_before_arithmetic.json',
                'artifact_manifest.json', 'source/contact_hypotheses.py')
    return dict(case=metadata['case'], metadata=metadata, registration=registration,
        input_files=[dict(path=str(directory/name), sha256=sha(directory/name)) for name in required])


def run(directories, output):
    if output.exists():
        raise FileExistsError(output)
    # Validate ALL three seals before opening any numerical projection input.
    bound = [validate_case(path) for path in directories]
    if sorted(item['case'] for item in bound) != sorted(CASES):
        raise ValueError('Exactly the three registered cases, once each, are required')
    core_hashes = {sha(path/'source/contact_hypotheses.py') for path in directories}
    if len(core_hashes) != 1:
        raise ValueError('Original cases do not share one frozen ranking source')
    output.mkdir(parents=True)
    source = output/'source';source.mkdir()
    shutil.copy2(directories[0]/'source/contact_hypotheses.py', source/'contact_hypotheses.py')
    shutil.copy2(Path(__file__), output/'pack_rank.py')
    shutil.copy2(Path(__file__).with_name('replay_rank.py'), output/'replay_rank.py')
    save(output/'registered_before_projection.json', dict(iteration='I-60',
        captured_before_projection_utc=datetime.now(timezone.utc).isoformat(),
        inputs=[item for case in bound for item in case['input_files']],
        packer_sha256=sha(Path(__file__)),
        replayer_sha256=sha(Path(__file__).with_name('replay_rank.py')),
        core_sha256=next(iter(core_hashes)), truth_accessed=False,
        method='Union of four original per-family mode minima, sorted original candidate indices; no rescoring or tuning is used to choose the compact bank.'))
    core = load_core(source/'contact_hypotheses.py')
    reports = {}
    for original, binding in zip(directories, bound):
        case = binding['case'];destination = output/case;destination.mkdir()
        features = load_npz(original/'projected_features.npz')
        scores = load_npz(original/'scores.npz')
        compact = project_minima(features, scores)
        options = binding['registration']['rank_options']
        computed = flatten_scores(core.rank_hypotheses(compact, **options))
        computed = original_candidate_indices(computed, compact['source_candidate_index'])
        fields = compare_scores(computed, scores)
        np.savez_compressed(destination/'projected_features.npz', **compact)
        # Stored original outputs provide independent parity targets for replay.
        np.savez_compressed(destination/'scores.npz', **scores)
        with np.load(original/'measurement_context.npz', allow_pickle=False) as raw:
            context = {key: raw[key].copy() for key in CONTEXT}
        np.savez_compressed(destination/'measurement_context.npz', **context)
        loaded = load_npz(destination/'projected_features.npz')
        if not all(exact(loaded[key], value) for key, value in compact.items()):
            raise ValueError('Compact array round-trip differs')
        saved_context = load_npz(destination/'measurement_context.npz')
        if not all(exact(saved_context[key], value) for key, value in context.items()):
            raise ValueError('Minimal evaluation context differs')
        shutil.copy2(original/'artifact_manifest.json', destination/'original_artifact_manifest.json')
        shutil.copy2(original/'registered_before_arithmetic.json', destination/'original_registration.json')
        save(destination/'metadata.json', dict(iteration='I-60', case=case,
            rows=int(len(compact['frame_valid'])), compact_candidate_slots=16,
            original_candidate_slots=int(len(features['family_index'])), rank_options=options,
            original_feature_file_sha256=sha(original/'projected_features.npz'),
            original_score_file_sha256=sha(original/'scores.npz'),
            original_context_file_sha256=sha(original/'measurement_context.npz'),
            source_candidate_semantics='Original candidate ID and exact selected feature values are preserved. World points were not stored in the original projected feature table.',
            scope='Four frozen score definitions and confidence rules only; new weights/feature generation require original full bank and raw states.'))
        reports[case] = dict(rows=int(len(compact['frame_valid'])),
            all_rank_fields_byte_exact=all(fields.values()), fields_byte_exact=fields,
            original_candidate_indices_byte_exact=True,
            compact_array_roundtrip_byte_exact=True, minimal_context_byte_exact=True,
            retained_valid_slot_count=int(compact['valid'].sum()),
            maximum_retained_valid_slots_per_frame=int(compact['valid'].sum(axis=1).max()),
            minimum_retained_valid_slots_per_frame=int(compact['valid'].sum(axis=1).min()))
    save(output/'projection_report.json', dict(iteration='I-60', cases=reports,
        all_cases_exact=all(item['all_rank_fields_byte_exact'] for item in reports.values()),
        truth_accessed=False, new_physics_steps=0))
    files = [p for p in output.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    save(output/'rank_manifest.json', dict(iteration='I-60', cases=list(CASES),
        files=[dict(path=str(p.relative_to(output)), sha256=sha(p), bytes=p.stat().st_size)
               for p in sorted(files)],
        scope='Minima-preserving projection; full geometric banks remain local and are bound by original hashes.'))
    print(json.dumps(dict(cases=len(reports), all_rank_fields_byte_exact=True,
        bytes=sum(p.stat().st_size for p in output.rglob('*') if p.is_file()), truth_accessed=False)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case-directories', type=Path, nargs=3, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run([p.resolve() for p in args.case_directories], args.output.resolve())
