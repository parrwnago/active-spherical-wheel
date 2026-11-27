"""Truth-free I72 copies: original projected features, physical points and N.

No selector, distance calculation, NNLS, new geometry or truth fields are used.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np

from packet_io import PROJECTED_KEYS, decode_rank
from source.i60_native_family_reference import dominant_labels as frozen_native_labels

CASES = ('m100_B', 'p030_B', 'p090_B')
CONTEXT_KEYS = ('time_s', 'exact_estimated_contact_point_m', 'exact_estimated_support_loads_N')


def same_bytes(a, b):
    return a.shape == b.shape and a.dtype == b.dtype and a.tobytes() == b.tobytes()


def native_family(loads):
    """Frozen I60 unique-positive-maximum rule, used only for final fallback."""
    loads = np.asarray(loads, float)
    finite = np.isfinite(loads).all(axis=1) & (loads >= 0).all(axis=1)
    clean = np.where(np.isfinite(loads), loads, 0.)
    maximum, total = clean.max(axis=1), clean.sum(axis=1)
    unique = (clean == maximum[:, None]).sum(axis=1) == 1
    known = finite & (total >= np.finfo(float).tiny) & (maximum > 0) & unique
    return np.where(known, np.argmax(clean, axis=1), -1).astype(np.int64)


def project(projected, points, context, layout):
    valid = np.asarray(projected['valid'])
    if valid.dtype != np.dtype(bool) or valid.ndim != 2: raise ValueError('original boolean valid mask required')
    n, c = valid.shape
    if np.shape(points) != (n, c, 3): raise ValueError('physical point bank shape differs')
    ids = np.asarray(layout['candidate_index'])
    if ids.shape != (c,) or not np.issubdtype(ids.dtype, np.integer) or len(np.unique(ids)) != c:
        raise ValueError('original unique ID-to-slot mapping required')
    if not np.array_equal(projected['family_index'], layout['family_index']) or not np.array_equal(projected['kind'], layout['kind']):
        raise ValueError('original family/kind slot ordering differs')
    if np.shape(context['time_s']) != (n,) or np.shape(context['exact_estimated_contact_point_m']) != (n, 3) or np.shape(context['exact_estimated_support_loads_N']) != (n, 4):
        raise ValueError('native N context shape differs')
    index = np.flatnonzero(valid.reshape(-1))
    values = np.array(points.reshape(-1, 3)[index], copy=True)
    if not np.isfinite(values).all():
        raise ValueError('missing physical coordinate for an original valid candidate; never drop it')
    rank = {'projected__'+key: np.array(projected[key], copy=True) for key in PROJECTED_KEYS}
    rank.update(time_s=np.array(context['time_s'], copy=True), candidate_ids=np.array(ids, copy=True),
        native_anchor_point_m=np.array(context['exact_estimated_contact_point_m'], copy=True),
        point_shape=np.array([n, c, 3], np.int64), point_flat_index=index, point_values_m=values)
    fallback = dict(time_s=np.array(context['time_s'], copy=True),
        native_point_m=np.array(context['exact_estimated_contact_point_m'], copy=True),
        native_support_loads_N=np.array(context['exact_estimated_support_loads_N'], copy=True),
        native_family=native_family(context['exact_estimated_support_loads_N']))
    reference_family = frozen_native_labels(context['exact_estimated_support_loads_N'], np.finfo(float).tiny, 0.)[0]
    if not np.array_equal(fallback['native_family'], reference_family):
        raise ValueError('native fallback labels differ from frozen I60 estimated-load semantics')
    decoded = decode_rank(rank)
    if any(not same_bytes(decoded['projected_features'][key], np.asarray(projected[key])) for key in PROJECTED_KEYS):
        raise ValueError('original projected feature bytes changed')
    if not same_bytes(decoded['world_points'].reshape(-1, 3)[index], points.reshape(-1, 3)[index]):
        raise ValueError('original valid physical point bytes changed')
    if not same_bytes(decoded['native_anchor_point_m'], context['exact_estimated_contact_point_m']):
        raise ValueError('native N anchor was transformed')
    if not same_bytes(fallback['native_support_loads_N'], context['exact_estimated_support_loads_N']):
        raise ValueError('native fallback loads changed')
    parity = dict(frames=n, candidate_slots=c, original_valid_point_count=len(index),
        projected_arrays_byte_exact=list(PROJECTED_KEYS), i70_cached_original_valid_point_coordinates_byte_exact=True,
        direct_native_anchor_byte_exact=True, fallback_point_and_loads_byte_exact=True,
        invalid_coordinates_decoded_nan=True, masks_and_ids_unmodified=True,
        fallback_family_exact_frozen_i60_rule=True,
        no_distance_selection=True, no_geometry_generation=True, no_selector_or_nnls=True)
    return rank, fallback, parity


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path, fields):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key] for key in fields}


def main():
    p = argparse.ArgumentParser(); p.add_argument('--i60-root', type=Path, required=True)
    p.add_argument('--i70-root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--source-input-seal', type=Path, required=True); args = p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    seal = json.loads(args.source_input_seal.read_text())
    for path, expected in seal['sha256'].items():
        if sha(path) != expected: raise ValueError('source/input seal changed: '+path)
    helper = Path(__file__).with_name('source')/'bank_points.py'
    spec = importlib.util.spec_from_file_location('i72_frozen_layout', helper)
    bank = importlib.util.module_from_spec(spec); spec.loader.exec_module(bank)
    layout = bank.candidate_layout()
    if len(layout['candidate_index']) != 102: raise ValueError('expected original102 slots')
    args.output.mkdir(parents=True)
    for case in CASES:
        projected_path = args.i60_root/case/'projected_features.npz'
        context_path = args.i60_root/case/'measurement_context.npz'
        points_path = args.i70_root/case/'features.npz'
        geometry_metadata_path = args.i70_root/case/'metadata.json'
        metadata = json.loads(geometry_metadata_path.read_text())
        # Prove that the cached geometry used the same original measured context
        # and projected feature source, without reading a new state or truth.
        for path in (projected_path, context_path):
            candidates = [value for key, value in metadata['input_sha256'].items()
                          if Path(key).name == path.name and Path(key).parent.name == case]
            if len(candidates) != 1 or candidates[0] != sha(path):
                raise ValueError('cached point bank came from a different original case/source')
        projected = load(projected_path, PROJECTED_KEYS)
        context = load(context_path, CONTEXT_KEYS)
        # Do not read I70's temporal valid mask: I60's original mask is authoritative.
        points = load(points_path, ('points_m',))['points_m']
        rank, fallback, parity = project(projected, points, context, layout)
        out = args.output/case; out.mkdir()
        np.savez_compressed(out/'rank_packet.npz', **rank)
        np.savez_compressed(out/'native_fallback.npz', **fallback)
        record = dict(case=case, source_input_seal_sha256=sha(args.source_input_seal),
            source_files={str(path): sha(path) for path in (projected_path, context_path, points_path, geometry_metadata_path, helper)},
            projected_whitelist=list(PROJECTED_KEYS), context_whitelist=list(CONTEXT_KEYS), point_whitelist=['points_m'],
            forbidden_ranking_inputs=['native_family', 'native_support_loads_N', 'native_fallback'],
            anchor='Direct native observer/locator load-weighted equivalent N; not Nalt or truth; no projection/normalization.',
            fallback_family='Original unique positive maximum of finite nonnegative estimated loads; ties/missing unknown(-1).',
            sparse_scope='Exactly original valid=True coordinates retained; invalid coordinates decodeNaN. No new eligibility gate.',
            point_byte_parity_source='Frozen I70 features.points_m, not unstored original I60 full geometry.',
            historical_i70_geometry_feature_parity=metadata['parity'],
            no_truth_fields_read=True, no_baseline_comparison_read=True, no_new_geometry_or_fits=True, parity=parity)
        (out/'metadata.json').write_text(json.dumps(record, indent=2, sort_keys=True)+'\n')
        manifest = {path.name: dict(bytes=path.stat().st_size, sha256=sha(path)) for path in out.iterdir() if path.is_file()}
        (out/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
        print(json.dumps(dict(case=case, **parity)), flush=True)
    for path, expected in seal['sha256'].items():
        if sha(path) != expected: raise ValueError('source/input changed during projection: '+path)


if __name__ == '__main__': main()
