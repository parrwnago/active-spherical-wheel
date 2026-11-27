"""I71 truth-free projection of frozen I70 costs and physical endpoint points.

No NNLS, geometry generation, candidate reranking, labels or baseline decisions.
Only IDs already selected by each original complete fit select its point.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path

import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
EVIDENCE_KEYS = ('node_cost', 'edge_cost', 'node_valid', 'edge_valid',
                 'node_candidate_index', 'edge_candidate_index')
PAIR_SETS = np.array(list(itertools.combinations(range(4), 2)), dtype=np.int64)


def slots_for_ids(selected_ids, layout_ids):
    ids, layout = np.asarray(selected_ids), np.asarray(layout_ids)
    if not np.issubdtype(ids.dtype, np.integer) or not np.issubdtype(layout.dtype, np.integer):
        raise ValueError('candidate IDs must be integers')
    if layout.ndim != 1 or np.any(layout < 0) or len(np.unique(layout)) != len(layout):
        raise ValueError('layout IDs must be unique and nonnegative')
    if np.any(ids < -1): raise ValueError('only -1 is an unknown selected ID')
    mapping = {int(ident): slot for slot, ident in enumerate(layout)}
    slots = np.full(ids.shape, -1, np.int64)
    for ident in np.unique(ids):
        if ident == -1: continue
        if int(ident) not in mapping: raise ValueError('selected ID absent from original bank layout')
        slots[ids == ident] = mapping[int(ident)]
    return slots


def project(time_s, evidence, points, feature_valid, layout):
    time = np.array(time_s, copy=True); points = np.asarray(points)
    valid_points = np.asarray(feature_valid, bool)
    ids, families = np.asarray(layout['candidate_index']), np.asarray(layout['family_index'])
    n, count = len(time), len(ids)
    if points.shape != (n, count, 3) or valid_points.shape != (n, count):
        raise ValueError('original feature point/valid shapes disagree')
    if families.shape != (count,) or not np.isin(families, np.arange(4)).all():
        raise ValueError('original four-family layout required')
    expected = dict(node_cost=(n, 21), edge_cost=(n, 6, 2, 2), node_valid=(n, 21),
                    edge_valid=(n, 6, 2, 2), node_candidate_index=(n, 21, 2),
                    edge_candidate_index=(n, 6, 2, 2, 2))
    for key, shape in expected.items():
        if np.shape(evidence[key]) != shape: raise ValueError(key+' shape mismatch')
    out = {key: np.array(evidence[key], copy=True) for key in EVIDENCE_KEYS}
    out.update(time_s=time, candidate_index=np.array(ids, copy=True),
               family_index=np.array(families, copy=True), pair_sets=PAIR_SETS.copy())
    if 'kind' in layout: out['candidate_kind'] = np.array(layout['kind'], copy=True)
    node_slots = slots_for_ids(out['node_candidate_index'], ids)
    edge_slots = slots_for_ids(out['edge_candidate_index'], ids)
    node_points = np.full((n, 21, 3), np.nan)
    edge_points = np.full((n, 6, 2, 2, 3), np.nan)
    node_endpoint_ids = np.full((n, 21), -1, np.int64)
    edge_endpoint_ids = np.full((n, 6, 2, 2), -1, np.int64)
    node_endpoint_valid = np.zeros((n, 21), bool)
    edge_endpoint_valid = np.zeros((n, 6, 2, 2), bool)

    def gather(slots, mask, family):
        rows = np.flatnonzero(mask)
        selected = slots[rows]
        if np.any(selected < 0): raise ValueError('valid complete fit has unknown point ID')
        if np.any(families[selected] != family): raise ValueError('complete-fit point has wrong endpoint family')
        if not valid_points[rows, selected].all(): raise ValueError('complete-fit point was not an original valid candidate')
        selected_points = points[rows, selected]
        if not np.isfinite(selected_points).all(): raise ValueError('valid complete fit has nonfinite physical point')
        return rows, selected, selected_points

    for mode in range(8):
        mask = out['node_valid'][:, mode].astype(bool)
        rows, selected, selected_points = gather(node_slots[:, mode, 0], mask, mode//2)
        node_points[rows, mode] = selected_points
        node_endpoint_ids[rows, mode] = ids[selected]
        node_endpoint_valid[rows, mode] = True
    for pair, pair_families in enumerate(PAIR_SETS):
        for role, family in enumerate(pair_families):
            for motion in range(2):
                mask = out['edge_valid'][:, pair, role, motion].astype(bool)
                # Validate both members of the original complete fit. Only its
                # current endpoint role is exposed as an endpoint point.
                for member, member_family in enumerate(pair_families):
                    gather(edge_slots[:, pair, role, motion, member], mask, member_family)
                rows, selected, selected_points = gather(edge_slots[:, pair, role, motion, role], mask, family)
                edge_points[rows, pair, role, motion] = selected_points
                edge_endpoint_ids[rows, pair, role, motion] = ids[selected]
                edge_endpoint_valid[rows, pair, role, motion] = True
    out.update(node_endpoint_points_m=node_points, edge_endpoint_points_m=edge_points,
               node_endpoint_candidate_index=node_endpoint_ids, edge_endpoint_candidate_index=edge_endpoint_ids,
               node_endpoint_valid=node_endpoint_valid, edge_endpoint_valid=edge_endpoint_valid)
    for key in EVIDENCE_KEYS:
        if (out[key].dtype != evidence[key].dtype or out[key].shape != evidence[key].shape
                or out[key].tobytes() != evidence[key].tobytes()):
            raise ValueError('frozen model cost/ID/validity bytes changed')
    if np.isfinite(node_points[:, 8:]).any(): raise ValueError('pair/unknown endpoint point was fabricated')
    parity = dict(frames=n, candidate_count=count, original_model_arrays_byte_exact=list(EVIDENCE_KEYS),
                  node_point_lookups=int(node_endpoint_valid.sum()), edge_point_lookups=int(edge_endpoint_valid.sum()),
                  physical_point_selection_exact=True, original_id_layout_validated=True,
                  current_role_uses_same_complete_fit_slot=True, no_new_candidate_selection=True,
                  pair_and_unknown_endpoint_points_all_nan=True)
    return out, parity


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path, keys):
    with np.load(path, allow_pickle=False) as packet:
        return {key: packet[key] for key in keys}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--evidence-root', type=Path, required=True)
    p.add_argument('--features-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--source-input-seal', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    seal = json.loads(args.source_input_seal.read_text())
    for path, digest in seal['sha256'].items():
        if sha(path) != digest: raise ValueError('source/input seal changed: '+path)
    helper = Path(__file__).with_name('source')/'bank_points.py'
    spec = importlib.util.spec_from_file_location('i71_original_layout', helper)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    layout = module.candidate_layout()
    if len(layout['candidate_index']) != 102: raise ValueError('original bank does not have102 slots')
    args.output.mkdir(parents=True)
    for case in CASES:
        epath = args.evidence_root/'inference/stage01'/case/'model_evidence.npz'
        tpath = args.evidence_root/'inference/stage01'/case/'predictions.npz'
        fpath = args.features_root/case/'features.npz'
        ev = load(epath, EVIDENCE_KEYS)
        time = load(tpath, ('time_s',))['time_s']
        features = load(fpath, ('points_m', 'valid'))
        packet, parity = project(time, ev, features['points_m'], features['valid'], layout)
        out = args.output/case; out.mkdir()
        np.savez_compressed(out/'inference_packet.npz', **packet)
        metadata = dict(case=case, source_input_seal_sha256=sha(args.source_input_seal),
            source_files={str(path): sha(path) for path in (epath, tpath, fpath, helper)},
            read_evidence_fields=list(EVIDENCE_KEYS), read_prediction_fields=['time_s'],
            read_feature_fields=['points_m', 'valid'], parity=parity,
            inference_boundary='No contact truth, baseline decisions, reference commands, NNLS or geometry generation.',
            point_convention='Original complete-fit selected candidate; only the current physical endpoint role is exposed. Pair/unknown endpoint point rows stayNaN.')
        (out/'metadata.json').write_text(json.dumps(metadata, indent=2, sort_keys=True)+'\n')
        manifest = {path.name: dict(bytes=path.stat().st_size, sha256=sha(path))
                    for path in out.iterdir() if path.is_file()}
        (out/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
        print(json.dumps(dict(case=case, **parity)), flush=True)
    for path, digest in seal['sha256'].items():
        if sha(path) != digest: raise ValueError('source/input changed during extraction: '+path)


if __name__ == '__main__': main()
