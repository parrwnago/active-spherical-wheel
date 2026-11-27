"""NumPy-only sparse point decoder and separated ranking/fallback input API."""
import hashlib
import json
from pathlib import Path
import numpy as np

PROJECTED_KEYS = ('excitation_m_s', 'family_index', 'frame_valid', 'moment_residual_norm',
    'normal_angle_rad', 'valid', 'velocity_residual_m_s', 'velocity_residual_norm_m_s', 'kind')


def decode_rank(packet):
    features = {key: np.array(packet['projected__'+key], copy=True) for key in PROJECTED_KEYS}
    n, c = features['valid'].shape
    expected = np.flatnonzero(features['valid'].reshape(-1))
    index = np.asarray(packet['point_flat_index'])
    if (not np.issubdtype(index.dtype, np.integer) or not np.array_equal(index, expected)
            or not np.array_equal(packet['point_shape'], [n, c, 3])):
        raise ValueError('sparse coordinates must cover exactly the original valid mask')
    values = np.asarray(packet['point_values_m'])
    if values.shape != (len(index), 3) or not np.isfinite(values).all():
        raise ValueError('finite physical coordinate required for every original valid candidate')
    points = np.full((n, c, 3), np.nan, dtype=values.dtype)
    points.reshape(-1, 3)[index] = values
    ids = np.array(packet['candidate_ids'], copy=True)
    if ids.shape != (c,) or len(np.unique(ids)) != c or not np.issubdtype(ids.dtype, np.integer):
        raise ValueError('explicit unique original candidate IDs required')
    anchor = np.array(packet['native_anchor_point_m'], copy=True)
    if anchor.shape != (n, 3) or np.shape(packet['time_s']) != (n,):
        raise ValueError('anchor/time shape differs from original candidate frames')
    return dict(time_s=np.array(packet['time_s'], copy=True), projected_features=features,
                world_points=points, native_anchor_point_m=anchor, candidate_ids=ids)


def load_case(root, case):
    folder = Path(root)/case
    manifest = json.loads((folder/'artifact_manifest.json').read_text())
    for name in ('rank_packet.npz', 'native_fallback.npz', 'metadata.json'):
        path = folder/name
        if path.stat().st_size != manifest[name]['bytes'] or hashlib.sha256(path.read_bytes()).hexdigest() != manifest[name]['sha256']:
            raise ValueError('input file binding mismatch: '+name)
    with np.load(folder/'rank_packet.npz', allow_pickle=False) as packet:
        result = decode_rank({key: packet[key] for key in packet.files})
    with np.load(folder/'native_fallback.npz', allow_pickle=False) as packet:
        fallback = {key: packet[key] for key in packet.files}
    if not np.array_equal(result['time_s'], fallback['time_s']):
        raise ValueError('ranking/fallback clocks differ')
    result['native_fallback'] = fallback
    return result
