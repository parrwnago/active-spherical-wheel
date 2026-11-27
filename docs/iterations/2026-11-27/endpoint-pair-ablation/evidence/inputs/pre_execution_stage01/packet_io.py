"""Portable NumPy-only loader for frozen I71 inference packets."""
import hashlib
import json
from pathlib import Path
import numpy as np


def load_packet(root, case):
    folder = Path(root)/case
    manifest = json.loads((folder/'artifact_manifest.json').read_text())
    for name in ('inference_packet.npz', 'metadata.json'):
        path = folder/name
        if (path.stat().st_size != manifest[name]['bytes']
                or hashlib.sha256(path.read_bytes()).hexdigest() != manifest[name]['sha256']):
            raise ValueError('packet file binding mismatch: '+name)
    with np.load(folder/'inference_packet.npz', allow_pickle=False) as packet:
        return {key: packet[key] for key in packet.files}
