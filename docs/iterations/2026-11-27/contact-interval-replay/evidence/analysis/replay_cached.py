"""Eager input adapter for the unchanged, sealed I-68 analyse() arithmetic.

The original CLI's lazy NPZ access repeatedly decompressed arrays in the loop.
This adapter changes only file loading and output plumbing.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from analyse_interval import analyse


def load(path):
    with np.load(path, allow_pickle=False) as packet:
        return {key: packet[key] for key in packet.files}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--precision', type=Path, required=True)
    parser.add_argument('--old-precision', type=Path)
    parser.add_argument('--radius-m', type=float, default=.12)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    a, step, report = analyse(load(args.trace), load(args.precision), radius_m=args.radius_m,
                             old_precision=load(args.old_precision) if args.old_precision else None)
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output/'interval_frames.npz', **a)
    np.savez_compressed(args.output/'substep_frames.npz', **step)
    report['inputs'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in (args.trace, args.precision, args.old_precision) if p}
    (args.output/'report.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n')
    manifest = {p.name: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                for p in args.output.iterdir() if p.is_file()}
    (args.output/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
    print(json.dumps(dict(intervals=report['intervals'], valid_primary=report['groups']['full_primary_time_1_to_41']['valid_rows'],
                          primary_comparison=report['groups']['full_primary_time_1_to_41']['comparisons']), allow_nan=False))


if __name__ == '__main__':
    main()
