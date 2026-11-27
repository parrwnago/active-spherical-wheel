"""Expand lossless cached capture and run unchanged frozen I68 analysis.

Requires Python and NumPy only. It does not start Gazebo, MuJoCo or a controller.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from capture_archive import load_trace, load_precision, manifest, sha
from analyse_interval import analyse


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    registered=manifest(args.package)
    arrays,steps,report=analyse(load_trace(args.package),load_precision(args.package),
        old_precision=load_precision(args.package,old=True),radius_m=.12)
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output/'interval_frames.npz',**arrays)
    np.savez_compressed(args.output/'substep_frames.npz',**steps)
    report['inputs']={registered[key]['path']:registered[key]['sha256']
        for key in ('trace_original','precision_original','old_precision_original')}
    (args.output/'report.json').write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n')
    bindings={p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in args.output.iterdir() if p.is_file()}
    (args.output/'artifact_manifest.json').write_text(json.dumps(bindings,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(intervals=report['intervals'],output=str(args.output))))


if __name__=='__main__':main()
