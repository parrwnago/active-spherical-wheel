"""NumPy-only cached I69 analysis; no solver call or physics is executed."""
import argparse
import json
from pathlib import Path
import numpy as np
from packet_archive import load_packets,load_trace,metadata,sha
from analyse_capture import analyse,match_contacts


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    data=metadata(args.package);report,arrays=analyse(load_packets(args.package))
    for label,key in (('new_trace_matches','new'),('historical_trace_matches','historical')):
        report[label]=match_contacts(report,load_trace(args.package,key))
        report[label]['trace_sha256']=data['traces'][key]['original_sha256']
    # These labels identify the original native inputs, whose mathematical
    # values are archived. They do not claim the portable file has those bytes.
    report['capture_sha256']=data['original_capture_sha256']
    report['reader_sha256']=data['original_reader_sha256']
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output/'checks.npz',**arrays)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n')
    manifest={path.name:sha(path) for path in args.output.iterdir()}
    (args.output/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    print(json.dumps(report['checks'],sort_keys=True))


if __name__=='__main__':main()
