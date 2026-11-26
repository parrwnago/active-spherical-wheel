"""Repeat one preserved I-50 saved-data audit in a fresh directory."""
import argparse
import importlib.util
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['deviation','rolling'],required=True)
    parser.add_argument('--output',type=Path,required=True,
                        help='New directory; existing output is never overwritten')
    args=parser.parse_args()
    family=Path(__file__).resolve().parent
    source=family/({'deviation':'deviation_audit/analyze_deviation.py',
                    'rolling':'sync_audit/audit_rolling_geometry.py'}[args.mode])
    output=args.output.resolve()
    if output.exists():parser.error('Output already exists; choose a fresh directory')
    spec=importlib.util.spec_from_file_location('i50_preserved_audit',source)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output.mkdir(parents=True,exist_ok=False)
    # Preserve the original input root and exact source path. Only the output
    # directory changes; the scripts retain their original arithmetic/guards.
    module.HERE=output
    module.main()

if __name__=='__main__':main()
