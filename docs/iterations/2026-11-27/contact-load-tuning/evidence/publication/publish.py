"""Publish compact I-62 evidence without replacing previously copied artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    records, omitted = [], []
    for folder in ('protocol', 'pre_execution_source', 'analysis', 'review',
                   'design_math', 'publication'):
        source = args.source_root / folder
        if not source.is_dir():
            raise FileNotFoundError(source)
        for path in sorted(source.rglob('*')):
            if not path.is_file() or '__pycache__' in path.parts or path.suffix == '.pyc':
                continue
            relative = path.relative_to(args.source_root)
            label = str(relative)
            item = dict(path=label, sha256=sha(path), bytes=path.stat().st_size)
            duplicate_replay_array = path.suffix == '.npz' and (
                label.startswith('analysis/replay_')
                or label.startswith('publication/replay_'))
            if duplicate_replay_array:
                item['reason'] = 'Duplicate replay array; canonical compact inputs and grid/selection arrays are retained.'
                omitted.append(item)
                continue
            destination = args.destination / relative
            if destination.exists() and sha(destination) != item['sha256']:
                raise RuntimeError('Refusing to overwrite historical evidence: ' + label)
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists():
                shutil.copy2(path, destination)
            records.append(item)
    args.destination.mkdir(parents=True, exist_ok=True)
    manifest = dict(iteration='I-62', date_local='2026-11-27',
                    origin_root=str(args.source_root), files=records,
                    local_only_omissions=omitted,
                    scope='Offline shared-threshold grid, episode-grouped fitting/evaluation and compact NumPy replay; no new physics or live policy.')
    (args.destination / 'portable_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(dict(files=len(records), bytes=sum(x['bytes'] for x in records),
                         omitted_arrays=len(omitted))))


if __name__ == '__main__':
    main()
