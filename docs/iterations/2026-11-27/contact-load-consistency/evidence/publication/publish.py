"""Copy sealed compact I-61 evidence, refusing to overwrite changed artifacts."""
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
    folders = ('protocol', 'pre_change_source', 'pre_execution_source', 'core',
               'campaign_tools', 'campaign', 'analysis', 'review', 'portable_tools',
               'publication')
    records, omitted = [], []
    for folder in folders:
        source = args.source_root/folder
        if not source.is_dir():
            raise FileNotFoundError(source)
        for path in sorted(source.rglob('*')):
            if not path.is_file() or '__pycache__' in path.parts or path.suffix == '.pyc':
                continue
            relative = path.relative_to(args.source_root)
            label = str(relative)
            heavy_original = path.suffix == '.npz' and (
                label.startswith('campaign/features_stage01/')
                or label.startswith('analysis/stage01/')
                or label.startswith('analysis/replay_stage')
                or label.startswith('publication/published_evaluation_replay/'))
            item = dict(path=label, sha256=sha(path), bytes=path.stat().st_size)
            if heavy_original:
                item['reason'] = 'Local full/duplicate arrays; frozen gate/policy methods replay from compact load/evaluation projections.'
                omitted.append(item)
                continue
            destination = args.destination/relative
            if destination.exists() and sha(destination) != item['sha256']:
                raise RuntimeError('Refusing to overwrite historical evidence: '+label)
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists():
                shutil.copy2(path, destination)
            records.append(item)
    args.destination.mkdir(parents=True, exist_ok=True)
    manifest = dict(iteration='I-61', local_date='2026-11-27', origin_root=str(args.source_root),
        files=records, local_only_omissions=omitted,
        scope='Compact full-rate stored-load-feature, veto/policy and evaluation replay; geometric Jacobian regeneration requires listed local originals/model assets.')
    (args.destination/'portable_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(files=len(records), bytes=sum(x['bytes'] for x in records),
                         omitted_arrays=len(omitted))))


if __name__ == '__main__':
    main()
