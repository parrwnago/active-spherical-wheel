#!/usr/bin/env python3
"""Publish sealed I-59 evidence, refusing to overwrite differing artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    folders = ('protocol', 'pre_change_source', 'command_tools', 'analysis',
               'review', 'publication')
    records, omitted = [], []
    for folder in folders:
        source = args.source_root / folder
        if not source.is_dir():
            raise FileNotFoundError(source)
        for path in sorted(source.rglob('*')):
            if not path.is_file() or '__pycache__' in path.parts or path.suffix == '.pyc':
                continue
            relative = path.relative_to(args.source_root)
            label = str(relative)
            heavy_original = path.suffix == '.npz' and (
                label.startswith('analysis/coefficient_stage01/')
                or label.startswith('analysis/pipeline_stage01/')
                or label.startswith('analysis/pipeline_replay_stage01/')
                or label.startswith('review/outcome_stage01/'))
            if heavy_original:
                omitted.append({'path': label, 'sha256': digest(path),
                                'bytes': path.stat().st_size,
                                'reason': 'Local full-stage/duplicate arrays; primary full-rate portable projection is tracked separately.'})
                continue
            destination = args.destination / relative
            if destination.exists() and digest(destination) != digest(path):
                raise RuntimeError(f'Refusing to overwrite historical evidence: {relative}')
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists():
                shutil.copy2(path, destination)
            records.append({'path': str(relative), 'sha256': digest(destination),
                            'bytes': destination.stat().st_size})
    manifest = {'iteration': 'I-59', 'local_date': '2026-11-26',
                'origin_root': str(args.source_root), 'files': records,
                'local_only_omissions': omitted,
                'scope': 'Core coefficient/effect bound replays from tracked compact inputs. Full pipeline/independent raw-log reruns require the listed local originals.'}
    args.destination.mkdir(parents=True, exist_ok=True)
    (args.destination / 'portable_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'files': len(records), 'bytes': sum(x['bytes'] for x in records),
                      'manifest': str(args.destination / 'portable_manifest.json')}))


if __name__ == '__main__':
    main()
