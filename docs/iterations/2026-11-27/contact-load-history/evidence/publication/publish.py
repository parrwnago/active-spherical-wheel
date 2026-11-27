"""Publish I-64 cached inputs and bound replay evidence without duplicates."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def omission_reason(label, path):
    if path.suffix != '.npz':
        return None
    if label.startswith('features/stage01/'):
        return 'Original full feature arrays stay local; portable_stage01 retains cached primitives and exact derived-array bindings.'
    if label.startswith('analysis/stage01/') and path.name.endswith('_frames.npz'):
        return 'Full frame table regenerates from cached inputs; exact per-array bindings retained. Event windows are copied.'
    if label.startswith(('features/replay_', 'analysis/replay_', 'publication/replay_')):
        return 'Duplicate replay array; original cached inputs, bindings and comparison receipts retained.'
    if label.startswith('review/results_stage01/'):
        return 'Independent full-rate duplicate output; reference source, comparison report and original hash retained.'
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    records, omitted = [], []
    for folder in ('protocol', 'pre_execution_source', 'features', 'analysis',
                   'review', 'portable_inputs', 'publication'):
        source = args.source_root / folder
        if not source.is_dir():
            raise FileNotFoundError(source)
        for path in sorted(source.rglob('*')):
            if not path.is_file() or '__pycache__' in path.parts or path.suffix == '.pyc':
                continue
            relative = path.relative_to(args.source_root)
            label = str(relative)
            item = dict(path=label, sha256=sha(path), bytes=path.stat().st_size)
            reason = omission_reason(label, path)
            if reason:
                item['reason'] = reason
                omitted.append(item)
                continue
            destination = args.destination / relative
            if destination.exists() and sha(destination) != item['sha256']:
                raise RuntimeError('Refusing to replace historical evidence: ' + label)
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists():
                shutil.copy2(path, destination)
            records.append(item)
    args.destination.mkdir(parents=True, exist_ok=True)
    manifest = dict(iteration='I-64', date_local='2026-11-27',
        origin_root=str(args.source_root), files=records,
        local_only_omissions=omitted,
        scope='Frozen endpoint maps/raw observer inputs, exact array/decision bindings, model consistency and contact-discrimination diagnostics. No new physics or controller actuation.')
    (args.destination / 'portable_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(dict(files=len(records), bytes=sum(x['bytes'] for x in records),
                         omitted_arrays=len(omitted))))


if __name__ == '__main__':
    main()
