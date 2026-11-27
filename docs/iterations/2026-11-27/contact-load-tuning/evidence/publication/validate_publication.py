"""Verify I-62 archive hashes, original inputs, unchanged runtime and authored links."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', type=Path, required=True)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    repository, archive = args.repository.resolve(), args.archive.resolve()
    manifest = json.loads((archive / 'portable_manifest.json').read_text())
    archived = []
    for item in manifest['files']:
        path = archive / item['path']
        assert path.is_file() and sha(path) == item['sha256'], item['path']
        assert path.stat().st_size == item['bytes'], item['path']
        archived.append(item['path'])
    original = json.loads((archive / 'protocol/input_source_seal.json').read_text())
    for item in original['files']:
        assert sha(repository / item['path']) == item['sha256'], item['path']
    source = json.loads((archive / 'protocol/pre_execution_source_seal.json').read_text())
    for item in source['files']:
        assert sha(archive / item['snapshot']) == item['sha256'], item['snapshot']
    runtime_paths = ['src', 'scripts', 'tests', 'config', 'GAZEBO.md']
    runtime_diff = subprocess.check_output(['git', 'diff', 'HEAD', '--', *runtime_paths],
                                           cwd=repository, text=True)
    assert not runtime_diff, 'Runtime/config/test source changed'
    authored = [repository / 'docs/iterations/2026-11-27/contact-load-tuning/README.md']
    links = []
    for path in authored:
        for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)', path.read_text()):
            if target.startswith(('http:', 'https:', '#', 'mailto:')):
                continue
            target = target.split('#')[0].strip('<>')
            resolved = path.parent / target
            assert resolved.exists(), str(path) + ': ' + target
            links.append(target)
    report = dict(iteration='I-62', archive_files=len(archived),
                  archive_bytes=sum(x['bytes'] for x in manifest['files']),
                  all_archive_sha256_match=True,
                  original_input_source_hashes_match=len(original['files']),
                  pre_execution_source_snapshots_match=len(source['files']),
                  selected_runtime_config_tests_unchanged=True,
                  authored_record_links_checked=len(links),
                  scope='Archive integrity and evidence links; scientific replay/audits have separate receipts.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
