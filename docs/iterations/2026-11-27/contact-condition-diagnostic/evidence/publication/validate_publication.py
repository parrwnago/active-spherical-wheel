"""Check I-63 evidence integrity and source boundaries using local originals."""
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
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    repository, archive, local = args.repository.resolve(), args.archive.resolve(), args.source_root.resolve()
    manifest = json.loads((archive / 'portable_manifest.json').read_text())
    for item in manifest['files']:
        path = archive / item['path']
        assert sha(path) == item['sha256'] and path.stat().st_size == item['bytes'], item['path']
    inputs = json.loads((archive / 'protocol/input_source_seal.json').read_text())
    for item in inputs['files']:
        assert sha(repository / item['path']) == item['sha256'], item['path']
    sources = json.loads((archive / 'protocol/pre_execution_source_seal.json').read_text())
    for item in sources['files']:
        assert sha(archive / item['snapshot']) == item['sha256'], item['snapshot']
        assert sha(local / item['path']) == item['sha256'], item['path']
        assert sha(archive / item['path']) == item['sha256'], item['path']
    registered = json.loads((archive / 'analysis/stage01/registered_before_evaluation.json').read_text())
    assert sha(archive / 'analysis/condition_diagnostic.py') == registered['source_sha256']
    assert sha(archive / 'analysis/i62_metrics.py') == registered['metrics_dependency_sha256']
    assert sha(archive / 'protocol/protocol.json') == registered['protocol_sha256']
    changed = subprocess.check_output(['git', 'diff', 'HEAD', '--', 'src', 'scripts', 'tests', 'config', 'GAZEBO.md'],
                                     cwd=repository, text=True)
    assert not changed, 'Selected runtime/config/test source changed'
    record = repository / 'docs/iterations/2026-11-27/contact-condition-diagnostic/README.md'
    links = []
    for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)', record.read_text()):
        if target.startswith(('https:', 'http:', '#', 'mailto:')):
            continue
        label = target.split('#')[0].strip('<>')
        assert (record.parent / label).exists(), label
        links.append(label)
    report = dict(iteration='I-63', archive_files=len(manifest['files']),
        archive_bytes=sum(x['bytes'] for x in manifest['files']),
        all_archive_hashes_match=True, original_input_asset_hashes_match=len(inputs['files']),
        initial_current_published_scientific_sources_match=len(sources['files']),
        canonical_source_bindings_match=True, runtime_config_tests_unchanged=True,
        authored_record_links_checked=len(links),
        scope='Integrity/source/links using local originals. Feature/evaluation replay and independent bound audit have separate receipts.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
