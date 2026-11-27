"""Check I-61 archive hashes, authored links and unchanged native sources."""
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
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.destination/'portable_manifest.json').read_text())
    for entry in manifest['files']:
        path = args.destination/entry['path']
        assert path.is_file() and sha(path) == entry['sha256'], path
    old = json.loads((args.destination/'protocol/input_source_seal.json').read_text())
    for entry in old['sources']+old['inputs']:
        path = Path(entry['path'])
        assert sha(path) == entry['sha256'], path
    supplemental = json.loads((args.destination/'protocol/event_input_seal_before_evaluation.json').read_text())['input']
    assert sha(Path(supplemental['path'])) == supplemental['sha256']
    new = json.loads((args.destination/'protocol/pre_execution_source_seal.json').read_text())
    for entry in new['sources']:
        assert sha(Path(entry['path'])) == entry['sha256'], entry['path']
    seal = json.loads((args.destination/'protocol/protocol_seal.json').read_text())
    assert sha(args.destination/'protocol/protocol.json') == seal['protocol_sha256']
    changed_old = subprocess.check_output(
        ['git', 'diff', 'HEAD', '--name-only', '--', 'src', 'tests', 'config', 'gazebo'], text=True).splitlines()
    allowed = {'src/spherical_wheel/contact_load_consistency.py', 'tests/test_contact_load_consistency.py'}
    assert set(changed_old) <= allowed, changed_old
    authored = [Path('ITERATION_LOG.md'), Path('docs/TEAMMATE_HANDOFF.md'),
                Path('docs/iterations/2026-11-27/README.md'), args.destination.parent/'README.md']
    checked, missing = [], []
    for source in authored:
        tracked = subprocess.run(['git', 'ls-files', '--error-unmatch', str(source)],
                                 capture_output=True).returncode == 0
        if tracked and source != args.destination.parent/'README.md':
            diff = subprocess.check_output(['git', 'diff', 'HEAD', '--', str(source)], text=True)
            text = '\n'.join(line[1:] for line in diff.splitlines()
                             if line.startswith('+') and not line.startswith('+++'))
        else:
            text = source.read_text()
        for target in re.findall(r'\]\(([^)]+)\)', text):
            target = target.strip('<>').split('#', 1)[0]
            if not target or '://' in target or target.startswith('mailto:'):
                continue
            checked.append(dict(source=str(source), target=target))
            if not (source.parent/target).exists():
                missing.append(checked[-1])
    assert not missing, missing
    report = dict(iteration='I-61', archive_files_verified=len(manifest['files']),
        archive_bytes_verified=sum(x['bytes'] for x in manifest['files']),
        omitted_local_arrays_inventoried=len(manifest['local_only_omissions']),
        old_sources_and_inputs_unchanged=True, new_sources_match_tested_seal=True,
        protocol_unchanged=True, tracked_runtime_changes=changed_old,
        authored_links_checked=len(checked), missing_links=missing, all_checks_pass=True,
        scope='Added links/full new records; hash-bound local originals required for source validation, not for portable arithmetic replay.')
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
