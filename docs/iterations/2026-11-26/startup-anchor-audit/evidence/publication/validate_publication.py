"""Check the published I-59 archive and authored documentation links."""
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
    for item in manifest['files']:
        path = args.destination/item['path']
        assert path.is_file() and sha(path) == item['sha256'], path
    authored = [Path('ITERATION_LOG.md'), Path('docs/TEAMMATE_HANDOFF.md'),
                Path('docs/iterations/2026-11-26/README.md'),
                Path('docs/iterations/2026-11-26/presentation-preparation/README.md'),
                args.destination.parent/'README.md']
    checked, missing = [], []
    for source in authored:
        if source == args.destination.parent/'README.md':
            text = source.read_text()
        else:
            diff = subprocess.check_output(['git', 'diff', 'HEAD', '--', str(source)], text=True)
            text = '\n'.join(line[1:] for line in diff.splitlines()
                             if line.startswith('+') and not line.startswith('+++'))
        for target in re.findall(r'\]\(([^)]+)\)', text):
            target = target.strip('<>').split('#', 1)[0]
            if not target or '://' in target or target.startswith('mailto:'):
                continue
            path = source.parent/target
            checked.append({'source': str(source), 'target': target})
            if not path.exists():
                missing.append(checked[-1])
    assert not missing, missing
    runtime_diff = subprocess.check_output(
        ['git', 'diff', '--name-only', '--', 'src', 'tests', 'config', 'gazebo'],
        text=True).splitlines()
    assert not runtime_diff, runtime_diff
    protocol = args.destination/'protocol/protocol.json'
    seal = json.loads((args.destination/'protocol/protocol_seal.json').read_text())
    assert sha(protocol) == seal['protocol_sha256']
    report = {'iteration': 'I-59', 'archive_files_verified': len(manifest['files']),
              'archive_bytes_verified': sum(x['bytes'] for x in manifest['files']),
              'local_only_arrays_inventoried': len(manifest['local_only_omissions']),
              'authored_markdown_links_checked': len(checked),
              'link_scope': 'Complete new record and added links in modified documentation',
              'missing_links': missing,
              'runtime_diff': runtime_diff, 'protocol_hash_matches': True,
              'physics_runs': 0, 'all_checks_pass': True}
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
