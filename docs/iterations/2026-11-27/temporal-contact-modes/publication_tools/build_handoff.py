"""Build an explicitly scoped I70 handoff; no private/unrelated workspace files."""
from pathlib import Path
import argparse
from datetime import datetime, timezone
import hashlib
import json
import zipfile

PREFIX = 'i70-contact-estimator/'
EXPERIMENT = Path('docs/iterations/2026-11-27/temporal-contact-modes')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--extract', type=Path, required=True)
    a = p.parse_args()
    root = a.root.resolve()
    if a.output.exists() or a.extract.exists():
        raise FileExistsError('Use new bundle and extraction paths')
    selected = [p for p in (root/EXPERIMENT).rglob('*') if p.is_file()
                and '__pycache__' not in p.parts and p.name != 'publication_qa.json']
    selected += [root/rel for rel in (
        'ITERATION_LOG.md', 'docs/CHATGPT_CONTROL_CONTEXT.md', 'docs/TEAMMATE_HANDOFF.md',
        'docs/iterations/2026-11-27/README.md',
        'docs/iterations/2026-11-26/presentation-preparation/control-and-observer-study-guide.pdf',
        'src/spherical_wheel/shadow_contact_models.py',
        'src/spherical_wheel/temporal_contact_filter.py',
        'tests/test_shadow_contact_models.py', 'tests/test_temporal_contact_filter.py')]
    payload = {str(path.relative_to(root)): path.read_bytes() for path in sorted(set(selected))}
    payload['START_HERE.md'] = (
        '# I-70 contact-estimator handoff\n\n'
        'Start with [the iteration record](docs/iterations/2026-11-27/temporal-contact-modes/README.md), '
        '[four-page PDF](docs/iterations/2026-11-27/temporal-contact-modes/evidence/plots_stage02/temporal_contact_report.pdf) '
        'and [standalone ChatGPT context](docs/CHATGPT_CONTROL_CONTEXT.md).\n\n'
        'This prototype failed six of sixteen adoption gates. Live control is unchanged. '
        'The scientific failure is preserved alongside the numerical checks.\n\n'
        'For complete inference without a simulator, enter '
        '`docs/iterations/2026-11-27/temporal-contact-modes/evidence` and follow the iteration README. '
        'Python 3.10+, NumPy and SciPy are required. Use a new output directory.\n\n'
        'This is a scoped evidence package, not a full installable robot checkout. '
        'Links to older experiments and other controller source files require the full repository. '
        'The standalone context remains readable without those files. '
        'Large duplicate caches/rollouts are omitted with hashes in the export plan.\n\n'
        '`bundle_manifest.json` binds every payload file. Publication QA is external to this bundle '
        'because it records checks made after packaging.\n'
    ).encode()
    manifest = dict(iteration='I-70', registered_utc=datetime.now(timezone.utc).isoformat(),
                    builder_sha256=sha(Path(__file__).read_bytes()),
                    excluded='publication_qa.json (post-packaging receipt); pycache; all unselected workspace files',
                    files={name: dict(bytes=len(data), sha256=sha(data)) for name, data in payload.items()})
    manifest_bytes = (json.dumps(manifest, indent=2)+'\n').encode()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.with_suffix('.registered.json').write_bytes(manifest_bytes)
    with zipfile.ZipFile(a.output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for name, data in list(payload.items())+[('bundle_manifest.json', manifest_bytes)]:
            info = zipfile.ZipInfo(PREFIX+name, date_time=(2026, 11, 27, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data)
    a.extract.mkdir(parents=True)
    with zipfile.ZipFile(a.output) as z:
        assert z.testzip() is None
        for name in z.namelist():
            if not (a.extract/name).resolve().is_relative_to(a.extract.resolve()):
                raise ValueError('Unexpected path outside extraction root')
        z.extractall(a.extract)
    extracted = a.extract/PREFIX
    for name, binding in manifest['files'].items():
        assert sha((extracted/name).read_bytes()) == binding['sha256'], name
    receipt = dict(bundle_path=str(a.output), sha256=sha(a.output.read_bytes()),
                   bytes=a.output.stat().st_size, files=len(payload),
                   crc_check_passed=True, all_extracted_payload_hashes_exact=True,
                   extracted_root=str(extracted), builder_sha256=manifest['builder_sha256'])
    a.output.with_suffix('.receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(json.dumps(receipt))


if __name__ == '__main__':
    main()
