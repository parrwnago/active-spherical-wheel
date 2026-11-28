"""Seal closed I86 physical outputs before analysis."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]


def binding(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return dict(bytes=path.stat().st_size, sha256=h.hexdigest())


def main():
    campaign = BASE / 'campaign'
    output = campaign / 'outputs_sealed.json'
    if output.exists():
        raise FileExistsError(output)
    observed = json.loads((campaign / 'native106p5_stage01.observed_exit.json').read_text())
    assert observed['actual_observed_exit_code'] == 0
    protocol_path = BASE / 'protocol/frozen_stage01/protocol.json'
    protocol = json.loads(protocol_path.read_text())
    for key in ('source_sha256', 'input_sha256'):
        for name, expected in protocol[key].items():
            assert binding(ROOT / name)['sha256'] == expected, name
    run = campaign / 'native106p5_stage01'
    parity = json.loads((run / 'parity_receipt.json').read_text())
    assert parity['passed'] and parity['prefix_rows'] == 21301
    assert len(parity['fields']) == 79 and all(row['bit_exact'] for row in parity['fields'])
    capture = json.loads((run / 'capture_receipt.json').read_text())
    assert capture['substep_rows'] == 1300
    assert capture['metadata']['completed_holds'] == 260 and capture['metadata']['failed_holds'] == 0
    manifest = json.loads((run / 'artifact_manifest.json').read_text())['files']
    for name, expected in manifest.items():
        assert binding(run / name) == expected, name
    files = {}
    for p in sorted(campaign.rglob('*')):
        if not p.is_file() or '__pycache__' in p.parts:
            continue
        files[str(p.relative_to(BASE))] = binding(p)
    result = dict(iteration='I-86', utc=datetime.now(timezone.utc).isoformat(),
        relative_base=str(BASE.relative_to(ROOT)), protocol_sha256=binding(protocol_path)['sha256'],
        registered_source_count=len(protocol['source_sha256']), registered_input_count=len(protocol['input_sha256']),
        original_precision_prefix_parity=True, actual_observed_exit_code=0, files=files,
        scope='All closed campaign files except Python cache; one unchanged106.5s diagnostic, not fullroute/fix acceptance.')
    with output.open('x') as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(output.relative_to(ROOT)), files=len(files), sha256=binding(output)['sha256'])))


if __name__ == '__main__':
    main()
