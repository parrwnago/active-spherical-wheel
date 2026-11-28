"""Bind the unchanged I86 B060 diagnostic before new physics."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]
OLD = ROOT / 'results/current_belt_capacity_20261128'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    out = BASE / 'protocol/frozen_stage01'
    if out.exists():
        raise FileExistsError(out)
    prior_path = OLD / 'protocol/frozen_stage01/protocol.json'
    prior = json.loads(prior_path.read_text())
    assert prior['iteration'] == 'I-82'
    sources = dict(prior['source_sha256'])
    inputs = dict(prior['input_sha256'])
    for group in (sources, inputs):
        for name, expected in group.items():
            assert sha(ROOT / name) == expected, name
    own = [Path(__file__).resolve(), BASE / 'protocol/launch.py', BASE / 'protocol/seal.py',
           ROOT / 'results/slope_response_trace_20261128/runner/run_trace.py',
           ROOT / 'results/current_belt_capacity_20261128/runner/run_case.py',
           ROOT / 'results/causal_replay_20261126/substep_trace.py']
    own += sorted((BASE / 'runner').glob('*.py'))
    assert len(list((BASE / 'runner').glob('test*.py'))) > 0
    for path in own:
        sources[str(path.relative_to(ROOT))] = sha(path)
    for path in sorted((BASE / 'runner').glob('*.json')):
        inputs[str(path.relative_to(ROOT))] = sha(path)
    baseline = OLD / 'campaign/runs/B060/child/campaign/runs/eight_m100_N'
    additional = [prior_path, OLD / 'campaign/outputs_sealed.json',
                  baseline / 'precision.npz', baseline / 'rollout.npz',
                  baseline / 'summary.json',
                  OLD / 'campaign/runs/B060/child/campaign/_workspaces/eight_m100_N/mirror/model.xml']
    seal = json.loads((OLD / 'campaign/outputs_sealed.json').read_text())
    for name in ('precision.npz', 'rollout.npz'):
        p = baseline / name
        assert sha(p) == seal['files'][str(p.relative_to(OLD))]['sha256']
    for path in additional:
        inputs[str(path.relative_to(ROOT))] = sha(path)
    settings = json.loads((BASE / 'runner/settings.json').read_text())
    protocol = dict(iteration='I-86', registered_before_physics_utc=datetime.now(timezone.utc).isoformat(),
        source_revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        question='Distinguish the initial physical contact disturbance from native controller recovery at105.53s; assess material simultaneous supports.',
        runner_settings=settings, source_sha256=sources, input_sha256=inputs,
        original_protocol_path=prior.get('original_protocol_path'),
        baseline_protocol_path=str(prior_path.relative_to(ROOT)), baseline_protocol_sha256=sha(prior_path),
        baseline_output_seal_sha256=sha(OLD / 'campaign/outputs_sealed.json'),
        gain_cache=prior['gain_cache'], bridge_binary_sha256=prior['bridge_binary_sha256'],
        exact_physics_binary_sha256=prior['exact_physics_binary_sha256'], parameters=prior['parameters'],
        design_bounds=[1.5, .6, .6, .3, .3], operating_bounds=[1.5, .6, .6, .6, .6],
        controller_intervention=False, default_changed=False,
        fixture='Original I76 eight rough stress; do not relabel as passing moderate-terrain screen.',
        measurement_boundary='Live native control uses exact simulated body state, previous actual commands, momentum observer and known geometry. Contacts/terrain recorded only for offline diagnosis.',
        parity_gate='All79 fields exact B060 prefix through106.5s; final command-held flag false because prefix ends.',
        simulation_scope='One new106.5s reset-to-event prefix, not saved-state restoration or full-route recovery/fix acceptance.',
        analysis_registration='Analysis and independent verifier may be registered separately during physics, but before reading new arrays.',
        limits=['Saved contact force times dt is a sampled impulse estimate, not exact solver impulse export.',
                'The command at a state row acts afterward; prior held commands advanced that state.',
                'Observed force/command timing alone does not establish a controller cure.',
                'Spatial clusters of solver records do not prove independent mechanical constraints.'])
    out.mkdir(parents=True)
    for name, expected in sources.items():
        target = out / 'source' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
        assert sha(target) == expected
    path = out / 'protocol.json'
    path.write_text(json.dumps(protocol, indent=2, sort_keys=True, allow_nan=False) + '\n')
    print(json.dumps(dict(protocol=str(path.relative_to(ROOT)), sha256=sha(path),
                          sources=len(sources), inputs=len(inputs), physics_started=False)))


if __name__ == '__main__':
    main()
