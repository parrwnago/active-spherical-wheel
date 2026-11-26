"""Independent full-endpoint comparison of two sealed isolated bridge fixtures."""
from pathlib import Path
import datetime
import hashlib
import json
import shutil

import numpy as np


HERE = Path(__file__).resolve().parent
FAMILY = HERE.parents[1]
ROOT = FAMILY.parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def main():
    if (HERE/'report.json').exists():
        raise FileExistsError('Preserve previous fixture QA before a new stage')
    protocol_path = FAMILY/'protocol.json'
    clarification_path = FAMILY/'protocol_clarifications.json'
    protocol = read(protocol_path)
    fixture = protocol['bridge_fixture']
    branches, input_files, failures = {}, [protocol_path, clarification_path], []
    for branch in ('batch', 'single'):
        folder = FAMILY/'review'/f'bridge_fixture_stage01_{branch}'
        for name in ('artifact_manifest.json', 'provenance.json', 'report.json', 'bridge_fixture.npz'):
            path = folder/name
            if not path.is_file():
                raise FileNotFoundError('Wait for both sealed bridge endpoints: '+str(path))
            input_files.append(path)
        provenance, report = read(folder/'provenance.json'), read(folder/'report.json')
        for row in read(folder/'artifact_manifest.json')['files']:
            if sha(folder/row['path']) != row['sha256']:
                failures.append(f'{branch}: artifact hash {row["path"]}')
        source_map = {}
        for row in provenance['sources']:
            source_map[row['origin']] = row['sha256']
            if sha(ROOT/row['origin']) != row['sha256'] or sha(folder/row['snapshot']) != row['sha256']:
                failures.append(f'{branch}: source/snapshot {row["origin"]}')
        binary = provenance['bridge_binary']
        if sha(ROOT/binary['path']) != binary['sha256'] or sha(folder/binary['snapshot']) != binary['sha256']:
            failures.append(branch+': plugin binary/snapshot hash')
        with np.load(folder/'bridge_fixture.npz') as saved:
            arrays = {k:saved[k].copy() for k in saved.files}
        branches[branch] = dict(folder=folder, provenance=provenance, report=report, source_map=source_map, arrays=arrays)
    recorded = {str(path.relative_to(ROOT)):sha(path) for path in input_files}
    a, b = branches['batch'], branches['single']
    checks = dict(
        both_complete=all(v['report']['completed'] and v['report']['source_hashes_unchanged'] for v in branches.values()),
        separate_processes=a['provenance']['process_id'] != b['provenance']['process_id'],
        source_maps_identical=a['source_map'] == b['source_map'],
        numerical_versions_identical=a['provenance']['versions'] == b['provenance']['versions'],
        engine_versions_identical=a['provenance']['engine_versions'] == b['provenance']['engine_versions'],
        plugin_hashes_identical=a['provenance']['bridge_binary']['sha256'] == b['provenance']['bridge_binary']['sha256'],
        protocol_bound_by_both=all(v['source_map'][str(protocol_path.relative_to(ROOT))] == sha(protocol_path) for v in branches.values()),
        full_state_fields_identical=np.array_equal(a['arrays']['bridge_state_fields'], b['arrays']['bridge_state_fields']),
        identical_registered_commands=all(np.array_equal(v['arrays']['commands'], fixture['commands']) for v in branches.values()))
    for name, expected in read(clarification_path)['additional_pre_execution_source_sha256'].items():
        checks['registered_source:'+name] = sha(ROOT/name) == expected
    state_a, state_b = a['arrays']['state'], b['arrays']['state']
    state_shape_ok = state_a.shape == state_b.shape and len(state_a) == fixture['held_intervals']+1
    state_gap = float(np.max(abs(state_a-state_b))) if state_shape_ok else None
    checks['all_state_endpoints_within_tolerance'] = bool(state_shape_ok and state_gap <= fixture['acceptance']['state_max_absolute_difference'])
    checks['initial_state_within_tolerance'] = bool(state_shape_ok and np.max(abs(state_a[0]-state_b[0])) <= fixture['acceptance']['state_max_absolute_difference'])
    for branch, value in branches.items():
        fields = list(value['arrays']['bridge_state_fields'])
        states = value['arrays']['state']
        checks[branch+': held_clock_steps'] = bool(np.allclose(np.diff(states[:, fields.index('time')]), .005, atol=1e-10, rtol=0.))
        checks[branch+': exact_iteration_steps'] = bool(np.array_equal(np.diff(states[:, fields.index('iterations')]), np.full(fixture['held_intervals'], 5)))
    contact_gaps = []
    for index in range(fixture['held_intervals']+1):
        contacts = []
        for value in (a, b):
            arrays = value['arrays']; lo, hi = arrays['contact_offsets'][index:index+2]
            rows = arrays['contacts'][lo:hi]
            contacts.append(rows[np.lexsort(rows.T[::-1])] if len(rows) else rows)
        if contacts[0].shape != contacts[1].shape:
            contact_gaps.append(dict(endpoint=index, passed=False, shape_batch=list(contacts[0].shape), shape_single=list(contacts[1].shape)))
        else:
            gap = float(np.max(abs(contacts[0]-contacts[1]))) if len(contacts[0]) else 0.
            contact_gaps.append(dict(endpoint=index, passed=gap <= fixture['acceptance']['contact_max_absolute_difference'], maximum_absolute_gap=gap,
                                     rows=len(contacts[0]), fields=contacts[0].shape[1]))
    checks['sorted_contacts_all_fields_within_tolerance'] = all(row['passed'] for row in contact_gaps)
    checks['all_input_artifacts_unchanged'] = recorded == {str(path.relative_to(ROOT)):sha(path) for path in input_files}
    source = Path(__file__).resolve()
    snapshot = HERE/'source_snapshot'; snapshot.mkdir(exist_ok=False); shutil.copy2(source, snapshot/source.name)
    report = dict(iteration='I-46', passed=not failures and all(checks.values()), checks=checks,
        source_failures=failures, state_maximum_absolute_gap=state_gap, state_shape=list(state_a.shape),
        named_state_fields=a['arrays']['bridge_state_fields'].tolist(), sorted_contact_endpoints=contact_gaps,
        inputs_sha256=recorded, comparator_source_sha256=sha(source),
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='Two isolated fresh-process stepping fixtures only; full raw endpoint state/contact data including initialization. No trajectory/control improvement or hardware claim.',
        tolerances=fixture['acceptance'], physics_or_controller_called_by_comparator=False)
    (HERE/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'], checks=checks, state_maximum_absolute_gap=state_gap), indent=2))
    if not report['passed']:
        raise RuntimeError('Bridge stepping equivalence failed; preserve both branches and this QA')


if __name__ == '__main__':
    main()
