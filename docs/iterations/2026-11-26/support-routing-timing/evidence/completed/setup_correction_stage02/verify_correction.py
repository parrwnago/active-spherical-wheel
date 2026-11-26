"""Pure setup/AST/hash checks; no server, controller or physics execution."""
from pathlib import Path
import ast
import hashlib
import json
import types
from unittest.mock import patch

import numpy as np

STAGE = Path(__file__).resolve().parent
FAMILY = STAGE.parent
ROOT = FAMILY.parents[1]


def main():
    declaration = json.loads((STAGE/'declaration_before_checks.json').read_text())
    for item in declaration['sources_before_checks']:
        assert hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest() == item['sha256']
    copy = STAGE/'audit-support-routing-flat.py'
    module = types.ModuleType('I46_setup_corrected_flat_only')
    module.__file__ = str(copy)
    exec(compile(copy.read_text(), str(copy), 'exec'), module.__dict__)
    jobs = json.loads((FAMILY/'campaign/jobs.json').read_text())['cases']
    flat = next(j for j in jobs if j['case_id'] == 'flat_compact_outback')
    class NoServerWorkspace:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs
    with patch('spherical_wheel.gazebo.experiments.GazeboWorkspace', NoServerWorkspace):
        workspace, terrain = module.build_native_workspace(FAMILY, 'flat_compact_outback', flat)
    assert terrain.is_flat and not terrain.features
    assert workspace.kwargs['ground'] == 'plane'
    for xy in flat['ground_check_points']:
        assert terrain.height(*xy) == 0.
        np.testing.assert_array_equal(terrain.normal_at(*xy), [0., 0., 1.])
    gate = module.verify_bridge_equivalence(FAMILY)
    assert gate['passed'] and gate['required_fixture_inputs'] == 4
    rejected = False
    try:
        module.collect_native(FAMILY, 'mars_compact_outback', json.loads((FAMILY/'protocol.json').read_text()))
    except ValueError as error:
        rejected = 'permits only' in str(error)
    assert rejected
    output = FAMILY/'campaign/runs/flat_compact_outback_setup_corrected'
    assert not output.exists()
    record = dict(passed=True, flat_featureless=True, flat_is_flat=True, zero_height_up_normal=True,
        gazebo_workspace_created=False, mock_workspace_only=True, original_strict_bridge_QA=gate,
        nonflat_collector_rejected_before_stage_or_server=True, fresh_output_path=str(output.relative_to(ROOT)),
        source_contract=declaration, scope='No Gazebo server/controller/dynamics or route execution.')
    (STAGE/'report.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(dict(passed=True,copy_sha256=hashlib.sha256(copy.read_bytes()).hexdigest(),output=str(output))))


if __name__ == '__main__':
    main()
