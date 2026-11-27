"""Read-only I56 registration contracts; no native controller or physics run."""
from pathlib import Path
from datetime import datetime, timezone
from importlib.util import spec_from_file_location, module_from_spec
from unittest.mock import patch
import hashlib
import json
import shutil
import sys
import tempfile

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SPEC = spec_from_file_location('_i56_runner_contracts', HERE/'run_case.py')
RUNNER = module_from_spec(SPEC)
sys.modules[SPEC.name] = RUNNER
SPEC.loader.exec_module(RUNNER)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    root = tmp_path/'fake_repo'
    output = root/'results/new'
    output.mkdir(parents=True)
    seed = root/'generated/offroad/flat/control_schedule.npz'
    seed.parent.mkdir(parents=True)
    shutil.copyfile(ROOT/'generated/offroad/flat/control_schedule.npz', seed)
    native = root/'src/native.py'
    native.parent.mkdir(parents=True)
    native.write_text('# registered source\n')
    adapter = root/'adapter.py'
    adapter.write_text('# registered command adapter\n')
    plugin = root/'bridge.so'
    plugin.write_bytes(b'registered but never loaded fake plugin')
    protocol = dict(iteration='I-56', cases=[dict(case_id='m100_B', variant='native', terrain='marsyard2021',
        phase_rad=-1., yaw_deg=45., duration_s=44., path=dict(kind='mars_route', route='plain_figure8'))],
        source_sha256={'src/native.py': digest(native)}, gain_cache=dict(path=str(seed.relative_to(root)), sha256=digest(seed)),
        bridge_binary_sha256=digest(plugin), parameters=dict(mode='hardware', rolling_allocation='least_squares', track_contact_point=False),
        candidate_adapter=dict(path='adapter.py', sha256=digest(adapter)), candidate_settings={})
    (output/'protocol.json').write_text(json.dumps(protocol))
    monkeypatch.setattr(RUNNER, 'ROOT', root)
    monkeypatch.setattr('spherical_wheel.gazebo.bridge.plugin_path', lambda: plugin)
    # These are constructors that read-only preflight must never enter.
    monkeypatch.setattr('spherical_wheel.offroad_control.FlatDesign.__init__', lambda *a, **k: (_ for _ in ()).throw(AssertionError('No gain design in preflight')))
    monkeypatch.setattr('spherical_wheel.gazebo.experiments.GazeboWorkspace.__init__', lambda *a, **k: (_ for _ in ()).throw(AssertionError('No native workspace in preflight')))
    return root, output, protocol


def write_protocol(output, protocol):
    (output/'protocol.json').write_text(json.dumps(protocol))


def tree_state(root):
    return {str(p.relative_to(root)): (p.stat().st_size, p.stat().st_mtime_ns, digest(p))
            for p in root.rglob('*') if p.is_file()}


def test_readonly_preflight_validates_without_outputs_or_models(workspace):
    root, output, _ = workspace
    before = tree_state(root)
    protocol, job, audit = RUNNER.preflight(output, 'm100_B')
    assert job['variant'] == 'native'
    assert audit['passed'] and audit['resolved_parameters']['mode'] == 'hardware'
    assert before == tree_state(root)
    assert not (output/'campaign').exists()


def test_changed_registered_source_refused(workspace):
    root, output, _ = workspace
    (root/'src/native.py').write_text('# changed\n')
    with pytest.raises(ValueError, match='source changed'):
        RUNNER.preflight(output)
    assert not (output/'campaign').exists()


def test_changed_adapter_refused(workspace):
    root, output, _ = workspace
    (root/'adapter.py').write_text('# changed adapter\n')
    with pytest.raises(ValueError, match='adapter source changed'):
        RUNNER.preflight(output)


def test_changed_plugin_refused_without_loading(workspace):
    root, output, _ = workspace
    (root/'bridge.so').write_bytes(b'changed')
    with pytest.raises(ValueError, match='bridge binary changed'):
        RUNNER.preflight(output)


def test_wrong_declared_bank_refused_even_if_same_file(workspace):
    _, output, protocol = workspace
    protocol['gain_cache']['sha256'] = '0'*64
    write_protocol(output, protocol)
    with pytest.raises(ValueError, match='bank bytes changed'):
        RUNNER.preflight(output)


def test_missing_case_refused_readonly(workspace):
    root, output, _ = workspace
    before = tree_state(root)
    with pytest.raises(ValueError, match='registered case'):
        RUNNER.preflight(output, 'not_registered')
    assert before == tree_state(root)


def test_duplicate_case_refused(workspace):
    _, output, protocol = workspace
    protocol['cases'].append(protocol['cases'][0].copy())
    write_protocol(output, protocol)
    with pytest.raises(ValueError, match='unique'):
        RUNNER.preflight(output)


def test_simulator_contact_mode_not_allowed(workspace):
    _, output, protocol = workspace
    protocol['parameters']['mode'] = 'terrain_aware'
    write_protocol(output, protocol)
    with pytest.raises(ValueError, match='measurement boundary'):
        RUNNER.preflight(output)


def test_simultaneous_gain_phase_change_not_allowed(workspace):
    _, output, protocol = workspace
    protocol['parameters']['schedule_on_ground_normal'] = True
    write_protocol(output, protocol)
    with pytest.raises(ValueError, match='original flat bank/phase'):
        RUNNER.preflight(output)
