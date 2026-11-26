"""Bounded I49 pure verification; no model construction or dynamics stepping."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import copy
import importlib.util
import json
import sys
import traceback
from types import SimpleNamespace as NS

import numpy as np

HERE = Path(__file__).resolve().parent


def load_runner():
    source = HERE/'run_capacity_case.py'
    spec = importlib.util.spec_from_file_location('i49_capacity_runner_pure_check', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fixture(module):
    config = {'actuation': {'torque_limits_Nm': [1.5, .6, .6], 'belt_speed_limit_m_s': .3}}
    plants = [NS(config=copy.deepcopy(config), bounds=module.BASE_BOUNDS.copy()) for unused in range(3)]
    design = NS(designer=plants[0], points=[dict(phase=.9, K=np.arange(90.).reshape(5, 18),
        eq=dict(qpos=np.zeros(10), qvel=np.zeros(9), u=np.zeros(5)), regime='shell', side='A')], weights='flat')
    return NS(config=copy.deepcopy(config), design=design), plants[1], plants[2]


def require_raises(exception, callback):
    try:
        callback()
    except exception:
        return
    raise AssertionError(f'Expected {exception.__name__}')


def execute(stage):
    if stage.exists():
        raise FileExistsError(f'Preserve earlier verification: {stage}')
    stage.mkdir(parents=True)
    module = load_runner()
    sources = [HERE/'run_capacity_case.py', Path(__file__).resolve(), HERE/'protocol.json', HERE/'campaign/jobs.json']
    captured = []
    for source in sources:
        destination = stage/'source_snapshot'/source.relative_to(module.ROOT)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(source.read_bytes())
        captured.append(dict(origin=str(source.relative_to(module.ROOT)), snapshot=str(destination.relative_to(stage)), sha256=module.sha(source)))
    module.save_json(stage/'before_execution.json', dict(captured_utc=datetime.now(timezone.utc).isoformat(), sources=captured,
        scope='Pure arrays/objects and read-only source/input/cache verification; no model construction or simulator execution.'))
    checks = []
    try:
        for cap in module.CAPS:
            workspace, physical, mirror = fixture(module)
            identity = module.design_identity(workspace.design)
            before = copy.deepcopy(workspace.config)
            original = module.apply_operating_cap(workspace, physical, mirror, cap)
            module.verify_operating_cap(workspace, physical, mirror, cap, original)
            assert module.digest(module.design_identity(workspace.design)) == module.digest(identity)
            assert workspace.config == before
            np.testing.assert_array_equal(physical.bounds[:3], module.BASE_BOUNDS[:3])
            np.testing.assert_array_equal(mirror.bounds[3:], [cap, cap])
        checks.append('All three caps change only separate physical+mirror operating bounds; design/config/numeric K unchanged')

        workspace, physical, mirror = fixture(module)
        mirror.bounds = physical.bounds
        require_raises(AssertionError, lambda: module.apply_operating_cap(workspace, physical, mirror, .6))
        workspace, physical, mirror = fixture(module)
        workspace.design.designer = mirror
        require_raises(AssertionError, lambda: module.apply_operating_cap(workspace, physical, mirror, .6))
        checks.append('Aliased design/physical/mirror objects or bounds rejected')

        for cap in (.45, 1., float('nan'), float('inf')):
            workspace, physical, mirror = fixture(module)
            require_raises(ValueError, lambda: module.apply_operating_cap(workspace, physical, mirror, cap))
        workspace, physical, mirror = fixture(module)
        physical.config['actuation']['belt_speed_limit_m_s'] = .6
        require_raises(AssertionError, lambda: module.apply_operating_cap(workspace, physical, mirror, .6))
        checks.append('Undeclared caps and changed generated-model configuration rejected')

        workspace, physical, mirror = fixture(module)
        module.apply_operating_cap(workspace, physical, mirror, .9)
        mirror.bounds[0] += .01
        require_raises(AssertionError, lambda: module.verify_operating_cap(workspace, physical, mirror, .9))
        checks.append('Torque-bound drift rejected after capacity override')

        ordinary = dict(equilibrium=dict(qpos=np.zeros(10), qvel=np.zeros(9), u=np.zeros(5), phase=.9),
            design=dict(K=np.zeros((5, 18)), A=np.eye(18), B=np.zeros((18, 5)), state_scales=np.ones(18), keep=np.arange(18),
                open_eigenvalues=np.array([.9+.2j]), closed_eigenvalues=np.array([.8-.1j])))
        signature = module.ordinary_identity(ordinary)
        changed = copy.deepcopy(ordinary)
        changed['design']['closed_eigenvalues'][0] += .1j
        assert module.ordinary_identity(changed) == signature
        changed['design']['K'][0, 0] = 1.
        assert module.ordinary_identity(changed) != signature
        changed = copy.deepcopy(ordinary)
        changed['equilibrium']['qpos'][0] = 1.
        assert module.ordinary_identity(changed) != signature
        checks.append('Ordinary real command-state hash accepts actual-style complex diagnostics and detects gain/equilibrium changes')

        protocol, jobs, helper, preflight = module.preflight(HERE)
        for job in jobs:
            assert module.validate_case(job['case_id'], job['cap_m_s'], jobs) == job
        require_raises(ValueError, lambda: module.validate_case('mars_cap030', .6, jobs))
        require_raises(ValueError, lambda: module.validate_case('mars_cap060', .6, jobs+jobs))
        changed = copy.deepcopy(jobs)
        changed[0]['path']['route'] = 'another_route'
        require_raises(ValueError, lambda: module.validate_case('mars_cap030', .3, changed))
        checks.append('Registered source/input/cache/default parameter preflight and fixed case/path validation')
        for row in captured:
            assert module.sha(module.ROOT/row['origin']) == row['sha256']
        report = dict(passed=True, checks=checks, check_count=len(checks), preflight=preflight,
            source_hashes_unchanged_after=True, sources=captured,
            scope='Six independent pure contracts; not repo pytest counts, physics tests or a full integration suite.')
        module.save_json(stage/'report.json', report)
        print(json.dumps(module.safe(report), indent=2))
    except Exception:
        module.save_json(stage/'report.json', dict(passed=False, completed_checks=checks,
            traceback=traceback.format_exc(), sources=captured))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, required=True)
    args = parser.parse_args()
    execute(args.stage.resolve())
