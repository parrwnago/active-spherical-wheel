"""I86 unchanged B060 baseline: 1 ms contact trace of interrupted recovery.

Original I76 native controller and original .30 design; I82's .60 operating
capacity is applied after design construction. Only105.0–106.3s holds are
split into five1ms solves. This diagnostic ends106.5s, not the full133s route.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import hashlib
import importlib.util
import json
import shutil
import sys
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
OLD = ROOT/'results/tangent_heading_routes_20261127'
I82 = ROOT/'results/current_belt_capacity_20261128'
I81_RUNNER = ROOT/'results/slope_response_trace_20261128/runner/run_trace.py'
I82_RUNNER = I82/'runner/run_case.py'
HELPER = ROOT/'results/causal_replay_20261126/substep_trace.py'
ORIGINAL_PROTOCOL_SHA = '53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2'
BASELINE = I82/'campaign/runs/B060/child/campaign/runs/eight_m100_N/precision.npz'
BASELINE_SEAL = I82/'campaign/outputs_sealed.json'
SETTINGS = dict(case_id='eight_m100_N', physical_stop_s=106.5,
    nominal_route_stop_s=133., operating_belt_cap_m_s=.6, design_belt_cap_m_s=.3,
    capture_window_s=[105., 106.3], analysis_window_s=[105.4, 106.1],
    initial_probe_window_s=[105.520, 105.530], physics_dt_s=.001,
    control_dt_s=.005, expected_substeps=1300, expected_holds=260,
    expected_preview_rows=261, expected_precision_rows=21301,
    preview_horizon_s=.12)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    sys.modules[name] = result
    spec.loader.exec_module(result)
    return result


legacy = module(I81_RUNNER, '_i86_immutable_i81_capture')
capacity = module(I82_RUNNER, '_i86_immutable_i82_capacity')
write = legacy.write


def in_capture_window(time_s):
    lo, hi = SETTINGS['capture_window_s']
    return lo-1e-9 <= float(time_s) <= hi+1e-9


def validate_settings(protocol):
    if protocol.get('iteration') != 'I-86' or protocol.get('runner_settings') != SETTINGS:
        raise ValueError('I86 runner settings differ from the registered unchanged-baseline diagnostic')


def preflight(protocol_path):
    protocol = json.loads(Path(protocol_path).read_text())
    validate_settings(protocol)
    for group in ('source_sha256', 'input_sha256'):
        for name, expected in protocol[group].items():
            if sha(ROOT/name) != expected:
                raise ValueError('I86 registered source/input changed: '+name)
    original_path = OLD/'protocol/protocol.json'
    if sha(original_path) != ORIGINAL_PROTOCOL_SHA:
        raise ValueError('Original I76 protocol changed')
    original = json.loads(original_path.read_text())
    for group in ('source_sha256', 'input_sha256'):
        for name, expected in original[group].items():
            if sha(ROOT/name) != expected:
                raise ValueError('Original native source/input changed: '+name)
    for path in (Path(__file__).resolve(), I81_RUNNER, I82_RUNNER,
                 HELPER, OLD/'campaign_tools/run_case.py'):
        if protocol['source_sha256'].get(str(path.relative_to(ROOT))) != sha(path):
            raise ValueError('Canonical runner dependency missing from I86 bindings: '+str(path))
    seal = json.loads(BASELINE_SEAL.read_text())
    bound = seal['files'][str(BASELINE.relative_to(I82))]
    if sha(BASELINE) != bound['sha256'] or BASELINE.stat().st_size != bound['bytes']:
        raise ValueError('Saved B060 baseline differs from its closed I82 seal')
    return protocol, original


def verify_prefix(old_path, new_path):
    checks = []
    with np.load(old_path, allow_pickle=False) as old, np.load(new_path, allow_pickle=False) as new:
        if set(old.files) != set(new.files) or len(old.files) != 79:
            raise AssertionError('All79 original precision fields are required')
        n = len(new['time_s'])
        if n != SETTINGS['expected_precision_rows'] or abs(float(new['time_s'][-1])-SETTINGS['physical_stop_s']) > 1e-8:
            raise AssertionError('Wrong diagnostic prefix length')
        for key in old.files:
            expected = old[key][:n].copy()
            if key == 'command_held_after_row':
                expected[-1] = False
            actual = new[key]
            checks.append(dict(field=key, bit_exact=(actual.dtype == expected.dtype
                and actual.shape == expected.shape and actual.tobytes() == expected.tobytes()),
                shortened_terminal_mask_exception=(key == 'command_held_after_row')))
    return dict(iteration='I-86', passed=all(x['bit_exact'] for x in checks),
        prefix_rows=n, actual_end_s=SETTINGS['physical_stop_s'], fields=checks,
        scope='All79 original I82B060 precision fields through106.5s, final unheld mask explicitly shortened. No full133s replay/completion claim.')


def captured_command(native_command, rows, counts):
    """Evaluate native command/observer/preview once; only copy existing state."""
    def command(controller, xy, velocity, acceleration):
        preview, update = controller.synchronizer.preview, controller.observer.update
        calls = dict(preview=0, observer=0)
        def counted_preview(*args, **kwargs):
            calls['preview'] += 1
            return preview(*args, **kwargs)
        def counted_update(*args, **kwargs):
            calls['observer'] += 1
            return update(*args, **kwargs)
        with patch.object(controller.synchronizer, 'preview', counted_preview), \
                patch.object(controller.observer, 'update', counted_update):
            applied, info = native_command(controller, xy, velocity, acceleration)
        if calls != dict(preview=1, observer=1):
            raise AssertionError('Exactly one native observer update and preview per command required')
        counts['commands'] = counts.get('commands', 0)+1
        counts['previews'] = counts.get('previews', 0)+calls['preview']
        counts['observer_updates'] = counts.get('observer_updates', 0)+calls['observer']
        if in_capture_window(controller.plant.data.time):
            before = legacy.capture_guard(controller, applied, info)
            row = legacy.export_preview_row(controller, xy, velocity, acceleration, applied, info)
            if legacy.capture_guard(controller, applied, info) != before:
                raise AssertionError('Read-only capture mutated native state/history/output')
            row['native_preview_call_count'] = calls['preview']
            row['native_observer_update_count'] = calls['observer']
            rows.append(row)
        return applied, info
    return command


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--protocol', type=Path, required=True)
    ap.add_argument('--output', type=Path)
    ap.add_argument('--preflight-only', action='store_true')
    ap.add_argument('--run', action='store_true')
    args = ap.parse_args()
    protocol, original = preflight(args.protocol.resolve())
    if args.preflight_only:
        print(json.dumps(dict(passed=True, physics_run=False, settings=SETTINGS)), flush=True)
        return
    if not args.run or args.output is None:
        raise SystemExit('New physics requires explicit --run and new --output')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    for name in protocol['source_sha256']:
        dest = out/'source'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, dest)
    shutil.copy2(args.protocol, out/'protocol.json')
    write(out/'registered_before_physics.json', dict(iteration='I-86',
        utc=datetime.now(timezone.utc).isoformat(), settings=SETTINGS,
        protocol_sha256=sha(args.protocol), sources=protocol['source_sha256'], inputs=protocol['input_sha256'],
        intervention='None: unchanged I82B060 controller, observer, design, limits and physics. One observer/preview/command per5ms; splitrun5only105.0–106.3.',
        measurement_boundary='Physical contacts are diagnostic-only; no terrain or solver contact feeds live inferred-contact controller.',
        capture='Raw bridge state before/after1ms solve and last-solve fullcontact rows; new command acts after command-start state.',
        impulse_qualification='Force-times1ms is a sampled integral, not an exact independent solver impulse record.',
        com_qualification='WholeCOM reconstructed offline from rawq/v and saved identical model; no additional live model forwards.',
        required_parity='All79 B060 fields through106.5s, final shortened unheld mask only exception.',
        no_controller_change=True, no_full_route_acceptance=True))
    child = out/'child'
    for folder, names in dict(adapter=['tangent_heading_adapter.py'],
                              runtime=['route.py', 'heading_reference.py'],
                              protocol=['protocol.json']).items():
        (child/folder).mkdir(parents=True)
        for name in names:
            shutil.copy2(OLD/folder/name, child/folder/name)
    parent = module(OLD/'campaign_tools/run_case.py', '_i86_original_native_parent')
    recorder = module(HELPER, '_i86_immutable_substep_helper')
    parent.BOUNDS = capacity.operating_bounds(.6)
    from spherical_wheel.offroad_control import OffroadController
    import spherical_wheel.gazebo.experiments as gx
    true_plant, true_mirror, true_run = gx.GazeboWorkspace.plant, gx.GazeboWorkspace.mirror, gx.run_gazebo
    true_command = OffroadController.command
    state, traces, rows, counts = {'configured': False}, [], [], {}
    success, receipt = False, None
    with ExitStack() as stack:
        def plant(workspace, terrain):
            obj = true_plant(workspace, terrain)
            if 'physical' in state and state['physical'] is not obj:
                raise AssertionError('Unexpected additional physical plant')
            state.update(workspace=workspace, physical=obj)
            if not traces:
                trace = recorder.SubstepTrace(obj, start_s=105., end_s=106.3)
                stack.enter_context(trace.instrument())
                traces.append(trace)
            return obj
        def mirror(workspace):
            obj = true_mirror(workspace)
            if not state['configured']:
                before = parent.design_identity(workspace.design)
                config_sha = capacity.apply_operating_cap(workspace, state['physical'], obj, .6)
                if parent.design_identity(workspace.design) != before:
                    raise AssertionError('Operating cap changed numerical design gains')
                state.update(mirror=obj, config_sha=config_sha, bank_identity=before, configured=True)
                write(out/'capacity_before_controller.json', dict(iteration='I-86',
                    cap_m_s=.6, design_config_belt_cap_m_s=.3, config_sha256=config_sha,
                    bank_numeric_sha256=before, original_designer_bounds=workspace.design.designer.bounds,
                    physical_operating_bounds=state['physical'].bounds, mirror_operating_bounds=obj.bounds,
                    applied_after_design_before_live_controller=True))
            elif obj is not state['mirror']:
                raise AssertionError('Native mirror was replaced')
            capacity.verify_operating_cap(workspace, state['physical'], obj, .6, state['config_sha'])
            return obj
        def run(workspace, terrain, path, params=None, **kwargs):
            kwargs['duration_s'] = SETTINGS['physical_stop_s']
            return true_run(workspace, terrain, path, params, **kwargs)
        try:
            with patch.object(gx.GazeboWorkspace, 'plant', plant), \
                    patch.object(gx.GazeboWorkspace, 'mirror', mirror), \
                    patch.object(gx, 'run_gazebo', run), \
                    patch.object(OffroadController, 'command', captured_command(true_command, rows, counts)):
                parent.run_case(child, SETTINGS['case_id'])
            success = True
        except BaseException:
            write(out/'failure.json', dict(traceback=traceback.format_exc()))
            raise
        finally:
            if traces:
                receipt = traces[0].save(out/'substeps.npz')
                write(out/'capture_receipt.json', receipt)
            write(out/'preview_rows.json', dict(schema_version=1, iteration='I-86',
                timing='Copies after native command, before actuation; returnedcommand actsnext. No preview recomputation.',
                nonfinite_values='null', rows=rows))
            write(out/'diagnostic_metadata.json', dict(iteration='I-86', native_run_returned=success,
                settings=SETTINGS, full_route_completion_claim=False, controller_intervention=False,
                command_preview_counts=counts, parity_validation_pending=True,
                inherited_metadata_override='Child I76 route labels/cap metadata are inherited; I86 specifies106.5sprefix,.60operatingcapand105.0–106.3capture.'))
    if receipt is None or receipt['substep_rows'] != SETTINGS['expected_substeps'] or receipt['metadata']['completed_holds'] != SETTINGS['expected_holds'] or receipt['metadata']['failed_holds'] != 0:
        raise AssertionError('Wrong physical substep/hold capture')
    if len(rows) != SETTINGS['expected_preview_rows'] or not all(row['capture_state_guard_passed'] for row in rows):
        raise AssertionError('Wrong read-only preview capture count')
    n = SETTINGS['expected_precision_rows']
    if counts != dict(commands=n, previews=n, observer_updates=n):
        raise AssertionError('Wrong native command/observer/preview count')
    np.testing.assert_allclose(np.asarray([row['time_s'] for row in rows]), np.arange(21000, 21261)*.005, atol=1e-8, rtol=0)
    parity = verify_prefix(BASELINE, child/'campaign/runs'/SETTINGS['case_id']/'precision.npz')
    write(out/'parity_receipt.json', parity)
    if not parity['passed']:
        raise AssertionError('Instrumentation changed B060 native evolution')
    capacity.verify_operating_cap(state['workspace'], state['physical'], state['mirror'], .6, state['config_sha'])
    if parent.design_identity(state['workspace'].design) != state['bank_identity']:
        raise AssertionError('Numerical bank changed during trace')
    preflight(args.protocol.resolve())
    write(out/'outcome.json', dict(iteration='I-86', completed=True, parity_passed=True,
        actual_end_s=SETTINGS['physical_stop_s'], precision_rows=n, physical_failure=None,
        controller_intervention=False, full_route_completion_claim=False,
        source_and_input_bindings_unchanged=True))
    write(out/'artifact_manifest.json', dict(files={str(p.relative_to(out)):
        dict(bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(out.rglob('*'))
        if p.is_file() and '__pycache__' not in p.parts}))
    print(json.dumps(dict(passed=True, prefix_rows=n, fields=len(parity['fields']),
        substeps=SETTINGS['expected_substeps'], command_rows=len(rows), no_controller_intervention=True)), flush=True)


if __name__ == '__main__':
    main()
