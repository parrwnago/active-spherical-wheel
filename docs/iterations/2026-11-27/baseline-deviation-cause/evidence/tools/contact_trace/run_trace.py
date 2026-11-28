"""I77 native eight: record actual 1 ms contacts around the earlier disturbance.

Reuses the unchanged I76 runner and I55 recorder. No controller intervention.
The nominal route is 133 s; this is a 97.5 s diagnostic prefix only.
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
OLD = ROOT / 'results/tangent_heading_routes_20261127'
HELPER = ROOT / 'results/causal_replay_20261126/substep_trace.py'
CASE = 'eight_m100_N'
HORIZON = 97.5
WINDOW = (96.0, 97.5)

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def write(p, value):
    with Path(p).open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')

def module(p, name):
    spec = importlib.util.spec_from_file_location(name, p)
    result = importlib.util.module_from_spec(spec)
    sys.modules[name] = result
    spec.loader.exec_module(result)
    return result

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--run', action='store_true')
    args = ap.parse_args()
    if not args.run:
        raise SystemExit('Explicit --run required for a new physics prefix')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    protocol = json.loads((OLD/'protocol/protocol.json').read_text())
    if sha(OLD/'protocol/protocol.json') != '53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2':
        raise ValueError('Original protocol changed')
    sources = [Path(__file__).resolve(), HELPER, OLD/'campaign_tools/run_case.py']
    sources += [ROOT/p for p in protocol['source_sha256']]
    for name, expected in {**protocol['source_sha256'], **protocol['input_sha256']}.items():
        if sha(ROOT/name) != expected:
            raise ValueError('Native source/input changed: '+name)
    baseline = OLD/'campaign/runs'/CASE/'precision.npz'
    seal_path = OLD/'campaign/outputs_sealed.json'
    seal = json.loads(seal_path.read_text())
    bound = seal['files'][str(baseline.relative_to(OLD))]
    if sha(baseline) != bound['sha256'] or baseline.stat().st_size != bound['bytes']:
        raise ValueError('Saved native precision differs from final I76 seal')
    bindings = {str(p.relative_to(ROOT)):sha(p) for p in set(sources)}
    inputs = [baseline, seal_path, OLD/'protocol/protocol.json', ROOT/protocol['gain_cache']['path']]
    for p in set(sources):
        dest = out/'source'/p.relative_to(ROOT)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dest)
    write(out/'registered_before_physics.json', dict(iteration='I-77',
        utc=datetime.now(timezone.utc).isoformat(), case_id=CASE,
        actual_requested_horizon_s=HORIZON, nominal_route_horizon_s=133.,
        recording_window_s=list(WINDOW), physics_interval_s=.001, control_interval_s=.005,
        question='Does the early desired-lean motion correspond to actual interval contact wrench/geometry, observer error, or another reference effect?',
        sources=bindings, inputs={str(p.relative_to(ROOT)):sha(p) for p in inputs},
        intervention='None. Only run(5) is split into five run(1) calls inside the recording window; no extra control/observer/mirror updates or state resets.',
        required_validation='Every shared original precision array through 97.5s must match exactly; explicitly validate shortened terminal command-held mask.',
        scope='Native contact timing diagnostic, no solver modification, estimator/controller tuning or full-route acceptance.'))
    child = out/'child'
    for folder, names in dict(adapter=['tangent_heading_adapter.py'],
                              runtime=['route.py','heading_reference.py'],
                              protocol=['protocol.json']).items():
        (child/folder).mkdir(parents=True)
        for name in names:
            shutil.copy2(OLD/folder/name, child/folder/name)
    parent = module(OLD/'campaign_tools/run_case.py', '_i77_contact_native_parent')
    recorder = module(HELPER, '_i77_contact_1ms_helper')
    import spherical_wheel.gazebo.experiments as gx
    true_plant, true_run = gx.GazeboWorkspace.plant, gx.run_gazebo
    traces = []
    success = False
    capture_receipt = None
    with ExitStack() as stack:
        def plant(workspace, terrain):
            physical = true_plant(workspace, terrain)
            if not traces:
                trace = recorder.SubstepTrace(physical, start_s=WINDOW[0], end_s=WINDOW[1])
                stack.enter_context(trace.instrument())
                traces.append(trace)
            elif traces[0].plant is not physical:
                raise AssertionError('Unexpected additional physical plant')
            return physical
        def run(workspace, terrain, path, params=None, **kwargs):
            kwargs['duration_s'] = HORIZON
            return true_run(workspace, terrain, path, params, **kwargs)
        try:
            with patch.object(gx.GazeboWorkspace, 'plant', plant), patch.object(gx, 'run_gazebo', run):
                parent.run_case(child, CASE)
            success = True
        except BaseException:
            write(out/'failure.json', dict(traceback=traceback.format_exc()))
            raise
        finally:
            if traces:
                capture_receipt = traces[0].save(out/'substeps.npz')
                write(out/'capture_receipt.json', capture_receipt)
            write(out/'diagnostic_metadata.json', dict(iteration='I-77', success=success,
                nominal_route_horizon_s=133., actual_requested_horizon_s=HORIZON,
                full_route_completion_claim=False, controller_intervention=False,
                inherited_metadata_override='Child I76 iteration/job horizon labels are inherited runner metadata. This I77 diagnostic metadata defines the experiment.'))
    if (capture_receipt is None or capture_receipt['substep_rows'] != 1500
            or capture_receipt['metadata']['completed_holds'] != 300
            or capture_receipt['metadata']['failed_holds'] != 0):
        raise AssertionError('Recorded substeps/holds differ from registered window')
    for relative, expected in bindings.items():
        if sha(ROOT/relative) != expected:
            raise AssertionError('Executed source changed during run: '+relative)
    produced = child/'campaign/runs'/CASE/'precision.npz'
    checks = []
    with np.load(baseline, allow_pickle=False) as old, np.load(produced, allow_pickle=False) as new:
        if set(old.files) != set(new.files):
            raise AssertionError('Precision field set differs')
        n = len(new['time_s'])
        if n != 19501 or abs(float(new['time_s'][-1])-HORIZON)>1e-8:
            raise AssertionError('Wrong diagnostic prefix length')
        for key in old.files:
            expected = old[key][:n].copy()
            if key == 'command_held_after_row':
                expected[-1] = False
            actual = new[key]
            okay = (actual.dtype==expected.dtype and actual.shape==expected.shape
                    and actual.tobytes()==expected.tobytes())
            checks.append(dict(field=key, bit_exact=okay,
                shortened_terminal_mask_exception=(key=='command_held_after_row')))
        if not all(check['bit_exact'] for check in checks):
            write(out/'parity_failure.json', dict(checks=checks))
            raise AssertionError('Instrumentation changed native evolution')
    write(out/'parity_receipt.json', dict(iteration='I-77', passed=True,
        prefix_rows=19501, actual_end_s=HORIZON, fields=checks,
        measured_scope='All shared precision fields for this prefix, not full 133s reproduction or an independent solver-state dump.',
        new_controller_intervention=False))
    write(out/'artifact_manifest.json', dict(files={str(p.relative_to(out)):
        dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(out.rglob('*'))
        if p.is_file() and '__pycache__' not in p.parts}))
    print(json.dumps(dict(passed=True, prefix_rows=19501, substeps=capture_receipt['substep_rows'],
                         full_route_completion_claim=False)), flush=True)

if __name__=='__main__':
    main()
