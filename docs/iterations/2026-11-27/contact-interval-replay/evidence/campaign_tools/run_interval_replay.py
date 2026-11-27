"""I68: unchanged I58 native replay with evaluation-only 1 ms capture.

The imported historical runner deliberately retains its I58 metadata. The
outer I68 protocol and receipts identify this new instrumented reproduction.
No live controller, estimator, plugin or original runner file is edited.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parents[1]
PARENT = ROOT/'results/balance_priority_20261126'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def compare_precision(new_path, old_path):
    with np.load(new_path, allow_pickle=False) as new, np.load(old_path, allow_pickle=False) as old:
        shared = sorted(set(new.files) & set(old.files))
        rows = {}
        for key in shared:
            a, b = new[key], old[key]
            exact = a.shape == b.shape and a.dtype == b.dtype and a.tobytes() == b.tobytes()
            rows[key] = dict(exact=exact, shape_new=list(a.shape), shape_old=list(b.shape),
                dtype_new=str(a.dtype), dtype_old=str(b.dtype))
            if a.shape == b.shape and np.issubdtype(a.dtype, np.number):
                finite = np.isfinite(a) & np.isfinite(b)
                rows[key]['max_finite_abs_gap'] = float(np.max(np.abs(a[finite]-b[finite]))) if finite.any() else None
                rows[key]['same_nan_mask'] = bool(np.array_equal(np.isnan(a), np.isnan(b)))
        return dict(shared_fields=len(shared), all_shared_fields_byte_exact=all(x['exact'] for x in rows.values()),
            new_only=sorted(set(new.files)-set(old.files)), old_only=sorted(set(old.files)-set(new.files)), fields=rows,
            new_sha256=sha(new_path), old_sha256=sha(old_path),
            scope='Every shared precision array over the complete replay; raw bytes, dtype and shape checked, including NaNs.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    if not args.run:
        raise SystemExit('Use --run for this registered new physics experiment')
    protocol = json.loads((HERE/'protocol.json').read_text())
    assert protocol['iteration'] == 'I-68'
    for group in ('source_sha256', 'input_sha256', 'binary_sha256'):
        for relative, expected in protocol[group].items():
            assert sha(ROOT/relative) == expected, (group, relative)
    output = HERE/'campaign/native_substeps'
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(PARENT/'protocol.json', output/'protocol.json')
    parent = load(PARENT/'campaign_tools/run_case.py', '_i68_unchanged_i58_runner')
    collector = load(HERE/'instrumentation/substep_trace.py', '_i68_substep_capture')
    parent.preflight(output, 'm100_B')  # No physics or gain design.
    snap = output/'i68_source_snapshot'
    for group in ('source_sha256', 'input_sha256'):
        for relative in protocol[group]:
            # Large historical precision and cache inputs remain hash-bound local inputs.
            if group == 'input_sha256' and Path(relative).suffix == '.npz':
                continue
            target = snap/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/relative, target)
    shutil.copyfile(HERE/'protocol.json', output/'i68_protocol.json')
    receipt = dict(iteration='I-68', started_utc=datetime.now(timezone.utc).isoformat(),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
        protocol_sha256=sha(HERE/'protocol.json'), inherited_i58_protocol_sha256=sha(output/'protocol.json'),
        process=dict(pid=os.getpid(), python=sys.version, numpy=np.__version__),
        invocation=sys.argv, boundary='Solver contacts are read only for evaluation; native commands and observer updates remain 5 ms.',
        generated_model_snapshot_timing='After native GazeboPlant construction/ground verification/reset, before the native control loop. Not before first setup physics.',
        source_semantics='Imported original I58 runner and its inner I58 metadata are unchanged. This outer receipt identifies a new I68 reproduction.')
    save(output/'i68_provenance_before.json', receipt)
    import spherical_wheel.gazebo.experiments as gx
    original_plant = gx.GazeboWorkspace.plant
    traces, models = [], []
    success = False
    with ExitStack() as stack:
        def plant(workspace, terrain):
            physical = original_plant(workspace, terrain)
            if not traces:
                # The original caller asks for its mirror immediately after plant().
                # Construct that same cached mirror before archiving its XML.
                workspace.mirror()
                trace = collector.SubstepTrace(physical, start_s=0., end_s=44., physics_dt_s=.001)
                stack.enter_context(trace.instrument())
                traces.append(trace)
                paths = dict(physical_mirror_xml=physical.model_path,
                    control_mirror_xml=workspace.directory/'mirror/model.xml',
                    physical_sdf=physical.world_path.parent/'models/active_spherical_wheel/model.sdf',
                    world=physical.world_path, world_info=physical.world_path.parent/'world_info.json')
                for name, path in paths.items():
                    path = Path(path)
                    if not path.is_file():
                        raise FileNotFoundError(path)
                    target = output/'i68_models'/name/path.name
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(path, target)
                    models.append(dict(name=name, origin=str(path.relative_to(ROOT)),
                        snapshot=str(target.relative_to(output)), sha256=sha(path)))
                save(output/'i68_generated_models_before_control.json', dict(timing=receipt['generated_model_snapshot_timing'], files=models))
            return physical
        try:
            with patch.object(gx.GazeboWorkspace, 'plant', plant):
                parent.run_case(output, 'm100_B')
            success = True
        except BaseException:
            save(output/'i68_failure.json', dict(traceback=traceback.format_exc()))
            raise
        finally:
            trace_receipts = [trace.save(output/'substeps.npz') for trace in traces]
            receipt.update(success=success, completed_utc=datetime.now(timezone.utc).isoformat(),
                traces=trace_receipts, models=models)
            save(output/'i68_provenance.json', receipt)
    if not success:
        raise AssertionError('Native runner did not complete')
    comparison = compare_precision(output/'campaign/runs/m100_B/precision.npz',
        PARENT/'campaign/runs/m100_B/precision.npz')
    save(output/'i58_precision_comparison.json', comparison)
    for group in ('source_sha256', 'input_sha256', 'binary_sha256'):
        for relative, expected in protocol[group].items():
            assert sha(ROOT/relative) == expected, (group, relative)
    assert comparison['all_shared_fields_byte_exact'], 'Instrumentation changed native precision history: retain run and diagnose'
    assert not comparison['old_only'] and not comparison['new_only'], 'Historical precision field set changed'
    with np.load(output/'campaign/runs/m100_B/precision.npz', allow_pickle=False) as precision:
        assert len(precision['time_s']) == 8801
    assert traces[0].completed_holds == 8800 and traces[0].failed_holds == 0
    save(output/'i68_artifact_manifest.json', dict(iteration='I-68', files=[dict(
        path=str(path.relative_to(output)), bytes=path.stat().st_size, sha256=sha(path))
        for path in sorted(output.rglob('*')) if path.is_file() and path.name != 'i68_artifact_manifest.json']))
    print(json.dumps(dict(success=True, completed_holds=traces[0].completed_holds,
        precision_fields=comparison['shared_fields'], all_shared_fields_byte_exact=True)), flush=True)


if __name__ == '__main__':
    main()
