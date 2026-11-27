"""Independent saved native-stage controls only; no anchor differences."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from native_pipeline import infer_fade, native_stages, propagate_fixed_state

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PROTOCOL = HERE.parent/'protocol/protocol.json'
RUNS = ROOT/'results/balance_priority_20261126/campaign/runs'
FIELDS = ('exact_feedback_gain', 'exact_error', 'exact_equilibrium_command',
    'exact_rolling_feedforward', 'exact_support_command', 'exact_lqr_command',
    'exact_transition_command_delta', 'exact_heading_command', 'exact_fade_command_delta',
    'exact_sync_weights', 'exact_native_requested', 'exact_native_applied',
    'exact_native_clipping_command_delta', 'exact_input_bounds', 'exact_qvel',
    'exact_reference_qvel', 'time_s')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def exact(a, b):
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def main():
    output = HERE/'saved_control_stage01'
    output.mkdir(exist_ok=False)
    protocol = json.loads(PROTOCOL.read_text())
    sources = [Path(__file__), HERE/'native_pipeline.py', HERE/'SOURCE_MAPPING.md',
               HERE/'SOURCE_MAPPING_STAGE02_ERRATUM.md', PROTOCOL]
    inputs = sources.copy()
    for case in protocol['datasets']:
        inputs.extend(RUNS/case/name for name in ('precision.npz', 'rollout.npz', 'provenance.json'))
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in inputs}
    (output/'registered_before_arithmetic.json').write_text(json.dumps(dict(
        registered_utc=datetime.now(timezone.utc).isoformat(), source_input_sha256=hashes,
        scope='Independent native LQR/stage/clipping/fade/zero-intervention controls on all six recorded traces. No gain-anchor comparisons or alternate commands.',
        closure_atol=protocol['numerical_closure_max_abs'], no_new_physics=True), indent=2)+'\n')
    snapshot = output/'source_snapshot'
    snapshot.mkdir()
    for p in sources:
        shutil.copy2(p, snapshot/p.name)
    report = dict(iteration='I-59', scope='Native-stage controls only; no anchor coefficient differences computed',
        cases={}, no_simulator_controller_observer_or_lookup=True)
    arrays = {}
    for case in protocol['datasets']:
        folder = RUNS/case
        with np.load(folder/'precision.npz', allow_pickle=False) as saved:
            data = {key: saved[key].copy() for key in FIELDS}
        with np.load(folder/'rollout.npz', allow_pickle=False) as saved:
            fade_recorded, time_recorded = saved['drive_fade'].copy(), saved['time_s'].copy()
        assert fade_recorded.dtype == np.float32
        np.testing.assert_array_equal(data['time_s'].astype(time_recorded.dtype), time_recorded)
        provenance = json.loads((folder/'provenance.json').read_text())
        damping = provenance['parameters']['airborne_damping_Nms']
        count = len(data['time_s'])
        closure_rows, fade_rows = [], []
        for index in range(count):
            row = {key: values[index] for key, values in data.items()}
            stages = native_stages(row, atol=protocol['numerical_closure_max_abs'])
            fade = infer_fade(stages, row['exact_qvel'], row['exact_reference_qvel'],
                damping, fade_recorded[index], closure_atol=protocol['numerical_closure_max_abs'])
            identity = propagate_fixed_state(stages, np.zeros(5), fade)
            assert exact(identity['native_request'], row['exact_native_requested'])
            assert exact(identity['native_clipped'], row['exact_native_applied'])
            assert np.all(identity['request_lower'] == identity['request_upper'])
            closure_rows.append(list(stages['residuals'].values()))
            fade_rows.append([fade[key] for key in ('estimate', 'lower', 'upper', 'float32lo', 'float32hi',
                                                  'closure_residual', 'identifiable')])
        closure_rows, fade_rows = np.asarray(closure_rows), np.asarray(fade_rows)
        arrays[case+'_time_s'] = data['time_s']
        arrays[case+'_stage_closure'] = closure_rows
        arrays[case+'_fade'] = fade_rows
        report['cases'][case] = dict(rows=count,
            max_abs_stage_closure=float(closure_rows.max()),
            max_abs_fade_closure=float(fade_rows[:, 5].max()),
            fade_identifiable_rows=int(fade_rows[:, 6].sum()),
            fade_ambiguous_rows=int(count-fade_rows[:, 6].sum()),
            max_fade_interval_width=float(np.max(fade_rows[:, 2]-fade_rows[:, 1])),
            float32_factor_consistency=True, zero_intervention_bitwise_request_and_applied=True,
            native_clipping_field_used='exact_native_clipping_command_delta',
            boundary='actual native pipeline' if case.endswith('_B') else 'native shadow at observed P state')
        print(json.dumps(dict(case=case, **report['cases'][case])), flush=True)
    assert hashes == {str(p.relative_to(ROOT)): sha(p) for p in inputs}
    np.savez_compressed(output/'controls.npz', **arrays)
    report['all_controls_passed'] = True
    report['input_hashes_unchanged'] = True
    report['fade_columns'] = ['estimate', 'lower', 'upper', 'float32lo', 'float32hi', 'closure_residual', 'identifiable']
    report['stage_closure_columns'] = ['lqr', 'requested', 'applied', 'clipping', 'shaft_sync_delta', 'belt_fade_delta']
    report['limitations'] = ['No alternate anchor coefficients, counterfactual trajectories or P reoptimization evaluated here.',
        'Fade estimates always retain feasible factor intervals; identifiable does not declare telemetry exact.']
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (output/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=str(p.relative_to(output)), sha256=sha(p), bytes=p.stat().st_size) for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json']), indent=2)+'\n')


if __name__ == '__main__':
    main()
