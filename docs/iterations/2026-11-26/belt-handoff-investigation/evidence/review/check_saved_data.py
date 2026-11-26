"""Independent NumPy-only I-50 arithmetic/protection checks; no simulation."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
FAMILY = ROOT / 'results/belt_handoff_event_20261126'
OLD = ROOT / 'results/belt_capacity_sweep_20261126'

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

checks = []
def check(name, condition, detail=None):
    checks.append(dict(name=name, passed=bool(condition), detail=detail))

out = dict(scope='Independent source/data-only checks; no controller or physics executed',
           preliminary_read_correction='An exploratory read used exact_requested; the archive field is exact_native_requested. It stopped before reductions; this script uses the inspected schema.',
           script_sha256=sha(Path(__file__)), cases={})
terms = ['equilibrium_command', 'rolling_feedforward', 'support_command',
         'lqr_command', 'transition_command_delta', 'heading_command',
         'fade_command_delta']
for arm in ['mars_cap030', 'mars_cap060', 'mars_cap090']:
    path = OLD / 'campaign/runs' / arm / 'precision.npz'
    with np.load(path) as d:
        t = d['time_s']
        raw = d['exact_native_requested']
        applied = d['exact_applied']
        reconstructed = sum(d['exact_' + term] for term in terms)
        reconstructed_lqr = -np.einsum('nij,nj->ni', d['exact_feedback_gain'], d['exact_feedback_error'])
        check(arm + ': command terms sum', np.allclose(raw, reconstructed, atol=1e-10, rtol=0))
        check(arm + ': LQR identity', np.allclose(d['exact_lqr_command'], reconstructed_lqr, atol=1e-10, rtol=0))
        check(arm + ': physical box clipping', np.array_equal(applied, np.clip(raw, -d['exact_input_bounds'], d['exact_input_bounds'])))
        weights = d['exact_sync_weights'][:, [1, 2, 3, 4]]
        loads = d['exact_estimated_support_loads_N']
        shares = np.divide(loads, loads.sum(axis=1, keepdims=True), out=np.zeros_like(loads), where=loads.sum(axis=1, keepdims=True) > 1e-12)
        check(arm + ': preparation disabled at estimated >=5pct share', not np.any((shares >= .05) & (weights > 1e-12)))
        check(arm + ': recorded17-19 has no belt preparation', np.max(d['exact_sync_weights'][(t >= 17-1e-7) & (t <= 19+1e-7), 3:]) == 0)
        err = np.linalg.norm(d['exact_qpos'][:, :2] - d['exact_reference_xy'], axis=1)
        rows = []
        for at in [6.7, 6.825, 6.86, 31.65, 31.655, 32.0, 32.16, 32.165, 32.17, 32.185, 32.19, 39.63, 41, 42.635, 43.985, 44]:
            i = int(np.argmin(abs(t-at)))
            pre = sum(d['exact_' + k][i] for k in terms[:4])
            w = d['exact_sync_weights'][i]
            recovered = np.full(5, np.nan)
            active = w > 1e-9
            recovered[active] = pre[active] + d['exact_transition_command_delta'][i, active]/w[active]
            rows.append(dict(index=i, time_s=float(t[i]), timed_error_m=float(err[i]),
                requested=raw[i].tolist(), applied=applied[i].tolist(),
                requested_step=(raw[i]-raw[max(0, i-1)]).tolist(),
                applied_step=(applied[i]-applied[max(0, i-1)]).tolist(),
                estimated_loads_N=loads[i].tolist(), truth_loads_N=d['truth_patch_loads_N'][i].tolist(),
                sync_weights=w.tolist(), recovered_prep_command=recovered.tolist(),
                drive_reference=d['exact_allocation_drive_reference'][i].tolist(),
                body_angular_velocity_rad_s=d['exact_qvel'][i,3:6].tolist(),
                terms={k:d['exact_'+k][i].tolist() for k in terms},
                lqr_by_state={
                    'position': (-d['exact_feedback_gain'][i,:,:3] @ d['exact_feedback_error'][i,:3]).tolist(),
                    'attitude': (-d['exact_feedback_gain'][i,:,3:6] @ d['exact_feedback_error'][i,3:6]).tolist(),
                    'joint_position': (-d['exact_feedback_gain'][i,:,6:9] @ d['exact_feedback_error'][i,6:9]).tolist(),
                    'linear_velocity': (-d['exact_feedback_gain'][i,:,9:12] @ d['exact_feedback_error'][i,9:12]).tolist(),
                    'body_angular_velocity': (-d['exact_feedback_gain'][i,:,12:15] @ d['exact_feedback_error'][i,12:15]).tolist(),
                    'joint_velocity': (-d['exact_feedback_gain'][i,:,15:18] @ d['exact_feedback_error'][i,15:18]).tolist()}))
        out['cases'][arm] = dict(input_sha256=sha(path), timed_peak_time_s=float(t[err.argmax()]),
            timed_peak_m=float(err.max()), samples=rows)

schedule=json.loads((OLD/'videos/video_display_schedule.json').read_text())
out['video_schedule_schema'] = str(type(schedule))
manifest=json.loads((FAMILY/'pre_change/manifest.json').read_text())
changed=[]
expected_docs={'ITERATION_LOG.md','docs/iterations/2026-11-26/README.md'}
for entry in manifest['sources']:
    path=ROOT/entry['path']
    if sha(path)!=entry['sha256'] and entry['path'] not in expected_docs:
        changed.append(entry['path'])
check('Protected preexisting sources unchanged', not changed, changed)
registration=json.loads((FAMILY/'scope.json').read_text())
changed_inputs=[path for path,digest in registration['inputs_sha256'].items() if sha(ROOT/path)!=digest]
check('Registered I49 inputs unchanged', not changed_inputs, changed_inputs)
out['checks']=checks
out['passed']=all(x['passed'] for x in checks)
(FAMILY/'review/independent_saved_data.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(dict(passed=out['passed'],checks=len(checks),failures=[x for x in checks if not x['passed']])))
if not out['passed']:
    raise SystemExit(1)
