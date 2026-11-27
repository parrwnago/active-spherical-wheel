"""Seal the I57 diagnostic design before native replay or perturbation results."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import subprocess

import numpy as np

ROOT = Path.cwd()
OUT = ROOT/'results/loaded_contact_validation_20261126'
OLD = ROOT/'results/constrained_handoff_20261126'
RUN = OLD/'campaign/runs/m100_H'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative(path):
    return str(Path(path).resolve().relative_to(ROOT))


path = OUT/'protocol.json'
if path.exists():
    raise FileExistsError('Preserve the sealed registration')
parent = json.loads((OLD/'protocol.json').read_text())
provenance = json.loads((RUN/'provenance.json').read_text())
with np.load(RUN/'precision.npz', allow_pickle=False) as z:
    times = z['time_s'].copy()
    native = z['exact_native_applied'].copy()
    governed = z['exact_applied'].copy()
    previous = z['exact_previous_applied'].copy()
caps = np.array([1.5, .6, .6, .3, .3])
slew = np.array([60., 60., 60., 6., 6.])
epsilon = np.array([.05, .02, .02, .005, .005])
events, cases, states = [], [], []
for event_id, index, reason in (
    ('early_shellB', 971, 'First sustained H-minus-B timed error above20mm in priorI56 saved analysis; shellB support.'),
    ('late_beltB', 7185, 'Peak primary modeled balance-response loss in priorI56; beltB support.'),
):
    event = dict(event_id=event_id, index=index, time_s=float(times[index]),
                 selection_reason=reason, selection_known_prior_outcome=True,
                 native_command=native[index].tolist(), governed_command=governed[index].tolist(),
                 previous_command=previous[index].tolist())
    events.append(event)
    perturbations = []

    def add(suffix, mode, delta, role, **extra):
        delta = np.asarray(delta, float)
        center = (native[index] if mode == 'hold_native' else
                  .5*(native[index]+governed[index]) if mode == 'hold_midpoint' else governed[index])
        held = center+delta
        assert np.all(abs(held) <= caps+1e-12)
        row = dict(case_id=event_id+'_'+suffix, event_id=event_id, mode=mode,
                   delta_input=delta.tolist(), role=role, **extra)
        if mode != 'continue_saved':
            row['held_command'] = held.tolist()
            row['first_step_candidate_slew_admissible'] = bool(np.all(abs(held-previous[index]) <= slew*.005+1e-12))
            perturbations.append(dict(id=row['case_id'], delta_input=(held-governed[index]).tolist()))
        cases.append(row)

    add('continue', 'continue_saved', np.zeros(5), 'exact_recorded_policy_control')
    add('native', 'hold_native', np.zeros(5), 'native_center_amplitude_only_counterfactual')
    add('governed', 'hold_governed', np.zeros(5), 'governed_center')
    add('governed_repeat', 'hold_governed', np.zeros(5), 'deterministic_duplicate_control')
    add('midpoint', 'hold_midpoint', np.zeros(5), 'native_governed_midpoint_amplitude_only_counterfactual')
    lower = np.maximum(-caps, previous[index]-slew*.005)
    upper = np.minimum(caps, previous[index]+slew*.005)
    direction = np.where(governed[index] <= .5*(lower+upper), 1., -1.)
    for column in range(5):
        for amplitude, multiplier in [('full', 1.), ('half', .5)]:
            delta = np.zeros(5)
            delta[column] = direction[column]*epsilon[column]*multiplier
            held = governed[index]+delta
            assert np.all(held >= lower-1e-12) and np.all(held <= upper+1e-12)
            add(f'u{column}_{amplitude}', 'hold_governed', delta,
                'governed_center_column_probe', input_column=column,
                amplitude=amplitude, epsilon_multiplier=multiplier)
    states.append(dict(id=event_id, index=index, time_s=float(times[index]), perturbations=perturbations))

sources = [*sorted((ROOT/'src/spherical_wheel').rglob('*.py')),
           *sorted((ROOT/'config').glob('*.yaml')),
           ROOT/'gazebo/plugin/SphericalWheelBridge.cc', Path(__file__).resolve(),
           OUT/'campaign_tools/run_probe.py', OUT/'model_tools/evaluate_frozen_model.py']
assert all(p.exists() for p in sources)
inputs = [RUN/'precision.npz', RUN/'provenance.json', RUN/'governor_diagnostics.json',
          OLD/'protocol.json', *[Path(p) for p in provenance['models'].values()],
          ROOT/parent['gain_cache']['path'], *[ROOT/p for p in parent['input_sha256'] if p.endswith('.so')]]
bridge_path = next(p for p in inputs if p.suffix == '.so' and sha(p) == parent['bridge_binary_sha256'])
protocol = dict(
    iteration='I-57', registered_utc=datetime.now(timezone.utc).isoformat(),
    base_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
    source_data=dict(precision_path=relative(RUN/'precision.npz'),
                     provenance_path=relative(RUN/'provenance.json'), protocol_path=relative(OLD/'protocol.json')),
    input_sha256={relative(p): sha(p) for p in inputs},
    source_sha256={relative(p): sha(p) for p in sources},
    bridge_binary=dict(path=relative(bridge_path), sha256=sha(bridge_path)),
    gain_cache=parent['gain_cache'], events=events, states=states, cases=cases,
    model_sha256={key: sha(Path(value)) for key, value in provenance['models'].items()},
    endpoint_offsets_steps=[0, 1, 2, 3, 4],
    horizons_s=[.005, .02], native_control_interval_s=.005, pulse_duration_s=.02,
    max_fresh_physics_runs=30, fresh_process_per_case=True, serial_execution=True,
    initial_pilots=[e['event_id']+'_continue' for e in events],
    replay_requirement='Bitwise qpos/qvel/time/actualpreviousinput at EVERY5ms prefix state in EVERYbranch; replaycommands[0:index], never restore savedpose.',
    controls='continue_saved must match original every5ms through+20ms; duplicate governed held endpoints/contact rows must exactly reproduce. Failure invalidates event interpretation and is retained.',
    command_scope='No controller or observer updates during openloop prefix or pulse after original initialization; native/midpoint holds are amplitude-feasible diagnostic counterfactuals and may violate candidate firststep slew. Columnprobes stayinside firststep amplitude+slewbox. No silent clipping.',
    measurements='Predictions use measuredstate, actualpriorinput and originalobserverwrench/geometry only. Gazebo endpointcontacts/forces evaluationonly.',
    angular_comparison='Convert each endpoint forkangularqvel to world with its own quaternion, subtractpairedworldvectors, then project with common initialR0.T. Rawbodyaxis subtraction is invalid.',
    model='Exact frozenI56 contactstrength10 and mismatchconfidence .02m/s; separately evaluate Gh at5/20ms, no retuning. Predictions sealedbefore anynative replay.',
    contact_scope='Record5msendpoint supportchanges; these do not certify unchangedmode inside5ms holds. Neverdrop changingcontact failures.',
    screening=dict(
        floors=dict(planar_m_s=2e-4, transverse_rad_s=2e-3, world_yaw_rad_s=2e-3, hinges_rad_s=2e-2),
        relative_vector_error_max=.5, cosine_min=.8, gain_ratio_bounds=[.5, 2.],
        both_below_floor='unresolved; no directional accuracy pass',
        predicted_negligible_observed_resolved='failure',
        transverse_nullspace='When predictedtransversenorm<=floor require observedtransversenorm<=floor; otherwise resolvedvector criteria.',
        full_half_nonlinearity_max=.25,
        nonlinearity_formula='norm(observed_full-2observed_half)/max(norm(observed_full),group_floor)',
        scope='Engineering screening assumptions, not calibrated physical tolerances or statistical significance.'),
    decision='Diagnostic only; no model/controller/default adoption evenifthese30probes pass. Preserve failures and bounds of inference.',
    no_new_physical_outcomes_read=True)
assert len(cases) == 30 and len({x['case_id'] for x in cases}) == 30
path.write_text(json.dumps(protocol, indent=2)+'\n')
print(json.dumps(dict(protocol=str(path.relative_to(ROOT)), sha256=sha(path), cases=len(cases), sources=len(sources))))
