# I-56 campaign runner

`run_case.py` is a new standalone experiment runner. It uses the current
native controller once, then calls an explicitly registered command adapter
for the candidate. It does not weaken or rewrite an old I-52 preflight.
No native execution has been launched by the harness author.

The seed gain cache must remain byte-identical to SHA-256
`45ab9990aa3961d1202533acf51d5d8a9294eefe5a4c7e6cfde6aa0b3ac462c1`.
Its metadata must report `cache_loaded=True`. The existing gain fingerprint
covers `control.py`, `model.py`, `sim.py`, `contact.py`, and
`recovery_control.py`; adding new handoff modules does not itself invalidate
that fingerprint. Every run still verifies the seed bytes and numeric design
identity before/after execution.

## Required protocol

The root-owned `results/constrained_handoff_20261126/protocol.json` must contain:

- `iteration: "I-56"`.
- `cases`: unique registered jobs with `case_id`, `variant` (`native` or
  `handoff`), `terrain: "marsyard2021"`, `phase_rad` (−1, +0.3, +0.9),
  `yaw_deg: 45`, `duration_s: 44`, and
  `path: {"kind": "mars_route", "route": "plain_figure8"}`.
- `source_sha256`: the current tested source map, including this runner, the
  adapter and new handoff model/optimizer modules. No old source expectations
  are silently amended.
- Optional `input_sha256`: original data/model hashes used by the protocol.
- `gain_cache: {"path": "generated/offroad/flat/control_schedule.npz",
  "sha256": "..."}` and `bridge_binary_sha256`.
- `parameters`: unchanged native observer-mode settings, normally
  `{"mode":"hardware","rolling_allocation":"least_squares",
  "track_contact_point":false}`. No normal-phase or tilt-reference change.
- `candidate_adapter: {"path":"...","sha256":"..."}` and the frozen
  `candidate_settings` dictionary.
- Optional `historical_baseline_paths`: map native case IDs to their original
  I-52 B precision NPZ paths. `require_historical_baseline_exact` defaults true.

The protocol can declare all six matched jobs while the root initially runs
only the −1-rad pair. Its acceptance gates and conditional campaign expansion
must be registered before native results are read. R's rejected joint-reference
guard is absent from the primary native/candidate comparison.

## Adapter contract

The source-bound adapter exports:

```python
instance = create(controller, settings)
final_requested, final_applied, diagnostics = apply(
    instance, controller, native_requested, native_applied, info)
```

The adapter owns its solver/history. It may reconstruct exactly the current
observer-derived patch records using
`controller.locator.records(controller.plant.data, *controller.observer.wrench())`.
It must not call the observer again, read physical solver contacts/terrain, or
mutate native reference/observer/plant history. The runner fingerprints those
boundaries around creation/application. Native request, clipping and command
terms are preserved separately. `final_applied` must equal the existing hard
box projection of `final_requested`; a bounded optimizer normally returns the
same vector for both.

`governor_diagnostics.json` records pre/post commands, native saturation and
JSON-safe solver diagnostics. Its payload must not contain opaque model/data
objects. The observer receives actual prior physical commands through the
unchanged runner; the adapter must not write previous-input state itself.

## Commands

After root freezes the protocol and source hashes, activate the Gazebo
environment and use one fresh process per case:

```bash
python results/constrained_handoff_20261126/campaign_tools/run_case.py --preflight
python results/constrained_handoff_20261126/campaign_tools/run_case.py --run --case-id m100_B
python results/constrained_handoff_20261126/campaign_tools/run_case.py --run --case-id m100_H
```

The IDs above are examples; the actual registered IDs are authoritative.
Existing case directories are refused. A setup/control/analysis failure is
retained separately and must not be replaced with a favorable retry.

The runner saves raw rollouts, full precision, unchanged native terms,
post-governor commands, evaluation-only contact truth, exact source snapshots,
fixed-bank checks and per-case manifests. Legacy rollout `actuator_saturated`
uses the final request; use `exact_native_saturation_mask` to measure native
infeasible demand and candidate diagnostics for QP active bounds. A low final
clipping count alone is not proof of better dynamic behavior.

## Verification completed here

The nine read-only preflight contracts pass in `preflight_checks_stage01/`.
They check unchanged valid inputs, source/adapter/plugin/bank changes, case
registration, and observer-only/no-sim-contact settings. No Gazebo, controller,
model constructor or gain design was run. Actual baseline numerical parity,
optimizer integration and closed-loop acceptance remain the native campaign's
required checks.
