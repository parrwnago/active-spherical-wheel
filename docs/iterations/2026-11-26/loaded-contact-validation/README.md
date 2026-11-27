# I-57 — validate loaded-contact response before changing control

2026-11-26. Diagnostic follow-up to the rejected
[I-56 bounded allocator](../constrained-handoff/README.md). Keep its measured-
contact model byte-identical and keep native LQR/observer/controller defaults
unchanged. Test whether the local response matrix predicts physical input
effects at the states that motivated further investigation.

## What the experiment tests

The model supplies `G_h`: a change in input is expected to change velocity by
`G_h Δu` over horizon `h`, with geometry, inferred contacts and confidence held
at their starting values. This is different from predicting the whole motion:
native actuation, drift, gravity and contact response also affect each branch.
Subtracting two physical branches with the same initial history isolates the
effect of changing their input over the tested interval.

Two previously observed I-56 H states are selected before the new perturbations:

| Anchor | Saved index | Recorded time | Reason |
| --- | ---: | ---: | --- |
| Early shell B | 971 | 4.855 s | Early sustained extra tracking error, with correct sampled support and no active slew limit |
| Late belt B | 7,185 | 35.925 s | Largest modeled first-stage balance-response loss |

These are purposeful case selections from known failures, not random or blinded
terrain samples. Two states cannot establish general off-road model validity.

At each state, 15 fresh Gazebo processes replay the exact I-56 applied commands
from the original initialization. Each checks every 5 ms `q`, `qdot`, clock and
actual prior input byte-for-byte before reaching the anchor. No saved-pose
reset is used to manufacture a branch, and no controller/observer update runs
after initialization. The five logged endpoints are 0, 5, 10, 15 and 20 ms.
All pulses retain the original `run(5)` stepping; no substep batching changes.

The 15 branches are the original changing-command continuation; held native,
held governed and their midpoint; a duplicate governed control; and five input
columns at full/half perturbation. Column steps are
`[.05, .02, .02 N m, .005, .005 m/s]`, directed toward the center of the
initial amplitude-and-slew admissible box. Their exact realized deltas are
registered. Native/midpoint probes remain inside amplitude limits but may
violate I-56's optional first-step slew assumption; they are explicit actuator-
response counterfactuals, not admissible outputs of that constrained policy.

The continuation must reproduce every original post-anchor state through 20 ms.
The duplicate governed branch must reproduce endpoint state and raw contact
rows exactly. If these controls fail, preserve the setup failure and withhold
physical interpretation. Do not quietly retry or relax numerical parity.

## Model and coordinate checks

Before any new physical run, reconstruct the original model from saved simulated
body/joint state, actual prior belt speed and observer wrench through the pure
support locator. Do not substitute the filtered aggregate normal for the fresh
patch normals. Verify agreement with original contact names, loads, rank,
estimated point, slip/confidence, both native/governed modeled impulses and
contact residuals. Seal predictions separately at 5 and 20 ms.

The frozen model's torque columns scale with horizon; its prescribed belt-speed
columns do not. This structural assumption is part of what is being tested,
not a parameter fitted after inspecting the responses. Model friction and
unilateral impulse feasibility are not enforced.

Different branches end with different orientations. Convert each measured
angular velocity to world coordinates using its own endpoint orientation,
subtract in world coordinates, then project into the common initial fork frame
for comparison with model transverse components. Subtracting angular vectors
expressed in different moving body frames is invalid. Translation is already
in world axes; relative encoder rates remain separate scalars.

## Registered screening assumptions

Retain raw vector errors, direction cosine, gain ratio and full/half nonlinearity.
Resolved effects use relative error ≤0.5, cosine ≥0.8 and gain ratio 0.5–2.
Magnitude floors are 0.0002 m/s planar, 0.002 rad/s transverse/yaw and
0.02 rad/s hinge rate. Both effects below floor are unresolved; do not claim
direction accuracy there. Predicted negligible response with resolved measured
response fails. A predicted transverse nullspace is judged against the absolute
measured transverse floor. Full/half nonlinearity is
`||observed_full − 2 observed_half|| / max(||observed_full||, floor)`, screened
at 0.25. These are coarse engineering checks, not calibrated physical limits.

Contact truth is evaluation-only and retained at all five endpoints. It cannot
certify that contact stayed unchanged inside each 5 ms hold. Mode changes remain
in the dataset, including failed predictions; no favorable subset replaces the
registered screen. Passing these tests would not establish controller stability
or justify adopting I-56.

## Completed result

All **30 fresh processes complete**, with no setup or physics failure. Every
prefix is byte-exact, both original continuations match through 20 ms, and both
duplicate governed controls match every endpoint, bridge state, input and raw
contact row. All registered cases remain in the final report. These are 30
prefix-and-pulse diagnostics, not 30 complete 44 s trajectory trials.

The important native-to-governed command differences are predicted accurately:

| 20 ms response | Early shell B | Late belt B |
| --- | ---: | ---: |
| Planar vector error | 1.322% | 1.351% |
| World-yaw response error | 0.091% | 1.988% |
| Transverse angular response | Both below resolution floor | 1.204% error |

At the early state, actual planar change is
`[−.002860729, +.000430672] m/s`, versus predicted
`[−.002823642, +.000421307]`. Removing the contact penalty predicts the opposite
planar direction, with 236.7% error. The early transverse effect is only
0.000203 rad/s, compatible with the predicted nullspace at the 0.002 floor.
This is an absolute quiet-response check, not a resolved direction estimate.
At the late state, predicted transverse change is
`[−.056736, −3.740082] rad/s`, versus observed
`[−.074986, −3.781789]`. Contact information is useful here; it does not make
every component accurate.

The complete loaded-model screen contains **118 passing, five failing and
165 unresolved group scores**. These are correlated output groups across
cases/horizons, not 288 independent robot tests. The five failures are small
late-state 20 ms cross-couplings, retained without changing the floors:

- For either shell-torque full probe, planar response magnitude is
  0.550341 mm/s observed versus 0.299711 predicted. Vector error is 73.26%,
  with cosine 0.69769.
- Their half probes have observed planar magnitude 0.275192 mm/s above the
  0.2 mm/s floor, while prediction 0.149856 lies below it.
- The loaded-belt-B full probe has observed yaw response 0.00226561 rad/s
  above the 0.002 floor, versus prediction 0.00154022 below it.

Full/half response checks give 67 passing and 77 unresolved groups, no failed
linearity checks. All 25 predicted-negligible transverse checks are physically
quiet within the declared floor. All sampled endpoint support modes stay shell
B or belt B as appropriate. No contact onset or transfer is validated here;
multiple contact and within-hold mode changes remain outside this evidence.

## What this establishes about the rejected allocator

At the late anchor, actual prior shaft torque is **+1.197503 N m**, while the
amplitude-clipped native request is **−1.500000 N m**. The added 60 N m/s slew
assumption permits a reduction of only 0.300 N m in one 5 ms period, forcing
the first command to **+0.897503 N m**. Reaching −1.5 under that rate would
require at least 44.96 ms. The model predicts a substantial balance-response
cost, and the paired experiment confirms it physically at this state. A tiny
second-stage equality residual never meant the unavailable native balance
response was delivered.

The native counterfactual is inside amplitude caps but outside that assumed
slew box. This is evidence about the cost of the experimental rate constraint,
not proof that violating a real actuator limit is acceptable. The 20 ms holds
are diagnostic; the original live controller refreshes commands every 5 ms.
The first 5 ms governed endpoint also reproduces the original H action exactly.
These facts do not establish the complete cause of the whole-route deviation.

## Exploratory follow-up: velocity baseline used by the yaw cost

After the paired-response agreement was known, register a separate read-only
analysis with no new acceptance gates. The I-56 secondary cost uses
`v_current + G_h (u_governed − u_native_clipped)`. It includes the effect of
changing the input, but assumes the native future velocity equals the current
one. The paired holds let us measure that omitted native change directly.

At the late anchor over 20 ms, world-yaw rate is:

| Quantity | rad/s |
| --- | ---: |
| Current state | −1.50185 |
| Actual held native endpoint | −0.70502 |
| Actual held governed endpoint | +0.76183 |
| Current-plus-correction proxy | −0.00584 |
| Frozen affine governed prediction | +0.73614 |

The proxy error is +0.76767 rad/s. It decomposes into omitted native future
change +0.79683 and paired response-model error −0.02917. Thus a good
differential response can coexist with a poor absolute baseline in the cost.
The early case shows the same distinction. This identifies another limitation
in the candidate's objective; it is not a causal test of a replacement policy.

A further exploratory audit checks every original H interval, using exact
current state and known actual command afresh at each of 8,800 steps. The
frozen affine `offset + G_5ms u` reduces one-step RMS error relative to constant
velocity: planar 0.019260→0.008616 m/s, transverse 0.112328→0.043244 rad/s,
and world yaw 0.096866→0.008064 rad/s. All model reconstructions succeed.
Tails remain substantial: affine planar peak error is 0.15857 m/s and shell-A
rate error reaches 8.6948 rad/s, worse than the constant baseline's peak there.
Only 92 endpoint patch-mask changes are observed; that does not certify all
within-step transitions. This is one-step analysis with supplied state/input,
not a multi-step forecast or validation of a new control cost. The affine
baseline is not adopted automatically.

## Decision and next isolated experiment

Keep native LQR, the momentum observer and current feedforward selected.
Do not discard the contact model wholesale or call it uniformly validated.
The tested dominant responses are accurate, with explicit weak-coupling limits.
No particle filter, contact forecast or controller default is introduced.

The next controlled candidate should retain the actual amplitude caps and give
balance priority over the **optional experimental command-smoothing bound**.
Test this hierarchy first with auxiliary yaw/momentum penalties disabled, using
the same trajectory comparison and regression gates. Investigate the velocity
baseline separately afterward. Combining both changes immediately would obscure
which one matters. This next controller experiment is proposed, not executed
or claimed successful in I-57.

## Evidence and reproduction

[Main response figure](evidence/analysis/stage01/native_governed_response.pdf)
· [All response scores](evidence/analysis/stage01/response_accuracy.pdf)
· [Yaw-baseline comparison](evidence/review/baseline_proxy_stage01/baseline_yaw_comparison.pdf)
· [One-step error distributions](evidence/review/one_step_affine_stage01/one_step_error_distributions.pdf).

The [canonical report](evidence/analysis/stage01/report.json),
[response table](evidence/analysis/stage01/response_scores.csv),
[linearity table](evidence/analysis/stage01/linearity_scores.csv) and
[independent audit](evidence/review/physical_audit_stage01/final_all30/report.json)
retain every outcome. The [protocol](evidence/protocol.json) fixes cases,
frames, floors and sources before physics. Predictions were sealed before
the first replay. The [portable manifest](evidence/portable_manifest.json)
and [case inventory](evidence/case_inventory.json) distinguish copied evidence
from full local prefixes and workspaces.

Replay all paired response scores and linearity metrics from the compact
archive with Python and NumPy, without Gazebo or the original I-56 rollout:

```bash
python docs/iterations/2026-11-26/loaded-contact-validation/evidence/analysis/portable_replay.py \
  --dataset docs/iterations/2026-11-26/loaded-contact-validation/evidence/analysis/stage01 \
  --output /tmp/i57-replayed-responses
```

The portable check reproduces 576 loaded/free group scores and 144 linearity
metrics exactly, plus direct archived-anchor, input-history, continuation and
duplicate controls. It explicitly treats full pre-anchor parity as recorded
certification, not something recomputed from endpoint data alone. Both pilot
prefix arrays are also archived; full prefixes for all 30 cases remain local.
These output counts are not additional independent experiments.

Eight pure replay contracts, eight separate arithmetic checks and a fabricated
end-to-end fixture validate the tooling; no full repository suite is rerun.
Preserve the first synthetic plot's clipped annotation and its cosmetic-only
correction, the baseline figure layout correction, and the exploratory audit's
initial stdout-manifest timing issue. Final manifests are resealed after output
completion, without changing numerical outcomes. Partial analyses remain
explicitly partial; their repeated counts are not added to the final result.

To run fresh physical diagnostics, restore result-owned tools to their manifest
origin paths, follow the [Gazebo environment guide](../../../../GAZEBO.md),
register new source/data/plugin/model hashes and use a new output root. The
archived `run_probe.py --preflight` verifies registration before `--run`.
Existing case folders are refused. Full local evidence remains under
`results/loaded_contact_validation_20261126/`.
