# I-81 — loaded-belt contact adds downhill force during the departure

2026-11-28. **Completed and analyzed; original native N remains selected.** One
new unchanged-controller Gazebo/DART diagnostic reproduces the original I-76
eight −1 rad recording. No controller, observer, gain, preparation, limit or
physics setting changes. This is a shortened investigation, not a tested fix.

The user prefers the current LQR/observer/feedforward setup and asks whether
unanticipated terrain simply exceeds holding torque. The trace finds a more
specific lead: during some acceleration pulses, loaded belt A adds downhill
contact force while the controller identifies that support correctly. Incoming
preparation contributes zero to those held commands. Its LQR balance/motion
response deserves a causal test; a bad preview or misassignment alone cannot
explain those particular pulses.

[Two-page PDF](evidence/analysis/completed_stage01/diagnostic_comparison.pdf) ·
[routing and response](evidence/analysis/completed_stage01/routing_response.png) ·
[force, normal and preview](evidence/analysis/completed_stage01/force_preview_diagnostics.png) ·
[full analysis summary](evidence/analysis/summary.md).

## What the new physical trace establishes

Force is projected onto the downhill tangent defined by each completed
interval's pressure-normal aggregate. Positive means downhill.

| Interval end | Ground contact | Gravity | Sampled sum | Contact carrying the positive pulse |
| --- | ---: | ---: | ---: | --- |
| 100.835 s | −1.991 N | +3.234 N | +1.243 N | Mixed shell/belt; ground partly opposes gravity |
| 100.900 s | **+10.598 N** | +3.234 N | **+13.833 N** | Belt A only |
| 101.000 s | **+7.330 N** | +2.966 N | **+10.296 N** | Belt A +9.686 N, shell A −2.356 N |

For the 100.900 s interval, use the command issued at **100.895 s**. Belt A
really carries the load and the controller also selects belt A. It requests
**−0.889 m/s**, clipped to **−0.300 m/s**. Incoming preparation adds **zero**.
The same pattern appears in the interval ending 101.000 s, with a −0.918 m/s
request clipped to −0.300 m/s. These are belt-speed commands, not motor torques.

The 100.895 s belt LQR contribution contains approximately:

| Error block | Contribution to belt-A speed request |
| --- | ---: |
| Roll/pitch attitude | −0.452 m/s |
| Body linear velocity | −0.198 m/s |
| Body angular velocity | −0.164 m/s |
| XY position | −0.078 m/s |

The algebra identifies what produces the request, not independent physical
causes. A balancing robot can move its base to correct platform attitude;
simply deleting a large balance term could make matters worse. At 100.900 s
the normal/pressure-direction gap is only 0.0106°, while the robot is 4.337°
from its own desired up direction. A large later normal transition does not
explain this earlier pulse automatically.

Motor-command sign alone is not uphill/downhill force. The held belt surface
moves about [−.169,−.247,−.026] m/s in world coordinates at this pulse, while
ground reaction pushes downhill. Ground force, gravity and independently
reconstructed whole-robot COM momentum agree closely. This is evidence of
downhill traction in the recorded closed loop, not a gravity/motor work
partition or proof that one feedback block causes the eventual departure.

## What the trace does not establish

The ground also brakes the robot during other intervals. Across the full
1.3 s capture, integrating contact force along the changing downhill direction
gives −3.180 N s versus gravity +3.635 N s. These are descriptive projections
onto changing directions, not a fixed-world momentum decomposition. Do not
present the selected positive pulses as the entire history.

A clipped dynamic correction does not prove insufficient static holding torque.
In particular, this model prescribes belt surface speed; it does not model a
finite belt-motor torque/load envelope. Neither friction capacity nor static
slope-holding feasibility is independently identified by this replay.

There is no controller intervention here. Correct routing at the highlighted
pulses does not exonerate timing elsewhere, and algebraic LQR contributions do
not establish which change improves both balance and tracking.

## Observer, timing and preview

Over all 260 completed intervals, observer-vs-physical force discrepancy has
RMS **0.02010 N**. The same-filter physical-wrench direction agrees with the
observer to **0.08265° RMS / 0.40917° maximum**. This physical filter is seeded
once from native filtered wrench at 100.5 s; no delay or parameter is fitted.
The subsequently filtered controller normal differs from the interval pressure
direction by up to 6.406°, and desired-up rate later peaks at 184.945°/s.

Within 181 interval-end focus rows, dominant inferred family differs from the
last physical sample at52 rows and from prior-5-ms mean pressure at56 rows.
Sixteen last-sample differences agree with the completed interval. Nine inferred
families have no pressure in any of those five prior samples. These comparisons
are different observables with filtering history, not a clean observer-error
count or causal proof.

The full original preview is now saved:261 snapshots and724 focus target/future
associations. For100 nonzero-preparation targets, extrapolated-versus-actual
future configuration margins on the **same frozen plane** differ by0.175 mm
RMS, maximum absolute0.533 mm. This isolates one geometric motion observable;
it does not validate unknown future terrain. Geometric exposure is not a force-
onset forecast, so future unloaded targets are not counted as wrong predictions.

An explicitly posthoc supplement counts181 holds whose command **starts** lie
in100.75–101.65 s. Belt A has148 nonzero applied commands:38 with no actual
belt contact,67 partial and43 all-five-substep pressure occupancy. Of103 clipped
belt commands,7 have no belt contact. This is a different shifted set from181
interval-**end** focus rows; denominators are labeled. An unloaded belt has no
direct ground traction in this model; an unloaded shell motor can still affect
balance by reaction torque. Original definitions/results remain unchanged.

## Conditions and validation

Source revision `979932b8a4ead8e713f176c6db6ef3cbfaf57c3f`. Original I-76 eight−1
rough-stress Mars route, origin[−2.5953655642,10.4602524267] m, yaw158°, original
gain bank and[1.5,.6,.6 N m,.3,.3 m/s] bounds. Native control uses exact simulated
body state, observer and known geometry; physical contact/terrain is scoring-only.
Heading helpers, earlier retiming and other candidates are inactive.

Nominal path133 s; diagnostic stop102 s. Contact capture100.50–101.80 s at1 ms;
control remains5 ms. Focus anchors100.75–101.65 s leave room for their full120 ms
future queries. The process exits zero and all79 original precision fields
match **20,401 rows exactly**, except the explicit final unheld-command flag.
All1,300 substeps/260 holds and261 preview rows are present. One original live
command, observer update and preview occur per control sample. No extra live
model step, reset or future-contact input is introduced.

[Frozen protocol](evidence/protocol/frozen_stage01/protocol.json) SHA256
`dc916da7831c5930a3f2dbd838f076c812acde3555bbcfc4dbbbd5cec51fbfa8`
binds94 sources/59 inputs before physics. [Output seal](evidence/campaign/outputs_sealed.json)
SHA256 `c38602c73d4167c837a153619c37f963d72a1a201c80fbff89d2ed61132507aa`
binds227 closed outputs. Sources/inputs remain unchanged.

Whole-COM external-force balance residual is0.01971 N RMS /0.17146 N maximum
over260 intervals, against model weight18.2466 N. Independent MuJoCo kinematics
and Jacobians (no dynamics integration) confirm COM and original preview
reconstruction. Independent analysis agrees across26,264 scalar leaves; largest
COM-force difference1.45e−12 N, largest angle discrepancy1.41e−10°.

Six runner contracts, nine final pure analysis contracts and three archive
contracts pass in separate scopes. Analysis stage02 includes the earlier eight;
repeat stages are not added. Sixteen random-state kinematic checks are also
separate from physical trials and the full repository suite, which is not rerun.

## Decision and next experiment

Keep the current native pipeline. The strongest next causal target is the
**loaded-belt LQR response during transfer**, with separate balance and tracking
endpoints. A bounded same-prefix intervention would isolate that feedback
contribution while retaining observer, support feedforward and preparation, and
would examine release effects too. This is proposed, not run or promoted.
The new evidence does not justify treating an unseen slope, poor preview,
misassignment or motor size as the complete explanation.

## Records and reproduction

[Main report](evidence/analysis/completed_stage01/report.json) ·
[held-command/force/LQR ledger](evidence/analysis/pulse_stage01/report.json) ·
[independent precision verification](evidence/verification/core_stage01/report.json) ·
[independent scalar agreement](evidence/verification/comparison_stage01/completed/report.json) ·
[compact manifest](evidence/manifest.json) ·
[exact source and structured diagnostics](evidence/source-and-diagnostics.tar.gz).

Compact evidence binds83 files plus its manifest, including389 verified archive
members. The [full local raw bundle,93.9 MB](../../../../results/slope_response_trace_20261128/publication/full_evidence_stage01/i81_full_evidence.tar.gz)
has586 verified members, SHA256
`361e762fa174ff7df312b40f41798487213dd471774f416efbd4a8a9240784a7`.
[Archive verification](evidence/publication/full_evidence_stage01/archive_verification.json) ·
[prerequisite inventory](evidence/publication/full_evidence_stage01/external_prerequisites.json) ·
[restore instructions](evidence/publication/full_evidence_stage01/REPRODUCE.md).

Preserve initial ROS-plugin/lark test-collection failure (corrected only by
disabling unrelated pytest plugin autoload), verifier syntax draft, and main/
pulse analysis compile drafts. Each correction has a new stage/source receipt;
frozen runtime/core and existing records do not change. Analysis wrappers are
registered separately before reading arrays. No outcome-dependent tuning or
new acceptance criteria are introduced.

After restoring result-owned sources at recorded paths, inspect the existing
sealed run without executing new physics:

```bash
.venv/bin/python results/slope_response_trace_20261128/verification/verify_core.py \
  --run results/slope_response_trace_20261128/campaign/native102_stage01 \
  --output results/slope_response_trace_20261128/verification/new_review
```

Use new output stages; closed results cannot be overwritten. The captured
runner's `--run` executes a **new** physical prefix and is not needed to review
the recorded evidence. Full-route acceptance and a controller cure remain untested.
