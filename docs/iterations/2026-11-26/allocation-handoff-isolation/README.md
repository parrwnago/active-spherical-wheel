# I-51 — isolate reference feasibility and handoff continuity

2026-11-26. **Five native cases completed; analysis and evidence checks completed. Defaults unchanged.**

The user asks whether the late I-49 departure was intentionally allowed by
the allocator, and authorizes investigating the reference and handoff problems
identified in I-50. The recorded baseline uses `rolling_allocation=least_squares`;
bounded/feasible allocation and motion-reference relaxation are inactive.
Original XY targets remain in use. Intentional travel relaxation therefore
does not explain the recorded 191 mm departure. Poor allocation or another
feedback/model problem could still contribute.

## Fixed comparisons

Four fresh 44 s Gazebo/DART Mars2021 figure-eight runs, phase +0.90 rad,
yaw 45°, original origin/radius/path timing and observer/contact boundary:

| Arm | Operating belt cap | Intervention |
| --- | ---: | --- |
| `base030` | 0.30 m/s | Original controller |
| `reference030` | 0.30 m/s | Limit joint-rate references only |
| `base060` | 0.60 m/s | Original controller |
| `handoff060` | 0.60 m/s | Limit total belt-command slew during transfers only |

The original 0.30 design configuration, flat gain bank and ordinary K remain
fixed. Operating bounds are applied to both physical plant and state mirror
after design construction, as in I-49. No combined candidate or outcome-based
tuning is included. Compare each candidate with its matching-cap baseline.

The reference guard keeps native least squares and original body/XY targets.
It projects the three joint-rate references into the existing assumed limits
of 20 rad/s and 100 rad/s². Apply after reference generation and before LQR,
including held/zero-motion references. Leave belt feedforward unchanged.
It is exactly inactive when the native references satisfy those bounds; it
does not deliberately move the position target or guarantee feasible total
feedback torque. Report introduced rolling residual and every activation.

The handoff guard detects crossings of the same observer-derived 5% load-share
boundary as preparation. During an episode, project the native box-clipped
total belt command into the previous actually applied command ±3.0 m/s² times
the control interval, intersected with the operating speed box. End an episode
when the native target lies inside that interval. Preserve all three joint
torques exactly. Initialize load bookkeeping without inventing a first-call
transition; no experiment-time activation gate or minimum hold is used.
This does not constrain every later loaded-belt feedback change.

These are existing numerical reference/acceleration assumptions, not new motor
ratings. Candidate code is isolated under ignored `results/`; runtime/default
files remain unchanged. Preserve native raw request, native clipped output,
candidate change and actual applied command separately. Observer updates must
use the actual previous applied command.

## Evaluation fixed before outcomes

First check saved difficult states and the helpers' meaningful contracts:
identity when inactive, reference speed/rate bounds, total belt speed/slew
intersection during episodes, sign reversals and resets. A saved-state replay
does not establish dynamic benefit.

For native runs, require baseline parity with I-49, fixed initial/path/gain
identities and guarded evaluator-only truth. Report all completion/fall,
original moving RMS, peak/final error, local departure, settling, all six fixed
turn overshoot/lag results, reference/applied/requested maxima and jumps,
clipping, slip/load, relative/absolute lean and candidate activation times.
Exclude unactuated terminal observations from held-duration metrics.

Mechanistic success means the reference guard obeys its declared bounds, or
the handoff guard obeys its episode command bounds. Broader benefit is separate:
at least 10% lower moving RMS, peak/final error no more than 1 mm worse, peak
relative lean no more than 0.5° worse, every turn overshoot no more than 5 mm
worse and signed lag no more than 0.1 s worse, all six reversals covered, with
the original settling/route gates reported independently. No automatic default
adoption follows. Broader trajectories are contingent on meaningful benefit;
do not expand a failed candidate merely to accumulate trials.

The late unsaturated drift is investigated separately using saved histories
and current-pose kinematics. Correct contact identity or exact command-term
arithmetic is insufficient to prove a unique physical cause. Preserve contrary
evidence and distinguish hypotheses from controlled interventions.

Protected sources and 19 prior presentation archives were captured before
this iteration in [the local manifest](../../../../results/allocation_handoff_isolation_20261126/pre_change/manifest.json).
The complete protocol, exact candidate hashes, outcomes and evidence will be
appended without replacing unsuccessful trials.

## Additional late-phase diagnostic, registered before its execution

The saved-state audit finds no intentional XY relaxation, almost zero rolling
residual/slip in the late window, and continuing joint-rate tracking error.
At 41 s, body-velocity error is approximately [−0.00256, +0.06057] m/s;
joint-rate error contributes [ +0.00902, +0.07933] m/s, partly offset by platform
rotation. The native gain schedule measures phase against world vertical,
which differs from the sensed-normal phase by 7.83° at that snapshot.
This motivates one additional diagnostic, without changing the locked four-arm
protocol or its sources.

`phase_late030` uses a separate protocol/runner and the original 0.30 m/s arm.
It follows the original controller until 38 s, then changes only
`schedule_on_ground_normal` from false to true through 44 s. Require exact
matching of every common precision field before 38 s and identical state at
the switch. Compare late [38,42] and [38,44] error, lean, yaw and joint-rate
tracking, plus original whole-route metrics. No fitting of weights or switch
time follows the outcome. Earlier global normal-relative scheduling failures
remain relevant; even improvement here would be a mechanism result rather
than a deployable time-gated policy. This caps new physics at five cases.

[Additional protocol](../../../../results/allocation_handoff_isolation_20261126/phase_schedule_isolation/protocol.json).

## Answer to the user's allocation hypothesis

The recorded 191 mm departure was **not intentionally permitted by a feasible
motion-reference allocator**. Every recorded desired XY and body-velocity target
matches the original program exactly. In baseline 38–42 s, the native rolling
equations have maximum residual about 1.22e-17 m/s and normalized condition
numbers no greater than 4.87. Evaluator contact-slip RMS is only 0.0000901 m/s,
while body-velocity tracking-error RMS is about 0.04039 m/s.

The robot can therefore roll with little slip while moving at the wrong speed
or direction. The computed joint-rate targets are states in the LQR objective,
not independent hard velocity servos. Their actual tracking competes with
attitude, angular-rate and other state errors. At 41 s, most positive world-Y
velocity error comes from joint-rate mismatch; platform rotation partially
offsets it. This establishes the kinematic source of velocity error, not a
unique proof of which dynamic feedback term should be retuned.

The static balance algebra is also consistent: at fixed late snapshots the
requested attitude places the total center of mass over the estimated support
to numerical precision, and estimated normals are accurate. Source and saved
coordinate checks find no simple velocity-frame sign error. These results
narrow the question toward dynamic local feedback/scheduling and its history,
rather than deliberate target relaxation or a gross static lean formula error.

## Native results: primary matched pairs

All five fresh cases finish 44 s without falling, with 8,801 observed control
calls each. The two primary baselines reproduce all **41 common I-49 precision
fields** byte-for-byte. Initial states, complete reference programs, fixed gain
bank, ordinary design, cache and bridge binary checks pass.

| Metric | Original 0.30 | Reference guard 0.30 | Original 0.60 | Handoff guard 0.60 |
| --- | ---: | ---: | ---: | ---: |
| Moving timed RMS | 50.486 mm | 44.213 mm | 52.521 mm | 42.056 mm |
| Full peak error | 190.901 mm | 97.186 mm | 110.009 mm | 92.651 mm |
| Final error | 190.191 mm | 32.687 mm | 49.231 mm | 15.575 mm |
| Peak reference-relative lean | 8.709° | 6.346° | 6.053° | 11.683° |
| First sustained 20 mm local departure | 6.860 s | 6.860 s | 6.850 s | 6.880 s |
| Original settling / full-route gates | Fail / Fail | Fail / Fail | Fail / Fail | Fail / Fail |
| Registered complete benefit screen | Baseline | Fail | Baseline | Fail |

The reference candidate reduces moving RMS by 12.43% and improves peak/final
error and relative lean. However, turn four's overshoot increases by 11.468 mm,
and some signed-lag guards fail. Its turn-four signed lag changes −0.444→+0.374 s:
absolute phase mismatch becomes slightly smaller, so do not describe that
signed increase as universally worse timing. All six events remain included.

In this candidate's **own native history**, the reference guard modifies only
53 held samples / 265 ms, first at 6.700 s. Maximum native joint reference is
5.380 rad/s, already below the 20 rad/s speed box. Therefore the speed ceiling
does not bind in this new history: the active intervention is the 100 rad/s²
reference-slew envelope. Guarded references peak at 3.119 rad/s. Earlier changes
alter the later configuration, so the native result is not an isolated replay
of baseline's 31.650 s singular state. The separate saved-input test demonstrates
bounded output at that old state, without predicting a counterfactual trajectory.

All 53 modified samples have reconstructible reference geometry. The guarded
rolling residual reaches 0.09005 m/s (RMS 0.02966 over those samples), with exact
single-patch versus float32 plural-point quality labeled. This is an intentional
local nominal mismatch introduced by the new guard, not the cause of the old
baseline drift. It is not actual slip or predicted achieved motion.

The handoff candidate reduces moving RMS by 19.92% and improves final error,
but peak reference-relative lean worsens from 6.053° to 11.683°. Turn guards
and settling also fail. Its helper changes 496 held samples / 2.480 s, first at
6.700 s, and obeys the 3 m/s² episode bound exactly. This confirms smoothness
and balance performance are separate requirements. It does not limit joint-rate
references; native references still reach 143.914 rad/s in its different history.

## Late phase scheduling has a demonstrated effect

For the fifth diagnostic, all **47 common precision fields across 7,600 samples
before 38 s** match the fresh 0.30 baseline exactly. Physical state, observer,
previous applied command and reference agree at the switch. The first gain
difference occurs at recorded 38.000 s. Thus the post-switch difference is an
intervention on scheduling after an identical past, not a new startup/route.

| Metric | Original 0.30 | Normal-relative phase after 38 s |
| --- | ---: | ---: |
| Moving timed RMS | 50.486 mm | 50.495 mm |
| Full peak error | 190.901 mm | 96.309 mm |
| Final error | 190.191 mm | 53.993 mm |
| Late 38–42 s RMS | 57.857 mm | 48.323 mm |
| Late 38–44 s RMS | 84.652 mm | 56.707 mm |
| Peak relative lean over 38–44 s | 8.709° | 1.745° |

This provides causal evidence that the scheduling choice affects this late
episode, consistent with the saved 6–8° phase discrepancy. It does not prove
that scheduling alone explains every part of the original drift. The route
before 38 s is unchanged, and moving RMS is essentially unchanged. The robot
still has a position offset; this is not demonstrated complete recovery or
whole-route qualification. Earlier global scheduling regressions are retained.
Do not deploy the hardcoded 38 s switch: it is a diagnostic to isolate an effect.
Heading remains free. Its net unwrapped yaw change over 38–44 s increases in
magnitude from −109.755° to −142.269°, even though body-yaw-rate RMS is lower.
This is a retained motion tradeoff, not failure of an enabled heading target.
All six fixed turn windows precede the switch and retain the baseline results;
this late diagnostic provides no evidence of improved earlier turns.

## Decision

Keep defaults unchanged. The reference-slew experiment is useful and more
promising than the old uniform damping, but remains experimental because turns
and settling regress/fail. Do not adopt the belt slew rule as a balance-safe
handoff solution; its lean regression is substantial. Do not combine candidates
and attribute improvement to an untested combination.

The next specific investigation is consistent gain scheduling under estimated
contact geometry, with joint-reference tracking and balance authority assessed
together. The late-phase intervention gives a concrete starting point. A general
configuration-based rule must pass earlier intervals, other starting phases and
the full turn/settling checks before adoption. No additional broad trajectory
campaign was launched after these candidates failed the registered screen.

## Evidence and verification

Candidate code lives only under this ignored results family. No repository
runtime/config/default files were edited. The observer receives actual previous
applied inputs in every arm; solved contacts remain evaluator-only under runtime
guards. Common raw requests retain their native meaning; common projection delta
includes both box clipping and handoff modification. Exact precision fields
separately preserve native box clipping, native applied output and handoff delta.
Native saturation flags do not measure the handoff envelope.

Seven reference-helper contracts and eight handoff-helper contracts pass; three
separate local handoff projections pass. These saved-signal checks do not add
robot trials. Metric helper checks repeat prior arithmetic and are not added to
historical totals. Independent native QA reconstructs both guards without
importing their implementations, verifies previous-final-input alignment and
component identities, and separates integrity from scientific benefit.

Retained development failures include inherited ROS pytest-plugin collection,
a static-audit variable collision, an initial shell-only assumption in a mixed
control snapshot, and a raw-QA whitespace syntax error before execution. Their
sources/logs remain before corrected stages. No failed native case was replaced,
no outcome tuning occurred, and no failed result was removed.

[Primary scientific report](../../../../results/allocation_handoff_isolation_20261126/analysis/stage01/report.json)
· [Independent native QA](../../../../results/allocation_handoff_isolation_20261126/review/native_qa/stage01/report.json)
· [Late saved-state audit](../../../../results/allocation_handoff_isolation_20261126/late_audit/README.md)
· [Independent late-phase check](../../../../results/allocation_handoff_isolation_20261126/review/late_phase_stage01/report.json).

[Four-arm comparison video](../../../../results/allocation_handoff_isolation_20261126/videos/allocation_handoff_comparison.mp4)
· [Late-phase matched-history video](../../../../results/allocation_handoff_isolation_20261126/videos/late_phase_diagnostic.mp4)
· [Compact scientific summary](../../../../results/allocation_handoff_isolation_20261126/analysis/README.md)
· [Phase scientific report](../../../../results/allocation_handoff_isolation_20261126/analysis/phase_stage01/report.json)
· [Independent interpretation/numeric review](../../../../results/allocation_handoff_isolation_20261126/late_audit/reducer_review/final_report.json)
· [Compact inventory](evidence/manifest.json)
· [Shareable source/data bundle](../../../../results/allocation_handoff_isolation_20261126/presentation-pack.zip).

The first movie shows all four primary native histories at 2×, with matched-cap
rows, common camera/scales and unfavorable result cards preserved. The second
shows baseline versus late-phase diagnostic over 36–44 s at 0.5×, with a clear
38 s switch marker. Both render saved Gazebo states and integrate no new physics.
The second card retains moving RMS to three decimal places, so 50.486→50.495 mm
is visible rather than rounded to an apparently exact match.

## Reproduction

Use the Gazebo environment in [GAZEBO.md](../../../../GAZEBO.md), with
`MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1` and the original
terrain assets. Frozen source/input hashes must match. The read-only check is:

```bash
python results/allocation_handoff_isolation_20261126/run_isolation_case.py --preflight
```

For new native trials, create a fresh output root with copies of `protocol.json`
and `campaign/jobs.json`, then run the frozen runner against it, once per case:

```bash
python results/allocation_handoff_isolation_20261126/run_isolation_case.py --output results/new_i51 --run --case-id base030 --cap .3
python results/allocation_handoff_isolation_20261126/run_isolation_case.py --output results/new_i51 --run --case-id reference030 --cap .3
python results/allocation_handoff_isolation_20261126/run_isolation_case.py --output results/new_i51 --run --case-id base060 --cap .6
python results/allocation_handoff_isolation_20261126/run_isolation_case.py --output results/new_i51 --run --case-id handoff060 --cap .6
```

The separate fifth runner uses copies of its separate protocol/jobs in a fresh
root and `--case-id phase_late030 --cap .3`. Existing native case directories
are refused. Its 38 s switch is solely a diagnostic.

Saved-data reductions use `analysis/analyze_isolation.py` and
`analysis/analyze_phase_addendum.py` with fresh `--output` stage directories.
Their captured source/layout and input hashes identify the exact reductions.
Do not overwrite sealed stages. The bundle includes the raw arrays, source,
private mirror geometry and movies for review/reduction; native regeneration
additionally needs the documented simulator environment and terrain assets.
