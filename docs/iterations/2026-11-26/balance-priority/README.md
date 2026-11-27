# I-58 — balance response before optional smoothing

2026-11-26. Implement and test the isolated next experiment from
[I-57](../loaded-contact-validation/README.md). The selected controller remains
native gain-scheduled LQR, momentum observer and current feedforward. This
candidate is opt-in; no existing runtime, observer, model, reference generator,
gain bank or controller default is modified.

## Question and one policy change

I-57 confirmed that the old experimental hard rate box can prevent the native
balance action. At a saved H state, prior shaft torque +1.197503 N m and the
60 N m/s box force +0.897503 even though the capped native command is -1.5.
The corresponding large transverse response difference is physically measured
and predicted accurately. A separate yaw-cost baseline issue is left for a
different experiment; both extra yaw and momentum weights stay zero here.

Compare three policies with identical model and soft weights:

| Policy | Meaning |
| --- | --- |
| B | Original native LQR/observer/feedforward, with its ordinary final clipping |
| A | Exact I-56 A: hard total amplitude and actual-prior rate boxes, best attainable local balance response, then soft secondary costs |
| P | New balance-priority allocator: hard total amplitude and exact nominal transverse response; command change remains a soft cost |

The amplitude caps remain `[1.5, .6, .6 N m, .30, .30 m/s]`. P preserves
`G20[3:5] (u − clip(u_native_total)) = 0`, where the nominal total command
contains the native LQR, feedforward and transition terms. This is a two-row
local response equality, not a separately identified pure balance torque or
a physical stability guarantee. The amplitude-clipped native command is always
a feasible seed and a verified fallback.

P retains exactly A's travel-response, native/readiness, command-change and
effort objective terms and weights, normalized by input caps. The old 60 N m/s
and 6 m/s² thresholds become diagnostics, rather than P feasibility bounds.
No fictitiously large rate limit is substituted. Actual previous applied input
still centers the soft change cost. Yaw/momentum weights must be zero.

Numerically, P uses a row-normalized equality basis, verifies the original
physical G equality after clipping and restores the nominal clipped command
after a failed/unverified solve. Thirty independent synthetic comparisons
agree with A within 5.97e-7 in command when A's rate box is nonbinding. This
supports unchanged secondary behavior while retaining explicit solver limits.

The frozen measured-contact model is unchanged: 20 ms held-geometry sensitivity,
soft contact strength 10, mismatch confidence scale 0.02 m/s, and current
observer-derived patch records. It does not predict contact onset or certify
friction/unilateral feasibility. Body/joint measurements remain exact simulation
state; contact truth is evaluation-only. The original 120 ms incoming-drive
preparation remains active.

## Registered comparison and gates

Initial fresh serial jobs B/A/P use the same 44 s Gazebo/DART Mars figure-eight,
initial shaft -1 rad and yaw 45°. Both B and A must reproduce every shared
historical precision field byte-for-byte, including dtype, signed zeros and
input-history fields. Sources, plugins, original gain-bank bytes and A/P
settings are registered before execution. A and P settings must match the
frozen I-56 A exactly; P replaces only its allocator policy.

Primary benefit is declared before results: P's total modeled transverse
discrepancy must be ≤1e-8 at every emission and its held RMS at most 10% of
A's, with A peak ≥0.01 rad/s to exclude a trivial comparison. Always sum
first-stage and secondary discrepancies; a small secondary equality alone
did not establish native response delivery in A. This compares along-trace
algorithm behavior, not same-state physical trajectory outcomes.

P must also pass nonregression versus B: completion; finite actual command
history; hard amplitude/equality checks; moving timed XY RMS within 10% +1 mm;
peak/final XY within +10 mm; raw lean peak/absolute tilt within +1°; raw lean
RMS within 10% +0.1°; each of six fixed turn overshoots within +10 mm; and
existing settling if B settles. Late yaw peak and absolute heading travel use
5% allowances plus 0.01 rad/s and 0.01 rad, respectively. Full-route planar
speed peak uses 10% +0.01 m/s, and 50 ms average 3D linear-acceleration peak
uses 10% +0.1 m/s². These are fixed engineering tolerances, not significance
claims. Actual acceleration is not bounded by a command constraint.

Command continuity is a fixed **secondary diagnostic**, not an alternative
benefit selected after outcomes: report caps-normalized applied-step L2 RMS,
99th percentile and maximum. Include the first actual prior input; exclude
the final emitted but unheld command. Do not use P's diagnostic rate exceedance
as a hard feasibility failure or its bounded request as an improvement claim.

Only if all lead gates pass do +0.30/+0.90-rad B/P pairs run, for at most seven
fresh simulations. Missing conditional cases remain explicit. No automatic
default promotion follows even a passing three-phase screen.

## Pre-execution checks

There are 31 new numerical allocator tests; the combined selected run has
72 passes including 41 existing allocator/model tests. Earlier subsets and
reruns are nonadditive; this is not the full repository suite. Sixteen separate
runner/adapter contracts pass, as do eight saved-metric contracts. The initial
ROS pytest plugin setup failure occurs before collection; plugin autoload is
disabled for the isolated run. An unexecuted metric draft's short-trace review
issue is corrected before freezing: missing late-window values now fail the
screen instead of crashing reduction.

Static regression at the exact two I-57 H anchors confirms that A and P
agree at the early state where the rate box is nonbinding. At the late state,
A commands shaft +0.897503 while P commands -1.5; total transverse discrepancy
changes from `[-.056736, -3.740082]` to numerical zero. No new Gazebo physics,
observer update or trajectory is executed by this static check.

## Initial configuration: all registered gates pass

All three -1.00-rad runs complete 44 s. Both B and A reproduce all 59 shared
historical precision fields byte-for-byte. Independent formulas agree with all
26 lead gates. P passes every gate, permitting the four conditional jobs.

| Initial-phase metric | B native | A hard rate | P balance priority |
| --- | ---: | ---: | ---: |
| Moving timed XY RMS, mm | 51.080 | 57.152 | 33.432 |
| Peak XY error, mm | 116.256 | 183.974 | 72.160 |
| Final XY error, mm | 42.648 | 15.470 | 6.293 |
| Peak raw-reference lean error, ° | 7.399 | 13.426 | 6.812 |
| Peak absolute tilt, ° | 13.718 | 15.039 | 13.621 |
| Late peak world-yaw rate, rad/s | 2.443 | 10.013 | 1.349 |
| Late absolute heading travel, turns | 1.800 | 1.081 | 1.152 |
| Normalized command-step RMS | 0.12753 | 0.12239 | 0.09553 |

Moving means 1–41 s under the unchanged legacy timestamp mask; late means
28–44 s. Other peaks use the entire 0–44 s observation. P's maximum absolute
modeled priority residual is 4.686e-16 rad/s and its held RMS is 7.045e-17,
versus A's RMS 0.267424. P uses no nominal fallback in this initial run.
Command-step RMS improves as a secondary diagnostic; it was not a post-hoc
alternative acceptance criterion.

Two qualifications remain important. **None of B, A or P passes the actual
final settling test.** P reaches 6.3 mm position error but retains rotation:
world-yaw rate at 44 s is +0.453 rad/s. The conditional settling gate passes
because B also fails, not because P is stationary. Heading has no commanded
target and the extra yaw penalty is disabled. Also, P's 50 ms 3D acceleration
peak rises from B's 8.07751 to 8.95456 m/s², narrowly below the registered
8.98526 allowance. Retain this regression despite the improved tracking.

[Lead overview, 4×](../../../handoff/media/i58-overview.mp4)
· [28–36 s replay, 0.5×](../../../handoff/media/i58-handoff-event.mp4).
These are display-only replays of actual saved Gazebo poses on the Mars DEM,
not extra physics or predicted motion. The HUD separates absolute tilt,
raw-target error and physical world-yaw rate.

## Broader result: do not adopt as a general replacement

All seven registered runs complete 44 s without code, weight, target or gate
changes. The additional native B runs reproduce all 56 shared historical fields
exactly; lead B/A reproduction has 59 fields. Independent formulas agree with
every final comparison. Only the lead phase passes the complete screen:

| Initial shaft phase | Moving XY RMS B→P, mm | Final XY B→P, mm | Full P screen |
| --- | ---: | ---: | --- |
| -1.00 rad | 51.08→33.43 | 42.65→6.29 | Pass |
| +0.30 rad | 56.96→54.37 | 20.82→52.62 | Fail |
| +0.90 rad | 50.49→48.37 | 190.19→63.85 | Fail |

At +0.30, final error exceeds the +10 mm allowance, and turn six overshoot
worsens by 12.903 mm. At +0.90, late peak yaw rate rises 1.835→3.084 rad/s,
full planar-speed peak rises 0.319→0.502 m/s, and turn-three/four overshoot
worsens by 65.644/26.167 mm. These fail the registered guards even though
peak XY error improves 190.90→130.92 mm and final error improves substantially.

P achieves its local mathematical objective everywhere: maximum absolute
priority discrepancy is at most 9.991e-16 rad/s across all three starts,
amplitude violations are zero, and no nominal fallback is used. Command-step
RMS decreases at every start. These findings do not override the physical
regressions. No case passes the actual final settling test. The final all-phase
screen is **false**; the initial extension decision remains true as a historical
fact, not a final adoption flag.

Keep native LQR selected and retain P as an experimental module. I-58 shows
that exact modeled balance-response priority can improve one full trajectory,
but does not establish a robust motion controller across these configurations.
No post-outcome tuning or substitute candidate is run.

## Exploratory goal-hold investigation

After observing the +0.30 endpoint failure, register a separate read-only audit
of the same fixed 41–44 s goal-hold window for all B/P phases. This adds no
physics, acceptance gate or tuned parameter and does not replace the main result.

At +0.30, P's error is 23.48 mm at 41 s, reaches a 19.01 mm minimum at
41.54 s, then rises to 52.62 mm by 44 s. The final half-second is nearly
stationary there: error improves only 0.071 mm and average radial velocity
toward the goal is +0.157 mm/s. This is not clear recovery. B passes close to
the goal at 43.455 s (1.69 mm error) before moving away to 20.82 mm.

At +0.90, P's error increases 34.70 mm during the hold, including 10.03 mm
in the final half-second. Mean final-half-second radial velocity is
-20.09 mm/s (negative means away), with final value -11.62 mm/s. It is still
moving away at the recorded endpoint, despite ending closer than B.

The current allocator's modeled radial travel adjustment is tiny in the last
half-second: -0.00394/-0.00758 mm/s for the two failed phases. These effects
are below the earlier model's 0.2 mm/s resolution floor. No final-half-second
clipping, diagnostic rate exceedance or support-label disagreement explains
the hold. At +0.30, disagreement bursts begin only at 42.265 s, after the
outward trend begins around 41.54 s. These observations do not establish a
unique cause, nor exclude earlier allocator-induced state-history effects.

The next diagnostic target is the existing native goal-hold feedback, desired
pose/velocity and drive-reference terms at these actual P states, plus turning
response. Inspect those before another weight change or estimator extension.
No proposed follow-up is executed as a controller change in this iteration.

## Evidence and reproduction

[Lead overview](../../../handoff/media/i58-overview.mp4)
· [Slow lead event](../../../handoff/media/i58-handoff-event.mp4)
· [Both additional starts](../../../handoff/media/i58-other-starts.mp4).

[Final canonical report](evidence/analysis/stage02/report.json)
· [All-case table](evidence/analysis/stage02/cases.csv)
· [All turns](evidence/analysis/stage02/turns.csv)
· [Lead plot](evidence/analysis/plots_stage02/m100_comparison.pdf)
· [+0.30 plot](evidence/analysis/plots_stage02/p030_comparison.pdf)
· [+0.90 plot](evidence/analysis/plots_stage02/p090_comparison.pdf)
· [Goal-hold plot](evidence/review/goal_hold_stage01/read002/goal_hold.pdf).

The [protocol](evidence/protocol.json),
[initial report](evidence/analysis/stage01/report.json),
[final independent audit](evidence/review/outcome_stage02/audit_result.json),
[source preservation](evidence/pre_change/manifest.json) and
[portable manifest](evidence/portable_manifest.json) retain chronology and
the distinction between execution, local priority restoration and acceptance.
Full raw precision/rollout/governor histories remain in ignored
`results/balance_priority_20261126/`; lossless metric projections retain every
original 5 ms timestamp in the archive, without downsampling.

With Python and NumPy, reproduce every case metric and comparison gate from
the copied source/data, without Gazebo or prior result directories:

```bash
python docs/iterations/2026-11-26/balance-priority/evidence/analysis/replay.py \
  --data-source docs/iterations/2026-11-26/balance-priority/evidence/analysis/stage02 \
  --output /tmp/i58-replayed-metrics.json
```

Portable replay verifies archive hashes and retains actual I-58 variants and
provenance. Only copied metric-dispatch labels adapt to immutable I-56 math;
native controller metadata is not rewritten. The standalone replay and
independent formulas reproduce the final results exactly. Older partial stages,
initial decision and fixtures remain visible; repeated counts are nonadditive.

For new physical experiments, read the
[runner contract](evidence/campaign_tools/README.md), restore result-owned files
to manifest origin paths, follow [GAZEBO.md](../../../../GAZEBO.md), register
fresh input/plugin/model/source hashes and use a new output root. Existing case
folders are refused. This is a recipe for a newly registered experiment, not a
claim that a clone contains every ignored historical raw dataset.
