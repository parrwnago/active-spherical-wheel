# I-52 — configuration-dependent scheduling and joint-reference tracking

2026-11-26. **Twelve policy cells completed and analyzed; configuration-dependent gains and regressions retained. Defaults unchanged.**

The user authorizes investigating contact-relative gain scheduling together
with joint-rate tracking. I-51 found useful tracking changes from reference
slew limiting and a late-only phase-scheduling intervention. Neither passed
the complete route/turn/settling screen. The 38 s intervention was diagnostic;
this iteration uses no time gate and does not adopt it as a policy.

## Conditions and two factors

Run 12 fresh Gazebo/DART cases: the original 44 s Mars2021 figure-eight at
initial common-shaft phases −1.00, +0.30 and +0.90 rad, with all four variants
at each start. These configurations are the three starts already used in the
repository. All use yaw 45°, origin [−1.6, 9.6] m, path radius 0.45 m,
motion from 1–41 s, 5 ms feedback and 1 ms physics. Both belts retain 0.30 m/s
operating bounds and joint torques retain [1.5, 0.6, 0.6] N m.

| Variant | Schedule phase against estimated normal | I-51 reference guard |
| --- | --- | --- |
| B | Off | Off |
| R | Off | On |
| N | On for the whole run | Off |
| NR | On for the whole run | On |

The guard is the exact frozen I-51 helper, with 20 rad/s joint-reference speed
limits and 100 rad/s² slew limits. It changes only the three joint-rate
references before LQR; original desired XY/body targets and belt feedforward
remain. No new effort weights, damping, heading hold, belt-transfer envelope,
camera, contact estimator or actuator model is added. No outcome tuning or
additional native cases are part of this registration. Maximum concurrency is
three isolated native processes.

All cases retain the original flat gain bank and 0.30 design configuration.
The phase-specific ordinary local design and shaft-row anchor must agree
within each four-arm group, but may differ between starting phases. Physical
initial state/observer/nominal references must match within a phase. Initial
scheduled K may differ between B/R and N/NR because scheduling is active from
the first feedback call; require initial K identity only B↔R and N↔NR.
The complete raw XY/time/velocity/acceleration program stays fixed globally;
normal-derived attitude references can diverge with native state histories.

Controller measurements remain exact simulated body/joint state plus actual
previous applied commands. The momentum observer and support locator supply
contacts to control; solved contacts and terrain are evaluation-only under
runtime guards. The incoming 120 ms kinematic preparation remains unchanged.
Candidate code is isolated under ignored results; runtime/config/default
files and all 20 older presentation archives are protected.

## Scope of the scheduling approximation

`schedule_on_ground_normal` changes the phase angle used for interpolation in
the existing flat-ground gain bank. It does not rotate all state/input frames,
account for tilted gravity in a new linearization or solve a terrain-specific
Riccati equation. Native feedback restores the phase-specific ordinary shaft
row after interpolation, so the schedule change affects the other rows.
Load blending and held gains remain unchanged. This factorial tests the
interaction of two existing approximations, not a newly established
terrain-consistent LQR or a stability guarantee.

The phase angle uses only the carrier-frame normal's X/Z projection. A lateral
normal component can change while that angle stays fixed. The full normal is
still used elsewhere for desired lean and holding feedforward; the information
loss concerns this one-dimensional gain lookup, not the whole controller.
Carrier-frame normal components and phase-coordinate norms are logged to keep
that limitation visible. A current phase diagnostic also does not mean the
gain was freshly selected: during dropouts native feedback can hold an older K.

The source audit also checks coordinate conventions and the prior evidence
behind earlier scheduling statements. Any correction to that historical
interpretation will be recorded here without editing old records or bundles.

**Historical correction, before new native outcomes.** The recovered old
MuJoCo tuning comparison has 14 runs per setting, zero falls for both, and
cost 2.7456547388→2.6436046469 (3.7168% better with normal-relative phase).
It missed the 5% adoption margin, whose required cost was 2.608372. Earlier
descriptions of global scheduling evidence as generic regressions were too
broad. A legacy source comment asserts more Mars falls, but its referenced
run folder is absent; do not turn that comment into a quantified finding.
The old tuning JSON and current explanatory source are preserved separately;
the latter is not claimed to be the exact source executed by that old trial.
[Recovered evidence](../../../../results/contact_scheduling_joint_tracking_20261126/frame_audit/stage02/historical_schedule_evidence.json).

## Metrics and decisions fixed before outcomes

Keep B as the fresh per-phase baseline. Report R−B, N−B and NR−B separately,
plus NR−N, NR−R and the descriptive interaction NR−R−N+B. These deterministic
contrasts are not statistical replication or a pooled performance claim.

For every arm report completion/fall, original moving RMS/peak/final error,
local geometric departure, settling, all six fixed component-turn overshoot/
signed lag values, absolute/reference-relative lean, actual/requested inputs,
command jumps, torque/belt clipping, slip/load and support assignments.
Evaluate actual joint rates against BOTH native LS and used guarded references;
reducing error to a modified target alone would move the comparison target.
Report reference speed/slew violations, activation and introduced kinematic
residual separately from actual motion/slip. Record exact world-vertical,
estimated-normal and selected phases; identify dropout-held gains explicitly.
Terminal unactuated observations are excluded from held metrics.

The provisional benefit screen is per phase: complete without falling, at
least 10% lower moving RMS, peak/final error at most 1 mm worse, peak relative
lean at most 0.5° worse, each turn overshoot at most 5 mm worse and signed lag
at most 0.1 s worse, all six reversals covered. Original settling/route gates
are reported separately and required for adoption. A variant must satisfy
the screen at all three starts before broader adoption discussion; a mean
cannot hide a failed start. Retain signed versus absolute lag distinctions.

Check +0.90 B/R exact common precision parity with I-51. The −1.00/+0.30
historical diagnostics have float32 rollouts and summaries, without the same
precise source/provenance capture; their compatibility/parity is reported at
that precision and cannot be called complete float64 reproduction.

Evidence will include source snapshots before execution, exact protocol/jobs,
meaningful pure helper contracts, independent source/raw/metric review and
saved-state plots/videos. Preserve regressions and setup errors. No default
change or combined-policy adoption follows merely from successful execution.

Local family: [results](../../../../results/contact_scheduling_joint_tracking_20261126).
Protected source/old-archive capture:
[manifest](../../../../results/contact_scheduling_joint_tracking_20261126/pre_change/manifest.json).

## Technical process failure and exact-policy retry

The first `m100_N` process exited 139 with an empty progress log and no usable
rollout, summary or saved command rows. Its provenance, source snapshot and
flat-model workspace remain intact. The exact native fault site is unknown;
absent progress prints do not establish whether physics had advanced. This
attempt cannot be scored as a robot fall or as a completed controller outcome.

Before retrying, register one fresh serial attempt with identical source,
parameters and copied protocol/jobs. Only Python fault tracing is enabled.
Accept its observed completion or robot failure; no control tuning or retry
based on performance is allowed. Keep the original 12 policy cells distinct
from the extra technical process attempt. Frozen control files/protocol remain
unchanged. The explicit case-path resolver is captured as a new reduction
input, with a separate source stage; no old reduction source is overwritten.

[Original failure](../../../../results/contact_scheduling_joint_tracking_20261126/campaign/technical_failure_m100_N/failure.json)
· [Retry registration](../../../../results/contact_scheduling_joint_tracking_20261126/campaign/technical_retry_m100_N/registration.json)
· [Case-path resolver](../../../../results/contact_scheduling_joint_tracking_20261126/campaign/resolved_case_paths.json).

**Execution outcome.** The registered serial retry completes 44 s with 8,801
observed control calls and no fall. All twelve resolved policy cells complete
the same full horizon. Count thirteen launched process attempts, including the
one unscorable native crash, separately from twelve completed policy cells.
No parameters, control code or initial conditions changed to obtain a better
outcome. The first crash site remains unresolved; a successful retry does not
prove its cause or make the earlier failed process disappear.

## Native results

All twelve resolved runs finish 44 s without falling. Independent raw QA
passes source/parameter/input, phase, guard-history, held-gain and per-phase
identity checks. The +0.90 B and R cases reproduce all 47 common I-51 precision
fields byte-for-byte. Scientific reduction retains all 72 turn events and the
explicit failed-process/retry history.

| Start phase | Variant | Moving RMS, mm | Peak error, mm | Final error, mm | Peak relative lean | Settled | Original route gates |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| −1.00 | B | 51.080 | 116.256 | 42.648 | 7.399° | No | Fail |
| −1.00 | R | 76.789 | 306.794 | 43.269 | 13.196° | No | Fail |
| −1.00 | N | 39.195 | 87.890 | 12.686 | 5.074° | Yes | Pass |
| −1.00 | NR | 49.688 | 148.608 | 31.785 | 12.517° | No | Fail |
| +0.30 | B | 56.963 | 154.280 | 20.821 | 9.484° | No | Fail |
| +0.30 | R | 48.415 | 143.301 | 32.001 | 9.355° | No | Fail |
| +0.30 | N | 53.936 | 192.143 | 16.575 | 10.706° | No | Fail |
| +0.30 | NR | 53.466 | 179.981 | 17.963 | 10.218° | Yes | Fail |
| +0.90 | B | 50.486 | 190.901 | 190.191 | 8.709° | No | Fail |
| +0.90 | R | 44.213 | 97.186 | 32.687 | 6.346° | No | Fail |
| +0.90 | N | 43.284 | 93.652 | 36.311 | 7.757° | No | Fail |
| +0.90 | NR | 44.972 | 99.390 | 9.527 | 6.195° | Yes | Pass |

N lowers moving RMS at all three starts, but at +0.30 it worsens peak error
and relative lean. R lowers RMS at +0.30/+0.90 but substantially worsens it
at −1.00. NR does not consistently outperform N or R; its effects are not
additive. Do not pool these results into a universal improvement claim.

The original difficult +0.90 case has a useful result: NR finishes 9.527 mm
from the goal, passes the existing last-second settling test and passes the
original route gates. The baseline finished 190.191 mm away and did not settle.
At −1.00, N alone settles and passes the original route gates. At +0.30, NR
settles but still fails the original route gates. These distinctions must stay
visible in the presentation.

**No variant passes the complete registered screen at every start—or even
one complete per-phase screen.** The promising −1.00 N and +0.90 NR cases
still regress in turning. For example, −1.00 N adds 9.532 mm overshoot at
turn two and +0.184 s signed reversal lag. At +0.90, NR adds 23.463 mm turn-two
overshoot; its signed lag shifts earlier by 0.866 s while absolute phase
mismatch worsens by 0.830 s. Several other signed-lag increases reduce
absolute mismatch, so the full signed/absolute tables remain authoritative.
R at −1.00 does pass its turn nonregression checks, but fails tracking/lean/
settling requirements. Do not claim every individual case fails every turn check.

## Joint-rate tracking and interpretation

At −1.00, R modifies only 55 held samples / 275 ms, first at 10.930 s.
Its native references never exceed the 20 rad/s ceiling; reference slew is
the active bound. Moving-window actual-minus-used joint-rate vector RMS
worsens from 3.458 to 5.116 rad/s, with similarly worse actual-minus-native
tracking. Actual joint-rate vector RMS rises from 3.617 to 5.249 rad/s and
peak relative lean rises from 7.399° to 13.196°. This is a real motion/balance
regression, not just a change in the target used to score tracking.

At +0.30 R, native least-squares targets have large mathematical spikes in the
new history: moving-window native-target vector RMS is 165.408 rad/s, whereas
used-reference RMS is 1.214 rad/s. Actual-minus-native error is 165.393 rad/s,
versus actual-minus-used 2.987 rad/s. Those are different questions, and the
large native error is not a missed achievable motor-speed requirement. The
guard intentionally changes these infeasible nominal targets. Current-pose
reference residuals are reported separately, with exact single-contact versus
float32 plural-point quality; they are not physical slip measurements.

The stored `speed_clip_held_samples_byjoint` field counts native targets
outside the speed box, including when a guard is disabled. It is a potential
reference-bound violation count, not proof that an inactive guard clipped
anything. Actual modified holds and policy-enabled flags are separate.

The phase bit changes a partial gain lookup and therefore the closed-loop
history. It can influence actual rate tracking, support sequence and later
near-singular allocation exposure. Neither a lower reference error nor a
different equivalent contact proves a complete dynamics/model correction.
The exact per-phase factorial effects, dual-target tracking and support/phase
statistics are preserved without causal claims from correlation alone.

## Decision and next investigation

Keep the existing defaults. N is the lead experimental scheduling candidate
because it improves moving RMS at all three starts, but its +0.30 transient
and turn regressions prevent adoption. NR provides a useful +0.90 recovery
demonstration, not a general solution. Do not enable the fixed reference-slew
guard universally; its −1.00 regression is substantial.

The next targeted question is why the N and NR variants regress at direction
changes: inspect reference/feedback consistency and available drive authority
through those turns using the full estimated contact geometry. A one-angle
flat-bank lookup and independent reference clipping remain approximations.
No new weight scan, time gate, additional controller type or broad trajectory
campaign was added after seeing these outcomes.

## Evidence

[Canonical scientific report](../../../../results/contact_scheduling_joint_tracking_20261126/analysis/stage01/report.json)
· [Case metrics](../../../../results/contact_scheduling_joint_tracking_20261126/analysis/stage01/case_metrics.csv)
· [All turns](../../../../results/contact_scheduling_joint_tracking_20261126/analysis/stage01/turn_metrics.csv)
· [Factorial contrasts](../../../../results/contact_scheduling_joint_tracking_20261126/analysis/stage01/factorial_effects.csv)
· [Independent raw QA](../../../../results/contact_scheduling_joint_tracking_20261126/review/native_qa/final_stage01/report.json)
· [Independent numerical review](../../../../results/contact_scheduling_joint_tracking_20261126/frame_audit/reducer_review/numeric_stage01/report.json).

Seven pure composition/frame/source contract groups pass. Independent raw and
numeric assertions validate the saved evidence; they are not additional robot
trials or a full repository test-suite run. Pure metric checks repeated across
numerical environments are not added to historical totals. No runtime/config/
default source changed. Unsuccessful setups and reviewed source versions are
retained, including the legacy metadata-schema error, reducer draft coverage
corrections, native process crash and resolver-only analysis-source stage.

[Tracking plot](../../../../results/contact_scheduling_joint_tracking_20261126/analysis/stage01/tracking_all_phases.pdf)
· [Turn plot](../../../../results/contact_scheduling_joint_tracking_20261126/analysis/stage01/turns_all_phases.pdf)
· [Joint-rate comparison](../../../../results/contact_scheduling_joint_tracking_20261126/analysis/compact_stage01/joint_tracking_all_phases.pdf)
· [Compact scientific notes](../../../../results/contact_scheduling_joint_tracking_20261126/analysis/compact_notes_stage01.md)
· [All-case overview video](../../../../results/contact_scheduling_joint_tracking_20261126/videos/scheduling_joint_tracking_overview.mp4)
· [Compact inventory](evidence/manifest.json)
· [Source/data/video bundle](../../../../results/contact_scheduling_joint_tracking_20261126/presentation-pack.zip).

The overview shows fixed chapters for −1.00/+0.30/+0.90, with all four arms in
each chapter, equal camera/plot scales and the full recorded 0–44 s program.
It plays at 4× using 200 ms saved poses and labels both playback and recorded
clocks. Scientific plots retain all 5 ms samples. It is an overview rather than
a fine twitch/timing replay. No physics is integrated for the movie. Per-frame
phase labels say requested lookup phase, since gains can be held in dropouts.

## Reproduction

Use the Gazebo environment in [GAZEBO.md](../../../../GAZEBO.md), the original
terrain assets and frozen source/input hashes. Preserve all existing results.
The original family supports a read-only registration check:

```bash
python results/contact_scheduling_joint_tracking_20261126/run_factorial_case.py --preflight
```

For a new native campaign, create a fresh output root with copies of the
locked `protocol.json` and `campaign/jobs.json`. Run one fresh process per
case, for example:

```bash
python results/contact_scheduling_joint_tracking_20261126/run_factorial_case.py --output results/new_i52 --run --case-id p090_NR --cap .3
```

The twelve IDs and exact fields are in the protocol/jobs. Existing case
directories are refused. For analysis, register explicit paths for the new
cases before reducing; do not silently reuse the original case-path resolver.
The original scientific reducer is preserved as `analysis/analyze_scheduling.py`;
the separately reviewed resolver-only version is
`analysis/analyze_scheduling_stage02.py`. Its `--output` must name a fresh stage.
Native process argv, source/input snapshots, versions and model/cache hashes
are recorded per case. The bundle supports review/reduction of saved data;
native regeneration additionally requires the documented simulator environment
and terrain assets. Generated worlds and prior presentation archives are not
recursively bundled.
