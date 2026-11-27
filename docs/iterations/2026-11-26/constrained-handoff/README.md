# I-56 — bounded balance and drive handoff

2026-11-26. An opt-in experiment after native gain-scheduled LQR, informed by
the [I-55 causal diagnostics](../causal-replay/README.md). Existing runtime
defaults, observer, reference generator, incoming-drive preparation and gain
bank are unchanged. This is a command allocator, not a new LQR design or MPC.

## Question and implementation

Can total-command bounds and continuity reduce support-transfer transients
without sacrificing balance and route tracking? Can the remaining command
freedom reduce world-vertical rotation and unnecessary unloaded-shell momentum?
Compare against native **B**, rather than the rejected reference guard **R**.

The native controller runs once. Its total request is clipped to the existing
amplitude box to define the nominal command. A two-stage optimization then:

1. Finds the closest available transverse body angular-velocity response to
   that nominal command, subject to total amplitude and actual-previous-command
   slew bounds.
2. Preserves the attained first-stage response through a linear equality,
   then penalizes travel-response changes, world-vertical rotation, unloaded
   relative shell momentum, native-command deviation, command change and effort.

The five amplitude caps remain `[1.5, .6, .6 N m, .30, .30 m/s]`. New slew
assumptions are `[60, 60, 60 N m/s, 6, 6 m/s²]`, applied from the first command
using the actual previous applied input. These are simulation assumptions,
not hardware specifications or bounds on actual robot acceleration. If the
rate box prevents native balance response, stage one records that loss; stage
two must add no further loss within its numerical tolerance.

A private measured-state rigid-body model supplies the local input-response
matrix. It uses current observer-derived patch locations, loads and normals,
not Gazebo contacts or a terrain map. A load-weighted soft contact penalty
relates material-point motion to ground. Tangential confidence decreases with
the existing estimated slip residual. The optimizer uses a **differential**
response over 20 ms, not an absolute prediction of future contact or motion.
Its priority preserves two local angular-response components, not a proof of
nonlinear balance stability. Friction, unilateral contact and onset prediction
are not certified by this model.

Existing 120 ms incoming-drive preparation supplies readiness and shell-rate
targets. A ready unloaded drive receives a stronger native-reference cost;
the vertical relative shell-momentum term fades with support load. No blanket
motor disabling or held attitude reference is used.

## Frozen comparison and gates

Initial fresh processes: native `m100_B`, bounds/continuity ablation `m100_A`,
and full candidate `m100_H`, all 44 s on Gazebo/DART Mars yard, plain
figure-eight, shaft −1.00 rad and yaw 45°. The ablation disables only the new
yaw and shell-momentum weights. Controller measurements remain exact simulated
body/joint state plus the existing momentum observer and support locator.

Registered before reading outcomes: require completion, exact native historical
parity, command/equality/boundary checks, at least 10% lower late peak yaw rate
and 20% lower late absolute heading travel, and nonregression gates on timed
tracking, fixed turn overshoot, relative attitude, absolute tilt and settling.
Only if the full −1.00-rad candidate passes every gate do the +0.30/+0.90 pairs
run. No automatic default promotion follows even a passing small screen.

## Development evidence and limitations

- One frozen I-55 state validates the unloaded-shell torque sensitivity:
  predicted first-5-ms yaw-rate change −0.271855 rad/s versus measured
  −0.270834, a 0.377% difference. This validates one column at one state;
  it does not validate loaded-belt response or a full trajectory forecast.
- The first readiness fixture exposed an excessive soft command-change cost:
  the incoming belt reached only 0.20598 m/s after 120 ms toward 0.30.
  Normalize this soft cost by input caps while retaining the separate hard
  slew box; the corrected fixture reaches at least 0.297. The failed source,
  assertion and later correction are preserved separately before physics.
- The sensitivity setup initially assigned a three-vector to a six-component
  surface-velocity row. The failed setup and correction to its translational
  slice are retained; it produced no valid sensitivity outcome.
- Final focused verification: 21 model tests and 20 allocator tests, 41 total;
  nine separate read-only preflight contracts. Earlier subsets/repeated runs
  are not additional tests or a full repository suite.
- Final-command clipping is zero by construction for a bounded allocator.
  Use active caps/rates, preserved native infeasible demand and actual motion
  to judge performance. A bounded request alone is not a dynamic improvement.
- Eight separate saved-data metric fixtures pass. Their initial pytest launch
  fails before collection because an unrelated ROS plugin lacks `lark`;
  disabling plugin autoload allows the unchanged fixtures to run. The failed
  launch is retained. Video setup also retains an overly strict float64/float32
  serialization comparison and its dtype-corrected check; the failed attempt
  rendered no poses. Neither setup correction changes the physics outcomes.

## Outcome: reject this candidate

All three jobs complete 44 s. Native B reproduces all 56 historical precision
fields exactly and all jobs use the same numeric gain bank. H and A satisfy
their amplitude and actual-prior slew bounds: worst slew residual is
`5.973e-14`. H's secondary balance-equality residual is `2.499e-13`, with
at most `5.011e-13` extra transverse response change. No solver recovery or
primary restoration is used. These technical checks pass; performance does not.

| Metric | B: native | A: yaw/momentum weights zero | H: full |
| --- | ---: | ---: | ---: |
| Moving timed XY RMS, mm | 51.080 | 57.152 | 55.135 |
| Peak timed XY error, mm | 116.256 | 183.974 | 136.157 |
| Final timed XY error, mm | 42.648 | 15.470 | 76.723 |
| Peak raw-target lean error, ° | 7.399 | 13.426 | 13.849 |
| Peak absolute tilt, ° | 13.718 | 15.039 | 18.964 |
| Late peak physical world-Z rate, rad/s | 2.443 | 10.013 | 8.753 |
| Late absolute heading travel, turns | 1.800 | 1.081 | 1.542 |

Moving means 1–41 s; late means 28–44 s. Other peaks cover 0–44 s.
Absolute heading travel sums rotation in either direction; it is different
from final-minus-initial heading. All original raw target formulas remain
unchanged, but targets naturally differ after the policies change robot state.
They are not counterfactual same-state comparisons after divergence.

H fails eight registered conditions: late yaw peak, late heading travel,
peak/final XY error, peak/RMS relative lean, absolute tilt and the combined
six-turn nonregression condition. Heading travel falls 14.4%, below the required
20%, while its peak rate more than triples. Turn six overshoot increases
20.416 mm. Moving RMS passes its declared tolerance, which does not rescue the
other failed gates. A also fails despite a better endpoint and smaller heading
travel. Its failure shows that removing the auxiliary yaw/momentum objectives
does not make this allocator acceptable; it does not isolate every mechanism.

**Do not adopt either candidate.** The conditional +0.30/+0.90 runs were not
launched, not successful unreported trials. No post-outcome tuning or alternative
candidate was substituted. Keep native LQR as the selected research controller.

The important distinction is between satisfying a local command optimization
and preserving physical balance. The hard slew box sometimes prevents the
native modeled balance response: H's maximum first-stage transverse discrepancy
is 3.741 rad/s in its 20 ms response model. Stage two preserves that degraded
response correctly. This does not establish that rate bounds alone caused the
whole regression. Neither the local contact model nor its preserved two response
components prove stability over support changes. Actual endpoint-derived
acceleration is also higher despite bounded command changes. Before another
variant, validate loaded-contact sensitivity and quantify the cost of the
rate box at the failing events. Do not infer an estimator-delay cause here.

An independent saved-array audit reproduces the metrics and every gate.
Exploratory event timing adds a qualification: H's first sustained extra
20 mm of timed error relative to B begins at 4.855 s, when no slew bound is
active, both local balance residuals are near machine precision, and estimated
shell-B support agrees with evaluated support. The first modeled primary loss
above 0.01 rad/s sustained for 10 ms starts at 10.075 s. Thus the early
regression cannot be explained solely by forced slew at that instant or the
later handoff. These are different policy histories, not matched-state causal
interventions. The absolute-tilt peak at 28.705 s occurs near larger modeled
primary losses; this coincidence also does not establish a unique cause.
The [independent audit](evidence/review/outcome_stage01/audit_result.json)
retains the event values and limits. Comparing the 20 ms differential proxy
directly to an unpaired actual next 5 ms change would not validate the model:
the latter also includes native action, drift, contact impulses and geometry.

Measured H model-plus-allocation wall time is 3.69 ms at the 95th percentile,
with 153 of 8,801 calls exceeding 5 ms. These are instrumented lockstep costs,
not demonstrated real-time operation; wall time does not create simulated
computation delay in these runs.

## Read and reproduce

[Three-way overview, 4×](../../../handoff/media/i56-overview.mp4)
· [28–36 s handoff window, 0.5×](../../../handoff/media/i56-handoff-event.mp4)
· [Comparison plot](evidence/analysis/plots_stage01/comparison.pdf).
These replay actual saved Gazebo poses on the Mars DEM using display-only
kinematics; rendering executes no additional physics. The HUD distinguishes
absolute tilt from target-relative lean and shows the recorded clock.

The [canonical report](evidence/analysis/stage01/report.json),
[case table](evidence/analysis/stage01/cases.csv) and
[turn table](evidence/analysis/stage01/turns.csv) contain the complete gates.
The [protocol](evidence/protocol.json) and
[execution registration](evidence/execution_registration.json) bind the frozen
source, settings, inputs and conditional experiment count before outcomes.
The [portable evidence manifest](evidence/portable_manifest.json) inventories
the compact archive. Full raw rollouts, precision and JSON diagnostics remain
in ignored `results/constrained_handoff_20261126/`.

To replay the exact numerical metrics from a clone with Python and NumPy,
without Gazebo or additional simulation:

```bash
python docs/iterations/2026-11-26/constrained-handoff/evidence/analysis/replay_projection.py \
  --data-source docs/iterations/2026-11-26/constrained-handoff/evidence/analysis/stage01 \
  --output /tmp/i56-replayed-metrics.json
```

The projection retains every 5 ms timestamp, without downsampling, and binds
its fields to original precision hashes. The replay verifies its archive
before comparing every case metric and gate against the original report.

For fresh physics, read [the runner contract](evidence/campaign_tools/README.md)
and [teammate environment/restore instructions](../../../TEAMMATE_HANDOFF.md).
Restore archived scripts to their manifest origin paths under ignored
`results/`, use a new output root and explicitly register its paths/hashes.
The old preflight is intentionally not rewritten into a generic rerun tool;
machine-specific native plugin hashes and the frozen gain-bank checks must be
accounted for honestly. The published portable replay is the direct supported
way to inspect these exact saved outcomes.
