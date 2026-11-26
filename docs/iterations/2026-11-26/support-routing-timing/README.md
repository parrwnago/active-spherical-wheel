# I-46 — current support routing and observer timing

2026-11-26. **Executed and analyzed; retain the existing filter.**
[Video: actual saved Mars/flat states with shadow diagnostics](../../../../results/support_routing_timing_20261126/videos/support_routing_timing.mp4)
· [Presentation/evidence bundle](../../../../results/support_routing_timing_20261126/presentation-pack.zip).
The geometry and traces in the video come from the completed native Gazebo/DART
runs, rendered from saved states; alternative routing never drives the robot.

The user asks to return to the existing LQR/momentum-observer/contact-feedforward
stack and whether actuator misassignment from delayed contact is inevitable.
The recommendation is to keep the tested observer-only baseline. The camera/map
branch was already offline and remains parked. The user's request to return to
the baseline is taken as the operational direction; no additional estimator is
introduced by this record.

## What is active

The selected recent baseline is `mode='hardware'`, least-squares rolling
allocation, synchronization enabled and tilt shaping disabled. Generic
`OffroadParams()` still defaults to `terrain_aware`; this description applies
to the selected observer-only experiments, not every repository mode.

- The momentum observer estimates external force/moment. The support locator
  uses that wrench and current robot geometry to infer shell/belt/rim records.
- Those inferred identities/points enter rolling feedforward; inferred shares
  weight scheduled LQR gains and static support-torque routing. Native
  least-squares rolling constraints are not numerically load-share weighted.
- A separate filtered normal affects lean, ground-following references and
  support torque. Filtered total load affects low-load fading.
- The **active 120 ms constant-velocity synchronizer** previews geometric
  exposure and prepares unloaded incoming drives. It does not predict exact
  future contact forces/load transfer or replace current-support feedback.
- The 200 ms dynamic contact forecast and four-camera forecasts remain offline.

Source: [controller flow](../../../../src/spherical_wheel/offroad_control.py:550),
[gain/holding path](../../../../src/spherical_wheel/offroad_control.py:768),
[rolling allocation](../../../../src/spherical_wheel/experiments.py:79),
[active preview](../../../../src/spherical_wheel/synchronization.py:154).

## Why disagreement is not a fixed delay

The observer computes a preceding-control-interval mean wrench and filters it
at 100 rad/s in this baseline: a nominal 10 ms first-order time constant, not a
measured fixed 10 ms latency. The control interval is 5 ms. The locator combines
the filtered wrench with **current** joint/body configuration. A filtered wrench
mixes history, while shell/belt boundaries move with configuration. Thus support
selection can lead, lag or chatter; a universal time shift is not established.
The additional normal filter is not the filter directly producing these
support records. Changing its time constant alone does not directly correct
load-share routing.

[I-41](../../2026-11-25/figure8-event-audit/README.md) records belt A inferred
while shell A actually carries load at 6.725 s, then shell A inferred while belt
A carries load at 6.860 s. This is not consistently late classification. A
causal motion/force observer cannot confirm an unforeseen force transfer before
it affects its inputs, but wrong large actuator commands need not be accepted
as unavoidable. Controller handling of short-lived ambiguity matters.

Sources: [interval/filter update](../../../../src/spherical_wheel/contact_sensing.py:330),
[current-configuration locator](../../../../src/spherical_wheel/contact_sensing.py:429).

## Original bounded audit proposal

Keep live commands unchanged. Reconstruct the observer's existing interval-mean
wrench and filtered wrench from saved measurements around the 6.4–7.4 s event.
Run the same locator at the same current configuration for both. Compare route
change timing, boundary margins, LQR gain/holding-torque routing and command
steps; retain both premature and late transitions. Use truth only for scoring,
with its last-physics-step convention distinguished from interval averages.
Broader transfer examples and noise sensitivity are required before calling
the faster signal a general improvement.

This tests whether wrench filtering materially creates the routing mismatch
using already available robot-like signals. It is not a new estimator, a live
fast-signal switch, a fixed time advance, or an assumed cure. Exact numerical
protocol and acceptance criteria must precede execution.

If this identifies a useful cue, a separate candidate can distinguish fast
transition detection from smoothed force/normal magnitude and test how the
existing controller tolerates uncertain support. Existing gain blending and
incoming-drive preparation must be accounted for; neither is a missing feature
to add blindly. Simply holding the old mode, adding hysteresis or blending both
drives can increase delay or disturb balance through unloaded-drive reaction.
Any new handoff rule needs a bounded same-state/simulation comparison.

Crucially, [I-42](../../2026-11-25/handoff-factorial/README.md) corrected feedback
shares without preventing the first departure. Before it, belt demand was
already about 0.835 m/s against the assumed 0.300 m/s limit. Earlier interval
truth-share tests in [I-25](../../2026-11-25/support-identity-probe/README.md)
also failed to fix another excursion. Do not promise that faster routing fixes
tracking or bypass reference/actuator feasibility. Stop pursuing this as the
explanation of that departure if the routing correction arrives after demand
is saturated or leaves the relevant command discontinuity unchanged.

Proposal-stage decision: clarify current architecture and test the timing
premise before changing control. At that stage no code or experiment had run.
The approved execution and observed result are recorded below.


## Registered execution — 2026-11-26

The user approved this audit and asked for a suitable map area. The original
proposal above is preserved; this follow-up records implementation and execution
without claiming a completed result. Live LQR, momentum observer, rolling
feedforward, normal filtering and incoming-drive preparation remain unchanged.

**Site selection before controller outcomes.** Scan 2,091 route centers in the
Mars DEM, excluding rocks/lava within a 0.22 m protected capsule around a 0.18 m
route. The first 2–6 degree slope / 1 mm RMS / 3 mm maximum planarity screen
returned zero candidates. An independent dense-sampling check found no fitting
bug. The explicit pre-control revision retained the grid, clearance, slope and
ranking, relaxing planarity to 3 mm RMS / 10 mm maximum; 16 sites qualify. Keep
the failed stricter screen as evidence, rather than calling this terrain flat.

Selected center **(-2.3, 8.4) m**, route heading 45 degrees, fitted slope
4.100 degrees, along-route grade -1.711 degrees, RMS roughness 2.776 mm,
maximum residual 8.845 mm. Nearest rock/lava is 1.176 m from the route segment.
The fit describes the protected corridor, not each individual triangle.

**Cases.** Replay the entire original 44 s Mars figure-eight history, scoring
the registered 6.4–7.4 s event. Run two fresh 16 s Gazebo native-controller cases:
a slow 18 cm out-and-back at the selected Mars site and a flat-plane companion,
with common phase 1.10 rad, yaw 45 degrees, 6 s quintic legs starting at 1 s.
The phase/route geometrically permit both shell-to-belt and belt-to-shell
transfers, but actual occurrence is an acceptance requirement, not an assumption.
The companion has different surface geometry and balanced initial lean; it does
not isolate slope as a single causal variable.

**Measurement and implementation.** New audit-only files compare filtered and
preceding-interval mean observer wrench at identical current configuration.
Alternative gain/holding-torque commands are computed in shadow only. The
original native feedback is called exactly once; all references, rolling
feedforward, synchronization, heading and applied commands follow that path.
The source observer interval mean is an inferred quantity available from the
same state/input history, not contact ground truth. Exact body state is still
simulated, as agreed. Historical truth is last-1-ms contact; fresh runs also
record the five completed 1 ms contact states, then average normal loads before
normalizing. Moving-center moment frames and body-contact exclusion are explicit.

Two fresh-process 0.1 s bridge fixtures must establish unchanged endpoints for
one held `run(5)` versus five held `run(1)` calls before any route collector can
start. The collector verifies a passed, source/protocol/plugin-hash-bound QA
report. This fixture is not a controller trajectory experiment.

**Registered gates.** Both fresh cases must complete, remain within the
selected protected corridor and have common matched transitions in both
directions. Require median paired absolute event-timing improvement of at least
5 ms; mismatch equivalent seconds decrease by both 20% and 10 ms in each case;
no material increase in false transitions, normalized requested command steps
or paired local wrench-noise label flips. The mismatch duration is a sampled
preceding-interval discrepancy measure, not observed actuation of the shadow
controller. Any success only licenses a later bounded live-candidate test.

**Implementation validation.** 24 unique focused audit tests passed in both
numerical environments. Repeated environments/stages are not additional tests.
An earlier test stage passed its cases but saw a driver change during addition
of the required collection gate; it is retained and superseded by the final
unchanged-source stage. The 30 existing observer/hardware/synchronizer regression cases also pass
in both environments: 54 distinct selected tests total, not the full repo suite. No live faster routing, controller improvement or real-hardware
result is established.

Registered compact evidence: [protocol](evidence/registered/protocol.json),
[clarifications](evidence/registered/protocol_clarifications.json),
[jobs](evidence/registered/jobs.json).
Raw local family: [results](../../../../results/support_routing_timing_20261126).


## Observed result and decision

Both native cases completed 16 s, each with 3,201 controller observations and
3,200 preceding intervals. Each interval has five raw post-solve physics samples.
The Mars center stayed within 25.2 mm of the registered segment, flat within
10.8 mm; both satisfy the 100 mm exposure guard. Native timed tracking RMS is
13.27 mm on Mars and 7.47 mm on flat; final errors are 0.396 mm and 0.083 mm.
These describe the unchanged native controller on this slow compact program,
not improvement from the shadow signal. Mars absolute platform tilt reaches
6.61 degrees; maximum attitude error relative to its lean reference is 1.92
degrees. Absolute verticality and tracking the scheduled reference are different
measurements. Both directions occurred in
each case. There are **seven common matched stable family transitions**, three
on Mars and four on flat. Median absolute event-timing improvement is **0 ms**:
only one Mars belt-to-shell event improves, from 10 ms early to 5 ms early.
Several other labels also lead truth, rather than consistently lagging it.

| Scored case / truth | Filtered routing mismatch | Interval-mean routing mismatch | Largest normalized requested step |
| --- | ---: | ---: | ---: |
| New Mars, mean preceding 5 ms | 5 ms | 0 ms | 0.39916 → 0.39916 |
| New flat, mean preceding 5 ms | 10 ms | 10 ms | 0.001515 → 0.001515 |
| Historical figure-eight 6.4–7.4 s, last 1 ms | 90 ms | 55 ms | 2.78247 → 2.78247 |

Mismatch is the sampled unique-patch routing-disagreement equivalent duration,
not time that an alternative controller was actually applied. The fresh cases
are scored over [0.5,15.5); the historical convention differs and is not pooled
with fresh timing gates. The native and alternative commands in this comparison
are same-state, pre-synchronization gain/holding sensitivities. The live input
never changes. Maximum steps are normalized per actuator bound so torque and
speed are not added as though their units were interchangeable.

The descriptive comparison against the *following* held interval on native
history is Mars **5→5 ms**, flat **0→0 ms**. Thus the Mars 5→0 ms reduction
against preceding-mean truth does not demonstrate improved routing of the command
about to be held. Future truth is an evaluator-only lookup, and the fast controller
is not executed. Historical source-specific event medians have unequal match
counts; only common paired events support a timing comparison.

Both fresh runs have zero extra stable family switches under the registered
rule. The local paired wrench-noise label flip rates are 0.325% → 0.304% on Mars
and 0.200% → 0.200% on flat; historical is 5.875% → 5.813%. These are offline
perturbations at fixed pose, not propagated hardware sensor noise. The exact
unperturbed locator load vectors reproduce before perturbation. Raw label
sensitivity, gain/holding sensitivity and true interval loads remain separate.

**Historical relevance.** The first actual native belt-A request at its bound
is 6.700 s. The first truth-correcting effective-route cue, and first shadow
command difference, are 6.725 s. The maximum requested step is unchanged. Thus
this faster signal does not explain away the first saturated departure. It
helps some historical classifications without satisfying the necessary timing
and command tests for that explanation. This is consistent with earlier I-42
true-share routing failing to prevent departure.

**Registered decision: retain_filter_no_integration.** Coverage, completion,
false-switch and command/noise nonregression conditions pass. The median timing
gain and required per-case mismatch benefit do not. Baseline disagreement in
these gentle cases is already small, so this result is evidence against this
specific fast-routing change, not proof that every future handoff is reliable.
No live candidate, new estimator, camera feedback, particle filter or tracking
improvement is adopted. Keep LQR/momentum-observer/contact feedforward and the
existing incoming-drive preview. For a later control improvement, return to the
measured reference/actuator saturation and turn response rather than assuming a
fixed observer delay is the dominant cause. Preserve the successful simple
route as a handoff regression case.

## Failures and checks retained

- The strict geometric selection returned zero candidates. The practical
  geometry revision preceded every controller outcome and remains explicit.
- The first flat attempt failed before server/controller creation because a
  zero-coefficient plane *feature* still makes `Terrain.is_flat` false. It has
  zero command samples. A separate, preserved driver copy uses featureless
  terrain for the same registered z=0 plane and writes a fresh result directory.
  Its exact diff, flat-only guard, AST comparison and preflight are saved.
  The failed attempt and original frozen driver were not rewritten. The generated
  flat model/cache workspace was reused; there was no simulator state to reuse.
- An early audit test stage observed a source change while the strict collection
  gate was added; the final unchanged-source stage passed the same cases.
- Independent reducer review caught an exact-floating-point tie choosing the
  later event. Before any raw reduction, the author preserved that source,
  corrected ties within the clock tolerance to choose earlier, and passed
  eleven independent synthetic checks. The first scientific reduction is sealed.
- The independent raw-truth checker first failed JSON serialization on a NumPy
  boolean. The original source/log are retained. A new stage changes serialization
  only and reconstructs all 32,000 saved route substeps: mean loads and wheel,
  body and all-external wrenches match exactly. No physics was rerun for this fix.
- Two fresh bridge fixtures match all 21 endpoints and all contact fields exactly.
  Historical replay reproduces all 8,801 requested/applied commands, observer
  wrenches and reference arrays bit-for-bit. Native feedback runs once per call;
  every shadow history and snapshot remains unchanged.

Preserved compact report: [scientific report](evidence/completed/analysis/stage01/report.json).
Compact evidence inventory: [origin/hash manifest](evidence/completed/manifest.json).

Scientific report: [local raw report](../../../../results/support_routing_timing_20261126/analysis/stage01/report.json).
Paired event table: [CSV](../../../../results/support_routing_timing_20261126/analysis/stage01/paired_events.csv).
The 54 selected unique pytest cases pass in each environment; the eleven reducer
checks and physical fixtures are reported separately, not added to that count.

## Reproduction

Run the numerical replay with the project environment and Gazebo collection in
the repository's Gazebo environment (see [Gazebo setup](../../../../GAZEBO.md)).
The original output family refuses overwrites; use a new family containing the
registered protocol/jobs and recorded inputs. Results are tied to exact saved
sources, plugin binary, map and gain-cache hashes; a new run is a new experiment.
The flat correction is a separate execution source, not a change to the
original completed Mars source.

```bash
python scripts/audit-support-routing.py --output NEW_FAMILY --historical --tag stage01
python scripts/audit-support-routing.py --output NEW_FAMILY --bridge-fixture batch --tag stage01
python scripts/audit-support-routing.py --output NEW_FAMILY --bridge-fixture single --tag stage01
# Independently compare/bind both fixtures before any collection.
python scripts/audit-support-routing.py --output NEW_FAMILY --collect mars_compact_outback
python results/support_routing_timing_20261126/setup_correction_stage02/audit-support-routing-flat.py --output NEW_FAMILY --collect flat_compact_outback
python results/support_routing_timing_20261126/analysis/reduce_routing.py --help
```

The strict collector proof binds original source paths; reproduce with the
preserved layout/provenance or register a new proof for new source locations.
Large raw arrays and media remain in ignored `results/`; compact reports,
source snapshots and hash inventory are preserved beside this record.
