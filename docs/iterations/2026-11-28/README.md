# Research presentation notes — 2026-11-28

**Latest: I-86 narrows baseline recovery to a single-support balance transition.**
[Report, force/recovery plot, PDF and slow-motion video](baseline-recovery-trace/README.md).
One unchanged B060 prefix reproduces all 79 fields through 106.5 s and captures
1,300 physical 1 ms samples. All load is shell A, within a 1.225 mm contact
manifold; no second loaded support explains this event. Isolated contact-free
solves occur at 105.525 and 105.724 s. Whole-COM force balance closes closely,
and the momentum observer agrees with the corresponding physical wrench.
The later velocity excursion grows while the native lean target changes: at
105.745 s wheel-centre/COM X velocities are -122/-56 mm/s, with a large relative
motion term. Desired-up rate is 24.7 degrees/s but the angular-rate reference
is zero. This supports a targeted balance-response investigation, not an
estimator replacement or proven cure. A native-angle/rate-only ablation is
proposed, not run; retain I-43's mixed smoothing/rate results. Baseline kept,
terrain assistance parked, no multi-contact expansion or default change.

**Latest: I-85 fails recovery benefit; park terrain assistance per the user.**
[Video, PDF, full comparison and baseline follow-up](terrain-lean-trial/README.md).
One fixed native/terrain-lean pair completes2×133s at the same0.60capacity;
fresh native matchesall79oldB060fields. Extra lean peaks.480° with actual
mapped observations. TrackingRMS23.362→21.809mm andpeak72.167→65.163mm
improve, but uninterrupted2s recovery entry117.675→118.325s is.650s LATER,
failing the≥1sbenefit requirement. Both stay goodafterward.36/37checks pass;
canonical/independent and legacy/corrected persistence decisions allSTOP.
Keep the native LQR/observer/feedforward, no tuning/retry. Old-baseline logs
point to a105.53s contact/normal transient before velocity-band failure while
commands are unsaturated, contact identity mostlycorrect and joint references
small; complete interval-force causality remains unproved. Baseline velocity
recovery is the next focus, with no new estimator or camera expansion.

**User-confirmed priority: baseline recovery first; multi-contact later if
needed.** The next diagnostic is proposed, not run: capture 1 ms contact-force
intervals and held commands around the existing 105.53 s event to separate the
physical disturbance from subsequent control recovery. Do not repeat target
smoothing as an untested idea; I-43 already found mixed results. Existing
[I-04 fixture evidence](../../../ITERATION_LOG.md#i-04--check-the-multi-contact-limitation)
shows genuinely loaded multiple supports, but does not establish their role
in this event or demonstrate multi-contact traversal. A sphere can roll along
a V-groove with both faces loaded; multiple contacts restrict available motion
without necessarily eliminating it. Defer estimator expansion until separated
loaded patches materially affect recovery, rather than counting solver contact
points. No further physical run or controller change accompanies this decision.

**I-84 sees future terrain but preparation is inactive in this event.**
[Observed-map preparation audit](terrain-recovery-preview/README.md).
Four-camera memory supplies accepted future centre/foot fits at all261focus
and901dense anchors, but both native and mapped preparation blends are0
through99.5–104s. Native reconstruction/current load/velocity/margin guards
have0error. The predeclared action gate fails0/261 versus required10; honor
its stop before a zero-action live trial. This is not a camera-visibility
failure or rejection of terrain information. Map-informed loaded-motion
references remain a different proposed experiment. No new physics is run.

**I-83 finds earlier return but mixed sustained-recovery timing.**
[Recovery definitions](recovery-assessment/README.md) ·
[Observed-map preparation proposal](terrain-recovery-preview/README.md).
Existing I-82 recordings give first acceptable path motion9.460→3.380s after
the100.5s anchor at0.30→0.60m/s. The0.60firstreturn lasts1.705s; its primary
two-second-dwell entry is.720s later (116.955→117.675s absolute), then stays
good through133s while0.30leaves again. Both retain rotation and never satisfy
the stricter full-motion bands. Independent raw reconstruction matches60signal
arrays,10sample-exact predicates and1,317numerical ledger leaves. No newphysics.
I-84's subsequent fixed0.60capacity audit does not demonstrate useful action
through the unloaded-preparation interface in this event.

**Latest: I-82 raises the belt operating limit and improves the current eight.**
[Full comparison, slope-event video, two-page PDF and evidence](current-belt-capacity/README.md).
Two full 133 s Gazebo/DART runs retain the original LQR gains, momentum observer,
inferred-contact feedforward and preparation; only physical/mirror operating belt
bounds change from 0.30 to 0.60 m/s. The fresh 0.30 arm reproduces all 79 original
I-76 arrays exactly. Moving timed RMS falls 46.36→23.36 mm, full timed peak
336.41→72.17 mm and final distance 32.88→0.88 mm. All 111 setup and 30 fixed
performance checks pass; independent scoring agrees. The larger-limit arm
nearly stops translating but retains about 6.1°/s rotation and fails strict
four-condition settling. It is a leading capacity variant for this case,
without automatic default promotion. Commands first differ at 80.880 s, so the
later downhill force comparison reflects different histories, not a same-state
100.9 s intervention. Earlier I-49's different-route mixed result stays retained.
The requested full 1× movie and 98–104 s excerpt are checked and shareable.

**Research objective clarification:** Allow temporary terrain-induced departure
and evaluate recovery time as well as peak/whole-route error. Stored terrain
could support anticipation while the momentum observer checks actual contact.
I-44/I-45 were offline map/contact diagnostics, not live recovery-control tests.
Recovery bands, event anchors and dwell criteria remain to be registered; no
new recovery score, simulation or map-assisted controller is claimed. Compare
policies at the same operating limits to separate knowledge from capacity.

**I-81 identifies loaded-belt downhill force pulses.**
[Report and two-page PDF](slope-response-trace/README.md).
One unchanged native102s prefix exactly reproduces all79 original precision
fields and captures1,300 new1ms contact samples plus261 full preview snapshots.
At100.900s ground adds10.598N downhill alongside3.234N gravity; whole-COM
momentum confirms the sum. BeltA is actually loaded and inferred correctly.
The preceding−.889m/s belt request clips−.300, preparation adds zero, and LQR
roll/pitch correction is the largest command block. Other intervals brake.
Thus neither gravity alone, wrong incoming preview nor misassignment alone
explains these pulses. Loaded-belt feedback is a specific next causal-test
target; no intervention, controller cure or default promotion is claimed.
Observer-wrench agreement is close in this window; current pressure, interval
pressure and filtered inference remain distinct. Independent numerical audits
and exact source/data bundles preserve the evidence and setup failures.

**I-80 checks the user's exact 101.5 s figure-eight frame.**
[Descent review, frames and PDF](slope-edge-review/README.md).
The native robot really descends an8.644° slope: measured vertical velocity
matches terrain-height decrease along its motion, with pressure retained at
every saved99–103s sample. Describe supported downhill motion with loss of
path tracking, rapid rotation and saturated correction. This does not by itself
prove false contact or gravity-only cause. The logs also show rapid belt/shell
routing changes near the frame; instantaneous pressure and filtered inference
have different timing. A larger grade break occurs after substantial acceleration
has developed. No new physics or controller change; causal contributions remain
unisolated.

**I-79 clarification: a terrain-induced preview failure is plausible, unverified.**
[Evidence review and narrow proposed diagnostic](terrain-contact-hypothesis/README.md).
The 120 ms preview extends current motion over a frozen estimated plane, so a
new slope can defeat it. It prepares incoming drives rather than declaring
current loaded contact. The eight's actual normal changes rapidly even while
the observer follows the filtered physical wrench closely. Wrong prediction,
filtered-normal timing and difficult response to a real change remain separate
mechanisms. Existing recordings omit full preview targets, so the complete
prediction-error/causality chain is not established. No new runs or code changes.

**I-78 tests coherent path retiming; one start improves, the candidate
fails its fixed comparison.** [Full report and videos](path-progress-governor/README.md) ·
[two-page PDF](path-progress-governor/evidence/evaluation/publication_stage01/progress_comparison.pdf).

- Keep original native N LQR, momentum observer and contact feedforward selected.
  Only an isolated virtual path clock changes; production code/defaults do not.
- Six fresh Gazebo/DART straight runs use three shaft starts. Native replays
  exactly match all79 original precision fields. G finishes the same path with
  0.985/0.600/0.005 s added travel time and a full ten-second goal hold. None stalls.
- Peak geometric departures N→G are148.9→102.0,124.6→124.4,59.7→59.2 mm.
  At−1 rad, clipping falls0.680→0.240 s; residual large demands remain.
- All180 setup checks pass. Two of82 performance checks fail: +0.3 peak tilt
  rises9.397→9.970° and speed0.1819→0.2284 m/s. Only G−1 meets all four settling
  conditions; approaching the goal alone is not full settling.
- Independent raw-data audits reproduce564 scalar leaves and all82 gate
  decisions. They are checks of these six recordings, not additional physics or
  a full-repository test count. No post-outcome tuning or default promotion.
- The governor is reactive, not a future contact forecast or constraint
  guarantee. +0.3 speed peaks while desired speed is lower and commands are
  unsaturated at that instant; its earlier altered dynamics remain relevant.
- I-77 establishes reference-window contributions; neither I-77 nor I-78 proves
  that the advancing path clock is the sole cause. Retain both benefit and
  regression in the presentation.

## Previous causal investigation

**I-77 returns to the original baseline and tests causes before another design.**
[Full investigation](../2026-11-27/baseline-deviation-cause/README.md) ·
[straight causal comparison](../2026-11-27/baseline-deviation-cause/evidence/component_audit/causal_publication_stage02/causal_comparison.png) ·
[aligned contact evidence](../2026-11-27/baseline-deviation-cause/evidence/contact_trace/analysis_stage01/aligned_contacts.png).

- Original gain-scheduled LQR, momentum observer, inferred contacts,
  rolling/holding feedforward, preparation and final caps remain selected.
  Heading helpers and prior estimator/allocator candidates stay experimental.
- Straight −1 has nearly parallel drive directions (0.389°), not simply the
  wrong contact label. The recorded pressure point still yields a badly
  conditioned inverse. Three 24 s exact-prefix diagnostics show a 250 ms
  feedback-reference intervention reduces later geometric peak from 148.934
  to 21.979 mm over 20.4–24 s. Removing the maximum 5 ms pulse alone gives
  150.665 mm. At the 20.650 s primary endpoint the cluster treatment is slightly
  worse. Preserve both the delayed benefit and the immediate adverse result.
- Forty-three reference/applied rows change in the cluster treatment. The
  ±5 rad/s diagnostic uses the old normalization scale; it is not a motor
  rating, a globally tested controller or a new default. Native24 and both
  intervention prefixes receive independent all-field verification.
- Eight −1's late near-path crossing retains velocity error. Trace farther back:
  exported pressure-weighted solver normals change rapidly, the observer closely reconstructs
  signed interval wrenches, and the normal-derived target moves rapidly while
  angular-rate reference remains zero. Estimated normal, contact radial
  direction, surface normal and nearest-centre DEM tangent must stay distinct.
- The unchanged eight replay captures 1,500 substeps over 300 holds and matches
  all 79 recorded precision fields through 97.5 s, with its explicitly shortened
  terminal held-command mask. Filtered physical-wrench direction vs observer:
  max 0.02345°, RMS 0.00974°. This is a qualified window-specific result.
- Preserve failed interpreter-path test setup, velocity-report bookkeeping,
  reference-attribution equilibrium anchor, and prephysics horizon/source
  corrections. Three scoped hook functions are counted once; the full repository
  suite is not rerun. No full-route acceptance is claimed.

The final attitude hold/release gives no immediate path improvement, but reduces
later geometric peak over 96.4–103 s from 316.714 to 62.191 mm. Release adds a
0.491 N m shaft-command step, so its effect is not isolated from holding. All
79 pre-intervention fields and all 20,601 feedback-error/LQR rows verify.
Saved original-native, fixed held and current branch-local attitude targets
must be evaluated separately: larger saved-target error alone is not a balance
failure once trajectories differ. No new controller is accepted. All five
targeted physical diagnostics finish their requested shortened horizons.
