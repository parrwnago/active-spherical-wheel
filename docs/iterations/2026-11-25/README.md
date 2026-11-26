# Presentation notes: developing contact-aware control

Compiled 2026-11-25 from saved artifacts dated 20261124 and 20261125.
[Full iteration log](../../../ITERATION_LOG.md) ·
[Evidence manifest](evidence_manifest.json) · [Focused trial table](all_trial_metrics.csv)

Latest implementation/experiment: [I-43: bounded tilt reference and matching rate](tilt-reference/README.md).
The optional generator is implemented; 109 selected tests pass in both envs
and the disabled-feature history replay is exact. Three fresh runs show early
local RMS 38.66→29.89 mm and belt demand 0.835→0.491 m/s, but onset is earlier,
full raw-reference error rises 8.71→13.58° and four turns regress. All finish
without falling with similar absolute tilt. The declared screen fails, so
the conditional eight runs do not launch and native defaults remain unchanged.
Angle-only/raw/shaped outcomes, corrections and a comparison video are retained.

Latest completed experiment: [I-42: support routing versus incoming-drive preparation](handoff-factorial/README.md).
Four matched native-LQR runs leave the first sustained 20 mm departure at
6.860 s, with almost identical early errors. Preparation masking during 4–10 s
improves later moving RMS from 50.49 to 41.68 mm and passes the original route
gates, but fails the early-improvement screen and worsens four of six turning
overshoots. This is a tradeoff, not an adopted fix. True feedback shares also
fail to resolve the early event. Exact matched histories, negative evidence,
source/check corrections and a four-arm video are retained; defaults unchanged.
A saved-data decomposition finds common loaded-belt demand near 0.83 m/s
against 0.30 m/s, dominated by attitude/rate feedback while the desired lean
changes with zero angular-rate reference. The next proposed candidate is a
smooth attainable tilt reference with consistent tilt rate; it is not implemented.

Latest completed investigation: [I-41: figure-eight departure and turning overshoot](figure8-event-audit/README.md).
The first sustained 20 mm local departure occurs at 6.860 s amid changing
terrain, shell/belt disagreement and clipped belt commands. Paired forecasts
retain model mismatch even with hindsight actual commands, but that shadow
forecast never actuated the robot. The first turn is delayed without geometric
overshoot; later turns and a new same-reference flat run do overshoot. Initial
balancing also differs between terrains, so a particular slope's causal role
remains unisolated. Live LQR is unchanged; one new Gazebo run and 35 paired
private diagnostics, their limitations, corrected helper and video are saved.

Latest completed experiment: [I-40: LQR contact forecast and trajectory audit](lqr-contact-forecast/README.md).
Eight fresh native cases finish without falls; four of seven controlled cases
settle and the Mars route fails its separate gates. All 113 private forecasts
retain exact reconstructed 5 ms feedback history. Dynamic motion forecasts
usually improve on constant velocity at 100–200 ms, while contact warnings,
12 poor current-load fits and 270 ms median calculation cost keep the preview
diagnostic. Original and corrected warning-count analyses, source stages,
failed checks and two videos are retained; live LQR is unchanged.

Previous discussion: [I-39: LQR, iLQR and predictive contacts](../../../ITERATION_LOG.md#i-39--keep-fast-lqr-separate-contact-prediction-from-ilqr).
The user prefers fast LQR as the working baseline. Contact prediction is a
separate model capability, already present on an observer-fitted local plane.
The next proposed diagnostic is a fine-feedback native-LQR contact forecast
audit before changing commands; iLQR is a later optimizer option, not a new
contact measurement or an implemented fix. No new experiment has run.

Latest completed experiment: [I-38: unchanged seeded MPC extension](mpc-extended-return/README.md).
One 16.4 s Gazebo/DART run exactly reproduces the old 8.4 s prefix. It returns within
2.936 mm at 10.755 s, then retreats to 20.492 mm and ends 19.100 mm away, moving inward
again. No full-second settled window occurs. The added interval has matching
shell-A identities and no patch handover or actuator saturation. Controller
and observer remain unchanged; the full video and negative settling result
are preserved alongside the real near-goal return.

Latest review: [I-37: MPC is returning when the video ends](mpc-return-review/README.md).
The user's observation is confirmed by saved data: original/seeded goal errors
decrease monotonically after their peaks, ending at 31.215/28.982 mm with inward
velocity. They have not settled within eight seconds of feedback; eventual
settling is untested. This qualifies I-36's pivot recommendation: the revised
proposal was one unchanged seeded live extension before rejection on recovery
grounds. I-38 above now provides that result; earlier packs and negative forecasts
remain intact.

Previous discussion: [I-36: reconsider the next research priority](../../../ITERATION_LOG.md#i-36--proposed-priority-review-after-the-negative-horizon-result).
The recommendation is to deprioritize direct-actuation MPC tuning and explain
the existing native controller's repeatable late X failure, using its saved
pre-onset data before choosing one ablation. The error grows before saturation
and true contact switching, so neither is established as the initial cause.
This is a proposal, not a new experiment or a proven fix; earlier packs stay
frozen.

Latest completed investigation: [I-35: longer-horizon frozen comparison](mpc-horizon/README.md).
Six forecasts compare 0.4/0.8/1.2 s with full-stage derivatives and fixed weights,
cadence and refinement budget. Neither longer horizon passes; no new Gazebo
trial is launched. Seed averaging produces large fine/coarse trajectory gaps,
and conditioning deteriorates. The result limits this implementation, rather
than proving that longer lookahead cannot help. Exact sources and failed plans
are retained for presentation.

Previous investigation: [I-34: MPC search and predicted contacts](mpc-search/README.md).
The practical seed uses present observer/controller history and private future
predictions, with no recorded future commands. It improves modeled initial cost
but still ends 28.982 mm from the goal versus 31.215 mm for original MPC.
Derivative approximation, control cadence and short-horizon balance/tracking
tradeoffs remain separate findings. Contact observation/prediction are already
in the pipeline; no estimator or default controller is replaced.

Previous investigation: [I-32–I-33: feedback recovery and planning diagnosis](feedback-recovery/README.md).
The unchanged LQR/feedforward controller recovers the demonstrated drift and
settles at 0.381 mm; current MPC ends at 31.215 mm after the same eight seconds.
The offline audit finds a feasible command sequence with about 196 times lower
cost that the local MPC search missed. No controller or estimator is replaced.
Longer original LQR trajectories also expose a separate late X contact-transition
failure, so one successful recovery does not establish broad robustness.

The [I-34 proposal and completed follow-up](../../../ITERATION_LOG.md#i-34--proposed-priority-diagnose-why-the-mpc-search-drifts)
remain outside the frozen I-32/I-33 pack. LQR remains the benchmark; the new
negative closed-loop result is retained rather than replaced by the improved
frozen-model costs.

Previous investigation: [I-31: rest-goal target verification](mpc-equilibrium/README.md).
Five saved configurations fail the private-model hold check; two separate
fixed-input target pairs also drift in Gazebo. The equilibrium prerequisite
fails, so no new terminal controller was launched. Tested objective support,
failed fits, repeated diagnostic fixtures and their sources are preserved.

Previous implementation: [I-27–I-30: predictive balance-control prototype](mpc-balance/README.md).
Initial MPC improves weak-direction balance but fails the retained goal; the
record preserves the model audits, rejected gate-only trial and trajectory
refinement. These are experimental controllers with unchanged defaults.

The original [I-31 proposal and completed follow-up](../../../ITERATION_LOG.md#i-31--proposed-next-experiment-reach-and-hold-a-consistent-goal-state)
remain outside the frozen I-27–I-30 pack. I-32 qualifies that interpretation:
open-loop drift did not preclude feedback recovery. I-33 points to local MPC
search quality before further target/cost changes; no new terminal design ran.

Previous targeted study: [I-24–I-25: support timing and contact-input comparisons](support-identity-probe/README.md).
The known disagreement affects commands, but four matched runs do not establish
a successful short-window correction. Historical and comparison videos are
preserved, including regressions.

The [I-26 MPC proposal](../../../ITERATION_LOG.md#i-26--proposed-alternative-constrained-predictive-balance-control)
is now implemented/tested in the later MPC record. The I-24–I-25 pack remains
frozen at its earlier stage.

Allocation follow-up: [I-17–I-21: implemented same-contact allocation/reference experiment](fixed-contact-allocation/README.md).
The fixed-contact kinematics pass, but Mars regressions prevent promotion.
That follow-up includes the failed trials and a new comparison video. The
original sections and first bundle below remain a record of the earlier stage;
their proposed handover-first priority was corrected in I-16.

Following review of those regressions, I-22 proposed a broad diagnostic
separation. The user's reminder of prior accuracy validation led to
[I-23's priority correction](../../../ITERATION_LOG.md#i-23--accuracy-was-already-validated-correct-the-next-step-emphasis):
retain the estimator and prioritize the controller's same-contact dynamic
response, with handover assessed separately. Exact-input comparisons remain
targeted diagnostics if an event warrants them. This is a review/proposal,
not a new physics result, and is outside the frozen I-17–I-21 presentation pack.

The talk follows the decisions and learning, including experiments that were
not adopted. The main result is a better-defined failure and a reproducible
comparison process, alongside the controller experiments.

## 1. Research goal and measurement boundary — I-01

**Message:** Develop in simulation while moving the controller's inputs toward
measurements that could eventually come from a robot.

- The existing controller is gain-scheduled LQR with contact-aware feedforward.
- An estimated net wrench is different from estimated per-surface contacts.
- Explicit `hardware` mode estimates contacts, but state inputs remain exact
  simulation measurements. No robot or motor-current capture exists yet.
- Initial ramp/hills evidence was MuJoCo; Gazebo/Mars evidence came separately.
  The original observer-mode crater run failed and is retained in the record.

Show the [mode comparison in the log](../../../ITERATION_LOG.md#research-question-and-starting-point)
and [initial observer crater report](initial_gazebo/crater_rim_climb_momentum_observer.json).

## 2. Establish a broader baseline — I-02

**Message:** Completion exposed useful capability, while quality gates exposed
the remaining work.

Show [Mars trajectories](figures/trajectory_paths.png).

- 21 runs over 15 routes and selected starting phases: 19 completed.
- Only 2 passed every acceptance gate; steep descent and lava crossing failed.
- Choose a five-case study to investigate corrections before making changes.

Say: “We kept completion, tracking, attitude, and settling separate. We did not
use the successful routes alone to judge the controller.”

## 3. Instrument before changing the algorithm — I-03 and I-04

**Message:** Diagnostics separated a numerical allocation problem from contact
estimation and multi-contact limitations.

- At 31.650 s, about 0.066 m/s desired body speed produced a -1760.5 rad/s shell
  reference near aligned drive directions. This was a reference, not actual speed.
- Velocity consistency records disagreement; it cannot uniquely identify slip
  or effective-radius error.
- Two-contact tests showed that a correct resultant wrench need not reveal the
  actual contact distribution.

Use [allocation event data](allocation_event.json) and, if needed,
[velocity-consistency figure](figures/velocity_consistency.png).

## 4. First attempted fix: damping — I-05

**Message:** Removing an extreme reference did not guarantee better tracking.

Play [baseline versus damped, 0.1×, 9.05 s](../../../results/contact_diagnostics_20261125/videos/figure8_phase090_event_slow.mp4).

- +0.90 maximum joint reference: 1760.5 → 4.4 rad/s.
- -1.00 RMS tracking: 51.1 → 86.4 mm in this campaign.
- Keep the default; add explicit bounds and separate costs next.

Show [all five damping cases](figures/uniform_damping_comparison.png), rather
than presenting only the improved singularity event.

## 5. Second attempted fix: bounded, weighted allocation — I-06 to I-08

**Message:** We tested three plausible tradeoffs against a fresh baseline and
preserved their regressions.

Use the [W20 comparison table](../../../ITERATION_LOG.md#w20-comparison-on-the-original-timed-trajectories)
and [stronger fixed-cost figure](figures/comparison_effort005.png).

Play [stronger fixed cost, 0.1×, 9.05 s](../../../results/weighted_allocation_20261125/videos/figure8_bounded_effort005_p090_event.mp4).

- All 20 W20 runs completed; each setting passed every gate on 1/5 cases.
- All 15 bounded runs respected the reference speed/acceleration envelope.
- Stronger cost improved +0.90 peak error 191 → 85 mm but worsened triangle
  peak error 69 → 166 mm.
- Geometry-based weighting recovered triangle RMS but regressed another start.

These costs penalize normalized drive-speed demand and reference changes, not
measured electrical effort. Reference caps do not cap actual joint motion:
show [the initial bounded run's 44.3 rad/s excursion](figures/bounded_initial_excursion.jpg)
if explaining that distinction. The original timed paths were unchanged.

## 6. Preserve the regression and use it to choose the next question — I-09

**Message:** The remaining large correction was not explained by allocation
infeasibility alone.

Play [adaptive regression, 0.5×, 4.05 s](../../../results/weighted_allocation_20261125/videos/figure8_bounded_adaptive_m100_event.mp4).

- Adaptive -1.00 peak error: 318 mm; baseline 105 mm over its complete run.
  At the clip's 29.975 s peak instant, baseline error is about 58 mm.
- At the inspected onset, allocation residual was about 0.0000006 m/s, with no
  active allocation bounds.
- Estimated support switched shell/belt 18 times over about 0.5 s; large LQR
  attitude/velocity demands occurred alongside those switches.

Show the [annotated decomposition](figures/adaptive_event.png).
Say: “This is evidence to inspect support-transition and gain-blending continuity.
It does not establish one cause. We retained the baseline rather than promoting
a candidate that only looked good on selected routes.”

## Closing discussion

What the iteration established:

- We can audit whether control reads solver contacts.
- We can distinguish reference demand, measured motion, and clipped commands.
- We have a common set of trajectories, reports, and clips for comparisons.
- Bounds work as specified; the desired closed-loop improvement remains unresolved.
- At this stage, the proposed next experiment concerned transition continuity.
  I-16 later corrected the priority to same-contact geometry; I-17–I-21 implement
  and test a projected reference prototype without a generally successful fix.
  Particle filtering and real-robot validation remain future work.

If the next experiment also regresses, append it to the same record with its
hypothesis, evidence, and resulting decision.

## Evidence and sharing

The figures, metrics, reports, provenance, and selected test logs in this folder
are small snapshots outside ignored `results/`. They include the initial failed
instrumentation test as well as later passing checks. Test counts overlap; use
the [verification table](../../../ITERATION_LOG.md#verification-record) instead
of summing them into one full-repo count.

The local shareable archive is
`results/iteration_history_20261125/presentation-pack.zip`. It includes the main
log, these notes, all snapshots, and the three short clips linked above at their
original relative paths. Full-length comparison videos and NPZ rollouts remain
in their original local `results/` directories. Videos replay saved Gazebo/DART
states using MuJoCo rendering; rendering is not a new physics experiment.

`reproduction/` retains original local scripts as source evidence. Some infer
directories from `__file__`; restore them to their original paths from the
manifest before executing. Current runnable campaign tools live under the repo's
`scripts/`; see the main log for commands and reproduction limits.
`source/` preserves the relevant current code at the end of this backfill;
it does not reconstruct every earlier uncommitted experimental version.
Archived source names end in `.py.txt` to avoid collecting historical test copies;
the manifest records their original repo paths.

## Follow-up discussion: tilt versus configuration — I-11

The first presentation archive captures the six sections above and I-01–I-10.
A subsequent [offline review](tilt-configuration-review/review.json) checks the
user's proposal to schedule weights by platform tilt or robot configuration.
At the same -20° platform roll and same shell support, changing shaft phase
changes the drive geometry from independent directions to a singular direction.
In the adaptive regression, weak geometry appears earlier than the large lean
error and rapid support switching. This refines the earlier selected-sample
inspection; it does not establish one cause.

The next proposed step is a controlled configuration/direction map, distinguishing
absolute tilt from error relative to the balanced reference. No new controller
policy or dynamic simulation was run for this follow-up. See
[I-11 in the log](../../../ITERATION_LOG.md#i-11--user-hypothesis-schedule-weights-by-tilt-or-configuration)
and its [manifest](tilt-configuration-review/manifest.json).

The broader study has now run: see the
[I-12–I-14 configuration-study presentation/evidence index](configuration-study/README.md)
for the maps, continuity measurements, 40 short Gazebo cases, retained
initialization limitation, and videos. The original first bundle remains a
snapshot of the earlier stages.
