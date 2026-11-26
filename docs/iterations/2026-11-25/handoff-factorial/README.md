# I-42: isolate support routing and incoming-drive preparation

2026-11-25. **Completed; defaults unchanged.** All four arms leave the
first departure unchanged. Temporarily masking preparation improves later
tracking and passes the existing route gates in one run, while failing the
new early-departure improvement screen and worsening overshoot at four of six
later turns. This is evidence of tradeoffs, not a validated general fix. The user agreed to investigate
the I-41 early departure and asked how we could improve it.

The proposed candidates were more consistent support selection during handoff
and preparation that accounts for its reaction on platform balance. We tested
the corresponding parts of the present controller while keeping the gain-bank
settings, observer/contact-estimation code, rolling-feedforward method, timed
motion program and physical limits fixed. Load-based gain blends and nominal
preparation references are the explicitly tested changes.

[Four-arm comparison video](../../../../results/handoff_factorial_20261125/videos/handoff_factorial_comparison.mp4)
· [Shareable evidence bundle](../../../../results/handoff_factorial_20261125/presentation-pack.zip)
· [Authoritative primary report](evidence/analysis/stage01/report.json).

[Declared protocol](../../../../results/handoff_factorial_20261125/protocol.json)
· [Previous I-41 investigation](../figure8-event-audit/README.md)
· [Earlier I-25 oracle limitations](../support-identity-probe/README.md).

Four fresh 44 s Gazebo/DART executions use the original Mars figure-eight,
phase +0.9 rad, yaw 45°, 5 ms controller and 1 ms physics. Their native histories
must agree exactly before intervention. A 4.0–10.0 s window covers belt
preparation that begins near 4.38 s, the departure near 6.86 s and early recovery.
The hooks stop at 10 s; retained physical/controller history is not reset.

| Arm | Feedback support shares | Incoming-drive preparation |
| --- | --- | --- |
| Baseline | Observer-derived | Native |
| Feedback shares | Diagnostic current true shares, scaled to inferred total | Native |
| Preparation off | Observer-derived | Command/reference preparation masked |
| Both | Diagnostic shares | Preparation masked |

The true-share diagnostic replaces only the load distribution supplied to
feedback gain blending and holding-torque routing. It does not replace the
observer wrench, normal, contact points, rolling references or synchronization
records. Last-physics-step loads have different timing from the filtered
observer; this is not an isolated identity-label test or hardware controller.
Native fallback applies when either side has insufficient load, and coverage
is logged. No future truth is used.

The preparation mask zeroes both command blend weights and the per-target
weights that advance material joint references. The private preview and plane
updates still execute normally. Therefore the test removes both preparation
effects; it is not a torque-only cancellation.

Primary metrics use 6.4–7.4 s; recovery uses 7.4–10.0 s. Also report the full
intervention and complete route, nominal tracking, lean, clipping, command
steps, path-departure thresholds and existing route gates. One execution per
arm supports descriptive contrasts and interaction, not statistical robustness.
Fresh baseline is checked against the complete original I-40 trace rather
than assuming reproduction.

A provisional screen for preparation-off requires completion without a fall,
at least 10% lower primary local-path RMS, no more than 0.5° worse peak lean
in the primary window or route, and at most 5% worse moving route RMS. This is
a declared experiment screen, not adoption or stability proof. Any improvement
using true support shares remains diagnostic.

I-25 already showed that true-share/full-record substitutions did not fix a
different late phase −1 event, with worse peak lean in each branch. That
negative result remains relevant. Smooth allocation penalties also regressed
in previous trials; a proposed but unexecuted gain blend is not described as a
failed implementation. No particle filter, iLQR or blanket command smoothing
is introduced here.

Exact pre-change sources and six older bundles are hash-preserved. Fresh runs
use isolated processes from time zero; restoring a Gazebo snapshot would not
preserve DART contact history. Source/check/failure stages, results and video
are retained.

## Results: first departure unchanged, later outcomes differ

All four fresh 44 s Gazebo/DART runs complete without falling. All three
warm-prefix comparisons match bit-for-bit, including extended controller
history at 4.0 s. The fresh baseline matches every one of the 17 common I-40
full-precision fields over the entire trace; derived baseline path geometry
also matches the I-41 cache exactly. Reference time/XY/velocity/acceleration
are identical across arms.

| Arm | First sustained 20 mm departure | Primary local-path RMS, 6.4–7.4 s | Primary peak relative lean | Primary clipping fraction |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 6.860 s | 38.661 mm | 5.328° | 9.0% |
| True feedback shares | 6.860 s | 38.850 mm | 5.323° | 9.5% |
| Preparation masked | 6.860 s | 38.548 mm | 5.226° | 9.5% |
| Both | 6.860 s | 38.792 mm | 5.227° | 11.0% |

Preparation masking improves the primary RMS by only **0.29%**, below the
predeclared 10% screen. Its peak local distance is slightly worse, 67.680 mm
versus 67.410 mm. All four also share the 10 mm startup threshold at 1.000 s
and 30 mm threshold at 6.910 s. Neither intervention remedies the first event.

| Arm | Moving position RMS | Peak timed error | Final error | Full peak relative lean | Settled at end | All existing route gates |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| Baseline | 50.486 mm | 190.901 mm | 190.191 mm | 8.709° | No | Fail |
| True feedback shares | 41.635 mm | 85.335 mm | 8.804 mm | 5.323° | No | Fail: settling |
| Preparation masked | 41.681 mm | 97.401 mm | 11.758 mm | 6.352° | Yes | Pass |
| Both | 45.355 mm | 101.198 mm | 26.032 mm | 8.600° | Yes | Fail: attitude |

The preparation-masked branch improves moving RMS by **17.44%** and passes
all original route gates in this single run. That later benefit is real within
this deterministic comparison, even though it fails the new early-event screen.
Combining the interventions performs worse than either alone in moving RMS;
their effects are not simply additive. Full factorial contrasts retain units
and are descriptive, not statistical significance.

The hooks stop at 10 s. Later improvements therefore include the consequences
of altered motion and controller history, not continued oracle use or a
permanent preparation shutdown. A timer chosen for this trajectory is not a
general configuration-based policy. No production setting is promoted.

## Secondary turn check: old route gates conceal a regression

After the primary reduction, a separately declared saved-data check reuses all
six exact I-41 component reversal times, [−1,+2] s windows, geometric extrema
and 100 ms reversal dwell. It changes no primary threshold or acceptance
criterion. Every turn occurs after the interventions stop at 10 s, so this
measures carried physical/controller history rather than active masking.

| Reference turn | Baseline geometric overshoot | Preparation-masked overshoot |
| --- | ---: | ---: |
| 11.775 s / y | 0 mm | 18.05 mm |
| 15.377 s / x | 0 mm | 0 mm |
| 18.301 s / y | 0 mm | 35.53 mm |
| 23.699 s / y | 9.84 mm | 38.92 mm |
| 26.623 s / x | 28.02 mm | 2.95 mm |
| 30.225 s / y | 60.20 mm | 68.41 mm |

At the first turn, baseline reversal lag is +1.101 s, versus +0.832 s masked:
less lag comes with newly positive overshoot. Oracle-only first-turn lag is
+0.616 s with 15.03 mm overshoot; the combined arm has +1.144 s and zero.
Four of six masked turns worsen this metric, one improves and one is unchanged.
A pass on the old route gates is therefore insufficient to adopt this change.
Multiple reversal candidates are flagged rather than silently selected.

[Turn report](evidence/analysis/turn_addendum/report.json)
· [All-arm turn table](evidence/analysis/turn_addendum/turn_metrics.csv)
· [Turn comparison plot](evidence/analysis/turn_addendum/turn_comparison.png).
The focused video shows the early event and full-route summary, not these
later turns; use the turn table/plot for that regression.

## What the interventions actually exercised

Every arm has 8,801 guarded command evaluations, including the unactuated
terminal observation; this is 35,204 evaluations, not additional unit tests.
Each intervention window contains 1,200 held commands. Oracle-only has
1,200/1,200 eligible injections; the combined arm has 1,199 and one explicit
fallback because current truth load is at most 0.05 N. Inferred total load is
preserved exactly. Shares materially differ on only nine calls per oracle arm.

All seven same-state scope checks pass. Both synchronization masks have zero
top-level and per-target weights and zero preparation reference increments
throughout the window, with native preview/plane computations retained.
The mask suppresses nonzero native preparation on 807/778 calls in the two
masked branches. No routing/masking occurs outside the window.

Oracle-only first changes the command at 6.725 s and physical state at 6.730 s.
Preparation masking first changes the command at 4.380 s but the physical
state/observer only at 6.700 s, during the first brief belt contact. Earlier
unloaded belt-speed commands need not move the simulated robot. The shared
6.860 s departure remains despite these genuine interventions.

## Secondary command decomposition: the loaded belt is already saturated

A separately declared saved-data audit fixes nine times around onset and
decomposes exact −K×error without executing the controller. At **6.825 s**,
all four arms correctly identify belt A but request **0.822–0.835 m/s** against
its **0.300 m/s** simulation bound. The baseline terms are:

| Belt A contribution | Request |
| --- | ---: |
| Attitude error feedback | +0.5455 m/s |
| Angular-velocity error feedback | +0.2021 m/s |
| Linear-velocity error feedback | +0.0747 m/s |
| Position error feedback | +0.0192 m/s |
| Joint-velocity error feedback | +0.0022 m/s |
| Total LQR feedback | +0.8438 m/s |
| Rolling feedforward | −0.0091 m/s |
| Preparation delta | 0 m/s |
| Requested / applied | 0.8347 / 0.3000 m/s |

Actual tilt is 7.641° versus desired 12.031°; full up-vector error is 5.294°.
Thus large balance-feedback demand precedes the 6.860 s routing switch and
persists with the tested routing/preparation interventions. It is not primarily
a large rolling-feedforward contribution at this sample. At 6.860 s, the wrong
shell routing adds a separate correction; the oracle avoids it without
preventing the departure. These observations identify where demand originates,
not a unique physical cause.

The desired angular-velocity reference is exactly zero in all 36 selected
snapshots although the desired up-vector changes. This justifies checking
reference consistency; adding rates is not proven to improve the saturated
response. Desired-up differentiation can amplify normal-estimate changes,
and the current yaw-carrying reference is deliberate.

Exact grouped terms reproduce the saved LQR component after its intended
float32 cast at every context sample. Maximum normalized component residual
is 2.30e−8. Quaternion tangent errors are not Euler roll/pitch.

[Shared-path audit and limitations](evidence/shared_path_audit/README.md)
· [Numeric report](evidence/shared_path_audit/stage01/report.json)
· [Command decomposition plot](evidence/shared_path_audit/stage01/shared_feedback_terms.png).

## Decision

The leading next candidate is a **smooth, attainable tilt reference with a
consistent desired tilt rate**, preserving the final observer-derived ground
lean and existing heading behavior. Shape only the attitude target, leaving
contact estimation/geometry and LQR gain-bank settings fixed for an isolated
test. A reference generator with an explicit rate state avoids treating a
noisy finite difference as a commanded rate. Test it against the unchanged
baseline, including the original timed path error, lean, saturation, all six
turns and broader trajectories. This is a proposal, not an implemented fix;
the present decomposition does not justify weakening balance feedback.

Do not adopt a blanket gain blend or turn preparation off globally. A measured
state-based preparation gate remains a candidate to test for later route
performance: reduce preparation when estimated support is uncertain or body
lean/rate leaves little balance margin. This candidate is proposed, not
implemented or validated; a broad trajectory regression is needed before
adoption, including the newly exposed turn regressions. It would use observer, encoder/body-motion information rather than
simulator contact truth.

The first departure needs a separate audit of the shared reference and
loaded-drive response, including clipping. The feedback-share oracle leaves
estimated contact geometry and rolling feedforward untouched, so these results
do not rule out their contribution or prove the estimator irrelevant. The
unused 200 ms dynamic forecast still cannot cause the live drift. No particle
filter or iLQR is justified merely by this result.

## Validation, corrections and reproduction

- [Primary event plots](evidence/analysis/stage01/event_comparison.png) and
  [full route comparison](evidence/analysis/stage01/full_trajectory_comparison.png).
- [Harness checks](evidence/harness_checks/stage01/report.json): 21 unique pure
  contracts pass in both numerical environments. Repetition is not 42 new
  tests; this is not the full repository suite.
- [Corrected independent raw QA](evidence/independent_QA/stage02_corrected/report.json)
  verifies case hashes, exact prefix/history, oracle scaling/fallbacks, masks
  and per-target reference effects. Initial QA source/failure is retained.
- [Campaign bookkeeping](evidence/campaign/completion_audit.json) records four
  attempts, maximum three concurrent processes and no retries. Its 72 launch
  references were unchanged at campaign completion; the presentation renderer
  was refined later and its earlier source retained.

The initial reduction compared float32 saved input bounds with double
provenance by exact equality, failing on a 2.384e−8 gap. The failed stage is
preserved; metadata validation now uses 1e−7 tolerance while metrics retain
the double bounds. Initial independent QA likewise assumed standard rollout
arrays were float64; corrected QA checks the exact float32 cast. Full-precision
clocks, source/history checks and intervention outcomes are unchanged. No
physics was rerun for either correction. Arithmetic checks separately cover
window endpoints, terminal holds, signed-zero identity and early termination;
repeated I-41 geometry checks are labeled overlapping evidence.

The first video preview clipped a footer. Its source/still are retained;
the final renderer shortens that footer and explicitly distinguishes the
failed early-improvement screen from the passed existing route gates. These
are presentation changes after simulation, not the exact launch renderer.

Full commands and exact sources are retained in the launch manifest, per-arm
provenance and stage snapshots. Archived Python sources use .py.txt to avoid
test discovery; restore them to their recorded origin for reproduction.
Generated worlds, terrain assets and numerical environments remain repository
dependencies. Compact reports/figures/sources are outside ignored results;
raw precision/rollout/full-preview files and MP4 are in the shareable bundle.

[Evidence manifest](evidence/manifest.json)
· [Payload manifest included in ZIP](../../../../results/handoff_factorial_20261125/presentation_manifest.json)
· [Post-package verification sidecar, outside ZIP](presentation_verification.json).
