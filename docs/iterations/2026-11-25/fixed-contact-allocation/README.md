# Same-contact feasible-reference experiment — I-17–I-21

2026-11-25 · [Iteration log](../../../../ITERATION_LOG.md#i-17--implement-directional-allocation-and-accepted-motion-feedback)
· [Evidence and source manifest](evidence/manifest.json)

**Implemented and tested, retained as an experiment. The overall robot-level
problem is not solved.** The single-contact kinematic fixture verifies bounded
allocation and a consistent horizontal velocity reference. Mars trajectories
still show substantial regressions, so `least_squares` remains the default.
The controller remains gain-scheduled LQR with contact-aware feedforward.

## Question and change — I-17

The common shaft and loaded hemisphere can produce nearly parallel ground
motion at one contact. Matching a request in the missing direction with an
unregularized inverse produces enormous opposing drive references. This does
not require both hemispheres touching or a shell/belt handover.

The new opt-in `rolling_allocation='feasible'` uses smooth directional effort
costs, reference speed/acceleration bounds, and the velocity that its allocated
drives imply as the body feedback reference. It projects position error and
nominal lean acceleration with a geometric response. This lets it relax a
poorly driven direction while retaining a useful direction. The original
timed path and destination remain separately logged and evaluated.

Defaults for the experiment: modal effort .001–.005, change cost .001,
normalized authority threshold .02, reference speeds `(20,20,20,.3,.3)`,
accelerations `(100,100,100,3,3)`, position-error cap .05 m, catch-up gain 1/s,
catch-up speed cap .06 m/s. Joint references use rad/s and belts m/s. These
are declared simulation assumptions, not measured actuator capabilities.
[OFFROAD.md](../../../../OFFROAD.md#experimental-feasible-motion-reference)
documents the math and parameter names.

Exact agreement is limited to the supplied single-contact horizontal model.
With several estimated contacts, the accepted velocity matches their weighted
mean and can leave per-contact residuals. Position projection uses geometric
authority without active bounds/reference history; it is a heuristic rather
than a complete feasibility guarantee. Projected nominal acceleration is not
the derivative of accepted velocity. Drive reference bounds do not bound
actual motor speed or body acceleration as geometry changes.

## Mechanism tests — I-18

The synthetic fixture prescribes one valid interior shell-B support through
an 8 s phase sweep, with fixed -20° platform roll. Three requested directions
and three allocators produce nine cases, with no support changes. All 14
assertions pass. LS requests up to 3.31 million rad/s near the singularity;
the bounded variants obey the declared reference limits. At the pole, the
candidate retains about .06 m/s in the useful x direction and rejects y.
The reference agrees with its allocated single-contact kinematics throughout.
Both bounded variants can still reach 20 rad/s transiently. No physics, LQR,
observer, measured travel or balance runs in this fixture. Shaft phase is
externally restored; recovery of geometry is not autonomous goal completion.

Show the [fixed-support comparison](evidence/fixed_support/comparison.png)
with its [assertions and geometry checks](evidence/fixed_support/summary.json).

Eight Gazebo warm-observer cases provide separate physical evidence. Both
branches run identical .8 s LS warm-up on a 10° side slope, then preserve
observer, estimator, synchronizer and physical history when enabling the
candidate. Shell A carries the support throughout the travel pulse, without
handovers. However, geometry evolves out of the declared weak-authority band;
neither pulse family passes the sustained-weak-geometry gate. Retain this
limitation rather than calling it a successful sustained-singularity test.

| Request | Nominal displacement | LS / initial candidate travel at motion end |
| --- | ---: | ---: |
| Long world y, .60 s | 16 mm | 8.5% / 8.9% |
| Long world x, .60 s | 16 mm | 85.4% / 91.6% |
| Short world y, .16 s | 4.4 mm | 69.4% / 63.7% |
| Short world x, .16 s | 4.4 mm | 180.6% / 179.0% (overshoot) |

Watch [short y comparison, 0.1×, 7.65 s](../../../../results/fixed_contact_allocation_20261125/short_pulse/videos/warm_world_y_baseline_vs_feasible.mp4).
Its original-goal errors after recovery are 6.2 / 5.5 mm, exceeding the 4.4 mm
request. Small absolute errors alone would be misleading. Full setup,
quantities and reproduction commands are in the
[archived warm-case notes](evidence/warm/WARM_CASES.md) and
[paired plot](evidence/warm/warm_case_comparison.png).

## Trajectory results — I-19–I-21

Use this campaign's fresh baseline rather than older D10/W20 results. All
paths retain their original timing and goals. Each setting/start is one trial,
not a statistically sampled robustness claim. Gazebo/DART supplies physics;
`hardware` mode receives exact-state simulation surrogates and observer/locator
contacts, with simulator contact APIs blocked during control. No robot exists
yet, and sensor uncertainty has not been reproduced.

| Original-path moving RMS, mm | Fresh LS | Initial: cap .05 m | V2: cap 1 m | V2 + balance guard |
| --- | ---: | ---: | ---: | ---: |
| Figure-eight, +.30 | 56.96 | 36.15 | 64.31 | 90.51 |
| Figure-eight, +.90 | 50.49 | 161.62 | 40.23 | 144.08 |
| Figure-eight, -1.00 | 56.05 | 25.20 | 28.40 | 101.02 |
| Triangle, +.30 | 24.35 | 21.64 | 18.09 | 27.32 |
| Out-and-back, +.30 | 35.57 | 27.95 | 72.34 | 46.65 |
| Runs completing requested duration | 5/5 | 5/5 | 5/5 | 5/5 |
| Runs passing every acceptance gate | 0/5 | 0/5 | 0/5 | 1/5 |

Presentation figures: [initial and ablation trials](evidence/analysis/prototype_comparison.png),
[V2 paths and destination accuracy](evidence/analysis/v2_reference_and_destination.png),
[balance-priority tradeoff](evidence/analysis/balance_tradeoff.png).
The [complete metrics CSV](evidence/analysis/physics_metrics.csv) and
[audited report](evidence/analysis/physics_report.json) retain every case.

**Initial failure retained.** The +.90 candidate reaches 504 mm original-path
error and 14.71° attitude error. Its projected-reference RMS is only 35.35 mm,
while original-path RMS is 161.62 mm. The reference separation reaches 454 mm.
We therefore report original-path error as the primary tracking metric.

Three targeted +.90 ablations test this architecture: no catch-up (gain 0,
cap .05 m), wider cap (gain 1/s, cap 1 m), and both changes. RMS is 45.80,
40.23 and 46.97 mm respectively; final errors are 50.19, 32.47 and 139.28 mm.
None passes all gates. Widening alone warrants the five-case V2 comparison,
but V2 then regresses +.30 and out-and-back. Good final distances conceal
large intermediate excursions.

Watch [V2 Mars regression, 0.25×, 8.05 s](../../../../results/fixed_contact_allocation_20261125/videos/mars_figure8_p030_v2_regression.mp4).
At the selected event, baseline original-path error is 77 mm and V2 296 mm;
full-run maxima are 154 and 296 mm. This is the **1 m cap, catch-up enabled,
balance guard off** trial, not the initial candidate or final guard family.
Captions distinguish original goals from projected references. Videos replay
saved Gazebo states through MuJoCo rendering; no new physics runs to make them.

The last trial scales travel by a smooth lean-error/rate priority (3° scale,
.1 s rate horizon). It leaves gain matrices/holding-torque formulas in place;
instantaneous scheduled gains can differ as paths diverge. It reduces some
attitude peaks, but increases original-path RMS against baseline in every case.
Triangle alone passes all gates. Its unsigned rate measure can also suppress
travel during recovery. Do not treat this heuristic as a stability proof.

There are 23 separately executed Mars cases: baseline five, initial five,
three ablations, V2 five, guard five. They produce 22 unique NPZ hashes: the
separately executed V2 +.90 case exactly repeats the wide-cap ablation output.
This is not evidence of statistically varied independent repetitions. All
240,423 route controller calls obey the contact-access guard. The eight warm
fixtures are separate. Every candidate obeys reference limits, but actual
joint speed reaches 45.81 rad/s in the initial family.

## Tests, provenance and presentation

The selected pre-guard regression passes **145 tests in 244.24 s**. Final
targeted integration passes **110 tests in 50.01 s**, including 46 tests of
the new reference behavior. Counts overlap; neither is the full-repo suite.
The fixture also passes 14 assertions. Tests cover modal directions, bounded
reference changes, accepted single-contact velocity, retained nominal goals,
invalid contact handling, logging and unchanged existing modes when the new
policy is inactive.

Retained failures include the `.venv` setup missing Gazebo bindings (no physics
started), the initial cap's severe lag, failed ablations/V2/guard cases, and an
initial guard-test fixture failure. The latter expected suppression for a
tilted pose, but the startup observer's inferred normal made that pose match
its intended lean, so priority correctly stayed at 1. Supplying the intended
flat-normal measurement corrected the fixture without a control change.
That initial failing test's stdout was not saved separately; this narrative
records the failure and correction, while final passing logs are archived.

The [evidence manifest](evidence/manifest.json) identifies copied reports,
plots, test logs, source stages and hashes. Launch-time snapshots and later
source snapshots are distinguished. All eight final guard source files match
the recorded launch hashes. The later live source changes a module comment
and aligns the held-request diagnostic with travel priority; commands are
unchanged. Historical source copies use `.py.txt` to avoid pytest collection.
Full raw NPZs and videos remain under ignored
`results/fixed_contact_allocation_20261125/`.

The local [presentation pack](../../../../results/fixed_contact_allocation_20261125/presentation-pack.zip)
contains these notes, iteration entries, compact evidence, source snapshots,
and the two linked videos at their original relative paths. Earlier bundles
remain unchanged. Restore archived helper scripts to their original paths in
the manifest before using scripts that derive paths from their location.

Reproduce with fresh output directories, using the Gazebo environment in
[GAZEBO.md](../../../../GAZEBO.md) for physics:

```bash
MUJOCO_GL=egl python scripts/test-fixed-contact-allocation.py --output results/new_feasible/fixed_support
MUJOCO_GL=egl python scripts/run-contact-diagnostics.py --variant baseline --output results/new_feasible/baseline
MUJOCO_GL=egl python scripts/run-contact-diagnostics.py --variant feasible --output results/new_feasible/initial
MUJOCO_GL=egl python scripts/run-contact-diagnostics.py --variant feasible --position-error-limit 1 --output results/new_feasible/v2
MUJOCO_GL=egl python scripts/run-contact-diagnostics.py --variant feasible --position-error-limit 1 --balance-guard --output results/new_feasible/balance
```

Current-source reruns are new experiments; no bitwise reproduction is promised.
For +.90 ablations add `--route plain_figure8 --phase .9` and the appropriate
`--catchup-gain 0` / `--position-error-limit 1` flags.

**Decision:** retain the default and this experimental branch. We have verified
the limited kinematic reference treatment, not robust closed-loop recovery.
Persistent rank loss requires a change of geometry to reach a blocked goal;
weights cannot create the absent direction. Deliberate configuration recovery
and a separately isolated support-transition policy remain future experiments.
No particle filter or new LQR was implemented.
