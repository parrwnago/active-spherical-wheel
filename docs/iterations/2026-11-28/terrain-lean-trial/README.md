# I-85 — terrain-informed lean fails the sustained-recovery objective

Status: **completed one fresh physical pair; STOP_CANDIDATE.** Park terrain
assistance and focus on baseline recovery, following the user's instruction.
No tuning, extra candidate or outcome-dependent retry follows this result.

## Video and result

- [Comparison video, 99.5–133 s at 1×](../../../../results/terrain_lean_trial_20261128/publication/video_stage01/native_vs_terrain_lean_99p5_133.mp4)
- [Comparison PDF](evidence/evaluation/completed_stage01/comparison.pdf) · [Plot](evidence/evaluation/completed_stage01/comparison.png)
- [Complete gates and metrics](evidence/evaluation/completed_stage01/report.json) · [Independent audit](evidence/verification/completed_stage01/report.json)
- [Shareable video/report/context ZIP](../../../../results/terrain_lean_trial_20261128/publication/visual_bundle_stage01/I85-terrain-lean-trial.zip)

Left is native N; right is terrain-informed T. **Both use 0.60 m/s belt limits.**
Gazebo/DART supplies the recorded dynamics; MuJoCo supplies visuals only.
The movie includes the final ten-second goal hold. All 336 encoded frames
decode and representative encoded frames were inspected. The comparison MP4
with a shorter filename is a byte-identical duplicate, not another trial.

| Measure | Native N | Terrain-lean T |
| --- | ---: | ---: |
| First two-second recovery entry, video clock | 117.675 s | 118.325 s |
| Confirmation after two seconds | 119.675 s | 120.325 s |
| Entry elapsed from the 100.5 s benchmark anchor | 17.175 s | 17.825 s |
| Stays in path-motion bands from entry through 133 s | Yes | Yes |
| Moving timed XY RMS | 23.362 mm | 21.809 mm |
| Full timed XY peak | 72.167 mm | 65.163 mm |
| Focus spatial-velocity residual peak | 33.177 mm/s | 23.498 mm/s |
| Focus intended-section path peak | 37.526 mm | 38.799 mm |
| Final goal distance | 0.877 mm | 0.986 mm |
| Actual-bound clipping duration | 0 s | 0 s |

The screen passes **36 of 37 checks**. The only failure is recovery benefit:
T must enter at least one second earlier than N, at or before 116.675 s. It
instead enters **0.650 s later**. Both then remain acceptable through the end.
Full-run tilt/speed/rotation peaks are unchanged. Partial tracking and local
velocity improvements remain recorded alongside slightly worse focus path
error and final goal distance. They do not meet the recovery objective.
The common anchor is not an identified physical disturbance onset. Strict
spin-inclusive recovery remains separate from translational return.

## One fixed candidate and matched conditions

Both fresh runs complete 133 s / 26,601 control rows without physical failure;
both process exits are observed zero. Fresh N reproduces **all 79 I-82 B060
precision arrays byte-for-byte**. T matches the inactive prefix and physical/
observer history through its first changed applied command at **99.505 s**.

Original native I-76 eight, shaft start −1 rad, yaw158°, rough-stress Mars
terrain, timed path, LQR gain bank/design, momentum observer, current contact/
normal/load inference, holding feedforward, rolling allocation and preparation
remain unchanged. Physical and mirror operating caps are 0.60 m/s after
constructing the original 0.30-based design. No heading helper or retiming.

The fixed candidate queries previously delivered four-camera memory at the
current inferred foot and 0.50 s ahead under measured constant velocity.
Future-minus-current mapped-normal contrast supplies an extra lean capped at
2°. It never replaces the current observer normal. Smooth only this correction
with tau0.10 s, SSP-Heun substeps no larger than1 ms, and quintic1 s ramps over
99.5–100.5 /103–104 s. The total additional chart/rate budget is0.15 rad/s,
including ramps; output is exactly native outside99.5–104 s. The angular-rate
component differentiates only the extra correction, retaining the native
zero-rate convention for its own changing target/yaw. An explicit28°combined
reference guard would stop the trial rather than silently project or retune.

Actual added lean peaks at **0.4797°** and additional angular rate at
**0.04968 rad/s**. Map queries are accepted at900/901 active-grid rows; one
floating-clock start-boundary row is outside the literal active window.
Actual feedback angular references match their exports, and added outputs are
exactlyzero outside the window. The largest normalized N/T applied-input
difference in the window is0.1805; it includes closed-loop history effects.
[Intervention audit](evidence/runner/intervention_audit_stage02/report.json).

Each arm captures2,661four-camera batches, delivers2,660 and ends with1pending.
Use ideal depth, static memory, exact simulated capture-time pose,20 Hz capture
and50 ms delivery. No future solver contact or direct future terrain truth
enters feedback. This is an active intervention, unlike I-84's inactive prep.

## Fair scoring, correction history and validation

Primary recovery uses the SAME unmodified native reference RULE at each arm's
current observer state. Candidate changed-target tracking and comparison with
same-clock N attitude are separate diagnostics. A changed target alone cannot
improve the recovery score. I-83 path/velocity/attitude bands and2 s dwell stay
fixed; require at least1 s earlier entry, uninterrupted good samples from that
entry to133 s, meaningful applied change and fixed nonregression margins.

Frozen physics protocol SHA256:
`20da805827396858da0dc1c60d51b12e045bca0e3a137065319262fef1e82667`,
117sources/218inputs. The final pair seal binds700files, SHA256
`e0c57507b2ade07880e5329bfea61aae67de3c0438460131eaf1cb0eb5f5b11b`.
Original source/design/cap identities remain unchanged.

Thirteen policy and seven runner contracts pass in both Python environments;
repetitions are not26/14tests. Three media contracts are a separate scope.
Physics uses Gazebo/DART; the mirror uses Python3.12.16/NumPy1.26.4/MuJoCo3.16.

Retain setup history and exact sources:

- Before physics, replace a draft same-clock-N target with the common native
  RULE at each current state, keeping the old draft. Restore I-83's eleven
  finite measured/reference/command fields for assessment; unloaded truth NaNs
  stay in replay checks. Four scorer contracts include earlierthree.
- After N completes, before new array reductions or T precision/outcome,
  review finds a persistence bug: a final2 s goodtail alone does not establish
  no exits after FIRST2 s entry. Preserve all117frozen sources; separately bind
  `evaluate_stage02.py` / `definitions_stage02.json` implementing the declared
  uninterrupted requirement. Five final scorer contracts include earlierfour;
  independentfour include earlierthree. No policy/numerical threshold/physics
  changes. Legacy-tail-only and corrected decisions are both STOP here.
- Supplemental intervention-audit JSON serialization initially fails on a
  NumPy boolean. Preserve partial output/source/failure, then correct only
  serialization in a new stage02. No trial or parameter rerun.

Independent raw reconstruction agrees on60signal arrays,10sample-exact
predicate arrays,1,568numerical recovery/metric leaves and all performance
gates. It audits identities/prefix, gain/input ledgers, camera/query clocks,
actual angular references and bounds. These overlap canonical validation;
none is an added physical trial or a full-repository test-suite count.

## Baseline recovery follow-up

[Old-baseline diagnostic](evidence/analysis/baseline_recovery/completed_stage01/report.json)
· [Plot](evidence/analysis/baseline_recovery/completed_stage01/baseline_return_interruption.png)
· [Interpretation](evidence/analysis/baseline_recovery/INTERPRETATION.md).

A separately registered posthoc reduction inspects original I-82 B060 over
105.4–106.1 s without reading new trial outputs or changing acceptance. Velocity
error exceeds20 mm/s at105.590 s and peaks107.139 mm/s while path distance
stays below8.237 mm and native up error below1.258°. Commands are unsaturated;
belts/preparation are inactive; inferred shellA does not switch; rolling joint
references remain small. It differs from the earlier large-reference spikes.

A last1 ms pressure sample iszero at105.525 s, returns at105.530 s and later
peaks at105.725 s. The normal-derived tilt reference relaxes from~9.3°to~5.6°
while angular-rate reference stayszero. Newly emitted commands act AFTER their
recorded states; previous held commands may still matter. These data establish
neither a complete5 ms airborne interval nor complete force causality.

**Decision:** park terrain assistance. Focus baseline work on velocity recovery
through short contact/normal/reference transitions, using the existing observer
and available command headroom. No reactive fix is implemented by this report.

## Evidence and reproduction

Eventmovie:1,310,609bytes, SHA256
`de093b8a31cb20f400ada37050d58a103345ac1fbe2ee32256359e0e15976c88`.
VisualZIP:2,049,259bytes/12CRC+SHA-verified members, SHA256
`e3a60521c9c8abc712f5af1a83ae18bf3e80734a89aae27a44679c0e12e41718`.
It includes the trial, baseline diagnostic and contextrevision20.

Restore exact captured sources and input paths in the recorded environment;
use fresh output directories. Recorded launch commands and actual exits live
under `campaign/`. N must pass completeall79 replay before T. Analysis-only
inspection needs no new physics; preserve original and corrected scorer records.

Local evidence: `results/terrain_lean_trial_20261128/`. The
[iteration log](../../../../ITERATION_LOG.md) retains every material stage.

The [compact manifest](evidence/manifest.json) binds148files/6,583,460bytes
plus itself; its [source/metadata archive](evidence/source-and-diagnostics.tar.gz)
has902verified members. Raw depths, complete policy rows, physical precision/
rollout arrays, models and videos are preserved in the
[full local archive](../../../../results/terrain_lean_trial_20261128/publication/evidence_archive_stage01/i85_full_evidence.tar.gz):
757,684,389bytes, SHA256
`950d57363ad251d904d9a2ae5c6b312f08dde40b6aa7b7f691036b8875cf9612`.
All2,381project members are stream-hashed/size-checked plus4byte-verified
metadata members,2,385regular members total. It restores335registered original-
path prerequisites and inventories24cache/curation omissions. See
[verification](evidence/publication/evidence_archive_stage01/verification.json),
[inventory](evidence/publication/evidence_archive_stage01/inventory.json)
and [restoration instructions](evidence/publication/evidence_archive_stage01/REPRODUCE.md).
Extract into an isolated checkout with the recorded environment; the archive
is not a complete simulator installation.
