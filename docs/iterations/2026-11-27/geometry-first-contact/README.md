# I-72 — choose geometry before velocity contact ranking

**Implemented and tested; spatial benefit, configured selector not adopted.**
The geometry-first selector lowers worst contact-point error, but makes three
more family errors than the earlier guarded estimator and accepts 54 fewer
samples. Four of ten registered gate rows fail; two are redundant coverage
constraints. All point-error guards against the native estimator pass.
The live LQR, momentum observer, support locator, feedforward and incoming-drive
preparation stay unchanged. The rejected HMM branch remains stopped.

[Three-page results and physical-candidate PDF](evidence/publication/plots_stage02/geometry_first_contact_report.pdf)
· [Comparison image](evidence/publication/plots_stage02/overview.png)
· [Physical candidate clouds](evidence/publication/plots_stage02/physical_candidates.png)
· [Standalone ChatGPT context](../../../CHATGPT_CONTROL_CONTEXT.md)

## Hypothesis and implementation

I-60 found that measured velocity helps identify the contact family, but its
joint point/family search sometimes selects a distant point to fit motion.
I-67 showed that useful nearby candidates already existed, using truth-assisted
capacity checks rather than an implemented selector. I-72 tests a deployable
choice rule that has access only to the native observer point:

1. Keep the original active physical candidate bank for each shell/belt family.
2. Within each family, choose the candidate with smallest **squared 3D world
   distance to the native observer equivalent point N**. Exact distance ties
   use the smallest original candidate ID.
3. Mask every other point and call the **unchanged I-60 ranker**, with its old
   fused/velocity scores and confidence settings.
4. The primary shadow policy **C** uses the new fused accepted family and its
   physical point; otherwise it retains the original native N family and
   equivalent point exactly.

The opt-in implementation is
[geometry_first_contact.py](../../../../src/spherical_wheel/geometry_first_contact.py).
It imports NumPy only and requires an explicitly supplied frozen rank callable.
It accepts no native family label, truth or temporal history. The consumer
accesses native identity only after geometry/ranking completes. No live control
module imports this new code.

The anchor is copied directly from I-60 measurement context's
`exact_estimated_contact_point_m`. Its source is guarded native-controller
diagnostics from `measure_records(locator.records(observer.wrench()))`.
It is an inferred load-weighted equivalent point, **not** simulator truth or
the candidate-bank alternative Nalt. It can lie inside the sphere between
multiple supports; it is neither normalized nor moved onto another body.
Accepted points belong to their selected physical family. Native fallback
retains its existing equivalent-point convention, which need not be one
physical contact. See the [source provenance](evidence/inputs/native_anchor_provenance.md).

Eligibility stays the old I-60 predicate: original validity, finite nonnegative
score inputs, finite velocity vector, valid frame and finite excitation. The
0.02 m/s minimum excitation is an acceptance threshold, not an extra raw-choice
admission rule. Missing/nonfinite anchor produces no new accepted hypothesis;
native fallback remains explicit. A missing active physical coordinate raises
an input error rather than silently dropping that candidate.

The old scores remain:

```text
G  = (normal_angle / 10 degrees)^2
WG = G + moment_residual_norm^2
VG = G + (tangential_velocity_residual / 0.02 m/s)^2
F  = G + moment_residual_norm^2
       + (tangential_velocity_residual / 0.02 m/s)^2
```

F is the primary ranking. Confidence still requires at least two distinct
families, excitation ≥0.02 m/s, score gap ≥0.25, selected residual ≤0.02 m/s,
and best/runner-up velocity-vector separation ≥0.01 m/s, without a score tie.
No cost, threshold, normal cone, force floor or geometry is tuned.
Raw F and VG, VG accepted/native fallback, and accepted-only summaries are
secondary outputs; none replaces C after seeing the results.

## Conditions and information boundary

Same three I-58 native **Gazebo/DART Mars figure-eight** recordings: 44 s,
5 ms control, 8,801 frames each, shaft phases −1.0, +0.3 and +0.9 rad, yaw 45°.
The selected pipeline retains gain-scheduled LQR, momentum observer,
rolling/holding feedforward and 120 ms incoming preparation. Assumed command
caps remain `[1.5,0.6,0.6] N m` and `[0.30,0.30] m/s` for the belts.
Baseline source commit: `0bfca57c346d475edadb9ee84abf3413346fc5ca`.

All 26,403 timestamps are processed. Inference reads original projected
features, current physical candidate points and native anchor. Measured motion
comes from exact simulated state and prior achieved belt motion under the
ideal prescribed-speed belt model. Desired velocity, future state, terrain and
solver contacts never enter ranking. Native loads/family are stored separately
for fallback. This is observer-conditioned inference, not independent sensing.

The [protocol](evidence/protocol/protocol.json) precedes new saved-data selection.
The [source/input seal](evidence/protocol/inference_source_input_seal.json) binds
the tested code and packets; [all three predictions](evidence/inference/all_three_sealed.json)
were sealed before new-output truth scoring. Preparatory historical schema and
provenance inspections are disclosed. These recordings were repeatedly inspected
in earlier iterations; they are retrospective comparisons, not unseen validation.

Original projected arrays and native anchors copy byte-exactly. Sparse point
storage preserves all 102 IDs and original masks, copying coordinates exactly
where I-60 `valid=True`; invalid coordinates decode as NaN and never participate.
Coordinate-byte parity is against the frozen I-70 cached bank, not a claim
about unstored original I-60 full geometry. Earlier I-70 reconstruction parity
is retained as provenance. No NNLS, new geometry, new physics or motion video
is produced. Compressed inference packets and metadata total 41.01 MB.

## Results on fixed domains

N is the actual recorded native estimate; old F and A are I-60's offline raw
fused ranking and accepted/native fallback. **A is not the live controller.**
The primary domain retains all 23,423 dominant-positive-pressure rows in
`[1,41)` s. Unknowns, fallback and wrong-family outputs remain scored.

| Policy | Family errors | Role |
| --- | ---: | --- |
| Native N | 412 | Recorded live estimate |
| Old raw F | 109 | Earlier raw-ranking comparator |
| Guarded A | 124 | Earlier accepted/native comparator |
| New raw F | 118 | Secondary |
| **New fused/native C** | **127** | **Primary** |
| New raw VG | 117 | Secondary |
| New VG/native fallback | 126 | Secondary; no winner switch |

| Shaft phase | Primary rows | A errors | C errors | A accepted | New fused accepted |
| --- | ---: | ---: | ---: | ---: | ---: |
| −1.0 rad | 7,946 | 14 | 14 | 2,291 | 2,231 |
| +0.3 rad | 7,505 | 64 | 67 | 2,158 | 2,159 |
| +0.9 rad | 7,972 | 46 | 46 | 2,453 | 2,458 |
| Pooled | 23,423 | 124 | 127 | 6,902 | 6,848 |

The old ambiguous/motion-eligible primary domain stays fixed at **7,970** rows;
it is not recomputed after reducing the candidates. Acceptance changes
**86.5997%→85.9222%** there and **29.4668%→29.2362%** over all primary rows.
Both coverage constraints use the same accepted numerator, so their failures
are not independent evidence. New accepted hypotheses are wrong on 16/6,848
rows, versus native N's 301 errors on exactly those rows. Old accepted F has
18/6,902 errors. Better accepted-only precision does not substitute for C's
complete-domain errors and coverage.

All point comparisons below use the **same full 23,423 rows**, with 100%
coverage, wrong families and native fallback retained. The reference is the raw
dominant-family normal-pressure centroid; it is not I-60's different equivalent
raw target.

| Point distance, mm | Native N | Guarded A | New C |
| --- | ---: | ---: | ---: |
| RMS | 3.13609 | 2.75906 | 2.90997 |
| 95th percentile | 7.38848 | 5.79700 | 6.34283 |
| Maximum | 26.90764 | 29.84286 | 25.56590 |

**C improves RMS, p95 and maximum against N. Against A, it improves maximum
but worsens RMS and p95.** The configured rule does not dominate either
comparator across every objective. Its worst-point reduction is real on this
dataset, while better contact identity or controller tracking is not established.

All 471 mixed-pressure pair rows remain in the 23,894-row loaded-set domain.
Every policy here returns one family and cannot represent those pair sets;
C has **598 loaded-set errors = 127 + 471**, versus A's 595. This iteration
does not demonstrate general multi-contact estimation.

## What changed at the difficult events

The original 294 useful, six harmful and eighteen accepted-wrong opportunities
are retained as fixed cohorts. C preserves **286/294** useful corrections,
repairs **1/6** harmful overrides and **1/18** accepted-wrong rows. Across all
primary rows it gains six correct decisions and loses nine versus A.

The [fixed physical-candidate plots](evidence/publication/plots_stage02/physical_candidates.png)
show both a useful correction and the unresolved wrong-drive case:

- **−1.0 rad, 7.875 s:** both A and C correctly choose shell A. Native point
  error is 13.67 mm; A is 6.47 mm and C is 8.00 mm. Anchoring to an imperfect
  native point can give back some of the old geometry improvement.
- **−1.0 rad, 35.205 s:** C still wrongly selects shell B against sampled
  belt-B pressure, but point error decreases **29.84→25.57 mm**. The native
  point is 15.16 mm from that reference. A smaller point spike does not fix
  this actuation-family assignment.
- **+0.9 rad, 12.555 s:** new acceptance rejects the old harmful override and
  retains N. Point error falls **20.56→0.55 mm**. This is one repaired event,
  not a general slip detector.

The other registered anchors and all six harmful rows remain in the
[machine report](evidence/evaluation/stage01/science.json). The
[fixed +0.3 rad timeline](evidence/publication/plots_stage02/fixed_window.png)
keeps label decisions, acceptance and point error separate. No event selects
parameters. C acquires all 71 defined pressure transitions with no misses and
20 ms absolute-latency p95. It has 22 reverted attempts and 30 false changes,
versus A's twenty and thirty; these are descriptive, not additional gates.

## Decision, limits and verification

Four of ten gate rows fail: pooled C error≤A, +0.3 rad C error≤A, and both
redundant acceptance-coverage comparisons. The other phase identity checks,
point coverage, RMS, p95 and maximum guards pass. **Keep the configured selector
out of live control.** Preserve the spatial tradeoff and rejected configuration;
do not change the confidence thresholds after seeing these results or restart
the stopped HMM. The experiment tests this choice rule, not every geometry-first
method or the user's broader velocity-consistency idea.

The anchor is estimated and sometimes biased; nearest-to-anchor is not
nearest-to-truth. Slip can make an unloaded body's velocity look more consistent
than the loaded body's. Pressure truth samples the last completed 1 ms physics
solve, while observer information covers the preceding filtered 5 ms; it does
not identify all force-bearing bodies or first physical touch. These existing
qualifications do not justify silently relabeling failures or resuming the
parked solver investigation. No new balance, trajectory or hardware gain is
claimed.

Thirty-nine new contract functions pass in four disjoint files: core sixteen,
consumer six, input projection four and evaluation thirteen. Five reported
subtests are included, not added as five more tests. The earlier twelve-test
evaluation stage overlaps the final thirteen; preliminary input tests likewise
overlap their final version. Frozen I-60/I-70 tests are reused history. This is
a focused subset, not the full repository suite. Four separate independent
verifier contracts also pass, giving 43 new focused contract functions in total.

The isolated NumPy replay reproduces **237 prediction arrays** across all three
cases exactly and verifies unchanged decoded inputs. Source/import bindings,
compact receipts and omitted duplicate outputs are recorded under
`evidence/publication/portable_tools` and `evidence/publication/replay_qa`.
Independent selection/rank verification is retained separately under
`evidence/review/independent_selector`; it imports neither production selector
nor frozen ranker. All 79 fields per case agree: choices, IDs, masks, flags,
families, physical points and fallback exactly; largest numeric difference is
2.22e−16. The first reference used infinity for unavailable family distance,
while the production API uses NaN. Both stages are preserved; the correction
changes only that diagnostic sentinel, not selection, scores, gates or outputs.
Prefix/batch and immutability checks pass. Mathematical/numerical checks do not
turn the failed gates into acceptance.

From this folder's `evidence/` directory, repeat frozen inference without
Gazebo, MuJoCo, ROS or SciPy:

```bash
python publication/portable_tools/replay_inference.py \
  --package . --output /tmp/i72_new_inference
```

Use a new output directory. Python 3.10 and NumPy 1.26.4 are the producer
environment; cross-version byte identity is not claimed. The repository-bound
`run_case.py` CLI checks original paths; use the portable wrapper for relocated
evidence. Self-contained projection and scoring commands are recorded in the
[portable evaluation-input README](evidence/publication/evaluation_inputs/README.md);
the [evaluation record](evidence/evaluation/README.md) retains original source paths.
Projection must replace
new-method points from fresh predictions while retaining old truth/N/F/A and
the fixed original ambiguous-motion mask. The first figure stage is preserved;
the second clarifies marker labels only.

The [export manifest](evidence/export_manifest.json) and
[transfer receipt](evidence/transfer_receipt.json) bind the copied evidence and
record every omission. Only interpreter caches and the three regenerated replay
prediction duplicates are omitted; canonical predictions, full-rate evaluation,
both verifier stages, tests and source snapshots remain. The compact historical
evaluation inputs preserve all 57 whitelisted arrays exactly, so inference and
evaluation can run without the original large rollouts or adjacent iterations.
The first final-package check stopped before archive creation because its
helper expected the older `files` manifest key rather than this export's
`copied_files`. The failed helper/log are preserved under
`publication_tools/attempt_stage01`; the correction changes packaging only.
