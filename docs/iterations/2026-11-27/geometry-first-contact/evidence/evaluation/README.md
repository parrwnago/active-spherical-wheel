# I-72 geometry-first contact evaluation

This fixed selector completes but **fails four of ten registered gates**.  It
is not adopted or qualified for a later validation proposal under this protocol.
No score, acceptance threshold, secondary winner or controller is changed after
the result.  The useful finding is a spatial tradeoff, not a closed-loop gain.

The canonical immutable report is `stage01/science.json`, SHA256
`54efd644ce76d4108e9e285f28ab445784deda2d6cc18d81748ff75f46cbbc54`.
All three inference outputs were sealed before new truth projection/evaluation.
`pre_truth_execution_stage01/source_input_seal.json` binds five evaluation source
files and eleven inputs before those calculations.  Optional runner native/C/VG
consumer fields pass exact checks against the registered fallback policy.

The selected hypothesis has two separate steps: choose one original physical
candidate per family nearest the native observer equivalent point, then apply
the unchanged I-60 ranking and acceptance rules.  Primary policy **C** uses the
accepted new fused physical point/family; otherwise it retains native **N**
family and equivalent point.  N is not Nalt.  Raw new fused, raw VG and accepted
VG/N fallback are secondary comparisons and cannot replace C after the result.

## Identity, coverage and cohorts

All identity results use the same 23,423 dominant-positive-pressure primary rows.

| Method | Primary errors |
|---|---:|
| Native N | 412 |
| Old raw fused F | 109 |
| Old accepted/fallback A | 124 |
| New geometry-first raw fused | 118 |
| New geometry-first raw VG, secondary | 117 |
| Primary C, new fused accepted else N | 127 |
| New VG accepted else N, secondary | 126 |

C has 14/67/46 errors at starts -1.0/+0.3/+0.9 rad; A has 14/64/46.  Pooled and
+0.3-phase nonregression gates therefore fail.  Compared on identical rows, C
gains six correct decisions and loses nine versus A.  Of 294 old useful
corrections, it preserves 286 and loses eight.  It repairs one of six old harmful
overrides, and one of eighteen old accepted-wrong rows.  Those local repairs do
not override the pooled regression.

New fused acceptance falls by 54 primary rows, from 6,902 to 6,848.  On the
**fixed old I-60 ambiguity/motion domain of 7,970 rows**, coverage falls from
86.5997% to 85.9222%.  On all primary rows, it falls from 29.4668% to 29.2362%.
Both coverage gate lines fail; they share a numerator and are explicitly
redundant engineering constraints, not independent failures or independent
statistical evidence.  New accepted fused outputs contain sixteen wrong-family
rows out of 6,848, versus native N's 301 errors on exactly those rows.  These
acceptance rules are not calibrated confidence probabilities.

All 471 mixed-pressure pair rows remain in the full 23,894 loaded-row comparison.
This single-family selector cannot match their pair sets.  C has 598 full-set
errors versus A's 595, equal to their primary errors plus all 471 mixed rows.
No evaluation denominator is removed to conceal missing multicontact capability.

## Point geometry on identical full-primary rows

All methods below have a finite point on **all 23,423 primary rows**.  The target
is the raw sampled dominant-family positive-pressure centroid.  Native N is the
observer equivalent point; it is neither a physical Nalt candidate nor the
I-60 raw equivalent-point truth target.  Do not substitute I-60's approximately
3.134 mm equivalent-target RMS for the N value below.

| Policy | Point RMS, mm | Point p95, mm | Maximum, mm |
|---|---:|---:|---:|
| N | 3.136094 | 7.388477 | 26.907642 |
| A | 2.759063 | 5.797002 | 29.842859 |
| C | 2.909967 | 6.342826 | 25.565903 |

C passes every registered point gate against N: RMS, p95 and maximum improve,
with 100% coverage and no missing point on N's fixed reference rows.  Relative
to A, C improves the maximum but worsens RMS and p95.  Thus it is a mixed
geometry benefit rather than uniform superiority.  Wrong-family estimates and
native fallback rows are retained; accepted-only statistics are secondary.

The raw new fused point maximum is 28.07675 mm versus old raw F's 44.03825 mm;
the registered C policy still includes its unchanged acceptance/native fallback.
No raw or accepted-only comparison replaces C's full-row gates.  Per-phase point
metrics are retained in the canonical report without being new acceptance gates.

## Fixed examples and timing

All five protocol anchors and all six old harmful rows are preserved:

- At -1 rad **7.875 s**, C retains the useful shell-A correction.  Its point
  error is 7.99830 mm, smaller than N's 13.66944 mm but larger than A's
  6.46754 mm.  A correct family is not automatically the most accurate point.
- At **32.110 s**, C preserves the accepted belt-B estimate and 2.78909 mm
  point distance, compared with N's shell-B identity and 12.45215 mm distance.
- At **35.055 s**, the old harmful shell-B assignment remains accepted despite
  belt-B pressure truth.  At **35.205 s**, that wrong family also remains,
  although point error falls from A's 29.84286 to C's 25.56590 mm.  N is
  15.15505 mm at the latter event.  Lowering the point peak does not fix routing.
- At +0.3 rad **13.470 s**, C preserves the useful shell-B estimate and
  4.83254 mm point distance.
- The repaired old harmful row is +0.9 rad **12.555 s**.  Raw new fused still
  selects shell A instead of pressure-labeled belt A, despite a point only
  0.22978 mm from the reference.  Fixed acceptance abstains, so C correctly
  falls back to native belt A and its 0.54945 mm equivalent-point distance.

Timing is descriptive, using the exact I-70 stable-pressure definition.  C
acquires all 71 events, with zero misses, median absolute latency 0 ms and p95
20 ms.  It has 22 reverted attempts and thirty false switches versus A's twenty
and thirty.  No new timing gate is invented after these outcomes.

## Evidence scope and reproduction

Thirteen new evaluation contracts pass, inclusive of stage01's twelve.
`synthetic_stage02` preserves the final sources and concrete consumer-alias
checks; do not add stage test counts.  Frozen I-70 helper contracts are reused
evidence, not additional new tests or the full repository suite.

The three episodes were already inspected.  Labels are the final 1 ms sampled
positive-pressure support every 5 ms, not all force routing, exact contact
history or first physical touch.  All robot motion remains the original native
LQR/observer/feedforward trajectory.  The spatial improvement does not establish
better balance or tracking, unique physical contacts, or unseen robustness.

From the repository root after all three new inference artifacts are sealed:

```bash
.venv/bin/python results/geometry_first_contact_20261127/evaluation/prepare_evaluation.py \
  --baseline-root docs/iterations/2026-11-27/temporal-contact-modes/evidence/evaluation/packets_stage01 \
  --old-evaluation-root results/velocity_contact_rank_20261127/analysis/stage01 \
  --prediction-root results/geometry_first_contact_20261127/inference/stage01 \
  --inference-seal results/geometry_first_contact_20261127/inference/all_three_sealed.json \
  --output /tmp/i72_evaluation_packets

.venv/bin/python results/geometry_first_contact_20261127/evaluation/evaluate_selector.py \
  --protocol results/geometry_first_contact_20261127/protocol/protocol.json \
  --packet-root /tmp/i72_evaluation_packets \
  --output /tmp/i72_evaluation
```

Use fresh output directories; canonical stage01 evidence is immutable.
