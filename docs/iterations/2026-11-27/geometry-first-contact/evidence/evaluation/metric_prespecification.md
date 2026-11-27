# I-72 evaluation specification before ranking outcomes

This is one fixed saved-log selector experiment.  No new physics, HMM, force
fit, threshold tuning, controller change or calibration claim.

The primary policy is **C**: use the new geometry-first **fused** accepted family
and its selected physical point; otherwise retain native **N** family and
original equivalent point.  A same-family accepted update still uses the new
physical point.  N is not Nalt.  New VG (normal plus velocity), its accepted/N
fallback policy, raw new fused and old raw fused are secondary comparisons.
They cannot replace C after outcomes are observed.

Projection occurs only after all three new inference artifacts are sealed.  It
reads frozen I-70 evaluation packets for truth/N/oldF/A points, and the original
I-60 evaluation packet for the fixed ambiguity/excitation domain.  Optional
runner consumer aliases are checked against this exact fallback construction.
All new physical points are from new predictions, never old selected-point
arrays.  The evaluation output is forbidden as an inference input.

Use the unchanged I-70 pressure truth definition and half-open `[1,41)` window:
total normal pressure at least 1 N, a unique family maximum at least 90% for
the 23,423 dominant-primary states.  Full loaded-set metrics retain all 23,894
rows, including the 471 mixed-pressure pairs.  This singleton selector cannot
match a pair set, so those rows remain errors.  Truth is sampled positive
pressure during the final 1 ms of each 5 ms interval, not all force routing.

Acceptance coverage uses the **old** I-60 domain: dominant-primary truth,
original at least two valid families and original measured excitation at least
0.02 m/s.  Do not recalculate eligibility from selected representatives, new
scores, acceptance or point availability.  Report pooled/per-phase coverage
on both this domain and all dominant-primary states.

Point reference is the raw sampled dominant-family pressure centroid in world
coordinates.  It may lie inside the spherical surface.  It is not I-60's raw
equivalent-point target or a radially projected proxy.  Fix the comparison rows
using primary truth, finite reference and finite N equivalent point.  Missing C
on any fixed row fails the comparison; do not silently shrink the row set.  Keep
wrong-family points and fallback rows.  Report all-primary, raw and accepted-only
point summaries separately; secondary subsets cannot substitute for primary C.

Ten prespecified gates, all required to qualify this fixed selector for a later
validation proposal rather than adoption:

1. Pooled C dominant-primary errors at most old A's 124.
2. Each of the three phase C error counts at most its old A count.
3. Pooled new acceptance coverage at least old A on the fixed old ambiguity/
   motion domain, and separately on all primary rows.  These two constraints
   share a numerator here and are explicitly not independent evidence.
4. C finite-point coverage no less than N, with no missing C on N's fixed
   reference rows.
5. C point RMS, p95 and maximum each no greater than N plus 1 micrometre
   (0.001 mm), on exactly those fixed rows.

The list has ten numeric gate rows: 1 pooled identity, 3 phase identity, 2
coverage, 1 point coverage and 3 point statistics.  There are no new timing or
per-phase point acceptance gates.  Timing is descriptive, using the exact frozen
I-70 stable-pressure function.  The same helper supplies truth/summary functions;
its previous metric contracts are reused evidence, not new test counts.

Preserve all 294 old useful corrections, six harmful overrides and eighteen old
accepted-wrong rows as fixed retrospective cohorts.  Report new accepted wrong
counts and native performance on those same accepted rows.  Preserve five
protocol anchors: -1 rad at 7.875, 32.110, 35.055 and 35.205 s, and +0.3 rad at
13.470 s; also publish all six old harmful rows mechanically.  No cohort or event
is used to choose parameters or representatives.

Thirteen new synthetic contracts are the final inclusive count.  Stage01's
twelve are preserved; stage02 adds concrete runner-consumer alias checks before
any saved-data evaluation.  Do not add the stage counts or call them the full
repository suite.
