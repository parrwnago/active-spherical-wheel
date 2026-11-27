# I-66 — separate contact-point error from load-model disagreement

2026-11-27. **Completed offline diagnosis; no controller or estimator adoption.**
Candidate geometry contributes to the earlier load-ranking failures, but a
sampled contact-point replacement does not explain every case. Keep the live
gain-scheduled LQR, momentum observer, geometry locator, rolling/holding
feedforward and 120 ms incoming-drive preparation unchanged.

[Comparison PDF](evidence/publication/plots_stage03/geometry_load_comparison.pdf)
· [Overview PNG](evidence/publication/plots_stage03/overview.png)
· [Eight-page event appendix](evidence/publication/plots_stage03/event_windows.pdf)
· [Numerical report](evidence/analysis/stage01/science.json)
· [Frozen protocol](evidence/protocol/protocol.json)
· [Previous I-65 comparison](../contact-covariance-ranking/README.md).

## Question and evidence boundary

I-65's full-vector load score rejected 41 useful velocity-based corrections.
Does the candidate have the wrong point, or does putting the entire observed
interval load on one current contact misrepresent the measurement?

Use the same three I-58 native Gazebo/DART Mars figure-eight recordings:
shaft -1.0/+0.3/+0.9 rad, yaw 45 degrees, 44 seconds, 5 ms, 8,801 states each;
command bounds `[1.5,0.6,0.6 N m,0.3,0.3 m/s]`. There is no new physics run,
trajectory improvement, gain/reference/limit change, threshold search, camera
or contact prediction. Exact recorded simulated pose supplies kinematics.
The recordings have already been inspected; this is retrospective diagnosis.

The source/schema audit establishes an important limitation:

- The observer's raw generalized load describes the preceding **5 ms**.
- Logged contact locations and normal loads describe the **last reported 1 ms
  physics step**. Only one such record is saved per 5 ms control state.
- Per-family points are **normal-load-weighted centroids**, not exact physical
  points or a force/moment-equivalent location. A centroid can lie inside the
  sphere. Retain it without projection and report material validity separately.
- Individual contact/per-family force vectors, their intervening history and
  independent true generalized joint loads were not saved. FLOAT32 total wheel
  force/moment are endpoint context, not an interval accuracy reference.

The executed runner does not instantiate the substep sampler merely present
in its source snapshot. Older substep archives describe different runs and
cannot supply the missing histories. This audit therefore uses a **sampled
centroid reference**, not an exact 5 ms oracle or guaranteed geometry ceiling.
[Source and schema audit](evidence/review/data_boundary/README.md).

## Controlled comparison

Cache load-independent endpoint kinematics at every recorded pose. For family
`k`, let `c` be the wheel centre, `Lk` its linear Jacobian transpose at `c`, and
`Ak` its world angular Jacobian transpose, restricted to the three joint rows.
Then a force at `p` predicts:

```text
joint_load = Lk F + Ak ((p-c) × F)
residual = raw_observer_joint_load - joint_load
```

The generator runs private kinematics/Jacobians only, with physics steps,
forward dynamics, collision and contact-force queries forbidden. Original
I-64 F/Nalt maps reproduce within `3.47e-17`; I-61 centre/angular/centre-load
controls reproduce exactly. Independent hinge-axis/anchor formulas agree.

The primary comparison changes **only the point**. Select the correct-family
physical proposal F first, otherwise the frozen physical candidate Nalt in
the native family. Never select using true-point distance or residual size.
Both use the same endpoint pose, raw interval observer force and raw joint
target. Native equivalent fallback N is a different virtual point; it is not
silently substituted for Nalt.

| Fixed diagnostic | Prediction |
| --- | --- |
| C: physical candidate | Original correct-family F/Nalt point with total raw observer force |
| T: sampled centroid | Replace only that point with its family's recorded centroid |
| W: full inferred wrench | Assign observer force **and centre moment** to that family; allow a free couple |
| S: normal-share proxy | Split total observer force in proportion to sampled family normal loads, using each family's own centroid |

W is point-independent and uses additional moment information from the **same
observer**, not an independent sensor. The raw world moment already uses the
observer's midpoint convention and is not rotated again. S assumes a common
force direction and instantaneous normal-load shares; it is not measured
tangential-force routing or a true interval decomposition. Missing positive
contributors remain unknown, rather than being removed and renormalized.
Neither control becomes a new estimator or scoring rule.

Focus on all **41 useful corrections rejected by I-65**, with context from all
294 useful changes, six harmful overrides, 6,890 accepted correct-family
physical pairs and 23,423 original dominant-support primary states. The primary
mask still permits weaker contacts elsewhere. All denominators and unknowns
remain explicit.

## Geometry helps, but incompletely

All 41 focus states are paired and finite. Their correct-family candidate is
6.26 mm from the sampled centroid at the median, 6.42 mm RMS, and 11.46 mm at
the maximum. This measures the **physical candidate**, not the native
observer's equivalent contact-point accuracy.

| Model on the same 41 focus states | Joint-vector RMS (N m) | p95 (N m) | Within unchanged component bound |
| --- | ---: | ---: | ---: |
| C: physical candidate | 0.13926 | 0.31801 | 39 / 41 |
| T: sampled centroid | 0.10568 | 0.25843 | 39 / 41 |
| W: full inferred wrench | 0.06893 | 0.18713 | 41 / 41 |
| S: normal-share proxy | 0.10568 | 0.25843 | 39 / 41 |

Point replacement lowers pooled RMS **24.1%**, improving 37 frames and worsening
four. It changes one formerly incompatible frame to compatible, but also one
formerly compatible frame to incompatible. A smaller overall error does not
remove every disagreement or establish correct identity.

The registered geometric-improvement criterion **fails** because the sole
-1.0-rad focus frame worsens: 0.11272→0.16406 N m at 32.110 s. The +0.3 and
+0.9 phase focus RMS values improve. This is partial geometric evidence, not
a qualified uniform improvement or a reason to deploy a truth-assisted model.

### What changes in the frozen I-65 decisions?

On the 294 useful and six harmful distinct-family opportunities only, replace
the **known-correct branch's point**, retaining the wrong branch, force scale
and previously fitted held-out I-65 mean/covariance. No calibration is refitted.
Same-family duplicates are never artificially split and rescored.

| Previously recorded group | Original correctly ranked | With sampled point | Newly correct | Newly incorrect |
| --- | ---: | ---: | ---: | ---: |
| All useful corrections | 253 / 294 | 276 / 294 | 27 | 4 |
| Harmful overrides, correctly reject F | 2 / 6 | 4 / 6 | 2 | 0 |
| The 41 rejected useful corrections | 0 / 41 | 27 / 41 | 27 | 0 |

There are no unknowns, ambiguities or ties in these comparisons. These are
**truth-assisted explanatory counterfactuals**, not new achieved live error
rates. Four previously correct preferences deteriorate, mostly in the -1.0
phase. Correct point geometry alone still leaves two harmful preferences wrong.

## Concrete failures

**At -1.0 rad, 35.205 s, candidate geometry is a substantial contributor.**
The physical candidate in the correct belt-B family is 26.04 mm from the
sampled belt centroid. Replacing it reduces its joint-error norm from 0.42181
to 0.20759 N m. Its covariance score falls 137.81→43.08, below the unchanged
wrong shell-B candidate's 132.65. The correct belt is now preferred.
That is evidence about a mislocated candidate within the right family; it does
not establish whether the bank lacks a better point or its ranking chose poorly.

**At -1.0 rad, 32.110 s, moving to the sampled point does not help.**
The correct-family point is only 2.79 mm away. Replacement increases the error
and still prefers the wrong family. Allowing the full inferred wrench lowers
the error to about 0.01621 N m. This localizes a load-representation limitation,
but does not prove a particular friction or timing mechanism.

**Two harmful preferences remain at +0.9 rad, 31.095 and 31.175 s.**
The sampled-point errors are 0.17881 and 0.07986 N m; full-wrench errors are
0.05276 and 0.00747 N m. Active-joint/free-couple disagreement matters here.
W was not recalibrated or used for classification, so a lower W error does
not establish that a wrench-based classifier would solve them.

## A structural limit on any current single-family explanation

A separately registered **post-result** algebraic audit uses the saved joint
ancestry. In the mirrored model:

| Contact family | Joints it can directly load |
| --- | --- |
| Shell A | Common shaft, hemisphere A |
| Shell B | Common shaft, hemisphere B |
| Belt A or B | Common shaft |

A joint outside that body's ancestry has a zero external-load Jacobian row,
for **any point or wrench on that body**. If the raw observer estimates load
in that row, changing the point cannot explain it under the single-family
model. The norm of those rows is an exact model-error lower bound for C/T/W.

At 35.205 s, the observed shell-B joint component is about -0.20663 N m even
though belt B is the sampled support. It remains unchanged under C, T and W.
This is below the old approximately 0.25106 N m component allowance, so the
structural remainder does **not** itself prove failure of that compatibility
gate. Geometry replacement already fixes this event's relative ranking.

Across the six harmful frames, the inactive-joint floor is 0.13518 N m RMS,
72.7% of T's squared residual. On the 41 focus states it is 0.03904 N m RMS,
13.6%; on stable accepted pairs it is 0.00213 N m RMS. These are orthogonal
coordinate lower-bound shares, not causal explained variance. The remaining
two harmful misranks must not be attributed wholesale to this floor.

All 294 useful and six harmful frames have just one **positive sampled family
normal load**, explaining why S equals T there. This allows multiple contacts
within that family and unknown transfers during the preceding interval. It
does not validate or exclude general multi-contact force estimation.
[Post-result structural audit](evidence/review/structural_routing_stage01/output/report.json).

## What is established and what remains open

There are at least two limitations worth keeping distinct:

1. The selected physical point can be misplaced enough to change the winning
   family. Point replacement repairs many saved decisions, including the
   35.205 s example, without changing thresholds.
2. A current point with the entire observed interval force is an incomplete
   explanation in other states. Additional inferred moment helps; some joint
   load cannot be routed through the sampled family at any point.

The logs cannot determine how much of the second limitation is earlier contact
within the 5 ms interval, distributed forces/moments, or observer/model/actuation
error. No computational latency is measured. Sampled net-force/moment differences
are reported with their endpoint/FLOAT32 qualification; no fork load or external
push occurs on the focus states. Neither low residual nor accurate equivalent
point establishes current drive identity.

Next proposed investigation: inspect candidate-bank **coverage versus selected
point** at the fixed geometry-sensitive events before enlarging or rescoring
the bank. Resolving the remaining temporal/load-routing ambiguity would require
matched substep contact-force logging in a later instrumented replay. Neither
follow-up is run here. Keep live LQR, momentum observer and feedforward selected.

## Verification and reproduction

Feature sources and 338 explicit inputs, including 306 original model/asset
paths, are sealed before kinematics. Analysis sources and 24 cached/dependency
inputs are separately sealed before residual arithmetic. Truth never enters
the pose-map extractor. All 26,403 poses are valid; no dynamics or contact
query executes. Fifteen geometry/extraction contracts and seventeen separate
analysis contracts pass. A pre-data generic statistic-label correction is
preserved, with both test stages; these are 32 distinct focused tests, not a
full controller suite or 49 tests from summing repeated runs.

Independent direct hinge-axis/anchor loads agree within `4.44e-16 N m`;
eigen-based frozen-covariance scores agree within `1.82e-12`. Validity,
compatibility, branch choices, cohort counts and all 300 counterfactual codes
match exactly. Producer NumPy 1.26.4 replay reproduces all **177 arrays**
byte-for-byte. NumPy 1.21.5 differs in floating bytes but preserves decisions
and summaries within the registered tolerances. Canonical values are retained.
The published package also passes an archive-only NumPy 1.21.5 replay launched
from `/tmp`, including all 177 array comparisons and scientific summaries.
Publication QA verifies source/input hashes, links, exact array bindings and
the one/eight PDF page counts.

The archive includes cached poses/context, sources, full canonical numeric
outputs and all 14 fixed event windows. It depends on the already tracked
I-61/I-64/I-65 packages. Original large state/model assets and duplicate replay
arrays remain local. Initial figure layouts are retained beside final stage03.

From the repository root, with a fresh output directory:

```bash
python3 docs/iterations/2026-11-27/contact-geometry-load-audit/evidence/analysis/audit.py \
  --protocol docs/iterations/2026-11-27/contact-geometry-load-audit/evidence/protocol/protocol.json \
  --pose-root docs/iterations/2026-11-27/contact-geometry-load-audit/evidence/features/stage01 \
  --i61 docs/iterations/2026-11-27/contact-load-consistency/evidence \
  --i64 docs/iterations/2026-11-27/contact-load-history/evidence \
  --i65 docs/iterations/2026-11-27/contact-covariance-ranking/evidence \
  --context-root docs/iterations/2026-11-27/contact-geometry-load-audit/evidence/inputs/context_stage01 \
  --output /tmp/i66-replay

python3 docs/iterations/2026-11-27/contact-covariance-ranking/evidence/review/verify_replay.py \
  --reference docs/iterations/2026-11-27/contact-geometry-load-audit/evidence/analysis/stage01 \
  --actual /tmp/i66-replay --output /tmp/i66-replay-check
```

Cached-map analysis needs Python and NumPy, with no running simulator. Rebuilding
the original private kinematic caches requires the hash-bound local model
assets and MuJoCo; it is not part of this portable analysis replay.
