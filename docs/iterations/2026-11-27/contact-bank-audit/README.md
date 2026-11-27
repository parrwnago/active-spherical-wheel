# I-67 — candidate availability versus point selection

2026-11-27. **Completed offline capacity audit.** All 29 I-66 reference-point
repairs can be reproduced by a usable candidate already in the original bank.
The next target is point selection and its measurement assumptions; these
events do not require a larger bank to reproduce that particular benefit.
No controller, observer, feedforward, score, candidate or gate is changed.

[Comparison PDF](evidence/publication/plots_stage02/bank_capacity_comparison.pdf)
· [Overview PNG](evidence/publication/plots_stage02/overview.png)
· [Eight-page candidate-cloud appendix](evidence/publication/plots_stage02/event_windows.pdf)
· [Numerical report](evidence/analysis/stage01/science.json)
· [Frozen protocol](evidence/protocol/protocol.json)
· [Standalone ChatGPT context](../../../CHATGPT_CONTROL_CONTEXT.md)
· [Previous geometry/load audit](../contact-geometry-load-audit/README.md).

## Question and comparison

I-66 replaced the known-correct family's physical candidate point with its
recorded normal-load centroid. That repaired 27 previously rejected useful
corrections and two additional harmful preferences. Was the original bank
missing useful alternatives, or did its selection objective prefer other points?

Use the same three native I-58 Gazebo/DART Mars figure-eight recordings:
shaft -1.0/+0.3/+0.9 rad, yaw45°,44s,5ms, command bounds
`[1.5,0.6,0.6 N m,0.3,0.3 m/s]`. There is no new driving episode or physical
tracking result. Live gain-scheduled LQR, momentum observer, geometry locator,
rolling/holding feedforward and 120ms incoming-drive preparation stay selected.

The audit region contains **705 saved rows**: all 294 useful and six harmful
primary opportunities, plus the fixed 14 I-66 event windows at ±100ms.
It includes all 41 I-65 lost useful frames. Overlapping windows are deduplicated;
all cohort denominators remain unchanged. No new favorable event is chosen.
All three episodes have been examined before; this is retrospective diagnosis.

The **original 102 slots** are reconstructed from known cached pose, original
intrinsic-normal hint, patch axes/bands and radius. There are 17 normal seeds:
one central, eight at5°, eight at10°. Each has six slots: shellA, shellB,
beltA, beltB, rimA, rimB, mapping to four material families. IDs remain
`6*seed+slot`. Duplicated points are not distinct contacts or independent samples.

The usable set is exactly the old rank-active set: original geometry/force
validity, finite nonnegative score inputs and velocity vectors, valid frame and
finite excitation. The 10° cone,1N compression, scales and confidence rules
are unchanged. We also reconstruct the original projections **before admission**
for a separately labeled distance check; excluded slots are never called usable.
No extra seed, normal, point or projection is introduced.

## Three different point choices

Within the evaluation's known-correct family, compare:

1. The original frozen fused-selected point: F if correct, otherwise Nalt.
2. The usable point nearest the sampled family centroid in three dimensions.
3. The usable point minimizing the frozen I-65 joint-load score.

The native equivalent point N is not the physical Nalt candidate. Neither
nearest geometry nor minimum load score is assumed to be a deployable rule.
The centroid is a last-reported1ms normal-weighted sample; the raw observer
load spans the preceding5ms. Interior centroids stay raw, with radial deviation
reported rather than being projected onto the sphere.

For each candidate in the correct family, predict joint load using the same
endpoint pose and raw observer force:

```text
tau_candidate = Lk Fraw + Ak ((p_candidate - centre) × Fraw)
e = (raw_joint_load - tau_candidate) / (0.03 + 0.10 R ||filteredF||)
score = (e-mu)^T C^-1 (e-mu)
```

The held-out I-65 `mu,C`, wrong-family point, pattern-ambiguity limit0.01 and
relative score-tie rule1e-9 are fixed. We check **whether any usable candidate
gives the correct relative preference**, not just whether its score is low.
A score minimum can be ambiguous. Only the 300 distinct-family opportunities
receive this paired capacity test; extra events with neither original family
correct receive geometry-only output.

The correct family is supplied by **evaluation truth**. This is conditional
capacity, not a newly implemented estimator, calibrated probability or achieved
error reduction. Since the original selected point is a bank member, existence
cannot lose a previous success; that is a set-inclusion check, not scientific
evidence that a selector improved.

## Result: all 29 reference repairs already have usable alternatives

The primary target is met: **29/29 I-66 newly correct decisions are supported**,
with zero unsupported frames. These are all 27 repaired lost-useful frames and
the two additional harmful rejections. No admission relaxation is required.

| Fixed cohort | Original selected-point correct preference | I-66 sampled-point reference | Any usable correct-family bank point | Nearest-geometry bank point |
| --- | ---: | ---: | ---: | ---: |
| I-65 lost useful | 0/41 | 27/41 | **27/41** | 25/41 |
| All useful | 253/294 | 276/294 | **280/294** | 275/294 |
| Harmful, correctly reject wrong F | 2/6 | 4/6 | **5/6** | 3/6 |

Every focus/harmful row has a known usable correct-family candidate. On the
41 focus states, the reference and bank repair exactly the same 27 rows; both
fail on the other 14. On harmful frames, one extra bank point wins where the
sampled reference does not, while one harmful frame remains unresolved.
That additional win need not mean better physical localization.

| 3D distance to sampled centroid | Selected, median/p95/max | Nearest usable, median/p95/max |
| --- | --- | --- |
| 41 focus rows | 6.26 /10.58 /11.46mm | **0.81 /3.40 /4.09mm** |
| Six harmful rows | 23.70 /27.60 /28.12mm | **3.28 /4.74 /4.76mm** |

Nearest pre-admission distance equals nearest usable distance on **every
focus and harmful row**. Thus the tested admission mask is not excluding a
closer original projection there. This does not prove that102 slots cover
every possible contact, terrain or multi-contact state.

Minimum-load-score points are often not the closest: median centroid distance
is3.81mm on focus rows and19.50mm on harmful rows, with harmful maximum33.53mm.
Ambiguous lever directions or model mismatch can favor a low-residual point
that is not well localized. Do not replace the existing selector by pure
load-score minimization based on these oracle counts.

## Why the original selector chose the farther point

The original selector correctly minimizes its own fused objective:

```text
normal_cost = (normal_angle / 10degrees)^2
moment_cost = normalized_net_moment_residual^2
velocity_cost = (tangential_velocity_residual / 0.02m/s)^2
fused_cost = normal_cost + moment_cost + velocity_cost
```

A point can be closer to the sampled centroid yet have a larger fused score.
Every nearer alternative in these focus/harmful comparisons has a strictly
higher unchanged fused cost. This is an objective/model tradeoff, not a broken
argmin implementation.

A separately registered post-result component summary compares alternative
minus selected costs. On the 41 focus rows, the largest positive penalty is
the normal prior in26 cases and velocity in15; on all six harmful rows it is
velocity. Net-moment cost is never the largest positive penalty in these
comparisons. This bookkeeping does not establish physical slip or a universal
weight correction.

At **-1rad,35.205s**, the selected correct-family belt point is26.04mm from
the centroid; an already usable point is4.70mm away. Moving to that nearer
point changes `(normal,moment,velocity)` cost by approximately
`(-0.1375,-0.00407,+5.5402)`. Its velocity penalty outweighs its improvements
in normal/moment terms. Existing minimum-load score43.60 is already below
the unchanged wrong branch's132.65, so a usable ranking repair exists.
The minimum-load point at this same event is33.40mm from the sampled centroid,
farther than the original26.04mm point. A lower score is not localization truth.

At **+0.3rad,13.470s**, a point0.50mm away exists versus the selected4.83mm.
Its larger normal prior outweighs a tiny velocity benefit. No usable point
fixes the frozen load comparison in this example, and the sampled reference
also failed. Simply locating a closer point is insufficient here.

At **+0.9rad,6.850s**, a0.31mm existing point replaces a10.21mm selected point
in the diagnostic and can repair the decision. These contrasting examples
are retained alongside every harmful anchor and the remaining failures.

## Decision and next hypothesis

Do not enlarge the bank or tune weights merely because the original point was
farther from the reference. The bank already reproduces all targeted reference
repairs. Focus the next experiment on **within-family point selection and the
validity/confidence of its velocity and normal terms**, keeping the controller
fixed. A change must be judged against the full useful/harmful/point-error
comparison, not just the29 repair witnesses.

Nor is minimum load score a complete replacement: it can choose a distant
point along weakly observable directions or compensate a load-model error.
The 14 unresolved focus states and remaining harmful frame retain the
interval/load-representation question. Exact force routing over5ms still
requires future matched substep logging; it is not reconstructed here.
No new point selector, sensor, actuation, particle filter or physical test is
implemented in this iteration.

## Verification and reproduction

Feature sources and18 explicit inputs are sealed before reconstruction;
analysis sources and10 inputs are separately sealed before capacity arithmetic.
The hypothesis generator reads only an explicit measurement/geometry/inferred-ID
allowlist, not the truth/reference/covariance fields in the evaluation packet.
ROI selection itself is fixed evaluation bookkeeping.

All original geometry masks reproduce exactly. Original selected F/Nalt points
reproduce byte-for-byte over all26,403 full-rate states. ROI batched coordinates
have at most2.22e-16m difference; finite patterns match and the preregistered
point tolerance is1e-12m. Original fused per-family IDs and full I-65 decision
codes on all300 opportunities reproduce exactly.

Fourteen inclusive geometry contracts and twelve separate analysis contracts
pass:26 distinct focused tests, not a controller/full-repository suite. The
first13-test helper source and its admission-offset clarification are preserved
before the final14-test stage, without adding repeated test counts. Producer
NumPy1.26.4 reproduces all177 canonical arrays byte-for-byte; NumPy1.21.5
preserves discrete outcomes and bounded values while floating bytes differ.
Canonical values are never replaced.

Independent hinge-axis/eigen capacity verification confirms all300 codes,
membership, any-point masks, cohort matrices and29/29 repairs. It shares cached
coordinates, not the primary load/score functions. A first verifier compared
undefined load minima on nine geometry-only rows; its failed scope check and
corrected masked stage are preserved, with no primary scientific change.
The first publication validator used the wrong independent-report flag name;
the stopped check and schema-corrected validator are kept separately. Final
publication QA passes without changing any scientific result. An archive-only
NumPy1.21.5 replay launched from `/tmp` also verifies all177 arrays and scientific
summaries, with exact discrete outcomes and explicit floating-byte differences.

The compact package contains705-row inputs, all102 original projections at
those rows, complete canonical summaries/frames, sources and14 fixed events.
Full original histories and duplicate replay outputs remain local and hash-bound.
The standalone [ChatGPT brief](../../../CHATGPT_CONTROL_CONTEXT.md) explains
the full dynamics/controller/observer system and this research sequence.

From the repository root, with fresh output directories:

```bash
python3 docs/iterations/2026-11-27/contact-bank-audit/evidence/analysis/bank_audit.py \
  --protocol docs/iterations/2026-11-27/contact-bank-audit/evidence/protocol/protocol.json \
  --packets-root docs/iterations/2026-11-27/contact-bank-audit/evidence/inputs/packets_stage01 \
  --banks-root docs/iterations/2026-11-27/contact-bank-audit/evidence/features/bank_stage01 \
  --output /tmp/i67-replay

python3 docs/iterations/2026-11-27/contact-covariance-ranking/evidence/review/verify_replay.py \
  --reference docs/iterations/2026-11-27/contact-bank-audit/evidence/analysis/stage01 \
  --actual /tmp/i67-replay --output /tmp/i67-replay-check
```

Cached analysis needs Python and NumPy only, with no simulator or original
model assets. Original full-history point reconstruction also needs the locally
bound context/pose packets; it is separate from this cached capacity replay.
