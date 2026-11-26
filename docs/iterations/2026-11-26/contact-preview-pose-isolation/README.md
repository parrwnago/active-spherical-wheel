# I-45 — decide whether to continue mapped contact preview

2026-11-26. **Completed; current map-to-contact integration parked.**
The user approved the proposed comparison below. I-44 sources, reports and sealed
presentation bundle remain historical evidence; this diagnostic does not replace
their results. No new physics execution or live-controller change is planned.

## Why one more diagnostic is worthwhile

[I-44](../terrain-map-preview/README.md) replayed unchanged LQR commands, so its
video could not demonstrate improved motion. The separate contact forecast also
regressed: 7.903→11.304 mm RMS on 271 matched future equivalent-point pairs.
Memory supplied terrain at 84.7% of uniform CV query locations, but only 3/33
surface queries and 1/33 contact-candidate queries during the dense bump window.
Better terrain information has not established a useful contact predictor.

The forecast assumes constant body/joint velocity. We have not yet isolated
that assumption from map/contact geometry using actual future configurations.
[I-41](../../2026-11-25/figure8-event-audit/README.md) supplied future applied
commands to a private dynamics model; it did not supply future actual states.
[I-42](../../2026-11-25/handoff-factorial/README.md) changed feedback load shares
and synchronization, without moving the initial departure. Earlier broader
contact-record substitution tested a different phase/window; it does not rule
out every full-information controller experiment. Repeating a broad live oracle
is less targeted than first explaining I-44's contact-forecast disadvantage.

## Proposed single experiment

Use the same saved Mars +0.9/yaw45° trace, 119 anchors and 50/100/200 ms horizons.
At each anchor accumulate exactly the delivered four-camera observations and
freeze memory, observer normal and observer tangent. Compare two terrain
providers under two configuration inputs:

| Terrain provider | Configuration supplied to geometric calculation |
| --- | --- |
| Frozen current observer tangent | Existing constant-velocity preview |
| Frozen four-view memory | Existing constant-velocity preview |
| Same frozen observer tangent | Saved actual future body and joint configuration |
| Same frozen memory | Same actual future configuration |

The last two are explicitly hindsight diagnostics, unavailable online. They
must not receive future depth, terrain geometry, observer updates, load labels
or contact forces during candidate generation. Future pose changes where the
frozen map is queried, but cannot create observations of unseen terrain.
Candidate checks and geometry settings stay identical. Future contact records
remain scoring only. Existing CV results should reproduce before contrasts are
interpreted; preserved I-44 results must not be overwritten.

Report availability/abstentions across all fixed slots first, including the
early window. Compare errors on fixed commonly eligible slots across the four
conditions and report exclusions. Use saved contact counts/load sets to separate
single-contact cases from multi-contact equivalent-point cases: a load-weighted
average need not be a physical contact point. Retain the legacy equivalent-point
metric separately for continuity, rather than silently redefining I-44's score.
Score candidate identity and ambiguity as well as location; broad sets do not
automatically count as accurate predictions.

This is one saved-data diagnostic, not a new trajectory suite or controller.
The original proposal required numerical matching/timing rules, practical
thresholds and input hashes before execution. These are now registered below.

## Decision branches

- If memory remains worse with actual future configuration, motion extrapolation
  alone cannot explain its disadvantage. Park the current map-to-contact/control
  integration and investigate the geometric representation/metric before more
  camera, SLAM or controller development.
- If memory becomes materially better only with actual future configuration,
  investigate the motion predictor next while retaining the live LQR baseline.
- If early coverage stays absent, do not present this map as a demonstrated
  remedy for the 6.86 s departure. Keep visibility separate from geometry error.

For driving improvement, the known live-control constraint remains relevant:
I-42 logged about 0.835 m/s requested against a 0.300 m/s assumed belt limit
before the departure, and flat-ground tests also showed overshoot. Those
observations do not prove one cause, but they justify returning to feasible
references/actuation and turning response if the contact-preview branch fails.
Prior allocation/tilt-shaping regressions remain preserved; this is not a
recommendation to repeat their tuning blindly.

Decision now: one bounded diagnostic is justified; expanding sensing or
connecting map outputs to live commands is not yet supported. The four-camera
memory idea may have other uses even if this particular contact predictor is
parked. No test outcome, improved tracking or hardware capability is claimed at
this registration stage.

## Registered implementation scope

Local family: `results/contact_preview_pose_20261126/`.
[Protocol](evidence/registered/protocol.json) fixes all 119 anchors and three
horizons, creating 357 unique query slots under four conditions. Sensor, map
and contact settings remain those of I-44. Eleven prior inputs are hashed;
pre-change source snapshots and 14 existing presentation bundles are preserved.
Current I-44 runtime/test sources match its sealed test manifest before work.

Generation will receive only an anchor-frozen query provider, known robot
geometry and either the CV endpoint or saved future qpos. The hindsight branch
passes future qpos with zero velocity and zero additional horizon through the
unchanged geometric preview, permitting quaternion normalization. It does not
update the observer or ingest future observations. Score truth only afterward.
No new scene/raycast is needed; all depth is reused. Do not duplicate the large
map snapshot or alter the old I-44 runner/module.

### Truth eligibility refinement before outcomes

Auditing the original logger revealed that saved per-patch contact counts
include only individual forces above 0.05 N, while patch points average **all**
forces. A count of one is not proof of exactly one physical contact. The primary
target is therefore called a **single resolved-contact patch mean**:

- Total resolved count equals one.
- Exactly one patch has load >=max(0.05 N, 5% of model weight).
- That patch has resolved count one and a finite recorded patch point.
- All other patch loads together are <=0.05 N.

Weaker contacts within the selected patch may still affect its mean. Other
truth classes and the legacy global equivalent-point score remain separate.
Saved future configuration is an integrated state; contact records come from
the last completed 1 ms solver step. Retain the same index convention without
alignment tuning; hindsight does not imply zero geometric error.

### Registered prioritization criteria

The primary uniform comparison uses a fixed four-way complete/nonempty
intersection, independent of predicted patch correctness. It needs at least
30 primary eligible slots and at least 50% of all eligible uniform slots.
Report all-slot coverage and exclusions before conditional errors.

Material memory advantage over the observer tangent requires both >=10% and
>=0.5 mm lower RMS, p95 no more than 0.5 mm worse, matched identity recall no
more than 5 percentage points worse, and matched ambiguity no more than
5 percentage points higher. Oracle-map complete/nonempty coverage must reach
70% of uniform slots and lose at most 5 percentage points against CV-map
coverage. It is not compared against the observer's assumed plane coverage.
Apply the same error/identity criteria to CV on the same matched set.

These thresholds choose an investigation branch, not a live-control acceptance
gate. Retain an inconclusive result if coverage/common support is insufficient.
If the oracle map remains worse, park current integration; if only the oracle
map has material advantage, investigate motion prediction. If both have an
advantage in the resolved-contact stratum, inspect metric/aggregation effects
before attributing the I-44 regression solely to motion. Dense early coverage
remains a separate limit in every branch. No outcome-dependent retuning.

## Completed diagnostic

Added [contact_pose_diagnostic.py](../../../../src/spherical_wheel/contact_pose_diagnostic.py),
[diagnose-contact-preview-pose.py](../../../../scripts/diagnose-contact-preview-pose.py)
and two test files. All prior runtime/scripts/tests/settings remain unchanged.
No scene, depth capture, physics step or controller command is executed.

One saved 44 s Gazebo/DART Mars figure-eight supplies 119 anchors and three
horizons: **357 unique slots and 1,428 condition rows**. These are nested
observations in one historical run, not new robot trials. Uniform/dense strata
overlap at 6.8 s. All **714 CV rows** reproduce I-44 geometry, statuses,
candidates and legacy error within 1e-10, with zero failures. All 119 delivery
prefixes match; the first 17,595-cell prefix matches exactly. All eleven input
hashes, source hashes and map-lock checks pass before/after execution.

### Same 244 eligible comparisons in all four conditions

Of 327 uniform slots, 292 satisfy the resolved patch-mean target rule. The
fixed four-way complete/nonempty set has 244 (83.56% of eligible slots): 35
slots are truth-ineligible, 47 have incomplete previews and one has no candidate.
Wrong predicted identities are retained.

| Frozen surface / supplied configuration | RMS mm | Median mm | p95 mm | Maximum mm | Correct patch in candidates |
| --- | ---: | ---: | ---: | ---: | ---: |
| Observer tangent / CV | 7.951 | 1.819 | 18.357 | 38.046 | 242/244 (99.18%) |
| Four-view memory / CV | 11.492 | 3.960 | 24.399 | 59.723 | 237/244 (97.13%) |
| Observer tangent / actual future configuration | 7.476 | 0.209 | 19.402 | 34.231 | 244/244 (100%) |
| Four-view memory / actual future configuration | 7.722 | 3.516 | 18.413 | 20.885 | 233/244 (95.49%) |

The target is a **single resolved-contact patch mean**, qualified above, not
an exact collision point. The metric is nearest geometric-candidate distance;
ambiguous sets can be optimistic. Actual future configuration is diagnostic
hindsight, unavailable online. No force or tracking improvement follows.

Actual configuration reduces memory RMS by about **32.8%** and narrows its
RMS disadvantage from 3.540 to 0.245 mm. Motion prediction contributes
substantially. However, memory remains 3.28% worse than the observer with the
same actual configuration; both material RMS improvement gates fail. Median
and identity are also worse. Preserve the mixed evidence: memory's actual-pose
p95 and maximum errors are lower, so this is not uniformly worse on all metrics.

### Coverage and the early event

| Complete/nonempty previews / all uniform slots | Count |
| --- | ---: |
| Observer / CV | 326/327 |
| Memory / CV | 276/327 |
| Observer / actual future configuration | 327/327 |
| Memory / actual future configuration | 276/327 |

Both memory conditions cover 84.40% of uniform slots. This is complete contact
availability, distinct from I-44's 84.71% centre-surface availability. All
registered common-support and coverage checks pass.

In the dense 6.7–6.9 s window, memory centre-surface availability rises from
3/33 under CV to 7/33 with actual configuration, but all seven latter cases
have incomplete contact-foot regions: **zero complete contact previews**,
versus CV's one. There are zero four-way commonly supported dense slots; no
matched dense RMS is reported. Accurate future configuration does not establish
a fix for the initial departure or create missing observations.

### Legacy metric and registered decision

On 271 legacy uniform four-way matched equivalent-point slots, CV RMS is
exactly the earlier observer 7.903 / memory 11.304 mm. Actual configuration
gives 7.393 / 7.602 mm. The old result is preserved separately rather than
silently redefined by the new resolved patch-mean target.

Registered branch: `park_current_map_to_contact_integration_oracle_memory_worse`.
Support, coverage, p95, identity-loss and ambiguity checks pass; the two RMS
improvement checks fail. Keep native LQR, momentum observer, feedforward and
synchronization unchanged. Retain memory offline and stop expanding this
particular map-to-contact integration on these results.

This does not reject terrain sensing generally or isolate one residual cause.
Local-plane smoothing/geometry, ideal rims, patch-mean targets and the 1 ms
contact/state convention remain possible contributors. No post-outcome tuning,
new motion model or live oracle is introduced to rescue the result. Actual
driving work should return to separately declared reference/actuator feasibility
and turning-response questions, retaining earlier negative trials.

### Validation and presentation

**61 unique selected tests pass in each environment:** 28 new and 33 existing
terrain-memory/campaign cases. Environment repetitions/development stages
overlap and are not additional unique tests or the full repository suite.
Exact pre-command source snapshots and both logs are sealed. A first fixture's
1e-16 quaternion expectation failed by 1.11e-16 under legacy double normalization;
its failed source/log and corrected legacy-equivalence check remain preserved.
The algorithm did not change. An unavailable unqualified `python` preflight
was corrected to the repo interpreter before calculations; its setup note stays.

Independent raw QA verifies every row/target/class, endpoint, delivery clock,
matched mask, conditional statistic and gate without rerunning the mapper or
physics. Main offline runtime is 45.56 s; no real-time guarantee follows.
Presentation QA preserves previews, decoded frames, fixed display clocks and
media hashes. An initial media-manifest pass captured an active redirected
log; the preserved correction excludes that active log, without changing media
or scientific outputs.

From the repo root, with the declared inputs available:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/diagnose-contact-preview-pose.py --output results/contact_preview_pose_20261126 --tag stage01 --run
```

Use an unused tag for a new execution; existing stages cannot be replaced.
Without `--run` only input hashes are checked. The large prior full-map snapshot
and I-44 archive are immutable dependencies referenced by hash, not nested into
the I-45 bundle. Regenerate the snapshot from I-44 inputs/source if needed.
Restore captured source at original paths in an isolated checkout for historical
reproduction.

- [Primary report](../../../../results/contact_preview_pose_20261126/campaign/stage01/report.json)
- [Independent QA](../../../../results/contact_preview_pose_20261126/review/final_report.json)
- [Comparison figure](../../../../results/contact_preview_pose_20261126/plots/primary_and_coverage.png),
  [PDF](../../../../results/contact_preview_pose_20261126/plots/primary_and_coverage.pdf)
- [Diagnostic video](../../../../results/contact_preview_pose_20261126/videos/contact_pose_diagnostic.mp4):
  2×2 candidate points/status/errors, with hindsight explicitly labeled;
  fixed early 6.4–10 s and late 30–36 s, no repeated driving-improvement claim.
- [Presentation bundle](../../../../results/contact_preview_pose_20261126/presentation-pack.zip)

Compact evidence is under `evidence/completed_stage01/`; full per-query outputs
and raw inputs remain local or in the identified bundle/dependencies. Original
I-44 source/data/bundle remain unchanged. Both favorable tails and unfavorable
RMS/median/identity results appear in the presentation.
