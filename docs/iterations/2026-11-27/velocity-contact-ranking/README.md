# I-60 — infer contact from measured velocity

2026-11-27. **The velocity signal improves offline contact-family ranking. It
does not yet justify replacing the contact geometry used by feedforward.**

This implements the user's proposal: compare measured motion with the motion
expected under different contact geometries. Native LQR, momentum observer,
rolling/holding feedforward and incoming-drive preparation remain unchanged.
There are zero new Gazebo physics runs, new compensating commands or trajectory
improvement claims. This ranks current contact; it adds no dynamic contact
forecast to the existing 120 ms preparation preview.

[Comparison and all fixed handoff windows (PDF)](evidence/analysis/plots_stage02/contact_ranking.pdf)
· [Overview PNG](evidence/analysis/plots_stage02/overview.png)
· [Canonical report](evidence/analysis/stage01/science.json).

## What was tested

Use all 26,403 saved states from the three native I-58 recordings: Gazebo/DART
Mars figure-eight, 44 s, yaw 45°, initial common-shaft phases -1/+0.3/+0.9 rad.
Their original input caps remain [1.5, .6, .6 N m, .3, .3 m/s]. No P allocator
states or fresh simulations are part of this comparison.

The estimator receives current simulated body/joint pose and velocity, the
previous actually held belt speeds, the observer wrench, and known robot
geometry. **Desired speed and the next requested command are not inputs.**
The belt command represents achieved surface motion only under the existing
ideal prescribed-speed belt model. Body/joint measurements remain exact
simulation state, without added measurement noise.

The existing `rolling_consistency` diagnostic checks a supplied contact. The
new opt-in [contact_hypotheses.py](../../../../src/spherical_wheel/contact_hypotheses.py)
searches alternatives using private kinematics, without reading contacts,
terrain or ground truth. The live controller does not import this module.

Candidate search uses the observer's intrinsic normal, matching the existing
locator rather than the separately filtered terrain normal. Seventeen normal
seeds (centre, 5° and 10° rings with eight azimuths) project onto four spherical
patch bands and two exposed shell rims: 102 slots, with invalid candidates
masked. Candidates must satisfy material geometry, a 10° normal cone and at
least 1 N compressive estimated force. Rim candidates retain their shell's
actuator family; their radial normal is a restricted local model, not general
edge/multicontact reconstruction.

For each candidate, compute tangential material-point velocity from measured
body/joint motion and prior held belt motion. Rolling without slip predicts
zero tangential velocity. Candidate scores compare:

| Arm | Complete score minimized within each actuator family |
| --- | --- |
| G | Squared normal displacement / 10° scale |
| WG | G + squared moment mismatch / `R*max(norm(force),1 N)` scale |
| VG | G + squared tangential velocity residual / 0.02 m/s scale |
| F | G + both residual terms |

The minimum must belong to one complete candidate; components are not minimized
at different points. Compare four actuator families (shell A/B, belt A/B), not
102 duplicate points. These are uncalibrated costs. The observer and velocity
check share state inputs, and the search domain is observer-conditioned:
**this is additional contact inference, not an independent observer replacement.**

The [protocol](evidence/protocol/protocol.json),
[input/source hashes](evidence/protocol/input_source_seal.json),
[final pre-execution source binding](evidence/protocol/pre_execution_source_seal.json)
and [blind review](evidence/review/design_stage01/preexecution_source_review.json)
precede actual scoring. All three feature/rank artifacts were sealed before
truth labels were opened for evaluation.

## Primary result: better family labels

Primary evaluation is the half-open moving interval [1,41) s. A state is
**dominant-support eligible** when at least 1 N total load is logged and one
family carries at least 90% of it. This is not a claim of a single physical
contact: up to 10% load can remain elsewhere, and one family can contain several
points. Unknown predictions, missing candidates and wrong singleton predictions
count as errors on all eligible states.

| Starting phase | Eligible states | Native errors | F errors | Native error rate | F error rate |
| --- | ---: | ---: | ---: | ---: | ---: |
| -1.0 rad | 7,946 | 51 | 11 | 0.642% | 0.138% |
| +0.3 rad | 7,505 | 226 | 51 | 3.011% | 0.680% |
| +0.9 rad | 7,972 | 135 | 47 | 1.693% | 0.590% |
| Pooled | 23,423 | 412 | 109 | 1.759% | 0.465% |

That is **73.5% fewer sampled label errors**, with 100% true-family candidate
coverage. Native, G and WG have the same error counts. VG and F have identical
raw family labels at all 26,403 states: no added label benefit from the moment
score is demonstrated here. F remains the preregistered primary arm; this is
not a retrospective switch to whichever arm looks best.

False label switches during adjacent eligible states with unchanged truth
decrease 17→13, 121→27 and 65→37. All **12 registered shadow gates pass**,
including per-start nonregression and added benefit over WG. Those gates are
correlated engineering checks, not independent experiments or significance tests.

On the preregistered subset whose dominant truth label stays unchanged ±10 ms,
errors decrease **75→3 across 21,983 states**. Thus the improvement is not
restricted to the immediate handoff-edge samples. This does not remove timing
uncertainty: logged truth describes the last completed 1 ms, while the observer
uses a filtered preceding 5 ms wrench. Event timing is descriptive and is not a
measurement of computation delay.

## Acceptance, abstention and failures

A heuristic accepted label requires at least two candidate families, sufficient
measured motion, a score margin ≥0.25, selected velocity residual ≤0.02 m/s,
and best/runner-up residual-vector separation ≥0.01 m/s. Geometry-forced
singletons do not count as velocity confidence. The motion metric includes
translation, body/joint rotation and held belt motion; it is an availability
screen, not proof of identifiability.

F accepts 6,902 states, with **18 errors (0.261%)**, versus 306 native errors
on **exactly those same states**. Acceptance covers 86.60% of the 7,970
geometrically ambiguous, motion-eligible states, but only **29.47% of all
primary eligible states**. The denominators must not be interchanged.
VG accepts 6,901 with 17 errors; the added moment score changes three accepted
decisions and offers no demonstrated improvement over VG.

The [fixed event inspection](evidence/analysis/events_stage01/events.json)
retains both helpful and adverse examples:

- At -1 rad, 7.875 s: native chooses shell B; actual dominant support and F
  are shell A. Their best tangential residuals are 156.7 versus 3.0 mm/s.
- At -1 rad, 35.205 s: native correctly chooses belt B, but F accepts shell B.
  The true belt family's best no-slip residual is 145.6 mm/s while shell B
  fits at 2.3 mm/s. A strong cost margin is therefore not proof of contact.
- At +0.9 rad, 6.710 s: the first new raw disagreement is wrong, but the
  94.6 mm/s residual causes abstention. This is why raw ranking must not be
  applied directly to the controller.

These are the preselected earliest disagreements/errors and maximum-margin
accepted errors, not a favorable-event selection. Slip, an imperfect search
domain, surface/normal modeling and timing can all undermine a no-slip inverse.
The data do not identify one unique physical cause for each adverse event.

## Post-primary point check: average gain, larger extremes

[Point discrepancy comparison (PDF)](evidence/analysis/point_plots_stage01/point_discrepancy.pdf)
· [PNG](evidence/analysis/point_plots_stage01/point_discrepancy.png).

After the positive family result, a separately registered descriptive check
reconstructs each selected point and compares it with the logged contact-point
references. Scores, candidates, thresholds and primary gates remain frozen.

| Matched subset, raw equivalent-point reference | Count | Native RMS | F RMS | Native maximum | F maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| Primary dominant-support states | 23,423 | 3.134 mm | 2.672 mm | 26.91 mm | 44.04 mm |
| F accepted states | 6,902 | 3.870 mm | 2.729 mm | 23.76 mm | 29.84 mm |
| One resolved contact above 0.05 N | 20,851 | 2.992 mm | 2.569 mm | 26.91 mm | 44.04 mm |

Every start improves RMS but worsens maximum point discrepancy. Dominant-patch
means and radial-R comparison proxies yield similar findings. The point can
move about 20.9 mm from the native estimate under the fixed 10° search cone.
This matters to effective rolling radius and feedforward. On the 23,314 states
where F chooses the correct family, RMS improves 3.011→2.300 mm and maximum
distance improves 26.91→25.32 mm. The enlarged overall and accepted extremes
therefore come from remaining wrong-family selections. A low average label
error rate does not eliminate consequential geometry spikes.

Logged points are load-weighted means with the historical last-1-ms convention;
they need not be physical contacts. The legacy field named
`strict_one_physical_point` actually selects one **resolved record above
0.05 N**; weaker contacts still contribute to loads and point means. See the
[count-semantics erratum](evidence/point_check/CONTACT_COUNT_ERRATUM.md) and
[point-check record](evidence/point_check/README.md). The larger extreme cannot
be dismissed by the resolved-count subset, but that subset does not exclude
all weak multicontact/averaging effects.

**Decision:** retain this as a promising shadow contact-family estimator.
Keep the original LQR/observer/feedforward/preparation selected. Before any
compensation, investigate bounded contact-point changes and slip-aware/ambiguous
hypotheses in a controlled comparison. No integration or further tuning occurs
in I-60. Existing drift, overshoot and actuator-feasibility issues remain open.

## Verification and portable reproduction

There are **28 selected core tests: 22 new plus six existing**. Earlier 19/27
runs overlap and are not additive. Seven separate synthetic evaluator contracts
check denominators, window endpoints, ties, timing and gates. Independent
arithmetic reproduces every ranking/evaluation decision and all 12 gates;
direct point-Jacobian checks agree at 27 fixed states. Neither count is a new
robot trial or a rerun of the full repository suite.

With Python 3 and NumPy, reproduce ranking, all evaluation arrays and gates:

```bash
python3 docs/iterations/2026-11-27/velocity-contact-ranking/evidence/analysis/replay_evaluation.py \
  --rank-data-source docs/iterations/2026-11-27/velocity-contact-ranking/evidence/portable_tools/rank_stage01 \
  --evaluation-data-source docs/iterations/2026-11-27/velocity-contact-ranking/evidence/analysis/evaluation_compact_stage01 \
  --output /tmp/i60-evaluation-replay
```

Reproduce the descriptive point statistics separately:

```bash
python3 docs/iterations/2026-11-27/velocity-contact-ranking/evidence/point_check/replay_points.py \
  --data-source docs/iterations/2026-11-27/velocity-contact-ranking/evidence/point_check/stage01 \
  --output /tmp/i60-point-replay.json
```

All timestamps are retained. The compact rank archive preserves the union of
each fixed arm's per-family minimizing candidates, with original index mapping
and byte parity. It reproduces the four frozen rankings, not an arbitrary new
weight search or the complete geometric candidate generator. Full banks/raw
states/model assets remain local and hash-bound. Rebuilding geometric features
requires those listed originals and MuJoCo; reproducing stored ranking and
evaluation arithmetic does not.

The [published ranking/evaluation replay receipt](evidence/publication/published_evaluation_replay/replay_comparison.json)
confirms exact report/gate reproduction and every frame array byte-for-byte.
The [point replay receipt](evidence/publication/published_point_replay.json) and
[independent evaluation audit](evidence/review/evaluation_stage01/report.json)
retain the separate checks. [Archive/source/link verification](evidence/publication/publication_qa_stage01.json).

The [portable inventory](evidence/portable_manifest.json) lists tracked evidence
and intentionally omitted large local arrays. Scientific plots use all three
starts and all fixed windows; the first plot version is retained, and the
second only moves an overlapping legend. There is no new driving video because
the underlying motion is unchanged.

This is our implementation of the user's velocity-consistency proposal, using
NumPy and MuJoCo's kinematics/Jacobians. Related velocity-based contact
localization has been studied on other robots; this is not a direct reproduction
of that method. [Wang et al., IROS 2020](https://publications.ri.cmu.edu/contact-localization-using-velocity-constraints).
