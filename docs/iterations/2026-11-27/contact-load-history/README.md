# I-64 — compare predicted and observed loads over the same time window

2026-11-27. **Matching filter history improves the load model, but does not
qualify a contact veto.** Joint-load prediction RMS falls 36.2% on the fixed
correct-family subset, with improvement at all three starts. Current-family
discrimination still fails. The live LQR, momentum observer, feedforward and
incoming-drive preparation stay unchanged; no new physics or threshold tuning.

[Comparison PDF](evidence/analysis/plots_stage01/load_history_comparison.pdf)
· [Overview PNG](evidence/analysis/plots_stage01/overview.png)
· [Eleven annotated event windows](evidence/analysis/plots_stage01/event_windows.pdf)
· [Canonical report](evidence/analysis/stage01/science.json)
· [Previous condition experiment](../contact-condition-diagnostic/README.md).

## What changed in the comparison

The momentum observer first estimates average generalized external load over
the preceding 5 ms, then applies its 100 rad/s filter. The old load check uses
today's candidate geometry with already filtered force. When geometry or
support changes, those quantities need not describe the same history.

For this diagnostic, use the raw interval force once, predict its three joint
loads at the frozen endpoint contact hypothesis, then pass those predictions
through the same filter. The predicted history is a separate accumulator:

```text
alpha = 1 - exp(-100 * dt)
predicted_branch = predicted_history_previous
                 + alpha * (H_current * raw_interval_force - predicted_history_previous)
```

`H` is the point Jacobian transpose restricted to the three joint torques.
This predicts how an **observed net force** would load the joints under a
contact hypothesis; it does not independently predict the net ground force.

All current alternatives use one common prior history. After scoring, update
that history with the frozen I-60 accepted/native-fallback path A, regardless
of the new scores. Do not rewrite past contact using today's candidate, feed
the diagnostic winner back, or filter an already filtered force again.
The A path is an offline shadow estimate on native-controller measured motion,
not the historically applied routing or recovered true contact history.

Initialize once from the measured observer state. Exclude 200 ms after a
seed/reset from scoring (20 filter time constants). Invalid committed maps,
low observed load or missing time intervals invalidate the history. Missing
alternative maps affect only that branch, not the committed history.

| Fixed model | Comparison |
| --- | --- |
| M0, original | Current endpoint H × filtered force, versus filtered joint load |
| M1, raw control | Same H × raw interval force, versus raw interval joint load |
| M2, primary | Raw prediction with the same filter and frozen A-history prior |
| M3, sensitivity control | Same calculation with native-equivalent history prior |

The native-family comparison uses the original physical candidate within that
family. The native-equivalent point used for fallback history is a different,
virtual assigned-body model; it is not silently projected onto a material
surface. There are 1,817 primary rows where that N-equivalent point fails the
material-sphere/band check, although its virtual Jacobian remains defined.

Endpoint H is held over the interval as an explicit approximation. Rolling
contact can migrate during the 5 ms; these logs do not supply an exact force-
weighted geometry history. No midpoint, guessed 50/50 transfer, true-contact
history, future normal or terrain information is added. This is a causal
interval-end load explanation, not a future-contact forecast.

## Frozen data and questions

Use the three original I-58 native Gazebo/DART Mars figure-eight recordings,
44 s, shaft -1/+0.3/+0.9 rad, yaw 45°, command caps
`[1.5,0.6,0.6 N m,0.3,0.3 m/s]`. Retain all 26,403 full-rate rows. The
original [1,41) primary mask has 23,423 dominant-support eligible rows:
at least 1 N normal load and a unique family carrying at least 90%.
Weaker additional contacts are possible.

All three truth-blind map/history packets are sealed before truth evaluation.
Truth labels select evaluation subsets only. The episodes were inspected in
earlier iterations, so this is retrospective diagnosis, not unseen validation.

Two questions are registered separately:

1. **Q1: does the model become more consistent?** Compare M2 with M0 on the
   same warmed, finite, accepted proposals with the correct current family.
   Require at least 10% pooled vector-RMS reduction, no pooled p95 or per-start
   RMS regression, at least 99% coverage, and every critical event scorable.
2. **Q2: does it distinguish the current family better?** Compare the same
   frozen F and native-family physical alternatives from a common prior.
   Use maximum absolute joint mismatch with the unchanged force-scaled
   normalization; ties/unknowns count as not correct. Require the registered
   useful/harmful preference and nonregression conditions.

Lower raw-to-filtered RMS alone cannot establish improvement: filtering the
interval residual is expected to smooth it. An observed-prior control confirms
that reseeding from observed previous load at **every** step merely scales the
raw residual by alpha. It is not the primary history method.

## Result: Q1 passes, Q2 fails

On the same **6,884 accepted correct-family frames**, vector RMS is:

| Initial shaft | Original M0 (N m) | Matched M2 (N m) |
| --- | ---: | ---: |
| -1.0 rad | 0.04187 | 0.02586 |
| +0.3 rad | 0.08291 | 0.04664 |
| +0.9 rad | 0.04331 | 0.03685 |
| Pooled | **0.05828** | **0.03721** |

Pooled vector p95 falls 0.13549→0.07141 N m. All primary states and all
6,902 accepted proposals are scorable; none of the six harmful overrides,
18 accepted-wrong frames or largest-point event is excluded. All Q1 gates
pass. The shaft channel supplies much of the gain; shell channels are not
uniformly better at every start, as the per-joint report shows.

The current-family preference has a different outcome:

| Evaluation group | Correct preference, M0 | Correct preference, M2 |
| --- | ---: | ---: |
| 294 useful corrections: prefer F | 212 / 294 | 243 / 294 |
| 6 harmful overrides: prefer native-family candidate | 2 / 6 | 1 / 6 |

There are no ties, unknowns or near-ties in these groups. Useful preferences
improve, but harmful preferences regress and the useful rate stays below the
registered 90%. Q2 fails. These are correlated saved frames, not independent
robot trials.

At -1 rad, **35.205 s**, the true/native family is belt B and F wrongly
selects shell B. Lower normalized mismatch is preferred:

| Model | Wrong F score | Correct-family alternative score |
| --- | ---: | ---: |
| M0 instantaneous | 0.68792 | 0.91330 |
| M1 raw interval | 1.24485 | 1.46475 |
| M2 matched history | 0.56821 | 0.65474 |

All three prefer the wrong F hypothesis at this event. M1 rejects F at the
old threshold because **both** mismatches exceed it; that does not establish
that the alternative model fits well. Only about 11% of M2's residual norm-sum
comes from inherited history here. Matching the filter does not provide a
unique current-family explanation at this example.

## Fixed shadow vetoes remain rejected

No coefficients are fitted. Apply the original component tolerance
`0.03 N m + 0.10 R norm(filtered F)` to each model only as a registered shadow
control. Unwarmed history with a valid F map retains A; invalid F retains the
original native-fallback semantics. All primary rows and the fixed common
point mask count, including fallbacks.

| Offline policy | Family errors | Harmful changes remaining | Useful corrections retained / 294 | Point maximum (mm) |
| --- | ---: | ---: | ---: | ---: |
| A, no extra veto | 124 | 6 | 294 | 29.843 |
| M0, original I-61 check | 153 | 5 | 264 | 29.843 |
| M1, raw control | 132 | 3 | 283 | 26.908 |
| M2, matched A history | 128 | 6 | 290 | 29.843 |
| M3, native-history control | 128 | 6 | 290 | 29.843 |

Matching history cuts lost useful corrections from 30 to four, but prevents
none of the six harmful overrides. All four shadow policies fail the unchanged
I-62 performance gates. M1 is a predeclared control, not a post-result replacement
for M2. No controller/estimator integration, new driving run or tracking gain
is claimed.

A [post-result row-set check](evidence/review/rowset_post_result_stage01/report.json)
shows that M2 restores **all 30** useful frames rejected by M0 and introduces
**four different** rejections: +0.3 rad at 14.415 s, and +0.9 rad at
17.530, 17.770 and 24.955 s. Thus the change is a net 26 fewer false rejections,
not the same four old failures remaining. This explanatory appendix changes
no model or registered gate.

## Verification and compact replay

The [protocol](evidence/protocol/protocol.json),
[337-input/model-asset seal](evidence/protocol/input_source_seal.json),
[eight-source freeze](evidence/protocol/pre_execution_source_seal.json) and
[three-packet evaluation release](evidence/protocol/evaluation_release.json)
preserve chronology. Nineteen feature contracts and fourteen separate analysis
contracts pass, inclusive of their development stages; these are not new full
controller-suite counts. An early unexecuted reconstruction across a missing
time interval was corrected before data arithmetic, with the earlier source
and a gap contract retained.

The original observer recurrence reconstructs all nine saved channels exactly.
M0 reproduces old flags/families/point metrics exactly; its residual-vector
difference from I-61 is at most 1.11e-16 N m. Independent recurrence, warm/reset
masks, every row's preference, all Q1/Q2 decisions and fixed-shadow metrics agree.
The largest independent score difference is 8.89e-16.

Producer NumPy 1.26.4 replay regenerates **228 feature arrays and 594 analysis
arrays byte-exactly**. System NumPy 1.21.5 fails strict byte comparison on 84
feature float arrays. The [retained full local pointwise comparison](evidence/features/cross_version_diagnostic_stage01/comparison.json)
measures at most 1.11e-16 N m in predictions/residuals and 1.26e-12 in auxiliary
norm shares. No scientific source, value or threshold is changed.

The compact archive retains original primitives and per-array hashes rather
than duplicate large derived tables. A hash mismatch cannot establish a
per-element floating bound. The separately sealed v2 verifier therefore checks
exact structures, finite patterns, integer/bool arrays and **36 per-frame
decision arrays**, plus scientific summary statistics under the registered
unit-aware allowances. It reports nonexact float bytes openly and does not
claim portable pointwise bounds. Norm shares are auxiliary, not scientific
acceptance gates. The original strict failure and producer exact replay remain.

[Independent comparison](evidence/review/canonical_comparison.json)
· [Strict producer replay](evidence/analysis/replay_producer_stage01/comparison.json)
· [Archive-isolated system replay](evidence/publication/replay_numpy121_stage02/comparison.json)
· [Feature definitions and limitations](evidence/features/README.md).

## Decision and reproduction

Retain matched filtering as a useful model diagnostic. Keep the native LQR,
momentum observer, rolling/holding feedforward and existing 120 ms preparation
selected. Do not add this load veto to the velocity-based estimator. The
result supports correcting the comparison window; it does not establish the
complete physical cause of contact errors or trajectory deviation. Candidate
point quality, within-interval transfer and force distribution remain open.

From the repository root, with Python 3 and NumPy installed, choose a new
output directory and run:

```bash
i64_package=docs/iterations/2026-11-27/contact-load-history/evidence
python3 "$i64_package/analysis/replay_history_analysis_v2.py" \
  --analysis-source "$i64_package/analysis" \
  --protocol "$i64_package/protocol/protocol.json" \
  --base-root "$i64_package/portable_inputs/i62_base" \
  --feature-package "$i64_package/features/portable_stage01" \
  --bindings "$i64_package/analysis/portable_stage02/bindings.json" \
  --local-feature-audit "$i64_package/features/cross_version_diagnostic_stage01/comparison.json" \
  --old-witnesses "$i64_package/portable_inputs/i62_old_witnesses.json" \
  --reference "$i64_package/analysis/stage01" \
  --output /tmp/i64-replay-20261127
```

The [archive-isolated NumPy 1.21.5 v2 replay](evidence/publication/replay_numpy121_stage02/comparison.json)
passes scientific-summary and exact per-frame decision verification against
the 1.26.4 producer. It explicitly reports feature/analysis float byte parity
false and portable pointwise floating bounds unverified; the quoted pointwise
error comes from the separate full local audit. Producer-version strict replay
remains available through `replay_history_analysis.py` and its stored receipts.

The portable package includes the unchanged baseline/evaluation data and
9.97 MB of cached map/raw-observer inputs. It needs neither ROS/Gazebo nor
MuJoCo for arithmetic replay. Original model Jacobian regeneration requires
the separately hash-bound local logs/models. Large full history/frame tables
and duplicate independent outputs remain local; their hashes, per-array
bindings, explicit omission reasons and regeneration tools are retained.

[Evidence inventory](evidence/portable_manifest.json)
· [Analysis/replay scope](evidence/analysis/README.md)
· [Teammate guide](../../../TEAMMATE_HANDOFF.md).
