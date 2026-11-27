# I-65 independent verification

The primary result is independently reproduced: shared bias/covariance calibration supplies a partial ranking improvement, but fails the registered discrimination and shadow-policy requirements. No fresh episode or integration is justified by this result.

The corrected verifier imports neither the I-65 scoring module nor its evaluator. It reconstructs the shared frozen I-64 input history, independently selects teacher rows, computes equal-episode means and Gram covariance about the global mean, regularizes the covariance, and scores by eigendecomposition rather than Cholesky. It independently computes ambiguity/ties/preferences, native fallback, family errors, point statistics and gates.

`corrected_stage02/output/comparison.json` passes 395 discrete/numeric comparisons. These are verification comparisons, not 395 controller tests or independent robot trials. All calibration row masks/counts, held-out ambiguity/validity/preferences/keep masks, pooled ranking counts and shadow gates match exactly. Maximum numerical differences are:

- Shared mean/covariance: 7.63e-17.
- Normalized residuals: zero.
- Eigen versus Cholesky score: 2.27e-12.
- Training weighted p95 score: 3.55e-14.
- Point SSE: 1.16e-10 mm²; point-distance summaries differ only at rounding scale.

The six fits use only their two training episodes. Each episode has half the calibration weight, with population covariance about the combined mean; there is no all-data deployment fit. Teacher branch counts per episode are 2,291 (2,289 F + 2 native), 2,151 (all F), and 2,448 (2,444 F + 4 native). These episode rows repeat across model/fold fits and must not be added into a count of independent observations. Correct-family labels are supervised simulation labels, not truth-free inference inputs.

## Independently reproduced outcomes

| M1 raw-interval scorer | Correct useful / 294 | Correct harmful / 6 | Shadow label errors |
| --- | ---: | ---: | ---: |
| Max absolute component | 244 | 1 | 173 |
| Euclidean squared norm | 244 | 2 | 172 |
| Shared bias/covariance | 253 | 2 | 163 |

The shared model removes two of A's six harmful changes while losing 41 useful corrections: `124 + 41 - 2 = 163` errors. Four harmful changes remain; worst point discrepancy remains 29.8429 mm. A's 124 errors remain better. M2 shared covariance gives 245 useful/2 harmful correct and 171 shadow errors; it remains a control, not a replacement winner.

The fair norm comparison uses I-65's relative-ranking veto for every scorer. I-64's absolute-tolerance M1 veto had a different decision rule; its 132 errors are not the I-65 maxabs reference. Shared calibration also changes both bias subtraction and covariance weighting, so this experiment does not isolate off-diagonal correlations as the cause of the gain.

## Ambiguity and limitations

The post-result `corrected_stage02/ambiguity_structure.json` checks the 6,602 accepted pattern-ambiguous rows. Every one has the same F/native physical family, original candidate index and H matrix; its normalized residual difference is exactly zero. No distinct-family accepted pair is pattern-ambiguous. These are duplicate alternatives, not evidence that 6,602 different physical contacts are fundamentally unobservable. The 12 accepted-wrong cases where neither alternative family is correct are among these duplicates (seven in +0.3 rad, five in +0.9 rad).

All 294 useful and six harmful different-family pairs are resolved, with no ties or unknowns. The failed discrimination therefore cannot be attributed to the new ambiguity convention. Full three-joint scoring improves some decisions, but does not satisfy the required useful/harmful tradeoff or repair the worst point.

No held-out labels enter calibration or score calculation; held-out truth only evaluates the output. All three histories and prior errors were already inspected, so this remains retrospective internal assessment. Family correctness does not establish exact point accuracy, single-contact validity or a calibrated probability.

## Preserved verification correction

The first independent verifier (`independent_review.py`) stopped because it expected `normalized_weights` in the compact fitted JSON. The primary intentionally omits that redundant vector. Its source, receipt, execution log and partial registration remain unchanged. The corrected source under `corrected_stage02/` compares published means/covariances/p95 and exact teacher masks/counts while deriving equal-episode weights independently. It changes no fit, score or acceptance rule. Its source receipt predates the rerun.
