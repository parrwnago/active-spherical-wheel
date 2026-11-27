# I-71 stopping-ablation evaluation

The restricted temporal estimator **fails its continuation decision**.  The
registered result is `STOP_THIS_HMM_VERSION`; no controller or estimator default
is changed, and there is no parameter tuning or new physics in this iteration.

All three inference outputs were sealed before truth projection/evaluation.
`pre_truth_execution_stage01/source_input_seal.json` binds five source files and
nine inputs before the first new truth/metric calculation.  `stage01/science.json`
is the immutable canonical result, SHA256
`206a8c558db7086359e157b714c3e62ccf54fb0e2c6ff7529d820da3d2d5fad3`.

The exact frozen I-70 evaluator is copied under `reference_i70/`, with source
SHA256 `33993c7fc892d71a4530666090af957cb77614543b03fe3b4316bfdc8b351008`.
Its metrics and sixteen adoption gates are unchanged.  The wrapper adds only
the separately prespecified continuation decision and historical comparisons.
Nine new projection/continuation contracts pass; twenty-one frozen I-70 metric
contracts are reused reference evidence, not new I-71 tests or a full repo suite.

## Results

The fixed primary domain is again 23,423 dominant-positive-pressure rows across
the three episodes.  Frozen N/F/A error counts reproduce exactly.

| Method | I-70 errors | I-71 errors | I-71 unknown errors |
|---|---:|---:|---:|
| Native geometry N | 412 | 412 | 0 |
| I-60 raw fused F | 109 | 109 | 0 |
| I-60 accepted/fallback A | 124 | 124 | 0 |
| Matching frame evidence | 305 | 303 | 241 |
| No-edge HMM control | 646 | 492 | 326 |
| Primary transition-edge HMM | 564 | 425 | 270 |

N is the original estimator comparison.  F and A are frozen offline I-60
comparisons; neither is silently promoted by this experiment.

Removing endpoint pairs reduces primary HMM errors by 139 relative to I-70.
Most of that change occurs at +0.3 rad, where errors fall from 295 to 160.  But
the restricted HMM still has 122 more errors than its matching restricted frame
control, with regression in every phase:

| Start phase | Primary rows | A errors | Restricted frame errors | Restricted edge-HMM errors | Edge-HMM unknowns |
|---|---:|---:|---:|---:|---:|
| -1.0 rad | 7,946 | 14 | 56 | 83 | 45 |
| +0.3 rad | 7,505 | 64 | 110 | 160 | 90 |
| +0.9 rad | 7,972 | 46 | 137 | 182 | 135 |

The continuation decision fails four of eight checks: strict pooled improvement
over matching frame evidence and all three phase nonregression checks.  Its four
timing checks pass.  The original adoption decision independently fails seven of
sixteen gates: pooled and all phase errors, loaded-set accuracy, set Brier and
maximum point distance.

All 471 mixed-pressure truth rows remain in the 23,894-row loaded-set domain.
Restricted inference has zero probability for endpoint pair states, so all 471
are errors, giving 896 loaded-set errors (425 + 471) versus A's 595.  The frozen
evaluator still recognizes pair categories for calibration; these rows are not
removed or relabeled as evaluation-unsupported.  Set Brier is 0.0636521 versus
A's 0.0498033.  HMM confidence-at-least-0.8 coverage is 97.7800%, with 76 covered
errors; unknown rows remain errors and do not provide confident coverage.

On the exact same 23,153 finite-singleton timestamps for current HMM and A,
point RMS is 2.32530 versus 2.61048 mm, but maximum distance worsens to 29.45375
versus 26.90764 mm.  Point coverage is 98.8473%; A is available on every matched
row.  Wrong-family singletons are included.  I-70 used a different matched subset,
so its RMS cannot be compared directly as if the timestamps were identical.

All 71 defined pressure transitions are matched, with zero misses.  Primary HMM
absolute latency median is 0 ms and p95 is 40 ms, versus frame p95 15 ms.  It has
ten reverted attempts and 85 false support changes versus frame's seventeen and
156.  This is not a general baseline timing improvement: A has thirty false
changes and 20 ms p95.  Restricted HMM singleton toggles are 33 versus frame's
eighteen; fewer unknown transitions (52 versus 138) explain the total-switch
reduction.

## Interpretation and limits

Endpoint pair persistence contributed to I-70's poor dominant-singleton result,
but it was not the whole problem.  Removing pairs barely changes memory-free
errors (305 to 303), and the restricted history still regresses against its own
frame evidence.  The combined observation model is also worse than A; this
ablation does not isolate force fitting from geometry or velocity terms.

Removing pair endpoints also changes allowed jump destinations and conditional
prior odds while preserving registered dwell probabilities.  The comparison is
not an isolated dwell-time experiment.  Two-family transition-interval evidence
remains distinct from simultaneous endpoint pair states.

Truth remains the final 1 ms sampled positive-pressure support every 5 ms, not
complete force routing, first physical touch, continuous contact truth or
slipping measurements.  All episodes were previously inspected.  This is a
retrospective stopping diagnostic, not an unseen validation or tracking result.
Partial point/timing gains do not override the failed continuation decision.

## Reproduction

After all three restricted inference outputs are sealed, from the repository root:

```bash
.venv/bin/python results/endpoint_pair_ablation_20261127/evaluation/prepare_evaluation.py \
  --baseline-root docs/iterations/2026-11-27/temporal-contact-modes/evidence/evaluation/packets_stage01 \
  --prediction-root results/endpoint_pair_ablation_20261127/inference/stage01 \
  --inference-seal results/endpoint_pair_ablation_20261127/inference/all_three_sealed.json \
  --output /tmp/i71_evaluation_packets

.venv/bin/python results/endpoint_pair_ablation_20261127/evaluation/evaluate_ablation.py \
  --protocol results/endpoint_pair_ablation_20261127/protocol/protocol.json \
  --packet-root /tmp/i71_evaluation_packets \
  --prediction-root results/endpoint_pair_ablation_20261127/inference/stage01 \
  --parent-science docs/iterations/2026-11-27/temporal-contact-modes/evidence/evaluation/stage01/science.json \
  --output /tmp/i71_evaluation
```

Use fresh output directories.  Canonical stage01 outputs are immutable.
