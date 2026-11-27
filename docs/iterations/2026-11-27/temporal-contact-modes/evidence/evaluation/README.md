# I-70 evaluation evidence

This folder evaluates the three saved I-58 native runs.  It does not change the
controller, generate motion, fit parameters, or run new physics.  All three new
inference outputs were sealed before evaluation truth was opened.

The frozen scientific evaluator is `evaluate_temporal.py`; the evaluation-only
projection is `prepare_evaluation.py`.  `schema.json` defines their API.
`metric_prespecification.md` and the parent protocol fix the metrics and sixteen
acceptance gates before new outcomes.  `pre_truth_execution_stage01` binds the
sources and all seventeen input artifacts before the truth read.

The primary domain reproduces all 23,423 historical dominant-positive-pressure
rows.  The baseline error counts also reproduce exactly:

| Method | Singleton errors | Unknown | Pair outputs |
|---|---:|---:|---:|
| Native N | 412 | 0 | 0 |
| I-60 raw fused F | 109 | 0 | 0 |
| I-60 accepted/fallback A | 124 | 0 | 0 |
| New frame evidence | 305 | 204 | 39 |
| No-edge HMM control | 646 | 241 | 258 |
| Primary transition-aware HMM | 564 | 223 | 201 |

The experiment completes but fails six of sixteen gates: pooled singleton error,
all three phase error comparisons, high-confidence wrong count and maximum point
distance.  This version is not accepted for live use.

There are partial improvements on other metrics.  On the full 23,894 loaded
rows, set errors fall from A's 595 to 582; set Brier falls from 0.0498033 to
0.0378962.  On 22,999 exactly matched finite-singleton rows, HMM point RMS is
2.22954 mm versus A's 2.54550 mm.  Its maximum worsens to 29.45375 mm versus
26.90764 mm.  Point coverage is 98.1898%; there are no HMM-scorable/A-missing
rows.  Wrong-family singleton point errors are retained in these comparisons.

All 71 defined pressure transitions are acquired.  HMM absolute latency median
is 5 ms and p95 is 40 ms.  It has ten reverted attempts and 97 false support
changes versus the frame control's seventeen and 158.  This is not a general
baseline timing improvement: A has thirty false changes and 20 ms p95 latency.
The frame-to-HMM reduction is mostly fewer unknown transitions (127 to 52);
singleton toggles increase from eighteen to twenty-seven.

Positive-pressure truth is the sampled final 1 ms of each 5 ms recording interval.
It does not identify every force-bearing body or first physical touch.  Unknown
and pair outputs count as errors against dominant-singleton labels.  Beliefs
remain conditional on the engineering likelihood assumptions; Brier/ECE assess
their calibration empirically rather than proving it.  No tracking or causal
controller improvement follows from this offline comparison.

`synthetic_stage01` preserves the original seventeen-contract evaluator draft.
`synthetic_stage02` preserves the final twenty-one-contract version, inclusive
of those earlier contracts; the counts must not be added.  Contracts cover
collapse-before-MAP, loaded-domain expressivity errors, unknown coverage, Brier,
event timing/reversion, false-switch breakdowns, point matching and same-family
accepted point updates.  They are a selected scientific subset, not the full
repository suite.

`source_boundary_review.json` records the pre-inference causal input/kernel
audit.  `runner_review_stage01` preserves an unfinished wrapper guard; stage02
records its correction before launch and the reviewed source hash.

`independent/check_recurrence.py` was written and sealed before outcome arrays
were read.  It reconstructs the filter using probability-domain arithmetic,
without importing the inference implementation or re-fitting forces.  Its
`stage01/report.json` confirms all collapsed support decisions and selected
predecessor/current modes exactly.  HMM probability differences are at most
5.56e-16; frame-control differences at most 3.38e-13 are within the prespecified
1e-12 independent-arithmetic check.  The 101-frame prefix is unchanged by later
frames.  These are numerical/causality checks, not additional scientific runs.

Reproduction, from the repository root after inference is sealed:

```bash
.venv/bin/python results/temporal_contact_modes_20261127/evaluation/prepare_evaluation.py \
  --original-root results/balance_priority_20261126/campaign/runs \
  --baseline-root results/velocity_contact_rank_20261127/analysis/stage01 \
  --point-root results/velocity_contact_rank_20261127/point_check/stage01 \
  --prediction-root results/temporal_contact_modes_20261127/inference/stage01 \
  --inference-seal results/temporal_contact_modes_20261127/inference/all_three_sealed.json \
  --output /tmp/i70_evaluation_packets

.venv/bin/python results/temporal_contact_modes_20261127/evaluation/evaluate_temporal.py \
  --protocol results/temporal_contact_modes_20261127/protocol/protocol.json \
  --packet-root /tmp/i70_evaluation_packets \
  --prediction-root results/temporal_contact_modes_20261127/inference/stage01 \
  --output /tmp/i70_evaluation
```

Use new output directories; canonical stage01 files are immutable.
