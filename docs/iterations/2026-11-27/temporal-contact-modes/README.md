# I-70 — temporal hybrid contact estimator

**Implemented and tested offline; not accepted for live use.** The first
prototype models rolling, slipping, two-family contact and uncertainty across
time. It improves some point and multi-contact metrics, but worsens dominant
contact assignments in every saved starting phase. Six of sixteen predeclared
acceptance gates fail. The live LQR, momentum observer, support locator,
feedforward and incoming-drive preparation remain unchanged.

Start with the [four-page explanation and results PDF](evidence/plots_stage02/temporal_contact_report.pdf)
or [comparison image](evidence/plots_stage02/overview.png).
The [standalone ChatGPT context](../../../CHATGPT_CONTROL_CONTEXT.md) explains
the whole selected control pipeline through this iteration.

## Question and experiment

Could keeping several plausible contact explanations across time avoid the
mistakes of independent point selection and scalar veto thresholds? This tests
a particular engineering implementation of that idea, not a reproduced paper
or a claim that temporal estimators generally perform poorly.

The inputs are the three existing I-58 **native-controller** Mars figure-eight
recordings, with initial common-shaft phases −1.0, +0.3 and +0.9 rad and 45° yaw.
Each contains 44 seconds and 8,801 control-state samples, at 5 ms spacing.
There are **26,403 inference frames**. The primary comparison uses the same
23,423 historically defined dominant-positive-pressure rows in `[1,41)` seconds.
All three episodes had already been inspected; these are retrospective tests,
not unseen validation. There is no new physics, motion video, threshold sweep,
controller tuning or solver investigation in this iteration.

Code, inputs and parameters were frozen before inference. All three outputs
were sealed before evaluation opened simulator contact labels. The
[protocol](evidence/protocol/protocol.json),
[source/input seal](evidence/protocol/inference_source_input_seal.json) and
[all-three-output seal](evidence/inference/all_three_sealed.json) preserve that
order. Later diagnostic event selection is explicitly marked posthoc.

## What was implemented

The new opt-in modules are
[shadow_contact_models.py](../../../../src/spherical_wheel/shadow_contact_models.py)
and [temporal_contact_filter.py](../../../../src/spherical_wheel/temporal_contact_filter.py).
Neither is imported by the live control loop. They need NumPy and SciPy,
already project dependencies.

The model has **21 modes**: four singleton support families and six pairs of
different families, each with rolling/slipping explanations, plus unknown.
The families are shell A, shell B, belt A and belt B. Roll/slip beliefs collapse
into eleven support-set probabilities before choosing a contact set. A point
estimate always belongs to a physical family; points on different bodies are
not averaged into an imaginary contact.

The model uses exact recorded simulated pose/velocity, previous actually applied
commands, known geometry and the momentum observer's **raw nine-component load
over the preceding 5 ms**. It compares measured body/material motion with
candidate contact geometry. Desired velocity is not substituted for measured
motion. No contact labels, terrain, trajectory reference or future sample enters
inference. The force-map angular coordinates are aligned with the observer's
midpoint frame; geometry remains an endpoint approximation.

For each support explanation, one or two point forces fit the observed
generalized load:

```text
minimize  0.5 ||diag(1/sigma) (r_raw − sum J_i^T F_i)||²
subject to inward normal force >= 0 and an inscribed friction pyramid.
For a pair, each family must carry at least 10% of fitted normal force.
```

Each family has up to two shortlisted points: its geometry-only best and its
geometry/rolling-velocity best, with identical points deduplicated. Complete
coupled fits are combined by a negative-log-mean-exponential score, including
normal and tangential material-velocity penalties. Continuous forces are
optimized, not integrated out. Consequently these are **approximate profile
potentials**, not exact marginal likelihoods. Point covariance describes the
shortlisted alternatives conditional on a mode; it does not cover all possible
geometry or model errors.

The causal filter updates beliefs using past beliefs and current evidence:

```text
b_k(j) ∝ sum_i b_(k−1)(i) T_ij(dt) exp(−w C_ij(k))
w = min(1, dt / 0.020 s)
```

For ordinary transitions, `C_ij` is the current mode cost. For a transition into
a singleton from another family or a two-family set, a directed edge fit can
explain the interval load on both exiting and incoming bodies while testing
rolling velocity only at the current endpoint. The output keeps exiting-body
interval forces separate from current contact points. This addresses the
difference between an interval measurement and current support; it does not
forecast terrain or determine exact contact onset.

Fixed engineering assumptions are force σ=0.5 N, moment/joint σ=0.03 N m,
rolling/slip velocity σ=0.02/0.15 m/s, normal σ=10°, nominal friction=0.7,
known/unknown dwell=50/20 ms, evidence-correlation time=20 ms, and unknown cost=9.
Initial singleton/pair/unknown prior masses are 0.8/0.1/0.1. Rolling receives
90% of each known-set prior. The transition matrix is a documented one-jump
holding-time approximation, not an exact continuous-time Markov exponential.
No parameter is fitted or selected from the new outcomes.

General three-family support and multiple contact points on one family are not
represented. The paired force model has no extra force-effort or explicit
pair-complexity penalty beyond its priors/constraints. Offline runtime was not
validated for a 5 ms live control deadline.

## Results and comparisons

`N` is the actual recorded native estimate. `F` and `A` are earlier I-60 offline
comparators: raw fused ranking, and accepted fused ranking with native fallback.
**A is not the live controller.** The frame-only and no-edge history methods are
fixed ablations of this new prototype. The transition-aware history model was
declared primary before execution; it remains primary after the failed result.

| Method | Dominant-family errors | Wrong singleton | Unknown | Pair on singleton truth |
| --- | ---: | ---: | ---: | ---: |
| Live native N | 412 | 412 | 0 | 0 |
| Raw fused F | 109 | 109 | 0 | 0 |
| Guarded/fallback A | 124 | 124 | 0 | 0 |
| New frame evidence | 305 | 62 | 204 | 39 |
| History without edge evidence | 646 | 147 | 241 | 258 |
| **Primary history + edge evidence** | **564** | **140** | **223** | **201** |

Unknown and pairs count as errors against dominant-singleton labels; they are
not quietly removed. Thus 564 does not mean 564 wrong singleton points. It
still includes 140 wrong singletons, exceeding A's 124, and poor assignments
would not be safe simply because a model labels them confidently.

| Starting phase | Primary rows | A errors | Frame errors | Primary errors |
| --- | ---: | ---: | ---: | ---: |
| −1.0 rad | 7,946 | 14 | 56 | 85 |
| +0.3 rad | 7,505 | 64 | 112 | 295 |
| +0.9 rad | 7,972 | 46 | 137 | 184 |

There are partial improvements:

- On all 23,894 loaded rows, exact support-set errors fall **595→582** versus A,
  a gain of only 13 samples, or 0.0544 percentage points in set accuracy.
- Those loaded rows comprise 23,423 singleton sets and 471 two-family sets at
  the registered pressure thresholds. The primary model correctly labels
  **453/471** pair rows; A has no pair output and labels none correctly.
- Set Brier improves **0.049803→0.037896** against A's deterministic one-hot
  predictions. This does not establish calibrated confidence. At confidence
  ≥0.8 the model covers 96.74% of dominant rows but makes **150** confident
  nonunknown mistakes, exceeding A's total 124 errors.
- On the **same 22,999 finite-singleton rows**, point RMS improves
  **2.54550→2.22954 mm**, while maximum worsens **26.90764→29.45375 mm**.
  Finite-singleton coverage is 98.19%; wrong-family points remain included.
  A has no missing points on those rows. Do not compare this maximum with A's
  different, full-domain maximum and claim a gain.
- All 71 defined stable pressure transitions are acquired. Absolute latency is
  median 5 ms and p95 40 ms, versus A's p95 20 ms. False support changes fall
  158→97 versus the new frame model, but A has only 30. Most of that reduction
  comes from fewer unknown transitions; singleton toggles increase 18→27.

The six failed gates are pooled dominant-family error, all three phase error
comparisons, confident wrong count, and matched maximum point distance.
The [complete machine report](evidence/evaluation/stage01/science.json) retains
all sixteen gates, coverage, risk curves, calibration, timing and point metrics.
The [evaluation explanation](evidence/evaluation/README.md) defines denominators
and timing precisely. Passing software checks is distinct from passing these
scientific adoption criteria.

## What the failure teaches us

The [separate posthoc diagnostic](evidence/evaluation/posthoc_temporal_errors/README.md)
compares paired rows without changing parameters or registered gates. Relative
to A, the primary model fixes **26** dominant-singleton decisions and loses
**466**. Its pair recognition almost cancels those losses when measuring all
loaded sets, explaining the tiny net set-accuracy gain. It retains 207/294
original useful velocity corrections and repairs none of the six original
harmful overrides. This version does not solve the earlier failure case.

The +0.3 rad episode accounts for **197/201** false pair outputs and all 97
confident false pairs. The
[longest constant false-pair spell](evidence/plots_stage02/longest_pair.png)
lasts nine samples, 45 ms of sampled disagreement, at 15.725–15.765 s.
Frame evidence already selects shell B at both ends; history retains shell A+B.
This is evidence of an undesirable temporal decision tradeoff at that event,
not proof that a single dwell parameter caused all failures. The truth itself
switches quickly in the surrounding window.

Edge evidence improves the no-edge history control by 86 gained and four lost
singleton decisions, a net 82, while losing five correct pair rows. History is
therefore not uniformly harmful, and interval-transition representation helps
this particular filter. Nevertheless both history variants underperform the
frame evidence and earlier guarded estimate overall.

At +0.9 rad, 31.830 s, the highest-confidence nonunknown mistake is belt B
instead of sampled shell B. Frame confidence is already wrong at 0.875; history
raises it to 0.994. Repeated evidence and model assumptions can reinforce a
wrong explanation. More confidence is not automatically more information.
The [largest point-error window](evidence/plots_stage02/worst_point.png)
also remains visible at −1.0 rad, 35.200 s.

Pressure labels use sampled positive normal load at the final 1 ms physics
step; observer loads cover the previous 5 ms. They do not identify every
force-bearing body or first physical touch. I-68/I-69 already document that
qualification. These metrics compare consistent historical labels; they do not
establish physical slip, computation lag, or the cause of trajectory drift.
No solver investigation was resumed, and no closed-loop gain is claimed.

**Decision:** retain the prototype as opt-in research code and keep the selected
live baseline. Temporal mode reasoning is feasible to implement, but this
likelihood/prior combination is not a validated estimator improvement. If work
resumes, a small independently evaluated study of evidence calibration and
single-versus-pair discrimination would be more defensible than tuning a whole
controller around this failed model. That is a proposal, not a completed or
already authorized next experiment.

## Verification and reproduction

Sixty-nine contracts pass in seven disjoint selected test files: filter 12,
force models 18, inference consumer 5, input features 6, evaluator 21,
independent force-check contracts 3, and posthoc diagnostics 4. This is a focused
subset, not the full repository suite. Earlier inclusive stages are retained
but not added again to that count. The
[combined receipt](evidence/review/integration_stage01/receipt.json) binds sources
and test output.

Independent checks include a probability-domain recurrence reconstruction with
identical support decisions and selected transition modes on all three runs,
and **14/14** Cartesian-force SLSQP fits at six fixed case/time frames. Their
maximum objective gap is 3.57e−13 and force-constraint violation 2.22e−16 N.
These are numerical checks with limited scope, not additional physical trials.

The compact input package contains 10.52 MB of measured-state primitives.
From this folder's `evidence/` directory, full inference can run without Gazebo,
MuJoCo, ROS, contact labels or the original large rollouts:

```bash
python portable_tools/replay_inference.py \
  --inputs portable_inputs --output /tmp/i70_new_inference

python evaluation/evaluate_temporal.py \
  --protocol protocol/protocol.json \
  --packet-root evaluation/packets_stage01 \
  --prediction-root /tmp/i70_new_inference \
  --output /tmp/i70_new_evaluation
```

Use new output directories. Inference requires Python 3.10+, NumPy and SciPy;
rendering also needs Matplotlib. The producer used Python 3.10, NumPy 1.26.4 and
SciPy 1.15.3. The isolated inference replay reproduces **108 prediction/evidence
arrays** and **18 uncached feature arrays** exactly across all three episodes,
with imports from copied packet sources rather than the live checkout. Compact
[replay receipts](evidence/replay_qa/receipt.json) preserve that check. A different
NumPy/SciPy version can change floating point/NNLS behavior; cross-version full
inference identity is not claimed.

The evaluation packets include the frozen method point arrays as well as
labels and old comparators. They are valid for this byte-exact replay. If a new
variant changes points, regenerate that part of the evaluation packet rather
than reusing this packet to score its point accuracy.

The [export manifest](evidence/export_manifest.json) binds copied files and
lists bulky/duplicate omissions. Original rollouts and the 210 MB feature cache
remain local; compact primitives regenerate the cache. Historical source/input
seals retain original locations even when the portable paths are relocated.
Initial pytest plugin setup failure, unfinished runner guard, primitive scalar
shape correction, inclusive test stages and the first figure layout remain
separate evidence. Those setup corrections preceded relevant execution or
changed packaging/layout only; no scientific parameters changed after outcomes.
