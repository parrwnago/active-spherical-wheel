# I-65 — compare the full joint-load error pattern

2026-11-27. **Implemented and evaluated offline; rejected for integration.**
Shared covariance scoring distinguishes the frozen contact alternatives slightly
better than the largest-error rule, but discards too many useful corrections.
The live LQR, momentum observer, geometry locator, feedforward and incoming-drive
preparation remain unchanged.

[Comparison PDF](evidence/publication/plots_stage02/covariance_comparison.pdf)
· [Overview PNG](evidence/publication/plots_stage02/overview.png)
· [Annotated events](evidence/publication/plots_stage02/event_windows.pdf)
· [Canonical numerical report](evidence/analysis/stage01/science.json)
· [Frozen protocol](evidence/protocol/protocol.json)
· [Previous history diagnostic](../contact-load-history/README.md).

## Question and actual change

The earlier check reduced three joint-load errors to their largest absolute
component and compared it with a force-scaled allowance. I-63 added one
observable-condition allowance; I-64 corrected the compared filtering history.
Those experiments did not test this new calculation.

Here each contact candidate predicts **all three joint loads** from the same
observer force and its frozen point Jacobian. Its error is the observed minus
predicted joint-load vector. Fit a shared empirical mean and covariance from
correct-family training examples; compare each candidate's complete error
pattern with that distribution. Errors along a normally variable direction
count less than errors along a normally consistent direction. Correlations
between joints also matter. Both candidates use exactly the same fitted model.

This is a different scoring calculation, not another coefficient sweep of the
old scalar cutoff. It does introduce fixed regularization assumptions, which
are declared before fitting rather than optimized against these outcomes.

## Frozen experiment

Reuse the original I-58 Gazebo/DART Mars figure-eight recordings: initial common
shaft -1.0/+0.3/+0.9 rad, yaw 45 degrees, 44 seconds, 5 ms, 8,801 states each.
Native gain-scheduled LQR, momentum observer, rolling/holding feedforward and
120 ms incoming-drive preparation generated the motion. The five command caps
are `[1.5,0.6,0.6 N m,0.3,0.3 m/s]`. No new trajectory, physics episode,
contact forecast, camera or actuation change is run.

The unchanged primary domain contains 23,423 states in `[1,41)` with at least
1 N truth normal load and a unique family carrying at least 90%. Weaker
additional contacts can remain. Keep the same finite point domain, including
all native fallbacks, for point metrics.

For each held-out starting phase, fit using only the other two whole episodes.
At each accepted primary training timestamp, take the physically valid F
proposal if its family matches training truth; otherwise take the valid
physical native-family candidate Nalt if its family matches. Use at most one
example per timestamp. Never choose a point using true-point distance or the
smallest residual. Each training episode contributes half the total weight.

These labels are **supervised simulator calibration**, not sensor measurements.
Held-out family truth enters evaluation only. All episodes were examined in
earlier iterations: these folds are retrospective internal comparisons, not
unseen validation. A correct family does not guarantee an exact point or a
correct model of the preceding 5 ms load distribution.

With residual `r` in N m and the unchanged candidate-independent scale:

```text
scale = 0.03 + 0.10 R ||filtered_observer_force||
e = r / scale
mu = sum(w e)
S = sum(w (e-mu)(e-mu)^T)
C = 0.8 S + 0.2 trace(S)/3 I + 0.05^2 I
score = (e-mu)^T C^-1 (e-mu)
```

Covariance is computed around one pooled mean, including between-episode bias.
The 0.05 floor and 0.2 shrinkage are fixed engineering regularization, not
measured sensor noise. Cholesky computes scores. No candidate receives its own
variance or allowance. These are consistency distances, not probabilities;
force and joint loads originate from the same momentum observer.

**Primary M1** uses the raw preceding-interval observer load and endpoint
geometry held over that interval. **Control M2** uses I-64's matched filter with
a frozen A-history prior and 200 ms warmup; it gets its own shared training-only
calibration. M2 cannot replace a failed M1 after seeing results. Maximum-absolute
and uncalibrated squared-Euclidean scores are fixed descriptive controls.

Missing scores remain unknown. If normalized candidate error vectors differ by
at most 0.01, classify them as indistinguishable. Otherwise apply a relative
`1e-9` score tie tolerance. These are declared ambiguity conventions, not
statistical confidence. All unknowns/abstentions stay in evaluation denominators.

For the offline shadow estimator, begin with I-60's accepted/fallback estimate
**A**. Veto an accepted proposal only when Nalt is clearly preferred; fall back
to the **original native equivalent point N**, not the Nalt physical point.
Ties, indistinguishable pairs and unknown comparisons retain A. An invalid F
map explicitly falls back to N. No choice changes past history or actuation.

## Results

All 6,902 accepted primary proposals have finite paired scores, including every
harmful and accepted-wrong frame and the worst point. In 6,602 cases the two
alternatives are exact duplicates: same family, candidate index and H matrix,
with zero residual difference. They are deliberately left unchanged, rather
than described as different contacts lacking observability. This includes 12
accepted-wrong rows where neither member of this pair has the correct family.
All 294 useful and six harmful changes are scorable and distinguishable.

| Offline policy | Contact-family errors | Harmful changes remaining / 6 | Useful corrections kept / 294 | Point RMS (mm) | Maximum (mm) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Native N | 412 | 0 | 0 | 3.13375 | 26.90764 |
| I-60 accepted/fallback A | **124** | 6 | **294** | **2.75693** | 29.84286 |
| M1 relative maximum-absolute | 173 | 5 | 244 | 2.77283 | 29.84286 |
| M1 relative Euclidean | 172 | 4 | 244 | 2.77192 | 29.84286 |
| **M1 shared covariance** | **163** | **4** | **253** | **2.76395** | **29.84286** |
| M2 shared covariance, control only | 171 | 4 | 245 | 2.76289 | 29.84286 |

The relative maximum-absolute policy above is a new paired-ranking veto; it is
**not** the absolute compatibility veto in I-61/I-64. Matching scores does not
make those different policies interchangeable.

M1 covariance correctly prefers F on 253/294 useful frames (86.1%), versus
244/294 for its maximum-absolute control. It correctly prefers Nalt on 2/6
harmful frames, versus 1/6. Thus it improves both relative-control counts, but
fails the registered minimum of 90% useful decisions and half the harmful
decisions. It rejects **41 useful corrections while preventing two harmful
changes**: `124 + 41 - 2 = 163` errors. This is a net regression of 39 frames
against A, not a controller or tracking improvement.

| Held-out initial shaft | Useful corrections kept | Harmful changes prevented | Family errors, A → M1 covariance |
| --- | ---: | ---: | ---: |
| -1.0 rad | 38 / 39 | 1 / 2 | 14 → 14 |
| +0.3 rad | 125 / 162 | no harmful examples | 64 → 101 |
| +0.9 rad | 90 / 93 | 1 / 4 | 46 → 48 |

The +0.3 phase contributes 37 of the 41 lost useful corrections. It contains
zero harmful examples, so harmful accuracy there is unscorable. There are only
six harmful single-frame events across these previously inspected episodes;
thousands of temporally correlated states do not establish broad robustness.

The primary method also fails the unchanged I-62 shadow gates for harmful
removal, useful retention, pooled errors, +0.3 phase error increase, point RMS
and maximum point error. Point p95 improves slightly, but does not qualify the
method. M2 fails too; no control is promoted after outcomes.

## Two illustrative events

- **Useful correction, -1.0 rad at 7.875 s:** F has score 9.18 and Nalt 314.69.
  The correct proposed shell family is retained. I-61's fixed load veto rejected
  this useful correction.
- **Harmful correction, -1.0 rad at 35.205 s:** wrong F has score 132.65 and
  correct-family Nalt 137.81. The wrong contact is still preferred and the
  29.84 mm point outlier remains. A smaller consistency distance does not prove
  the contact identity. Both absolute scores are reported, without turning a
  post-result cutoff into a new acceptance rule.

Both scores at 35.205 s exceed that fold's training weighted p95 of 9.56.
Across phases, 407 accepted M1 comparisons have both candidates above their
training p95. This describes poor model fit, not calibrated out-of-distribution
probabilities; no post-result rejection cutoff is introduced.

The event appendix also includes every harmful frame, the first useful frame
and worst A point at each phase, and each phase's first newly lost useful
correction. Plot windows include future recorded samples for context; the
scores themselves use no future observations.
There are 14 unique event anchors, presented on eight pages including the
explanation page. The one-page overview is also available as a PNG.

## Interpretation and decision

The joint-error pattern contains some additional discrimination relative to
largest-error scoring, but the improvement is insufficient to protect I-60's
useful velocity-based corrections. Keep this negative result; do not integrate
the veto, fit a more complex conditional model, or run a new drive merely to
seek a favorable example.

The new score changes bias subtraction and covariance weighting together.
This experiment does not isolate joint correlations as the cause of its modest
relative gain.

The next proposed investigation is the **contact/load model at these same
failures**: does the correct-family candidate have the wrong point, or does
assigning the entire interval force to one point fail during load transfer?
Covariance weighting cannot repair a missing candidate or an inadequate
multi-contact model. These results do not determine which cause is responsible,
nor do they prove that every alternative momentum-observer method would fail.

## Verification and reproduction

The protocol, tested sources and six input packets were sealed before fitting
(13 explicit files). The archived inputs and NumPy-only history helper are
dependencies of the already tracked I-64 package, avoiding duplicate recordings.
The original I-60 A identities and I-61 gate choices reproduce exactly; old
endpoint residuals differ by at most `1.11e-16 N m`.

Fourteen method contracts and ten separate evaluator contracts pass. They
cover weighting, correlated patterns, regularization, finite inputs, branch
swaps, calibration deduplication, label-free held-out scoring, fallback and
full evaluation denominators. This is a focused offline-method check, not the
full controller suite. An initial test launcher used one parent directory too
many and exited 127 before Python ran; its logs and corrected launch are kept.

Producer NumPy 1.26.4 replay reproduces all **129 output arrays byte-for-byte**.
System NumPy 1.21.5 produces different floating bytes, but the same discrete
decisions and scientific results within the frozen comparison tolerances.
Comparisons retain canonical values; raw-byte equality is not claimed across
versions. An independent weighted-Gram/eigen implementation agrees on every
teacher selection, classification, fallback decision and scientific gate.
Mean/covariance differences are at most `7.63e-17`, score differences
`2.27e-12`, and point-SSE differences `1.16e-10 mm²`. Its initial verifier
expected an unpublished diagnostic weight array; that schema failure and the
corrected review are preserved without changing the primary scientific source.
An archive-only replay launched from `/tmp` with system NumPy also passes,
using the published I-65 files and tracked I-64 dependencies. Both pointwise
arrays and scientific summaries are checked, without replacing canonical values.
Publication QA verifies hashes, links, array bindings and PDF page counts.
The initial figure layout and its corrected render remain separately archived.

From the repository root, use a fresh output directory:

```bash
python3 docs/iterations/2026-11-27/contact-covariance-ranking/evidence/analysis/evaluate.py \
  --protocol docs/iterations/2026-11-27/contact-covariance-ranking/evidence/protocol/protocol.json \
  --previous-package docs/iterations/2026-11-27/contact-load-history/evidence \
  --source-root docs/iterations/2026-11-27/contact-covariance-ranking/evidence \
  --output /tmp/i65-replay

python3 docs/iterations/2026-11-27/contact-covariance-ranking/evidence/review/verify_replay.py \
  --reference docs/iterations/2026-11-27/contact-covariance-ranking/evidence/analysis/stage01 \
  --actual /tmp/i65-replay --output /tmp/i65-replay-check
```

No MuJoCo, Gazebo, ROS, renderer, map download or live simulator is required for
this cached-map replay. NumPy is required. Original Jacobian generation still
belongs to I-64 and is not repeated by this experiment.
