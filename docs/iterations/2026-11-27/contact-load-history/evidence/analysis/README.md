# I-64 matched-history diagnostic

Matching the candidate prediction to the observer's filtering window improves the load model, but does not qualify a better contact decision or veto. This experiment uses existing native-controller logs only. No threshold is tuned, no candidate is reranked, no controller is changed, and no new driving episode runs.

## Results

The primary model-consistency question passes its registered gates. On the same 6,884 accepted proposals with the correct current family, vector residual RMS decreases from **0.058283 to 0.037207 N m** (about 36.2%). The vector 95th percentile decreases from 0.135488 to 0.071414 N m. All three starting phases improve their vector RMS. Coverage is 100% of primary and accepted states, including all 18 critical accepted-wrong frames.

The current-contact discrimination question fails. Useful corrections correctly prefer the proposal over the physical native alternative in **212/294 → 243/294** cases, but harmful overrides correctly prefer native in **2/6 → 1/6** cases. There are no ties, unknowns, or near ties in those groups. The result neither preserves harmful-decision accuracy nor reaches the registered 90% useful/50% harmful criteria.

The unchanged-tolerance shadow veto also fails the old I-62 performance gates:

| Offline policy | Label errors | Harmful changes remaining | Useful corrections retained / 294 | Maximum point error (mm) |
| --- | ---: | ---: | ---: | ---: |
| A: no additional veto | 124 | 6 | 294 | 29.843 |
| M0: original instantaneous/I-61 check | 153 | 5 | 264 | 29.843 |
| M1: raw interval check, diagnostic control | 132 | 3 | 283 | 26.908 |
| M2: matched A-history check, primary | 128 | 6 | 290 | 29.843 |
| M3: matched native-history sensitivity | 128 | 6 | 290 | 29.843 |

M2 avoids most of the original check's lost useful corrections, but prevents none of A's six harmful changes. Compared with A, its four lost useful corrections account for the four extra errors. M1/M3 are controls, not methods selected after seeing results. None is adopted.

## What the history means

Current endpoint geometry approximates the preceding 5 ms interval. The prediction uses the observer's raw interval force once, then the same 100 rad/s EWMA as the observed joint loads. This is causal retrodiction available when that interval ends; it is not future-contact prediction.

M2 uses one common predicted prior, commits the frozen A path independently of all new diagnostics, and scores the current proposal/native alternative from that same prior. M3 repeats this with native-equivalent committed history as a sensitivity control. An observed seed interval is not model agreement; each seed requires another 200 ms of contiguous valid history before scoring.

The matched residual contains an inherited term and a current-interval term. On current-family-correct accepted frames, the median inherited norm share is about 60.8%; 1,081/6,884 have oppositely aligned inherited/current terms. Thus a smaller residual is not necessarily better evidence of the current contact. The decomposition identity is checked numerically. An observed-prior one-step control is separately verified to equal a scaled raw residual; its smaller size is algebraic, not evidence of improved inference.

The result supports correcting the time-window inconsistency in model diagnostics. It does not establish a unique physical cause of prior errors, solve multiple contact, demonstrate better driving, or justify live integration. Dominant truth support permits weaker contributors and records a sampled final-substep condition, not the full 5 ms contact/force history.

## Evidence and verification

`stage01/science.json` contains Q1/Q2 gates, per-phase/group comparisons, fixed shadow-policy reports, history decomposition, control benchmarks and event definitions. Full-rate frame arrays and event traces remain local. The original I-62 99 useful-witness identities are retained; group denominators are unchanged. M0 reproduces I-61 flags/families/distances exactly, while map-times-force vectors, full observer recurrence, one-step scaling and matched-history decomposition satisfy registered numeric guards.

The analyzer's **14 synthetic contracts** passed before historical arithmetic, inclusive of its earlier 12-contract development stage. This is a scoped analysis suite, not the repository controller suite. The older source/test hashes and correction provenance remain under `pre_execution_stage01/`.

The strict producer replay under NumPy 1.26.4 reproduces **228 feature arrays and 594 analysis arrays** exactly. Bindings replace duplicate reference NPZ containers in the compact archive; freshly derived outputs are not substituted with stored values. The scientific source is unchanged.

The feature-only system NumPy 1.21.5 strict check reports 84 byte-inexact float arrays. A separate local pointwise comparison against the full canonical data measures at most 1.11e-16 N m and about 1.256e-12 in auxiliary norm shares; masks and nonfinite patterns match. Strict failure/source receipts are retained. A byte-inexact array's pointwise distance cannot be inferred from hashes alone.

The separate v2 verifier checks exact integer/bool/nonfinite patterns, **36 per-frame preference/compatibility/shadow-decision arrays**, and unit-aware scientific summaries. It explicitly reports raw float byte parity and `pointwise_float_bounds_verified_from_archive=false`; it cites the separate local pointwise audit without claiming that audit was repeated from compact hashes. V2 producer verification passes. Root's archive-isolated system-v2 result is a separate receipt.

## Standalone compact replay

From the standalone I-64 evidence-package root:

```sh
python3 analysis/replay_history_analysis_v2.py \
  --analysis-source analysis \
  --protocol protocol/protocol.json \
  --base-root portable_inputs/i62_base \
  --feature-package features/portable_stage01 \
  --bindings analysis/portable_stage02/bindings.json \
  --local-feature-audit features/cross_version_diagnostic_stage01/comparison.json \
  --old-witnesses portable_inputs/i62_old_witnesses.json \
  --reference analysis/stage01 \
  --output /tmp/i64-replay-v2
```

The output path must not already exist. Only NumPy and Python's standard library are needed; cached kinematic maps are replayed, not regenerated with MuJoCo or Gazebo. No live control or physics is invoked.
