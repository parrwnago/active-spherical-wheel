# I-64 independent review

Matching the model prediction to the observer's filtering window improves joint-load consistency, but does not qualify a better contact discriminator or a live estimator change. The preregistered model-consistency question Q1 passes; current-branch discrimination Q2 fails. All fixed-threshold shadow policies fail the unchanged acceptance gates. No threshold tuning, new physics, controller change or estimator adoption occurs.

## Independent results

On the same 6,884 accepted family-correct proposal frames, instantaneous M0 versus matched-history M2 has vector RMS 0.0582828177652 to 0.0372066594929 N m (about36.2% lower) and vector P95 0.135488428579 to 0.0714135296869 N m. Every phase's vector RMS decreases. All 23,423 primary states, all6,902 accepted proposals, six harmful overrides,18 accepted-wrong frames and the one pooled unsafe-point frame are scorable. No warmup/unknown exclusion explains the improvement.

Q2 separately asks which physical current alternative is favored. M0 correctly prefers F on212/294 useful corrections and native-family Nalt on2/6 harmful overrides. M2 improves the former to243/294, but reduces the latter to1/6. That fails the frozen90% useful criterion, at least3/6 harmful criterion and harmful nonregression. There are no ties, unknowns or near-ties in these two groups; the outcome is not a numerical tie artifact. M1 and M3 remain predeclared controls and cannot replace the primary model after inspection.

| Fixed shadow policy | Family errors | Harmful retained | Useful retained | Point maximum mm | All gates |
|---|---:|---:|---:|---:|---|
| A, no extra veto |124|6|294|29.8429|baseline|
| M0, old I-61 control |153|5|264|29.8429|fail|
| M1, raw interval |132|3|283|26.9076|fail|
| M2, matched A history |128|6|290|29.8429|fail|
| M3, native-history sensitivity |128|6|290|29.8429|fail|

At the fixed -1-rad35.205s event, current F is the wrong shell family and Nalt has the correct belt family. M0 F/Nalt scores are0.68792/0.91330; M1 raw scores1.24485/1.46475; M2 scores0.56821/0.65474. All rank the wrong F branch ahead of Nalt. M1 rejects F only because both residuals exceed the fixed bound. M2's inherited-term norm share is about0.11035 there; that share is a ratio of vector norms, not a variance or causal attribution fraction. These observations do not establish one physical cause of the misassignment or of trajectory drift.

## What was checked independently

`independent_history_audit.py` SHA77407aec7f860ba6de54ff2e67f5c964b94cf10bf2836ecc6baaf4602025049e was compiled and sealed before new I-64 historical arithmetic. It uses chronological scalar loops, per-row map products and the alternate lambda*prior+alpha*input recurrence; it does not import the producer. The source was unchanged after release.

`results_stage01/report.json` retains independent raw outputs and compares them with the feature producer. All validity/warmup/reset masks are byte exact. The largest non-ratio numerical difference is7.10543e-15; the largest auxiliary norm-share difference is6.49986e-12 with no finite-mask mismatches. The predeclared ratio-only caveat was not needed to mask a meaningful discrepancy. All original-value differences are retained; none were replaced to force parity.

`canonical_comparison.json` independently verifies exact per-frame current-branch preferences, rank/group counts, Q1/Q2 decisions, critical-event coverage, and all fixed-shadow count/point metrics and13 acceptance flags. The maximum normalized score difference is8.88178e-16. Full9 observer recurrence, observed-prior scaling and matched-history decomposition identity errors are at most7.10543e-15. The old M0 shadow reproduces I-61 flags exactly.

The feature owner reports19 inclusive synthetic contracts; the evaluator reports14 separate contracts. These are scientific-tool tests, not a full repository or controller suite. This reviewer compiled the independent source and executed the independent reconstruction and thin canonical comparison; no new physics was run.

## Timing, causality and interpretation

Only raw interval force enters each predicted interval load. The independently propagated accumulator uses the same observer cutoff100rad/s, current endpoint geometry and the frozen inferred A path; native-equivalent history is a separate sensitivity control. Measured filtered loads seed a history once, followed by200ms/20-time-constant warmup. Invalid committed maps, low filtered force, invalid observations or clock gaps break the chain; missing current alternatives do not rewrite or reset an otherwise valid committed history.

A current counterfactual uses the committed prior plus its current increment. It never assumes today's contact existed throughout the past. The observed-prior one-step control is only alpha times the raw residual; its smaller magnitude is an algebraic scaling. Filtering the raw residual also smooths it by construction. Q1 therefore compares matched-history M2 against instantaneous filtered-force M0 on identical frames, rather than calling raw-to-filtered smoothing an independent success.

Endpoint H multiplied by raw interval-average force is still a5ms approximation, available only at interval end. Contact geometry can migrate, and within-interval force/geometry covariance is not reconstructed. Actual contact-force history is not recovered. N_eq deliberately uses an unprojected virtual equivalent point assigned to a native family:21606/23423 primary N_eq points pass the physical material test, while all23423 assigned-body maps are finite. F and Nalt physical maps are materialvalid on all primary states. The A history is an offline frozen shadow path on native-controller motion, not the historically actuated routing.

All episodes and previous failures were already inspected. Saved last-substep truth labels evaluate current and past50ms subsets only; they do not supply an exact5ms contact/force oracle. No oracle history is used here. Better load consistency is established for this model comparison, while better current contact inference, physical tracking and live adoption are not.

## Reproduction

Run from repository root, choosing a new output directory to preserve recorded evidence:

```bash
.venv/bin/python results/contact_load_history_20261127/review/independent_history_audit.py \
  --features-root results/contact_load_history_20261127/features/stage01 \
  --base-root results/contact_load_history_20261127/portable_inputs/i62_base \
  --output /tmp/i64_independent_new
```

The thin comparison reads the original sealed independent result and canonical report:

```bash
.venv/bin/python results/contact_load_history_20261127/review/compare_canonical.py \
  --root results/contact_load_history_20261127
```

Its output is `canonical_comparison.json`; preserve the original before rerunning it. Cached H maps permit NumPy-only arithmetic replay. Regenerating endpoint Jacobians requires the original model and is not claimed by this numerical replay.
