# I-63 independent review

The preregistered combined-risk test fails its necessary-condition screen. Rejecting the one accepted point above the pooled native maximum forces loss of 99 useful family corrections within the class `tau = a + b R ||F|| + c risk`, with shared nonnegative coefficients. There are only six harmful override opportunities. Thus the conservative family-error lower bound is `124 + 99 - 6 = 217`, exceeding the unvetoed I-60 hybrid A's 124 errors. This is a limitation of this frozen score and rule class on already inspected episodes, not a claim that every adaptive threshold or contact estimator is impossible.

The combined-risk forced counts for full data and training without -1.0/+0.3/+0.9 rad are respectively 99, 0, 31, and 72. Every count survives the entire preregistered absolute +/-1e-4 risk perturbation envelope. The minimum full-data robust risk-margin surplus is 0.0157050869533 N m. The zero bound for the subset excluding -1.0 rad is inconclusive, not a successful fit; the full-data stop condition controls continuation.

The registered stopping rule was followed: no 18-setting adaptive schedule fit and no new episode were run. Live LQR, momentum observer, feedforward and incoming-drive preparation remain unchanged. This audit uses the old I-58 Gazebo/DART episodes and new truth-blind private-kinematics features; it performs zero new physics steps.

## Independent evidence

- `independent_condition_audit.py` SHA ff226cfabd5597d83b96c90b466194f04168c104093c1ae0aa398cf4c47e669f was frozen before new feature arithmetic. It reconstructs primary risk with independent scalar dot products/backward differences and checks dominance using exact Fraction products of original radius and force values. It does not import producer helpers.
- `results_stage01/report.json` and three independent condition packets retain raw reconstructed values, numerical comparison details, all exact witnesses and robust-envelope counts. All validity masks and innovation-risk arrays are byte exact. Maximum combined-risk difference is 2.70339306496e-13, within the quantity-specific allowances registered before arithmetic. No values were replaced or normalized to achieve parity.
- `canonical_comparison.json` confirms exact primary bound counts, maxima and witness identity sets for all four groups, exact identity matching of the old 99 witnesses, all diagnostic group counts/event clusters, and exact fixed distribution statistics recomputed from frozen producer arrays. This last descriptor check is distinct from independent raw feature reconstruction.
- Eligible frames: 23,423; accepted proposals: 6,902; useful corrections: 294; harmful overrides: 6; accepted-wrong frames: 18. The six harmful frames form six contiguous predicate clusters under the registered 5ms rule, which does not establish six independent physical events.

The unsafe 35.205s frame has combined risk 0.9376007, dominated by seam risk rather than innovation risk 0.5193425. The bad proposal therefore also looks transition-risky. The same nonnegative risk allowance that is intended to protect good corrections does not isolate this harmful point. Its point error is 29.842858743mm versus the pooled native maximum 26.907642232mm. These are evaluated equivalent-contact distances, not live truth inputs.

Innovation-only and seam-only supplementary bounds were also fixed before arithmetic; they force 59 and 99 useful losses respectively. They are supplemental diagnostics and did not replace the frozen combined-risk primary test. No adaptive policy was selected from these results.

## Design and source chronology

The unexecuted early design advice is retained in `design_stage01`. Before any feature arithmetic, axial seam distance was replaced with signed spherical arclength to avoid polar coordinate compression; 12mm now denotes a physical surface distance. Per-seam signed backward rates use only the preceding adjacent 5ms sample. Unknown values remain unknown. The earlier optional risk-only cutoff was not selected.

Pre-arithmetic receipts preserve the independent source before clarification of exact Fraction radius-times-force products, then before registration of quantity-specific numerical allowances and robust risk-envelope checks. The final frozen source was not changed after release. Producer source corrections preserving policy force inputs and pooled99 identities were reviewed before data evaluation.

The reviewer observed the feature owner's 12 inclusive synthetic contracts and analyzer owner's 15 selected contracts; these are source-contract checks, not a full repository suite or physical trials. The reviewer compiled the independent audit and ran the two evidence commands below.

## Reproduction

Run from repository root into a new output directory (the existing directory is immutable evidence):

```bash
.venv/bin/python results/contact_condition_diagnostic_20261127/review/independent_condition_audit.py \
  --features-root results/contact_condition_diagnostic_20261127/features/stage01 \
  --compact-root results/contact_condition_diagnostic_20261127/portable_inputs/i62_base \
  --output /tmp/i63_independent_new
```

The comparison adapter reads the recorded independent report and frozen canonical stage01 without altering them:

```bash
.venv/bin/python results/contact_condition_diagnostic_20261127/review/compare_canonical.py \
  --root results/contact_condition_diagnostic_20261127
```

The comparison output is `canonical_comparison.json`; preserve the original output before rerunning that adapter. NumPy-only risk reconstruction uses cached kinematic axes. Regenerating those axes requires the original model and is not claimed by this replay. All episodes were previously inspected; no unseen validation or causal improvement in physical tracking is established.
