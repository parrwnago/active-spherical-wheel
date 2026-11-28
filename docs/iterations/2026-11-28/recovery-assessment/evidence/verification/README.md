# I-83 independent verification

`independent.py` reconstructs the fixed last-quarter projection, spatial
quintic-speed reference, attitude/spin signals, five predicates and full recovery
ledgers without importing canonical scoring functions. Fourteen final synthetic
contracts pass in `contracts_stage02/`; these include the initial ten and are
not additive across retained development stages.

The completed audit in `completed_stage01/` checks 60 signal arrays and all ten
Boolean predicate arrays across both 26,601-row recordings. Every predicate is
sample-exact. Maximum signal difference is 2.996e-14; all 1,317 numeric ledger
leaves and every nested status, censor, count, null, entry, confirmation, later
exit and terminal-tail field agree. These are overlapping checks of two existing
recordings, not new physical trials or a full repository test suite.

The 0.60 case has earlier first entry and one-second return, but its first
two-second path-motion interval starts at 117.675 s versus 116.955 s for 0.30.
It then remains in band through 133 s; the baseline leaves again. Neither
achieves strict motion recovery with the spin bound. These posthoc descriptive
metrics do not change the earlier I-82 acceptance gates.

Reproduce into a new empty output directory:

```sh
.venv/bin/python results/recovery_assessment_20261128/verification/verify_recovery.py \
  --root-registration results/recovery_assessment_20261128/protocol/frozen_stage01/protocol.json \
  --canonical results/recovery_assessment_20261128/evaluation/completed_stage01 \
  --output results/recovery_assessment_20261128/verification/new_stage
```

The wrapper registers sources, definitions, raw input and canonical output
hashes before opening arrays, and checks unchanged inputs after calculation.
