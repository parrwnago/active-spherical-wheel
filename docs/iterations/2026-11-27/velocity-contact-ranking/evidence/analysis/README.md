# I-60 saved-state contact-family ranking

All three I-58 native B trajectories were evaluated offline at every saved
timestamp. No controller, observer, reference, command or physics history was
changed. The search domain uses a normal inferred from the momentum observer,
so velocity ranking here is not an independent wrench-observer replacement.

During the preregistered moving window [1,41), 23,423 states have a unique
dominant truth family carrying at least 90% of at least 1 N total load.
This does not mean there is exactly one physical contact point. All truth
families are covered by the candidate bank in that denominator.

Native, normal-prior-only and normal-plus-wrench rankings each disagree with
truth at 412 states (1.759%). Adding measured-motion residuals reduces this to
109 (0.465%) for both velocity and fused raw ranking. The fused accepted subset
contains 6,902 states with 18 disagreements (0.261%); native has 306 disagreements
on exactly those same accepted states. Acceptance covers 86.60% of the 7,970
geometrically ambiguous, motion-eligible states, and 29.47% of all primary
truth-eligible states. All twelve registered shadow gates pass. These correlated
saved-state comparisons establish neither improved physical tracking nor a
new multi-contact decomposition.

## Chronology and source boundaries

- `campaign_tools/extract_features.py` was frozen before extraction. It loads
  only allowed measurement fields, uses the original historical mirror XML
  with private kinematics/Jacobians, and guards against physics/contact queries.
  Its three case outputs are in `campaign/features_stage01/`. The all-case
  seal predates every truth-array read.
- The original evaluator source was frozen in `pre_execution_stage01/`.
  Before truth access, `pre_execution_stage02/` adds precision-file hash
  verification and a timing-description qualification. Seven synthetic
  denominator/window/timing contracts pass; the earlier six are an overlapping
  subset, not additional tests.
- `stage01/` contains the immutable scientific report and full frame arrays.
  Truth is the last completed 1 ms physics step; the observer represents a
  filtered preceding 5 ms interval. Stable-truth and descriptive timing subsets
  do not reconstruct an unlogged computation delay.
- `replay_stage01/` exactly reproduces the original full evaluation. The later
  `evaluation_compact_stage01/` preserves full-rate truth inputs and the
  dtype/shape/byte hash of every original frame array while avoiding duplicate
  ranked-score histories. `replay_stage02/` reproduces all those arrays and the
  entire report/gates exactly from this compact package and the separate
  minima-preserving rank package. This is a mechanical projection after known
  outcomes, with no criterion changes.

## Standalone replay

Python 3 and NumPy suffice; MuJoCo and the historical raw files are unnecessary
for fixed-mode rank/evaluation replay:

```sh
python3 PATH/analysis/replay_evaluation.py \
  --rank-data-source PATH/portable_tools/rank_stage01 \
  --evaluation-data-source PATH/analysis/evaluation_compact_stage01 \
  --output NEW_REPLAY_DIRECTORY
```

The replay verifies compact input hashes, recomputes every rank/evaluation,
checks all regenerated frame-array bytes against their original bindings and
compares the full scientific report and all gates exactly. Regenerated
`CASE_evaluation.npz` files support rerendering the scientific figures.

The rank package preserves the union of all four modes' per-family winning
candidate indices in sixteen slots per frame. It reproduces the fixed ranking
costs, original candidate indices and acceptance controls; it does not regenerate
all 102 geometry candidates without the historical model. Full original feature
banks and repeated evaluation arrays can remain local, with their origin hashes
and source registrations retained in the publication inventory.

The remaining accepted errors, unknown states, goal-window behavior, transient
clock differences and exact-simulation-state assumptions must stay visible.
Passing the shadow screen supports considering a controlled integration test;
it does not authorize or establish a live controller improvement.
