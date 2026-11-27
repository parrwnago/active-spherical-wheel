# I-71 — remove current pair states, keep interval-transfer evidence

**Completed: stop this HMM version.** Removing current two-family states reduces
the temporal model's dominant-support errors from 564 to 425, but the matching
frame-only model has 303 errors. History worsens every starting phase. Four of
eight predeclared continuation checks fail, so this branch stops without tuning.
Seven of the original sixteen adoption gates also fail. The live LQR, momentum
observer, geometry support locator, feedforward and incoming-drive preparation
remain unchanged.

[Three-page explanation/results PDF](evidence/publication/plots_stage01/endpoint_pair_ablation_report.pdf)
· [Comparison image](evidence/publication/plots_stage01/overview.png)
· [Interval-versus-current-contact illustration](evidence/publication/plots_stage01/interval_vs_endpoint.png)
· [Standalone ChatGPT context](../../../CHATGPT_CONTROL_CONTEXT.md)

## Question and fixed conditions

The observer describes forces over the **previous 5 ms**, while the controller
needs the current contact. Evidence of shell A and shell B can arise from a
transfer inside that interval, or from both being loaded together. I-70 already
included directed transfer likelihoods, but its temporal model also retained
current pair states incorrectly against many sampled pressure labels.

This approved diagnostic removes current pair states while retaining those
interval-transfer likelihoods. It asks whether that restriction makes temporal
reasoning consistently useful; it does not claim that real multi-contact can
be ignored in the application.

The recordings are the same three I-58 **native Gazebo/DART Mars figure-eight**
episodes: 44 seconds, 5 ms control, 8,801 frames each, initial shaft phases −1.0,
+0.3 and +0.9 rad, and yaw 45°. The selected controller has gain-scheduled LQR,
momentum observer, rolling/holding feedforward and 120 ms incoming preparation;
assumed bounds are `[1.5,0.6,0.6] N m` and `[0.30,0.30] m/s` for the belts.
There is no new physics, driving video, controller change, geometry generation,
NNLS/force refit, parameter fitting or solver investigation.

Baseline commit: `dd4f74b1209cd27a8ef6233dfb50e4bdd39fca81`.
The [protocol](evidence/protocol/protocol.json) was registered before new
extraction/filter arithmetic. The
[inference source/input seal](evidence/protocol/inference_source_input_seal.json)
binds the scientific implementation. All three outputs were
[sealed before truth evaluation](evidence/inference/all_three_sealed.json).
The three starts were previously inspected; this remains retrospective evidence.

## Exact change

The output still uses I-70's 21-mode indices, with the twelve pair-state
probabilities fixed to **exactly zero**. Allowed current modes are the four
singleton families, rolling/slip each, and unknown: indices `0..7,20`.

The initial prior is I-70's prior conditioned on those allowed modes. Every
allowed transition keeps the original **diagonal holding probability exactly**:

```text
P'(i,i) = P(i,i)
P'(i,j) = (1 − P(i,i)) P(i,j) / sum_(allowed k != i) P(i,k),  j != i
```

Thus known/unknown dwell settings stay 50/20 ms, and relative odds among
allowed jump destinations remain fixed. Removing destinations necessarily
changes jump probabilities. Renormalizing the entire row instead would also
change its diagonal and effective dwell time; that is not what was tested.

The frame control uses the same conditioned initial prior and 20 ms evidence
tempering. The no-edge history control adds the reduced transition graph.
The **primary history model** also retains I-70's directed two-family interval
likelihood when moving between different singleton families. Unknown, self and
same-family transitions use the current node likelihood. Frame-to-primary
comparison therefore includes both temporal history and directed edge evidence;
the no-edge control helps separate those contributions.

All costs, validity masks and complete-fit candidate IDs are copied byte-for-byte
from I-70. Each reported physical point is the original complete fit's **current
family endpoint**, resolved through its candidate ID. No candidate is reranked,
refitted or averaged across bodies. The 2.83 MB inference packets contain only
time, frozen costs, validity, IDs and endpoint point lookups. They do not contain
pressure truth, old family decisions, terrain or trajectory references.

Noise scales, friction, point shortlist, rolling/slip penalties, unknown cost,
prior ratios, dwell settings, affinity coefficients, reset behavior and the
original sixteen adoption gates remain fixed. Evaluation uses fresh I-71
point arrays rather than reusing I-70's cached method points.

## Results

The primary domain remains 23,423 dominant-positive-pressure rows in `[1,41)` s.
Unknown counts as an assignment error. `N` is the actual recorded native
estimate; `F` and `A` are earlier I-60 offline raw and accepted/fallback
comparators. **A is not the live controller.**

| Method | Dominant-support errors | Wrong singleton | Unknown | Pair output |
| --- | ---: | ---: | ---: | ---: |
| Live native N | 412 | 412 | 0 | 0 |
| Earlier raw F | 109 | 109 | 0 | 0 |
| Earlier guarded/fallback A | 124 | 124 | 0 | 0 |
| I-70 full frame | 305 | 62 | 204 | 39 |
| I-70 full no-edge history | 646 | 147 | 241 | 258 |
| I-70 full primary history | 564 | 140 | 223 | 201 |
| **I-71 restricted frame** | **303** | **62** | **241** | **0** |
| I-71 restricted no-edge history | 492 | 166 | 326 | 0 |
| **I-71 restricted primary history** | **425** | **155** | **270** | **0** |

| Starting phase | Primary rows | A errors | Restricted frame | Restricted primary |
| --- | ---: | ---: | ---: | ---: |
| −1.0 rad | 7,946 | 14 | 56 | 83 |
| +0.3 rad | 7,505 | 64 | 110 | 160 |
| +0.9 rad | 7,972 | 46 | 137 | 182 |

The restriction helps the old temporal model by 139 dominant decisions,
especially at +0.3 rad, where errors fall 295→160. Yet primary history still
adds **122 errors** over its matching frame control and makes **301 more** than
A. Even without history, the new observation score remains worse than A.
Current pair states therefore contributed to the tradeoff, but cannot explain
the whole failure. This does not isolate a particular force, geometry, velocity
or noise assumption as the physical cause.

Directed transfer evidence remains useful within this model: primary history
has 67 fewer errors than no-edge history, 492→425. That partial improvement does
not overcome the regression against independent-frame decisions.

The full loaded domain retains **23,894 rows**, including **all 471 true
two-family pressure sets**. The restricted model cannot predict any of those
sets and fails all 471. Its loaded-set errors are **896 = 425 + 471**, versus
A's 595 and I-70 primary's 582. No two-contact sample is silently removed to
make the restriction look better. Set Brier is **0.063652**, worse than A's
0.049803. The frozen metric schema's `representable` and
`unsupported_truth_count` fields still refer to I-70's eleven-set evaluation
bank (three/four-family sets are outside it); the restricted model's additional
471 pair limitations are reported explicitly here and in the diagnostic report.

Other metrics must retain their comparison scope:

- All 71 defined stable pressure transitions are acquired; no misses. Primary
  absolute latency has median 0 ms and p95 40 ms, versus frame p95 15 ms and
  A p95 20 ms. Primary has ten reverted attempts and 85 false support changes,
  versus frame's seventeen and 156; A has thirty false changes.
- At confidence ≥0.8, primary nonunknown coverage is 97.78%, with 76 incorrect
  decisions, versus frame's fourteen at 92.89% coverage. Unknowns remain errors
  and do not contribute confident coverage. These beliefs are conditional
  engineering scores, not a calibrated sensor guarantee.
- On the **same 23,153 finite-singleton rows**, point RMS improves versus A
  **2.61048→2.32530 mm**, while maximum worsens **26.90764→29.45375 mm**.
  Point coverage is 98.8473%; A has no missing point on those rows. Wrong-family
  points remain included. These are different matched rows from I-70, so its
  RMS is not a directly interchangeable comparison.

The [complete machine report](evidence/evaluation/stage01/science.json) retains
all old adoption gates and all new continuation checks. The
[evaluation record](evidence/evaluation/README.md) gives exact definitions and
reproduction commands. The [previously problematic transfer window](evidence/publication/plots_stage01/fixed_transfer.png)
is retained as a fixed descriptive view; it did not select parameters.

## Decision and limits

Four of eight continuation checks fail: pooled error reduction against the
matching frame control and all three phase nonregression checks. Timing checks
pass. Seven of sixteen original adoption gates fail: pooled and per-phase
comparisons to A, loaded-set accuracy, set Brier and matched maximum point error.
Software completion is separate from those scientific failures.

**Stop this HMM version.** Keep the rejected candidates, sources and evidence;
do not run another dwell/weight/threshold sweep under this iteration. This is
not a finding that every HMM, particle filter or temporal estimator is unsuitable.
It does establish that removing endpoint pairs does not rescue this particular
model, and that its frame evidence already has a limitation before memory acts.
No other estimator is implemented or promoted as a result of this diagnostic.

Pressure truth is sampled positive normal load from the last 1 ms solve at each
5 ms snapshot. It is not the complete preceding-interval contact history, every
force-bearing body or first physical touch. The I-68/I-69 qualification remains
in force; no solver work was resumed. There is no new closed-loop tracking,
balancing, trajectory or hardware claim.

## Verification and portable replay

There are 27 new contracts across three disjoint files: input projection four,
restricted inference fourteen and evaluation projection/continuation nine.
Earlier per-component runs overlap those 27 and are not added again.
The original 21 I-70 metric contracts are reused history, not new test cases.
Four separate independent-verifier contracts pass, giving 31 new focused
contracts in total. Its initial three-contract stage overlaps the corrected
four-contract stage and is not added again. Independent probability-domain
checks are preserved under `evidence/review/independent_recurrence`; they import
none of the production filter helpers. All three case decisions, physical
points, IDs and edge indices match exactly; probability differences are at most
6.66e−16 and prefix-101 checks agree. The first verifier attempt failed while
formatting boolean-array differences. Its source/log/seal remain archived;
the corrected stage changes comparison plumbing only, with an added contract.
No inference parameter or recurrence changed. This is a focused set of checks,
not the full repository suite.

The portable replay uses NumPy only and calls the exact frozen `infer_packet`
consumer in fresh isolated processes. From this folder's `evidence/` directory:

```bash
python publication/portable_tools/replay_inference.py \
  --package . --output /tmp/i71_new_inference

python evaluation/prepare_evaluation.py \
  --baseline-root evaluation/packets_stage01 \
  --prediction-root /tmp/i71_new_inference \
  --inference-seal /tmp/i71_new_inference/receipt.json \
  --output /tmp/i71_new_evaluation_packets

python evaluation/evaluate_ablation.py \
  --protocol protocol/protocol.json \
  --packet-root /tmp/i71_new_evaluation_packets \
  --prediction-root /tmp/i71_new_inference \
  --parent-science reference_i70/science.json \
  --output /tmp/i71_new_evaluation
```

Use new output directories. The existing I-71 evaluation packets supply only
the unchanged truth/N/F/A fields to the projector; every new-method point comes
from the fresh predictions. An exact copy of the registered I-70 parent summary
is included with its copy provenance; inference itself reads neither labels
nor that summary. The original repository-bound `run_case.py` CLI verifies original
repo-relative seals; use the portable wrapper for relocated evidence.
All 81 canonical prediction arrays reproduce byte-exactly across the three
isolated workers. Replay receipts and source/import bindings are retained in
`evidence/publication/replay_qa`. Python 3.10 and producer NumPy 1.26.4 were used;
cross-version identity is not claimed. No Gazebo, MuJoCo, ROS or SciPy is needed
for this frozen-cost inference comparison.

The [export manifest](evidence/export_manifest.json) binds the copied evidence
and records bulky duplicate/cache omissions. Publication preflight initially
resolved an inference seal's paths relative to its manifest folder instead of
the iteration root and stopped before copying. Both the failed helper/plan/log
and the corrected resolver/export are preserved under `export_attempt_stage01`
and `export_attempt_stage02`. This packaging correction changed no scientific
source, parameter or output.
