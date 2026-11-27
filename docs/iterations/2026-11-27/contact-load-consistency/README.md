# I-61 — check contact proposals against joint-load residuals

2026-11-27. **The fixed joint-load veto is rejected.** It prevents one harmful
override but loses 30 useful corrections. No live controller, observer,
feedforward, gain, reference, limit or default is changed. No new physics runs.

[Comparison PDF](evidence/analysis/plots_stage01/load_veto_comparison.pdf)
· [Overview PNG](evidence/analysis/plots_stage01/overview.png)
· [Canonical report](evidence/analysis/stage01/science.json)
· [Previous I-60 result](../velocity-contact-ranking/README.md).

## Question and one frozen change

I-60 showed that measured velocity improves contact-family ranking, but a
wrong unloaded hemisphere can fit rolling motion better than the real loaded
belt. Could the three additional joint-load residuals reject such an override?

Use the same three native I-58 Gazebo/DART recordings: Mars figure-eight,
44 s, yaw 45°, initial shaft -1/+0.3/+0.9 rad, caps
[1.5,.6,.6 N m,.3,.3 m/s]. All 26,403 timestamps are retained. I-60's candidate
bank, geometry, scores, raw labels and heuristic acceptance are frozen.
There is no threshold fitting, rescoring, force optimization, point cap or
temporal smoothing. These histories were inspected in earlier iterations;
this is a controlled replay comparison, not a held-out robustness evaluation.

The new opt-in [contact_load_consistency.py](../../../../src/spherical_wheel/contact_load_consistency.py)
predicts a candidate's generalized load `J_point(q).T * F_observed` using private
kinematics. Free-base angular rows are rotated into world coordinates before
comparison with the saved observer vector. The last three rows remain joint
torques. The veto requires each extra joint mismatch to satisfy:

```text
abs(observed_joint_load - predicted_joint_load)
    <= 0.03 N m + 0.10 * R * norm(observed_force)
```

These are fixed engineering tolerances, not calibrated noise probabilities.
The same observed total force is assigned to one candidate. This does not
resolve general multicontact forces. First-three force rows reproduce that
force by construction; base moment repeats lever consistency, and the shaft
row largely projects net moment. The main added identity information comes
from the two hemisphere-joint rows, which can remain ambiguous near torque-null
configurations.

[Protocol](evidence/protocol/protocol.json),
[input/source seal](evidence/protocol/input_source_seal.json) and
[final extraction-source binding](evidence/protocol/pre_execution_source_seal.json)
precede historical residual arithmetic. All three feature/policy outputs are
sealed before truth evaluation. Inference uses only recorded state, I-60
proposals/native estimates and the observer output; true contacts and terrain
are evaluator-only.

## Compare the correct policies

| Policy | Family and point supplied by this offline policy |
| --- | --- |
| N | Original native estimate |
| A | I-60 accepted proposal; otherwise N |
| C | Same accepted proposal only if the new load check passes; otherwise N |

Accepted point changes are included even if the family label stays the same.
No point is interpolated into an invalid surface. A is the relevant guarded
I-60 comparator: its error count is **124**, rather than the **109** for all
raw I-60 rankings. Unaccepted raw proposals are not automatically applied.

Primary scoring is the same half-open [1,41) interval and 23,423
dominant-support eligible states: >=1 N total load and >=90% one family.
This permits weaker support elsewhere; it is not a strict single-contact
claim. Every eligible state counts, including native fallback and rejected
proposals. The last-1-ms truth and filtered preceding-5-ms observer convention
are retained; this is not a measurement of computational latency.

## Outcome: the new check rejects too many useful proposals

| Initial shaft phase | N label errors | A label errors | C label errors |
| --- | ---: | ---: | ---: |
| -1.0 rad | 51 | 14 | 16 |
| +0.3 rad | 226 | 64 | 84 |
| +0.9 rad | 135 | 46 | 53 |
| Pooled | 412 | 124 | 153 |

The primary error rates are 1.759%, 0.529% and 0.653%, respectively. C remains
better than native N as a shadow estimate, but worsens the guarded I-60 policy
at every start. It is not accepted as an improvement.

Of 6,902 A proposals, the new check vetoes 46 and retains 6,856:

- Harmful overrides of a **correct native label** decrease only **6→5**:
  16.7% removed, below the fixed 50% requirement.
- Useful corrections of a **wrong native label** decrease **294→264**:
  89.796% retained, below the fixed 90% requirement.
- The other 15 vetoes are label-neutral point fallbacks. Losing 30 correct
  corrections while preventing one new error yields 29 additional errors.

The six harmful overrides are a subset of the 18 I-60 accepted errors:
the other 12 accepted errors occur where the native estimate was already
wrong. These denominators must not be interchanged.

| Point discrepancy to logged equivalent-point reference, same primary rows | N | A | C |
| --- | ---: | ---: | ---: |
| RMS | 3.134 mm | 2.757 mm | 2.807 mm |
| 95th percentile | 7.336 mm | 5.797 mm | 5.902 mm |
| Maximum | 26.91 mm | 29.84 mm | 29.84 mm |

The new veto does not reduce the largest guarded-policy point error. The
earlier 44.04 mm raw I-60 extreme was already excluded by I-60 acceptance;
it must not be presented as a benefit of this new check.

**Five of 13 registered gates fail:** harmful-override reduction, useful
retention, pooled C-versus-A error, +0.3-rad C-versus-A error, and maximum
point discrepancy versus N. The conditional controlled handoff simulation
does not run. No post-result tuning or default promotion follows.

## What the adverse events establish

At -1 rad, **35.205 s**, the true/native dominant support is belt B but A
selects shell B. Its extra-joint residuals are approximately
`[0.173, -0.000002, -0.105] N m`, below the 0.251 N m component tolerance.
C therefore keeps the wrong shell and the 29.84 mm point discrepancy.
The logged-point reference assigned to the true belt also passes compatibility.
Both hypotheses are plausible at this declared resolution.

At **7.875 s**, A correctly changes the native shell-B label to shell A.
Residuals approximately `[0.212, -0.301, 0.272] N m` exceed a 0.266 N m
tolerance, so C discards that useful correction. Simply tightening this same
gate cannot reject the illustrated bad hypothesis while retaining this good one.

Post-primary diagnosis, without changing the frozen gate, finds:

- **All 30 lost useful corrections are outside the prespecified ±10 ms stable
  truth subset.** On stable primary states, N has 75 errors and A/C each have
  one; the new check adds no label benefit there.
- All 30 lost corrections also fail the compatibility check when the evaluator
  inserts the **logged dominant-family point** instead of the candidate point.
- Shaft mismatch contributes to **40 of the 46 vetoes**. Shell A/B each
  contribute to six; these row counts overlap.

The current-point/single-total-force assumption can disagree with a
history-filtered residual during transfer. Current geometry, signal filtering,
force distribution and sampled point conventions remain possible contributors.
These results do not isolate a unique physical cause or prove the momentum
observer itself is inaccurate.

The logged-point diagnostic passes 99.870%, 99.881% and 99.352% of **stable**
eligible states, satisfying the three model-validity gates. That does not
validate the transition states where the veto loses useful corrections. It
uses the same **observed** total force and a load-weighted point, not independent
ground-truth generalized forces. Weak additional contacts remain possible;
the recorded point-count subset counts only contacts above 0.05 N.

[Independent post-primary diagnosis](evidence/review/post_primary_stage01/report.json)
and the canonical fixed-event payloads retain both helpful and adverse cases.

## Decision

Keep native LQR, momentum observer, rolling/holding feedforward and existing
120 ms incoming-drive preparation selected. Retain I-60 as a promising offline
velocity-based family estimate; **do not add this instantaneous load veto to
the controller**. This rejects one fixed model/test, not all uses of the extra
observer channels.

The next useful diagnostic is a time-consistent load prediction or explicit
transition/force-distribution hypothesis, evaluated before another hard gate.
A distance cap alone cannot correct actuator identity. No new compensation,
contact forecast or physical tracking improvement is claimed in I-61.

## Verification, chronology and reproduction

45 selected core tests pass: **17 new plus 28 existing**. Nine separate
synthetic evaluator contracts pass. These are separate invocations, not a new
full repository suite. Independent lever/hinge-axis calculations reproduce all
historical predictions within 6.7e-16 load units, and all gates/policies and
paired point statistics agree independently. The compact decoder/gate replay
reproduces all 59 packet fields byte-for-byte; geometric Jacobian predictions
are explicitly stored inputs to that replay, not regenerated without the model.

The archive retains the unexecuted extractor schema correction, a failed
synthetic fixture caused by shared boolean-array mutation and its correction,
and the original reports. One final reviewer receipt was written 16 seconds
after the canonical truth phase began, while the reviewer still had not seen
outcomes; its original pre-truth wording is corrected by an append-only
[chronology clarification](evidence/review/design_stage01/CHRONOLOGY_CLARIFICATION.md).
Core/extractor/evaluator source, thresholds and independent check scripts were
already frozen. Do not claim the pipeline waited for that final receipt.

Reproduce gate outputs, N/A/C policies, oracle compatibility and evaluation
from the compact full-rate archive with Python 3 and NumPy:

```bash
python3 docs/iterations/2026-11-27/contact-load-consistency/evidence/analysis/replay_evaluation.py \
  --gate-data-source docs/iterations/2026-11-27/contact-load-consistency/evidence/portable_tools/gate_stage01 \
  --evaluation-data-source docs/iterations/2026-11-27/contact-load-consistency/evidence/analysis/evaluation_compact_stage01 \
  --output /tmp/i61-evaluation-replay
```

All state arrays, primary/fixed-window metrics and 13 gates match exactly in
the tested producer NumPy 1.26.4 and portable NumPy 1.21.5 environments. The
whole report is exact with the producer version. With NumPy 1.21.5, five scalar
event-distance norms differ by at most **3.56e-15 mm**; the replay explicitly
reports those paths and permits only that event-norm round-off within 1e-12 mm.
It does not replace values or loosen any scientific gate. The initial strict
cross-version comparison failure and both replay receipts are preserved.

[Published replay receipt](evidence/publication/published_evaluation_replay/replay_comparison.json)
· [Independent evaluation audit](evidence/review/evaluation_stage01/report.json)
· [Archive/source/link checks](evidence/publication/publication_qa_stage01.json).

Full state/model assets, large feature packets and duplicate regenerated arrays
remain local and hash-bound. Compact replay reproduces stored-feature
arithmetic, not a new physical experiment. The
[portable inventory](evidence/portable_manifest.json) identifies intentional
omissions and source origins. Selected-contact load projection follows the
standard generalized-force relation also used in contact localization research;
this is not an implementation of a full particle filter.
[Manuelli and Tedrake, IROS 2016](https://groups.csail.mit.edu/robotics-center/public_papers/Manuelli16.pdf).
