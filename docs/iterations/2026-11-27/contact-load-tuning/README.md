# I-62 — tune the shared joint-load tolerance

2026-11-27. **No shared tolerance passes the registered criteria.** The best
label count improves only 124→123; the worst point discrepancy stays 29.84 mm.
This completes the user's proposed tuning experiment. No live controller,
observer, feedforward, reference, actuator limit or default is changed. No new
driving simulations are run.

[Tradeoff PDF](evidence/analysis/plots_stage02/tolerance_tradeoffs.pdf)
· [Overview PNG](evidence/analysis/plots_stage02/overview.png)
· [Canonical report](evidence/analysis/stage02/science.json)
· [Previous fixed-veto experiment](../contact-load-consistency/README.md).

## Question and fixed comparison

I-60 uses measured body/joint motion to rank physically valid contact
hypotheses. Its guarded policy A uses an accepted proposal and otherwise keeps
the native estimate N. I-61 added a joint-load check to those same proposals.
That fixed check prevented one harmful label change but discarded 30 useful
corrections. The tolerances were engineering choices, so testing a bounded
range is reasonable before rejecting this model entirely.

The new gate keeps a proposal only when every extra joint residual satisfies:

```text
max(abs(observed_joint_load - predicted_joint_load))
    <= a + b * radius * norm(observed_force)
```

Both parameters range from 0 to 0.30 in steps of 0.005: **3,721 pairs**. The
same pair applies to all three joint rows and all three starting configurations.
The original `(0.03 N m, 0.10)` setting and an explicit no-additional-veto A
baseline are controls. A zero threshold is checked directly, without division;
the runtime helper's positive-absolute-tolerance API is unchanged.

Keep I-60 candidate points, labels, acceptance and I-61 residuals fixed. There
is no rescoring, temporal filter, per-joint tuning, force fit or interpolation.
Inference uses saved simulated state and observer outputs; solver truth is used
only to select/evaluate these offline settings. It is never a controller input.

The [registered protocol](evidence/protocol/protocol.json),
[original input/source hashes](evidence/protocol/input_source_seal.json) and
[initial source freeze](evidence/protocol/pre_execution_source_seal.json)
retain the design and provenance.

The source recordings are the three native I-58 Gazebo/DART Mars figure-eight
runs, initial shaft -1.0/+0.3/+0.9 rad, yaw 45°, command caps
`[1.5, 0.6, 0.6 N m, 0.3, 0.3 m/s]`. Primary scoring retains the same
half-open [1,41) window and dominant-support eligibility: total normal load at
least 1 N and one family carrying at least 90%. This is not strict single
contact. Rows are correlated 5 ms records, not independent robot trials.

## Qualification and grouped fitting

Fit on two complete starts and evaluate the third, three times. Every training
candidate must remove at least half of A's harmful overrides, retain at least
90% of useful corrections, avoid increasing pooled label errors, satisfy the
per-start label/harm limits, avoid worsening point RMS/p95 versus A, and keep
maximum point discrepancy no greater than N. At least one proposal must remain.
Point metrics always use the fixed common finite N/A/truth mask, including
fallbacks, rather than only proposals that the setting retains.

Among qualified candidates choose the fewest label errors, then smallest point
SSE, fewest vetoes, and finally smallest integer parameter indices. A training
fold with no qualified candidate records a failed fit and uses A as a diagnostic
fallback. An all-rejected or nearest-miss setting is not promoted. The pooled
out-of-fold assessment requires all three fits to qualify. Relative harmful
reduction is unscorable for a phase with no harmful A examples; that phase must
still introduce no new harm.

All three runs were examined previously. This is **retrospective grouped
assessment**, not unseen validation. The three fold-specific settings evaluate
a tuning procedure; a full-data fit is descriptive and does not establish a
single validated deployment setting. Contact-label improvement alone would
also not establish better physical trajectory tracking.

The protocol and input/source hashes are preserved before sweep arithmetic.
Before that freeze, the point mask was clarified to use actual policy A's
point: an invalid raw proposal that A had already declined must not exclude a
valid native fallback. This was a design correction without new numeric results.

## Why retaining useful corrections matters

Here a harmful override means N has the correct family label but A changes it
to a wrong one. A useful correction means N is wrong and A fixes it. These are
offline labels, not measurements of actual controller damage.

For any extra veto on the same fixed proposals:

```text
new errors - A errors = useful corrections lost - harmful overrides removed
```

The known baseline has only six harmful overrides and 294 useful corrections.
Even removing all six bad changes permits losing at most six good corrections
if the total label error is to remain no worse. Thus the pooled error gate is
stricter than the nominal 90% retention gate.

A previously inspected good/bad pair also has incompatible requirements for
this particular affine threshold. That pair alone does not prove that no
aggregate tuning can improve. The grid and independent dominance audit test
the broader tradeoff explicitly.

## Result: a tiny label gain, no qualified setting

All three two-start training folds and the full-data fit have **0/3,721
qualified settings**. Consequently all folds explicitly fail selection and
fall back to A. The resulting out-of-fold 124 errors reproduce A; this is a
failed tuning procedure, not a newly successful estimator.

The following are descriptive pooled witnesses, selected after imposing only
the condition named in the row. **None is a qualified final candidate.**

| Offline policy / descriptive condition | a (N m), b | Label errors | Harmful changes remaining | Useful corrections retained / 294 | Point RMS (mm) | Point maximum (mm) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A: no additional veto | off | 124 | 6 | 294 | 2.757 | 29.843 |
| Original I-61 check | .030, .100 | 153 | 5 | 264 | 2.807 | 29.843 |
| Fewest errors anywhere in grid | .000, .150 | 123 | 5 | 294 | 2.752 | 29.843 |
| Fewest errors with at least 50% harmful removal | .000, .060 | 232 | 3 | 183 | 2.993 | 26.908 |
| Fewest errors with point maximum no greater than N | .005, .075 | 221 | 4 | 195 | 2.944 | 26.908 |

The best label-only witness rejects two proposals, prevents one harmful change,
and loses no useful correction. Its gain is **one correlated saved sample**;
it still fails both harmful-reduction and maximum-point requirements. This is
not evidence of improved driving or reliable generalization.

No grid setting simultaneously removes half the harmful overrides and retains
90% of useful corrections, even before the other gates. Among settings meeting
the harmful-removal requirement, useful retention reaches at most
183/294 = **62.24%**. The native N reference has 412 label errors and a
26.9076 mm maximum point discrepancy. A remains an offline comparator, not the
selected live estimator.

## Why a finer grid will not fix the worst-point tradeoff

The preplanned independent dominance audit finds that restoring point maximum
to N's level requires rejecting the -1 rad, 35.205 s proposal: its point error
is 29.8429 mm versus a native global maximum of 26.9076 mm.

Any shared nonnegative affine tolerance that rejects this proposal must also
reject **at least 99 useful corrections** on these saved states. There are
only six harmful changes available to prevent. Even removing all six therefore
gives at least `124 + 99 - 6 = 217` errors. The finite grid's tighter measured
minimum under the point-maximum constraint is 221.

This limitation extends beyond the particular grid for this fixed threshold
form and these fixed inputs. It does not rule out another model, temporal
reasoning, slip handling, or the velocity-consistency idea. The plain-language
problem is that some wrong hypotheses have a smaller joint-load mismatch than
good corrections. Moving a cutoff cannot reliably distinguish them.

[Independent bound audit](evidence/design_math/bounds_audit.json),
[mathematical derivation](evidence/design_math/threshold_geometry_strengthened.md)
and a [99-witness exact-sign certificate](evidence/design_math/strict_margin_certificate.json)
retain the evidence. The certificate is explicitly a **post-result supplement**,
not a preregistered test; its first execution preceded its own source receipt.
The byte-identical source was subsequently sealed and replayed. Its minimum
separating margin is 0.00603 N m, so the 99-count conclusion is not a floating
point boundary artifact.

## Verification and retained setup corrections

The protocol and 15 original input/source hashes precede new data arithmetic.
The initial source and independent audit sources were frozen before release.
The first primary attempt stopped at its old-setting benchmark: choices,
families, distance vectors and counts matched exactly, but batched SSE differed
from single-row summation by at most 2.26e-10 mm² (RMS 5.33e-15 mm). NumPy's
advanced column indexing had produced a different memory layout and reduction
order. No fold/full-data outcome was emitted by that attempt.

The correction makes each candidate row contiguous before summation and adds
an 8,003-point chunk-size-invariance contract. It preserves the original
comparison guard, score formula, parameter ranges, gates and selection order.
The corrected source is sealed before the canonical **stage02** run. The
original I-61 flags, family labels and distance vectors reproduce exactly;
all batched/single benchmark metric differences are now zero.

**11 synthetic replay/selection contracts pass**, inclusive of the earlier
nine/ten-check development stages; these are not additional controller tests
or a full repository suite. An abandoned, unexecuted validation-tolerance draft
and a ROS plugin collection failure are preserved separately. Disabling pytest
plugin autoload resolves the environment failure before the final test run.

Independent evaluation reads the original full I-61 inputs and recomputes every
grid cell, training gate, selection and out-of-fold fallback without importing
the primary scorer. All 63 main metric series, oracle counts, gate vectors,
selections and selected policy arrays match; even the floating metric arrays
are byte-exact. A separate audit checks point-maximum feasibility and
useful-loss bounds.

The published package is replayed from `/tmp` with system NumPy 1.21.5 against
the NumPy 1.26.4 producer. **All 139 arrays and the CSV are byte-exact**;
scientific JSON is exactly equal apart from NumPy-version metadata. No values,
thresholds or comparison tolerances are adjusted for this replay.

[Independent full-grid comparison](evidence/review/audit_stage01/canonical_comparison.json)
· [Independent bound/grid comparison](evidence/design_math/grid_bounds_crosscheck.json)
· [Standalone replay receipt](evidence/publication/replay_numpy121_stage01/replay_report.json)
· [Numerical correction/source receipt](evidence/analysis/pre_execution_stage04/correction_and_source_seal.json)
· [Eleven-contract log](evidence/analysis/pre_execution_stage04/pytest.log).

## Decision and reproduction

Keep native gain-scheduled LQR, momentum observer, rolling/holding feedforward
and 120 ms incoming-drive preparation selected. Do not introduce this load
veto. Preserve I-60's positive velocity-ranking result, which this experiment
does not reverse. Neither the original I-61 check nor the tuned witnesses
actuate the robot.

The next investigation should address what the instantaneous load hypothesis
misses, especially its time alignment and single-contact assumption during
transfer. I-61 found lost useful corrections near transitions; that motivates
the question but does not establish a unique physical cause. More threshold
tuning alone cannot satisfy the demonstrated worst-point/label tradeoff.

The compact package retains the full-rate inputs needed for this grid and
selection replay, cached Euclidean distances, source and complete numeric
reports. It does not regenerate geometric Jacobians, simulate driving, or
provide new trajectory/video evidence. Original full feature and truth files
remain local and are identified by hashes; the earlier I-61 archive retains
their separate compact reconstruction tools.

From the repository root, with Python 3 and NumPy installed (verified with
1.21.5 and 1.26.4), choose a new output directory and run:

```bash
python3 docs/iterations/2026-11-27/contact-load-tuning/evidence/analysis/replay_sweep.py \
  --package docs/iterations/2026-11-27/contact-load-tuning/evidence \
  --output /tmp/i62-replay-20261127
```

This repeats every setting and fold and checks against the archived canonical
report. It requires neither ROS/Gazebo nor local ignored results. The
[compact-input README](evidence/analysis/compact_stage01/README.md) describes
the cached quantities and limits of reconstruction. Original-input audit and
projection scripts are provenance tools requiring the listed local originals;
they are not needed for the standalone grid replay.

[Evidence inventory](evidence/portable_manifest.json)
· [Teammate guide](../../../TEAMMATE_HANDOFF.md)
· [Presentation notes](../README.md).
