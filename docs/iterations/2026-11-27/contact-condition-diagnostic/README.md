# I-63 — observable conditions for a dynamic contact-load threshold

2026-11-27. **The tested condition rule does not resolve the tradeoff.** The
signals are available without solved contacts, but the worst bad proposal also
looks transition-risky. The predeclared mathematical stop applies: no adaptive
schedule is fitted and no new driving simulation is run. The selected LQR,
momentum observer, feedforward and incoming-drive preparation stay unchanged.

[Overview PDF](evidence/analysis/plots_stage01/condition_diagnostic.pdf)
· [Overview PNG](evidence/analysis/plots_stage01/overview.png)
· [Annotated event windows](evidence/analysis/plots_stage01/event_windows.pdf)
· [Canonical report](evidence/analysis/stage01/science.json)
· [Previous tolerance sweep](../contact-load-tuning/README.md).

## Question and one fixed condition rule

The user suggested allowing the joint-load tolerance to change with observable
operating conditions. I-62 excluded a shared force-scaled rule; it did not
exclude every dynamic rule. I-63 tests one additional condition score, fixed
before new feature arithmetic. High score means the instantaneous load model
might deserve more allowance; it is not a contact probability.

Use two candidate-independent signals:

- **Observer change u.** Compare the observer's latest interval load estimate
  with its filtered output. Normalize the largest extra-joint difference by
  the old engineering scale `0.03 N m + 0.10 R norm(F)` and clip to [0,1].
  The interval ends at the current timestamp. This measures filtering/change,
  not computational delay or independent true force.
- **Seam condition s.** From the raw observer normal, current known patch axes
  and shell/belt geometry, compute signed surface distance to each polar seam.
  Use its adjacent backward rate to enlarge a 12 mm proximity window by
  `120 ms * abs(rate)`. Compute a continuous proximity score for each seam
  and take the largest. Both approaching and recently receding seams count.

The primary score is `r = max(u,s)`. The two signals share observer information;
they are not independent sensors. There is no use of candidate identity,
native load-share assignment, solver slip, true contacts/counts, terrain, or
future samples. Axes come from private model kinematics applied to saved
simulated pose; they are explicitly cached for portable replay.

Before execution, replace the proposed axial distance by spherical surface
arclength. The known polar cap has only about 4.35 mm axial depth, so a 12 mm
axial window would flag its entire surface. For signed axial coordinate z,
seam coordinate b and radius R, use:

```text
h = R * (acos(clip(z/R, -1, 1)) - acos(b/R))
s = max_over_seams clip(1 - abs(h)/(0.012 + 0.12*abs(backward_rate(h))), 0, 1)
```

Each seam is differentiated separately. Centered differences and differentiation
of a switching nearest-distance signal are excluded from the primary score.
Arccos clipping protects numerical limits near the pole. The first sample or
a missing/invalid predecessor gives unknown risk, which is retained explicitly.
These fixed scales reuse existing engineering settings; they are not fitted
noise statistics. Unexecuted axial advice/source remains in the archive.

## Saved runs, evaluation boundary and conditional plan

Use the same three native I-58 Gazebo/DART Mars figure-eight runs, 44 s each,
shaft -1.0/+0.3/+0.9 rad, yaw 45°, command caps
`[1.5,0.6,0.6 N m,0.3,0.3 m/s]`. All three 8,801-row condition artifacts are
generated without reading truth or candidates and sealed before evaluation.

Primary scoring keeps the original [1,41) window: 23,423 states with at least
1 N total normal load and a unique family carrying at least 90%. This permits
weaker additional contacts. Policy N is native; A uses an accepted I-60
velocity proposal and otherwise N. A has 124 family errors, including six
harmful overrides, and fixes 294 native errors. Its 18 accepted wrong labels
include 12 cases where N was already wrong. These denominators are distinct.

All runs were inspected in previous iterations. This is retrospective
diagnosis, not unseen validation. Frames and event clusters are reported
separately; six isolated bad frames are not six independent physical trials.
Truth labels and point references are evaluation-only.

The predeclared adaptive class is:

```text
tolerance(t) = a + b * R * norm(F(t)) + c * r(t), with a,b,c >= 0
```

First ask whether any coefficients in this class could satisfy both total
errors no worse than A and point maximum no worse than N. For an unsafe point
that must be rejected, the exact-sign dominance test identifies useful
corrections that necessarily must be rejected too. It retains the actual
load mismatch and force scale; a risk-only classifier is not substituted.

If this necessary condition did not exclude the class, the protocol allowed
18 fixed schedules: three preselected `(a,b)` anchors times six c values,
with unchanged I-62 gates and whole-start folds. Only a qualified full fit
and qualified pooled/fold assessment would permit a fresh +0.6-rad native
episode, still evaluating the adaptive estimator offline. None of these
conditional stages runs because the first stop condition fails.

## Observations and the stop decision

| Diagnostic quantity | Result |
| --- | ---: |
| Primary states with known combined risk | 23,423 / 23,423 |
| Accepted proposals with known risk | 6,902 / 6,902 |
| Combined scores on six harmful overrides | 0.892–0.959 |
| Median combined score, all 294 useful corrections | 0.971 |
| Median combined score, old 99 forced-loss useful corrections | 0.627 |
| Useful losses still forced by the primary dynamic class | **99** |
| Harmful changes available to prevent | **6** |

At -1 rad and 35.205 s, N correctly identifies belt B while A selects shell B.
This produces the largest guarded point discrepancy, 29.8429 mm, above N's
pooled maximum of 26.9076 mm. Its scores are `u=0.51934`, `s=r=0.93760`.
It looks highly transition-risky, so an extra nonnegative risk allowance also
protects this wrong hypothesis.

The extended exact-sign calculation shows that rejecting this point still
requires discarding **all 99** useful corrections identified by I-62. Even
preventing all six harmful changes implies at least:

```text
124 + 99 - 6 = 217 family errors
```

This is a conservative mathematical lower bound on the fixed recorded inputs,
not an achieved error count or a controller simulation. The 221-error result
belongs to I-62's separate finite static grid, not a new I-63 run.

The added risk inequality has a minimum positive margin of 0.01574 N m.
All 99 losses remain certified under the entire preregistered ±1e-4 risk
comparison envelope. Unknown features do not cause the failure: every primary
risk value is known. Training-subset bounds also reproduce independently.

Aggregate separation can be misleading. A useful-versus-harmful pair has a
higher combined score 66.84% of the time, but those correlated pair ranks do
not rescue the particular useful corrections that must survive. The
observer-only component, reported as a predeclared descriptive check, still
forces 59 useful losses. It does not replace the primary combined rule.

## Verification, evidence and interpretation

The [protocol](evidence/protocol/protocol.json),
[334-file input/model-asset seal](evidence/protocol/input_source_seal.json) and
[eight-source freeze](evidence/protocol/pre_execution_source_seal.json)
precede feature arithmetic. The [evaluation release](evidence/protocol/evaluation_release.json)
binds all three sealed truth-blind feature artifacts before truth evaluation.
Twelve feature contracts and fifteen separate evaluator contracts pass; these
are distinct scientific-tool checks, not a new full controller test suite.

Private axis kinematics runs with physics/contact-query guards. Pure NumPy
producer replay regenerates all 108 feature arrays byte-exactly and reproduces
all six analysis NPZ outputs plus scientific JSON. An independent implementation
uses separate dot-product/geometry and Fraction-bound calculations without
importing the producer. Counts, original 99 witness identities and stop
decisions match exactly. Its maximum primary score difference is only
2.71e-13; quantity-specific numerical comparison allowances were registered
before arithmetic, and no values are replaced to obtain agreement.

[Independent comparison](evidence/review/canonical_comparison.json)
· [Independent full report](evidence/review/results_stage01/report.json)
· [Producer replay comparison](evidence/analysis/replay_producer_stage01/comparison.json).

The archived package also runs from `/tmp` with system NumPy 1.21.5 against
the NumPy 1.26.4 producer: all 108 feature arrays and 183 analysis arrays are
byte-exact, and scientific JSON has zero numeric differences (version metadata
is excluded). No reported values, rules or tolerances are changed for that
replay. [Archive-isolated replay receipt](evidence/publication/replay_numpy121_stage01/comparison.json).

This result excludes the specified nonnegative affine allowance on the fixed
combined score. It does **not** exclude every dynamic threshold, prove a unique
physical cause, or invalidate the positive I-60 velocity-consistency result.
In particular, proximity and rapid changes indicate uncertainty; they do not
uniquely identify which family carries the load. General multi-contact force
decomposition remains unresolved.

Keep the live system unchanged. Before adding another hard veto, the next
proposed investigation is to compare candidate load histories with matching
observer filtering and timing. That hypothesis is not implemented here.
These offline point/label criteria do not establish a cause of trajectory
drift or a physical tracking improvement.

## Reproduction

Cached axes do not reconstruct the original model
kinematics; the compact package reconstructs conditions and analysis from them.
Original full measurement/model files remain local and hash-bound. The
package includes the I-62 baseline inputs so it does not depend on ignored
local originals for arithmetic replay.

From the repository root, with Python 3 and NumPy installed, choose a new
output directory and run:

```bash
i63_package=docs/iterations/2026-11-27/contact-condition-diagnostic/evidence
python3 "$i63_package/analysis/replay_condition_analysis.py" \
  --analysis-source "$i63_package/analysis" \
  --protocol "$i63_package/protocol/protocol.json" \
  --data-source "$i63_package/portable_inputs/i62_base" \
  --feature-source "$i63_package/features/stage01" \
  --feature-helper "$i63_package/features/feature_packets.py" \
  --old-witnesses "$i63_package/portable_inputs/i62_old_witnesses.json" \
  --reference "$i63_package/analysis/stage01" \
  --output /tmp/i63-replay-20261127
```

No ROS/Gazebo or MuJoCo is needed for this arithmetic replay. The archived
extraction scripts that regenerate axes require the hash-bound local models
and original logs; they are separate provenance tools. The two source
directories contain their own scoped contract logs. Original 5.3 MB primitive
inputs are retained directly; an unexecuted packing option was abandoned to
keep replay simple.

[Evidence inventory](evidence/portable_manifest.json)
· [Feature definitions and limitations](evidence/features/README.md)
· [Analysis details](evidence/analysis/README.md)
· [Teammate guide](../../../TEAMMATE_HANDOFF.md).
