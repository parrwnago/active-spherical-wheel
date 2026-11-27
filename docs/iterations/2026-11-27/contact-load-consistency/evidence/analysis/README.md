# I-61 campaign and portable analysis

This is a completed offline veto experiment, with **no new physics, live command,
controller/default change or threshold retuning**. All three I-58 native B
histories retain all 8,801 original states each.

The truth-blind feature stage consumes only the I-60 measured-state/context and
frozen selected-ID score artifacts. It reconstructs the proposed F point and a
physical candidate in the native family, then predicts current-pose generalized
load for the observed total force. The primary gate tests only the three joint
residuals against `0.03 N m + 0.10 R norm(observed_force)`. Native-relative load
comparison remains diagnostic. The q/F-only affine maps exported for oracle
evaluation contain no true contact point or label.

All three feature/policy artifacts and source hashes were sealed before the
evaluator loaded truth. Native N is the original family and equivalent point.
A uses the I-60 accepted proposal, including accepted same-family point changes,
and otherwise returns N. C adds the fixed extra-three load veto and returns N
after rejection. **A is not the all-frame raw I-60 F ranking.** Its 124 primary
errors must not be silently compared with the old raw-F figure of 109.

Primary `[1,41)` includes 23,423 dominant-support eligible states:

| Policy | Wrong/unknown family samples | Error fraction | Point RMS | Point maximum |
| --- | ---: | ---: | ---: | ---: |
| N | 412 | 1.759% | 3.134 mm | 26.908 mm |
| A | 124 | 0.529% | 2.757 mm | 29.843 mm |
| C | 153 | 0.653% | 2.807 mm | 29.843 mm |

C vetoes 46 of 6,902 accepted proposals. It prevents one of six harmful overrides,
while losing 30 of 294 useful family corrections: harmful overrides 6→5 and
retention 89.796%. Fifteen other vetoes change only geometry, not the family.
Five of thirteen registered gates fail: harmful-override reduction, useful
retention, pooled C-versus-A errors, +0.3-rad C-versus-A errors and point maximum
versus N. The candidate is not adopted or tuned after these outcomes.

Oracle-geometry compatibility passes at 99.870%, 99.881% and 99.352% on each
phase's stable dominant-support subset. This uses the **observed** total force
at the logged dominant-family mean point; it is not independent force truth,
nor a calibration of sensor noise. Raw mean points bypass material-band/radius
validation for this diagnostic only. Dominant support permits weaker additional
contacts; the count-one subset means one resolved record above 0.05 N, not one
physical contact.

The fixed events remain in `stage01/science.json`. At -1 rad, 35.205 s, the wrong
shell-B proposal still passes: joint residuals about `[.173, -.000002, -.105]`
N m versus `.251` N m tolerance. At 7.875 s, a correct shell-A correction is
rejected. Those observations show why the fixed veto fails; they do not establish
one unique physical cause or prove that every use of generalized load is useless.

## Chronology and verification

- Root source/input/protocol seals precede all historical load arithmetic.
  `campaign/features_stage01/all_features_sealed_before_truth.json` precedes the
  evaluator's truth access. Historical data, native control and I-60 scores are
  unchanged.
- Nine new evaluator contracts pass. The initial synthetic failure was a test
  fixture aliasing its truth/stable/oracle Boolean arrays; copies fixed the
  fixture. The evaluator mathematics did not change. Both source/test stages and
  failure log remain under `pre_execution_stage01/02`.
- The core separately reports 45 selected passes, including 17 new and 28
  existing tests. These scopes and earlier overlapping runs are not additive.
- The evaluator's second source stage adds root event-input binding and
  post-read precision-file hash verification, without changing decisions.
- Canonical full evaluation arrays are local under `stage01/`. The compact
  truth package preserves all original truth times, loads, patch means, resolved
  counts and equivalent points, plus the preselected event context and every
  canonical output array's dtype/shape/byte hash.

## Standalone reproduction

Python 3 and NumPy suffice; no MuJoCo or Gazebo is needed for stored-feature
arithmetic. With producer NumPy 1.26.4, the full scientific JSON is exact:

```sh
python3 PATH/analysis/replay_evaluation.py \
  --gate-data-source PATH/portable_tools/gate_stage01 \
  --evaluation-data-source PATH/analysis/evaluation_compact_stage01 \
  --output NEW_REPLAY_DIRECTORY
```

The default system NumPy 1.21.5 also reproduces every frame array, all primary
and fixed-window metrics and all gates exactly. Five scalar point distances in
fixed forensic events differ by at most `3.553e-15 mm`. The current validation
wrapper permits only the four named fixed-event N/A/C distance fields, within
`1e-12 mm`, and retains both raw values and difference paths. It reports
`full_science_exact=false`; it does not replace or normalize numbers. Seven
separate validation-adapter contracts reject all other differences. Producer
exact replay is `replay_stage02`; the older-version qualified pass is
`replay_stage03`. The original older-version exception and difference report
remain in `replay_stage01`, and `replay_environment.json` records the versions.

The compact gate package preserves all 59 feature/policy fields and reproduces
their derived values byte-for-byte. Geometric predictions and affine maps are
stored inputs: reconstructing them again requires the hash-bound original
states/model and MuJoCo. The replay regenerates full evaluation NPZs for plots,
but those duplicate arrays can stay local. No new trajectory is generated.
