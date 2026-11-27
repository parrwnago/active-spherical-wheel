# I59 independent raw-log numerical audit: negligible anchor effect

The independent full native-command pipeline audit agrees with the coefficient-only indication: retained startup shaft gain and equilibrium trim do **not** materially explain the recorded phase-performance differences. The source path exists, but its actual coefficients are almost identical in these three anchors. The earlier practical attribution to startup gain/trim should be withdrawn for these recorded runs.

Own pure NumPy source and all raw input hashes were registered before extracting coefficients or executing arithmetic. The calculation uses all 8,801 states of each of the six I58 B/P traces, the three B-derived anchors, and separate row-only, trim-only and combined interventions. It holds the recorded state, error, references, other gain rows, synchronization targets/weights, heading and fade conditions fixed. There are no new physical trials, trajectory predictions, resets, controller/model/observer calls or fitted thresholds.

| Intervention | Maximum native-request change, N m | Maximum native-clipped change, N m |
| --- | ---: | ---: |
| Shaft row only | 2.5274356e-8 | 1.9333110e-9 |
| Equilibrium trim only | 5.0397186e-13 | 5.0404125e-13 |
| Combined | 2.5274608e-8 | 1.9330585e-9 |

The maxima occur in torque components; belt differences are zero. All effects are below both independently reported engineering resolutions: 0.001 N m per torque /0.0001 m/s per belt and 0.001 of an input cap. The largest possible caps-normalized request effect is about 1.685e-8. Absolute and normalized material-state counts are zero for every dataset, anchor, intervention and fixed window, including uncertainty bounds from the float32 fade factor. Raw values and classifications are retained rather than relabeled mathematically identical.

All traces have byte-constant row 0 and trim histories, and B/P coefficients match byte for byte at the same phase. Across phase anchors, they are not byte-identical: the maximum shaft-row coefficient difference is 5.6830964e-8, and trim difference 5.0397186e-13. Row coefficients have heterogeneous state-dependent units; command effects in the table provide the relevant common-unit comparison.

All independent controls pass. Maximum full-stage/LQR recomposition discrepancy is 1.4210855e-14 command units, below 1e-10. This includes a separately vectorized matrix-times-error calculation, so its maximum is a different formulation from the model helper's approximately 1.39e-17 fade-stage residual; both satisfy the same declared tolerance. Actual-anchor request and native-clipped commands reproduce the recorded arrays byte for byte for all six datasets and all three intervention modes. These 18 correlated algebra controls are not new experiments or an added repository suite.

Fade is independently inferred from the exact pre-fade command, requested command and fixed damping, intersecting per-component closure constraints with the float32 recording bin. All states have a nonzero informative denominator; none is structurally degenerate in these traces. The full factor intervals are still propagated. Their maximum width is approximately 2.98e-8, and no candidate's material classification is ambiguous. The factor is not described as exactly known from rounded telemetry.

The audited boundary is native requested/native-clipped output. On P traces this is a native-policy shadow at actual recorded P states, not the final reoptimized P output. Shell/belt geometry, current phase/load gain scheduling, contact sequence, other initialization/history paths and terrain encounter timing remain possible contributors to whole-run phase differences. This result does not establish any one of those as the cause. It removes the retained shaft-row/trim explanation as a useful practical target for these logs.

`report.json` retains coefficient vectors/differences, exact controls and every window's maxima/classifications. Six NPZ files retain independent full-rate counterfactual deltas and fade bounds. The supplemental actual-anchor byte check was registered after the null result and is explicitly labeled as confirmation, not part of a blind hypothesis test. Source/input hashes remained unchanged. Canonical cross-comparison will be a separate immutable record when the complete primary pipeline outputs are available.
