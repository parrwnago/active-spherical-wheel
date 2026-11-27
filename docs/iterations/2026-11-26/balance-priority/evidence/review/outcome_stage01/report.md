# I58 independent lead saved-data audit

All 26 preregistered lead gate booleans agree with the canonical reducer and pass. Fresh B and A also reproduce all 59 shared historical precision fields byte for byte under independent comparison. This makes the conditional +0.3/+0.9 B/P campaign eligible; those four runs were not yet available in this stage and remain untested here.

The audit independently recomputes timed XY error, raw reference lean/absolute tilt, world yaw/absolute heading travel, full planar speed, full 50 ms three-component linear acceleration, total first-plus-secondary priority discrepancy, total-input caps, actual-input history, held continuity, six geometric overshoots and original settling from saved arrays. It does not import the canonical metric formulas. Bank/execution assertions remain recorded provenance, and model priority values are read from saved diagnostic components rather than independently re-evaluating G at every state. Turn reversal-lag details are not independently rederived; the overshoots used by gates are.

| Quantity | Native B | Hard-slew A | Priority P |
| --- | ---: | ---: | ---: |
| Moving timed XY RMS, mm | 51.08 | 57.15 | 33.43 |
| Full timed XY peak, mm | 116.26 | 183.97 | 72.16 |
| Final timed XY error, mm | 42.65 | 15.47 | 6.29 |
| Raw reference lean peak, degrees | 7.40 | 13.43 | 6.81 |
| Raw reference lean RMS, degrees | 1.206 | 1.725 | 1.021 |
| Absolute tilt peak, degrees | 13.72 | 15.04 | 13.62 |
| Late world yaw peak, rad/s | 2.443 | 10.013 | 1.349 |
| Full planar speed peak, m/s | 0.47171 | 0.93767 | 0.47220 |
| Full 50 ms linear acceleration peak, m/s² | 8.0775 | 21.9007 | 8.9546 |
| Caps-normalized held command-step RMS | 0.12753 | 0.12239 | 0.09553 |

P's maximum absolute modeled transverse discrepancy is 4.6865e-16 rad/s, and held RMS is 7.0450e-17 rad/s, versus A's 0.26742 rad/s held RMS. This verifies the intended priority restoration inside the model and hard amplitude bounds; it does not prove physical stability or predict the whole path.

The physical acceleration guard passes narrowly: P's 8.9546 m/s² is below the predeclared 8.9853 m/s² limit, while exceeding native B's 8.0775 m/s² by about 10.9%. This must remain visible alongside improved tracking and command-step metrics. Removing optional rate constraints does not bound actual acceleration. P still exceeds the old diagnostic 60/6 thresholds at some steps; those are explicitly not hard P constraints.

All three completed 44 seconds, and none passed the original final settling test. The conditional settling gate therefore passes vacuously. Do not describe the lead result as successful settling. Continuity is a prespecified secondary diagnostic; its improvement is not substituted for the mandatory priority benefit or any failed gate.

`audit_result.json` contains exact independent values, all gate booleans and explicit unavailable phase pairs. `registered_before_outcomes.json` and source-freeze records precede new outcome access. `audit_inputs_registered_before_arithmetic.json` binds source-stage, projections, diagnostics and both fresh/historical full precision files before numerical arithmetic. Input/source hashes remained unchanged. No simulation, controller, observer or model calls were made by this audit. No runtime sources or thresholds changed.
