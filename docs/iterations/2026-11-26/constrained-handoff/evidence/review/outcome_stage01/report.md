# I-56 independent saved-log audit

Independent NumPy formulas reproduced the canonical tracking, attitude, late yaw, absolute heading travel, final input amplitude/rate residuals, optimizer residual maxima and all six geometric overshoots for B, A and H. The resulting 21 gate booleans agree for each candidate: H fails eight conditions and A fails five. The audit reads recorded execution/provenance assertions for historical parity and frozen-bank identity; it does not independently rerun native physics or reconstruct every execution fingerprint. No simulation, controller, observer or model calls were made.

| Quantity | Native B | No-yaw/no-momentum A | Full H |
| --- | ---: | ---: | ---: |
| Moving timed XY RMS, mm | 51.08 | 57.15 | 55.14 |
| Full timed XY peak, mm | 116.26 | 183.97 | 136.16 |
| Final timed XY error, mm | 42.65 | 15.47 | 76.72 |
| Peak raw reference lean error, degrees | 7.40 | 13.43 | 13.85 |
| Peak absolute tilt, degrees | 13.72 | 15.04 | 18.96 |
| Late world yaw peak, rad/s | 2.443 | 10.013 | 8.753 |
| Late absolute heading travel, turns | 1.800 | 1.081 | 1.542 |

All three completed 44 seconds; none passed the original final settling test. That makes the conditional settling comparison pass vacuously, rather than demonstrating good settling. H and A enforce the assumed final total-input boxes and slew limits to floating-point precision. Their small secondary residuals demonstrate the implemented local numerical hierarchy, not native balance equivalence or physical stability.

H fails late yaw peak, late absolute heading travel, full tracking peak, final tracking error, raw lean peak/RMS, absolute tilt peak, and the six-turn overshoot conjunction (the sixth turn fails). A fails late yaw peak, full tracking peak, raw lean peak/RMS and absolute tilt peak. H's moving RMS passes because the registered allowance is `1.1*B + 1 mm`, not because RMS improves. The conditional extension rule therefore stops the additional phases. Neither arm is an adopted alternative.

## Event evidence and limits

These event thresholds are exploratory saved-log analysis selected after outcomes, not preregistered causal tests.

- The first sustained H timed-error excess over B of 20 mm begins at **4.855 s**. H is 28.72 mm from its timed reference versus B's 8.64 mm. Both primary and secondary response differences are at numerical zero, no final slew bound is active, raw reference lean error is only 0.129 degrees, and the estimated shell-B load is 18.099048 N versus 18.099010 N in saved simulator truth. This rules out forced slew loss or incorrect shell/belt assignment at that sampled onset as a sufficient explanation for the early degradation. It does not identify a unique physical cause or prove that every earlier contact estimate was correct.
- Absolute tilt first stays above 10 degrees at **7.735 s**, but raw reference lean error there is 1.186 degrees. The reference itself is tilted. This event must not be described as a 10-degree balance-tracking failure. Primary/secondary deltas remain numerical zero and no slew bound is active at that sample.
- The first primary discrepancy norm above **0.01 rad/s** sustained for at least 10 ms begins at **10.075 s**. The audit JSON key `over_01rad_s` means 0.01, not 0.1. Its threshold and duration are analysis choices, not actuator or stability limits.
- Raw reference lean first stays above 5 degrees at **28.530 s**. Peak absolute tilt is **18.964 degrees at 28.705 s**. The nearby +/-100 ms window contains primary response-loss norms up to 2.239 rad/s while secondary differences remain numerical zero. This distinguishes unavoidable first-stage loss inside the chosen slew boxes from a failure of the stage-two equality. It is a temporal association, not proof that rate limiting caused the entire event.
- The maximum primary loss is **3.741 rad/s at 35.925 s** with estimated belt-B support and active torque/belt slew bounds; secondary discrepancy is zero. H's largest timed XY error, **136.16 mm at 41.655 s**, happens with no active slew and numerical-zero primary/secondary discrepancy. Accumulated state error can persist after the limiting event ends.

The recorded yaw proxy is `current velocity + G*(candidate-native_at_this_state)`, a 20 ms local differential counterfactual. The measured next 5 ms velocity includes the native input's response, common drift, changing geometry and contact impulses. Directly comparing these unpaired quantities cannot validate or refute G. The logs establish that constraints and equality were enforced, and that early degradation exists without forced primary loss. They do not isolate model inadequacy, auxiliary damping, horizon mismatch or another physical mechanism. A same-state paired sensitivity test would be required to estimate the candidate-minus-native physical response; none was run here.

## Reproduction and archive

Run `.venv/bin/python results/constrained_handoff_20261126/review/outcome_stage01/audit_saved_arrays.py` from the repository. The script reads the canonical portable full-rate metric projections and hashes its inputs. Its event excerpts additionally use the local full H precision and governor-diagnostic files; those large files are not duplicated in this review folder. Exact values, selected indices, command deltas, support loads and diagnostic excerpts are in `audit_result.json`.

Current source review is separately sealed in `review/design_stage02`; earlier review source/provenance corrections are preserved there. This audit is not an added repository test-suite count. It adds no physics executions.
