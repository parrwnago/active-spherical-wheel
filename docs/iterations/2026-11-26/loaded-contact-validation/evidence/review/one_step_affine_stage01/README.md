# Exploratory teacher-forced one-step affine audit

The unchanged frozen contact model was reconstructed at all 8,800 held intervals of the existing I56 H trace. All 8,800 reconstructions matched the saved observer-derived contact names, loads, point, mismatch and confidence; no interval failed. No physics, controller or observer updates ran. Every prediction receives the exact current simulated state and the known actual applied command, then estimates the next 5 ms velocity. This is a teacher-forced one-step diagnostic, not a multi-step rollout or future-contact predictor.

| Error RMS | Frozen affine | Constant current velocity | Ratio |
| --- | ---: | ---: | ---: |
| Planar, m/s | 0.008616 | 0.019260 | 0.447 |
| Transverse angular, rad/s | 0.043244 | 0.112328 | 0.385 |
| World yaw, rad/s | 0.008064 | 0.096866 | 0.083 |
| Shaft, rad/s | 0.059664 | 0.205317 | 0.291 |
| Shell A, rad/s | 0.228735 | 0.664364 | 0.344 |
| Shell B, rad/s | 0.271982 | 0.540988 | 0.503 |

Average one-step errors improve in every group, but that does not make the affine map reliable at every interval. Affine peak planar error is 0.15857 m/s and transverse error is 0.82283 rad/s. Peak shell-A error is 8.69476 rad/s, worse than the constant-current baseline's 6.38737 rad/s maximum. The full report retains RMS, median, p90, p95, p99, peaks and peak indices/times for each group and endpoint-support stratum.

There are 8,708 intervals with matching four-patch support masks at their endpoints and 92 with changed masks (threshold 0.05 N). The affine RMS improvement is weaker on changed-mask intervals. Matching endpoints do not certify that support stayed constant inside the 5 ms hold. This evaluation-only truth never enters reconstruction or prediction; there is no extra body-contact category in the four-patch mask.

The comparison supports using the affine map as a candidate short-horizon baseline for further study. It does not validate a multi-step prediction, controller objective, stability claim or a unique cause of the I56 trajectory error. Two-state paired sensitivity agreement and broad teacher-forced affine-error statistics answer different questions. No model coefficient, contact penalty, confidence scale or test threshold was fitted here.

`full_rate_predictions_errors.npz` retains every common-frame prediction, observed velocity, error, confidence, load and support-mask label. `error_statistics.csv`, JSON, PNG and PDF outputs are preserved. The report uses endpoint angular velocities rotated separately into world before expressing them in the common initial fork frame. The exact source and all source/input hashes were registered before arithmetic.

Reconstructing this audit requires the original local I56 full-rate log and mirror XML. Copy the script into a new sibling review stage before running it; it refuses to overwrite an existing registration. The saved full-rate predictions/errors support independent NumPy-only recomputation of the reported statistics without MuJoCo or the original log.

Archive setup correction: the initial script manifest hashed its redirected stdout before the final summary print completed. That initial manifest is retained as `artifact_manifest_initial.json`; the final `artifact_manifest.json` was sealed after process completion, includes the complete stdout and this note, and is the publication manifest. The analysis source and numerical outputs were not changed.
