# Motion content review

Initial material finding: guide SHA256 `03fb09e3fbd031ae5e8fec9ad488427217bf751b4525254626682aa644772af1`.

Resolution reviewed against guide SHA256: `8bc3812204331d09ee2157cf356747b4dbd6f4f704c4ebbceebd850c16f262b7`.

The equations, matrix signs, numerical authority examples, single-contact qualification, original31.650s case identity, six-turn metrics and causal limits match the sealed motion evidence. The material sentence was flagged and has now been corrected in the latest reviewed version. Remaining notes are precision clarifications.

## Resolved material finding — page25, turn_tradeoffs

The sentence “Consistent desired tilt rate reduced early demand relative to the same shaped angle with zero rate” is misleading/incorrect for the recorded belt command. In the I43 demand window [6.6,6.9]s, angle_only versus consistent_rate gives:

| Metric | Angle only | Consistent rate |
| --- | ---: | ---: |
| Peak belt-A requested speed |0.376269m/s|0.490620m/s|
| Local path RMS |14.251872mm|14.642777mm|
| Raw-reference peak lean |5.305167°|3.677860°|

Thus consistent rate improves raw-reference lean in this window, while belt demand and local RMS increase. The two closed-loop arms share generator/settings, not an identical shaped-angle history after commands/states diverge.

The corrected guide now states the same distinction as this recommendation: “Adding the consistent rate improved raw-reference lean in the demand window, but increased peak belt demand relative to angle-only. The arms used the same generator settings; their closed-loop shaped-angle histories differ. The full candidate still failed the raw-target attitude and turn screen.”

## Reproduction/precision clarifications — page22, authority_event

The worked regularized calculation is numerically correct. To reproduce it on this page, state `S=diag(5,5)rad/s` and `v_ref=(0.0580884703,0.0311704930)m/s`. The rates +502.66/−1811.23rad/s are reconstructed, not the exact logged references. Explicitly retain the float32 saved-pose/contact-point limitation; actual saved references are +488.601/−1760.525rad/s. The table's “same-state allocation calculation” wording already helps, so this is a precision clarification rather than an incorrect formula.

## Small clarifications

- Page21: add m/rad to the physical singular values [0.12,0]; scaled values already carry m/s correctly.
- Page23: make clear that100ms dwell qualifies the actual reversal time, not the geometric overshoot magnitude. The overshoot uses the fixed [−1,+2]s event window.
- The local geometric distance is the branch-constrained reference-polyline projection within±2s. “Limited progress mismatch” is accurate; explicitly naming±2s would make the definition easier to defend.

## Items passed

- Page20 direct body map D=−P_xy[a×r], one loaded shell, roll−20°/phase−90°, yaw/normal qualifications.
- Page21 the two2×2 matrices, physical-versus-normalized units, feasible x and unavailable y requests, and pseudoinverse explanation.
- Page22 condition number≈3961, normalized-rate regularization,56.84mm/s residual, distinct31.650s versus6.825s mechanisms, correct original baseline/phase/yaw/route/engine metadata.
- Page23 firstMars86.46mm timed/11.52mm local/0overshoot/+1.101slag and flat8.03mm/+0.193s; component reversal is not a full stop; no sole-cause attribution.
- Page24 D10 and later candidate tradeoffs match iteration records; separate campaigns/baselines are qualified.
- Page25 synchronization4–10s intervention, first-turn1.101→.832slag with0→18.05mm overshoot, final60.20→68.41mm, and carried history after restoration.
- Page26 proposed directions are clearly unimplemented and do not promise recovery or a stability guarantee.
- Page27 rate-reference constraints versus complete torque/surface-speed command constraints, normalized speed versus power, and distinct early-balance/late-authority examples.

No root content was edited. No simulations, runtime functions or scientific reductions were rerun. The review used sealed JSON/NPZ-derived reports and existing implementation/iteration notes.
