# I-85: one bounded terrain-contrast lean trial

This policy adds a small anticipatory lean correction to the original observer-based
lean. It does not smooth or replace the native reference, current support estimate,
holding feedforward, rolling references, gains, heading behavior or trajectory clock.
Its parameters are fixed in [settings.json](settings.json); this proposal precedes
any recorded-array reduction or new physics for this trial.

At each measured command time, let `n_obs` be the current observer normal, `p` the
wheel-center XY position, `v` the measured world XY velocity and `R` the wheel radius.
Query the same delivered terrain memory at `x0 = p - R*n_obs.xy` and
`x1 = x0 + 0.5*v`. Both queries use the current delivery time. Future terrain is
not read directly from the scene; unseen or rejected map queries produce a zero
correction target. Current support and contact force are not predicted by this rule.

Convert each mapped normal through the native lean-vector and tilt-chart functions,
using the same current desired acceleration. The correction target is

`d = radial_cap_2deg(theta(n_map_future,a) - theta(n_map_current,a))`.

The native chart `theta_native = theta(n_obs,a)` remains unchanged. A two-dimensional
state `z` follows only `d`, with time constant 0.1 s and smooth radial velocity cap
0.08455015305021264 rad/s. SSP-Heun substeps are at most 1 ms and integrate the
previous held target before registering the current target. Convexity keeps `z`
inside the 2° target disk. Unknown map data changes the target to zero, allowing a
bounded return rather than abruptly deleting a previous correction.

A quintic envelope `w` ramps from zero to one over 99.5–100.5 s, stays one through
103 s, and returns to zero over 103–104 s. Its maximum derivative is 1.875/s.
The extra chart is `e = w*z`, with exact added derivative `e_dot = w*z_dot + w_dot*z`.
Thus `|e| <= 2deg` and `|e_dot| <= 0.08455015305021264 + 1.875*(2deg) = 0.15rad/s`.
Outside the window the original native quaternion is returned bitwise and the
feedback wrapper leaves the original angular reference untouched.

The modified quaternion is `q_eq * Exp([theta_native + e, 0])`. The extra spatial
angular reference is `R_eq * J_left([theta_native+e,0]) * [e_dot,0]`, transformed
into the actual body frame before feedback. This differentiates only the added
correction. Native changes in lean and carried yaw retain their original zero-rate
convention; this is not a complete derivative of the full desired attitude.

If the combined chart exceeds 28°, the policy raises `PolicySafetyError` and
retains failure diagnostics. It does not project the combined reference, alter the
native target or choose a new parameter after an outcome. The extra chart/rate
bounds also have explicit failure guards. Same-time identical reference calls
return cached copies without map queries or state advancement; changed arguments
at the same clock are rejected.

The 0.5 s horizon is a fixed source-based choice: it exceeds the 50 ms delivery
latency plus the 2°/0.15rad/s correction time, with reserve. No saved trial outcome
selects it. Thirteen synthetic contracts cover the bounds, frame derivative,
causality, fallback, cache, zero-effect cases and failure guard; they are not a
full repository suite or evidence of physical improvement.

The exported `native_quat` is the original rule evaluated at this candidate arm's
current observer normal and acceleration. Primary balance evaluation must use that
unaltered rule, alongside absolute tilt, rather than reward closeness to the moved
candidate target. The same-clock reference from the other physical arm is a
separate secondary comparison because the states can diverge.
