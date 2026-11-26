# I55: two matched-prefix causal diagnostics

The experiments establish that the unloaded shell-A actuation contributes to the initial yaw surge, and that the moving attitude reference contributes to the entering state. Neither diagnostic is a controller improvement to adopt.

All four fresh Gazebo cases stop at **32.500 s** on the original 40 s figure-eight program, starting from the original −1 rad configuration and yaw 45°. This is a completed diagnostic horizon, **not** a completed 44 s trajectory test. Batches use the original observer, gains, reference guard, 0.30 m/s assumed belt limits, and exact simulated state. The only physical branches are the two registered interventions below. No simulation or controller evaluation was performed by these post-processing tools.

## Replay and intervention checks

The native batch case reproduces all **56 shared precision fields over 6,501 rows** exactly. Splitting each logged hold into five 1 ms steps produces the same 56 fields exactly. Selected controller-history fingerprints also match. Both intervention prefixes, first intervention-time measurements, output/reference scope, known previous applied inputs, and all 1 ms clock checks pass.

Each trace has 1,100 consecutive 1 ms solves in 220 controller holds over 31.400–32.500 s. Before/after bridge states are retained; the controller remains at 5 ms. Every exported contact row is retained, including rejected and body contacts. This is a full native prefix replay, not a pose-only restore; DART hidden solver memory is not independently dumped.

Inherited `exact_native_*` fields describe the returned command after the I55 hook. The I55 sidecar's `pre_output_*` fields preserve the request before the actuator override; in the attitude arm, that request already includes the held quaternion. It is not a second untreated-command calculation.

## 1. Remove only unloaded shell-A torque

From **32.250 to 32.275 s**, only shell-A output torque is set to zero after the native command. Both cases have zero shell-A contact force and zero body contact force over all 25 masked substeps. All other held inputs, disturbance and friction match exactly for the **first 5 ms**. Later feedback may respond to the altered state, so the full 25 ms response includes mediated changes in other commands.

| Endpoint | Native world-z rate (rad/s) | Shell-A torque removed (rad/s) | Difference (rad/s) |
|---:|---:|---:|---:|
| 32.250 s, initial | 3.825019 | 3.825019 | 0 |
| 32.251 s | 3.923975 | 3.869608 | −0.054367 |
| 32.255 s | 4.315864 | 4.045030 | −0.270834 |
| 32.275 s | 5.992068 | 4.829581 | −1.162487 |
| 32.305 s | 5.957415 | 5.813504 | −0.143911 |
| 32.500 s | 2.170991 | 2.188376 | +0.017385 |

The first 5 ms yaw-rate rise falls from **0.490845 to 0.220012 rad/s**, a **55.2% reduction**, with the other inputs matched. The intervention therefore identifies a contribution from the unloaded shell-A actuation; it does not explain every part of the surge or make removing that actuation a safe controller policy.

At 32.255 s, treated-minus-native vertical angular momentum changes are fork+carrier **−0.002845950**, shell A **+0.002840411**, shell B **+0.000002654**, and whole assembly **−0.000002885 kg m²/s**. The nearly opposing body-group differences are consistent with the causal motor-reaction exchange, while the total changes much less. These are body-group differences between controlled branches, not a claim that every individual contact or constraint impulse has been uniquely identified. By 32.305 s planar speed is slightly greater in the treated branch, and the later yaw benefit largely disappears.

## 2. Hold the entering attitude target briefly

Only the yaw-free reference quaternion is held at its native **31.580 s** value until **31.745 s**, exclusive. The normal observer, gain lookup, XY program, velocity references and feedforward algorithms remain active. Their later numerical values can change naturally after the state diverges. The first resumed command is computed at 31.745 s and acts during the next hold.

The primary state at **31.745 s**, before release acts on the physics, is:

| Metric | Native | Held attitude |
|---|---:|---:|
| World-z angular velocity | 3.132654 rad/s | **1.872042 rad/s** |
| Absolute tilt from world vertical | 2.709158° | **4.898688°** |
| Error to the **same untreated desired up direction** | 9.848087° | **15.931811°** |
| Error to that arm's current normal-derived target | 9.848087° | 10.623560° |
| Error to the frozen 31.580 s target | 5.850142° | **0.495429°** |

The actual up directions differ by **6.128745°**. Thus target motion causally changes the entering platform state and yaw. The very small error to the obsolete held target is not evidence of better tracking of the original desired attitude: the fair same-target error and absolute tilt both worsen.

Release effects are retained separately. The resumed 31.745 s request is approximately **[2.279, −0.654, −4.912] N m**, clipped to **[1.5, −0.6, −0.6] N m**. The branch subsequently reaches **5.565 rad/s at 31.800 s**. This post-primary descriptive readout was motivated by the fixed-window plot and is explicitly labeled as such; it changes no primary result or gate. Over the same full post-release interval, the native peak is 6.635 rad/s at 32.290 s. Holding and abruptly releasing the target shifts and changes the transient; it is not an adopted fix.

## Force and angular-momentum accounting

The recorded forces are read after each completed physics step. Versioned DART and gz-physics source supports `force = solved contact impulse / dt` and force pass-through. Reconstructing `F × dt` is therefore source-supported, conditional on sign, ordering, completeness and the actual engine implementation. The raw force sign is never changed to improve closure.

Contact moments use the **same exported individual point** with the midpoint system COM as the registered primary center. Prior/post COM sensitivity is always reported; no center or clock is selected for a better result. All contact rows enter the total budget, rather than only admitted wheel-support records.

| Native interval | State-derived ΔH_z (N m s) | Midpoint all-contact yaw impulse (N m s) | Contact minus state | Registered closure |
|---|---:|---:|---:|---|
| 32.250–32.275 s | 0.001385443 | 0.001311511 | −0.000073933 | **Pass**, 5.34% residual |
| 32.250–32.305 s | 0.002798098 | 0.002467804 | −0.000330294 | **Fail**, 11.80% residual |

The threshold is `abs(residual) ≤ max(0.0001 N m s, 0.10 × abs(ΔH_z))`. Prior/post centers also pass the 25 ms interval and fail the 55 ms interval. The failed 55 ms budget is retained; no force-clock shift or tolerance relaxation is applied. A failure means force/model accounting is incomplete at the requested precision, not that motor reaction is absent.

All body masses, COM offsets and inertia tensors are checked against each case's actual SDF/mirror pair. Static world-frame COM Jacobians reconstruct spin plus orbital angular momentum. Individual-body/group budgets include known shaft and shell motor couples and the moving-system-COM transport correction `−m_i r_i,rel × ΔV_COM`, whose sum cancels in the whole assembly. Remaining body-group discrepancies can include joint/passive transfer, discretization and force/model residuals.

Linear momentum is independently checked against all raw contact forces, gravity and the known push. For the native 25 ms interval, the force-minus-state impulse residual is **[−0.001784, +0.001476, +0.000358] N s**, versus state Δp **[+0.074277, +0.172444, −0.017238] N s**. These convention checks support the raw force orientation without claiming exact closure of every substep. Recorded contact points are solver-stage geometry, while the published body state is post-step; this is not evidence of a 1 ms publication delay.

## Artifacts and execution history

- [Canonical four-arm report](stage01/report.json) and [paired endpoint CSV](stage01/paired_outcomes.csv).
- [Compact summary](presentation_stage02_three_targets/summary.json).
- [Shell-A causal response](presentation_stage02_three_targets/shell_A_causal_response.png), [PDF](presentation_stage02_three_targets/shell_A_causal_response.pdf).
- [Attitude response with three target comparisons](presentation_stage02_three_targets/attitude_causal_response.png), [PDF](presentation_stage02_three_targets/attitude_causal_response.pdf).
- [Original declaration](stage01/registered_before_reduction.json), [math checks](checks_stage02_corrected/report.json), and [early native/zero-A report](early_stage01/report.json).
- [Force-source audit](../review/budget_source_review_stage01/report.json) and [same-step source-timing audit](../review/source_timing_stage01/report.json).
- [Post-primary release readout](release_followup_stage02_stdout_corrected/report.json), labeled separately from the registered primary endpoint.

Nine math/boundary checks pass; this is one set of artifact checks, not additional robot trials or repository test counts. The first check stage's inclusive floating-point boundary failure is preserved, followed by a correction adding only a 2×10⁻¹⁵ N m s comparison-roundoff allowance; the registered physical tolerance is unchanged. The first release follow-up wrote its data successfully but its final console print failed on NumPy-array serialization. That attempt and its output remain intact; a fresh stage corrects only the stdout formatting. The initial two-target figure remains alongside the later three-target presentation revision. None overwrites the canonical report or any earlier campaign.

Executed commands use `.venv/bin/python`: `analysis/check_math.py`, `analysis/analyze_available.py --output .../early_stage01`, `analysis/analyze_causal.py --output .../stage01`, `analysis/present_causal.py`, `analysis/present_causal_stage02.py`, and `analysis/describe_release.py`. Each stage preserves executed source, input hashes and a manifest; logs are in this directory. Outputs refuse overwrite. All raw arrays and large derived momenta remain local to the registered family.
