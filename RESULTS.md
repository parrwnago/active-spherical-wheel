# Simulation results

Prepared 18 October 2026. These are actual runs of a provisional model, not physical measurements.

## What is implemented

The repository combines native CAD display meshes, CAD-derived 120 mm outer radius / 110 mm inner radius, 32 mm polar apertures and 20 mm equatorial gap, a simple fork/platform, finite-friction ground contact, and five explicit drive channels. The floating fork/platform has six base degrees of freedom; three relative hinges preserve the carrier and independent shells.

Three commands are bounded equivalent output torques. Two are ideal belt surface speeds. Component masses, payload location, friction and actuator bounds remain assumptions. The belts use ideal spherical-cap exposed contact surfaces; complete belt routing and actual load-dependent drive dynamics are not modeled.

## Demonstrated runs

The new interactive controller is assessed in [CONTROL_UPGRADE.md](CONTROL_UPGRADE.md). Its matched 81-case repeated-push suite reduces falls from 20 to 2 with unchanged physical parameters and actuator bounds. Strict end-of-run settlement is 46/81 for the original and 45/81 for the revision, so improved survival must not be interpreted as improved convergence in every case. A controlled incoming-shell handoff reduces measured contact slip from 0.07285 to 0.01478 m/s. See [selected paired rollouts](results/recovery_control/README.md), [complete comparison](results/recovery_control/stress_comparison.json), and [plots](results/recovery_control/comparison.png).

The saved demonstration table below records the earlier local-feedback implementation. These historical runs have not been relabeled as results of the new interactive controller.

All selected runs below use MuJoCo 3.16.0, a 1 ms physics step, a 5 ms control interval and `analytic_clipped_floor` contact. Balance uses a discrete full-state LQR designed in the reachable subspace. Tracking adds contact-velocity feedforward, integrated nominal drive positions and a quasi-static lean approximation. The transition run schedules local gains by phase and contact regime.

| Experiment | Duration | Final reference-relative tilt | RMS planar position error | Status | Evidence |
| --- | --- | --- | --- | --- | --- |
| Shell-contact balance, 2° roll + 2° pitch | 8 s | 0.00104° | 4.389 mm | Passed | [metrics](results/balance/summary.json), [plots](results/balance/plots.png) |
| Polar-belt balance, 2° roll + 2° pitch | 8 s | 0.00073° | 5.408 mm | Passed | [metrics](results/polar_balance/summary.json), [plots](results/polar_balance/plots.png) |
| 0.5 N horizontal pulse for 0.1 s | 8 s | 0.00149° | 6.666 mm | Passed | [metrics](results/disturbance/summary.json), [plots](results/disturbance/plots.png) |
| 4 cm forward curve, 2 cm lateral excursion | 10 s | 0.01790° | 1.186 mm | Passed | [metrics](results/tracking/summary.json), [plots](results/tracking/plots.png) |
| 6 cm curve crossing shell-to-belt contact | 10 s | 0.00995° | 1.447 mm | Passed | [metrics](results/transition/summary.json), [plots](results/transition/plots.png) |

The balance run starts with 2° in each tilt axis (about 2.83° combined). Its commands saturate for one control interval; no fork/platform floor contact occurs. The disturbance run starts at equilibrium and applies the logged world-frame force to the fork body from 1.0 to 1.1 s. Tracking references are gentle, finite trajectories; these cases do not establish high-speed or arbitrary-path capability.

The transition run begins at shaft phase 1.10 rad and ends near 1.612 rad. Normal load transfers through mixed shell-rim/belt contact around 4.3–4.9 s. Two local gain-design points near the boundary were rejected and remain in `gain_schedule.json`; the experiment uses a nearby available gain from the same contact regime. This successful trial does not certify all contact transitions.

Saved animations: [balance](results/balance/preview.mp4), [tracking](results/tracking/preview.mp4), and [shell-to-belt transition](results/transition/preview.mp4).

## Interactive lab and polar belt checks

The local browser lab steps the same nonlinear MuJoCo plant. Run `bash scripts/run-lab.sh` on Linux and open `http://127.0.0.1:8765` on that computer. The interface supplies finite force pulses, initial tilt, drive toggles, trajectory references, live rendering/plots, and CSV export. See [INTERACTIVE_LAB.md](INTERACTIVE_LAB.md) for controls and assumptions.

An independent polar-contact comparison isolates transverse belt authority. From either +2° or −2° roll, the enabled belt recovers to 0.000408° tilt after 8 s; the three joint torque commands remain below 6.3e-13 N m. Holding belt speeds at zero with the same controller produces a fall at 0.975 s. A ±0.5 N transverse force for 0.1 s gives 0.583° peak tilt and 0.00110° final tilt with the belt enabled; the corresponding stationary-belt case falls. Only the belt touching the floor supplies this traction. These comparisons use ideal speed servos and do not establish real motor effort capacity. See [comparison and reproduction script](results/polar_belt_check/README.md).

The saved earlier live-runtime regression also starts at the pole, follows a 4 cm forward / 2 cm lateral curve for 10 s, and applies a 0.5 N transverse pulse for 0.1 s at 2 s. It completes without a fall, ending at 0.01896° tilt and 4.394 mm position error. Actual support moves from belt A onto shell A; local gains follow phase and contact regime. Its free root has no equality constraints, no externally applied generalized forces, and only the specified fork force pulse. See [historical portable saved run](results/lab_polar_tracking/README.md). The same physical regression also passes under the revised runtime in the current test suite.

Actual Chromium interaction tests on Linux check polar recovery, button/keyboard/drag pushes, pause, stationary-belt failure, a pushed 10 s curved path, CSV download, camera persistence, and narrow-screen layout. Browser errors are recorded in [browser QA](results/lab_ui/browser_qa.json); the optional [reproduction script](results/lab_ui/reproduce_browser_qa.py) requires Playwright and Chromium. These are QA tools only, not runtime dependencies. The headless EGL renderer and HTTP endpoints were also exercised in [live service smoke checks](results/lab_smoke/summary.json).

## Physics and control checks

- **44 automated tests passed.** They cover free-base topology, physical inertias, isolated internal torque reactions and momentum conservation, correct belt response, floor-contact failure detection, clipped-contact geometry, pulse duration and logging, paused simulation time, live trajectory references, and polar-to-shell tracking under a push.
- A tilted zero-command run falls until a fork arm contacts the floor at about 0.442 s. This is the expected unsupported behavior; zero belt speed still represents an ideal holding servo.
- The static-equilibrium controller-interval residual is approximately 2.66e-13. Independent finite-difference prediction agrees with nonlinear perturbations at about 3.3e-5 relative error for the recorded small perturbation.
- Full 18-state closed-model eigenvalues are inspected. The nominal reachable rank is 12; stable contact modes and neutral cyclic/unregulated coordinates remain. The model is not claimed to regulate every coordinate asymptotically.

See [physics diagnostics](results/physics_validation.json), [control diagnostics](results/control_validation.json), and [linear analysis](results/analysis/linear_analysis.json). Test invocation is recorded in [test results](results/test_results.txt).

## Numerical and provisional-design sensitivity

The shell-to-belt run was repeated with a 0.5 ms physics step and a newly designed local gain schedule. Both runs completed without a fall: RMS position error was 1.447 mm at 1 ms and 1.454 mm at 0.5 ms. This compares numerical implementations, not fixed-gain robustness. See [comparison](results/transition/numerical_comparison.json).

A separate fixed-gain study kept the nominal controller while changing the platform from 0.8 kg / 0.18 m center height to 1.2 kg / 0.22 m. All three illustrated cases recovered from 2° roll and pitch for five seconds. See [sensitivity metrics](results/sensitivity/summary.json). These three cases do not establish broad robustness to unknown masses, friction or motor limits.

## Preserved failures and scope limits

The earlier faceted shell-contact model created artificial material-phase-dependent rolling behavior. A full-state local LQR failed the large-tilt trial, and an earlier reduced controller missed the tracking target. Those runs remain in `results/faceted_full_lqr_large_tilt`, `results/faceted_tracking`, and the `faceted_*_validation.json` reports.

The selected flat-floor model uses exact sphere-plane support only when the support point lies inside the corresponding shell band or belt cap. Outside that region, the proxy is excluded and finite rim/edge geometry supplies contact. Tests verify that no whole-sphere contact bridges the equatorial or polar openings. Raw MJCF remains a safe faceted fallback; the Python plant installs the clipped-contact rule. Arbitrary terrain is outside this rule's scope.

Hardware performance, motor torque-speed/current limits, belt effort/load dynamics, estimation and sensor delays, guaranteed yaw regulation, trajectory optimization and global stability remain unvalidated or unimplemented. Native saved CAD geometry is used for appearance and measured dimensions; it is not a completed material-assigned mechanical twin.

## Reproduce

Use the exact commands in [README.md](README.md). Each main run retains its own configuration, generated model and meshes, controller data, commands, contacts, disturbances and compressed rollout. Commands logged at time t are held over the following controller interval; the final row is a terminal sample and is excluded from actuator-use summary metrics.
