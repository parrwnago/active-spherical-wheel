# Contact-transition controller upgrade

The interactive lab now defaults to **Transition aware** feedback (`transition_aware`) with **Prepare incoming drives** enabled. It combines a library of local LQR gains with contact-load blending, heading-aware state errors, cyclic joint-angle handling, and kinematic preparation of an incoming shell or belt. **Original local LQR** (`local_lqr`) remains available for comparisons with the earlier controller.

This is a controller change. The wheel geometry, masses, friction, contact treatment, physics step, and actuator bounds remain unchanged. The implementation does not add a balancing force, hold the platform upright, reset a disturbed state, or silently move the requested position target. No model predictive controller is implemented.

## What the earlier controller missed

The earlier controller was designed around a local equilibrium. It could recover small disturbances while that operating point remained suitable, but repeated pushes could move the mechanism into a different contact regime.

- **Hold could retain a stale gain.** A stationary reference did not guarantee a stationary wheel configuration. The old hold schedule could contain just one gain even after the carrier rolled away from its initial phase.
- **Joint phase alone did not identify the ground-facing surface.** Fork pitch changes the carrier's orientation relative to the floor. The loaded hemisphere can change even while the shaft joint angle remains on the same side of its nominal value. Distinguishing only “shell” from “belt” also misses the difference between shell A and shell B.
- **A fixed linearization did not follow heading changes.** Translation errors are expressed in world axes, while local balance actions depend on the mechanism's current heading. Applying the old map after yaw changes could direct a corrective action along the wrong horizontal direction.
- **Unbounded material angles could create unnecessary unwinding commands.** An incoming shell may intentionally rotate before it is loaded. Treating accumulated turns as a large positional error can make the feedback fight that preparation.
- **Stable local eigenvalues were insufficient at finite rims.** Some very narrow boundary configurations produced extreme shaft gains while still passing the equilibrium and local spectral-radius checks. Those designs were unsuitable for direct gain switching under finite actuator bounds.

The earlier implementation is preserved at baseline commit `bb05156`. Its identifier and runtime source snapshot are retained under [results/recovery_control/baseline](results/recovery_control/baseline/commit.txt). That baseline is the original local LQR comparison; it is different from the older experimental `frozen_phase_feedback` design.

## Feedback follows the support geometry

[recovery_control.py](src/spherical_wheel/recovery_control.py) measures the carrier's orientation relative to the floor. For the declared horizontal-floor model, its effective phase is `atan2(-R[2,0], R[2,2])`, where `R` is the carrier's world rotation matrix. Unlike the shaft joint coordinate alone, this includes the effect of upper-body orientation.

The controller maintains separate gain groups for shell A, shell B, belt A, and belt B. It interpolates nearby phase gains within a group, then blends those gains using the measured normal loads. A contact handover can therefore change the feedback continuously while more than one surface carries load. The well-conditioned ordinary-pose shaft row is retained instead of importing an extreme finite-rim row.

Position targets and recorded trajectories stay in world coordinates. The controller rotates horizontal position and linear-velocity errors into the current heading frame. Its reference heading follows the platform's unregulated yaw; this upgrade does **not** provide an absolute-heading controller. Cyclic joint errors are wrapped to their equivalent shortest angular differences. References for deliberately pre-spun shells are also advanced so the position feedback does not undo their preparation.

The default-model library contains 80 accepted local designs. Building the library rejects nonfinite or nonstatic equilibria, locally growing closed-loop models, and extreme shaft pitch gains above 100 N m/rad in magnitude. Rejected phases and reasons remain recorded. These checks reject problematic designs; they do not establish a nonlinear region of attraction.

The library cache is an NPZ file containing numeric arrays and Unicode JSON metadata, loaded with `allow_pickle=False`. Its fingerprint includes the physical configuration, actuator bounds, MuJoCo version, generated model XML, and relevant controller/model/plant/contact source files. Changed inputs trigger a rebuild. The first uncached startup takes longer than a subsequent startup using a valid cache.

## Prepare the incoming surface before it carries load

[synchronization.py](src/spherical_wheel/synchronization.py) previews upcoming shell and belt exposure using current position and velocity. The default lookahead is 0.12 s. It evaluates several future poses with constant-velocity kinematics, checks the patch boundary and approximate support gap, and prepares an approaching drive while it is still unloaded.

The quantity to coordinate is **tangential surface velocity at the ground**, measured in m/s. Equal motor RPM is not sufficient. A shell's contact velocity depends on its instantaneous rotational axis, contact lever arm, carrier motion, and platform motion. A belt adds surface translation along its module's local travel direction. These mechanisms can have different drive directions and different effective lever arms at the same handover.

Conceptually, the ground-tangent velocity is the tangential projection of body translation, rotational velocity at the support point, and any belt surface velocity. The synchronizer chooses an incoming drive target that reduces the component of predicted slip that the drive can control. It also reports the outgoing surface's measured tangential velocity and the incoming mismatch. It does not claim that a single drive can match every velocity component, or that a slipping outgoing contact is an exact no-slip reference.

For a shell, the requested target is converted into a bounded **motor torque** that accelerates the real simulated shell inertia. The corresponding reaction torque acts on the carrier and upper body through the existing joint. Incoming-shell preparation is blended conservatively with the balance command. For a belt, the target remains an ideal moving-surface speed command under the existing belt model. Belt rotor inertia, motor current limits, and load-dependent speed dynamics remain unmodeled.

Preparation fades out as that drive receives normal load; loaded-drive feedback then takes over. The preview changes actuator commands only. It does not add a collision, teleport a surface, set a shell's velocity directly, or apply an extra external stabilizing force. It is a kinematic lookahead, **not** a dynamic contact forecast or nonlinear MPC rollout. Poor prediction, limited friction, saturation, or unavailable control directions can still prevent a smooth transfer.

## Compare the modes in the interface

1. Choose **Transition aware** under **Feedback mode** and leave **Prepare incoming drives** enabled.
2. Select a contact scenario, set the initial tilt, and press **Reset**, then **Run**.
3. Apply a repeatable push sequence or start a reference trajectory. Observe contact names, tilt, actuator commands, tracking error, and incoming-drive preparation status.
4. Export the run before resetting.
5. Repeat from the same initial conditions with **Prepare incoming drives** disabled to isolate the preview's contribution, or with **Original local LQR** to compare against the earlier feedback. Incoming preparation is inactive in that baseline mode.

Overlapping force pulses add. For example, 0.5 N pulses lasting 0.30 s and issued every 0.15 s can produce a 1 N net force during overlap. Compare the actual net force and event timings, rather than only the slider setting. Both successful and failed runs are useful evidence.

The backend exposes `controller_mode` and `synchronization_enabled` in state and saved runs. Mode changes use `{"action":"control_mode","mode":"transition_aware"}` or `"local_lqr"`; the preparation switch uses `{"action":"synchronization","enabled":true}`. Balance-controller and polar-belt switches retain their existing meanings. A disabled belt is held at zero surface speed with frictional contact still present.

## Measured results and remaining limits

The integrated [81-case comparison](results/recovery_control/stress_comparison.json) uses the same model, input bounds, initial conditions and intended force histories for both controllers. It covers three starting scenarios, three pulse magnitudes, three direction patterns, and three duration/spacing profiles. Each case requests eight pushes and runs for 10 s unless it falls. Pulses add physically, without clipping the disturbance.

| Measurement in this suite | Original | Transition aware with preparation |
| --- | --- | --- |
| Falls | 20 / 81 | 2 / 81 |
| Upright through the end | 61 / 81 | 79 / 81 |
| Strictly settled at the end | 46 / 81 | 45 / 81 |

Strict settlement requires tilt below 0.5°, planar speed below 0.03 m/s and total base angular speed below 0.08 rad/s. It does not require return to the position target. The new controller improves survival in 18 cases and introduces no new falls in this suite, but does not improve that stricter settlement count. Several survivors retain tilt/position bias or yaw motion. Both remaining falls involve strong overlapping same-direction pushes reaching 4 N combined force. These are observed limits of this controller, not proof that recovery is physically impossible.

The [selected matched runs](results/recovery_control/README.md) isolate synchronization from the feedback upgrade. A controlled belt-to-shell handoff reduces incoming candidate speed 5 ms before its first 1 N load from 0.08880 to 0.03899 m/s, and actual contact slip at that event from 0.07285 to 0.01478 m/s. Both runs use the same starting state and velocity reference. Preparation changes the contact time through real reaction torques. Other exploratory probes show that preparation does not improve every transfer.

Both difficult repeated-push cases also stay upright with a [0.5 ms physics step](results/recovery_control/numerical_step_check.json) and freshly designed gains. The shell case's final position bias changes from about 53 to 105 mm; this is not evidence of accurate position convergence. The polar case's final position error remains about 4.4 mm. [Browser checks](results/lab_ui/browser_qa.json) and [new control/overlap checks](results/recovery_ui/summary.json) cover the actual interface; UI-only fixtures are identified separately from physics evidence.

These are sampled recoveries, not a proof for arbitrary pushes, every hemisphere/belt transition, or a full range of orientations. Balance survival, settling, tracking accuracy, slip and transfer preparation need separate assessment. The earlier saved experiment results in [RESULTS.md](RESULTS.md) retain their original controller versions.

The underlying design study still uses assumed masses and friction, ideal belt speed servos, simulator state feedback, and a flat-floor contact approximation. Hardware validation requires measured physical properties, the final belt route and contact shape, motor/gear dynamics and limits, sensor estimation, latency, and experimentally checked transition behavior.
