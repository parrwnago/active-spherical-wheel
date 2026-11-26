# I51 late-drift source and saved-data audit

The191 mm I49 baseline deviation was **not an intentional relaxation by the feasible allocator**. I49 used `rolling_allocation='least_squares'`. The damping passed to that branch was zero; the recorded0.01 damping parameter was inactive. The recorded20 rad/s joint-reference limits, reference acceleration limits, effort/change costs and catch-up parameters belonged to the inactive bounded/feasible branches. They must not be described as enforced in I49.

The desired XY position and XY velocity are unchanged on all8801 saved rows, exactly. No contact-point shift, balance-based travel reduction or motion-reference projection was active. The planar rolling equation was solved without a material residual in the late interval.

This does not make actual velocity a hard constraint. The LQR applies a weighted state-feedback torque; the requested joint rates are state references inside that feedback, not separate velocity servos. Tracking competes with balance and input costs. This distinction explains what the controller asks for, but does not by itself identify which weights or model terms should change.

## What the late data rule out

The baseline interval38–42 s contains800 samples under the declared half-open window. It has:

| Diagnostic | Baseline38–42 s |
|---|---:|
| Maximum nominal planar allocation residual |1.22e−17 m/s|
| Median / maximum normalized rolling condition number |3.39 /4.87|
| Samples with a requested actuator limit violation |0 on all5 drives|
| Body velocity error RMS |40.39 mm/s|
| Saved solver-contact slip RMS |0.0901 mm/s|
| Median estimated-to-solver equivalent point distance |0.0235 mm|
| Raw dominant-identity disagreement samples |1 /800|

The one identity disagreement includes endpoint contact availability; it is not evidence that the entire interval has exact physical contact knowledge. The estimated contact remains a single shell support in all800 samples. These results identify continued rolling at an incorrect body motion, rather than a large allocated velocity residual, dominant sliding, reference clipping or a late geometric singularity.

Fixed comparison intervals10–14,20–24,34–38 and raised-cap38–40 s are retained in the JSON. They are different configurations and physical histories, not controlled independent trials. In particular, the raised-cap runs also have material late motion error despite small allocation residuals and moderate conditioning.

## Static reference balance is internally consistent

A separate current-configuration calculation used the exact saved model masses and COM Jacobians, a plane with the currently estimated filtered normal, one spherical support at `center−R*normal`, and a vertical support force equal to total weight. It evaluated current and requested body attitudes with the same actual joint configuration and carried yaw. Only kinematics, COM positions and Jacobians were evaluated. No dynamics, contact solver, controller, observer, simulation step or equilibrium optimizer ran.

Once desired acceleration is zero, the requested body attitude places the model COM vertically over the assumed support point to numerical precision. At41 and42.11 s, the floating generalized static force/moment balance residual is approximately4e−13 N or N m. There is no gross static lean-formula or equilibrium-offset error under these assumptions.

The actual body still differs from that attitude: up-vector error is0.870° at41 s and0.734° at42.11 s, leaving millimetre-scale horizontal COM offsets. Holding torques computed at the requested attitude differ by a few millinewton metres from the logged feedforward, which uses the current axes; that comparison does not reveal an erroneous feedforward formula. A valid frozen-plane static reference is not a proof of dynamic stability on the evolving physical terrain.

## The velocity error can be traced to continued rotation

For a current estimated point, the exact kinematic identity is

```
v_actual − v_desired
  = contact_material_velocity
    − omega_world × r
    − J_joint*(joint_rate_actual − joint_rate_reference)
    − belt_axis*(previous_applied_belt − rolling_feedforward_belt)
    − nominal_allocation_residual.
```

The previous held belt input is used because the command at the current row acts over the next5 ms. Contact material velocity at the estimated point is separate from the saved solver-contact slip, which may use different points and normals. All fixed-snapshot identities reproduce within3.2e−17 m/s.

At baseline41 s, the planar components, in mm/s, are:

| Term | X | Y |
|---|---:|---:|
| Actual minus desired body velocity |−2.558|+60.567|
| Body-z yaw rotation contribution |−14.053|−14.753|
| Transverse body rotation contribution |+2.496|−3.955|
| Actual-minus-reference joint-rate contribution |+9.019|+79.329|
| Estimated-point material velocity |−0.020|−0.055|

The nominal allocation residual and belt contribution are zero at this shell-support sample. The body-z angular rate is−0.555 rad/s even though up-vector lean error is less than one degree. Continued joint rotation provides most of the positive-Y motion; heading rotation also couples substantially into translation on the slope. These are kinematic contributions, not independent causal effects. The feedback and contact dynamics jointly generate the measured rates.

Heading position is free: the target carries the actual yaw. Angular-rate feedback still targets zero and weakly damps yaw, so it is inaccurate to call yaw wholly uncontrolled. The saved heading-frame linear velocity errors, body angular velocity errors and native scheduled gain matrices reproduce from the source calculations. No simple current frame-conversion bug was identified.

## A specific scheduling hypothesis remains testable

I49 uses `schedule_on_ground_normal=False`: the gain-bank phase is measured against world vertical, while the lean, support geometry and feedforward use the estimated ground normal. Those phases differ during the late drift.

| Baseline time | Normal-relative minus native phase | Same-error shell-B command change if only the phase changed |
|---|---:|---:|
|38.640 s|−4.03°|−0.0490 N m|
|40.625 s|+6.33°|+0.4316 N m|
|41.000 s|+7.83°|−0.0033 N m|
|42.110 s|+9.49°|−0.0103 N m|

The shaft row remains the same ordinary-pose row. These values are an algebraic sensitivity using the existing gain bank and the same saved error, not a predicted trajectory. A different contact-relative phase can change feedback substantially; a flat-ground gain bank still cannot represent all slope, gravity, yaw and changing-contact effects by one phase parameter.

Previous global normal-relative scheduling trials had worse outcomes and remain negative evidence. This audit initially proposed one matched late-window phase-only test after the current allocation/handoff study. The parent subsequently registered a separate fifth diagnostic: native history through38 s, then only `schedule_on_ground_normal=True` through44 s, with exact pre38 parity required. That prospective experiment is not a result of this saved-data audit, and even improvement in that single case would not support adoption by itself.

## Evidence and limits

[Compact report](compact_stage05/summary.json) consolidates [static/reference results](stage02/report.json) and [phase/frame/velocity results](stage04_phase_frames/report.json). [Snapshot CSV](stage02/snapshots.csv) retains the fixed-time values. Each stage records source and input hashes before arithmetic and verifies unchanged inputs afterward.

The static plane and point are estimates. Saved truth is the final1 ms solver sample at5 ms endpoints. Logged DEM normal is a terrain-reference proxy, not a force-weighted physical collision normal. General multi-contact, contact-patch compliance and unlogged actuator/sensor delays cannot be identified from this calculation. The actual dynamic reason why the native loop permits these angular and joint-rate errors remains to be tested; normal feedback cancellation alone is not a demonstrated bug.

Two implementation failures were preserved: stage01 stopped before case reduction because a local variable shadowed the input loader; stage03 stopped because a control snapshot used a belt while the initial identity helper assumed shell support. Corrected stages02 and04 retain the original metrics and extend the identity to belt terms. Neither failure altered runtime code or generated physics.
