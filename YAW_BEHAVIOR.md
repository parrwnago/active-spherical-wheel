# Why the platform can yaw

This is **one free spherical wheel assembly**. Its internal motors can turn the platform through reaction torque and ground-contact forces. There is no separate motor applying a world-frame yaw torque, and there is no vertical swivel bearing isolating the platform's heading from the fork.

The modeled connections are:

```text
free platform + rigid fork       heading ψ
  └─ common shaft hinge, fork y   angle φ, motor torque τc
       └─ carrier + fixed polar belt frames
            ├─ shell A hinge, carrier x   motor torque τA
            └─ shell B hinge, carrier x   motor torque τB
```

The shaft hinge allows relative rotation about its y axis. Its bearings transmit moments about the other axes. A shell motor applies torque to its shell and an opposite torque to the carrier; that reaction can reach the fork and platform through the bearings. Mounting the motors inside the wheel does not remove this reaction.

## At a pole, shell spin produces a yaw reaction

For an upright fork, the shells' common drive axis in world coordinates is

```text
a = (cos φ, 0, −sin φ)
```

Its vertical component is zero at φ = 0 and −1 at φ = π/2. At polar orientation, shell spin is therefore rotation about a vertical axis. Accelerating a shell exchanges vertical angular momentum with the carrier and platform, just as a reaction wheel does.

For a tilted fork with rotation matrix R, the shell-axis vertical component is `R[2,0] cos φ − R[2,2] sin φ`; the common shaft's is `R[2,1]`. Thus a tilted shaft motor can also have a vertical torque component. These are projections of internal torque pairs, not additional world torque actuators.

An isolated MuJoCo check makes the reaction explicit. Only its private test copy has gravity and contacts disabled; default masses, inertias, motor bounds and joint damping remain. A **0.1 N m shell-A pulse for 0.1 s** at the pole turned the platform **2.774°**, ending at **0.957 rad/s** yaw rate. Shell A carried about **−0.00996 kg m²/s** vertical angular momentum, balanced by the other bodies. Total angular momentum error stayed below **1.9 × 10⁻¹⁷ kg m²/s**. No floor contact, heading constraint, or external force was present.

Equal and opposite 0.1 N m shell torques at the pole produced zero platform yaw in the same check: the two rotor momentum changes canceled. This demonstrates that coordinated shell actuation can manage a reaction; it does not create an independent unlimited yaw motor.

## Near the equatorial gap, the rims can produce a ground yaw couple

The two hemispheres can contact the floor at their equatorial rims, approximately **x = ±10 mm** from the wheel center. They remain parts of one spherical wheel. Different tangential forces at those two contact regions can form a yaw couple:

```text
Mz = Σ (rx Fy − ry Fx)
```

In a symmetric upright rim-support check, **τA = +0.1 N m** and **τB = −0.1 N m** for 0.1 s gave almost zero net horizontal ground force, yet a **−0.01679 N m** ground yaw moment and **−0.3288°** platform yaw. The motor stator reactions cancel in this example; the ground-force couple still turns the assembly. The actual contact torques were zero—the moment came from force lever arms.

For an upright, rolling, no-slip approximation with negligible platform roll, subtracting the two rim velocity equations gives

```text
ψ̇ ≈ −Rrim / (2a) · (ωA − ωB)
```

Here `a = 10 mm` and `Rrim ≈ 119.58 mm`, giving a factor of about **5.98**. A small difference in shell speeds can therefore produce a substantial heading rate in this narrow two-rim contact configuration. This kinematic relation describes possible rolling motion; actual acceleration and slip still depend on inertia, torque and friction.

## Pushes, delayed preparation, and the controller

Mouse drags and direction buttons apply a world-frame force at the fork/platform body's center of mass. The simulation explicitly supplies **zero external torque components** for those pushes. Dragging at a different screen position does not add a hidden yaw torque.

Opposite pushes can have zero total linear impulse without canceling angular impulse: their application points, body orientation and ground contacts change between pulses. The relevant moment is `r(t) × F(t)`. Even if angular momentum eventually returns to zero, the accumulated heading angle need not return to its starting value.

The translational rolling map also does not guarantee an unchanged platform heading. The current balance architecture follows the platform's heading when expressing feedback; it does not impose the original heading as a fixed angle target. Heading-rate damping and returning to an original heading are different objectives.

The controller audit identified another issue: accumulated shell **material angle** was being treated as a position error. For the intended axisymmetric shell, that angle is a cyclic coordinate; it is not a platform-heading target. Trying to unwind accumulated rotation after rolling or pre-spinning can keep driving a shell and feed yaw motion. Version 0.3.1 removes the two shell material-position errors while retaining common-shaft phase, shell-speed feedback, and all body velocity terms. The [matched yaw/control comparisons](results/yaw_control/README.md) report balance, saturation, sustained yaw-rate settling and regressions together; this is not a heading lock or a change to the mechanical model.

Pre-spinning an incoming shell also creates its own stator reaction before ground contact. It needs enough lead time to reach the required **contact-point speed**, and that speed includes base motion, common-shaft motion and heading rate. Near a rim, the `ψ̇ × r` contribution at the real offset contact point matters. Matching raw shell RPM alone is insufficient. A constant-velocity contact preview and conservative torque blend can prepare too late during rapid caster acceleration; improving that behavior requires physical timing and torque allocation.

## Contact-model limit and reproducible checks

The current contact dimension is **3**. MuJoCo therefore models normal force and tangential sliding friction, without a direct torsional-friction moment about the normal or rolling resistance. The configuration's torsional and rolling friction coefficients do not activate those effects at dimension 3. See the [official MuJoCo contact-dimension reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom-condim).

The three torque motors have bounded torque but no torque-speed curve or maximum rotor-speed model. The ideal belt speed inputs omit belt motor and rotor dynamics. Simulated spin rates therefore describe this assumed model rather than a hardware speed prediction.

A single ideal polar point contact consequently supplies no direct spin-resisting yaw moment. A real belt footprint may resist twist differently. That affects quantitative yaw behavior; adding friction or pinning heading would change the model and is not the explanation for the controller fix.

Run the three physical mechanism regressions and regenerate their evidence from the project root:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_yaw_mechanics.py
.venv/bin/python results/yaw_control/mechanics/reproduce.py
```

The [saved mechanics results](results/yaw_control/mechanics/summary.json) include body angular momenta, actual rim contact positions and forces, direct contact torques, model configuration and provenance hashes. These small mechanism checks explain physically available yaw paths. They do not establish a recovery envelope, prove heading regulation, or validate real hardware parameters.
