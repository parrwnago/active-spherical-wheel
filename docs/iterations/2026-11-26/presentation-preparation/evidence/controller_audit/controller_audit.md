# Current controller, constraints and defensible answers

Read-only presentation preparation, 2026-11-26. This report uses existing source and saved arrays. No controller, observer, dynamics, sensor or physics execution was performed. The exact saved-array calculation is in `verify_saved_decomposition.py`; inputs are hash-bound in `inputs_before_execution.json`.

## A short explanation suitable for the talk

“Our current controller is a gain-scheduled LQR with contact-aware rolling references and gravity-holding feedforward. We estimate the net ground force and moment from the robot's motion and known previous commands, then infer which part of the wheel supports it. That estimate changes the drive references and feedback routing. A short geometric preview prepares an incoming drive. Everything is currently evaluated in simulation, with exact simulated body state supplied to the controller. The important limitation is that balance corrections and contact geometry can demand more actuation than the assumed limits allow. We have identified and measured that conflict; we have not established a globally stable, constraint-satisfying controller or a validated hardware operating envelope.”

This is gain-scheduled **LQR**, not the rejected MPC/iLQR experiments. Neither the dynamic forecast nor the camera-map preview is feeding this selected controller. The term `hardware` is a software information-boundary mode, not evidence of a physical robot. [Selected settings](../../support_routing_timing_20261126/protocol.json), [live measurement path](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:547).

## What the state and inputs actually are

There are nine mechanical degrees of freedom: six for the floating fork/platform and three relative revolute joints. The stored configuration has ten numbers because orientation uses a quaternion:

`qpos = [x,y,z, qw,qx,qy,qz, shaft_angle, shell_A_angle, shell_B_angle]`.

`qvel = [vx,vy,vz, body_omega_x,body_omega_y,body_omega_z, shaft_rate,shell_A_rate,shell_B_rate]`.

Translation and its velocity are world-frame quantities. Angular velocity in the controller mirror is body-frame; Gazebo's world angular velocity is explicitly rotated into that frame. The shaft rotates the carrier about the fork's local y axis. Each shell rotates relative to the carrier about local x. There are no additional belt rotor states in this model. [Body/joint construction](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/model.py:294), [state transfer](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/gazebo/plant.py:148).

| Command channel | Meaning | Current assumed magnitude limit |
|---|---|---:|
| 0: shaft | Joint torque, Nm | 1.5 Nm |
| 1: shell A | Joint torque, Nm | 0.6 Nm |
| 2: shell B | Joint torque, Nm | 0.6 Nm |
| 3: belt A | Local belt-surface speed, m/s | 0.30 m/s |
| 4: belt B | Local belt-surface speed, m/s | 0.30 m/s |

These are five commanded channels with mixed physical units, not five equivalent torque motors. The belt speed is a contact-surface boundary condition. It is not the robot's translational speed, and it is not a modeled motor-current command. The simulator does not model belt motor load dynamics or gearing. In Gazebo the first three inputs become joint forces, while belt inputs change tangential contact-surface motion. [Configuration](/home/parrwnago/active-spherical-wheel/config/model.yaml:57), [joint forces](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:223), [belt contact boundary](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:439).

## Dynamics and what the observer estimates

A useful mechanical description is

`M(q) v_dot + h(q,v) = S_tau*tau + f_passive(q,v) + sum(J_contact(q)^T*f_contact) + f_disturbance`.

Here `M` is the mass matrix and `h` contains gravity/Coriolis terms. The three joint torques enter through `S_tau`. Contact forces are solved by the simulator and depend on geometry, friction, motion and the belt-surface commands. They are not directly commanded forces. A conceptual unilateral/friction model has nonnegative compressive force and a friction limit; the implemented solver also uses penetration allowance and contact regularization, so this equation is not a claim of exact rigid-contact complementarity. [Model terms](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:296), [contact admission](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:390).

With momentum `p=Mv`, the observer estimates the external generalized force over the preceding 5 ms interval by subtracting modeled momentum changes due to torque, gravity and passive terms. It then low-pass filters that interval estimate with cutoff 100 rad/s. The free-body rows produce a world-frame force and moment about the wheel centre. This uses measured-state surrogates and a dynamics model; it reads no contact solver forces during native control. It does not require a direct acceleration sensor. [Observer equations](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:227), [implemented update](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:330).

On an actual robot, the missing work includes obtaining sufficiently good base velocity/pose, calibrating inertia and the relationship between command and applied torque, and measuring the effects of noise/delay. Model error and an external push can be interpreted as contact force. A net wrench can locate an equivalent support but cannot uniquely recover an arbitrary multi-contact distribution. The locator normally chooses one spherical band; shared equatorial rims have special two-point handling. This is not a general multi-contact posterior. [Observer input caveat](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:255), [locator](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:379).

## What LQR means here

The local plant transition is numerically differentiated over one complete held-command interval to obtain discrete matrices `A,B`. The local state is an **18-component tangent error**, not twenty raw quaternion position/velocity numbers. Its groups are position(3), orientation tangent(3), joint position(3), linear velocity(3), angular velocity(3), joint velocity(3). Orientation error is generated by MuJoCo's quaternion differentiation; it is not simply Euler-angle subtraction. [Error](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/sim.py:97), [linearization](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/control.py:47).

The offline design minimizes an infinite-horizon discrete quadratic cost of normalized state and input deviations. In schematic notation:

`J = sum(x_scaled^T Q x_scaled + u_scaled^T R u_scaled)`.

The state scales are `[.15,.15,.005,.15,.15,.5,1,1,1,1,1,.1,1,1,1,10,10,10]`; inputs are normalized by their assumed plant bounds. `Q` has the diagonal `[2,2,1,30,30,.01,.001,.001,.001,2,2,.1,4,4,.05,.01,.01,.01]`; `R=diag(.3,.3,.3,1,1)`. The reachable subspace is retained and unstable discarded modes cause a design failure. The Riccati equation supplies the local gain. These numbers are tuning choices in scaled coordinates, not physical energy or motor-current weights. [Scaling/cost/design](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/control.py:66).

A bank of local shell/belt gains is interpolated by carrier phase and estimated load shares. The shaft row is replaced by the ordinary operating point's row. During a short estimated dropout, the last loaded gain/load information is retained. Default phase scheduling is measured relative to **world vertical**, because `schedule_on_ground_normal=False`; estimated terrain normal still affects the lean target and holding torque. [Gain interpolation](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/recovery_control.py:272), [runtime feedback](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:768).

Runtime also modifies the error deliberately: it carries measured yaw, follows current ground height, rotates horizontal errors into the heading frame, ignores cyclic shell-angle position errors, and uses a rolling-consistent shaft error. Explicit heading hold is disabled by the selected defaults, although yaw-rate feedback can still contribute. The platform follows a terrain-dependent lean, not necessarily vertical upright. The 28° setting caps desired lean; it is not a proven maximum safe slope or actual body-tilt limit. [Error modifications](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:787), [lean target](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:513).

## Where effort costs and hard constraints differ

The rolling calculation maps desired horizontal velocity to three joint-rate references and two belt-speed feedforward commands using estimated-contact Jacobians. Native least squares uses normalizers `[5,5,5,.3,.3]`. Those are preferences, not hard speed limits. Its damping coefficient is zero in the selected mode. Nearly aligned or vanishing drive directions can therefore produce very large algebraic drive references even at a single contact. [Least squares](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/experiments.py:29), [estimated-record path](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/experiments.py:79).

The pre-synchronization command is

`u_pre = u_equilibrium + u_rolling + u_holding - K*error`.

Then the controller applies incoming-drive blending, optional heading torque, unloaded torque fading, and finally clips the five commands to the magnitude bounds. The actuator limits are enforced on **the total command**, after LQR. They are not inequalities inside the Riccati optimization. [Combination and clipping](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:691), [feedback sum](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:815).

The opt-in bounded allocator separately enforces drive-reference speed/acceleration bounds and adds speed/change penalties. It is inactive in native `least_squares` mode. Its speed cost is a kinematic demand proxy, not electrical effort. Even an enabled reference bound does not bound actual joint speed or the final LQR-plus-feedforward command. The current controller has no general hard constraint on body speed, body acceleration, actual joint speed, command slew, tilt or contact force. [Activation](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:381), [bounded optimization](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/rolling_allocation.py:38).

## Verified saturation example: why 0.835 m/s against a 0.300 m/s bound?

At saved native Mars time **6.825 s**, the exact belt-A request is **0.8347418461 m/s**, clipped to **0.3000000000 m/s**. The desired body velocity is only `(0.03243725,0.03127917) m/s`; these are different quantities.

| Belt-A contribution | m/s |
|---|---:|
| Orientation-tangent feedback | +0.545549684 |
| Angular-velocity feedback | +0.202096840 |
| Linear-velocity feedback | +0.074738058 |
| Position feedback | +0.019179720 |
| Joint-velocity feedback | +0.002239955 |
| Joint-position feedback | approximately 0 |
| Total LQR | **+0.843804258** |
| Rolling feedforward | **−0.009062412** |
| Holding / synchronization / heading / fade / equilibrium, belt A | **0** |
| Total requested / applied | **0.834741846 / 0.300000000** |

Orientation plus angular-rate contributions supply **88.604% of the positive LQR request** at this sample. Thus it is inaccurate to describe this particular spike primarily as a huge rolling-feedforward speed. It is mostly the balance-feedback demand under the existing reference/state/gain, with a small opposing rolling term. The applied command respects its bound; the requested balance correction does not become feasible merely because it is clipped. The desired angular-rate reference is zero at these selected times even though desired lean changes.

The exact sum has zero residual using the previously saved I46 full-precision component logging. That logging is verified against identical I42 baseline states, requests, gains and errors. Across all 36 selected I42 contexts, exact `-K*error` cast to float32 equals the saved LQR component; reconstruction using exact LQR and the remaining float32 terms differs by at most `8.854e-9` of an actuator bound. Full context and units are retained in `saved_decomposition.json`.

This identifies the origin of the command, not a unique physical cause of the trajectory departure. Routing, the reference, momentum/model error, contact geometry, friction and saturation can interact. The four I42 arms all identify belt A at this sample and request approximately 0.822–0.835 m/s; their physical histories are not identical after the interventions. [Previous interpretation](/home/parrwnago/active-spherical-wheel/docs/iterations/2026-11-25/handoff-factorial/README.md:166).

## What earlier changes taught us

These are separately declared, saved comparisons, not a new experiment or a parameter recommendation. Completion is separate from passing all gates; different campaigns have different fresh baselines.

| Saved candidate | Useful result | Regression / limitation | Current decision |
|---|---|---|---|
| D10 uniform rolling damping, coefficient .01 | +.90 figure-eight maximum joint reference 1760.5→4.4 rad/s | −1.00 RMS 51.1→86.4 mm; no run passes every gate | Native LS retained |
| W20 bounded speed/change references, effort .001 | All five obey reference bounds; four improve moving RMS | +.90 RMS 50.5→67.6 mm; actual free-shell speed reaches44.3 rad/s despite20rad/s reference cap | Experimental only |
| W20 stronger fixed effort .005 | +.90 peak190.9→85.3mm, final190.2→19.9mm | Triangle RMS24.4→35.2mm | No universal weight found |
| W20 geometry-adaptive effort .001–.005 | Triangle RMS24.4→23.4mm | −1.00 RMS41.3→77.1mm; speed bounds do not settle coupled balance problem | Experimental only |
| Feasible reference / wider-cap / balance guard | Some direction-specific and route improvements; references remain bounded | Other routes regress; initial +.90 original-path RMS161.62mm vs50.49baseline; projected-reference error can hide goal departure | No promotion |
| I42 true feedback shares / preparation masking | Masked preparation later improves route RMS50.486→41.681mm and passes that route's original gates | All four still depart at6.860s; primary early RMS gain only0.29%; effects are not additive | Keep native selection |
| I43 bounded lean plus consistent rate | Early RMS38.661→29.893mm; belt demand .835→.491m/s | Full raw-target lean error8.709→13.581° and4/6 turns worsen | Prototype off |

Sources: [D10 saved table](/home/parrwnago/active-spherical-wheel/docs/iterations/2026-11-25/diagnostic_comparison.csv:1), [W20 saved table](/home/parrwnago/active-spherical-wheel/docs/iterations/2026-11-25/comparison_initial.json:1), [configuration/direction study](/home/parrwnago/active-spherical-wheel/docs/iterations/2026-11-25/configuration-study/README.md:14), [feasible-reference outcomes](/home/parrwnago/active-spherical-wheel/docs/iterations/2026-11-25/fixed-contact-allocation/README.md:90), [I42](/home/parrwnago/active-spherical-wheel/docs/iterations/2026-11-25/handoff-factorial/README.md:85), [I43](/home/parrwnago/active-spherical-wheel/docs/iterations/2026-11-25/tilt-reference/README.md:120). Machine-readable D10/W20 rows are copied without retuning into `saved_decomposition.json`.

## Short answers to difficult questions

**“Why not add a cost on motor effort?”** LQR already penalizes normalized input magnitude. The earlier rolling allocator studies also penalized normalized speed demand. A soft cost discourages demand; it does not enforce an inequality. Moreover, a speed-demand proxy is not measured torque/current/power. Simply increasing a weight can reduce one spike while increasing lag or balance error elsewhere.

**“Does clipping solve the constraint problem?”** It guarantees command magnitude stays inside the assumed simulator bounds. It does not guarantee the requested motion is achievable, that actual joint speeds stay bounded, or that the robot remains balanced. The feedback law was designed without those online inequalities.

**“Is LQR optimal and stable?”** The local Riccati design is optimal for its specified quadratic cost and linearized model on the modeled reachable subspace. We do not have a global guarantee for this nonlinear robot with changing contacts, interpolated gains, observer error, friction, finite actuation and clipping. Local eigenvalue checks and successful trials do not supply that missing guarantee.

**“Can you enforce all limits with the bounded allocator?”** It enforces that allocator's drive-reference speed/acceleration assumptions. It does not constrain the complete downstream feedback command or physical joint speed. A joint torque loop can exceed a reference speed during transients; contact geometry changes while the robot moves.

**“Why not always use the newest contact estimate?”** A lower-latency signal can also be noisier, premature or wrong near a boundary. The saved I46 shadow comparison did not establish sufficient benefit, and it did not actuate a faster selector. Filter delay alone is not demonstrated as the departure's cause.

**“Are you reading simulator contact truth?”** The selected hardware-mode controller receives exact simulated state and known applied commands, and its state mirror has contacts disabled. Contacts/terrain are inferred from momentum and known geometry. True contacts are read by a separate evaluator. Exact state and an ideal actuation model remain strong simulation assumptions.

**“Do you know the real motor torque without current?”** Not yet. Simulation supplies the declared direct torque input. Hardware needs a calibrated command/torque relation, current/torque estimation or other actuation model; inertia and friction errors also affect the observer. There is no real-robot validation here.

**“Can it handle arbitrary multi-contact terrain?”** No such general claim is supported. The current locator returns an equivalent spherical support with special rim sharing. Many contact distributions can produce the same resultant wrench.

**“Is the camera/map already controlling it?”** No. The active incoming-drive aid is a120ms constant-velocity geometric preview on the currently estimated plane. Dynamic and camera-memory previews were diagnostic shadow pipelines. They are not a deployed SLAM/contact-planning controller. [Active preview](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/synchronization.py:1), [dynamic shadow](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_prediction.py:1).

**“What is the maximum safe speed or slope?”** We have no universal validated operating envelope. The0.30m/s number is a belt-surface command limit;28° caps desired lean. Neither is a guaranteed robot travel speed or safe terrain slope. Tested outcomes are conditional on each specified model, terrain, initial state and reference.

## Scope of the conclusion

The defensible contribution is an inspectable simulation/controller pipeline, a contact-free measurement boundary for the selected controller, and quantitative evidence explaining why simple allocation or handoff changes do not consistently improve the coupled robot. The work does not establish a novel optimal-control theorem, a solved saturation problem, general multi-contact observability, hardware readiness or globally robust terrain traversal.
