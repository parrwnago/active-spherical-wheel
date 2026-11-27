# I-57 independent proposed loaded-contact validation design

This review reads existing I56 source and logs only. It performs no physics, runtime edits or outcome-dependent tuning. The proposals below are not campaign gates until explicitly frozen in the parent's execution protocol.

## Question and minimum informative experiment

The actual question is whether the frozen estimated-contact map predicts the *difference* in short-horizon physical response when the input changes at the same full physical state. An unpaired next-step motion comparison cannot answer this because the common native response and drift remain in the observation.

At the preselected H states at 4.855 s (shell B) and 35.925 s (belt B), the smallest directly useful probe uses the exact saved clipped-native and governed inputs. Let `u(a) = u_native_clipped + a*(u_governed_saved-u_native_clipped)`. Three fresh replays with constant held `a=0,0.5,1` commands give candidate-minus-native response and an amplitude-linearity check along the actual allocation direction. Every hold is amplitude feasible because the input box is convex. Evaluate both 5 ms and 20 ms endpoints from the same hold. This answers a finite secant question; it is not an infinitesimal local derivative or a closed-loop intervention.

If the purpose is instead a full local Jacobian check, use independently registered column perturbations around one declared base command. Do not conflate that with the actual native-to-governed direction. The native-clipped base matches the allocator's counterfactual anchor; the governed base matches its actual applied command. At 35.925 s the native-clipped command is amplitude feasible but not I56-slew feasible from the saved actual prior input. A native-centered pulse is therefore an explicit actuator-response probe outside the constrained policy, not a realizable constrained-controller action. A governed-centered local probe can instead require the first perturbation to remain inside the final amplitude-and-prior-slew box. These are different questions; label them rather than silently switching bases.

## Physical matching and replay acceptance

Use a fresh native Gazebo process per branch with the exact registered mechanics, terrain, initial state, seed, timestep, plugin and input encoding. Replay the saved actual applied command prefix open loop. At saved state index `i`, replay commands `[0:i]`, excluding `i`: state `i` precedes the command emitted at `i`. Its actual previous input must equal `exact_previous_applied[i]`, normally applied command `i-1`. Start the pulse only after checking bitwise qpos, qvel and simulation time parity at the target, plus exact actual previous input. Require this independently for every branch.

Full replay is preferable to writing qpos/qvel into a simulator because contacts and hidden solver/plugin state depend on history. q/v/clock equality alone is necessary but does not logically certify equality of every hidden state. A duplicate constant-reference branch should reproduce its 5/20 ms endpoints bitwise as a negative control; where exposed, also fingerprint applied-input/force/contact histories immediately before the pulse. Preserve failed-parity runs and stop their response analysis. Do not repair them by fake reset or silently relaxing the parity criterion.

Apply the declared commands directly with the same physical bridge and motor/belt encoding as I56. Do not recompute observer, LQR or preview during the short pulse, because then the input trajectories differ beyond the controlled perturbation. Keep observer contacts, confidence, model parameters, initial mass matrix and initial geometry frozen for evaluating G. Actual physics remains free to evolve.

## Exact perturbations and horizons

Never clip intended plus/minus probes without recording the changed perturbation. If both signs fit the declared command box, a central finite difference compares the actual endpoints using their exact `u_plus-u_minus`; if a cap prevents one sign, use a declared feasible one-sided difference. Use actual command differences in every model calculation. Bounds must apply to all five total inputs. Extra physical slew constraints must be declared separately: amplitude-feasible does not imply I56-rate feasible.

For local probes, a useful predeclared size is 5% of each amplitude cap, with a second 2.5% size on selected informative directions to assess local linearity. A bound-active base requires a feasible inward sign; tiny available steps should be marked uninformative rather than used as unstable divisors. Central and one-sided estimates have different truncation errors and must retain their labels. A finite actual native-to-governed jump, especially the large late torque correction, should not be described as a small derivative probe.

Evaluate G separately at `h=0.005` and `h=0.020` seconds with exactly the same initial records/confidence. Its torque columns scale linearly with h; its ideal belt-speed columns do not. Multiplying the entire 20 ms matrix by one quarter is incorrect. A constant command over the physical interval is the matching input assumption. Twenty milliseconds of actual controller feedback would be a separate experiment.

## Compare all responses in common axes

Generalized base translation is world-frame, but `qvel[3:6]` is fork-frame angular velocity. The physical endpoints have different fork rotations, so raw subtraction of their angular components compares different axes.

For branch endpoints a and b, form `delta_omega_world = R_a*omega_a - R_b*omega_b`, then `delta_omega_initial_fork = R_initial.T*delta_omega_world`. Compare that with `G_h[3:6]*(u_a-u_b)`. Equivalently rotate the model vector into world axes with the *initial* rotation. Compare world planar velocity differences with `G_h[:2]*delta_u`. Relative hinge-rate differences remain scalars in their matching joint coordinates. World yaw response is the world-Z component of the transformed angular difference, not a subtraction of endpoint body-Z components.

The allocator preserves only the two initial-fork transverse rows. Report their observed physical residual explicitly, particularly where the modeled response is nearly zero. A nonzero physical transverse response along a predicted null direction is the most directly relevant failure for its primary-equality premise.

## Proposed coarse screening criteria

Do not mix linear velocity, base angular velocity and hinge rates into one unscaled nine-dimensional error. Report planar translation (2D), initial-fork transverse angular response (2D), world-Z angular response (scalar) and relative hinge-rate responses as separate groups.

The following are useful coarse predeclared simulation screening choices, not theoretical accuracy guarantees or motor specifications:

| Response group | Absolute signal/error floor |
| --- | ---: |
| Planar velocity | 0.0002 m/s |
| Base transverse angular velocity | 0.002 rad/s |
| World yaw angular velocity | 0.002 rad/s |
| Relative hinge rates | 0.02 rad/s |

Raise each floor to at least ten times any duplicate-reference discrepancy, while also recording the original floor and the discrepancy. If duplicate reference is not bitwise identical, explicitly classify that replay/determinism issue before treating small differences as authority evidence. The fixed floors are pragmatic numerical/physical resolution assumptions and should be visible in the registration.

For model prediction p and observed difference o in one group, report absolute error, `||o-p|| / max(||o||, floor)`, observed/predicted magnitude ratio when the prediction is resolved, and directional cosine when both vectors are resolved. A coarse resolved-response pass could require relative vector error <=0.5, directional cosine >=0.8, and observed/predicted magnitude ratio within [0.5,2]. For scalar resolved responses this requires the correct sign. If prediction is below the floor but observation exceeds it, report a false-negligible-authority failure rather than suppressing the row. If both are below the floor, report unresolved/consistent-with-small, not accurate direction or gain. If prediction is resolved but observed response is below the floor, report overpredicted authority. Expose all raw values, not just passes.

For the allocator's predicted transverse null direction, use the absolute floor directly; relative errors against an almost-zero prediction are meaningless. A physical transverse change above the floor challenges the relevant nullspace approximation, even if another larger rotor component is predicted accurately. A selected-column screen does not validate the untested columns, states or arbitrary nullspace combinations.

For the half/full-amplitude probe, report `||observed_full - 2*observed_half||` in each group and its normalized value. A suggested local-linearity screen is <=25% of the full observed magnitude, subject to the same absolute floor. Failure is evidence of finite-perturbation/nonlinear or contact-mode effects, not automatically a coding defect in G.

## Contact changes and causal interpretation

Record true contact labels, load shares, point counts and contact impulses only for offline evaluation at the initial state and throughout each pulse. Use a fixed, documented load threshold to label active support; retain continuous loads to avoid treating threshold noise as a definite transition. Initial labels should match between branches under exact replay. Later labels/loads may differ because of the perturbation.

Never silently drop a branch because contact changes make its prediction poor. Report two views: all predeclared probes, and a labeled subset retaining the same support mode. Mode-changing probes test the held-contact model's operational limitation; same-mode probes more directly test its load-dependent sensitivity approximation. A change in contact is part of the causal physical response, not an uncontrolled external change if it resulted from the deliberately changed input.

Matched differences can establish local causal response to the command perturbation at these two saved states. They cannot establish that the full I56 trajectory would improve under a different controller, validate global stability, certify unilateral/friction feasibility, or prove a unique cause of the entire drift episode. No model parameter should be fitted to these two points and then claimed independently validated on the same points.
