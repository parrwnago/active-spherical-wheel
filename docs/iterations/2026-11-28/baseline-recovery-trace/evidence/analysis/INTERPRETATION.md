# I-86 baseline recovery: physical transient plus a larger balance-motion response

Completed unchanged-controller diagnostic; **no controller fix tested or accepted**.
The 106.5 s Gazebo/DART prefix reproduces all 79 original I-82 B060 precision
fields exactly, with only the declared terminal unheld-command flag changed.
The original 5 ms LQR/observer/feedforward pipeline and 0.60 m/s operating belt
limits remain unchanged. One-millisecond capture is evaluation-only.

[Primary report](completed_stage01/report.json) ·
[Plot](completed_stage01/recovery_trace.png) ·
[PDF](completed_stage01/recovery_trace.pdf) ·
[Separate posthoc peak interpretation](posthoc_interpretation_stage01/report.json).

## What happened

The short ground-force interruption is now resolved. The solve ending
105.525 s has zero exported contact force and a whole-robot COM vertical
velocity change of -9.810 mm/s, consistent with one millisecond of gravity.
At 105.526 s vertical ground force is 36.172 N and contact is loaded again.
This establishes a one-step unload/reload in the sampled physics, not a full
five-millisecond airborne interval or a continuous-time clearance measurement.

That small initial event must be separated from the larger subsequent motion.
The maximum spatial velocity residual is **107.139 mm/s at 105.745 s**. At that
time world-X wheel-centre/root velocity is -122.048 mm/s, while whole-robot COM
velocity is -55.847 mm/s. The signed difference is -66.201 mm/s. From 105.530 s
to that peak, the X-velocity changes decompose exactly as:

| Quantity | X velocity change |
| --- | ---: |
| Wheel-centre/root | -106.982 mm/s |
| Whole-robot COM | -38.555 mm/s |
| Root relative to COM contribution | -68.428 mm/s |

This is the kinematic identity `v_root = v_COM - d(COM_offset)/dt`, not a
partition into independently established physical causes. Reorientation and
internal motion matter to what the path controller calls robot velocity. A
single external horizontal impulse is not a complete description of the
root-velocity excursion. The actual whole-robot COM also accelerates; the
event is not merely a coordinate artifact.

The desired balance tilt relaxes from 9.278 to 6.146 degrees over that same
anchor-to-peak interval; actual tilt relaxes from 9.300 to 6.939 degrees. At
the peak, desired-up direction changes at 24.692 degrees/s and actual-up at
30.912 degrees/s. Desired-up rate reaches 37.335 degrees/s earlier, at 105.700 s,
while the supplied body-angular-velocity reference remains zero. This is a
specific reference/feedback consistency question, not proof that the zero-rate
convention is the cause or that a nonzero rate reference will improve recovery.

Ground force plus gravity agrees with independently reconstructed whole-COM
momentum very closely: focus-window 1 ms force-residual norm RMS is 0.000400 N,
maximum 0.002391 N; 5 ms RMS is 0.000376 N. Thus the measured force history
accounts for COM acceleration. `force * dt` remains a sampled impulse
approximation, not an exact internal solver impulse export.

## What the trace rules out in this window

Commands remain unsaturated, rolling joint references remain small, preparation
and belt commands remain zero, and only shell A carries positive pressure.
The separate spatial/motor-family audit finds no second loaded support in the
whole 105.0-106.3 s trace. This is not a reason to expand multi-contact estimation
for this event. Multiple solver contact points in a small patch do not establish
independent mechanical supports.

The momentum observer's interval-force error norm RMS is 0.000296 N. Matching
its fixed filter to physical wrench yields an intrinsic-normal discrepancy of
0.00636 degrees RMS. The subsequently filtered controller normal differs from
current interval pressure direction by 1.373 degrees RMS / 4.499 degrees maximum.
These distinguish filtering observables; they do not establish that observer
failure or normal-filter lag causes the excursion.

The controller stays within 8.237 mm of the intended path and 1.258 degrees of
native desired up. The failed recovery band here is velocity, not positional
departure or loss of balance. This shortened diagnostic does not measure full
133 s sustained-recovery acceptance.

## Narrow next causal test, proposed only

Test whether angular-rate damping against a moving balance target contributes
to the recovery transient. Compare native against a local **rate-only**
diagnostic that retains the exact native desired angle, current observer normal,
holding feedforward, routing, gains and limits, but supplies a causal desired-up
angular-rate reference. Register its window, derivative/frame convention and
bounds before running; preserve inactive-prefix equality and report release
effects. This isolates one convention rather than changing several controller
components together. Same-state command differences are algebra; new matched
physical runs would be needed to establish effects on motion.

This is not an untested claim that reference smoothing solves the problem:
I-43 already tested shaped target angles plus consistent rate and failed broader
tracking/turning gates despite local benefits. No new intervention, retuning,
estimator expansion or default change is performed in I-86. Full cause and a
validated recovery improvement remain open.
