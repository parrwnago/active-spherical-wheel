# Old B060 first-return interruption: posthoc interpretation

The separately registered diagnostic uses only the original I-82 B060 recording
over **105.4–106.1 s**, with the already known I-83 interruption at
105.590–106.000 s. It reads no new I-85 trial outputs and changes no acceptance
criterion. [Exact report](completed_stage01/report.json) and
[plot](completed_stage01/baseline_return_interruption.png) retain the evidence.

The robot remains close to its intended path: maximum distance is **8.237 mm**.
Native-reference up error stays below **1.2583°**, absolute tilt below **9.3029°**.
The failed recovery condition is velocity: spatial residual crosses 20 mm/s at
105.590 s and reaches **107.139 mm/s**. This is not a large positional departure
or a balance-band failure in the fixed window.

All five commands are unsaturated throughout the 0.700 s held window. Both belt
commands are zero. Rolling feedforward, preparation/transition, heading, fade
and clipping increments are identically zero. Joint references stay small:
shaft ≤0.06164 rad/s, shell A ≤0.18855 rad/s, shell B zero. Actual joint speeds
rise much more, so this is not the previously identified enormous rolling-
reference inverse problem.

The last completed **1 ms** solver contact sample has no resolved load at
105.525 s while inferred shell-A support remains 16.57 N. The next recorded
sample, 105.530 s, has shell-A load 17.85 N. These samples do not demonstrate
a full 5 ms contact-free interval or prove an airborne event. Shell A otherwise
agrees between inferred and instantaneous dominant support: only one of 141
rows differs; the inferred family never switches.
The same last-step load record later reaches **36.068 N at 105.725 s**, near
the velocity peak. This is a scalar instantaneous pressure sample, not the
interval external-force vector needed to explain that acceleration.

The largest five-millisecond actual-velocity and requested-command changes
both end at 105.530 s. The new command at that row acts afterward; it cannot
explain the velocity change that has already occurred in that row's state.
The estimated normal then changes from approximately [0.137,0.093,0.986] to
[0.052,0.092,0.994], while platform tilt falls from roughly 9.3° to 5.6°.
The native angular-velocity reference remains zero. Static holding commands
are substantial, while large opposing LQR terms can leave a small net command;
term size alone is not an independent causal effect.

This narrows a possible baseline investigation to the response through a short
contact/normal transient: how the normal-derived lean and holding reference
interact with velocity recovery. It does not establish whether geometry,
physical contact forces, filtered-normal timing or a particular gain initiates
the event. No 1 ms interval-force trace exists here, and no same-state control
intervention is performed. It does not support blaming saturation, a persistent
wrong-drive assignment, preparation or a joint-reference spike for this window.
