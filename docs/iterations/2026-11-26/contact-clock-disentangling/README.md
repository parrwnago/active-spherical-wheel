# I-48 — distinguish contact clocks and radius conventions

2026-11-26. **Read-only saved-data diagnostic; no controller change or new simulation.**

The user asked whether applied clipping was external to the controller and
whether earlier runs could investigate contact disagreement. The code already
clips the complete request inside `OffroadController.command`, and again in
`Plant.apply_input`. The unresolved design question is constraint-consistent
reference/feedback behavior before that clamp. The Riccati effort cost does
not impose a total-command inequality.

For contact investigation, the prior records are useful. They do not record
per-command computation/transport delay; the controller and Gazebo execute in
lockstep. This follow-up tests two observable comparison conventions rather
than attributing disagreement to unrecorded computational latency.

## Registered primary diagnostic

The C++ plugin caches center/carrier axes in PreUpdate; OnContact applies
its admission rule using that geometry. PostUpdate publishes updated pose and
contact records. Importantly, admission normalizes each center-to-contact
vector to radius R before testing the patch's signed axial band.

Using the existing I-46 raw 1 ms arrays, compare:

- Current saved post-step pose versus preceding saved pose as a pre-step proxy.
- Raw individual solver-point vector versus radial projection to radius R.

Keep each point, tag and load fixed across all four conditions. Eligible
records have admitted smooth-surface tags 0–3 and compressive normal load above
0.05 N. Exclude the first substep; check clock, iteration and reset continuity.
Use the registered 1e−12 m numerical band tolerance. Rim/body contacts and
rejected/weak records are not part of this loaded-support statistic. This is
not a complete reproduction of all admission decisions.

## Observed result

| Comparison | Mars outside-band records / 17,107 | Flat / 15,999 |
| --- | ---: | ---: |
| Post-step pose, raw point | 4 | 10 |
| Prior-pose proxy, raw point | 5 | 10 |
| Post-step pose, radial-R point | 0 | 0 |
| Prior-pose proxy, radial-R point | 0 | 0 |

Changing the pose proxy alone does not remove the discrepancies. Applying the
plugin's radius convention removes these individual-point band violations in
both cases. Producer and independent NumPy implementations agree on all counts
and load-weighted fractions. No observer or filter was rerun.

This identifies a **comparison-convention sensitivity**, not corrected
observer routing or improved driving. The earlier load-weighted equivalent
point is not necessarily one physical contact and cannot be substituted for
an individual solver point. A prior published pose is only a proxy for the
unlogged callback/cache state. The original trajectories/forces remain fixed;
these alternative projections do not establish what a changed rule would do.

## Separately registered historical follow-up

The original figure-eight has 5 ms state/equivalent-point history, not these
1 ms individual-contact records. A secondary check keeps its 41 samples from
6.700–6.900 s and compares raw versus radial-R projection of the true equivalent
mean at the current pose. All 41 meet the fixed eligibility rule. Both
conventions have 8 outside-band cases. At 6.855 s the margin changes
−14.044→−9.928 micrometers; at 6.860 s, −78.173→−74.168 micrometers.

Therefore the normalization result on the new individual-contact data does
**not** explain away the original historical equivalent-point discrepancy.
The latter lacks 1 ms individual points and callback/prior-pose state; it
cannot be treated as the same geometry test. Remaining disagreement is real
under that recorded convention, but a unique latency/estimator/model cause is
not identified. The secondary question was registered before its arithmetic
and does not replace the primary eligibility, thresholds or outcome.

## Consequence for the investigation

We can investigate filtering, geometry and timestamp conventions with old
logs. We cannot measure an unrecorded per-command CPU/transport delay from
aggregate wall time. Validate the evaluator's time/geometry convention before
labeling every support disagreement “estimation lag.” Existing I-46 shadow
results and the current selected filter/controller remain unchanged.

Turning remains a closed-loop performance question: reference design, feedback,
saturation and contact dynamics can interact. Delayed reversal and geometric
overshoot need separate metrics; this task does not select a new turning fix.

Raw primary evidence:
[registered protocol](../../../../results/contact_clock_disentangling_20261126/source_audit/protocol_before_execution.json),
[producer report](../../../../results/contact_clock_disentangling_20261126/source_audit/report.json),
[independent calculation](../../../../results/contact_clock_disentangling_20261126/root_spot_check.json).
Compact evidence is preserved beside this record.
