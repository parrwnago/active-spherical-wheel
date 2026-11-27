# I-60 independent blind design review

This review was written without reading recorded numerical outcomes. The task
is an offline, measured-velocity contact-hypothesis ranking diagnostic. It does
not modify the live LQR, momentum observer, feedforward or incoming-drive
preparation, and does not run new physics.

## Information boundary and residual

Candidates must come from the robot's current measured pose, its own patch/rim
geometry, and an observer-derived normal with a fixed uncertainty model. Their
point generation must not read solver contact points, labels, normals, terrain,
future poses, future observations, or truth-conditioned candidate subsets.
The evaluator alone may consume solver truth after predictions are complete.

Use actual measured body and articulated rates. For a candidate world point p,
normal n and body b, its rolling residual is the tangential material-point
velocity P(n) [J_b(p) qdot + belt_surface_velocity], with P=I-nn'. For a belt,
the speed is the previous actually held command in this ideal speed-actuated
model, transformed by the current measured geometry. Do not use a desired
velocity, rolling allocation reference, newly requested input, or a speed
computed from the same contact assumption. The existing
`contact_kinematics.rolling_consistency` states this boundary explicitly.

A small residual is consistency with one no-slip hypothesis, not proof of that
contact. It can arise for multiple surfaces, during low motion, or accidentally
under slip. A large residual cannot distinguish a wrong point, slip, normal
error, state error or model error. The previous input is not encoder evidence
for physical belt speed; it is exact applied surface speed only in this model.

## Geometry and normal uncertainty

Each candidate must satisfy its own material patch's band and common spherical
radius; shell rims need explicit geometry, and two-rim alternatives are not
single-contact observations. A candidate at one shell point cannot simply be
relabelled as the other shell or belt. Normal uncertainty should use a frozen
finite cone/grid and an explicit prior or separate ablation. An unconstrained
normal search can rotate the tangent plane to hide velocity and falsely favor
the wrong patch. Record normal offsets, cone-edge optima and generator misses.

Declare scoring units and normalization before outcomes. A velocity residual
scale of 0.02 m/s matches the existing inconsistency diagnostic, but is an
engineering choice, not measured sensor covariance. Any angular prior scale,
candidate cap, tie tolerance and score-margin threshold must be equally fixed.
Record raw residuals as well as a combined normalized score. A normal prior
derived from a current observer does not make that observer an independent
truth source.

## Eligibility, coverage and confidence

Evaluate forced ranking on all predefined evaluator-only single-support states,
including wrong-singleton and no-candidate states as failures. Also report
true-label candidate coverage; ranking quality conditional on coverage must not
hide generation failure. Reuse the existing strict single-resolved-patch-mean
definition or explicitly preregister a changed truth definition. Multi-contact,
weak/unresolved and unloaded truth form separate reported strata.

Use the same baseline support rule and deterministic tie convention as the
recorded controller. Track baseline all-zero/no-support/tied states rather than
silently making a confident argmax label. The ranking itself may use only
estimated/measured inputs, even when evaluator eligibility later selects rows.

Singleton candidates are geometry-forced choices, not velocity-disambiguated
confidence. Confidence requires competing distinct patches, informative measured
motion, sufficiently small absolute residual and a separated runner-up score.
Low motion, high residual and tied/near-tied scores should abstain. Report forced
labels, confident labels, singleton choices, abstentions and their denominators.
Compare confident ranking error to baseline on exactly the same accepted rows;
comparing it to baseline over all rows gives an unfair abstention advantage.
No zero-coverage confidence success may count as useful improvement.

Suggested, not yet parent-adopted, engineering gates: pooled forced identity
error improves at least 10% relative and 0.1 percentage point absolute, with
no individual phase worsening by more than 0.1 percentage point. Confident
coverage is at least 50% of the fixed motion-eligible population; confident
error is at most 1% and no worse than baseline on the identical accepted subset.
The parent must freeze its final choices before outcomes; these suggestions
do not authorize post-outcome threshold selection.

## Clocks and controls

Current geometry/rates, a preceding 5 ms momentum-observer interval and solver
contact records accumulated on a different physics interval are different
quantities. State each recorded timestamp convention. A last-1-ms solver label
against a 5-ms observer is a sampled-clock agreement diagnostic, not an exact
instantaneous contact truth or evidence of computation delay. Any label-shift
sensitivity is separately labeled exploratory and cannot replace primary scores.

Meaningful contracts include independent hand-computed rigid-point velocities,
shell/belt frame signs, previous-held versus newly requested belt input, rigid
base-plus-joint contributions, band/rim validity, normal normalization, invalid
inputs, ambiguous low-motion/ties, singleton confidence rejection, and physical
nonmutation. Include a wrong candidate with a lower velocity residual under
synthetic slip, so the API cannot claim general identity observability.

Before real-data arithmetic, freeze helper/tests, campaign source, protocol,
raw input hashes and generated-model metadata. Preserve failures and corrected
stages. After results, independently recompute coverage, forced/confident label
errors, fixed-window/per-phase metrics and all gate values. Do not call static
rows independent experiments or add them to repository test counts.

Even a passing result would justify another estimation experiment, not a live
controller change or a claim that contact can generally be inferred from
velocity alone. A failure or low coverage is a valid limitation to retain.
