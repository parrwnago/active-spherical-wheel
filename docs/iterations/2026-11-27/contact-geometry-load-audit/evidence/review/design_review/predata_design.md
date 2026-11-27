# I-66 pre-data scientific design review

Status: design advice only, before any new historical point-map/residual arithmetic. Root's later frozen protocol is authoritative. No controller, scoring calibration, threshold or physics change is proposed here.

## Isolate geometry before attributing model failure

For the primary comparison choose the physical candidate in the sampled dominant family by a fixed rule: F first if valid/correct-family, otherwise physical Nalt if valid/correct-family. Do not choose by residual magnitude or distance to truth. Use exactly the same current q, family/body assignment, raw interval observer force and target raw joint loads for both predictions; replace only the candidate point by that family's sampled pressure-weighted mean point. Retain raw residual vectors and the old common normalization.

This isolates a point-location substitution conditional on the chosen family. On six harmful overrides F has the wrong family: comparing wrong F directly with a truth-family reference changes both family and point. The primary geometry pair should instead use the correct-family Nalt. Original wrong-F results remain separately informative.

Predeclare the 41 I-65 lost-useful frames, all 294 useful corrections, six harmful overrides, retained useful cases and all appropriate primary rows. Cohorts overlap and must not be added into independent trial counts. Preserve all fixed events and unknown rows. Full-primary mapping is reasonable if the saved-q kinematics generator is cheap; it introduces no new physics.

## What the saved reference means

The logging helper `truth_contact_row` forms each family point as a normal-force-weighted average of admitted contact positions. The normal-load sum includes all positive normal forces; the count increments only above 0.05 N. Thus one resolved contact means one record above that threshold, not necessarily one physical contact. Gazebo's contact retrieval is a last-physics-step snapshot, not the actual full preceding 5 ms contact/force history.

An average of several points can lie inside the spherical surface. Keep the raw mean point as an assigned-body virtual reference, with separate material-band validity; do not radial-project it and call the result truth, or exclude interior means as incorrect contacts. A normal-pressure-weighted point plus the total estimated force need not reproduce distributed tangential moments, even at one instant.

The I-61 oracle residual used filtered observed force. It cannot be directly compared with the new raw-M1 residual. Recompute the reference with raw interval force. In I-61's cached affine maps, `joint_at_centre` is already multiplied by filtered force, whereas `joint_moment_map` is force-independent; preserve that distinction.

## Four fixed models can be informative

1. Correct-family physical candidate point with raw observer force.
2. Sampled family mean point with the same force/current q/family.
3. Full observed net wrench assigned to that one family: point-independent rigid-body pullback through linear/angular Jacobians.
4. Sampled normal-share force split over family mean points, with every assigned force parallel to the observed raw total force.

The third can expose a wrench/couple that one force point cannot represent, but base force/moment and joint loads come from the same observer and model. Its world moment follows the observer's midpoint-frame convention, while geometry is current endpoint geometry. It is not independent true-wrench validation.

The fourth is a truth-assisted equal-direction allocation proxy. Snapshot normal shares do not specify 5 ms force shares or individual tangential directions. Improvement does not uniquely demonstrate multiple-contact causality; failure does not refute multiple contact. Report all models side by side, without choosing a winner after outcomes.

## Recommended reporting criteria

Use identical valid paired frames for geometric RMS/p95 comparisons, with full cohort denominators and unknown coverage shown separately. A material geometry contribution can use the prior 10% vector-RMS reduction convention plus p95/per-phase RMS nonincrease, declared before arithmetic. Report every residual in N m, residual-change signs, point distances, and compatibility with the unchanged I-61 engineering bound. A lower model residual alone is not evidence of better contact discrimination.

For the frozen I-65 counterfactual, change only the truth-correct candidate's point, keep the wrong alternative and held-out mean/covariance fixed, and report decision changes on the 294 useful/six harmful distinct-family comparisons. This is oracle-assisted evaluation, not an estimator. In the 6,602 duplicate same-family comparisons, changing only F can manufacture a preference between equivalent alternatives; do not call that family-discrimination improvement. Do not automatically apply the old single-point calibration to the different full-wrench/split residual models.

Remaining error at the sampled point demonstrates that this geometry substitution is insufficient under the recorded assumptions. It does not separate within-interval geometry/force covariance, weaker family loads, friction distribution, observer/model bias or inaccurate estimated force, and does not establish a unique cause of trajectory drift.
