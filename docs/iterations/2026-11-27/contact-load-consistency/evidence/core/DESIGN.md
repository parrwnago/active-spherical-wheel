# I-61 selected-point generalized-load consistency

The new opt-in module changes no live controller, I-60 geometry/ranking,
confidence rule, observer or default. No historical load residual was computed
before the root protocol and final source/test seal. Synthetic tests use a small
known robot model, with no dynamics or solver contacts.

For the same selected I-60 world point p and material-family body, private
kinematics supplies the translational point Jacobian Jp. The single-resultant
hypothesis assigns the entire observed world force F to that point and predicts
`Jp.T @ F`. MuJoCo's free-base angular columns correspond to fork-local angular
velocity. Rotate prediction rows 3:6 by fork R before comparing with the
observer generalized residual, whose first six components are world-frame force
and moment. Rows 6:9 remain three scalar joint loads. Full nine-component
predictions/residuals are diagnostics; the registered veto uses only the extra
three joint rows.

The primary tolerance is absolute plus relative:

`abs(gamma_joint - predicted_joint) <= .03 N m + .10 * R * norm(F)`

The inequalities must hold for all three joints. These are fixed engineering
bounds, not probabilities or calibrated noise. A valid failure vetoes the
proposed override; invalid inputs are unknown and never positively compatible.
No native-relative gate, score retuning, force fitting or low-velocity target is
introduced. Candidate geometry remains strict: known spherical-band/rim points
only. Native-family comparison uses its valid frozen candidate rather than
relabeling an equivalent mean onto a material patch.

`joint_maps(qpos,wrench)` supplies truth-blind affine maps for later oracle
evaluation: centre c, `Jlinear(c)_joint.T F`, and `Jangular(c)_joint.T` per family.
After feature sealing the evaluator can insert a separately supplied raw point p
as `base + map @ cross(p-c,F)`. This diagnostic works even when a load-weighted
point is inside/off the sphere: it does not clamp, project, or reject oracle
points for failing primary candidate geometry. Truth never enters feature
generation. Affine maps are checked against direct per-point Jacobians.

Current instantaneous geometry is compared with a filtered preceding-control-
interval observer estimate. Model/friction errors, other contacts and external
pushes can cause mismatch even for a correct candidate. One resultant force at
one material body is not arbitrary multi-contact recovery. Shaft torque is
largely redundant with net moment for these descendant wheel bodies; the two
material-joint rows supply the main added identity information. A force aligned
with a material joint axis can leave shell versus belt identity unobservable.
Passing this check does not prove correct contact or point geometry.

Synthetic validation has 17 new passes. The combined selected invocation has
45 passes including the same 28 existing I-60/rolling-consistency cases; counts
are not additive. Tests cover rotated-base frames, virtual work, correct
shell/belt joint routing, affine maps at off-sphere evaluation points, force
superposition, shaft redundancy, axis-aligned ambiguity, a multi-contact example
that still passes, frozen I-60 point reconstruction, input/model/live-state
immutability, forbidden solver/dynamics calls and NumPy-only gate replay.

Source snapshots and test logs are under `synthetic_stage01/`, with final
module/test/protocol hashes in `final_source_seal.json`. No empirical estimator,
point-accuracy, controller or trajectory result is claimed by this core alone.
