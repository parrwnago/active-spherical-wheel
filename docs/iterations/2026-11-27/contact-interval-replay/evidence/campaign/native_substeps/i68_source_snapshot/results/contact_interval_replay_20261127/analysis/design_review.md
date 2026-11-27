# I-68 pre-execution scientific review

Status: design advice only. No new rollout or historical numerical analysis was
performed for this note. The authoritative protocol and collector schema are
owned by the parent agent and may make narrower choices before source freeze.

## Interval and force convention

For observer interval `[t0,t1]`, retain each of the five completed physics steps
with its own before/after state and actual duration `h`. Associate the contact
rows returned after `run(1)` with that completed step. Exclude reset microsteps;
do not reuse the previous step's contacts as the next step's force sample.
Commands remain held over the original 5 ms controller interval.

The expected DART convention is contact force = solved contact impulse / step
duration. The corresponding primary reference is

`Q_contact = W(R_mid) sum_j h_j sum_c J_native(q_before_j,p_j,c)^T F_jc / (t1-t0)`.

`W` changes only free-base angular rows 3:6. `R_mid` is exactly the observer's
normalized, hemisphere-corrected sum of endpoint fork quaternions. Free-base
linear rows are already world components; rows 6:9 are scalar joint torques.
The native angular Jacobian rows use the instantaneous fork-body convention.
Thus the operation is average native generalized components, then rotate once.
Averaging world moments directly is a distinct control, not this convention.
Per-step moment origins are each step's fork origin; using Jacobians avoids an
unannounced change to a fixed world moment origin.

DART constructs and solves contact constraints before position integration, so
`q_before` is the primary geometry. Repeat the identical mapping at `q_after` as
a declared geometry-timing sensitivity control, preserving both results. This
control is not an interval of statistical confidence and does not include all
model error.

Primary source checks (implementation convention, not proof of the installed
binary's exact source):

- [DART ContactConstraint.cpp](https://github.com/dartsim/dart/blob/v6.19.0/dart/constraint/ContactConstraint.cpp): `applyImpulse` stores normal and tangential impulses divided by the step duration; its spatial Jacobians use contact points and body transforms at constraint construction.
- [DART World.cpp](https://github.com/dartsim/dart/blob/v6.19.0/dart/simulation/World.cpp): constraint solve precedes position integration.
- [gz-physics SimulationFeatures.cc](https://github.com/gazebosim/gz-physics/blob/gz-physics7/dartsim/src/SimulationFeatures.cc): last-step collision records forward the point and contact force.

Record actual installed package/plugin versions and hashes. Version/source
matching and numerical controls remain necessary; browsing a branch is not a
binary provenance certificate.

## What the reference includes

Map every raw own-collision tag to its mirror body, including fork/platform
contacts; report unknown tags and do not silently discard their force. Preserve
admitted and rejected rows. Summarize any nonzero rejected-row force separately.
The bridge skips own-own internal collision pairs and exports no contact torque.

Call this a **solver-contact-force mapped reference**. It does not independently
measure every possible generalized constraint force. Do not invent a contact
couple from unavailable metadata. Applied external fork wrench, if nonzero, is
mapped separately and added once. Joint actuator torques and modeled damping
belong to the observer's known term and must not be added a second time. Other
joint constraints, unexported wrenches, mirror/engine model mismatch and the
observer's trapezoidal integration error can remain in the comparison residual.

## Separate the two questions

Preserve the exact vector decomposition

`mu - endpoint = (mu - interval_contact_proxy) + (interval_contact_proxy - endpoint)`.

Report vectors and norm cross terms, not an additive attribution of squared
error when the two terms cancel. The observer comparison uses the unfiltered
last-interval `interval_mean`, not the EWMA output. A separately matched EWMA
comparison may describe filtering but cannot replace the raw-interval test.
Numerical identity tolerances are distinct from any physical agreement claim.
The old `.03 + .10 R ||F||` compatibility rule can be reported as a historical
reference; it is not a calibrated measurement uncertainty.

Report per-family/per-body generalized contributions over all five substeps.
An endpoint belt contact can coexist with a preceding shell impulse in the same
5 ms interval. Finding that contribution would support interval routing as an
explanation of inactive endpoint-joint load; it would not establish CPU latency.
If observer-minus-integrated-proxy remains large, report the remainder rather
than attributing it exclusively to momentum-observer filtering or bad geometry.

## Slip and normal controls

Use recorded solver contact points/normals and measured before/after kinematics,
independently of the inferred contact point. For each contact, compute tangential
material velocity, adding the commanded belt surface motion on belt contacts.
The bridge's belt tangent is `speed * (carrierY - n*(n dot carrierY))`; its
magnitude includes the tangent projection factor.

For positive pressure weights `w=max(0,F dot n)`, retain all three quantities:

- norm of the pressure-weighted mean tangential velocity vector (legacy metric);
- `sqrt(sum(w*||v_t||^2)/sum(w))`, which cannot hide opposing slips;
- maximum individual tangential speed among positive-load contacts.

Missing positive load makes slip unknown. The before/after values bound a
sampling sensitivity, not continuous-time slip. Do not label the kinematic
quantity a direct slip sensor or a calibrated friction-confidence measurement.
Compare observer intrinsic-normal hint to pressure-weighted solver normals
separately, including normal dispersion; net-force direction may differ from
surface normal under tangential traction. A discrepancy alone does not identify
which prior or velocity likelihood should be retuned.

## Controls and interpretation

Use the prescribed 32--36 s window and the fixed old event times, with 7.875 s as
a good-case control. Verify replay state/command agreement against the old
native baseline before describing an old event as reproduced. If the new native
path differs, preserve the run, quantify that difference and label the event
comparison conditional; do not select a better matching event after outcomes.

Report coverage, finite/timestamp checks, force-tag coverage, five-step interval
completeness, commanded-wrench contribution and pre/post geometry sensitivity
before interpreting model residuals. Prefer continuous magnitudes and the
registered comparison criteria over a newly chosen physical-noise cutoff. The
experiment diagnoses interval consistency, normal guidance and rolling-model
assumptions; it changes no estimator selection, controller or score.
