# I-77 kinematic reconstruction and independent causal verification

All three original N recordings reproduce their raw rolling references exactly:
9,001 rows each for straight −1.0 and +0.9, and 26,601 rows for figure-eight
−1.0. Inferred loads and equivalent points also reproduce exactly. Reconstruction
uses recorded state, previous applied input, recorded observer wrench and a
contact-disabled StateMirror. It never integrates physics or reruns the observer.

## Straight excursion: weak geometry and a repeated reference burst

At 20.585 s, the native world-XY rolling map has rank two but condition number
571.82 after its native column scaling. Available shaft and shell-B directions
are only 0.389° apart. A 20.04 mm/s requested component along the weak singular
direction produces shaft/shell-B references +26.056/−91.896 rad/s. The least-
squares residual is about 5.4e−16 m/s: the solver matches its requested equations,
but needs very large opposing rates.

Inferred and last-solve positive-pressure families both identify shell B. The
point discrepancy is 0.140 mm. Substituting that same-family pressure-aggregate
point still gives condition 383.97 and references +17.727/−61.675 rad/s. Point
error amplifies the demand; replacing this point does not eliminate the weak
configuration. The +0.9 comparator at the same time has direction separation
86.21°, condition 3.074 and references −0.273/+1.618 rad/s. It is a smaller-
late-deviation comparator, not a globally deviation-free recording.

The largest 5 ms peak is part of a burst. In the fixed 18–22 s window, native
angular references exceed the diagnostic 20 rad/s level at 20.430–20.435,
20.485, 20.525, 20.555–20.560 and 20.585 s: seven rows. The 20 rad/s number
is an existing assumed experimental reference scale, not an active native
constraint or a measured motor rating.

Native allocation stacks per-contact XY equations **without load weighting**.
Its column scales `[5,5,5,.3,.3]` are normalized-reference preferences, not hard
actuator limits. The raw observer-intrinsic point and selected family enter the
map; the filtered normal does not directly enter these XY Jacobian rows.

- [Full reconstructed histories and snapshots](reconstruction_stage01/report.json)
- [Every 5 ms row from 20.560–20.620 and burst groups](findings_stage01/report.json)

## Late figure-eight: a different pattern

Around recovery exit at 99.180 s, condition is 3.300 and shaft/shell-A references
are only −0.113/−0.307 rad/s. Inferred and instantaneous pressure family agree
on shell A, with 0.051 mm point discrepancy. At the later geometric-error
threshold, 100.040 s, condition is 4.046 and references remain small. Across
98.5–102.5 s there are no dropout-held rows; the largest condition (47.07) and
joint reference (4.064 rad/s) occur later, at 101.285 s. This is not the straight
reference-burst pattern at the initiating late growth.

The velocity accounting identity is verified to 1.67e−16 m/s:

```text
v_actual − v_desired = material_residual − base_rotation_contact_velocity
                      − A(actual_drives − reference_drives)
                      − (A reference_drives + v_desired).
```

Actual belt speeds use the previous held applied input. Per-contact terms and
inferred-load-weighted means are retained. At 99.180 s, velocity error is
`[−16.891,+24.397]` mm/s. Its signed terms are material residual
`[−.002,−.023]`, negative base rotation `[+9.108,−10.352]`, negative drive-
tracking effect `[−25.996,+34.772]` mm/s, with negligible allocation residual.
Loaded shell A is actually −1.2475 rad/s against reference −.3072 rad/s.
The controller sends torque; these references are not hard velocity servos.
Body rotation partly cancels the larger drive discrepancy at this instant.

From 98.5 to 100.8 s, Y position error increases 43.714 mm. The corresponding
term integrals are +56.596 mm drive discrepancy, −12.921 mm base rotation,
−.0085 mm material residual and negligible allocation residual; the numerical
position/velocity quadrature difference is .0478 mm. These are **correlated
kinematic accounting terms**, not independent forces or measured causal effects.
They do not prove that adding a body-rotation reference fixes the controller.

This late interval does not exonerate the earlier disturbance around 96 s.
The wider 96–99.2 s chronology has larger material disagreement. The separate
native 1 ms contact trace is needed to compare full signed interval wrench,
contact geometry and the earlier normal/lean reference.

- [Full figure-eight reconstruction](reconstruction_stage02/report.json)
- [Every 5 ms row in 98.5–102.5 s](findings_stage02/report.json)
- [Velocity identity, snapshots and window integrals](velocity_decomposition_stage02/report.json)

## Independent checks of the physical causal branches

These checks analyze the separate physical runs; they do not create new physics.
Native24 matches all 79 shared precision fields against original N through 24 s,
with the explicit shortened-terminal held-mask exception. This is not a full
45 s reproduction. Candidate command prefixes and first changed-state/native-
feedback rows match the original inclusively/exclusively as appropriate:

| Branch | Complete-command prefix | First changed reference/applied row | Changed applied rows |
|---|---:|---:|---:|
| one_pulse | `[0,4117)` | 4117 / 20.585 s | 1 |
| cluster_cap5 | `[0,4083)` | 4083 / 20.415 s | 43 |

Feedback changes are confined to joint-rate reference/error components. Raw
allocation references, history preservation, one observer update and zero
heading commands pass independent checks. The cluster arm is armed for 50 rows;
it changes 43 rows after its state diverges, so it is not merely suppression of
the seven original >20 rad/s samples.

| Metric | Native24 | One pulse | Cluster cap5 |
|---|---:|---:|---:|
| Geometric error at original peak, 22.635 s | 148.934 mm | 149.544 mm | 18.773 mm |
| Geometric error at 24 s | 98.211 mm | 95.334 mm | 7.415 mm |
| Peak geometric error after row 4083 | 148.934 mm | 150.665 mm | 21.979 mm |

This supports a causal role for the selected feedback-reference window in this
straight excursion. The single worst 5 ms pulse is insufficient to explain it.
The short, selected intervention is not an adopted controller or proof of
all-route improvement or goal settling.

- [Native24 independent receipt](causal_precision_verification_stage01/native24/report.json)
- [One-pulse independent receipt](causal_precision_verification_stage01/one_pulse/report.json)
- [Cluster independent receipt](causal_precision_verification_stage01/cluster_cap5/report.json)

## Final eight attitude-reference hold: causal effect, not an adopted fix

The final diagnostic holds only the raw quaternion entering terrain feedback
at its 96.395 s value during `[96.400,96.900)` s, then returns to the native
reference. It stops at 103 s. All 79 precision fields before row 19280 match
the original N recording exactly. State, observer, raw references and gains
also match inclusively through that first active row. Errors, requests and
applied commands first change at 19280; physical state, raw reference and
estimated normal first change on the next row, 19281.

Independent tangent-coordinate reconstruction matches all 20,601 untreated
and used feedback errors, vertical velocity references and LQR terms byte for
byte. The quaternion rule applies for exactly 100 rows. Other error components,
one observer update, zero heading command and no truth/terrain reads pass.

| State time | Native geometric error | Hold geometric error |
|---|---:|---:|
| 96.900 s, primary before release acts | 5.924 mm | 6.898 mm |
| 97.500 s | 28.534 mm | 31.072 mm |
| 102.135 s, original late peak | 316.714 mm | 59.372 mm |
| 103.000 s, diagnostic end | 223.483 mm | 47.485 mm |

At the primary state, up error to the fixed held target changes from 3.296°
to 0.607°. Against the **same original untreated target**, it changes from
0.420° to 2.942°. The emitted target at 96.900 s belongs to the newly released
command, so it is not called the held target for that already measured state.
Later native and branch targets differ because the physical trajectories
differ. Reduced late path departure shows that this reference-response path
has causal influence on this excursion. The primary XY errors are slightly
worse, and the experiment does not establish that ignoring terrain, holding
attitude generally, or a particular smoother is the correct controller change.
Post-release results combine the intervention and return to native references.

- [Independent hold verification and four-target endpoint comparison](attitude_precision_verification_stage02/report.json)
- [Preserved initial strict-byte audit](attitude_precision_verification_stage01/report.json)

## Qualifications and preserved setup history

Pressure-based oracle maps use last-solve family aggregates. They are neither
full contact manifolds nor full 5 ms wrench truth. I-68/I-69 showed that
intra-interval transfers and admitted tangential forces with zero normal
pressure can invalidate a simple pressure-only support label. Oracle allocation
changes here are offline algebra, not physically executed counterfactuals.

Each stage snapshots source and binds inputs before calculation; manifests
verify all produced artifacts. A final Path/string hash-helper error prevented
publication of velocity stage01 after its arithmetic. Its exact source and
partial output remain under that stage and `development/`; only the bookkeeping
helper was corrected for stage02. No physics or controller code was changed.
The first attitude-verification analysis also preserves its strict-byte failure:
using scalar multiplication in place of the native scalar `**2` yaw expression
changed tangent errors by at most 5.56e−16. Restoring that exact operation in
analysis stage02 makes every error and LQR term byte-exact; no physical run was
repeated. Both immutable stages retain their exact analysis source snapshots.
These identities and recorded-field checks are not additional repository-suite
or independent software-function counts.
