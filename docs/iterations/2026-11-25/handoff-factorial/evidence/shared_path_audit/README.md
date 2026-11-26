# Secondary audit of the shared feedback path

The large early belt request mainly comes from balance feedback. At 6.825 s,
all four runs correctly identify belt A as the dominant support and request
0.822–0.835 m/s against the existing 0.300 m/s simulation bound. This demand
persists when incoming-drive preparation is masked and when feedback receives
the true support shares.

For the baseline at that sample, the belt A LQR term is +0.8438 m/s:

| Exact feedback group | Contribution (m/s) |
|---|---:|
| Attitude error | +0.5455 |
| Angular velocity error | +0.2021 |
| Linear velocity error | +0.0747 |
| Position error | +0.0192 |
| Joint velocity error | +0.0022 |

Rolling feedforward adds −0.0091 m/s and synchronization adds zero, giving a
0.8347 m/s request clipped to 0.300 m/s. The actual platform tilt is 7.641°
against a 12.031° reference; the full up-vector error is 5.294°.

The support-estimation error remains consequential. At 6.860 s the baseline
routes feedback to shell A while belt A carries the largest physical load.
The belt feedback row becomes zero and shell A requests −0.5686 N m, including
−0.6341 N m LQR feedback and +0.0654 N m holding torque. Its large attitude,
linear-velocity and angular-velocity contributions partly cancel. Oracle arms
avoid that particular feedback-routing error while retaining the same primary
departure time. The error can affect the correction without explaining the
whole initial departure.

The next focused question is whether the common desired lean and balance
feedback can be tracked through this transition within the existing drive
limits. A reference-consistency check is also justified: the angular velocity
target is exactly zero throughout the inspected window while the desired
up-vector changes. This observation does not establish that adding an angular
rate reference would improve control; differentiation can amplify estimated
normal changes. A candidate to test is a smooth, rate-limited tilt reference
with a consistent desired tilt rate, preserving the static ground lean and
existing heading behavior. The current reference intentionally carries actual
yaw; a naive yaw derivative would change that behavior. Assess reference shape,
loaded-drive authority and rate tracking before choosing a bounded candidate.
The current data do not justify reducing balance feedback or simply disabling
synchronization. [The candidate note](candidate_note.json) records the direct
saved-rate check and source locations; it makes no controller change or fix claim.

[The figure](stage01/shared_feedback_terms.png) shows reference/actual tilt,
rate context, baseline grouped contributions and all four command traces.
[The numeric report](stage01/report.json) covers the nine fixed requested
timestamps; [selected records](stage01/selected_records.json) retain all five
inputs, full exact error, loads, drive references and component terms.

This is a secondary saved-data audit, declared in [protocol.json](protocol.json)
before calculation. It runs no controller, simulator or dynamics model and
changes no main factorial results or screening decision. Exact gains/errors
reconstruct the saved LQR component after the intended float32 cast at every
inspected sample. The largest normalized component reconstruction residual is
2.30×10⁻⁸. Quaternion tangent errors are distinct from Euler roll/pitch, and
reference-up finite-difference speed is an evaluation signal rather than a
velocity target supplied to the controller. Exact sources and input hashes
are preserved under `stage01/source_snapshot/` and in its provenance.
