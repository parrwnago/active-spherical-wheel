# I-54 — acceleration and sustained platform spin

2026-11-26. **Saved-data investigation. No controller defaults changed.**

The user observes spinning after the −1.00-rad reference-guard (R) burst in
I-52 and asks for a cause before more controller changes. This follow-up
separates the earlier trajectory history, the discrete feedback change,
the angular-momentum exchange and the subsequent heading behavior.

The supported picture is a rapidly moving balance target, an abrupt
support-dependent feedback request, strong shell/platform momentum exchange,
and continued heading rotation under a free-heading policy. The first three
are quantified below; their exact individual physical contributions to the
complete departure are not yet isolated.

The baseline is the completed I-52 four-arm, 44 s Mars figure-eight at initial
common-shaft phase −1.00 rad and yaw 45°. B is unchanged, R applies the exact
I-51 joint-reference guard, N changes only the gain-bank phase to the estimated
normal, and NR combines them. All use observer-derived contact information,
exact simulated body/joint state and the same assumed 0.30 m/s belt cap.
The explicit original resolver selects N's recorded technical retry.

Original overview playback 10 s is recorded 32 s; commands at a logged time
act on the next 5 ms interval. The full raw studies and the previous I-53
records remain unchanged.

## The spinning is platform motion and starts before the largest burst

World angular velocity is reconstructed by rotating the saved free-joint
body-frame angular velocity into the world. Heading is independently obtained
from the horizontal projection of the platform's x axis, with its analytic
heading derivative and projection validity checked. Shell/shaft material
angles are kept separate. No heading singularity occurs in the interval below.

| Recorded time | Platform world-vertical angular velocity | Observation |
| --- | ---: | --- |
| 32.245 s | 3.739 rad/s | Already spinning; old belt command still applied |
| 32.250 s | 3.825 rad/s | Belt-B → shell-B feedback change sampled |
| 32.275 s | 5.992 rad/s | Both shells have accelerated strongly |
| 32.290 s | **6.635 rad/s** | Peak world-vertical rate, about 380°/s |
| 32.305 s | 5.957 rad/s | Peak planar speed, 1.025 m/s |
| 44.000 s | 0.297 rad/s | Rotation has slowed but remains present |

From recorded 32.250 to 44 s, R turns monotonically by **2.748 full turns**.
At 36 s its planar speed is only about 0.004 m/s while vertical angular
velocity remains 1.736 rad/s. At 42 s, after the timed path has stopped,
vertical angular velocity is still about 1.161 rad/s.

| Arm | Net platform heading turns, 32.25–44 s | Absolute heading travel | Peak absolute world-vertical rate |
| --- | ---: | ---: | ---: |
| B | −0.751 | 1.428 turns | 2.443 rad/s |
| R | +2.748 | 2.748 turns | 6.635 rad/s |
| N | +0.017 | 0.122 turns | 0.525 rad/s |
| NR | +1.898 | 1.898 turns | 1.726 rad/s |

These synchronized windows do not put the four arms in the same state.
All have free heading; different excitation and preceding histories account
for different subsequent motions. Free heading alone does not explain why R
spins more than N.

## The discrete command change is reconstructed exactly

The original cached gain lookup is reproduced at every loaded sample in all
four arms: 8,801 rows per case, zero maximum absolute gain difference.
At 32.250 s, estimated load share changes from 100% belt B to 100% shell B.
The solver endpoint label changes at the same sample. Thus this command jump
can occur with correct support classification; an estimator delay is not
required to explain it.

A symmetric decomposition of the shell-B LQR request change gives:

| Source of change | Shell-B request change |
| --- | ---: |
| Support-share change | **−7.429 N m** |
| Lookup-phase change | −0.172 N m |
| Changing feedback error | −0.011 N m |

The total shell-B request changes approximately −0.092 → −7.719 N m and
clips at −0.600 N m. Shell A also clips at −0.600 N m. The entering lean
error is already about 13.17°. The guard's instantaneous modification at
this state softens shell-B demand by about +0.098 N m; both versions still
clip. This does not remove the guard's effect on the preceding history.

The interval ending at 32.250 s was driven by the previous command, including
the capped 0.30 m/s belt. It already contains a +0.086 rad/s yaw-rate change
and a roughly 0.045 m/s planar velocity increment. The first interval under
the new saturated shell commands, 32.250–32.255 s, adds about +0.491 rad/s
to world-vertical angular velocity. New endpoint commands cannot cause the
velocity change that has already ended at that endpoint.

At this bad entering state, blending the old and new gain alone is not enough
to put all raw requests inside the existing command caps. With the current
error and other terms held fixed, shell B requires blend fraction λ ≤ 0.064,
whereas belt B requires λ ≥ 0.887. No fraction satisfies both. This is
frozen-state algebra, not a tested blending policy or a proof that another
nonlinear control action is infeasible.

## Why shell acceleration can spin the platform

The shell hinges share the carrier x axis. At the handoff its world-vertical
component is approximately +0.961: the axes are nearly vertical. Applying
negative torque to both shells therefore accelerates them with negative
vertical angular momentum and applies the opposite reaction to their parent
assembly. The same physical motors used for balance/rolling can generate a
large platform yaw response in this configuration.

The response subsequently reverses: by 32.305 s, shell torques are about
+0.582/+0.600 N m, the projected reaction changes sign, and yaw rate falls
rapidly. This is a driven, oscillatory momentum exchange followed by slower
continuing rotation. It is not evidence of a constant-rate undamped coast.

Offline momentum reconstruction uses the saved poses and velocities plus the
model's body masses, COM offsets and inertia tensors. The corresponding SDF
and mirror tensors agree; body angular velocities are checked against an
independent kinematic chain. No dynamics is integrated to obtain these values.
About the total-system COM, over **32.250–32.275 s**:

| Body group | Change in vertical angular momentum |
| --- | ---: |
| Fork/platform and carrier | **+0.022915 kg m²/s** |
| Both shells | **−0.021530 kg m²/s** |
| Whole assembly | **+0.001385 kg m²/s** |

The large opposing changes support a strong internal reaction-wheel exchange.
Motor torques cancel in a whole-system external-torque balance. The much
smaller total change is the state-derived net external angular impulse under
this verified rigid-body model; it is not a separately measured 1 ms contact
impulse history. Motor reactions, joint constraints, passive damping and
contact forces can all affect individual body groups, so the accounting does
not uniquely attribute every body-momentum increment to one motor.

## Why the heading keeps rotating

The selected feedback constructs its attitude target using the current
heading. There is no fixed heading-angle restoration target in these trials.
`heading_gain=0` and `heading_reference='none'`. The separate `_heading_torque`
function returns before applying its `heading_damping=0.06` term when the gain
is zero. All four saved heading-command arrays are exactly zero.

Native LQR body-angular-velocity feedback still exists and its angular
velocity reference is zero. Therefore it would be incorrect to say there is
no yaw feedback at all. However, a zero separate heading command and a
heading-following attitude target do not guarantee yaw-rate settling while
balance and trajectory corrections continue. At 32.305 s the angular-velocity
state group contributes about +9.893 N m to shell-B demand, helping reverse
the total request. That group includes all three body angular rates; its
whole contribution cannot be labeled a pure yaw-damping term.

## The bad entering state has a longer history

B and R are identical up to recorded 10.930 s. At that sample the guard first
changes reference and applied command on the same state; state/observer/normal
histories first differ at 10.935 s. The first gain difference follows at
11.285 s. Thus the two arms have had different histories for over 21 s before
the violent handoff.

The first change occurs when a now-unloaded shell-A native rate reference
returns to zero but the slew guard temporarily retains +1.705 rad/s.
This changes feedback/applied torque even though both reference alternatives
have essentially zero current belt-contact rolling residual. A reference can
be kinematically irrelevant at the current contact and still affect coupled
feedback outputs. A small initial perturbation does not identify the unique
cause of a much later large departure.

There are 55 modified R holds in 13 clusters. The local cluster starts at
31.745 s, after lean error has already crossed 5° at 31.685 s and 8° at
31.725 s. The preceding modified sample was 26.455 s. At 31.745 s lean is
already about 9.85°. Therefore the onset of this local tilt growth precedes
the local reference guard's activation. The large cluster also modifies the
common-shaft reference and can change shaft feedback; it is not only a shell
speed modification.

### A moving attitude target is a major part of the entering error

The large relative lean error must be distinguished from absolute tilt from
world vertical. In the fixed 31.400–31.745 s window, R's relative lean error
grows **1.715 → 9.848°**, while its absolute tilt **decreases 6.042 → 2.709°**.
The desired up direction moves 15.621° net (15.643° accumulated arc), while
the actual up direction moves 7.577° net. At 31.700 s, desired-up motion is
about 150°/s versus 78°/s actual over the preceding 5 ms. This is a rapidly
moving target with tracking lag, not merely a growing fall from vertical.
At 32.250 s, the 13.166° relative error corresponds to only 6.171° absolute
tilt.

The estimated normal moves 16.031° over that entering window, while requested
XY acceleration remains small and smooth. The logged DEM tangent proxy
moves 17.826° in R; it is a terrain-model proxy, not the individual DART
collision normal. These records support a normal-driven reference swing,
without uniquely separating real terrain change from estimation/filtering
and contact-geometry effects. The angular-velocity reference remains zero
while the desired attitude changes. That reference-consistency issue is a
specific candidate for a controlled diagnostic, not an already proven fix.

R still selects shell B over this window. Around 31.685 s the lookup enters
interpolation between cached shell-B phases −1.9 and −1.2 rad, a 0.7-rad gap
without intermediate shell-B designs. This is interpolation, not endpoint
clamping. The flat gain-bank approximation is distinct from the later hard
support-share change. Prepared belt B is already at its 0.30 m/s cap from
31.650 s; the three requested motor torques do not hit their caps in this
entering window. The belt preparation, normal-driven target and coupled
motion all precede the local guard, but their separate physical effects
are not identified by the saved data.

## What is established and what remains open

The support-share discontinuity explains the command jump exactly. The
saved-state momentum accounting supports a substantial momentum-exchange
contribution, consistent with the known shell-motor reaction direction.
The platform's continued rotation is
measured, and the absence of separate heading control is verified in source
and logs. Those are more specific findings than an unexplained visual twitch.

The exact contributions to the earlier relative attitude error and the
precise contribution of every contact impulse remain open. Endpoint contact
forces are only the
last 1 ms sample in each 5 ms hold; the full intra-hold contact-force history
is unavailable. Differences between completed arms are not matched-state
causal interventions. No claim that observer computation latency, one guard
sample or the instantaneous reference clamp uniquely causes the complete
trajectory is supported by these records.

Before changing the live controller, the next causal diagnostic should use
an identical replayed prefix, verify matching state/controller history, then
change one identified feedback/drive action over a short registered window.
It should retain the native reference branch and log every 1 ms contact
impulse, rotor/platform momentum and actual applied input. A restored pose
alone is insufficient to guarantee matching solver/controller history.
This is a proposed diagnostic, not a completed experiment or an adopted fix.

## Evidence and validation

The compact evidence below preserves registration, executed post-processing
sources, reports, arithmetic checks and figures. Full raw arrays remain in
the original I-52 local campaign. Source hashes bind the exact tested runs.
Arithmetic/clock/kinematic checks are evidence QA, not additional robot trials
or an additional full repository test-suite run. No new trajectory campaign
or controller/default change was made for I-54.

The mechanics audit retained its first draft and corrects a snapshot-axis
array-alias error in a separate stage; it did not affect the computed angular
momenta or interval differences. Development issues and corrections remain
visible in the evidence record.
The initial entering-window plot and its first readability revision are
retained locally. A further annotation-only stage moves text clear of the
legend; it changes no arithmetic or recorded data. The links below use that
separately registered final layout.

- [Moving balance target](evidence/history_audit/plots_stage03/actual_vs_desired_up.pdf)
  · [image](evidence/history_audit/plots_stage03/actual_vs_desired_up.png)
  · [entering-window numbers](evidence/history_audit/compact_stage02_entering/summary.json).
- [Platform heading and joint motion](evidence/spin_audit/compact_stage02/platform_and_material_spin.pdf)
  · [image](evidence/spin_audit/compact_stage02/platform_and_material_spin.png)
  · [detailed onset](evidence/spin_audit/compact_stage02/R_spin_onset_detail.pdf)
  · [compact spin report](evidence/spin_audit/compact_stage02/summary.json).
- [Momentum exchange](evidence/mechanics_audit/stage02_corrected/momentum_exchange.pdf)
  · [image](evidence/mechanics_audit/stage02_corrected/momentum_exchange.png)
  · [model-derived budget](evidence/mechanics_audit/summary.json)
  · [corrected detailed report](evidence/mechanics_audit/stage02_corrected/report.json).
- [Gain jump and frozen-state feasibility](evidence/gain_audit/plots_stage01/gain_handoff_feasibility.pdf)
  · [gain report](evidence/gain_audit/stage01/report.json)
  · [full guard/history summary](evidence/history_audit/compact_stage01/summary.json).
- [Registration](evidence/registration.json)
  · [preserved mechanics correction](evidence/mechanics_audit/stage01_preserved/self_review.json)
  · [final record checks](evidence/review/verification.json)
  · [portable evidence inventory](evidence/manifest.json).

These copies preserve their original bytes. Archived scripts retain their
original `results/` path semantics and raw-input requirements; they are source
evidence, not a new fresh-clone analysis launcher. The portable manifest lists
exactly what is copied here. Copied original-stage manifests can also inventory
larger local files that are not in this compact snapshot.
