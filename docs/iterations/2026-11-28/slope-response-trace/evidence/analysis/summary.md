# I-81 analysis — the departure is not explained by gravity alone

One new unchanged-native Gazebo/DART diagnostic finishes at 102 s of the original
133 s figure-eight route. The runner verifies all 79 precision fields byte for
byte against original I-76 through 20,401 rows, with its explicit shortened
terminal-held-mask exception. No controller/observer/physics parameter changes.
This is diagnostic completion, not full-route acceptance or a tested fix.

## Main finding: actual belt contact adds downhill force during large pulses

The newly recorded 1 ms contacts allow completed 5 ms intervals to be aligned
with their **previous actually held command** and next observer update.

| Completed interval end | Contact force along downhill | Model gravity along downhill | Sampled sum | Contact carrying the positive pulse |
| --- | ---: | ---: | ---: | --- |
| 100.835 s | −1.991 N | +3.234 N | +1.243 N | Mixed shell/belt; contact partly opposes gravity |
| 100.900 s | **+10.598 N** | +3.234 N | **+13.833 N** | Belt A only |
| 101.000 s | **+7.330 N** | +2.966 N | **+10.296 N** | Belt A +9.686 N, shell A −2.356 N |

At the 100.900 s pulse the prior held command, issued at 100.895 s, prescribes
belt A speed −0.300 m/s after clipping a −0.889 m/s request. Incoming preparation
adds **zero** during this hold: the loaded LQR/rolling branch supplies the belt
request. Belt LQR terms include roll/pitch −0.452, body-linear velocity −0.198,
body-angular velocity −0.164 and XY-position −0.078 m/s. These are algebraic
components of the same command, not independent causal contributions.

The held belt surface moves approximately [−.169,−.247,−.026] m/s in world
coordinates while the measured ground reaction acts downhill. Motor-command
sign alone cannot be interpreted as uphill/downhill motion. At 101.000 s the
same pattern occurs with a −0.918 m/s request clipped to −.300 and no preparation
delta. Thus some large acceleration pulses occur while **belt A really carries
load and the inference also identifies belt A**. Misassignment alone cannot
explain those pulses.

This rejects a gravity-only description of these sampled force pulses. It does
not yet isolate whether the drive's feedback action, passive/contact dynamics,
or their interaction is responsible. No motor/gravity work partition or causal
controller intervention is performed. The next targeted causal comparison
should examine loaded-belt feedback/gain response, rather than assume incoming
preparation or a bad geometric preview is the initiating cause.

## Force and COM checks

The whole-robot COM velocity is computed from the captured model masses and
local inertial positions, including body/joint rotation. Wheel-centre velocity
is not treated as whole-COM velocity. Across all 260 intervals:

- Mean exported contact force + model gravity versus `mass * ΔCOM_velocity/5ms`
  has residual RMS **0.01971 N**, maximum **0.17146 N**, against model weight
  18.2466 N. All/admitted/native-contact-filter force branches agree here.
- The world-vector impulse residual over the full 1.3 s trace is approximately
  [0.000079,0.000194,0.000041] N s.
- Integrating force along the **changing** downhill tangent gives contact
  −3.180 N s and gravity +3.635 N s. These descriptive integrals retain strong
  opposing-contact intervals as well as downhill pulses; they are not a fixed
  world momentum decomposition.

The exported forces are sampled forces, not a solver-impulse oracle. Independent
private MuJoCo **kinematics only** reproduce analytic COM positions/velocities
within 5.33e−15 m / 2.50e−15 m/s, and interval momentum rates within 1.45e−12 N.
Signed physical/filtered wrench, commands and all family labels agree between
the two implementations. No private dynamics integration is used.

## Observer, contact timing and normal response

Over the full 100.5–101.8 s trace, observer interval-force RMS discrepancy is
0.02010 N; moment RMS is 0.004641 N m. Same-filter physical-wrench direction
versus observer direction has RMS 0.08265°, maximum 0.40917°. The diagnostic
filter is seeded once from native filtered wrench at 100.5 s; no fitted delay.

The 181 **interval-end** focus rows have force/moment RMS 0.02407 N / 0.005557
N m and direction RMS 0.09881°. These are a subset of the same 260 intervals,
not additional trials or a conflicting score.

There are 52 inferred-vs-last-solve dominant-family differences and 56 versus
prior-interval mean pressure. Sixteen last-solve differences agree with the
completed-interval family. Temporal alignment therefore explains **some**, not
all, apparent differences. Nine inferred-family rows have no positive pressure
on that family in any of the preceding five physical samples. This is not a
clean observer-error count: support classification, filtered wrench and pressure
dominance are different observables.

At 100.900 s the subsequently filtered controller normal differs from interval
pressure-normal aggregate by only 0.0106°, while the robot is 4.337° from its own
desired up direction. The later 101.550 s event has a 6.406° normal gap and
184.945°/s desired-up rate. A large later normal transition should not be
silently assigned as the cause of the earlier belt-force pulse.

## Original preview is now recorded, but exposure is not force onset

All 261 original preview rows and four targets per row are retained. The fixed
181 focus anchors produce 724 target/future associations, scored at the first
physical 1 ms sample at or after each original selected horizon. Maximum rounding
lag is 0.857 ms; no delay is fitted.

Independent analytic reconstruction of original target margins and prepared
tangent residuals agrees within 3.19e−13 SI. For the 100 nonzero-preparation
targets, constant-velocity versus actual-future-pose margin disagreement on the
**same frozen anchor plane** has RMS 0.175 mm and maximum absolute 0.533 mm.
This checks motion extrapolation in that geometric observable; it is not proof
of the future terrain/contact model.

Forty-three of those 100 targets carry substantial physical pressure at the
selected future sample. Do **not** call the other 57 wrong predictions: original
preview exposure is not a prediction of force onset, and the targets do not
model later closed-loop dynamics. No preview-accuracy acceptance gate is created.

## Explicit posthoc held-belt supplement

Over 181 holds whose **command starts** lie in 100.75–101.65 s, belt A has 148
nonzero applied commands: 38 holds with no belt pressure/contact, 67 partial
occupancy and 43 pressure in all five solves. Of 103 clipped belt-A holds,
7 have no belt contact, 59 partial occupancy and 37 all-five occupancy. Belt B
is not commanded. No-positive-pressure cases also have no native belt contacts
and zero recorded belt force. An unloaded shell torque is different: it can
still affect body balance by reaction and is not called ineffective.

This new supplement was requested after canonical results. Its source/input
registration explicitly labels it posthoc; original prephysics definitions,
sources and closed results remain unchanged.

## Artifacts and development record

- `completed_stage01/report.json`, `interval_rows.json`, `preview_pairs.json`,
  `routing_response.png`, `force_preview_diagnostics.png`, two-page
  `diagnostic_comparison.pdf` and source/input seals.
- `pulse_stage01/report.json` and `held_belt_rows.json` distinguish prior held
  commands, new requests, force families and LQR blocks.
- `independent_closure_stage01/report.json` binds the independent agreement.
- Nine unique pure synthetic core contracts pass; stage02 includes the prior
  eight rather than adding them. Sixteen private random-state kinematic checks
  pass without case arrays or physics. Neither is a full repository suite.
- A missing-space SyntaxError in the new orchestrator and another in the new
  pulse supplement were caught before their new array reductions. Original
  source/compile receipts and corrected stages remain preserved. Frozen runtime,
  core/definitions and physical records were not changed by these draft fixes.

No further numerical edits are planned. A new intervention would require its
own declared comparison and evidence; this analysis has not implemented a cure.
