# I-77 baseline deviation: what the saved chronology establishes

Only original native N is analyzed. No heading helper, parameter tuning or new simulation is used here.

## Two different event patterns

**Straight −1:** the rolling map approaches a weak configuration while cross-track error is still closing. Large alternating reference pulses begin around20.430s; the largest normalized clipping event is20.585s. At that latter row geometric error is11.635mm and still decreasing, while attitude error is already growing. The sustained30mm crossing is21.040s and the148.934mm peak is22.635s. The exact reconstructed map is ill-conditioned even with the saved pressure-point comparator; this supports an allocation/configuration hypothesis, not a claim that every contact estimate is correct or that the maximum pulse alone initiates the event.

[Straight timeline](straight_m100_N_onset.png)

**Eight −1:** the last jointly low-position/low-velocity interval is96.035–96.160s under the declared diagnostic thresholds. Body yaw rate is already about−.30rad/s there. The first reference-up rate≥1deg/s marker is96.150s while the LQR angular-rate reference remains zero; at96.4s desired up-vector rate is84.32deg/s versus actual0.46deg/s, and velocity mismatch reaches82mm/s at96.5s. The rolling allocation is well conditioned and no command clips in96.0–99.2s. Later support transfers and the low-position crossing near99s occur with persistent velocity mismatch. The101.505s maximum clipping event follows, rather than defines the onset of, the late excursion.

[Eight precursor timeline](eight_m100_N_precursor.png)

The99s interval was only a position recovery: at99.175s error is about10mm but XY velocity mismatch is29.7mm/s. It must not be called a recovered physical state or an initiating disturbance. The later30mm crossing is100.040s; peak geometric error is316.714mm at102.135s.

## What is not established

- The local saved DEM normal is a nearest-centre tangent proxy. Its disagreement with a filtered force-derived normal does not prove the observer is wrong or delayed.
- Instantaneous positive-pressure family labels do not represent all forces over the observer's prior5ms interval. Correct labels and a close point do not rule out every geometry/load/normal issue.
- A current command is generated after the recorded state. Peak feedback can be a reaction; motor-reference conditioning, earlier attitude motion, handoff and preparation need separate causal tests.
- LQR component cancellation is algebraic. Total commands also include support/holding and preparation; cancellation alone is not a gain bug.

The next defensible evidence is a verified same-prefix intervention at one component, not an estimator replacement or event-specific retuning. Full timelines/context arrays remain in the local evidence bundle; the compact JSON records source bindings, exact event rules, qualifications and outstanding hypotheses.
