# I-48: what existing contact records can distinguish

This registered algebraic audit uses existing I-46 arrays only. It runs no observer, controller, simulator or new trajectory. The four conditions cross prior/current **published pose** with raw/radius-normalized **individual solver-point** vectors. First substep, rejected/weak contacts, rims and body contacts are excluded from the primary statistic.

## Source-backed clock and geometry boundary

- `PreUpdate` caches the wheel center and carrier axes; `OnContact` uses that cache for patch admission. [Cache update](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:223), [geometry](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:378), [callback](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:409).
- Admission projects **R times the normalized center-to-contact direction**, not the raw displacement. [Exact band rule](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:392).
- `PostUpdate` publishes a state and reads contact messages. Normal orientation and unmatched-decision fallback still use the cached geometry. [State/contact publication](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:234), [fallback and normal orientation](/home/parrwnago/active-spherical-wheel/gazebo/plugin/SphericalWheelBridge.cc:278).
- The I-46 sampler reads these outputs after each blocking single physics step. Consecutive published poses are therefore available, but the preceding one remains a **pre-step proxy**. Per-contact callback/cache values, producer timestamps and decision-match flags were not serialized. Iteration/clock continuity cannot recover those missing fields. [Sampler](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/support_routing_audit.py:256).

## Registered individual-point result

All rows below require own smooth tag0–3, admitted flag, and normal load above0.05 N. Band tolerance is1e-12 m. Counts refer to contact records, not independent experiments.

| Case | Eligible records | Prior/raw violations | Current/raw violations | Prior/radial-R | Current/radial-R |
| --- | ---: | ---: | ---: | ---: | ---: |
| Mars | 17,107 | 5 | 4 | 0 | 0 |
| Flat | 15,999 | 10 | 10 | 0 | 0 |

Raw-vector violating normal-load fractions are0.031734%/0.024998% for Mars prior/current and0.062504%/0.062504% for flat. Both normalized conditions have zero violating load fraction. The previous-pose proxy alone does not remove these few boundary violations; radial normalization does in this admitted, significantly loaded subset. This is **not complete admission verification**: rejected and weak rows were not scored. It identifies a representation issue in that diagnostic, not an observer correction or a measured time delay.

Exact tables: [conditions.csv](conditions.csv), [worst10_per_case.csv](worst10_per_case.csv), [registered primary report](report.json).

## The original historical equivalent-point discrepancy remains

A separately registered secondary check uses all41 current-pose samples from6.700–6.900 s, with loaded dominant-share≥0.8 and a finite saved global equivalent mean. It has no1ms prior-pose data and makes no pre-step reconstruction.

| Historical sample | Raw dominant-band margin | Radial-R margin |
| --- | ---: | ---: |
| 6.855 s | −14.044 μm | −9.928 μm |
| 6.860 s | −78.173 μm | −74.168 μm |

Eight of41 equivalent-point rows remain outside their dominant ideal band under either representation. Normalization therefore does **not** explain away that historical discrepancy. A load-weighted equivalent mean is not an individual collision point; solver/state conventions, geometry and averaging remain distinct possible contributors. [Secondary report](historical_equivalent_secondary/report.json).

Existing records thus support a useful investigation now: separate a geometric projection definition from a pose convention before blaming filter/computation delay. They do not identify an unknown force timestamp or show that observer error is fixed. Exact callback-state attribution would require additional logging fields in a separately authorized instrumentation experiment; none were added here. No old evidence or PDF was changed.
