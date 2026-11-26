# I-54: exact gain lookup and the incoming state

This is offline algebra on the four saved I-52 −1.00-rad cases. No controller,
observer, gain design or physics is executed. The original runs and I-53
records remain unchanged.

The cached gain lookup is reconstructed using the original support groups,
phase wrapping, interpolation, normalized load weights and fixed ordinary
shaft row. Every loaded sample in all four cases matches the logged gain
exactly (8,801 per case, maximum absolute difference zero).

## Why the gain jumps

At recorded 32.250 s in R, support share changes from 100% belt B to 100%
shell B in one 5 ms sample. The phase changes by only −0.010359 rad. A
symmetric decomposition separates the support-share change, phase change
and changing state error in the shell-B LQR request:

| Contribution | Change in request |
| --- | ---: |
| Support-share change | −7.429231 N m |
| Lookup-phase change | −0.172447 N m |
| Error change | −0.011469 N m |

These sum to the logged LQR request change. Other command terms explain
the remaining difference in total request. This sharp change does not
require an erroneous contact estimate: both solver endpoint truth and
inferred support change at this sample. The locator selects one patch
with full share when the inferred sphere point crosses a patch band.
Within a fixed support group the gain is interpolated, but a discrete
support change can select a substantially different gain.

This explains the command discontinuity exactly. It does not uniquely
attribute the preceding tilted state, an individual contact impulse or
the full subsequent trajectory to that discontinuity.

## Why simply blending these gains is not an immediate feasible solution

For the same current error and non-LQR terms, let
`K(λ) = (1−λ) K_previous + λ K_current`, with `0 ≤ λ ≤ 1`.
The endpoints request:

| Input | Previous gain at current state | Current gain at current state | Cap |
| --- | ---: | ---: | ---: |
| Shell B torque | −0.112159 N m | −7.718970 N m | 0.600 N m |
| Belt B speed | 2.645521 m/s | 0 m/s | 0.300 m/s |

Shell B requires `λ ≤ 0.064132`; belt B requires `λ ≥ 0.886601`.
There is no intersection. Even the best point on a 10,001-point blend grid
has a worst normalized request of 5.278 times its input cap. The analytic
interval intersection and the independent grid check agree.

This is a frozen-state feasibility calculation, not a proposed controller.
It retains current preparation/fade/feedforward terms and does not model
the changed contact dynamics. An infeasible blend does not rule out a
different nonlinear control action, and a feasible blend would not prove
balance. It shows why smoothing only the gain at this already bad state
is insufficient to place all raw requests inside the existing caps.

At the earlier 31.770 s shell-to-belt transfer, the dominant belt-B LQR
jump is also due to support-share change (+1.608791 m/s); its phase
contribution is approximately −0.000021 m/s. Earlier intervention may
have a different outcome. No trajectory counterfactual is claimed here.

## Evidence

- [Registered input hashes and rules](stage01/registered_before_arithmetic.json).
- [Complete four-case report](stage01/report.json) and [executed source](stage01/analyze_gain.py).
- [Figure](plots_stage01/gain_handoff_feasibility.pdf), [PNG](plots_stage01/gain_handoff_feasibility.png).
- [Arithmetic-stage manifest](stage01/manifest.json) and [plot manifest](plots_stage01/manifest.json).

Execution: `.venv/bin/python results/acceleration_spin_cause_20261126/gain_audit/analyze_gain.py`.
The script refuses an existing stage. Preserve it and select a fresh stage
for any subsequent analysis.
