# I-69 — why the simulator produced zero-normal tangential force

27 November 2026. **Completed source audit, installed-library probe and native
solver capture.** The specific shell forces identified in I-68 already exist in
the installed Dantzig solver's applied impulses. Its documented approximation
sets friction bounds from an intermediate normal solution and keeps those
bounds while later solving can change the normals.

[One-page explanation PDF](evidence/publication/plots_stage02/contact_force_cause.pdf)
· [Native numerical report](evidence/bench/native_capture_stage01/report.json)
· [Plain-language interpretation](evidence/bench/native_result_review.md)
· [Controlled probe](evidence/bench/README.md)
· [Previous I-68 interval investigation](../contact-interval-replay/README.md)
· [Current standalone ChatGPT context](../../../CHATGPT_CONTROL_CONTEXT.md).

## What this means in plain language

Friction should depend on how firmly a contact presses against the ground.
This solver first estimates that pressing impulse and uses it to set an allowed
sideways impulse. It then continues solving the coupled contact system. A
contact's final pressing impulse can become zero while its earlier sideways
allowance stays positive. That can leave a sideways impulse on a contact that
has no final pressing impulse.

The force then affects the simulated robot, so the momentum observer correctly
sees its effect. A label based only on positive normal pressure misses that
force route. Fitting another estimator threshold around this behavior could
fit a simulation approximation rather than improve a physically sound contact
model. This does not establish the cause of every trajectory error.

Native gain-scheduled LQR, momentum observer, geometry support locator,
rolling/holding feedforward,120ms incoming-drive preparation, gains and input
limits remain unchanged. No new contact estimator, prediction method or solver
is adopted, and no tracking improvement is claimed.

## Source inspection and installed-library test

The reviewed DART v6.19.5 ODE LCP implementation solves the non-friction rows,
converts tangent coefficients into fixed absolute bounds using that current
normal solution, then continues solving. Those fixed bounds are not recomputed
when later pivots change the normal impulses. The caller's success check does
not independently enforce the final-normal-dependent friction bounds.
[Official implementation](https://github.com/dartsim/dart/blob/ce5c6526035ccc9a22d895670b2579052a2b8903/dart/external/odelcpsolver/lcp.cpp),
[DART caller](https://github.com/dartsim/dart/blob/ce5c6526035ccc9a22d895670b2579052a2b8903/dart/constraint/BoxedLcpConstraintSolver.cpp).

This is distinct from using a friction pyramid instead of a circular cone.
For the pyramid, each tangent component must satisfy
`abs(lambda_t_i) <= mu_i*abs(lambda_n)` at the final solution. When the final
normal impulse is zero, either representation requires zero tangential impulse.

Before new numerical execution, register one positive-definite controlled
problem and link a new standalone executable against the **existing installed
library**, without rebuilding it:

```text
A = [[1, 1.5, 0], [1.5, 4, 0], [0, 0, 2]]
b = [1, 4, 0]      mu = 0.7
initial x = 0      friction rows reference normal row 0
```

Dantzig returns success with `(normal,tangent1,tangent2) = (0,.7,0)` for both
tested early-termination settings. This satisfies its frozen absolute bounds,
but exceeds a final-normal-dependent tangent allowance of zero. The controlled
problem has normalized test units; these numbers are not robot motor limits.
The analytical coupled reference is approximately `(.487805,.341463,0)`.

Default PGS returns failure on this same problem and has substantial remaining
complementarity error. A simple solver switch is therefore not an established
fix. No PGS parameter tuning was performed.

The installed public headers match the official tag and cached conda package.
The installed library's executable section matches the package exactly; its
whole-file hash differs because of recorded conda prefix relocation. The
external ODE solver binary is identical. These source/package checks support
the convention audit; the installed-library witness and native capture provide
the direct behavioral evidence.
[DART provenance and source review](evidence/source_review/dart/REVIEW.md).

## Connect that mechanism to the actual robot recording

Run one new reproduction of I-58 `m100_B`: Gazebo8.10.0/DART6.19.5, Mars
figure-eight, shaft−1rad, yaw45°,44s,5ms control/1ms physics, original command
bounds and gain bank. Replay the full prefix; do not jump directly to an event.

A private diagnostic library copies inputs/outputs of the installed Dantzig/PGS
methods and the actual contact-impulse application, then calls the original
methods unchanged. It does not replace a solve, edit a native plugin or feed
truth into control. Logging is active only for the five physics steps preceding
each of four fixed observer times:7.875,32.110,35.055,35.205s.

The replay reproduces **all59 original precision arrays and all36 I-68 substep
arrays byte-for-byte**, including motion, commands and reported contacts.
Capture140 applied contacts and20 Dantzig calls; all20 report success and PGS
is never called. Every applied impulse matches its parent solution segment
exactly. Reconstructing force from impulse, normal, tangent basis and1ms duration
agrees within3.55e−15N. All140 contacts match the new and old bridge records;
all25 nonzero-force contacts have an unambiguous raw collision-family tag match.

At the observer interval ending35.205s, these are the actual applied shellB
impulses, in N s:

| Physics sample | Original normal row | Tangent rows referencing it | Applied `(normal,t1,t2)` | Final allowed tangents |
|---|---:|---|---|---|
| 35.203s | 1 | 2,3 | `(0, .0201248655, −.0172934405)` | `(0,0)` |
| 35.205s | 1 | 2,3 | `(0, .0104874728, −.0072003873)` | `(0,0)` |

Their reconstructed tangential forces are26.5344N and12.7213N, exactly the
force-bearing shell records that the old positive-pressure labels omitted.
The applied solver solution therefore explains those records before the export
or observer interpretation. It also explains the shellB load that a single
belt-point model cannot produce regardless of where that belt point is placed.

Across the four selected intervals,12 of140 applied contacts exceed a
final-normal-dependent component bound under the frozen numerical screen.
This is a diagnostic sample, not a frequency estimate across all trajectories:

| Fixed interval ending | Applied contacts exceeding final component bounds | Interpretation |
|---|---:|---|
| 7.875s | 2 | Two zero-normal rimA contacts, also present in the good control |
| 32.110s | 0 | Ordinary earlier-shell/later-belt history explains the I-68 discrepancy |
| 35.055s | 5 | Four beltB overshoots and one zero-normal shellB contact |
| 35.205s | 5 | Three beltB overshoots and the two zero-normal shellB contacts above |

These counts do not demonstrate that the approximation caused the large path
deviation. The good control also contains it. The finding is narrower and
stronger: the specific unusual force records originate in applied native
solver impulses, consistent with the documented fixed-bound approximation.

## Checks and preserved issues

- Register the overall question/controlled problem before execution, and freeze
  the private capture sources, six inputs and five binaries before native replay.
  The capture protocol binds74 source files. Native plugins and installed
  libraries retain their original hashes.
- A three-call interposition check reproduces the standalone executable's
  original output exactly before new scene physics. Independent wrapper review
  verifies command/step/observer boundaries.
- Four numerical-core contracts and seven separate capture-adapter contracts
  pass. Preserve the first synthetic exact-zero assertion failure at1.14e−13N;
  its test-only tolerance correction precedes the frozen native arithmetic.
- An independent binary decoder confirms all20 contexts,59/36 parity, applied
  solution segments and force/export association. Solver matrix padding is not
  mathematical data; final checks use original-order inputs and final unpermuted
  impulses, not permuted post-solver auxiliary arrays.
- Private contact-field access is checked against installed headers and by
  reconstructing every recorded force. Fallback-buffer copying is handled;
  no fallback occurs in this native capture. Frictionless tangent bases are
  explicitly unavailable. These focused checks are not the full repo suite.

The source/export audit found no demonstrated stale-copy or force/normal-pairing
cause for these records. It records a separate conditional collision-order sign
hazard, without claiming that it occurred here; a sign flip alone cannot create
the observed zero normal projection.
[Gazebo boundary audit](evidence/source_review/gazebo/README.md).

## Reproduction and next step

The tracked package contains the20 solver problems, applied-contact records,
selected raw contact contexts, reports, original source bindings and a NumPy-only
replay. Full rollouts, complete precision, full substep traces and private compiled
diagnostic binaries remain local under
`results/contact_force_boundary_20261127/`. Uninitialized matrix padding is
omitted from shared numerical inputs and reconstructed as explicit non-data;
all active mathematical entries are preserved.

The compressed packet/context data totals28,319bytes, plus metadata. Two storage
contracts pass. Producer NumPy1.26.4 cached replay reproduces the entire report
and all220 derived arrays exactly. NumPy1.21.5 preserves original inputs and all
discrete decisions, with derived array differences at most1.39e−17 and report
differences3.55e−15. Its strict byte check fails and is preserved; a separate,
scoped portability check accepts bounded derived roundoff without changing any
values. The source and native-run checks above remain separate from this cached
reproduction.

From a repository clone, with Python and NumPy installed:

```bash
python3 docs/iterations/2026-11-27/contact-force-boundary/evidence/publication/portable_tools/replay_capture.py \
  --package docs/iterations/2026-11-27/contact-force-boundary/evidence/publication/portable_inputs \
  --output /tmp/i69-replay-new
```

Use a new output directory. The [portable instructions](evidence/publication/README.md)
provide strict and cross-version verification commands. This makes no solver
calls and does not start a simulator. It stores only selected matching contexts;
unknown parts of its sparse in-memory view are explicitly missing, not measured
zero-force data.

Keep the current controller as the baseline. The next experiment should validate
a contact-solver/model correction against these saved problems: final friction
bounds, complementarity/convergence and force reconstruction must pass before
testing a controlled shell/belt transfer. Then revisit the pressure-based labels
and estimator ranking. Changing only reported forces would leave the simulated
motion and observer inconsistent; a valid correction must address the applied
contact solution.

No corrected solver has been implemented or adopted by I-69. Establishing this
source of misleading contact evidence is the result of this iteration.
