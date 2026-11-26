# I-35 compute-only acceleration feasibility

Read-only inspection on 2026-11-25. No model, solver, timing, or parity runs were
performed for this note, and no controller source was changed. The parent
experiment reported 14–23 s per full-Jacobian, five-refinement N=20 update. At
that rate 400 updates require about 1.6–2.6 hours, before additional horizon cost.
Speedup estimates below remain hypotheses until measured on preserved source.

## First candidate: native repetition of the same physics step

Installed MuJoCo 3.16.0 exposes `mj_step(model, data, nstep=steps)`.
`EstimatedPlanePredictor.transition` currently holds the input fixed and calls
the one-step function in a Python loop (predictive_model.py:184–202). Replacing
only that loop is a small compute change: retain `_assign`, step count, input,
plane, checks, and all finite-difference arithmetic. The [official Python API
documentation](https://mujoco.readthedocs.io/en/stable/python.html#functions)
states that the two forms perform the same computation and that native repetition
avoids reacquiring the GIL between steps. The installed `bindings_test.py:708`
also exercises repeated steps. Neither source establishes this robot's parity;
verify it before adoption. No promised speedup: collision callbacks still run.

## Independent finite-difference nodes

After the serial nominal rollout, each stage's 46 perturbed transitions is
independent. Workers can evaluate whole nodes and return A/B in the original
index order; keep nominal rollout, root chart, epsilon, central differences,
matrix assembly, offsets, costs, QP, and line search unchanged. A worker needs
both its own model and its own data: `_apply_input` writes model surface speeds
(predictive_model.py:91–100), so sharing one model between data objects races.
Copy the already calibrated plane, friction, reset state, flags and options
exactly; do not recalibrate or reconstruct a different model per task.

ThreadPool correctness is plausible with isolated clones, but speedup is
uncertain. The configured analytic clipped floor installs a Python contact
filter (contact.py:22–84; offroad.py:126; config/model.yaml:51). The [official
callback documentation](https://mujoco.readthedocs.io/en/stable/python.html#callbacks)
confirms each Python callback reacquires the GIL. Native stepping releases it,
but frequent filter work can serialize threads. Installed MuJoCo rollout tests
demonstrate exact threaded results on their own models; they do not benchmark or
certify this custom callback and mutable surface-speed model.

If multicores are necessary, a persistent ProcessPool is a stronger candidate:
one private predictor and identical Python callback per process; one whole node
per task; BLAS/OpenMP limited to one thread per worker. Use spawn/forkserver
rather than fork an active multithreaded runtime. Serialize the calibrated
predictor configuration once per update, amortize model construction, and gather
by node index. Benchmark a small worker-count ladder before assuming all 20
available CPUs help. This is a proposed implementation, not tested evidence.

## Adoption gate and exclusions

1. Preserve exact serial source, proposed source, library versions, input arrays,
   private-plane calibration and reproduction command before comparison.
2. For the same saved flat/slope/transition and selected failure inputs, require
   bitwise equality of predicted qpos/qvel, integration state and relevant
   contact outputs for Python-loop versus nstep transitions.
3. Require bitwise A/B/c/nominal equality for serial versus accelerated full-node
   linearizations and unchanged selected command, nonlinear costs and acceptance
   decisions on the saved frozen case. Report any first mismatch; do not mask it
   with a numerical tolerance and call the source equivalent.
4. Measure full update wall time and predictor time at N=20/40/60; distinguish
   startup from steady-state and include worker count and callback configuration.

Do not disable the clipped-floor filter, change contact geometry, coarsen finite
differences, skip nodes, alter optimizer budgets/objectives, or switch callbacks
to C merely to accelerate this comparison. A native callback might eventually
help, but changed dot-product rounding near the patch boundary can change contact
admission and needs a separate validation. MuJoCo's general native rollout pool
also is not a verified drop-in for per-input model surface-speed changes and the
current per-transition reset semantics.
