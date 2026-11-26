# I-34 frozen search stages

These are private prediction/search calculations from two saved I-32 states,
not new Gazebo executions or online future command playback.

- `provenance.json`, `source_snapshot/`, `anchor_0.4/`, `anchor_2/`,
  `frozen_search_launch.log` (one directory above) and `failure.txt` retain the
  first **per-seed five-refinement** exploration. It computed all chains but
  stopped at an overly strict 2 s comparison with the historically warm QP solver.
  Cold reoptimization differs in cost by 4.96e-5 (relative 8.5e-7), and commands by
  up to 0.00373; this is not exact reproduction of the old solver's cache. Saved
  selected-plan nonlinear cost and all nonlinear states reproduce exactly.
  Do not read the top-level in-progress report as a completed runtime matrix.
- `runtime_aligned/` is a fresh completed **independent global-best continuation**
  exploration. It fixes the baseline check, explicitly reports cold-QP
  differences and preserves every initial seed. Its independent per-chain QP
  caches can change finite-iteration line-search results.
- `runtime_smoke/` calls the actual old/new runtime classes at 0.4 s. Default
  commands, plans and nonlinear costs match bit for bit. The new practical
  native seed lowers cost to 129.08895, with no source-controller mutation.
- `runtime_entrypoints/` is the **authoritative 12-cell matrix** through the
  actual current runtime class (2anchors ×2seedbanks ×1/2/5refinements).
  Each independent call has a fresh QP cache; the2s prior selected MPC sequence
  is restored before the shift. This matches runtime algorithm/order, not the
  historical continuously warmed solver state.
- `derivative_audit/` compares sparse stride 5 and full stride 1 Jacobians at
  the current runtime's selected one-refinement plans. It evaluates actual
  nonlinear directional changes, affine predictions and fixed line-search
  candidates. Central probes may exceed bounds only in the private derivative
  calculation; accepted control candidates remain bounded. This is diagnostic,
  not a controller parameter change or a general Jacobian certificate.

The history-reconstructed native shadow requested/applied commands, observer
wrench and estimated normal match all saved past/current samples exactly at
both anchors. Practical seeds clone that state, predict their own future at 5 ms
and average four commands into each bounded 20 ms interval. Saved future native
commands are never passed to this seed generator.

At the initial anchor, the actual runtime original 1/2/5 costs are 24525.748,
12610.934, 5599.801. Adding the practical native seed gives 129.08895,
127.55563, 99.87995. At2s all remain approximately 58.338715. The proposed live
2-refinement gate (>=10% better than 1 at BOTH anchors) is not met.

The seed policy is a feasible candidate, not proof of closed-loop recovery.
The source-control state is unchanged by private rollouts. Cold solver cache
paths affect near-flat directions; do not claim every independent optimization
returns identical physical commands merely because costs are close.
