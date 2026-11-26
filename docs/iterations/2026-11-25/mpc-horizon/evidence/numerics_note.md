# I-35 numerics review

Read-only review of `predictive_control.py` and the trajectory experiment before
the horizon calculations. No tests, solver calls or dynamics calculations are
performed by this note.

- Inputs are normalized by physical bounds and state costs use fixed scales.
  The condensed mapping still multiplies stage Jacobians across the horizon.
  N=20/40/60 produces 100/200/300 input variables; longer products can amplify
  unstable dynamics and finite-difference/contact errors despite finite nominal
  nonlinear rollouts.
- With the present nonnegative quadratic costs, the condensed Hessian is
  positive semidefinite in exact arithmetic, with positive input/change terms.
  A backend nonconvexity report or small negative computed eigenvalue can be a
  conditioning/roundoff failure. It is not proof that a longer horizon is
  physically infeasible. The nonlinear contact problem remains nonconvex.
- Preserve every trial's condensed-mapping/affine-offset maxima, Hessian norms,
  eigenvalue diagnostics, linear-term magnitude, OSQP status/iterations/
  residuals, bound violation and external box-KKT result. Current code performs
  neither objective rescaling nor PSD projection; do not introduce either
  silently to make a horizon run pass.
- A successful external box-KKT check certifies only numerical stationarity of
  the supplied affine QP at its configured tolerance. It does not establish
  nonlinear convergence, derivative validity, safe contact, or goal recovery.
- The nonlinear candidate bank retains its best bounded finite sequence. A
  successful runtime plan may therefore be the original seed even if all QPs
  fail or all tested line steps worsen cost. Report accepted nonlinear
  refinements and selected source alongside runtime success, not only QP counts.
  The minimum line-search fraction remains .25 and can miss smaller descent.
- The native-policy seed's first 20 coarse inputs should be identical across
  horizons from the same state/history/plane: the policy does not know the MPC
  horizon. Checking that prefix helps detect an initialization confound.
- Compare goal/velocity/tilt over common physical times and selected/native
  cost ratios at the same horizon. Raw accumulated costs are not comparable
  between horizons. The terminal multiplier stays 4; it is not a derived
  infinite-horizon value. The nonlinear scorer still omits the QP's tiny
  absolute-input numerical regularizer, as in the earlier experiments.
- If longer horizons merely retain the same policy seed and identical initial
  decisions, observing a later return in the longer rollout alone does not show
  that horizon optimization improved the command. Require a changed decision
  or common-prefix trajectory attributable to an accepted refinement before
  making that claim. Numerical failure and lack of nonlinear improvement must
  be separated from failure of the horizon hypothesis.

Sources reviewed: `AffineMPC._prediction_varying`, `AffineMPC.solve`,
`TrajectoryPredictiveExperiment.command`, and its nonlinear `evaluate` method.
