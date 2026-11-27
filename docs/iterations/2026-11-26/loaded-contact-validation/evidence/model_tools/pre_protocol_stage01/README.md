# Frozen I56 model predictions for I57

`evaluate_frozen_model.py` reconstructs the exact I56 model call from the saved H-case measured state, prior belt commands and observer wrench. The pure support locator supplies per-patch geometry. It does not replace those records with the separately filtered normal or aggregate contact point.

At 20 ms the evaluator requires agreement with the original saved contact names, loads, rank, slip/confidence, estimated point, native/governed soft impulses and material-velocity residual norms. It exports all five loaded/free input columns at 5 and 20 ms, all nine generalized components, and physical world/fork projections. It runs no dynamics, physics integration, observer update or terrain query.

A root registration JSON needs `states`, each with `id`, `time_s` and an optional `perturbations` list (`id`, `delta_input` in shaft/shell A/shell B N m, belt A/belt B m/s order). Horizons are fixed to `[0.005, 0.02]`. Perturbations exceeding native amplitude limits are rejected, without silent clipping. Unit response columns are exported even when no perturbation vector is provided.

```sh
.venv/bin/python results/loaded_contact_validation_20261126/model_tools/evaluate_frozen_model.py --registration results/loaded_contact_validation_20261126/protocol.json --stage results/loaded_contact_validation_20261126/model_tools/prediction_stage01
```

Each attempt must use a fresh stage. The declaration records source and input hashes before prediction; the stage retains source snapshots, JSON predictions, exact matrix arrays and a hash manifest. The stage reads no later validation outcomes. The root owns the research iteration entry and native validation campaign.
