# I-58 campaign runner

This is a new result-owned runner, copied from the frozen I-56 capture pattern. No historical runner or preflight is edited or bypassed. The controller, observer, raw target, 5 ms native command timing, physical mechanics, amplitude caps and original flat gain bank remain fixed. One fresh Python process runs each registered case.

Registered variants:

- `native`: original native LQR, without an optional adapter.
- `hard_slew`: imports the unchanged original I-56 adapter with the exact I-56 A settings.
- `balance_priority`: imports the new P adapter, which calls the same original create/apply extraction and replaces only its allocator with `BalancePriorityAllocator`.

Both adapted arms must retain the exact original A soft costs, readiness and contact-model settings, including zero yaw and momentum weights. P changes the hard constraint policy. It retains amplitude limits and native transverse-response equality; the original rate values become diagnostics. Assumed rate exceedance is an observed command-rate metric for P, not a failed hard constraint or a hardware capability claim.

The root-owned protocol includes the ordinary I-56 registration fields and these replacements:

```json
{
  "iteration": "I-58",
  "frozen_A_protocol_path": "results/constrained_handoff_20261126/ablation/protocol.json",
  "adapters": {
    "hard_slew": {"path": "results/constrained_handoff_20261126/campaign_tools/adapter.py", "sha256": "REGISTERED", "settings": "EXACT I56 A SETTINGS OBJECT"},
    "balance_priority": {"path": "results/balance_priority_20261126/campaign_tools/adapter.py", "sha256": "REGISTERED", "settings": "SAME EXACT SETTINGS OBJECT"}
  },
  "historical_baseline_paths": {
    "m100_B": "results/constrained_handoff_20261126/campaign/runs/m100_B/precision.npz",
    "m100_A": "results/constrained_handoff_20261126/ablation/campaign/runs/m100_A/precision.npz"
  },
  "require_historical_baseline_exact": true
}
```

`input_sha256` must bind the original A protocol and every historical precision path. All native and hard-slew reproduction jobs require a hash-bound historical dataset. The comparison checks **every shared precision field** using exact shape, dtype and bytes, including signed zero/NaN payload and native/final command fields when present. A reproduction mismatch preserves the complete new run and failure evidence; it cannot silently become a baseline.

Root freezes all sources, physical binaries, data, hypotheses and acceptance criteria before any new integration. Preflight is read-only:

```sh
.venv/bin/python results/balance_priority_20261126/campaign_tools/run_case.py --preflight
```

After the protocol is frozen and physics is authorized, run cases serially, each through a separate process:

```sh
.venv/bin/python results/balance_priority_20261126/campaign_tools/run_case.py --run --case-id m100_B
.venv/bin/python results/balance_priority_20261126/campaign_tools/run_case.py --run --case-id m100_A
.venv/bin/python results/balance_priority_20261126/campaign_tools/run_case.py --run --case-id m100_P
```

Precision fields retain I-56 semantics. `exact_native_requested` is the unbounded original native total demand; `exact_native_applied` is its amplitude-clipped counterfactual. `exact_requested` and `exact_applied` are final governed demand/actual input. The actual previous governed input is fed naturally through ordinary physical stepping and state mirroring. Original command components remain separately recorded. The optional governor diagnostics JSON carries all model/allocator diagnostic fields; no component is folded into native feedback diagnostics. Final saturation flags describe final demand, while `exact_native_saturation_mask` describes original infeasible demand.

The wrapper verifies native targets, controller history, observer history, plant state and previous input are unchanged around adapter creation/apply. Contacts supplied to either adapter come from the same momentum-observer/locator path; solved contacts remain evaluation-only. The original raw-target and gain phase are not replaced by an easier target.

`checks_stage01/` preserves the first 13-contract read-only check subset. `checks_stage02/` preserves the complete 16-contract invocation, which includes those 13 tests plus three adapter dispatch/boundary tests. Do not add these overlapping counts. Adapter tests use a stub replacement allocator and do not validate optimizer mathematics or simulate physics; separate new-module tests own that evidence.
