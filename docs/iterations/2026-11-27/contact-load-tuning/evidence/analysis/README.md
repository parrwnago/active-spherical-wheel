# I-62 retrospective tolerance sweep

The completed scientific result is `stage02/science.json`, with every registered
grid metric in `stage02/all_grid_metrics.npz` and per-case metrics in
`stage02/all_grid_case_metrics.csv`. All 3,721 nonnegative global tolerance pairs
were evaluated on three already examined episodes. No pair satisfied all
registered gates, either in any two-episode training fold or in the full-data
descriptive fit. Each failed selection therefore uses the explicit off/A
fallback; that fallback is not a successful fitted policy. No new simulation or
controller change was made.

The original I-61 tolerance reproduces every original keep flag, family choice
and matched point distance, including its 16/84/53 primary errors. The off/A
baseline has 14/64/46 primary errors, rather than the raw I-60 candidate family's
109 pooled errors: A includes the original acceptance and native fallback.

The pooled minimum over the numeric grid has 123 errors rather than A's 124,
but removes only one of six harmful overrides and retains the unsafe point
maximum. Requiring at least 50% harmful-override removal raises the best count
to 232, with 183/294 useful corrections retained. Enforcing a point maximum no
worse than native yields a minimum of 221 errors, with 195/294 useful corrections
retained. These are descriptive tradeoff witnesses, not qualified selections.

The first campaign, `stage01`, stopped before fold selection because a numerical
validation guard detected batch-dependent sum roundoff. Its source and log are
retained. `pre_execution_stage04` fixes candidate rows to contiguous float64
memory before their row reduction, preserving the original guard and every
scientific threshold, weight and selection rule. All original benchmark metric
differences are then zero. A considered validation-only alternative in stage03
was not campaign-executed. The first stage03 test invocation failed before test
collection because an automatically loaded ROS pytest plugin lacked `lark`;
the current contracts use `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. There are 11
passing synthetic contracts, inclusive of the earlier 10, not 21 separate tests.

Producer commands from the repository root were:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q results/contact_load_tuning_20261127/analysis/test_sweep_contracts.py
.venv/bin/python results/contact_load_tuning_20261127/analysis/tolerance_sweep.py --protocol results/contact_load_tuning_20261127/protocol/protocol.json --data-source results/contact_load_tuning_20261127/analysis/compact_stage01 --output /tmp/i62_fresh_sweep
```

The compact package requires Python 3 and NumPy. From an extracted evidence
package, a strict end-to-end replay command is:

```sh
python3 PACKAGE/analysis/replay_sweep.py --package PACKAGE --output /tmp/i62_replay
```

The replay verifies frozen input/source bindings, recomputes every grid setting
and selection, then compares all array bytes, scientific JSON values and CSV
bytes. Only the recorded NumPy version is excluded from exact JSON equality.
It neither normalizes computed values nor replaces them with reference values.
The source is sealed before replay in `replay_source_stage01`; generated replay
receipts identify the NumPy version and any differences explicitly. Cached
geometry and truth inputs are described in `compact_stage01/README.md`.
