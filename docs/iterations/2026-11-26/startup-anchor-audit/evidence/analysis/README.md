# I-59: fixed-state startup-anchor audit

The startup shaft-feedback coefficients differ numerically, but their largest gain-row-only command contribution over all six saved B/P traces is approximately **2.53e−8 N·m**. This is about 40,000 times below the frozen 1e−3 N·m shaft resolution. Shaft synchronization weight is zero, unload fade lies in [0,1], and amplitude clipping cannot amplify the mathematical command difference. The full native pipeline independently gives approximately 1.93e−9 N·m maximum clipped change. Trim-only changes are approximately 5e−13 N·m. No physical simulation was run.

This is a null result for the proposed practical startup-gain explanation at these recorded states. The physically different complete trajectories remain real; this calculation does not identify their full cause or produce an alternate trajectory. P rows audit the native policy at recorded P states. They do not reoptimize the final P command.

The compact tracked primary evidence is `coefficient_compact_stage01/`. It retains every original 5 ms timestamp and exact error, actual shaft-gain row, three anchor rows and computed pre-fade delta. Redundant per-coordinate contributions and native command histories are omitted. `projection_binding.json` records byte-exact retained fields, each original local NPZ hash and the original stage manifest. The original larger `coefficient_stage01/` remains sealed locally.

Standalone primary replay needs only Python and NumPy, without a repository or results-owned import:

```sh
python3 PATH/TO/analysis/replay_coefficients.py \
  --data-source PATH/TO/analysis/coefficient_compact_stage01 \
  --output NEW_COEFFICIENT_REPLAY.json
```

`coefficient_compact_replay_stage01.json` records exact reproduction of all six effect tables and every full-rate delta array. The material threshold defaults to the protocol's 0.001 N·m; changing it cannot silently reproduce the sealed report.

The scientific figure is `plots_stage01/anchor_effect.pdf` and `anchor_effect.png`. It combines the coefficient-difference table with maximum raw/native-clipped gain-row effects for all three alternative anchors at every saved state. Gray cells are exact identity swaps. The engineering resolution and native-shadow scope are shown explicitly.

The supplementary `pipeline_stage01/science.json` contains the complete fixed-window reports: full 0–44 s, moving 1–41 s, goal 41–44 s and the three preselected event windows. It verifies native command decomposition, native clipping, synchronization/fade propagation and exact identity swaps. All reconstructed fades retain intervals consistent with their float32-only telemetry; they are not described as exactly recorded scalars. Material raw/clipped changes and exposure are zero under both absolute and normalized resolutions, including uncertainty bounds. Scalar helper and vectorized propagation agree at the fixed check states.

Detailed pipeline replay was also completed exactly in `pipeline_replay_stage01/replay_comparison.json`:

```sh
python3 PATH/TO/analysis/pipeline_audit.py \
  --data-source PATH/TO/FULL_LOCAL_PIPELINE_STAGE \
  --output NEW_PIPELINE_REPLAY_DIRECTORY
```

The full pipeline's six `inputs/*.npz` files are approximately 43 MB and remain local. They are intentionally excluded from compact publication; source, report, metadata, input indexes and origin hashes are retained. Restore those exact local NPZ files into the published `pipeline_stage01/inputs/` directory before detailed pipeline replay. The portable compact primary replay above does not need them. Similarly, excluded original coefficient-stage and independent-review NPZ files remain local; their publication manifests retain the original hashes rather than pretending the archive contains those inputs.

The protocol was frozen before coefficient arithmetic. The minimal coefficient script/input registration was captured before its calculation. The supplementary pipeline script was frozen after the preliminary coefficient bound was known and before its own arithmetic; this chronology is recorded in its registration. The protocol and numerical mapping were unchanged. All actual states, errors, references, other gain rows, contact context and native components were held fixed. There were no controller, estimator, reference, default or runtime edits.
