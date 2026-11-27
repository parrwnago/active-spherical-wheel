# I-72 isolated inference replay

From the published `evidence/` directory, using Python 3 and NumPy:

```sh
python publication/portable_tools/replay_inference.py --package . --output /tmp/i72_replay_new
```

The output directory must be new. The wrapper starts three fresh Python `-I`
workers with `/tmp` as their working directory. Each imports the byte-preserved
`inference_tools/geometry_first_contact.py`, `frozen_i60_rank.py`, `run_case.py`
and `inputs/packet_io.py` from this package and records each `__file__` and hash.
It calls only `run_case.infer_case(packet_io.load_case(...))`, bypassing the
runner's repository-bound CLI and seal loader.

`replay_source_input_seal.json` binds all required sources, input packets,
metadata and canonical reference bindings before execution. Every input array,
including the nested projected features and separated native fallback, is
checked for mutation. Sources and disk inputs are hash-checked before and after
replay. Each canonical prediction array is compared by field name, dtype, shape
and SHA256 of its contiguous bytes; NPZ compression bytes are not compared.

The producer replay passed with Python 3.10 / NumPy 1.26.4: all 79 arrays in each
of three cases, 237 arrays across 26,403 frames, were byte-exact. All 17 decoded
input arrays per case were unchanged. Cross-version byte identity is not
asserted. A mismatch preserves the recomputed values and logs and exits with
an error; no values are normalized or replaced to force a pass.

This reproduces cached numerical inference. Original valid candidate
coordinates and projected I-60 features are inputs; their geometry generation
is not rerun. Missing inactive coordinates remain NaN. No truth labels, force
fit, temporal filter, simulator, model assets, controller or live package is
needed. Success establishes reproduction, not scientific acceptance or a
driving improvement.

`prepare_replay.py` records canonical output bindings after the released
all-three inference seal. It is provenance tooling, not part of the replay
command, and requires the local canonical prediction files if rerun.

Compact QA receipts and logs are in `publication/replay_qa/stage01/`.
`publication/replay_qa/artifact_manifest.json` identifies the three regenerated
prediction NPZ files omitted from the shareable snapshot: they duplicate the
canonical arrays, and their complete array bindings remain in each case's
`replay_report.json` and in `reference_array_bindings.json`.
