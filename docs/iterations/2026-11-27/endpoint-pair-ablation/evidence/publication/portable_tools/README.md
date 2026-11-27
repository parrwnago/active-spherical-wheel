I-71 cached inference replay requires Python 3.10+ and NumPy only.
From the root of this iteration's published `evidence/` tree:

```sh
python publication/portable_tools/replay_inference.py \
  --package . --output /tmp/i71_replay_new
```

The output directory must not already exist. Three fresh `python -I` workers
run from `/tmp`. They import the exact package copies of `inference_tools/`
and `inputs/packet_io.py`, record each module's `__file__` and SHA256, and call
only `run_case.infer_packet`. They do not call its repository-bound CLI or
seal loader. No live `spherical_wheel` package, labels, new geometry, force fit,
NNLS or physical simulator is loaded.

`reference_array_bindings.json` records all 27 canonical prediction arrays per
case before replay. `replay_source_input_seal.json` binds the wrapper, scientific
modules, input packets/manifests, protocol and reference bindings by relative
package paths. Sources and inputs are checked before and after execution;
in-memory input arrays are also checked for mutation. Every output dtype,
shape and array-byte hash must match. A failure preserves the computed output
and is not converted into a success by replacing values.

The producer replay used NumPy 1.26.4 and reproduced all **81 arrays** across
**26,403 frames** exactly, with unchanged inputs. This verifies cached numerical
reproduction, not estimator quality, calibrated confidence or robot motion.
Different numerical-library versions are not promised byte-exact identity.

The output contains each case's `predictions.npz`, import/registration record,
comparison report and progress log, plus an aggregate `receipt.json`.
Published QA retains the compact reports/logs; its recomputed prediction files
are derivable duplicates and can remain local. The canonical input uses the
sealed I-70 costs and selected endpoint points, so replay does not rerun I-70
force fitting or its original simulations.
