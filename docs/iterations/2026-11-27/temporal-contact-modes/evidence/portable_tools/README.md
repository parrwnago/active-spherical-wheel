# I-70 full inference replay

This wrapper reconstructs all 26,403 measured-state frames and repeats the
frozen shadow inference. It needs Python, NumPy and SciPy. It uses no MuJoCo,
Gazebo, ROS, terrain or contact-truth labels.

```sh
python portable_tools/replay_inference.py \
  --inputs portable_inputs \
  --output /tmp/i70_new_replay
```

The output directory must be new. Three fresh isolated Python workers run in
parallel, with one BLAS thread each. `--case m100_B` can select one case. All
scientific imports resolve to byte-preserved copies in this packet; the empty
`source/spherical_wheel/__init__.py` is generated solely for packaging and does
not import the live package initializer. Import paths and source hashes are
recorded before computation.

The same copied `evaluate_frame`, `ShadowInference` and `frame_features` perform
the computation. `frame_features` runs uncached from the compact primitives.
The wrapper streams hashes of all six reconstructed feature arrays against the
original local cache bindings, then compares every prediction and model-evidence
array's dtype, shape and raw bytes with the canonical bindings. Reference output
hashes are used only for verification, never as inference inputs.

The frozen scientific module and runner copies are unchanged. Wrapper changes
are limited to package import isolation, primitive loading, output plumbing and
verification. The source seal precedes the first replay NNLS/temporal arithmetic.
No live controller or simulator is executed.

Each case writes fresh `predictions.npz`, `model_evidence.npz`, a registration
record and `replay_report.json`; the parent writes `replay_summary.json`.
Large recomputed NPZs can remain local. Compact replay reports and array bindings
are enough to verify the archive without duplicating the original derived
arrays.

Bit-exact replay is assessed in the producer environment. A different NumPy or
SciPy version may change floating point or NNLS behavior; no cross-version
numerical guarantee is asserted by this wrapper. The underlying mode beliefs
remain conditional engineering outputs, not empirically calibrated contact
probabilities or a demonstrated controller improvement.
