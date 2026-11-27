This is the lossless cached-data input for I-68. It does not run Gazebo,
MuJoCo, a controller, a new estimator, or a fit.

From the directory containing `portable_inputs/` and `portable_tools/`:

```sh
python3 portable_tools/capture_archive.py check --package portable_inputs
python3 portable_tools/replay_analysis.py --package portable_inputs --output /tmp/i68-replay-new
```

The output directory must not already exist. Python and NumPy are required.
The complete 44-second capture is restored in memory. To write its original
36 array fields as a new NPZ, use:

```sh
python3 portable_tools/capture_archive.py expand --package portable_inputs --output /tmp/i68-substeps-new.npz
```

`capture_payload.npz` stores the post-step raw bridge states and all reported
contact rows. Consecutive pre-step states and contact packets are reconstructed
from the preceding post-step records. The first pre-step records and any
discontinuities are stored explicitly; this run needed zero exceptions.
Pre/post qpos and qvel are derived with the unchanged frozen collector function.
All other original capture fields are retained. The manifest binds every
restored array's dtype, shape and SHA256 of its bytes; restoration is refused
if any field differs. The compressed NPZ container's bytes need not match.

`essential_precision.npz` contains ten original precision fields needed by the
frozen offline analysis. These fields were checked independently against both
the old I-58 baseline and the new I-68 precision file before deduplicating the
identical data. Both original input files and both sets of array bindings are
recorded in the manifest. This subset is separate from the campaign's complete
59-field parity receipt.

Producer NumPy 1.26.4 and system NumPy 1.21.5 both restored all 36 capture arrays
and both ten-field precision sets with identical dtype, shape and bytes.
The original physics outputs remain local; this package replays cached
analysis and makes no claim about missing contact couples or total solver
constraint forces.
