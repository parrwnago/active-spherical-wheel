# I-61 portable load gate and N/A/C policy replay

`gate_stage01/` is a 7.80 MB standalone Python/NumPy package. It preserves every
original timestamp of all three 8,801-row cases and reproduces **all 59 original
feature/policy packet fields byte-for-byte**. This includes full residuals,
primary/diagnostic load flags and ratios, and N/A/C family/point policies.

Geometric predictions, point validity flags, selected points, observer values
and affine maps are feature **inputs**, not computations reproduced by this
NumPy replay. Reproducing those geometric features additionally requires the
original raw states, frozen robot model and private-kinematics code. The original
packet/source hashes remain attached. The replay is not an independent force
measurement or a simulation of changed controls.

The package retains the observed generalized vector and wrench, interval-mean
diagnostic vector, both proposed/native-alternative full predictions, candidate
IDs/points, native fallback points and four-family oracle-affine maps. It never
contains or reads truth. An independent evaluation replay owns its separate
truth inputs.

To avoid duplicating derived arrays, the replay recomputes generalized residuals,
the fixed extra-three-joint gate, the native-relative diagnostic and N/A/C policy
outputs. N/A/C include same-family point changes. The original dtype, shape and
SHA-256 of each entire output array are checked. Array values and masks are not
rounded or downsampled.

Input storage uses exact aliases, a constant-radius representation and fixed
64-bit XOR references for closely related arrays. XOR is lossless bit storage,
not numerical subtraction: NaN payloads, signed zero and every floating-point
bit are retained. Each decoded input is verified before gate arithmetic. Native
alternative full-nine diagnostics and observer interval means were retained
even though this makes the package slightly larger than the initial 6 MB goal.

Run with Python 3 and NumPy, without MuJoCo or repository imports:

```sh
python3 PATH/TO/gate_stage01/replay_gate.py \
  --data-source PATH/TO/gate_stage01 \
  --output NEW_LOAD_GATE_REPLAY_DIRECTORY
```

Both `replay_stage01/load_gate_replay.json` and
`external_replay_stage01/load_gate_replay.json` confirm complete exact replay.
The second invocation runs system Python from `/tmp`, outside the repository.
For end-to-end evaluation, import `replay_gate.py` and call
`load_and_gate_case(package_path, case)`, returning `(full_packet, parity_report)`.

The packer validates **all three** complete feature seals before opening any
numerical packet. Its source was frozen in `pre_execution_stage01/` before
actual packing; prior aggregate outcome knowledge is explicitly recorded there.
The synthetic compression/policy contract checks exact reconstruction, signed
zero/NaNs and same-family point changes. It is not an additional repository
test suite or a physical trial. No threshold, proposal, gate, policy or existing
core source changed during this preservation work.

Regenerate another package only from the complete sealed local feature cases:

```sh
python3 PATH/TO/pack_gate.py \
  --case-directories FULL_LOCAL/m100_B FULL_LOCAL/p030_B FULL_LOCAL/p090_B \
  --output NEW_PORTABLE_PACKAGE
```
