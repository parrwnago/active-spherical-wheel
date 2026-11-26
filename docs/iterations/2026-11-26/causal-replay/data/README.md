# Inspect the causal comparison without launching Gazebo

The three `*_substeps.npz` files are byte-exact copies of the complete recorded
31.400–32.500 s traces. Each has 1,100 rows: raw world-frame bridge state before
and after each 1 ms solve, all exported contact rows with ragged offsets,
actual held inputs and both controller/plugin clocks. `state_fields`,
`contact_fields` and `contact_tag_names` supply the column labels.

The three `*_control_window.npz` files preserve every original precision and
sidecar field over the same window (221 rows at 5 ms). Keys are prefixed
`precision__` or `sidecar__`. These are exact-value subsets, with no resampling
or dtype conversion. The manifest binds original paths, hashes and derivations.

For example, from this directory with NumPy installed:

```python
import numpy as np

for case in ("native_substeps", "zero_free_shellA"):
    with np.load(case + "_substeps.npz", allow_pickle=False) as log:
        i = np.argmin(abs(log["time_s"] - 32.255))
        wz = list(log["state_fields"]).index("wz")
        print(case, log["bridge_state"][i, wz])
```

Expected rates are approximately 4.315864 and 4.045030 rad/s. Bridge `wz`
is world-vertical angular velocity; the angular part of precision `qvel`
is body-frame and must not be substituted directly.

Commands are `[shaft torque, shell A torque, shell B torque, belt A speed,
belt B speed]`, with three N m entries followed by two m/s entries. Inputs
are held throughout each five-step interval. Contacts sampled after a step
belong to that completed solve. Keep the exported points and their force
conventions; the accounting report documents COM-center sensitivity.
The terminal 32.500 s control command is computed but is not applied before
the diagnostic stops.

Inherited precision names `exact_native_requested`/`exact_applied` describe
the final returned values in these diagnostics. Sidecar `pre_output_*` gives
the computation before the actuator override. In the attitude branch that
computation already uses the held target, so it is not an untreated
counterfactual. `native_qref_quat` and `used_qref_quat` distinguish the
pre-hook normal-derived target from the quaternion supplied to feedback.

The full 32.5 s prefix logs and original full raw campaign remain local.
Parity/selectivity reports and hashes verify those longer prefixes. These
files support inspection of the registered event; they are not complete
rollouts or a new controller implementation.
