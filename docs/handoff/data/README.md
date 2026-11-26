# Selected recorded data

`i53-m100-28-36.npz` is an exact-value subset of the four I-52 −1.00-rad arms
over recorded time 28–36 s. It contains 5 ms rows without interpolation or dtype
conversion. It is a derived subset, not a full rollout or an additional trial.
The adjacent JSON records the original source hashes, fields, window and clock.

```python
import numpy as np

with np.load("docs/handoff/data/i53-m100-28-36.npz", allow_pickle=False) as data:
    t = data["m100_R__time_s"]
    qvel = data["m100_R__exact_qvel"]
    body_speed = np.linalg.norm(qvel[:, :2], axis=1)
    reference = data["m100_R__exact_guarded_joint_reference"]
    actual_joint_rates = qvel[:, 6:9]
```

Commands at time t act on the next 5 ms hold. A backward finite-difference
acceleration ending at t belongs with the previous held command, not the new
command at t. The bundle keeps requested/applied commands, the separate LQR/
feedforward/preparation terms, estimated/evaluated supports and native/used
references so the burst can be inspected without a simulator.
