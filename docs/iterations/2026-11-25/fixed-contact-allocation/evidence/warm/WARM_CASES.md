The eight short Gazebo cases compare existing least-squares control with the
implemented `rolling_allocation='feasible'` candidate. The candidate changes
rolling allocation and feedback motion references; it retains the existing
LQR library and gain schedule. Instantaneous scheduled gain matrices can vary
when trajectories diverge. The original destination remains logged and is
evaluated independently of the candidate's projected feedback reference.

Both branches run the same 0.8-second LS warm-up on a 10-degree world-Y slope,
starting at shaft phase pi/2. At motion start, both nominal paths are anchored
to the measured wheel centre, with `track_contact_point=False`. The candidate
then replaces only `params`, `rolling_allocator`, and `motion_reference`.
The live observer, estimator, synchronizer, nominal joint references, robot
state and simulator history are preserved. Warm prefixes agree within 2.7e-14.
Controller calls run with simulator-contact APIs forbidden and the StateMirror
has no ground or contacts. Truth contact geometry is reconstructed afterward
for evaluation and never supplied to control.

| Pulse | Direction | Requested travel | LS / feasible travel along request at motion end | LS / feasible original-goal endpoint error |
| --- | --- | --- | --- | --- |
| 0.60 s, ramps 0.20 s | world Y | 16.0 mm | 8.5% / 8.9% | 15.03 / 14.97 mm |
| 0.60 s, ramps 0.20 s | world X | 16.0 mm | 85.4% / 91.6% | 3.50 / 2.89 mm |
| 0.16 s, ramps 0.05 s | world Y | 4.4 mm | 69.4% / 63.7% | 2.88 / 2.84 mm |
| 0.16 s, ramps 0.05 s | world X | 4.4 mm | 180.6% / 179.0% (overshoot) | 3.55 / 3.47 mm |

All eight runs complete. Actual shell A carries 100% of shell/belt support
throughout each pulse, with zero patch handovers and at least 90.4% of robot
weight supported. Neither family passes the declared sustained weak-geometry
gate: true normalized sigma2 does not remain below 0.02 m/s throughout samples
with nominal speed at least 0.02 m/s. In the short world-Y pair, the gate is
violated after about 40 ms (LS) / 75 ms (candidate); only 13% / 43% of the peak
window stays below threshold. These are transient same-hemisphere comparisons,
not proof of sustained weak-geometry performance or completed blocked goals.
The 0.60-second family is mixed-geometry evidence. No further parameter search
was run after the declared short-pulse family.

The short-family world-Y original-goal error after its 0.4-second recovery is
6.2 / 5.5 mm; the endpoint percentages above refer to motion end, not recovery
end. Small absolute errors must be assessed relative to the 4.4 mm request.

Videos and paired stills are saved under `videos/` and `short_pulse/videos/`.
The 0.60-second family covers simulation time 0–2 seconds, played at 0.25x in
8.05-second videos. The short family shows simulation time 0.6–1.36 seconds,
played at 0.1x in 7.65-second videos. All are 1920 by 720 pixels at 20 fps; final
frames decode successfully. Videos replay saved states in the analytical
MuJoCo terrain viewer and run no new physics. Orange is the original centre
goal, green the feedback reference, and cyan actual travel. Full metadata and
SHA256 hashes are in each video's `video_metadata.json`.

`warm_case_comparison.json`, `.csv`, and `.png` summarize both families.
Original run summaries and rollouts were retained. Source snapshots precede
each campaign under `source_snapshot/` and `short_pulse/source_snapshot/`.
The first invocation used `.venv`, which lacks `gz` bindings, and stopped before
Gazebo simulation. Its traceback remains in `setup_failed_missing_gz.log`;
the corrected environment is used below.

Reproduce the 0.60-second family from the repository root with a fresh output
directory (the harness refuses to overwrite completed cases):

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH MAMBA_ROOT_PREFIX="$PWD/build/mamba" build/tools/bin/micromamba run -p "$PWD/build/gazebo-env" env MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -u results/fixed_contact_allocation_20261125/run_warm_contact_cases.py --candidate-params results/fixed_contact_allocation_20261125/candidate_params.json --output results/fixed_contact_allocation_repeat/long
```

For the short family add `--motion-s .16 --ramp-s .05 --recovery-s .4` and use a
different output directory. Analysis and saved-state replays:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python results/fixed_contact_allocation_20261125/analyze_warm_cases.py
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u results/fixed_contact_allocation_20261125/render_warm_comparisons.py
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u results/fixed_contact_allocation_20261125/render_warm_comparisons.py --family results/fixed_contact_allocation_20261125/short_pulse --speed .1 --start-s .6
```
