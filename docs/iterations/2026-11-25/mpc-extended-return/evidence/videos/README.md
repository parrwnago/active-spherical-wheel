# I-38 extended seeded-MPC replay

The [video](seeded_mpc_extended_return.mp4) reconstructs the saved extended
Gazebo/DART trajectory in MuJoCo. Rendering integrates no new physics.

The retained XY goal is shown by the orange marker. The small world-XY inset
keeps that goal fixed on screen while the robot and its path move relative to
it, independently of the 3D camera. The three charts show goal distance,
XY speed and absolute/reference tilt. The platform intentionally leans about
9.73° on this uniform 10° side slope; it is not a Mars map.

The gold dashed line marks the old 8.4 s cutoff. The newly observed interval
uses the unchanged I-34 seeded controller: .4 s horizon, one refinement,
stride-5 Jacobians, 20 ms held commands and 5 ms observer updates. The first
8.4 s of nine precision state/input/observer/reference fields must match the
old trace exactly before this renderer proceeds. The final goal error is
19.100 mm; the final full-second settled-return criterion is not met.

[Metadata](video_metadata.json) records input/source hashes, exact-prefix and
endpoint checks, frame count, dimensions, decoded final-frame verification and
the exact pre-render source snapshot. Playback is 0.5× simulated time. A goal
crossing is not presented as settling: the gate requires a full second within
5 mm, XY speed <=5 mm/s and reference-relative attitude error <=2°.

Reproduction from repository root (use a fresh rendering stage; existing
completed videos are deliberately protected from overwrite):

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u results/mpc_extended_return_20261125/videos/render_extended_return.py
```
