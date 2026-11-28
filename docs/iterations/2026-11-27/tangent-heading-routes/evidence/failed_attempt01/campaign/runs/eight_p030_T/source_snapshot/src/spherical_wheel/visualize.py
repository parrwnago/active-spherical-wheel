"""Replay saved trajectories with the real CAD display meshes and simple fork.

``render_rollout(log_path, output_path, fps=30, width=960, height=720)``
accepts an NPZ containing ``time_s`` (N,) and ``qpos`` (N, model.nq).
Optional ``reference_xy`` (N, 2), ``tilt_rad`` (N,), and scalar ``model_path``
are displayed when present. Output is MP4, GIF, or a PNG of the final state.

``view(model_path=None, controller=None)`` opens an interactive desktop viewer.
An optional callback ``controller(plant) -> five commands`` controls the plant.
Without a callback it designs local LQR balance feedback for the loaded model
and its saved configuration. Both functions return the output path
or run until the interactive viewer closes; neither changes a rollout log.
"""
from __future__ import annotations

import argparse
from functools import lru_cache
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]


def _model_path(value=None, log_path=None):
    if value is not None:
        value = np.asarray(value).item()
        if isinstance(value, bytes):
            value = value.decode("utf-8")
        path = Path(str(value)).expanduser()
        candidates = [path] if path.is_absolute() else [REPO_ROOT / path]
        if log_path is not None and not path.is_absolute():
            candidates.insert(0, Path(log_path).resolve().parent / path)
        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()
        raise FileNotFoundError(f"Saved model path does not exist: {path}")
    path = REPO_ROOT / "generated" / "model.xml"
    if not path.is_file():
        from .model import generate_model
        path = generate_model()
    return path


def _headless_subprocess_needed():
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return False
    # MuJoCo selects its GL context during import. Changing the environment
    # afterward cannot replace GLFW in a CLI that imported the plant first.
    imported = sys.modules.get("mujoco")
    if imported is not None:
        context = getattr(imported, "GLContext", None)
        return "glfw" in getattr(context, "__module__", "")
    return False


@lru_cache(maxsize=16)
def _font(size):
    from PIL import ImageFont
    for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"):
        if Path(path).is_file():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


def _overlay(frame, elapsed, tilt, position, has_reference):
    from PIL import Image, ImageDraw
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image, "RGBA")
    width, height = image.size
    scale = max(.7, width / 960)
    margin = int(18 * scale)
    draw.rectangle((0, 0, width, int(80 * scale)), fill=(10, 18, 27, 225))
    draw.text((margin, int(10 * scale)), "ACTIVE SPHERICAL WHEEL", font=_font(int(20 * scale)), fill=(235, 243, 250))
    draw.text((margin, int(42 * scale)),
              f"t = {elapsed:5.2f} s     tilt = {np.degrees(tilt):4.1f} deg     x = {position[0]:+.2f} m     y = {position[1]:+.2f} m",
              font=_font(int(15 * scale)), fill=(185, 211, 225))
    draw.rectangle((0, height - int(35 * scale), width, height), fill=(10, 18, 27, 220))
    draw.text((margin, height - int(27 * scale)), "Provisional physics / assumed masses and actuator limits",
              font=_font(int(13 * scale)), fill=(231, 190, 119))
    if has_reference:
        label = "Cyan: wheel path   Amber: reference"
        box = draw.textbbox((0, 0), label, font=_font(int(12 * scale)))
        draw.text((width - margin - box[2], int(15 * scale)), label,
                  font=_font(int(12 * scale)), fill=(210, 225, 231))
    return np.asarray(image)


def _add_path(mujoco, scene, points, rgba, width=2.):
    if len(points) < 2:
        return
    indices = np.unique(np.linspace(0, len(points) - 1, min(len(points), 120)).astype(int))
    points = points[indices]
    for a, b in zip(points[:-1], points[1:]):
        if scene.ngeom >= scene.maxgeom or np.linalg.norm(a - b) < 1e-6:
            continue
        geom = scene.geoms[scene.ngeom]
        # Simple screen-space lines avoid hundreds of tessellated capsules
        # per frame, which are costly on software OpenGL renderers.
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.ones(3), np.zeros(3), np.eye(3).ravel(), np.asarray(rgba))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, a, b)
        geom.category = mujoco.mjtCatBit.mjCAT_DECOR
        scene.ngeom += 1


def render_rollout(log_path, output_path, fps=30, width=960, height=720):
    """Render an NPZ trajectory locally, returning the created ``Path``.

    Frame poses are selected by nearest saved timestamp rather than linearly
    interpolating quaternion components. A PNG shows the final saved pose.
    """
    log_path = Path(log_path).expanduser().resolve()
    output_path = Path(output_path).expanduser().resolve()
    if fps <= 0 or width < 160 or height < 120:
        raise ValueError("fps must be positive, with width >= 160 and height >= 120")
    suffix = output_path.suffix.lower()
    if suffix not in (".mp4", ".gif", ".png"):
        raise ValueError("Output extension must be .mp4, .gif, or .png")
    if _headless_subprocess_needed():
        env = os.environ.copy()
        env["MUJOCO_GL"] = "egl"
        src = str(REPO_ROOT / "src")
        env["PYTHONPATH"] = src + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        subprocess.run([sys.executable, "-m", "spherical_wheel.visualize", "render", str(log_path), str(output_path),
                        "--fps", str(fps), "--width", str(width), "--height", str(height)], check=True, env=env)
        return output_path
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import imageio.v2 as imageio

    with np.load(log_path, allow_pickle=False) as log:
        timestamps = np.asarray(log["time_s"], float)
        qpos = np.asarray(log["qpos"], float)
        reference = np.asarray(log["reference_xy"], float) if "reference_xy" in log else None
        tilts = np.asarray(log["tilt_rad"], float) if "tilt_rad" in log else None
        model_path = _model_path(log["model_path"] if "model_path" in log else None, log_path)
    if timestamps.ndim != 1 or not len(timestamps) or np.any(np.diff(timestamps) <= 0):
        raise ValueError("time_s must contain a nonempty, strictly increasing vector")
    if not np.all(np.isfinite(timestamps)) or not np.all(np.isfinite(qpos)):
        raise ValueError("Rollout timestamps and qpos must be finite")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if qpos.shape != (len(timestamps), model.nq):
        raise ValueError(f"qpos must have shape ({len(timestamps)}, {model.nq}), got {qpos.shape}")
    if reference is not None and (reference.shape != (len(timestamps), 2) or not np.all(np.isfinite(reference))):
        raise ValueError("reference_xy must have shape (N, 2) and contain finite coordinates")
    if tilts is not None and (tilts.shape != timestamps.shape or not np.all(np.isfinite(tilts))):
        raise ValueError("tilt_rad must have shape (N,) and contain finite angles")
    model.vis.global_.offwidth = max(int(width), model.vis.global_.offwidth)
    model.vis.global_.offheight = max(int(height), model.vis.global_.offheight)
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = 135.
    camera.elevation = -23.
    camera.distance = 1.10
    option = mujoco.MjvOption()
    option.geomgroup[3:] = 0  # hide the contact decomposition beneath CAD visuals
    actual_path = np.column_stack((qpos[:, :2], np.full(len(qpos), .003)))
    reference_path = None if reference is None else np.column_stack((reference, np.full(len(reference), .005)))
    frame_times = np.arange(timestamps[0], timestamps[-1] + 1e-9, 1. / fps)
    if not len(frame_times) or frame_times[-1] < timestamps[-1] - 1e-8:
        frame_times = np.r_[frame_times, timestamps[-1]]
    if suffix == ".png":
        frame_times = timestamps[-1:]
    indices = np.searchsorted(timestamps, frame_times)
    indices = np.minimum(indices, len(timestamps) - 1)
    previous = np.maximum(indices - 1, 0)
    indices = np.where(np.abs(timestamps[previous] - frame_times) < np.abs(timestamps[indices] - frame_times), previous, indices)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = None
    try:
        if suffix == ".mp4":
            writer = imageio.get_writer(output_path, fps=fps, codec="libx264", quality=8,
                                        macro_block_size=2, ffmpeg_log_level="error")
        elif suffix == ".gif":
            writer = imageio.get_writer(output_path, mode="I", duration=1000. / fps, loop=0)
        with mujoco.Renderer(model, height=int(height), width=int(width)) as renderer:
            for index in indices:
                data.qpos[:] = qpos[index]
                data.time = timestamps[index]
                mujoco.mj_forward(model, data)
                camera.lookat[:] = qpos[index, :3] + np.array([0., 0., .14])
                renderer.update_scene(data, camera=camera, scene_option=option)
                # Detailed CAD creates slow shadow passes and stippling on
                # headless Mesa renderers. Diffuse lighting retains the shape.
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                if reference_path is not None:
                    _add_path(mujoco, renderer.scene, reference_path, [1., .64, .22, .8])
                _add_path(mujoco, renderer.scene, actual_path[:index + 1], [.16, .85, .9, .9])
                frame = renderer.render()
                quat = qpos[index, 3:7]
                tilt = tilts[index] if tilts is not None else np.arccos(np.clip(1. - 2. * (quat[1] ** 2 + quat[2] ** 2), -1., 1.))
                frame = _overlay(frame, timestamps[index], tilt, qpos[index, :2], reference is not None)
                if writer is None:
                    imageio.imwrite(output_path, frame)
                else:
                    writer.append_data(frame)
    finally:
        if writer is not None:
            writer.close()
    return output_path


def view(model_path=None, controller=None):
    """Run a desktop viewer, optionally applying ``controller(plant)`` each tick.

    The default LQR gain is computed for this exact plant configuration and
    configured shaft phase. A supplied callback is used without modification.
    """
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise RuntimeError("Interactive viewing needs a desktop display; use render_rollout for a headless MP4 or PNG")
    import mujoco
    import mujoco.viewer
    from .sim import Plant
    path = _model_path(model_path)
    saved_config = path.parent / "model_config.json"
    plant = Plant(config=saved_config if saved_config.is_file() else None, model_path=path)
    if controller is None:
        from .control import find_equilibrium, design_lqr
        phase = plant.config["simulation"].get("initial_shaft_phase_rad", .30)
        equilibrium = find_equilibrium(plant, phase=phase)
        design = design_lqr(plant, equilibrium)
        q0, v0, u0 = (equilibrium[k] for k in ("qpos", "qvel", "u"))
        gain = design["K"]
        plant.reset(qpos=q0, qvel=v0)
        controller = lambda p: u0 - gain @ p.error(q0, v0)
        print("Interactive viewer: local LQR balance designed for the loaded configuration is active.")
    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        viewer.cam.azimuth = 135.
        viewer.cam.elevation = -23.
        viewer.cam.distance = 1.10
        viewer.opt.geomgroup[3:] = 0
        while viewer.is_running():
            start = time.monotonic()
            command = np.zeros(5) if controller is None else controller(plant)
            with viewer.lock():
                plant.step(command)
                viewer.cam.lookat[:] = plant.data.qpos[:3] + np.array([0., 0., .14])
            viewer.sync()
            time.sleep(max(0., plant.dt - (time.monotonic() - start)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    render = commands.add_parser("render")
    render.add_argument("log")
    render.add_argument("output")
    render.add_argument("--fps", type=float, default=30)
    render.add_argument("--width", type=int, default=960)
    render.add_argument("--height", type=int, default=720)
    desktop = commands.add_parser("view")
    desktop.add_argument("--model")
    args = parser.parse_args()
    if args.command == "render":
        print(render_rollout(args.log, args.output, fps=args.fps, width=args.width, height=args.height))
    else:
        view(args.model)


if __name__ == "__main__":
    main()
