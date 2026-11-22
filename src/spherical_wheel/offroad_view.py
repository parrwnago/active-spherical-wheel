"""Rendered stills and replays of the wheel on its terrain, from saved runs.

Rendering needs an OpenGL context. On a headless Linux machine set
``MUJOCO_GL=egl`` before starting Python. Nothing here changes a run: the
plant's state is restored after every render.
"""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np

_TRAIL = np.array([.1, .85, .95, 1.])
_REFERENCE = np.array([1., .62, .2, 1.])


def _camera(position, azimuth, elevation, distance):
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth, camera.elevation, camera.distance = azimuth, elevation, distance
    camera.lookat[:] = np.asarray(position)+np.array([0., 0., .06])
    return camera


def _line(scene, start, end, rgba, width=3.):
    if scene.ngeom >= scene.maxgeom or np.linalg.norm(end-start) < 1e-6:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.ones(3), np.zeros(3), np.eye(3).ravel(), rgba)
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, start, end)
    geom.category = mujoco.mjtCatBit.mjCAT_DECOR
    scene.ngeom += 1


def _marker(scene, position, rgba, radius=.012):
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.full(3, radius), np.asarray(position, float),
                        np.eye(3).ravel(), rgba)
    geom.category = mujoco.mjtCatBit.mjCAT_DECOR
    scene.ngeom += 1


def _draw(plant, renderer, log, index, camera, option, shadows):
    """Render the logged state ``index`` with the path so far and the reference point."""
    plant.data.qpos[:] = log['qpos'][index]
    plant.data.qvel[:] = 0.
    plant._forward()
    renderer.update_scene(plant.data, camera=camera, scene_option=option)
    scene = renderer.scene
    scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = bool(shadows)
    stride = max(1, index//150)
    trail = np.column_stack((log['qpos'][:index+1:stride, :2], log['ground_height_m'][:index+1:stride]+.004))
    for start, end in zip(trail[:-1], trail[1:]):
        _line(scene, start, end, _TRAIL)
    x, y = (float(value) for value in log['reference_xy'][index])
    _marker(scene, [x, y, plant.terrain.height(x, y)+.012], _REFERENCE)
    return renderer.render()


def render_view(plant, log, time_s, output, width=1280, height=800, azimuth=112., elevation=-14., distance=1.25):
    """Render the plant at the logged state nearest ``time_s`` and return the path.

    The ground squares are 0.1 m. The cyan line is the wheel centre's path so
    far, drawn just above the ground, and the orange ball is the reference.
    """
    index = int(np.argmin(np.abs(log['time_s']-time_s)))
    saved = plant.capture()
    try:
        option = mujoco.MjvOption()
        option.geomgroup[3:] = 0
        camera = _camera(log['qpos'][index, :3], azimuth, elevation, distance)
        with mujoco.Renderer(plant.model, height=int(height), width=int(width)) as renderer:
            frame = _draw(plant, renderer, log, index, camera, option, shadows=True)
    finally:
        plant.restore(saved)
    import imageio.v2 as imageio
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(output, frame)
    return output


@lru_cache(maxsize=8)
def _font(size):
    from PIL import ImageFont
    for path in ('C:/Windows/Fonts/segoeui.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
                 '/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf'):
        if Path(path).is_file():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


def _caption(frame, heading, lines, footer):
    from PIL import Image, ImageDraw
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image, 'RGBA')
    width, height = image.size
    scale = max(.6, width/960)
    margin = int(16*scale)
    draw.rectangle((0, 0, width, int((34+20*len(lines))*scale)), fill=(10, 18, 27, 215))
    draw.text((margin, int(7*scale)), heading, font=_font(int(19*scale)), fill=(235, 243, 250))
    for row, line in enumerate(lines):
        draw.text((margin, int((33+20*row)*scale)), line, font=_font(int(14*scale)), fill=(190, 213, 226))
    draw.rectangle((0, height-int(28*scale), width, height), fill=(10, 18, 27, 215))
    draw.text((margin, height-int(23*scale)), footer, font=_font(int(12*scale)), fill=(231, 190, 119))
    return np.asarray(image)


def render_replay(plant, runs, output, fps=30, speed=1., width=960, height=720, azimuth=112., elevation=-14.,
                  distance=1.25, title=None):
    """Write an MP4 replay of one or more saved runs on the same terrain, side by side.

    ``runs`` is a list of ``(label, log)``. Each run gets its own panel with a
    camera that follows its wheel, so a run that falls behind stays in view.
    The cyan line is the path travelled and the orange ball is where the
    reference is at that moment. ``speed`` above 1 plays faster than real
    time and is stated in the caption. A run that ended early holds its last
    frame, marked as stopped. Returns the output path.
    """
    import imageio.v2 as imageio
    runs = list(runs)
    if not runs:
        raise ValueError('A replay needs at least one run')
    end = max(float(log['time_s'][-1]) for _, log in runs)
    times = np.arange(0., end+1e-9, speed/fps)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    width, height = int(width)-int(width) % 2, int(height)-int(height) % 2
    option = mujoco.MjvOption()
    option.geomgroup[3:] = 0
    saved = plant.capture()
    writer = imageio.get_writer(output, fps=fps, codec='libx264', quality=8, macro_block_size=2,
                                ffmpeg_log_level='error')
    footer = 'Simulated provisional model. Ground squares 0.1 m. Cyan: path travelled. Orange ball: reference.'
    if speed != 1.:
        footer += f' Played at {speed:g}x.'
    try:
        with mujoco.Renderer(plant.model, height=height, width=width) as renderer:
            for moment in times:
                panels = []
                for label, log in runs:
                    stopped = moment > log['time_s'][-1]+1e-9
                    index = min(int(np.searchsorted(log['time_s'], moment)), len(log['time_s'])-1)
                    camera = _camera(log['qpos'][index, :3], azimuth, elevation, distance)
                    frame = _draw(plant, renderer, log, index, camera, option, shadows=False)
                    slope = np.degrees(np.arccos(np.clip(log['ground_normal'][index, 2], -1., 1.)))
                    error = np.linalg.norm(log['qpos'][index, :2]-log['reference_xy'][index])*1000.
                    lines = [f"t = {log['time_s'][index]:5.1f} s    ground slope {slope:4.1f} deg    "
                             f"body lean {np.degrees(log['tilt_rad'][index]):4.1f} deg",
                             f"distance from reference {error:6.0f} mm    "
                             f"ground load {log['normal_force_N'][index]:5.1f} N"]
                    if stopped:
                        lines[1] = f"run stopped at {log['time_s'][-1]:.1f} s"
                    panels.append(_caption(frame, label if title is None else f'{title}: {label}', lines, footer))
                writer.append_data(np.concatenate(panels, axis=1))
    finally:
        writer.close()
        plant.restore(saved)
    return output
