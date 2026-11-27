"""I56 fixed B/A/H comparison from saved Gazebo states; no new physics.

The scene follows the old I40/I52 Mars presentation style, but is built without
TerrainPlant/reset/mj_forward. Only static kinematics updates each saved pose.
Campaign outcomes must be sealed and the parent must authorize rendering first.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw
import yaml

from spherical_wheel.gazebo.mars import DEM, MarsYard, build_route
from spherical_wheel.gazebo.mars_compare import _crop_features
from spherical_wheel.offroad import build_offroad_model
from spherical_wheel.offroad_view import _camera, _font, _line, _marker

HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
ROOT = FAMILY.parents[1]
PLAN = HERE / 'presentation_plan.json'
WIDTH, HEIGHT, FPS = 1920, 1080, 20
BG, FG, MUTED = (10, 20, 31), (222, 234, 245), (160, 187, 205)
WARN = (246, 193, 108)
COLORS = {'B': (230, 174, 87), 'A': (92, 200, 220), 'H': (176, 135, 218)}


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def text(draw, xy, value, size=20, color=FG):
    draw.text(xy, str(value), font=_font(size), fill=color)


def wrapped(draw, xy, value, width, size=25, color=FG):
    x, y = xy
    line = ''
    for word in str(value).split():
        candidate = (line + ' ' + word).strip()
        if draw.textbbox((0, 0), candidate, font=_font(size))[2] > width and line:
            text(draw, (x, y), line, size, color)
            y += size + 10
            line = word
        else:
            line = candidate
    text(draw, (x, y), line, size, color)
    return y + size + 10


def nearest(times, instant):
    index = min(int(np.searchsorted(times, instant)), len(times) - 1)
    if index and abs(times[index - 1] - instant) <= abs(times[index] - instant):
        index -= 1
    return index


def rotation_matrices(quats):
    q = np.asarray(quats, dtype=float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.stack((1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y),
                     2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x),
                     2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)), axis=-1).reshape(q.shape[:-1] + (3, 3))


def load_run(spec, route):
    folder = FAMILY / spec['path']
    manifest = read(folder / 'artifact_manifest.json')
    for item in manifest['files']:
        assert sha(folder / item['path']) == item['sha256'], (spec['key'], item['path'])
    with np.load(folder / 'precision.npz') as saved:
        exact = {key: saved[key].copy() for key in saved.files}
    with np.load(folder / 'rollout.npz') as saved:
        rollout = {key: saved[key].copy() for key in saved.files}
    log = {name: exact[key] for name, key in (
        ('time_s', 'time_s'), ('qpos', 'exact_qpos'), ('qvel', 'exact_qvel'),
        ('input', 'exact_applied'), ('reference_xy', 'exact_reference_xy'),
        ('reference_quat', 'exact_reference_quat'))}
    n = len(log['time_s'])
    assert n > 1 and np.all(np.diff(log['time_s']) > 0)
    assert all(len(values) == n for values in log.values())
    assert all(np.isfinite(values).all() for values in log.values())
    # The precision and ordinary logs refer to the same pre-command states.
    # Ordinary rollout is intentionally stored in float32. Check its exact
    # serialization rounding, retaining full precision for every rendered pose.
    for key in ('time_s', 'qpos', 'qvel'):
        np.testing.assert_array_equal(log[key].astype(rollout[key].dtype), rollout[key])
    nominal = np.array([route(t)[0] for t in log['time_s']])
    np.testing.assert_allclose(log['reference_xy'], nominal, atol=1e-10, rtol=0)
    actual_R = rotation_matrices(log['qpos'][:, 3:7])
    target_R = rotation_matrices(log['reference_quat'])
    log['absolute_tilt_deg'] = np.degrees(np.arccos(np.clip(actual_R[:, 2, 2], -1, 1)))
    cosine = np.einsum('ni,ni->n', actual_R[:, :, 2], target_R[:, :, 2])
    log['raw_target_error_deg'] = np.degrees(np.arccos(np.clip(cosine, -1, 1)))
    np.testing.assert_allclose(log['raw_target_error_deg'], np.degrees(rollout['attitude_error_rad']), atol=2e-6, rtol=0)
    # MuJoCo's free-joint angular qvel is local; a yaw-coordinate derivative
    # and raw qvel[5] are not the physical world-Z angular velocity.
    log['world_wz_rad_s'] = np.einsum('ni,ni->n', actual_R[:, 2, :], log['qvel'][:, 3:6])
    log['timed_error_mm'] = 1000*np.linalg.norm(log['qpos'][:, :2] - log['reference_xy'], axis=1)
    log['speed_mm_s'] = 1000*np.linalg.norm(log['qvel'][:, :2], axis=1)
    log['ground_height_m'] = rollout['ground_height_m']
    return dict(spec=spec, folder=folder, log=log, summary=read(folder/'summary.json'))


def forbidden(*unused, **ignored):
    raise AssertionError('Saved-state rendering attempted dynamics, collision solving, or integration')


def static_scene(config, yard, bounds, directory):
    """Adapt replay_plant's visual construction without creating a plant."""
    directory.mkdir(parents=True)
    path, visual = build_offroad_model(config, yard, directory, visual_bounds=bounds)
    root = ET.parse(path).getroot()
    asset, world = root.find('asset'), root.find('worldbody')
    body = ET.SubElement(world, 'body', name='mars_features')
    colors = {'lava': '0.30 0.25 0.22 1', 'rocks_big': '0.42 0.33 0.27 1', 'rocks_medium': '0.48 0.38 0.30 1'}
    for name, mesh_path in _crop_features(bounds, directory):
        ET.SubElement(asset, 'mesh', name=name+'_crop', file=str(mesh_path))
        ET.SubElement(body, 'geom', name=name+'_visual', type='mesh', mesh=name+'_crop',
                      rgba=colors.get(name, '0.4 0.35 0.3 1'))
    for texture in asset.findall('texture'):
        if texture.get('name') == 'terrain_tex':
            texture.set('rgb1', '0.62 0.40 0.27')
            texture.set('rgb2', '0.68 0.45 0.31')
    # Even accidental collision invocation has no contact-enabled display geom.
    for geom in root.iter('geom'):
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
    path = directory / 'model_display_only.xml'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode', xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(path))
    field = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, 'terrain')
    start = int(model.hfield_adr[field])
    normalized = (visual['heights'] - visual['low']) / visual['span']
    model.hfield_data[start:start+normalized.size] = normalized.ravel()
    return SimpleNamespace(model=model, data=mujoco.MjData(model), terrain=yard)


def draw_scene(scene, renderer, log, index, camera, planned):
    scene.data.qpos[:] = log['qpos'][index]
    scene.data.qvel[:] = 0
    mujoco.mj_kinematics(scene.model, scene.data)
    mujoco.mj_comPos(scene.model, scene.data)
    mujoco.mj_camlight(scene.model, scene.data)
    option = mujoco.MjvOption()
    option.geomgroup[3:] = 0
    renderer.update_scene(scene.data, camera=camera, scene_option=option)
    visual = renderer.scene
    visual.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
    for p, q in zip(planned[:-1], planned[1:]):
        _line(visual, p, q, np.array([1., .62, .2, .7]), width=2.)
    stride = max(1, index//180)
    trail = np.column_stack((log['qpos'][:index+1:stride, :2], log['ground_height_m'][:index+1:stride] + .006))
    for p, q in zip(trail[:-1], trail[1:]):
        _line(visual, p, q, np.array([.1, .85, .95, 1.]))
    x, y = log['reference_xy'][index]
    _marker(visual, [x, y, scene.terrain.height(float(x), float(y))+.012], np.array([1., .62, .2, 1.]))
    return renderer.render()


def inset(pixels, log, index, bounds, planned):
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image, 'RGBA')
    left, top, width, height = 440, 238, 185, 140
    draw.rounded_rectangle((left, top, left+width, top+height), radius=5, fill=(8, 17, 26, 225))
    text(draw, (left+6, top+4), 'Fixed world XY', 12)
    x0, x1, y0, y1 = bounds
    def project(point):
        x, y = point
        return (left+10+(x-x0)/(x1-x0)*(width-20), top+25+(y1-y)/(y1-y0)*(height-35))
    draw.line([project(p) for p in planned[:, :2]], fill=(243, 161, 65), width=2)
    points = log['qpos'][:index+1:max(1, index//300), :2]
    clipped = np.clip(points, (x0, y0), (x1, y1))
    if len(points) > 1:
        draw.line([project(p) for p in clipped], fill=(30, 217, 240), width=2)
    x, y = project(np.clip(log['qpos'][index, :2], (x0, y0), (x1, y1)))
    draw.ellipse((x-4, y-4, x+4, y+4), fill=(239, 245, 250))
    return image


def make_chart(runs, key, title, low, high):
    pic = Image.new('RGB', (640, 230), BG)
    draw = ImageDraw.Draw(pic)
    left, top, width, height = 62, 37, 557, 150
    text(draw, (14, 4), title, 21)
    draw.rectangle((left, top, left+width, top+height), fill=(4, 12, 20), outline=(60, 81, 98))
    for value in (low, (low+high)/2, high):
        y = top+height-(value-low)/(high-low)*height
        text(draw, (4, y-7), f'{value:g}', 14, MUTED)
        draw.line((left, y, left+width, y), fill=(36, 53, 67))
    for run in runs:
        log = run['log']
        points = [(left+t/44*width, top+height-(v-low)/(high-low)*height) for t, v in zip(log['time_s'], log[key])]
        draw.line(points, fill=COLORS[run['spec']['key']], width=2)
    for t in (0, 11, 22, 33, 44):
        text(draw, (left+t/44*width-7, top+height+8), str(t), 14, MUTED)
    return pic


def coverage(run):
    times = run['log']['time_s']
    return f'{times[0]:.3f}–{times[-1]:.3f} s'


def card(runs, plan, kind, speed):
    pic = Image.new('RGB', (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(pic)
    title = 'I56: native LQR and two fixed command-allocation variants'
    text(draw, (62, 52), title, 37, WARN)
    y = 145
    lines = [
        f'Actual saved native Gazebo/DART Mars figure-eight states. Initial shaft phase −1.00 rad. Playback {speed:g}×.',
        'B retains the native LQR + momentum observer and contact feedforward. A adds total-command bounds and continuity. H also penalizes yaw and auxiliary relative-spin momentum.',
        'All three use the same unchanged raw reference trajectory and original feedback targets. The candidate changes the applied total command.',
        'Orange is the planned route / current reference. Cyan is the actual path travelled. Cameras and world XY scale are shared.',
        'Absolute tilt is measured from world up. Raw-target error compares platform up axes and excludes yaw. World-Z ω is physical angular velocity, not raw qvel[5].',
        'Only saved poses are drawn. No dynamics integration, controller, observer update, contact solve, or new forecast occurs in rendering.'
    ]
    if kind == 'outro':
        lines = [
            'Coverage is retained for every fixed case, including an early failure. A completed run is separate from passing the registered scientific screen.',
            'An ended panel holds only its final observed pose and remains marked ENDED / NO LATER OBSERVATION. Its scientific trace stops at that sample.',
            'The overview samples poses every 200 ms. Full 5 ms logs remain the source for event timing and scientific conclusions.',
            'This video is a descriptive comparison. Refer to the sealed analysis and iteration record for acceptance gates and causal limitations.'
        ]
    for line in lines:
        y = wrapped(draw, (70, y), line, 1770, 29) + 25
    y = max(y, 805)
    for run in runs:
        s = run['summary']
        label = f'{run["spec"]["label"]}: observed {coverage(run)}; completed={s.get("completed")}; failure={s.get("failure")}'
        y = wrapped(draw, (70, y), label, 1770, 24, COLORS[run['spec']['key']]) + 10
    text(draw, (70, 1035), 'Simulation evidence only | fixed case selection and media clocks were registered before outcome inspection', 22, MUTED)
    return np.asarray(pic)


def frame(runs, scenes, renderers, charts, moment, frame_index, speed, camera, bounds, planned):
    pic = Image.new('RGB', (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(pic)
    text(draw, (22, 12), f'I56 | recorded t={moment:05.2f} s | playback {speed:g}× | movie t={frame_index/FPS:05.2f} s | initial phase −1.00 rad', 28)
    samples = {}
    for column, (run, scene, renderer) in enumerate(zip(runs, scenes, renderers)):
        left = column*640
        log, spec = run['log'], run['spec']
        index = nearest(log['time_s'], moment)
        ended = moment > log['time_s'][-1]+1e-7
        text(draw, (left+15, 60), spec['label'], 25, COLORS[spec['key']])
        text(draw, (left+15, 93), 'Actual saved native Gazebo state / Mars DEM', 17, MUTED)
        pixels = draw_scene(scene, renderer, log, index, camera, planned)
        pic.paste(inset(pixels, log, index, bounds, planned), (left, 120))
        draw = ImageDraw.Draw(pic)
        if ended:
            draw.rectangle((left+8, 128, left+630, 194), fill=(87, 25, 30))
            text(draw, (left+20, 136), 'ENDED / NO LATER OBSERVATION', 26, (255, 218, 208))
            text(draw, (left+20, 169), f'Final saved pose: {log["time_s"][-1]:.3f} s', 18)
        text(draw, (left+15, 523), f'Saved t={log["time_s"][index]:.3f} s | coverage {coverage(run)}', 20)
        text(draw, (left+15, 556), f'Timed XY error {log["timed_error_mm"][index]:.1f} mm', 22)
        text(draw, (left+15, 589), f'Absolute tilt {log["absolute_tilt_deg"][index]:.2f}° | raw-target {log["raw_target_error_deg"][index]:.2f}°', 21)
        text(draw, (left+15, 622), f'World-Z ω {log["world_wz_rad_s"][index]:+.3f} rad/s | speed {log["speed_mm_s"][index]:.1f} mm/s', 20)
        command = log['input'][index]
        text(draw, (left+15, 655), 'Applied torque [N m]: '+', '.join(f'{v:+.3f}' for v in command[:3]), 19)
        text(draw, (left+15, 684), 'Applied belt [m/s]: '+', '.join(f'{v:+.3f}' for v in command[3:]), 19)
        samples[spec['key']] = dict(index=index, saved_time_s=float(log['time_s'][index]), ended=bool(ended))
    for column, chart in enumerate(charts):
        chart = chart.copy()
        d = ImageDraw.Draw(chart)
        x = 62+moment/44*557
        d.line((x, 37, x, 187), fill=(249, 248, 239), width=2)
        pic.paste(chart, (column*640, 730))
    draw = ImageDraw.Draw(pic)
    for column, run in enumerate(runs):
        text(draw, (25+column*640, 975), run['spec']['label'], 21, COLORS[run['spec']['key']])
    text(draw, (25, 1014), 'Scientific traces: all recorded 5 ms samples. Absolute tilt and raw-target error are distinct. No trace extends beyond observed coverage.', 20, MUTED)
    text(draw, (25, 1044), '4× overview poses are 200 ms apart; the fixed slow window uses 25 ms poses. A final saved pose can be held only with the ENDED label.', 18, WARN)
    return np.asarray(pic), samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--campaign-ready', action='store_true', help='Parent has sealed all three cases and authorized outcome reads')
    parser.add_argument('--mode', choices=('preview', 'render'), required=True)
    parser.add_argument('--stage', required=True, help='Fresh output folder; previous stages are never overwritten')
    args = parser.parse_args()
    if not args.campaign_ready:
        raise SystemExit('Wait for the parent campaign-ready message; then pass --campaign-ready')
    stage = HERE / args.stage
    if stage.exists():
        raise FileExistsError(f'Preserve earlier render attempt: {stage}')
    stage.mkdir()
    started = datetime.now(timezone.utc).isoformat()
    plan = read(PLAN)
    route, yaw = build_route(plan['route'])
    assert yaw == 45.0
    runs = [load_run(spec, route) for spec in plan['cases']]
    inputs = [Path(__file__), PLAN, FAMILY/'protocol.json', FAMILY/'ablation/protocol.json', DEM,
              ROOT/'src/spherical_wheel/model.py', ROOT/'src/spherical_wheel/offroad.py',
              ROOT/'src/spherical_wheel/offroad_view.py', ROOT/'src/spherical_wheel/gazebo/mars.py',
              ROOT/'src/spherical_wheel/gazebo/mars_compare.py']
    for run in runs:
        inputs.extend(run['folder']/name for name in ('artifact_manifest.json', 'precision.npz', 'rollout.npz', 'summary.json', 'source_snapshot/config/model.yaml'))
    hashes = {str(path.relative_to(ROOT)): sha(path) for path in inputs}
    snapshot = stage/'source_snapshot'
    snapshot.mkdir()
    for source in inputs:
        if source.suffix in ('.py', '.json', '.yaml') and source.stat().st_size < 250000:
            target = snapshot/source.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    # Original result helpers are archived for provenance but never executed.
    legacy = [ROOT/'results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py',
              ROOT/'results/contact_scheduling_joint_tracking_20261126/videos/render_overview.py']
    legacy_hashes = {}
    for source in legacy:
        target = snapshot/'reference_only_not_executed'/source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        legacy_hashes[str(source.relative_to(ROOT))] = sha(source)
    xy = np.array([route(t)[0] for t in np.linspace(0, 44, 401)])
    mid = .5*(xy.min(axis=0)+xy.max(axis=0))
    span = max(float(np.ptp(xy, axis=0).max())+.65, 1.4)
    bounds = (mid[0]-span/2, mid[0]+span/2, mid[1]-span/2, mid[1]+span/2)
    terrain_bounds = (mid[0]-1.2, mid[0]+1.2, mid[1]-1.2, mid[1]+1.2)
    yard = MarsYard()
    planned = np.array([[x, y, yard.height(float(x), float(y))+.006] for x, y in xy])
    camera = _camera(np.r_[mid, yard.height(*mid)+.16], **{
        'azimuth': plan['camera']['azimuth_deg'], 'elevation': plan['camera']['elevation_deg'], 'distance': plan['camera']['distance_m']})
    error_high = max(25., np.ceil(max(run['log']['timed_error_mm'].max() for run in runs)/25)*25)
    attitude_high = max(2., np.ceil(max(run['log']['raw_target_error_deg'].max() for run in runs)/2)*2)
    yaw_high = max(.5, np.ceil(max(abs(run['log']['world_wz_rad_s']).max() for run in runs)*2)/2)
    charts = [make_chart(runs, 'timed_error_mm', 'Timed XY error [mm]', 0, error_high),
              make_chart(runs, 'raw_target_error_deg', 'Raw-target up-axis error [deg]', 0, attitude_high),
              make_chart(runs, 'world_wz_rad_s', 'Physical world-Z angular rate [rad/s]', -yaw_high, yaw_high)]
    slow_start, slow_end = plan['slow_event']['recorded_window_s']
    slow_ok = all(run['log']['time_s'][0] <= slow_start and run['log']['time_s'][-1] >= slow_end-1e-7 for run in runs)
    slow_reason = None if slow_ok else 'At least one fixed case lacks observed coverage of the full preselected 28–36 s window; the window was not changed after outcomes.'
    metadata = dict(iteration='I-56', started_utc=started, user_local_date=plan['user_local_date'],
        rendering_boundary=plan['rendering_boundary'], input_sha256=hashes,
        legacy_reference_only_sha256=legacy_hashes, fps=FPS, dimensions=[WIDTH, HEIGHT],
        coverage={r['spec']['key']: dict(first_s=float(r['log']['time_s'][0]), last_s=float(r['log']['time_s'][-1]), completed=r['summary'].get('completed'), failure=r['summary'].get('failure')) for r in runs},
        shared_camera=plan['camera'], shared_world_xy_bounds=list(bounds),
        shared_plot_ranges=dict(timed_error_mm=[0, error_high], raw_target_error_deg=[0, attitude_high], world_wz_rad_s=[-yaw_high, yaw_high]),
        slow_window_eligible=bool(slow_ok), slow_window_skip_reason=slow_reason,
        no_new_physics=True, no_contact_solve=True, no_controller_observer_forecast=True, videos={})
    (stage/'declaration.json').write_text(json.dumps(metadata, indent=2)+'\n')
    scenes, renderers, schedule = [], [], []
    try:
        with ExitStack() as guards:
            for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward', 'mj_inverse', 'mj_collision', 'mj_integratePos'):
                guards.enter_context(patch.object(mujoco, name, forbidden))
            for run in runs:
                config = yaml.safe_load((run['folder']/'source_snapshot/config/model.yaml').read_text())
                scene = static_scene(config, yard, terrain_bounds, stage/'_render_scenes'/run['spec']['key'])
                scenes.append(scene)
                renderers.append(mujoco.Renderer(scene.model, height=390, width=640))
            windows = [('overview', plan['overview'])]
            if slow_ok:
                windows.append(('slow_28_36', plan['slow_event']))
            for name, settings in windows:
                speed = settings['playback_speed']
                first, last = settings['recorded_window_s']
                moments = plan['preview_recorded_times_s'] if args.mode == 'preview' and name == 'overview' else np.arange(int(round((last-first)*FPS/speed))+1)*speed/FPS+first
                if args.mode == 'preview' and name != 'overview':
                    continue
                intro, outro = card(runs, plan, 'intro', speed), card(runs, plan, 'outro', speed)
                imageio.imwrite(stage/f'{name}_intro.jpg', intro, quality=95)
                imageio.imwrite(stage/f'{name}_outro.jpg', outro, quality=95)
                writer, count = None, 0
                output = stage/f'{name}.mp4'
                try:
                    if args.mode == 'render':
                        writer = imageio.get_writer(output, fps=FPS, codec='libx264', quality=8, macro_block_size=2, ffmpeg_log_level='error')
                        for _ in range(plan['cards']['intro_frames']):
                            writer.append_data(intro)
                            count += 1
                    for moment in moments:
                        pose_frame = plan['cards']['intro_frames']+int(round((moment-first)*FPS/speed))
                        pixels, samples = frame(runs, scenes, renderers, charts, float(moment), pose_frame, speed, camera, bounds, planned)
                        if writer is not None:
                            assert count == pose_frame
                            writer.append_data(pixels)
                            count += 1
                        else:
                            imageio.imwrite(stage/f'{name}_{moment:05.2f}s.jpg', pixels, quality=95)
                        schedule.append(dict(video=name, frame=pose_frame, movie_time_s=pose_frame/FPS, recorded_time_s=float(moment), case_samples=samples))
                    if writer is not None:
                        for _ in range(plan['cards']['outro_frames']):
                            writer.append_data(outro)
                            count += 1
                finally:
                    if writer is not None:
                        writer.close()
                if args.mode == 'render':
                    expected = plan['cards']['intro_frames']+len(moments)+plan['cards']['outro_frames']
                    with imageio.get_reader(output) as movie:
                        assert movie.count_frames() == count == expected
                        info = movie.get_meta_data()
                        assert list(info['size']) == [WIDTH, HEIGHT] and abs(info['fps']-FPS) < 1e-9
                        assert abs(info['duration']-count/FPS) < .051
                        for index in (0, plan['cards']['intro_frames'], plan['cards']['intro_frames']+len(moments)//2, count-1):
                            imageio.imwrite(stage/f'{name}_decoded_{index:04d}.jpg', movie.get_data(index), quality=95)
                    metadata['videos'][name] = dict(path=str(output.relative_to(ROOT)), sha256=sha(output), bytes=output.stat().st_size, frames=count, fps=FPS, duration_s=info['duration'], recorded_window_s=[first, last], playback_speed=speed)
                print(json.dumps(dict(video=name, mode=args.mode, pose_frames=len(moments), complete=True)), flush=True)
    finally:
        for renderer in renderers:
            renderer.close()
    assert hashes == {str(path.relative_to(ROOT)): sha(path) for path in inputs}
    assert legacy_hashes == {str(path.relative_to(ROOT)): sha(path) for path in legacy}
    for row in schedule:
        for run in runs:
            sample = row['case_samples'][run['spec']['key']]
            assert sample['saved_time_s'] <= run['log']['time_s'][-1]+1e-9
            if not sample['ended']:
                assert abs(sample['saved_time_s']-row['recorded_time_s']) <= .002501
    metadata['finished_utc'] = datetime.now(timezone.utc).isoformat()
    metadata['qa'] = dict(input_hashes_unchanged=True, forbidden_dynamics_guard_active=True,
                          no_samples_beyond_coverage=True, observed_pose_time_error_at_most_s=.002501,
                          precision_ordinary_pose_agreement=True, nominal_reference_agreement=True,
                          raw_target_up_axis_error_matches_ordinary_log=True)
    (stage/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    (stage/'display_schedule.json').write_text(json.dumps(schedule, indent=2)+'\n')
    files = [p for p in stage.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    (stage/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=str(p.relative_to(stage)), sha256=sha(p), bytes=p.stat().st_size) for p in sorted(files)]), indent=2)+'\n')
    print(json.dumps(dict(stage=str(stage), coverage=metadata['coverage'], slow_window_eligible=slow_ok, slow_window_skip_reason=slow_reason, videos=metadata['videos']), indent=2))


if __name__ == '__main__':
    main()
