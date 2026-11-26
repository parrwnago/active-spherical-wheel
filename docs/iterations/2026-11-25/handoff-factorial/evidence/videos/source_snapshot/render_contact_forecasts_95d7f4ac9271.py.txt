"""I40 saved-state presentation: native trajectories and private contact forecasts.

No physics integration, new forecast, or controller command occurs here.
Render only after the campaign and root-selected forecast example are complete.
"""
from pathlib import Path
import argparse
import hashlib
import json

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw
import yaml

from spherical_wheel.offroad import TerrainPlant
from spherical_wheel.offroad_view import _camera, _caption, _draw, _font, _marker
from spherical_wheel.terrain import Terrain
from spherical_wheel.gazebo.mars import MarsYard, DEM
from spherical_wheel.gazebo.mars_compare import replay_plant


HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
ROOT = FAMILY.parents[1]
RUNS = FAMILY / 'campaign/runs'
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
COLORS = ((63, 197, 166), (91, 161, 242), (244, 175, 66), (220, 112, 186))
FPS = 20
OVERVIEW_CASES = (('flat_x', 1.), ('flat_outback', 1.), ('mars_figure8_p090', 2.))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_read(path):
    return json.loads(Path(path).read_text())


def nearest(times, instant):
    i = min(int(np.searchsorted(times, instant)), len(times) - 1)
    return i - 1 if i and abs(times[i - 1] - instant) <= abs(times[i] - instant) else i


def tilt(quaternions):
    q = np.asarray(quaternions)
    return np.degrees(np.arccos(np.clip(
        1 - 2 * np.sum(q[..., 1:3] ** 2, axis=-1) / np.sum(q * q, axis=-1), -1, 1)))


def load_case(case):
    folder = RUNS / case
    provenance = json_read(folder / 'provenance.json')
    for name, digest in provenance['generated_sha256'].items():
        assert sha(folder / name) == digest, (case, name)
    with np.load(folder / 'rollout.npz') as saved:
        log = {k: saved[k].copy() for k in saved.files}
    with np.load(folder / 'precision.npz') as saved:
        exact = {k: saved[k].copy() for k in saved.files}
    log.update(time_s=exact['time_s'], qpos=exact['exact_qpos'], qvel=exact['exact_qvel'],
               input=exact['exact_applied'], reference_xy=exact['exact_reference_xy'])
    log['planned_xy'] = log['reference_xy'][::max(1, len(log['time_s']) // 200)]
    log['goal_error_mm'] = np.linalg.norm(log['qpos'][:, :2] - log['reference_xy'], axis=1) * 1000
    log['speed_mm_s'] = np.linalg.norm(log['qvel'][:, :2], axis=1) * 1000
    log['absolute_tilt_deg'] = tilt(log['qpos'][:, 3:7])
    log['reference_tilt_deg'] = tilt(log['reference_quat'])
    return {'case': case, 'folder': folder, 'provenance': provenance,
            'summary': json_read(folder / 'summary.json'), 'log': log, 'exact': exact}


def bounds_for(log, padding=.35):
    points = np.vstack((log['qpos'][:, :2], log['reference_xy']))
    return (float(points[:, 0].min() - padding), float(points[:, 0].max() + padding),
            float(points[:, 1].min() - padding), float(points[:, 1].max() + padding))


def scene(run, suffix='', private_plane=None):
    config = yaml.safe_load((run['folder'] / 'source_snapshot/config/model.yaml').read_text())
    folder = HERE / '_render_scenes' / (run['case'] + suffix)
    bounds = bounds_for(run['log'])
    if private_plane is not None:
        normal = np.asarray(private_plane['normal'], float)
        point = np.asarray(private_plane['calibrated_point_m'], float)
        if normal[2] <= .1:
            raise ValueError('Viewer requires a locally upward plane; preserve unsupported case')
        slope = -normal[:2] / normal[2]
        offset = float(point[2] - slope @ point[:2])
        terrain = Terrain(name='PRIVATE_estimated_plane', features=[{
            'type': 'plane', 'slope': slope.tolist(), 'offset': offset}])
        plant = TerrainPlant(config, terrain, folder, visual_bounds=bounds)
    elif run['summary']['contact_forecast_case']['case']['terrain'] == 'marsyard2021':
        terrain = MarsYard()
        plant = replay_plant(config, terrain, bounds, folder)
    else:
        terrain = Terrain.from_spec(json_read(run['folder'] / 'terrain.json'))
        plant = TerrainPlant(config, terrain, folder, visual_bounds=bounds)
    return plant


def frame_times(end, speed):
    times = np.arange(0., end + 1e-9, speed / FPS)
    if not len(times) or abs(times[-1] - end) > 1e-8:
        times = np.r_[times, end]
    return times


def support_text(loads, cutoff=.05):
    parts = [f'{name.replace("_", " ")} {float(load):.1f}N'
             for name, load in zip(LABELS, loads) if load > cutoff]
    return ' + '.join(parts) if parts else 'wheel unloaded'


def xy_inset(frame, log, index, bounds=None):
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image, 'RGBA')
    width, height = image.size
    size = 220 if width < 1000 else 260
    left, top = width - size - 15, height - size - 44
    draw.rounded_rectangle((left, top, left + size, top + size), radius=6,
                           fill=(8, 17, 26, 225))
    draw.text((left + 8, top + 7), 'Fixed world XY / planned path', font=_font(12), fill=(230, 240, 247))
    if bounds is None:
        bounds = bounds_for(log, .02)
    x0, x1, y0, y1 = bounds
    span = max(x1 - x0, y1 - y0, .04)
    cx, cy = .5 * (x0 + x1), .5 * (y0 + y1)
    def point(xy):
        return (left + size / 2 + (float(xy[0]) - cx) / span * (size - 30),
                top + size / 2 + 10 - (float(xy[1]) - cy) / span * (size - 40))
    for values, color in ((log['reference_xy'], (245, 165, 62, 245)),
                          (log['qpos'][:index + 1, :2], (61, 207, 216, 255))):
        stride = max(1, len(values) // 300)
        points = [point(p) for p in values[::stride]]
        if len(points) > 1:
            draw.line(points, fill=color, width=2)
    for value, color in ((log['reference_xy'][index], (245, 165, 62, 255)),
                         (log['qpos'][index, :2], (61, 207, 216, 255))):
        x, y = point(value)
        draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill=color)
    draw.text((left + 8, top + size - 20), f'Orange: reference | cyan: robot | width {span*1000:.0f}mm',
              font=_font(10), fill=(202, 220, 231))
    return np.asarray(image)


def chart_strip(run, index, width, height=190):
    log = run['log']
    image = Image.new('RGB', (width, height), (10, 20, 30))
    draw = ImageDraw.Draw(image, 'RGBA')
    end = float(log['time_s'][-1])
    for column, (key, title, threshold) in enumerate((
            ('goal_error_mm', 'Distance from reference (mm)', 5.),
            ('absolute_tilt_deg', 'Platform tilt: actual / reference (deg)', None))):
        left, top = 45 + column * width / 2, 35
        w, h = width / 2 - 65, height - 66
        maximum = max(float(np.max(log[key])) * 1.1, 10.)
        def point(t, value):
            return left + w * float(t) / max(end, .001), top + h - h * float(value) / maximum
        draw.text((left - 30, 6), title, font=_font(13), fill=(225, 237, 247))
        for fraction in (0., .5, 1.):
            y = maximum * fraction
            draw.line([point(0, y), point(end, y)], fill=(170, 190, 210, 45))
            draw.text((left - 32, point(0, y)[1] - 6), f'{y:.0f}', font=_font(10), fill=(186, 206, 219))
        if threshold is not None:
            draw.line([point(0, threshold), point(end, threshold)], fill=(170, 196, 151, 180))
        stride = max(1, index // 300)
        if index > 0:
            draw.line([point(t, v) for t, v in zip(log['time_s'][:index + 1:stride], log[key][:index + 1:stride])],
                      fill=(61, 207, 216, 255), width=3)
            if key == 'absolute_tilt_deg':
                draw.line([point(t, v) for t, v in zip(log['time_s'][:index + 1:stride], log['reference_tilt_deg'][:index + 1:stride])],
                          fill=(245, 197, 76, 255), width=2)
        draw.line([point(log['time_s'][index], 0), point(log['time_s'][index], maximum)], fill=(230, 239, 245, 100))
        draw.text((left, height - 24), f'0                         simulation time                         {end:g}s',
                  font=_font(10), fill=(185, 207, 223))
    return np.asarray(image)


def verify_video(path, expected_frames, dimensions, last_name):
    with imageio.get_reader(path) as reader:
        metadata = reader.get_meta_data()
        count = reader.count_frames()
        decoded = reader.get_data(count - 1)
    assert count == expected_frames
    assert tuple(metadata['size']) == tuple(dimensions)
    assert metadata['fps'] == FPS
    imageio.imwrite(HERE / last_name, decoded, quality=92)
    return {'path': str(path.relative_to(ROOT)), 'sha256': sha(path), 'bytes': path.stat().st_size,
            'dimensions': list(metadata['size']), 'fps': FPS, 'frames': count,
            'video_duration_s': metadata['duration'], 'decoded_final_verified': True}


def render_overview(preview=False):
    target = HERE / 'native_lqr_trajectory_overview.mp4'
    if target.exists():
        raise FileExistsError('Preserve prior video; use a new rendering stage')
    width, view_height = 1280, 720
    runs = [load_case(case) for case, _ in OVERVIEW_CASES]
    writer = None if preview else imageio.get_writer(target, fps=FPS, codec='libx264',
        quality=8, macro_block_size=2, ffmpeg_log_level='error')
    frames, chapters = 0, []
    try:
        for chapter, (run, (_, speed)) in enumerate(zip(runs, OVERVIEW_CASES), 1):
            plant = scene(run)
            saved = plant.capture()
            log = run['log']
            end = float(log['time_s'][-1])
            option = mujoco.MjvOption(); option.geomgroup[3:] = 0
            terrain = run['summary']['contact_forecast_case']['case']['terrain']
            moments = frame_times(end, speed)
            if preview:
                moments = np.array([end])
            try:
                with mujoco.Renderer(plant.model, height=view_height, width=width) as renderer:
                    for moment in moments:
                        i = nearest(log['time_s'], moment)
                        camera = _camera(np.r_[log['qpos'][i, :2], log['qpos'][i, 2]], 112., -25., 1.35)
                        image = _draw(plant, renderer, log, i, camera, option, False)
                        lines = [
                            f'Chapter {chapter}/3 | t {log["time_s"][i]:.3f}s / {end:g}s recorded | playback {speed:g}x',
                            f'XY error from reference {log["goal_error_mm"][i]:.2f}mm | XY speed {log["speed_mm_s"][i]:.2f}mm/s',
                            f'ABS tilt {log["absolute_tilt_deg"][i]:.2f}deg | reference {log["reference_tilt_deg"][i]:.2f}deg | relative {np.degrees(log["attitude_error_rad"][i]):.2f}deg',
                            'Actual support (evaluation): ' + support_text(run['exact']['truth_patch_loads_N'][i]),
                            'Observer support: ' + support_text(log['estimated_support_loads_N'][i]),
                            'Same native 5ms LQR/feedforward | contact forecast NEVER changes these commands',
                            ('ERC2021 Mars yard DEM + rock visuals; actual Gazebo/DART motion' if terrain == 'marsyard2021'
                             else 'Flat-plane controlled path; actual Gazebo/DART motion')]
                        if not run['summary']['completed']:
                            lines.append(f'RUN STOPPED: {run["summary"]["failure"]}; showing recorded interval only')
                        image = _caption(image, f'I40 native LQR: {run["case"]}', lines,
                            'Saved Gazebo states rendered in MuJoCo | cyan: robot trail | orange: planned reference | no new physics')
                        image = xy_inset(image, log, i)
                        frame = np.concatenate((image, chart_strip(run, i, width)), axis=0)
                        if writer is not None:
                            writer.append_data(frame)
                        frames += 1
                    prefix = 'overview_preview' if preview else 'overview'
                    imageio.imwrite(HERE / f'{prefix}_{run["case"]}_final.jpg', frame, quality=92)
            finally:
                plant.restore(saved)
            chapters.append({'case_id': run['case'], 'playback_speed': speed,
                'recorded_interval_s': [0., end], 'completed': run['summary']['completed'],
                'failure': run['summary']['failure'], 'terrain': terrain,
                'ending_goal_error_mm': float(log['goal_error_mm'][-1]),
                'ending_xy_speed_mm_s': float(log['speed_mm_s'][-1])})
    finally:
        if writer is not None:
            writer.close()
    if preview:
        return {'preview_only': True, 'chapters': chapters}
    return {'video': verify_video(target, frames, (width, view_height + 190), 'overview_decoded_final.jpg'),
            'chapters': chapters,
            'scope': 'Three saved trajectories with the SAME unchanged native controller; chapter speeds differ explicitly. This is not a controller A/B comparison.'}


def load_selected_forecast(selection):
    run = load_case(selection['case_id'])
    folder = run['folder']
    provenance = json_read(folder / 'forecast_provenance.json')
    for name, digest in provenance['generated_sha256'].items():
        assert sha(folder / name) == digest, name
    candidates = [row for row in json_read(folder / 'forecasts.json')['rows']
                  if row['success'] and abs(row['anchor_time_s'] - selection['anchor_time_s']) < 1e-8]
    if 'selection_kind' in selection:
        candidates = [row for row in candidates if row['selection_kind'] == selection['selection_kind']]
    if not candidates:
        raise ValueError('Root-selected forecast row is missing or failed')
    selected = candidates[0]
    forecast = selected['forecast']
    times = np.asarray(forecast['time_s'], float)
    assert len(times) == 41 and abs(times[-1] - times[0] - .2) < 1e-8
    assert run['log']['time_s'][-1] >= times[-1] - 1e-8
    np.testing.assert_array_equal(np.asarray(forecast['qpos'])[0], run['exact']['exact_qpos'][selected['anchor_index']])
    return run, selected


def load_chart(loads, times, index, threshold, width, private=False, maximum=None):
    height = 206
    image = Image.new('RGB', (width, height), (9, 19, 29)); draw = ImageDraw.Draw(image, 'RGBA')
    left, top, w, h = 42, 40, width - 62, 135
    maximum = max(22., float(np.max(loads)) * 1.1) if maximum is None else float(maximum)
    def point(t, value):
        return left + w * float(t) / .2, top + h - h * float(value) / maximum
    draw.text((13, 6), 'Predicted patch loads (N)' if private else 'Actual patch loads (N), evaluation only',
              font=_font(14), fill=(226, 239, 248))
    for k, name in enumerate(LABELS):
        draw.text((width - 440 + k * 110, 7), name.replace('_', ' '), font=_font(12), fill=COLORS[k])
    for tick in (0., .05, .1, .2):
        x = point(tick, 0)[0]
        draw.line([(x, top), (x, top + h)], fill=(237, 205, 117, 120))
        draw.text((x - 12, top + h + 5), f'+{tick*1000:.0f}ms', font=_font(11), fill=(221, 227, 230))
    draw.line([point(0, threshold), point(.2, threshold)], fill=(180, 202, 159, 100))
    for k in range(4):
        draw.line([point(t, value) for t, value in zip(times, loads[:, k])], fill=COLORS[k] + (85,), width=1)
        if index:
            draw.line([point(t, value) for t, value in zip(times[:index + 1], loads[:index + 1, k])],
                      fill=COLORS[k] + (255,), width=3)
    draw.line([point(times[index], 0), point(times[index], maximum)], fill=(244, 247, 250, 220), width=2)
    draw.text((5, top - 5), f'{maximum:.0f}', font=_font(10), fill=(194, 212, 226))
    draw.text((5, top + h - 8), '0', font=_font(10), fill=(194, 212, 226))
    return np.asarray(image)


def render_forecast(selection_path, preview=False):
    target = HERE / 'contact_forecast_vs_gazebo.mp4'
    if target.exists():
        raise FileExistsError('Preserve prior video; use a new rendering stage')
    selection = json_read(selection_path)
    run, row = load_selected_forecast(selection)
    forecast = row['forecast']; anchor = float(row['anchor_time_s'])
    ft = np.asarray(forecast['time_s'], float); leads = ft - anchor
    actual_indices = np.array([nearest(run['log']['time_s'], t) for t in ft])
    actual = {key: value[actual_indices] for key, value in run['log'].items()
              if isinstance(value, np.ndarray) and value.ndim and len(value) == len(run['log']['time_s'])}
    predicted = {'time_s': ft, 'qpos': np.asarray(forecast['qpos']), 'qvel': np.asarray(forecast['qvel']),
        'reference_xy': actual['reference_xy'].copy(), 'planned_xy': actual['reference_xy'].copy()}
    private_plant = scene(run, '_private', forecast['metadata']['frozen_plane'])
    predicted['ground_height_m'] = np.array([private_plant.terrain.height(*p[:2]) for p in predicted['qpos']])
    actual['planned_xy'] = actual['reference_xy'].copy()
    actual_plant = scene(run, '_actual_forecast')
    plants, logs = (actual_plant, private_plant), (actual, predicted)
    loads = (run['exact']['truth_patch_loads_N'][actual_indices], np.asarray(forecast['patch_loads_N']))
    load_axis_maximum = max(22., max(float(np.max(values)) for values in loads) * 1.1)
    points = (run['exact']['truth_patch_points_m'][actual_indices], np.asarray(forecast['patch_points_m'], float))
    threshold = max(.05, .05 * float(run['provenance']['weight_N']))
    anchor_ref_tilt = float(actual['reference_tilt_deg'][0])
    width, height, speed = 960, 720, .025
    bounds = bounds_for({'qpos': np.vstack((actual['qpos'], predicted['qpos'])),
                         'reference_xy': np.vstack((actual['reference_xy'], predicted['reference_xy']))}, .015)
    saved = [plant.capture() for plant in plants]
    option = mujoco.MjvOption(); option.geomgroup[3:] = 0
    renderers = [mujoco.Renderer(plant.model, height=height, width=width) for plant in plants]
    camera = _camera(np.r_[actual['qpos'][0, :2], .2], 112., -25., 1.2)
    moments = frame_times(.2, speed)
    moments = np.r_[np.zeros(FPS), moments, np.full(FPS, .2)]
    if preview:
        moments = np.array([0., .05, .1, .2])
    writer = None if preview else imageio.get_writer(target, fps=FPS, codec='libx264',
        quality=8, macro_block_size=2, ffmpeg_log_level='error')
    saved_leads = set()
    try:
        for number, lead in enumerate(moments):
            i = nearest(leads, lead)
            panels = []
            for side, (plant, renderer, log) in enumerate(zip(plants, renderers, logs)):
                error = np.linalg.norm(log['qpos'][i, :2] - log['reference_xy'][i]) * 1000
                velocity = np.linalg.norm(log['qvel'][i, :2]) * 1000
                absolute = float(tilt(log['qpos'][i, 3:7]))
                ref = anchor_ref_tilt if side else float(actual['reference_tilt_deg'][i])
                image = _draw(plant, renderer, log, i, camera, option, False)
                for p, load, color in zip(points[side][i], loads[side][i], COLORS):
                    if load > threshold and np.isfinite(p).all():
                        _marker(renderer.scene, p, np.r_[np.asarray(color) / 255., 1.], .006)
                image = renderer.render()
                lines = [f'{run["case"]} | anchor {anchor:.3f}s | lead +{leads[i]*1000:.0f}ms | {speed:g}x + endpoint holds',
                    selection.get('label', 'Root-selected diagnostic example; see event-selection report'),
                    f'Goal error {error:.3f}mm | XY speed {velocity:.2f}mm/s',
                    f'ABS tilt {absolute:.2f}deg | '+('anchor ref ' if side else 'actual ref ')+f'{ref:.2f}deg',
                    ('Private support: ' if side else 'Actual support: ') + support_text(loads[side][i], threshold),
                    ('Frozen observer-fitted plane | fine5ms feedback on PRIVATE predicted states' if side else
                     'Recorded Gazebo state/contact evaluation | unchanged native5ms feedback')]
                if side:
                    residual = float(row['calibration']['normal_load_residual_N'])
                    fit = 'within tolerance' if row['calibration']['load_fit_within_tolerance'] else 'OUTSIDE tolerance'
                    lines.append(f'Anchor load fit: residual {residual:+.3f}N, {fit}')
                    lines.append('Private forces: final1ms solver step, before endpoint-state integration')
                else:
                    lines.append('Raw5ms patch loads shown; event classification uses15ms stable dwell')
                image = _caption(image, 'PRIVATE MODEL FORECAST' if side else 'ACTUAL GAZEBO / DART', lines,
                    'Same policy, different state/model future | predicted contacts are NOT measurements | saved states only; no new physics')
                image = xy_inset(image, log, i, bounds)
                panels.append(np.concatenate((image, load_chart(loads[side], leads, i, threshold, width,
                                                                 bool(side), load_axis_maximum)), axis=0))
            frame = np.concatenate(panels, axis=1)
            if writer is not None:
                writer.append_data(frame)
            lead_ms = int(round(leads[i] * 1000))
            if lead_ms in (0, 50, 100, 200) and lead_ms not in saved_leads:
                prefix = 'forecast_preview' if preview else 'forecast'
                imageio.imwrite(HERE / f'{prefix}_lead_{lead_ms:03d}ms.jpg', frame, quality=92)
                saved_leads.add(lead_ms)
    finally:
        if writer is not None:
            writer.close()
        for renderer in renderers:
            renderer.close()
        for plant, original in zip(plants, saved):
            plant.restore(original)
    record = {'selection': selection, 'anchor_time_s': anchor,
        'forecast_interval_s': [anchor, anchor + .2], 'playback_speed': speed,
        'endpoint_holds_video_s': 1., 'horizon_markers_s': [.05, .1, .2],
        'scope': 'Actual recorded Gazebo future versus stored PRIVATE estimated-plane forecast under the same native5ms policy; not two live controllers.',
        'private_reference_display': 'Private panel displays the known anchor reference tilt; future cloned policy reference quaternions were not stored. Actual panel displays actual logged reference.',
        'force_timing': forecast['metadata']['contact_force_timing'],
        'shared_load_axis_maximum_N': load_axis_maximum,
        'future_private_force_offset_s': forecast['metadata']['advanced_contact_force_offset_s']}
    if not preview:
        record['video'] = verify_video(target, len(moments), (width * 2, height + 206), 'forecast_decoded_final.jpg')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('overview', 'forecast', 'all'), default='all')
    parser.add_argument('--selection', type=Path, help='Root-approved example selection JSON; required for forecast')
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    if args.mode in ('forecast', 'all') and args.selection is None:
        parser.error('--selection is required; example selection must be declared explicitly')
    if args.selection is not None:
        args.selection = args.selection.resolve()
    HERE.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve(); digest = sha(source)
    snapshot = HERE / 'source_snapshot' / f'render_contact_forecasts_{digest[:12]}.py'
    snapshot.parent.mkdir(exist_ok=True)
    if not snapshot.exists():
        snapshot.write_bytes(source.read_bytes())
    assert sha(snapshot) == digest
    record = {'iteration': 'I-40', 'render_source_snapshot': str(snapshot.relative_to(ROOT)),
              'render_source_sha256': digest, 'no_new_physics': True, 'preview_only': args.preview}
    if args.mode in ('overview', 'all'):
        record['overview'] = render_overview(args.preview)
    if args.mode in ('forecast', 'all'):
        record['forecast'] = render_forecast(args.selection, args.preview)
    paths = [source, snapshot, FAMILY / 'protocol.json']
    cases = [case for case, _ in OVERVIEW_CASES] if args.mode in ('overview', 'all') else []
    if args.selection is not None:
        paths.append(args.selection)
        case = json_read(args.selection)['case_id']
        cases.append(case)
        paths += [RUNS / case / name for name in ('forecasts.json', 'forecast_provenance.json', 'anchor_selection.json')]
    for case in set(cases):
        paths += [RUNS / case / name for name in ('precision.npz', 'rollout.npz', 'summary.json', 'provenance.json', 'terrain.json', 'source_snapshot/config/model.yaml')]
    if 'mars_figure8_p090' in cases:
        paths.append(DEM)
    record['input_source_sha256'] = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    record['stills'] = {p.name: sha(p) for p in HERE.glob('*.jpg')}
    name = f'{args.mode}_'+('preview' if args.preview else 'video')+'_metadata.json'
    target = HERE / name
    if target.exists():
        raise FileExistsError('Preserve prior render metadata stage')
    target.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({k: v for k, v in record.items() if k not in ('input_source_sha256', 'stills')}, indent=2))


if __name__ == '__main__':
    main()
