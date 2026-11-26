"""I46 actual saved native Mars/flat motion with non-actuated routing diagnostics.

Rendering sets saved qpos/qvel for visual context and calls forward kinematics
only.  It does not run Gazebo, integrate dynamics, call a controller, locate
contacts, perturb observations or rerun the canonical scientific reduction.
"""
from pathlib import Path
from functools import lru_cache
import argparse
import hashlib
import json
import shutil
import types

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
ROOT = FAMILY.parents[1]
VIEWER = ROOT/'results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py'
CASES = ('mars_compact_outback', 'flat_compact_outback')
FOLDERS = ('mars_compact_outback', 'flat_compact_outback_setup_corrected')
TITLES = ('ACTUAL native Mars: selected natural-ground area', 'ACTUAL native flat: setup-corrected control')
WIDTH, HEIGHT, FPS = 1600, 1200, 20
BG, FG, WARN = (10, 20, 31), (221, 235, 246), (246, 193, 109)
NATIVE, FAST, TRUTH = (110, 193, 244), (242, 178, 94), (147, 226, 183)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


@lru_cache(maxsize=32)
def font(size):
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)


def text(draw, xy, value, size=20, color=FG):
    draw.text(xy, str(value), font=font(size), fill=color)


def wrap(draw, xy, value, width, size=20, color=FG):
    line, y = '', xy[1]
    for word in str(value).split():
        joined = (line+' '+word).strip()
        if line and draw.textlength(joined, font=font(size)) > width:
            text(draw, (xy[0], y), line, size, color); y += size+7; line = word
        else:
            line = joined
    if line:
        text(draw, (xy[0], y), line, size, color)
    return y+size+7


def number(value, scale=1., digits=3):
    return 'n/a' if value is None else f'{scale*value:.{digits}f}'


def load_case(case, folder_name):
    folder = FAMILY/'campaign/runs'/folder_name
    if not (folder/'artifact_manifest.json').is_file():
        raise FileNotFoundError('Wait for sealed native raw outputs')
    for row in read(folder/'artifact_manifest.json')['files']:
        assert sha(folder/row['path']) == row['sha256'], row['path']
    with np.load(folder/'precision.npz') as saved:
        exact = {k:saved[k].copy() for k in saved.files}
    with np.load(folder/'rollout.npz') as saved:
        log = {k:saved[k].copy() for k in saved.files}
    log.update(time_s=exact['time_s'], qpos=exact['exact_qpos'], qvel=exact['exact_qvel'],
               input=exact['exact_applied'], reference_xy=exact['exact_reference_xy'])
    log['planned_xy'] = log['reference_xy'][::max(1, len(log['time_s'])//200)]
    log['goal_error_mm'] = 1000*np.linalg.norm(log['qpos'][:, :2]-log['reference_xy'], axis=1)
    q = log['qpos'][:, 3:7]
    log['absolute_tilt_deg'] = np.degrees(np.arccos(np.clip(1-2*np.sum(q[:, 1:3]**2, axis=1)/np.sum(q*q, axis=1), -1., 1.)))
    return dict(case=case, folder=folder, exact=exact, log=log, provenance=read(folder/'provenance.json'), summary=read(folder/'summary.json'))


def shell_share(loads, minimum=.001):
    loads = np.asarray(loads)
    total = loads.sum(axis=-1)
    result = np.full(total.shape, np.nan)
    valid = np.isfinite(loads).all(axis=-1) & (total > minimum)
    result[valid] = loads[valid, :2].sum(axis=-1)/total[valid]
    return result


def patch_text(loads, minimum=.001):
    loads = np.asarray(loads)
    if not np.isfinite(loads).all() or loads.sum() <= minimum:
        return 'unknown / weak / unloaded'
    shares = loads/loads.sum()
    if shares.max() < .8:
        return 'mixed support'
    return ('shell A', 'shell B', 'belt A', 'belt B')[int(shares.argmax())]


def line_plot(draw, rectangle, times, series, now, ylim, title):
    left, top, width, height = rectangle
    draw.rectangle((left, top, left+width, top+height), fill=(4, 12, 20), outline=(63, 84, 101))
    text(draw, (left, top-26), title, 17)
    lo, hi = ylim
    for value in (lo, hi):
        text(draw, (left+4, top+4 if value == hi else top+height-20), f'{value:.1f}', 13, (144, 166, 185))
    for values, color in series:
        points = []
        # Display direct saved values; split unavailable observations into gaps.
        for t, value in zip(times[::2], values[::2]):
            if not np.isfinite(value):
                if len(points) > 1: draw.line(points, fill=color, width=2)
                points = []; continue
            points.append((left+t/16*width, top+height-(np.clip(value, lo, hi)-lo)/(hi-lo)*height))
        if len(points) > 1:
            draw.line(points, fill=color, width=2)
    x = left+np.clip(now, 0., 16.)/16*width
    draw.line((x, top, x, top+height), fill=(248, 247, 238), width=2)
    for t in (0, 4, 8, 12, 16):
        text(draw, (left+t/16*width-6, top+height+4), str(t), 13)


def diagnostic_panel(run, index):
    picture = Image.new('RGB', (800, 610), BG); draw = ImageDraw.Draw(picture)
    d, times = run['exact'], run['log']['time_s']
    weight = run['provenance']['weight_N']; threshold = max(.05, .05*weight)
    native = d['native_feedback_effective_loads_N']
    fast = d['shadow_used_feedback_effective_loads_N']
    truth = d['truth_interval_mean_loads_N']
    now = float(times[index])
    text(draw, (22, 9), 'CURRENT POSE / SAME NATIVE HISTORY', 21)
    text(draw, (22, 44), 'Native filtered: '+patch_text(native[index]), 20, NATIVE)
    text(draw, (22, 74), 'Interval-mean SHADOW: '+patch_text(fast[index]), 20, FAST)
    text(draw, (22, 104), 'EVALUATOR preceding5ms mean: '+patch_text(truth[index], threshold), 20, TRUTH)
    text(draw, (22, 134), 'Last1ms solver sample: '+patch_text(d['truth_patch_loads_N'][index], threshold), 17, (156, 180, 199))
    family_truth = shell_share(truth, threshold)
    line_plot(draw, (52, 205, 705, 132), times,
              [(shell_share(native), NATIVE), (shell_share(fast), FAST), (family_truth, TRUTH)], now, (0., 1.),
              'Shell-family load share (0=belt, 1=shell); unknowns remain gaps')
    bounds = d['input_bounds']
    native_demand = np.max(abs(d['native_pre_sync_command'])/bounds, axis=1)
    fast_demand = np.max(abs(d['shadow_pre_sync_command'])/bounds, axis=1)
    peak = max(1., float(np.nanmax(np.r_[native_demand, fast_demand])))
    high = np.ceil(min(peak, 6.)*2)/2
    line_plot(draw, (52, 411, 705, 125), times,
              [(native_demand, NATIVE), (fast_demand, FAST)], now, (0., high),
              f'Pre-sync candidate max |command/bound|; displayed range0–{high:g}')
    text(draw, (30, 572), 'Orange is NOT actuated. Later native sync/heading/fade/clipping still apply.', 16, WARN)
    return np.asarray(picture)


def intro(protocol):
    picture = Image.new('RGB', (WIDTH, HEIGHT), BG); draw = ImageDraw.Draw(picture)
    text(draw, (35, 28), 'I46: support-routing timing — selected Mars area, unchanged native controller', 30)
    source = FAMILY/'site_selection/revision_stage02/map.png'
    map_image = ImageOps.contain(Image.open(source).convert('RGB'), (950, 930))
    picture.paste(map_image, (25, 145)); draw = ImageDraw.Draw(picture)
    selected = protocol['map_selection']['selected']
    y = 150
    for message, color in [
        ('Geometry-only site selection, before controller outcomes.', FG),
        ('Strict1mm RMS /3mm max scan:0 of2091 sites qualified. Failed stage retained.', WARN),
        ('Pre-control revision:3mm RMS /10mm max; same2–6deg slope, feature clearance and ranking.', FG),
        (f'Selected centre({selected["center_xy_m"][0]:.1f}, {selected["center_xy_m"][1]:.1f})m; slope{selected["plane_slope_deg"]:.2f}deg.', FG),
        (f'Local plane-fit RMS{1000*selected["plane_rmse_m"]:.2f}mm; maximum{1000*selected["plane_max_abs_m"]:.2f}mm.', FG),
        ('Native Mars and flat each follow the full16s out-and-back program. Every executed command stays native.', FG),
        ('Interval-mean wrench routing is SHADOW arithmetic only. No faster selector commands the robot.', WARN),
        ('Flat initialization/geometry differ. This comparison does not isolate slope as a sole cause.', FG),
        ('Original flat placement failure retained (0 native commands); corrected setup is labeled.', WARN),
    ]:
        y = wrap(draw, (1010, y), message, 550, 23, color)+22
    text(draw, (35, 1120), 'Two isolated stepping fixtures verified identical full endpoints before route collection. Simulation evidence only.', 23)
    return np.asarray(picture)


def render_frame(viewer, runs, scenes, renderers, moment, protocol):
    picture = Image.new('RGB', (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(picture)
    text(draw, (25, 10), f'I46 ACTUAL SAVED NATIVE MOTION | recorded t={moment:.3f}s | native LQR + observer + synchronization unchanged', 24)
    for column, run in enumerate(runs):
        log = run['log']; index = viewer.nearest(log['time_s'], moment)
        plant, renderer = scenes[column], renderers[column]
        center = np.r_[np.array(protocol['map_selection']['selected']['center_xy_m']), float(np.median(log['qpos'][:, 2]))]
        option = mujoco.MjvOption(); option.geomgroup[3:] = 0
        pixels = viewer._draw(plant, renderer, log, index, viewer._camera(center, 112., -25., 1.05), option, False)
        pixels = viewer.xy_inset(pixels, log, index, bounds=tuple(protocol['cases'][column]['visual_bounds']))
        pixels = viewer._caption(pixels, TITLES[column], [
            f'Exact saved t{log["time_s"][index]:.3f}s | timed error{log["goal_error_mm"][index]:.2f}mm | absolute tilt{log["absolute_tilt_deg"][index]:.2f}deg',
            'Orange diagnostic commands below are never applied to this robot.'],
            'Saved native Gazebo/DART states re-rendered; no new physics or alternative-controller trajectory.')
        picture.paste(Image.fromarray(pixels), (column*800, 55))
        picture.paste(Image.fromarray(diagnostic_panel(run, index)), (column*800, 535))
    draw = ImageDraw.Draw(picture)
    text(draw, (25, 1166), 'Blue: native filtered routing | orange: interval-mean SHADOW | green: preceding5ms evaluator truth. No live improvement claimed.', 19, WARN)
    return np.asarray(picture)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview-mars', action='store_true')
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--analysis-stage', default='stage01')
    args = parser.parse_args()
    protocol = read(FAMILY/'protocol.json')
    viewer = types.ModuleType('_I46_saved_state_viewer'); viewer.__file__ = str(VIEWER)
    exec(compile(VIEWER.read_text(), str(VIEWER), 'exec'), viewer.__dict__)
    viewer.HERE = HERE
    source = Path(__file__).resolve(); snapshot = HERE/'source_snapshot'; snapshot.mkdir(exist_ok=True)
    preserved = snapshot/f'{source.stem}_{sha(source)[:12]}.py'
    if not preserved.exists(): shutil.copy2(source, preserved)
    selected_cases = list(zip(CASES, FOLDERS))[:1] if args.preview_mars else list(zip(CASES, FOLDERS))
    runs = [load_case(*item) for item in selected_cases]
    scenes, renderers = [], []
    try:
        for column, run in enumerate(runs):
            config = viewer.yaml.safe_load((run['folder']/'source_snapshot/config/model.yaml').read_text())
            bounds = tuple(protocol['cases'][column]['visual_bounds'])
            folder = HERE/'_render_scenes'/run['case']
            if column == 0:
                plant = viewer.replay_plant(config, viewer.MarsYard(), bounds, folder)
            else:
                plant = viewer.TerrainPlant(config, viewer.Terrain.from_spec(read(run['folder']/'terrain.json')), folder, visual_bounds=bounds)
            scenes.append(plant); renderers.append(mujoco.Renderer(plant.model, height=480, width=800))
        if args.preview_mars:
            run = runs[0]; index = viewer.nearest(run['log']['time_s'], 7.)
            option = mujoco.MjvOption(); option.geomgroup[3:] = 0
            center = np.r_[protocol['map_selection']['selected']['center_xy_m'], run['log']['qpos'][index, 2]]
            pixels = viewer._draw(scenes[0], renderers[0], run['log'], index, viewer._camera(center, 112., -25., 1.05), option, False)
            imageio.imwrite(HERE/'preview_mars_actual_7s.jpg', pixels, quality=94)
            imageio.imwrite(HERE/'preview_mars_diagnostics_7s.jpg', diagnostic_panel(run, index), quality=94)
            imageio.imwrite(HERE/'preview_site_intro.jpg', intro(protocol), quality=94)
            (HERE/'mars_preview_provenance.json').write_text(json.dumps(dict(render_source_sha256=sha(source), viewer_sha256=sha(VIEWER),
                precision_sha256=sha(run['folder']/'precision.npz'), simulation_calls=0, controller_calls=0, recorded_time_s=float(run['log']['time_s'][index])), indent=2)+'\n')
            return
        raise RuntimeError('Final scientific endcard integration awaits sealed canonical analysis; use --preview-mars for context now')
    finally:
        for renderer in renderers: renderer.close()


if __name__ == '__main__':
    main()
