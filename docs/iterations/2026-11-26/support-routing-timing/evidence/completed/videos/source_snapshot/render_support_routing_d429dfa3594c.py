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


def diagnostic_panel(run, index, case_report=None):
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
    text(draw, (22, 104), 'EVALUATOR preceding 5ms mean: '+patch_text(truth[index], threshold), 20, TRUTH)
    text(draw, (22, 134), 'Last 1ms solver sample: '+patch_text(d['truth_patch_loads_N'][index], threshold), 17, (156, 180, 199))
    if case_report is not None:
        events = case_report['truth_tables']['preceding_mean']['common_paired_events']
        past = [event for event in events if event['truth_time_s'] <= now+1e-9]
        if past:
            event = past[-1]
            text(draw, (22, 166), f'EVAL matched transfer {event["truth_time_s"]:.3f}s: native {1000*event["native_signed_offset_s"]:+.0f}ms / shadow {1000*event["fast_signed_offset_s"]:+.0f}ms (− early)', 15, WARN)
    family_truth = shell_share(truth, threshold)
    line_plot(draw, (52, 220, 705, 120), times,
              [(shell_share(native), NATIVE), (shell_share(fast), FAST), (family_truth, TRUTH)], now, (0., 1.),
              'Shell-family load share (0=belt, 1=shell); unknowns remain gaps')
    bounds = d['input_bounds']
    native_demand = np.max(abs(d['native_pre_sync_command'])/bounds, axis=1)
    fast_demand = np.max(abs(d['shadow_pre_sync_command'])/bounds, axis=1)
    peak = max(1., float(np.nanmax(np.r_[native_demand, fast_demand])))
    high = np.ceil(min(peak, 6.)*2)/2
    line_plot(draw, (52, 411, 705, 125), times,
              [(native_demand, NATIVE), (fast_demand, FAST)], now, (0., high),
              f'Pre-sync candidate max |command/bound|; displayed range 0–{high:g}')
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
        ('Strict 1mm RMS / 3mm max scan: 0 of 2091 sites qualified. Failed stage retained.', WARN),
        ('Pre-control revision: 3mm RMS / 10mm max; same 2–6deg slope, feature clearance and ranking.', FG),
        (f'Selected centre ({selected["center_xy_m"][0]:.1f}, {selected["center_xy_m"][1]:.1f}) m; slope {selected["plane_slope_deg"]:.2f} deg.', FG),
        (f'Local plane-fit RMS {1000*selected["plane_rmse_m"]:.2f} mm; maximum {1000*selected["plane_max_abs_m"]:.2f} mm.', FG),
        ('Native Mars and flat each follow the full 16s out-and-back program. Every executed command stays native.', FG),
        ('Interval-mean wrench routing is SHADOW arithmetic only. No faster selector commands the robot.', WARN),
        ('Flat initialization/geometry differ. This comparison does not isolate slope as a sole cause.', FG),
        ('Original flat placement failure retained (0 native commands); corrected setup is labeled.', WARN),
    ]:
        y = wrap(draw, (1010, y), message, 550, 23, color)+22
    text(draw, (35, 1120), 'Two isolated stepping fixtures verified identical full endpoints before route collection. Simulation evidence only.', 23)
    return np.asarray(picture)


def render_frame(viewer, runs, scenes, renderers, moment, protocol, report):
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
        picture.paste(Image.fromarray(diagnostic_panel(run, index, report['cases'][run['case']])), (column*800, 535))
    draw = ImageDraw.Draw(picture)
    text(draw, (25, 1166), 'Blue: native filtered routing | orange: interval-mean SHADOW | green: preceding 5ms evaluator truth. No live improvement claimed.', 19, WARN)
    return np.asarray(picture)


def resultcard(report):
    picture = Image.new('RGB', (WIDTH, HEIGHT), BG); draw = ImageDraw.Draw(picture)
    text(draw, (40, 32), 'I46 result: retain the existing filter; no faster routing was actuated', 33, WARN)
    text(draw, (40, 97), 'Both native cases completed 16s. Same-state shadow gains/holding use interval-mean shares; all other policy inputs stay native.', 22)
    text(draw, (40, 155), 'Primary: preceding 5ms mean contact loads, scoring window [0.5,15.5)s. Last 1ms truth is a separate table.', 22)
    y = 235
    for case, label in zip(CASES, ('Selected Mars', 'Flat control (corrected setup)')):
        table = report['cases'][case]['truth_tables']['preceding_mean']
        following = report['cases'][case]['truth_tables']['next_hold_aux']['sources']
        native, fast = table['sources']['native'], table['sources']['fast']
        commands = report['cases'][case]['commands']
        text(draw, (45, y), label, 27)
        text(draw, (65, y+48), f'Routing-mismatch equivalent time: {number(native["routing_mismatch_equivalent_s"], 1000, 0)} → {number(fast["routing_mismatch_equivalent_s"], 1000, 0)} ms (native → shadow)', 24)
        text(draw, (65, y+87), f'Following-hold EVAL auxiliary: {number(following["native"]["routing_mismatch_equivalent_s"], 1000, 0)} → {number(following["fast"]["routing_mismatch_equivalent_s"], 1000, 0)} ms; no improved actuation demonstrated.', 20)
        text(draw, (65, y+122), f'Max normalized pre-sync step: {number(commands["native"]["requested"]["normalized_adjacent_step"]["maximum"], 1., 6)} → {number(commands["fast"]["requested"]["normalized_adjacent_step"]["maximum"], 1., 6)}', 21)
        y += 180
    gate = report['registered_gate']; timing = gate['paired_timing_improvement_s']
    text(draw, (40, y+20), f'{timing["n"]} common fresh transfers: median absolute timing improvement {number(timing["median"], 1000, 0)} ms (required ≥5ms).', 25, WARN)
    text(draw, (40, y+72), f'Registered gate: {"PASS" if gate["passes_all_conditions"] else "FAIL"} — {gate["decision"].replace("_", " ")}. Completion and source integrity passed separately.', 24, WARN)
    h = report['historical_causality']; old = report['cases']['historical_mars_figure8']['truth_tables']['last1ms']['sources']
    wrap(draw, (40, 785), f'HISTORICAL figure-eight, last 1ms truth only: mismatch {number(old["native"]["routing_mismatch_equivalent_s"], 1000, 0)} → {number(old["fast"]["routing_mismatch_equivalent_s"], 1000, 0)} ms. '
         f'First correcting cue {number(h["first_truth_correcting_effective_route_s"])}s follows at-bound native request {number(h["first_native_belt_A_request_saturation_s"])}s; maximum command step is unchanged.', 1510, 24)
    wrap(draw, (40, 940), 'Mismatch-equivalent time counts sampled routing disagreement; it is not a measured duration of wrong actuation over the following hold. The newer cue is not demonstrated to fix the earlier departure.', 1510, 22)
    wrap(draw, (40, 1040), 'Paired local wrench perturbations are a sensitivity check, not a physical sensor-noise model. One selected route and a flat control do not isolate slope causality or establish hardware performance.', 1510, 21)
    text(draw, (40, 1140), 'All driving shown is ACTUAL native simulation. Orange commands are SHADOW ONLY. No closed-loop improvement claim.', 24, WARN)
    return np.asarray(picture)


def scientific_plots(report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out = FAMILY/'plots'; out.mkdir(exist_ok=False)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    x = np.arange(2); labels = ['Selected Mars', 'Flat control']
    for source, shift, color, label in [('native', -.18, '#5297be', 'Native filtered'), ('fast', .18, '#df9c3c', 'Interval mean (shadow)')]:
        mismatch = [1000*report['cases'][case]['truth_tables']['preceding_mean']['sources'][source]['routing_mismatch_equivalent_s'] for case in CASES]
        axes[0, 0].bar(x+shift, mismatch, width=.36, color=color, label=label)
        step = [report['cases'][case]['commands'][source]['requested']['normalized_adjacent_step']['maximum'] for case in CASES]
        axes[1, 0].bar(x+shift, step, width=.36, color=color, label=label)
        noise = [100*report['cases'][case]['noise']['sources'][source]['label_flip_fraction'] for case in CASES]
        axes[1, 1].bar(x+shift, noise, width=.36, color=color, label=label)
    axes[0, 0].set(title='Sampled routing discrepancy vs preceding mean', ylabel='Mismatch-equivalent time (ms)', xticks=x, xticklabels=labels)
    axes[0, 0].legend(fontsize=9)
    events = report['registered_gate']['common_paired_events']
    for i, event in enumerate(events):
        values = 1000*np.array([event['native_signed_offset_s'], event['fast_signed_offset_s']])
        axes[0, 1].plot([i-.12, i+.12], values, color='#929292', linewidth=1)
        axes[0, 1].scatter(i-.12, values[0], color='#5297be', s=45)
        axes[0, 1].scatter(i+.12, values[1], color='#df9c3c', s=45)
    axes[0, 1].axhline(0, color='black', linewidth=.7)
    axes[0, 1].set(title='Same 7 truth events, matched to both estimators', ylabel='Signed onset offset (ms); negative = early', xticks=range(len(events)), xticklabels=[('M' if e['case_id']==CASES[0] else 'F')+f' {e["truth_time_s"]:.3f}s' for e in events])
    axes[0, 1].tick_params(axis='x', labelrotation=40)
    axes[1, 0].set(title='Maximum same-state candidate command step', ylabel='Infinity norm of Δcommand / input bounds', xticks=x, xticklabels=labels, yscale='log')
    axes[1, 1].set(title='Paired static local-wrench sensitivity', ylabel='Dominant-label flip fraction (%)', xticks=x, xticklabels=labels)
    for ax in axes.flat: ax.grid(axis='y', alpha=.2)
    fig.suptitle('I46: no actuated faster selector; registered benefit gate fails\nOne fresh native Mars run + corrected flat control, 15s scoring window each', fontsize=13)
    for ext in ('png', 'pdf'): fig.savefig(out/f'routing_summary.{ext}', dpi=180)
    plt.close(fig)
    folder = FAMILY/'campaign/historical_stage01'
    with np.load(folder/'precision.npz') as saved: old = {k:saved[k].copy() for k in saved.files}
    t = old['time_s']; keep = (t >= 6.4-1e-9) & (t <= 7.4+1e-9)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    for loads, color, label in [(old['native_feedback_effective_loads_N'], '#5297be', 'Native filtered'),
                                (old['shadow_used_feedback_effective_loads_N'], '#df9c3c', 'Interval-mean shadow'),
                                (old['truth_patch_loads_N'], '#359c7d', 'Last1ms evaluator truth')]:
        total = loads.sum(axis=1); values = np.full(len(t), np.nan); valid = total > .001
        values[valid] = loads[valid, 2]/total[valid]
        axes[0].plot(t[keep], values[keep], color=color, label=label)
    axes[0].set(ylabel='Belt A share', ylim=(-.05, 1.05)); axes[0].legend(fontsize=9)
    bounds = old['input_bounds']
    axes[1].plot(t[keep], (old['exact_native_requested']/bounds)[keep, 3], color='#5297be', label='Actual native requested belt A')
    axes[1].plot(t[keep], (old['exact_applied']/bounds)[keep, 3], color='#385569', linestyle='--', label='Actual native applied belt A')
    axes[1].axhline(1., color='gray', linestyle=':', linewidth=1)
    axes[1].set(ylabel='Belt A command / bound'); axes[1].legend(fontsize=9)
    for key, color, label in [('native_pre_sync_command', '#5297be', 'Native pre-sync'), ('shadow_pre_sync_command', '#df9c3c', 'SHADOW pre-sync')]:
        step = np.r_[np.nan, np.max(abs(np.diff(old[key], axis=0))/bounds[1:], axis=1)]
        axes[2].plot(t[keep], step[keep], color=color, label=label)
    axes[2].set(ylabel='Normalized candidate step', xlabel='Historical recorded time (s)'); axes[2].legend(fontsize=9)
    causal = report['historical_causality']
    for ax in axes:
        for value, color in [(causal['first_native_belt_A_request_saturation_s'], '#a94b4b'),
                              (causal['first_truth_correcting_effective_route_s'], '#ad7d21'), (6.86, '#777777')]:
            ax.axvline(value, color=color, linestyle=':', linewidth=1)
        ax.grid(alpha=.2)
    fig.suptitle('Historical event: at-bound request 6.700s, correcting cue 6.725s, departure 6.860s\nSaved last1ms truth only; no following-interval truth or closed-loop improvement inferred', fontsize=12)
    for ext in ('png', 'pdf'): fig.savefig(out/f'historical_timing.{ext}', dpi=180)
    plt.close(fig)
    with np.load(FAMILY/'campaign/runs/mars_compact_outback/precision.npz') as saved:
        mars = {k:saved[k].copy() for k in saved.files}
    times = mars['time_s']; mask = (times >= 3.85-1e-9) & (times <= 3.89+1e-9)
    fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
    for values, color, label, style in [
        (mars['native_feedback_effective_loads_N'], '#5297be', 'Native effective filtered', '-'),
        (mars['shadow_used_feedback_effective_loads_N'], '#df9c3c', 'Interval-mean SHADOW', '--'),
        (mars['truth_interval_mean_loads_N'], '#359c7d', 'Preceding5ms EVAL truth', '-.'),
        (np.r_[mars['truth_interval_mean_loads_N'][1:], np.full((1, 4), np.nan)], '#75578b', 'Following-hold EVAL auxiliary', ':')]:
        ax.plot(times[mask], shell_share(values)[mask], color=color, label=label, linestyle=style, marker='o', markersize=4)
    ax.set(title='Post-outcome explanatory zoom: same native Mars transition\nRaw shares, current-pose routing; signed onset statistics stay in canonical report',
           xlabel='Recorded time (s)', ylabel='Shell-family share', ylim=(-.05, 1.05))
    ax.grid(alpha=.2); ax.legend(fontsize=9)
    for ext in ('png', 'pdf'): fig.savefig(out/f'mars_transition_zoom.{ext}', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview-mars', action='store_true')
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--analysis-stage', default='stage01')
    parser.add_argument('--plots', action='store_true')
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
        stage = FAMILY/'analysis'/args.analysis_stage
        for row in read(stage/'artifact_manifest.json')['files']:
            assert sha(stage/row['path']) == row['sha256'], row['path']
        report = read(stage/'report.json')
        assert read(FAMILY/'review/campaign_raw_qa_stage01/report.json')['passed']
        prefix = 'preview' if args.preview else 'video'
        output, metadata = HERE/'support_routing_timing.mp4', HERE/f'{prefix}_metadata.json'
        if metadata.exists() or (not args.preview and output.exists()):
            raise FileExistsError('Preserve prior render before changing source or rerunning')
        inputs = [source, VIEWER, FAMILY/'protocol.json', FAMILY/'protocol_clarifications.json',
                  stage/'artifact_manifest.json', stage/'report.json', stage/'case_metrics.csv', stage/'paired_events.csv',
                  FAMILY/'review/campaign_raw_qa_stage01/report.json', FAMILY/'review/bridge_equivalence_stage01/report.json',
                  FAMILY/'site_selection/revision_stage02/map.png', FAMILY/'site_selection/revision_stage02/report.json',
                  FAMILY/'campaign/runs/flat_compact_outback/failure.json', FAMILY/'setup_correction_stage02/report.json']
        for run in runs:
            inputs += [run['folder']/name for name in ('artifact_manifest.json', 'precision.npz', 'rollout.npz', 'summary.json', 'provenance.json', 'terrain.json', 'source_snapshot/config/model.yaml')]
        hashes = {str(p.relative_to(ROOT)):sha(p) for p in inputs}
        if args.plots: scientific_plots(report)
        startcard, endcard = intro(protocol), resultcard(report)
        imageio.imwrite(HERE/f'{prefix}_site_intro.jpg', startcard, quality=94)
        imageio.imwrite(HERE/f'{prefix}_resultcard.jpg', endcard, quality=94)
        writer = None if args.preview else imageio.get_writer(output, fps=FPS, codec='libx264', quality=8, macro_block_size=2, ffmpeg_log_level='error')
        count, schedule = 0, []
        try:
            if writer is not None:
                for _ in range(2*FPS): writer.append_data(startcard); count += 1
            moments = [0., 7., 12., 16.] if args.preview else np.arange(0., 16.+1e-9, .05)
            for moment in moments:
                pixels = render_frame(viewer, runs, scenes, renderers, float(moment), protocol, report)
                if writer is not None: writer.append_data(pixels)
                if args.preview or abs(moment-7.) < 1e-9 or abs(moment-16.) < 1e-9:
                    imageio.imwrite(HERE/f'{prefix}_native_{moment:.3f}s.jpg', pixels, quality=94)
                schedule.append(dict(frame=count, display_time_s=float(moment), case_state_times_s={run['case']:float(run['log']['time_s'][viewer.nearest(run['log']['time_s'], moment)]) for run in runs}))
                count += 1
            if writer is not None:
                for _ in range(4*FPS): writer.append_data(endcard); count += 1
        finally:
            if writer is not None: writer.close()
        assert hashes == {str(p.relative_to(ROOT)):sha(p) for p in inputs}
        provenance = dict(iteration='I-46', scope=__doc__, render_source_sha256=sha(source), input_sha256=hashes,
            fps=FPS, dimensions=[WIDTH, HEIGHT], frames=count, actual_window_s=[0., 16.], playback_speed=1.,
            intro_seconds=2., resultcard_seconds=4., diagnostic_truth='Preceding5ms primary; last1ms and following-hold evaluation auxiliary separated.',
            shadow_actuated=False, predictions_or_scientific_reduction_recomputed=False, gazebo_physics_calls=0, controller_calls=0,
            native_cases={run['case']:str(run['folder'].relative_to(ROOT)) for run in runs},
            retained_failure='Original flat setup attempt retained with0nativecommands; corrected setup explicitly labeled.')
        if writer is not None:
            with imageio.get_reader(output) as movie:
                actual_count = movie.count_frames(); meta = movie.get_meta_data()
                for index in (0, 40, count//2, count-1):
                    imageio.imwrite(HERE/f'decoded_frame_{index:04d}.jpg', movie.get_data(index), quality=94)
            assert actual_count == count and tuple(meta['size']) == (WIDTH, HEIGHT) and meta['fps'] == FPS
            provenance['video'] = dict(path=str(output.relative_to(ROOT)), sha256=sha(output), bytes=output.stat().st_size,
                frames=actual_count, fps=meta['fps'], duration_s=meta['duration'])
        metadata.write_text(json.dumps(provenance, indent=2)+'\n')
        (HERE/f'{prefix}_display_schedule.json').write_text(json.dumps(schedule, indent=2)+'\n')
        print(json.dumps({k:v for k,v in provenance.items() if k != 'input_sha256'}, indent=2))
    finally:
        for renderer in renderers: renderer.close()


if __name__ == '__main__':
    main()
