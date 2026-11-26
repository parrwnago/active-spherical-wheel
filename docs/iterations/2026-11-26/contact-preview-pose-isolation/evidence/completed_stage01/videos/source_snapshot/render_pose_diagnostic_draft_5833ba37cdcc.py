"""Present sealed I45 geometry ablations without running their scientific pipeline.

The four panels read saved predictions and evaluator-only targets.  They never
query a mapper, acquire depth, integrate a pose or call a controller.  Fixed
early and late windows show the latest stored anchor; hindsight is explicit.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import shutil

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
ROOT = FAMILY.parents[1]
WIDTH, HEIGHT, FPS = 1600, 1200, 20
CHAPTERS = (dict(id='early_bump', start_s=6.4, stop_s=10., speed=.5),
            dict(id='late_revisit', start_s=30., stop_s=36., speed=1.))
ARMS = ('observer_cv', 'memory_cv', 'observer_actual_pose', 'memory_actual_pose')
LABELS = ('Observer plane / CV', 'Terrain memory / CV',
          'Observer plane / ACTUAL FUTURE POSE', 'Terrain memory / ACTUAL FUTURE POSE')
COLORS = {'shell_A':(110, 195, 248), 'shell_B':(154, 227, 179),
          'belt_A':(250, 182, 94), 'belt_B':(221, 150, 241)}
PLOT_LIMIT_MM = 100.
BG = (10, 20, 31)
FG = (220, 233, 244)
WARN = (248, 195, 111)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def font(size):
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)


def text(draw, xy, value, size=20, color=FG):
    draw.text(xy, str(value), font=font(size), fill=color)


def number(value, factor=1., digits=2):
    return 'n/a' if value is None else f'{factor*value:.{digits}f}'


def wrapped(draw, xy, value, width, size=18, color=FG, spacing=6):
    words, line, y = str(value).split(), '', xy[1]
    for word in words:
        candidate = (line+' '+word).strip()
        if draw.textlength(candidate, font=font(size)) > width and line:
            text(draw, (xy[0], y), line, size, color)
            y += size+spacing
            line = word
        else:
            line = candidate
    if line:
        text(draw, (xy[0], y), line, size, color)
    return y+size+spacing


def target_for(row):
    truth = row['truth']
    if truth['primary_eligible']:
        return np.asarray(truth['primary_target_m']), 'PRIMARY resolved-patch mean'
    if truth['legacy_equivalent_eligible']:
        return np.asarray(truth['legacy_target_m']), 'LEGACY equivalent mean; PRIMARY EXCLUDED'
    return None, 'NO ELIGIBLE TARGET; primary and legacy excluded'


def candidate_panel(image, arm_index, row, common_primary):
    col, line = arm_index % 2, arm_index // 2
    left, top, width, height = col*800, 104+line*460, 800, 460
    draw = ImageDraw.Draw(image)
    draw.rectangle((left+8, top+4, left+792, top+450), fill=(15, 29, 43), outline=(45, 66, 83))
    label = LABELS[arm_index]
    text(draw, (left+24, top+17), label, 24, WARN if line else FG)
    text(draw, (left+24, top+49), 'HINDSIGHT: future full configuration unavailable online' if line else
         'DEPLOYABLE INPUTS: current pose + constant velocity', 16, WARN if line else (150, 208, 233))
    plot_left, plot_top, side = left+38, top+109, 286
    cx, cy = plot_left+side/2, plot_top+side/2
    draw.rectangle((plot_left, plot_top, plot_left+side, plot_top+side), fill=(4, 12, 20), outline=(91, 112, 130))
    draw.line((cx, plot_top, cx, plot_top+side), fill=(47, 68, 84))
    draw.line((plot_left, cy, plot_left+side, cy), fill=(47, 68, 84))
    text(draw, (plot_left, plot_top-27), f'XY offset from EVALUATOR target (±{PLOT_LIMIT_MM:g}mm)', 13)
    text(draw, (plot_left, plot_top+side+9), 'World X →   |   world Y ↑   |   clipped points use triangles', 12)
    target, target_name = target_for(row)
    candidates = row['contact_preview'].get('candidates', [])
    if target is not None:
        draw.line((cx-8, cy, cx+8, cy), fill=(250, 250, 250), width=3)
        draw.line((cx, cy-8, cx, cy+8), fill=(250, 250, 250), width=3)
        for candidate in candidates:
            delta = 1000*(np.asarray(candidate['point_m'])-target)
            clipped = bool(np.any(abs(delta[:2]) > PLOT_LIMIT_MM))
            xy = np.clip(delta[:2]/PLOT_LIMIT_MM, -1., 1.)
            px, py = cx+xy[0]*side/2, cy-xy[1]*side/2
            color = COLORS[candidate['patch']]
            if clipped:
                draw.polygon([(px, py-7), (px-6, py+5), (px+6, py+5)], fill=color)
            else:
                draw.ellipse((px-6, py-6, px+6, py+6), outline=color, width=3)
    else:
        wrapped(draw, (plot_left+28, plot_top+100), 'No eligible evaluator target for offset plot', 230, 19, WARN)
    score, preview = row['contact_score'], row['contact_preview']
    if not preview['supported_surface']:
        status = 'UNKNOWN SURFACE'
    elif not score['contact_preview_complete']:
        status = 'PARTIAL / CONTACT REGION UNKNOWN'
    elif not score['nonempty_candidates']:
        status = 'COMPLETE / NO NEAR CONTACT'
    else:
        status = 'COMPLETE / CANDIDATES AVAILABLE'
    x, y = left+350, top+101
    y = wrapped(draw, (x, y), status, 420, 21, WARN if 'UNKNOWN' in status else (160, 231, 197))
    y = wrapped(draw, (x, y+6), str(preview.get('reason')), 420, 17)
    patches = ', '.join(score['candidate_patches']) or 'none'
    y = wrapped(draw, (x, y+9), 'Possible patches: '+patches, 420, 19)
    text(draw, (x, y+6), f'Geometric points {score["geometric_candidate_count"]} | ambiguous {score["candidate_ambiguous"]}', 17)
    y += 41
    text(draw, (x, y), f'Primary 3D error: {number(score["primary_nearest_candidate_error_m"], 1000)} mm', 21)
    text(draw, (x, y+32), f'Primary patch identity hit: {score["primary_patch_in_candidates"]}', 18)
    text(draw, (x, y+60), f'Four-way PRIMARY matched slot: {common_primary}', 18, WARN)
    text(draw, (x, y+92), f'Legacy 3D error: {number(score["legacy_nearest_candidate_error_m"], 1000)} mm', 18)
    wrapped(draw, (left+24, top+421), target_name, 750, 15, WARN)


def frame(rows, moment, chapter, common_keys):
    image = Image.new('RGB', (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    anchor = rows[ARMS[0]]
    now, endpoint = anchor['anchor_time_s'], anchor['future_time_s']
    text(draw, (25, 13), 'I45: does contact-preview error come from motion prediction or the surface?', 29)
    text(draw, (25, 56), f'{chapter["id"]} | display {moment:.3f}s | frozen knowledge {now:.3f}s | evaluated +200ms endpoint {endpoint:.3f}s', 21)
    key = (anchor['anchor_index'], .2)
    for index, arm in enumerate(ARMS):
        candidate_panel(image, index, rows[arm], key in common_keys)
    draw = ImageDraw.Draw(image)
    text(draw, (28, 1045), 'Same recorded run and frozen observations in all four panels. Bottom row changes only pose input to saved future qpos.', 20)
    text(draw, (28, 1080), 'White cross = saved evaluator target; colored circles = geometric candidates. 3D error includes height, plot shows XY only.', 19)
    x = 28
    for patch, color in COLORS.items():
        draw.ellipse((x, 1122, x+12, 1134), outline=color, width=3)
        text(draw, (x+22, 1116), patch, 18, color)
        x += 180
    text(draw, (28, 1160), 'OFFLINE DIAGNOSTIC ONLY: no new physics, no future depth, no controller commands and no demonstrated tracking improvement.', 19, WARN)
    return np.asarray(image)


def resultcard(report):
    image = Image.new('RGB', (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    text(draw, (40, 30), 'I45 — saved future pose is a diagnostic, not an online predictor', 32)
    text(draw, (40, 92), 'All four conditions reuse one native Gazebo/DART Mars trajectory. Live LQR and observer remain unchanged.', 23)
    primary = report['primary_common_fourway']['uniform']['all_horizons']
    legacy = report['legacy_common_fourway']['uniform']['all_horizons']
    text(draw, (40, 150), f'PRIMARY: single resolved-contact patch mean. Uniform {primary["matched_slots"]}/{primary["fixed_truth_eligible_slots"]} eligible slots matched four ways.', 24, WARN)
    text(draw, (40, 191), f'All-slot coverage uses 327 uniform queries per arm; matched errors use the same {primary["matched_slots"]} endpoints for every arm.', 22)
    y = 270
    headings = [('Condition', 45), ('Complete + nonempty', 640), ('Primary RMS / P95', 1000), ('Identity', 1380)]
    for label, x in headings:
        text(draw, (x, y), label, 20, (155, 196, 220))
    for index, arm in enumerate(ARMS):
        y += 75
        metrics = primary['arms'][arm]
        coverage = report['summary']['uniform'][arm]['all_horizons']
        text(draw, (45, y), LABELS[index], 21, WARN if index >= 2 else FG)
        text(draw, (640, y), f'{coverage["complete_nonempty"]}/327 ({100*coverage["complete_nonempty_coverage"]:.1f}%)', 22)
        text(draw, (1000, y), f'{number(metrics["rmse_m"], 1000)} / {number(metrics["p95_m"], 1000)} mm', 22)
        text(draw, (1380, y), f'{number(metrics["identity_recall"], 100, 1)}%', 22)
    y += 105
    legacy_values = ' / '.join(number(legacy['arms'][arm]['rmse_m'], 1000) for arm in ARMS)
    wrapped(draw, (40, y), f'LEGACY global equivalent target: four-way N={legacy["matched_slots"]}; RMS in the same arm order: {legacy_values} mm. This is a different target/denominator.', 1510, 22)
    dense = report['summary']['dense_bump']['memory_actual_pose']['all_horizons']
    text(draw, (40, y+100), f'Dense early bump: memory + actual future pose complete/nonempty {dense["complete_nonempty"]}/{dense["query_slots"]}. Coverage limitations remain explicit.', 22, WARN)
    decision = report['registered_decision']['decision_branch'].replace('_', ' ')
    wrapped(draw, (40, y+160), 'Registered diagnostic decision: '+decision, 1510, 26, WARN)
    wrapped(draw, (40, 1025), 'Counts resolve contacts above 0.05N; selected patch means may include weaker same-patch contacts. They are not exact physical contact points. Nearest-candidate distance does not measure force, load share or slip.', 1510, 21)
    text(draw, (40, 1132), 'HINDSIGHT UNAVAILABLE ONLINE • NO LIVE ADOPTION • NO CLOSED-LOOP IMPROVEMENT CLAIM', 24, WARN)
    return np.asarray(image)


def scientific_plots(report, directory):
    """Plot published report tables verbatim; do not rescore or rematch rows."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory.mkdir(parents=True, exist_ok=False)
    colors = ['#607c98', '#289b80', '#a88c56', '#426bb2']
    labels = ['Observer\nCV', 'Memory\nCV', 'Observer\nactual pose*', 'Memory\nactual pose*']
    primary = report['primary_common_fourway']['uniform']['all_horizons']
    summary = report['summary']['uniform']
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.6), constrained_layout=True)
    ax = axes[0, 0]
    bottom = np.zeros(4)
    for name, label, color in [('complete_nonempty', 'Complete + candidates', '#399b87'),
                               ('known_no_near_contact', 'Complete / none near', '#adb9c5'),
                               ('partial_contact_region', 'Partial contact region', '#e7b46a'),
                               ('unknown_surface', 'Unknown surface', '#32485e')]:
        values = np.array([summary[arm]['all_horizons'][name] for arm in ARMS])
        ax.bar(np.arange(4), values, bottom=bottom, label=label, color=color)
        bottom += values
    assert np.array_equal(bottom, np.full(4, 327))
    ax.set(title='Availability: all 327 uniform slots per arm', ylabel='Endpoint queries', xticks=range(4), xticklabels=labels)
    ax.legend(fontsize=8)
    ax = axes[0, 1]
    values = [1000*primary['arms'][arm]['rmse_m'] if primary['arms'][arm]['rmse_m'] is not None else np.nan for arm in ARMS]
    p95 = [1000*primary['arms'][arm]['p95_m'] if primary['arms'][arm]['p95_m'] is not None else np.nan for arm in ARMS]
    ax.bar(range(4), values, color=colors, label='RMS')
    ax.scatter(range(4), p95, color='black', marker='_', s=180, label='P95')
    ax.set(title=f'Primary common-fourway error (N={primary["matched_slots"]})', ylabel='Nearest-any-candidate distance (mm)', xticks=range(4), xticklabels=labels)
    ax.legend(fontsize=8)
    ax = axes[1, 0]
    x = np.arange(4)
    for shift, key, label, color in [(-.18, 'identity_recall', 'Patch identity recall', '#537da2'), (.18, 'ambiguity_fraction', 'Ambiguous preview', '#d3a055')]:
        values = [100*primary['arms'][arm][key] if primary['arms'][arm][key] is not None else np.nan for arm in ARMS]
        ax.bar(x+shift, values, width=.36, label=label, color=color)
    ax.set(title='Same primary common-fourway denominator', ylabel='Percent', ylim=(0, 105), xticks=range(4), xticklabels=labels)
    ax.legend(fontsize=8)
    ax = axes[1, 1]
    dense = report['summary']['dense_bump']
    values = [dense[arm]['all_horizons']['complete_nonempty'] for arm in ARMS]
    ax.bar(range(4), values, color=colors)
    ax.set(title='Dense bump: all 33 nested slots per arm', ylabel='Complete + nonempty queries', ylim=(0, 35), xticks=range(4), xticklabels=labels)
    fig.suptitle('I45: frozen surface × CV or hindsight full configuration\n* Actual future pose is unavailable online; one historical run, no control change', fontsize=13)
    for ax in axes.flat:
        ax.grid(axis='y', alpha=.18)
    for extension in ('png', 'pdf'):
        fig.savefig(directory/f'primary_and_coverage.{extension}', dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    for ax, table_name, target_label in zip(axes, ('primary_common_fourway', 'legacy_common_fourway'),
                                          ('Single resolved-contact patch mean', 'Legacy global equivalent mean')):
        table = report[table_name]['uniform']['all_horizons']
        values = [1000*table['arms'][arm]['rmse_m'] if table['arms'][arm]['rmse_m'] is not None else np.nan for arm in ARMS]
        ax.bar(range(4), values, color=colors)
        ax.set(title=f'{target_label}\nN={table["matched_slots"]} fixed four-way slots', ylabel='Nearest-any-candidate RMS (mm)', xticks=range(4), xticklabels=labels)
        ax.grid(axis='y', alpha=.18)
    fig.suptitle('Target definition matters: each table has its own explicit fixed denominator', fontsize=13)
    for extension in ('png', 'pdf'):
        fig.savefig(directory/f'primary_vs_legacy.{extension}', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default='stage01')
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--plots', action='store_true')
    args = parser.parse_args()
    stage = FAMILY/'campaign'/args.stage
    sentinel = stage/'artifact_manifest.json'
    if not sentinel.exists():
        raise FileNotFoundError('Wait for sealed main campaign before rendering')
    for name, entry in read(sentinel).items():
        assert sha(stage/name) == entry['sha256'], name
    report = read(stage/'report.json')
    if not report['integrity_passed']:
        raise ValueError('No scientific presentation of failed-integrity outputs')
    rows = read(stage/'predictions.json')
    anchors = read(stage/'anchor_records.json')
    table = {}
    for row in rows:
        if abs(row['horizon_s']-.2) < 1e-10:
            table.setdefault(row['anchor_index'], {})[row['arm']] = row
    anchor_times = np.array([a['anchor_time_s'] for a in anchors])
    common_keys = {tuple(key) for key in report['primary_common_fourway']['all_unique']['0.20s']['matched_keys']}
    prefix = 'preview' if args.preview else 'video'
    output = HERE/'contact_pose_diagnostic.mp4'
    metadata = HERE/f'{prefix}_metadata.json'
    if metadata.exists() or (not args.preview and output.exists()):
        raise FileExistsError('Preserve prior render attempt before changing sources')
    snapshot = HERE/'source_snapshot'
    snapshot.mkdir(exist_ok=True)
    source = Path(__file__).resolve()
    saved_source = snapshot/f'{source.stem}_{sha(source)[:12]}.py'
    if not saved_source.exists():
        shutil.copy2(source, saved_source)
    assert sha(saved_source) == sha(source)
    inputs = [source, FAMILY/'protocol.json', sentinel, stage/'report.json', stage/'predictions.json', stage/'anchor_records.json']
    before = {str(p.relative_to(ROOT)):sha(p) for p in inputs}
    if args.plots:
        scientific_plots(report, FAMILY/'plots')
    schedule, count = [], 0
    writer = None if args.preview else imageio.get_writer(output, fps=FPS, codec='libx264', quality=8, macro_block_size=2, ffmpeg_log_level='error')
    try:
        for chapter in CHAPTERS:
            n = round((chapter['stop_s']-chapter['start_s'])/chapter['speed']*FPS)
            moments = [chapter['start_s'], chapter['stop_s']] if args.preview else chapter['start_s']+np.arange(n)*chapter['speed']/FPS
            for moment in moments:
                anchor_i = int(np.searchsorted(anchor_times, moment+1e-9, side='right')-1)
                anchor = anchors[anchor_i]
                assert anchor['anchor_time_s'] <= moment+1e-9
                selected = table[anchor['anchor_index']]
                assert set(selected) == set(ARMS)
                picture = frame(selected, moment, chapter, common_keys)
                if writer is not None:
                    writer.append_data(picture)
                if args.preview or count in (0, n-1):
                    imageio.imwrite(HERE/f'{prefix}_{chapter["id"]}_{moment:.3f}s.jpg', picture, quality=93)
                schedule.append(dict(frame=count, display_time_s=float(moment), anchor_time_s=anchor['anchor_time_s'],
                                     anchor_index=anchor['anchor_index'], endpoint_time_s=selected[ARMS[0]]['future_time_s'], chapter=chapter['id']))
                count += 1
        card = resultcard(report)
        imageio.imwrite(HERE/f'{prefix}_resultcard.jpg', card, quality=93)
        if writer is not None:
            for _ in range(4*FPS):
                writer.append_data(card)
                count += 1
    finally:
        if writer is not None:
            writer.close()
    assert before == {str(p.relative_to(ROOT)):sha(p) for p in inputs}
    info = dict(scope=__doc__, render_source_sha256=sha(source), input_sha256=before,
                chapters=CHAPTERS, horizon_s=.2, pixel_offset_limits_mm=[-PLOT_LIMIT_MM, PLOT_LIMIT_MM],
                frames=count, fps=FPS, dimensions=[WIDTH, HEIGHT],
                scientist_report_read_only=True, recomputed_predictions=False,
                hindsight_future_pose_explicit=True, new_physics=0, new_controller_calls=0,
                diagnostic_target='Primary single resolved-contact patch mean; otherwise separately marked legacy target; no target if neither eligible')
    if not args.preview:
        with imageio.get_reader(output) as video:
            decoded_count = video.count_frames()
            meta = video.get_meta_data()
            for index in (0, count//2, count-1):
                imageio.imwrite(HERE/f'decoded_frame_{index:04d}.jpg', video.get_data(index), quality=93)
        assert decoded_count == count and tuple(meta['size']) == (WIDTH, HEIGHT) and meta['fps'] == FPS
        info['video'] = dict(path=str(output.relative_to(ROOT)), sha256=sha(output), bytes=output.stat().st_size,
                             frames=decoded_count, fps=meta['fps'], duration_s=meta['duration'])
    (HERE/f'{prefix}_display_schedule.json').write_text(json.dumps(schedule, indent=2)+'\n')
    metadata.write_text(json.dumps(info, indent=2)+'\n')
    print(json.dumps({k:v for k, v in info.items() if k != 'input_sha256'}, indent=2))


if __name__ == '__main__':
    main()
