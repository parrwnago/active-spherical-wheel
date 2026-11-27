"""I58 conditional B/P overview: BOTH +.30 and +.90 starts, no phase selection."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw
import yaml

HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
ROOT = FAMILY.parents[1]
LEAD = HERE/'render_comparison.py'
LEAD_SHA = 'f321be09c74e89bca9991eb0fb53cf9995b2e8b58eb3b969df24888af981c046'
PLAN = HERE/'broader_presentation_plan.json'
WIDTH, HEIGHT, FPS = 1920, 1080, 20


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def helper():
    if sha(LEAD) != LEAD_SHA:
        raise ValueError('Frozen lead helper changed; do not silently rebind its paths')
    spec = importlib.util.spec_from_file_location('_i58_frozen_lead_display', LEAD)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def card(m, plan, runs=None, chapter=None):
    picture = Image.new('RGB', (WIDTH, HEIGHT), m.BG)
    draw = ImageDraw.Draw(picture)
    m.text(draw, (65, 60), 'I58: both additional starting phases, complete B/P comparisons', 36, m.WARN)
    if chapter is not None:
        m.text(draw, (70, 255), f'Starting shaft phase {chapter["phase_rad"]:+.2f} rad', 56)
        m.wrapped(draw, (75, 390), 'Both cases replay the complete fixed 0–44 s window at 4×. Early-ended observations remain visible and labeled.', 1750, 35)
    else:
        y = 190
        for line in (
            'Two fixed chapters: +0.30 rad, then +0.90 rad. All four registered B/P cases are retained; there is no favorable-phase selection.',
            'B is native LQR + momentum observer/contact feedforward. P preserves native local modeled balance response with hard total-command amplitude caps and the same secondary soft command-change cost.',
            'Reference trajectory, feedback targets, contact model, observer and secondary travel/readiness/native/change/effort weights remain the registered design. P has zero yaw/momentum weights; old rate thresholds are diagnostic only.',
            'Every panel draws actual saved native Gazebo states on the Mars DEM. Orange is reference, cyan is actual travel. Common camera, world XY scale and metric axes span all four cases.',
            'Absolute world-up tilt, raw-reference up-axis error and physical world-Z angular velocity are shown separately. Static kinematics only: no new physics, controller, observer update or forecast.',
            'A completed run is separate from passing the registered scientific gates. Compare each phase independently; this video makes no general adoption claim.'
        ):
            y = m.wrapped(draw, (75, y), line, 1760, 28)+24
    if runs is not None:
        y = 790
        for run in runs:
            label = f'{run["phase_key"]} {run["spec"]["label"]}: observed {m.coverage(run)}; completed={run["summary"].get("completed")}; failure={run["summary"].get("failure")}'
            y = m.wrapped(draw, (75, y), label, 1760, 23, m.COLORS[run['spec']['key']])+8
    m.text(draw, (75, 1030), 'Conditional broader comparison | all selected cases retained | full 5 ms scientific logs remain authoritative', 22, m.MUTED)
    return np.asarray(picture)


def inset(m, pixels, log, index, bounds, planned):
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image, 'RGBA')
    left, top, width, height = 730, 280, 210, 170
    draw.rounded_rectangle((left, top, left+width, top+height), radius=5, fill=(8, 17, 26, 225))
    m.text(draw, (left+7, top+5), 'Fixed world XY', 14)
    x0, x1, y0, y1 = bounds
    def point(p):
        return (left+12+(p[0]-x0)/(x1-x0)*(width-24), top+30+(y1-p[1])/(y1-y0)*(height-42))
    draw.line([point(p) for p in planned[:, :2]], fill=(243, 161, 65), width=2)
    path = log['qpos'][:index+1:max(1, index//300), :2]
    path = np.clip(path, (x0, y0), (x1, y1))
    if len(path) > 1:
        draw.line([point(p) for p in path], fill=(30, 217, 240), width=2)
    x, y = point(np.clip(log['qpos'][index, :2], (x0, y0), (x1, y1)))
    draw.ellipse((x-4, y-4, x+4, y+4), fill=(239, 245, 250))
    return image


def frame(m, chapter, runs, scenes, renderers, charts, moment, movie_frame, camera, bounds, planned):
    picture = Image.new('RGB', (WIDTH, HEIGHT), m.BG)
    draw = ImageDraw.Draw(picture)
    m.text(draw, (24, 12), f'I58 BOTH BROADER STARTS | phase {chapter["phase_rad"]:+.2f} rad | recorded t={moment:05.2f} s | 4× | movie t={movie_frame/FPS:05.2f} s', 28)
    samples = {}
    for column, (run, scene, renderer) in enumerate(zip(runs, scenes, renderers)):
        left = column*960
        log, spec = run['log'], run['spec']
        index = m.nearest(log['time_s'], moment)
        ended = moment > log['time_s'][-1]+1e-7
        m.text(draw, (left+20, 62), spec['label'], 28, m.COLORS[spec['key']])
        m.text(draw, (left+20, 98), 'Actual saved native Gazebo state / Mars DEM / common camera', 19, m.MUTED)
        pixels = m.draw_scene(scene, renderer, log, index, camera, planned)
        picture.paste(inset(m, pixels, log, index, bounds, planned), (left, 130))
        draw = ImageDraw.Draw(picture)
        if ended:
            draw.rectangle((left+10, 140, left+950, 210), fill=(87, 25, 30))
            m.text(draw, (left+25, 147), 'ENDED / NO LATER OBSERVATION', 29, (255, 218, 208))
            m.text(draw, (left+25, 184), f'Final saved pose {log["time_s"][-1]:.3f} s', 20)
        m.text(draw, (left+20, 617), f'Saved t={log["time_s"][index]:.3f} s | coverage {m.coverage(run)} | timed XY error {log["timed_error_mm"][index]:.1f} mm', 23)
        m.text(draw, (left+20, 652), f'Absolute tilt {log["absolute_tilt_deg"][index]:.2f}° | raw-reference error {log["raw_target_error_deg"][index]:.2f}° | world-Z ω {log["world_wz_rad_s"][index]:+.3f} rad/s', 22)
        command = log['input'][index]
        m.text(draw, (left+20, 687), 'Applied torque [N m]: '+', '.join(f'{v:+.3f}' for v in command[:3])+' | belt [m/s]: '+', '.join(f'{v:+.3f}' for v in command[3:]), 20)
        samples[spec['key']] = dict(index=index, saved_time_s=float(log['time_s'][index]), ended=bool(ended))
    for column, image in enumerate(charts):
        image = image.copy()
        d = ImageDraw.Draw(image)
        x = 62+moment/44*557
        d.line((x, 37, x, 187), fill=(249, 248, 239), width=2)
        picture.paste(image, (640*column, 735))
    draw = ImageDraw.Draw(picture)
    m.text(draw, (25, 980), 'B: native LQR + observer', 23, m.COLORS['B'])
    m.text(draw, (985, 980), 'P: balance priority + soft change', 23, m.COLORS['P'])
    m.text(draw, (25, 1020), '4× overview: 200 ms pose spacing. Full 5 ms traces, stopped at last observation. No physics or estimator execution during rendering.', 21, m.MUTED)
    m.text(draw, (25, 1050), 'Both +0.30 and +0.90 starts are shown in fixed order; local modeled balance preservation is not a stability/friction guarantee.', 19, m.WARN)
    return np.asarray(picture), samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--broader-ready', action='store_true', help='Root authorized reads after the lead passed and all four broader jobs ran/sealed')
    parser.add_argument('--mode', choices=('preview', 'render'), required=True)
    parser.add_argument('--stage', required=True)
    args = parser.parse_args()
    if not args.broader_ready:
        raise SystemExit('Wait for the root all-four-broader-ready message')
    stage = HERE/args.stage
    if stage.exists():
        raise FileExistsError('Preserve earlier broader presentation attempts')
    stage.mkdir()
    m, plan = helper(), read(PLAN)
    route, yaw = m.build_route(plan['route'])
    assert yaw == 45.
    allruns, byphase = [], {}
    for chapter in plan['chapters']:
        runs = [m.load_run(spec, route) for spec in chapter['cases']]
        for run in runs:
            run['phase_key'] = chapter['phase_key']
        byphase[chapter['phase_key']] = runs
        allruns.extend(runs)
    paths = [Path(__file__), PLAN, LEAD, FAMILY/'protocol.json', m.DEM,
             ROOT/'src/spherical_wheel/offroad.py', ROOT/'src/spherical_wheel/offroad_view.py',
             ROOT/'src/spherical_wheel/model.py', ROOT/'src/spherical_wheel/gazebo/mars.py',
             ROOT/'src/spherical_wheel/gazebo/mars_compare.py']
    for run in allruns:
        paths.extend(run['folder']/name for name in ('artifact_manifest.json', 'precision.npz', 'rollout.npz', 'summary.json', 'source_snapshot/config/model.yaml'))
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    snap = stage/'source_snapshot'
    for path in paths:
        if path.suffix in ('.py', '.json', '.yaml') and path.stat().st_size < 250000:
            target = snap/path.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    xy = np.array([route(t)[0] for t in np.linspace(0, 44, 401)])
    mid = .5*(xy.min(axis=0)+xy.max(axis=0))
    span = max(float(np.ptp(xy, axis=0).max())+.65, 1.4)
    bounds = (mid[0]-span/2, mid[0]+span/2, mid[1]-span/2, mid[1]+span/2)
    terrain_bounds = (mid[0]-1.2, mid[0]+1.2, mid[1]-1.2, mid[1]+1.2)
    yard = m.MarsYard()
    planned = np.array([[x, y, yard.height(float(x), float(y))+.006] for x, y in xy])
    camera = m._camera(np.r_[mid, yard.height(*mid)+.16], plan['camera']['azimuth_deg'],
                       plan['camera']['elevation_deg'], plan['camera']['distance_m'])
    maximum = lambda key: max(float(np.max(abs(run['log'][key]))) for run in allruns)
    highs = dict(timed_error_mm=max(25., np.ceil(maximum('timed_error_mm')/25)*25),
        raw_target_error_deg=max(2., np.ceil(maximum('raw_target_error_deg')/2)*2),
        world_wz_rad_s=max(.5, np.ceil(maximum('world_wz_rad_s')*2)/2))
    metadata = dict(iteration='I-58', started_utc=datetime.now(timezone.utc).isoformat(),
        scope=plan['scope'], input_sha256=hashes, phases=[c['phase_key'] for c in plan['chapters']],
        cases=[run['phase_key']+'_'+run['spec']['key'] for run in allruns],
        coverage={run['phase_key']+'_'+run['spec']['key']:dict(first_s=float(run['log']['time_s'][0]), last_s=float(run['log']['time_s'][-1]), completed=run['summary'].get('completed'), failure=run['summary'].get('failure')) for run in allruns},
        fps=FPS, dimensions=[WIDTH, HEIGHT], playback_speed=4., recorded_window_per_chapter_s=[0,44],
        shared_camera=plan['camera'], shared_world_xy_bounds=list(bounds), shared_plot_highs=highs,
        no_new_physics=True, no_contact_solve=True, no_controller_observer_forecast=True,
        no_favorable_phase_selection=True)
    (stage/'declaration.json').write_text(json.dumps(metadata, indent=2)+'\n')
    writer, count, schedule = None, 0, []
    intro = card(m, plan, allruns)
    imageio.imwrite(stage/'intro.jpg', intro, quality=95)
    output = stage/'broader_overview.mp4'
    try:
        if args.mode == 'render':
            writer = imageio.get_writer(output, fps=FPS, codec='libx264', quality=8, macro_block_size=2, ffmpeg_log_level='error')
            for _ in range(plan['intro_frames']):
                writer.append_data(intro)
                count += 1
        with ExitStack() as guards:
            for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward', 'mj_inverse', 'mj_collision', 'mj_integratePos'):
                guards.enter_context(m.patch.object(mujoco, name, m.forbidden))
            for chapter_index, chapter in enumerate(plan['chapters']):
                runs = byphase[chapter['phase_key']]
                charts = [m.make_chart(runs, 'timed_error_mm', 'Timed XY error [mm]', 0, highs['timed_error_mm']),
                    m.make_chart(runs, 'raw_target_error_deg', 'Raw-reference up-axis error [deg]', 0, highs['raw_target_error_deg']),
                    m.make_chart(runs, 'world_wz_rad_s', 'Physical world-Z angular rate [rad/s]', -highs['world_wz_rad_s'], highs['world_wz_rad_s'])]
                phase_card = card(m, plan, runs, chapter)
                imageio.imwrite(stage/(chapter['phase_key']+'_card.jpg'), phase_card, quality=95)
                if writer is not None:
                    for _ in range(plan['chapter_card_frames']):
                        writer.append_data(phase_card)
                        count += 1
                scenes, renderers = [], []
                try:
                    for run in runs:
                        config = yaml.safe_load((run['folder']/'source_snapshot/config/model.yaml').read_text())
                        scene = m.static_scene(config, yard, terrain_bounds, stage/'_render_scenes'/(chapter['phase_key']+'_'+run['spec']['key']))
                        scenes.append(scene)
                        renderers.append(mujoco.Renderer(scene.model, height=470, width=960))
                    moments = plan['preview_recorded_times_s'] if args.mode == 'preview' else np.arange(221)*.2
                    for moment in moments:
                        expected = plan['intro_frames']+chapter_index*(plan['chapter_card_frames']+221)+plan['chapter_card_frames']+int(round(moment/.2))
                        pixels, samples = frame(m, chapter, runs, scenes, renderers, charts, float(moment), expected, camera, bounds, planned)
                        if writer is not None:
                            assert count == expected
                            writer.append_data(pixels)
                            count += 1
                        else:
                            imageio.imwrite(stage/f'{chapter["phase_key"]}_{moment:05.2f}s.jpg', pixels, quality=95)
                        schedule.append(dict(frame=expected, movie_time_s=expected/FPS,
                            phase_key=chapter['phase_key'], recorded_time_s=float(moment), case_samples=samples))
                finally:
                    for renderer in renderers:
                        renderer.close()
                print(json.dumps(dict(phase=chapter['phase_key'], complete=True, mode=args.mode)), flush=True)
        if writer is not None:
            outro = card(m, plan, allruns)
            for _ in range(plan['outro_frames']):
                writer.append_data(outro)
                count += 1
    finally:
        if writer is not None:
            writer.close()
    if args.mode == 'render':
        assert count == plan['expected_total_frames']
        with imageio.get_reader(output) as movie:
            assert movie.count_frames() == count
            info = movie.get_meta_data()
            assert list(info['size']) == [WIDTH, HEIGHT] and abs(info['fps']-FPS) < 1e-9
            assert abs(info['duration']-count/FPS) < .051
            for index in (0, 60, 281, 301, 522, 581):
                imageio.imwrite(stage/f'decoded_{index:04d}.jpg', movie.get_data(index), quality=95)
        metadata['video'] = dict(path=str(output.relative_to(ROOT)), sha256=sha(output), bytes=output.stat().st_size,
                                frames=count, fps=FPS, duration_s=info['duration'])
    assert hashes == {str(p.relative_to(ROOT)): sha(p) for p in paths}
    for row in schedule:
        for run in byphase[row['phase_key']]:
            sample = row['case_samples'][run['spec']['key']]
            assert sample['saved_time_s'] <= run['log']['time_s'][-1]+1e-9
            if not sample['ended']:
                assert abs(sample['saved_time_s']-row['recorded_time_s']) <= .002501
    metadata['finished_utc'] = datetime.now(timezone.utc).isoformat()
    metadata['qa'] = dict(input_hashes_unchanged=True, all_four_cases_retained=True,
                         observed_pose_clock_error_at_most_s=.002501, no_observation_extended=True)
    (stage/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    (stage/'display_schedule.json').write_text(json.dumps(schedule, indent=2)+'\n')
    (stage/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=str(p.relative_to(stage)), sha256=sha(p), bytes=p.stat().st_size) for p in sorted(stage.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json']), indent=2)+'\n')


if __name__ == '__main__':
    main()
