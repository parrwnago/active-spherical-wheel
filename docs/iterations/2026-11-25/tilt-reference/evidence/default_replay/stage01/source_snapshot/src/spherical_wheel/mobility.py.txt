"""Multidirectional off-road evaluation. Run with python -m spherical_wheel.mobility."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np

from .model import REPO_ROOT, load_config
from .offroad import TerrainPlant
from .offroad_control import OffroadParams
from .offroad_experiments import Workspace, run_offroad, save_run
from .terrain import Terrain, random_waves

BOUNDS = (-1.8, 1.8, -1.8, 1.8)
PHASES = (.30, .90, 2.20, -1.)
SEEDS = (7, 23, 41)
# Engineering acceptance targets declared before collecting results, not hardware limits.
TARGETS = {'rms_m': .05, 'p95_m': .10, 'max_m': .20, 'final_m': .05,
           'attitude_deg': 8., 'saturated_fraction': .10, 'closure_m': .05}


class MobilityPath:
    """C2 position references, with exact velocity and acceleration, at rest at both ends."""
    def __init__(self, kind='circle', duration_s=36., radius_m=.45, clockwise=False,
                 start_s=1., origin=None):
        if kind not in ('circle', 'figure8', 'lateral', 'square', 'spokes'):
            raise ValueError(kind)
        if duration_s <= 0 or radius_m <= 0:
            raise ValueError('Positive duration and radius required')
        self.kind, self.duration_s, self.radius_m = kind, float(duration_s), float(radius_m)
        self.clockwise, self.start_s = bool(clockwise), float(start_s)
        self.origin = np.array(origin if origin is not None else
                               ((radius_m, 0.) if kind == 'circle' else (0., 0.)), float)
        self.direction = np.array([1., 0.])  # Legacy summary projections are removed below.

    @staticmethod
    def blend(u):
        return (10*u**3-15*u**4+6*u**5, 30*u**2-60*u**3+30*u**4,
                60*u-180*u**2+120*u**3)

    def waypoints(self):
        r = self.radius_m
        if self.kind == 'lateral':
            return np.array([[0, 0], [0, r], [0, -r], [0, r], [0, -r], [0, 0.]])
        if self.kind == 'square':
            return np.array([[0, 0], [r, 0], [r, r], [-r, r], [-r, -r], [0, -r], [0, 0]])
        points = [np.zeros(2)]
        for angle in np.arange(8)*np.pi/4:
            points.extend([r*np.array([np.cos(angle), np.sin(angle)]), np.zeros(2)])
        return np.array(points)

    def __call__(self, t):
        u = float(np.clip((t-self.start_s)/self.duration_s, 0., 1.))
        if self.kind in ('circle', 'figure8'):
            s, v, a = self.blend(u)
            sign = -1. if self.clockwise else 1.
            theta, omega, alpha = sign*2*np.pi*s, sign*2*np.pi*v/self.duration_s, sign*2*np.pi*a/self.duration_s**2
            r = self.radius_m
            if self.kind == 'circle':
                p = r*np.array([np.cos(theta), np.sin(theta)])
                d = r*np.array([-np.sin(theta), np.cos(theta)])
                dd = -p
                p += self.origin-np.array([r, 0.])
            else:
                p = self.origin+r*np.array([np.sin(theta), .5*np.sin(2*theta)])
                d = r*np.array([np.cos(theta), np.cos(2*theta)])
                dd = r*np.array([-np.sin(theta), -2*np.sin(2*theta)])
            return p, d*omega, dd*omega**2+d*alpha
        points = self.waypoints()
        count = len(points)-1
        k = min(int(u*count), count-1)
        s, v, a = self.blend(u*count-k)
        segment = self.duration_s/count
        delta = points[k+1]-points[k]
        return self.origin+points[k]+s*delta, v*delta/segment, a*delta/segment**2

    def spec(self):
        samples = [self(t) for t in np.linspace(self.start_s, self.start_s+self.duration_s, 2001)]
        return dict(kind=self.kind, duration_s=self.duration_s, radius_m=self.radius_m,
                    clockwise=self.clockwise, start_s=self.start_s, origin=self.origin.tolist(),
                    sampled_peak_speed_m_s=float(max(np.linalg.norm(row[1]) for row in samples)),
                    sampled_peak_acceleration_m_s2=float(max(np.linalg.norm(row[2]) for row in samples)))


def terrain_for(kind, seed=7):
    """Two-dimensional ground spanning the entire path, with no flat entry corridor."""
    features, patches = [], []
    if kind in ('hills', 'rough', 'slick', 'bank'):
        features.append(random_waves(seed, .008 if kind == 'rough' else .005, (.65, 1.6), components=12))
    if kind == 'bank':
        features.append({'type': 'plane', 'slope': [0., float(np.tan(np.deg2rad(6.)))]})
    if kind == 'bumps':
        rng = np.random.default_rng(seed)
        for x in (-.4, 0., .4):
            for y in (-.4, 0., .4):
                features.append({'type': 'gaussian', 'amplitude': float(rng.uniform(-.008, .014)),
                                 'center': [x, y], 'sigma': [.15, .18]})
    if kind == 'slick':
        patches = [{'mu': .22, 'x': [-.15, .15], 'edge': .03}]
    return Terrain(features, patches, name=f'mobility_{kind}_{seed}')


CASES = {
    'flat_circle': ('flat', 'circle', 36., False),
    'hills_circle_ccw': ('hills', 'circle', 36., False),
    'hills_circle_cw': ('hills', 'circle', 36., True),
    'rough_figure8': ('rough', 'figure8', 40., False),
    'rough_side_to_side': ('rough', 'lateral', 36., False),
    'bank_circle': ('bank', 'circle', 40., False),
    'bumps_square': ('bumps', 'square', 48., False),
    'slick_figure8': ('slick', 'figure8', 40., False),
    'hills_eight_directions': ('hills', 'spokes', 64., False),
    'pushed_circle': ('hills', 'circle', 36., False),
}


def evaluate(log, summary, path):
    """Time-aligned tracking gates; geometric shape alone cannot pass a stationary wheel."""
    error = np.linalg.norm(log['qpos'][:, :2]-log['reference_xy'], axis=1)
    moving = (log['time_s'] >= path.start_s) & (log['time_s'] <= path.start_s+path.duration_s)
    p95 = float(np.percentile(error[moving], 95)) if moving.any() else float('inf')
    closure = float(np.linalg.norm(log['qpos'][-1, :2]-log['qpos'][0, :2]))
    heading = np.degrees(np.unwrap(log['heading_rad'])-log['heading_rad'][0])
    gates = {
        'completed': summary['completed'],
        'rms': summary['tracking']['rms_error_while_moving_m'] <= TARGETS['rms_m'],
        'p95': p95 <= TARGETS['p95_m'],
        'max': summary['tracking']['max_error_m'] <= TARGETS['max_m'],
        'final': summary['tracking']['final_error_m'] <= TARGETS['final_m'],
        'closure': closure <= TARGETS['closure_m'],
        'attitude': summary['attitude']['max_error_from_reference_deg'] <= TARGETS['attitude_deg'],
        'saturation': summary['actuation']['saturated_time_fraction'] <= TARGETS['saturated_fraction'],
        'settled': summary['settled_at_end'],
    }
    extra = {'p95_moving_error_m': p95, 'loop_closure_m': closure,
             'max_heading_excursion_deg': float(np.max(np.abs(heading))),
             'heading_within_10_deg': bool(np.max(np.abs(heading)) <= 10.),
             'gates': gates, 'passed': bool(all(gates.values()))}
    velocity = np.array([path(t)[1] for t in log['time_s']])
    active = np.linalg.norm(velocity, axis=1) > .01
    direction = np.floor((np.arctan2(velocity[:, 1], velocity[:, 0])+np.pi/8) % (2*np.pi)/(np.pi/4)).astype(int)
    extra['directional_tracking'] = {
        str(deg): {'samples': int(np.sum(active & (direction == i))),
                   'rms_m': float(np.sqrt(np.mean(error[active & (direction == i)]**2)))
                   if np.any(active & (direction == i)) else None}
        for i, deg in enumerate(range(0, 360, 45))}
    if path.kind == 'circle':
        centre = path.origin-np.array([path.radius_m, 0.])
        radial = log['qpos'][:, :2]-centre
        angles = np.unwrap(np.arctan2(radial[:, 1], radial[:, 0]))
        turn = float((angles[-1]-angles[0])/(2*np.pi))*(-1 if path.clockwise else 1)
        extra.update(signed_laps=turn, radial_error_rms_m=float(np.sqrt(np.mean((np.linalg.norm(radial, axis=1)-path.radius_m)**2))))
        gates['full_circle'] = .95 <= turn <= 1.05
        extra['passed'] = bool(all(gates.values()))
    # Straight-path projections would misdescribe a closed path.
    summary.pop('distance_along_path_m', None)
    summary.pop('final_cross_track_m', None)
    summary['mobility'] = extra
    return summary


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=True)+'\n', encoding='utf-8')
    temporary.replace(path)


def run_pair(job):
    """Isolated process workspace; persist each run before attempting a replay."""
    name, seed, phase, scale, output, cache, replays, timestep = job
    tag = f'{name}_s{seed}_p{phase:+.2f}_v{scale:g}'
    directory = Path(output)/tag
    directory.mkdir(parents=True, exist_ok=True)
    done = directory/'pair.json'
    if done.exists():
        return json.loads(done.read_text())
    kind, shape, duration, clockwise = CASES[name]
    terrain = terrain_for(kind, seed)
    audit = terrain.audit(BOUNDS, .12, samples=101)
    if not audit['single_contact_valid']:
        raise ValueError(f'{tag}: terrain outside single-contact model validity: {audit}')
    config = load_config()
    config['simulation']['timestep_s'] = timestep
    scope = hashlib.sha256(str(output).encode()).hexdigest()[:12]
    workspace = Workspace(config, REPO_ROOT/'generated'/'offroad'/'mobility'/scope/tag, cache, plant_limit=1)
    plant = TerrainPlant(config, terrain, workspace.directory/'arena', visual_bounds=BOUNDS)
    workspace._plants[json.dumps(terrain.spec(), sort_keys=True)] = plant
    path = MobilityPath(shape, duration/scale, clockwise=clockwise)
    pushes = [(path.start_s+.35*path.duration_s, .1, [.5, 0., 0.]),
              (path.start_s+.65*path.duration_s, .1, [0., -.5, 0.])] if name == 'pushed_circle' else []
    runs, summaries = [], []
    from .offroad_analysis import plot_run
    for mode in ('terrain_aware', 'flat_baseline'):
        run_dir = directory/mode
        if (run_dir/'summary.json').exists() and (run_dir/'rollout.npz').exists():
            summary = json.loads((run_dir/'summary.json').read_text())
            with np.load(run_dir/'rollout.npz') as data:
                log = dict(data)
        else:
            log, summary = run_offroad(workspace, terrain, path, OffroadParams(mode=mode), phase=phase, pushes=pushes)
            summary = evaluate(log, summary, path)
            summary.update(seed=seed, start_phase_rad=phase, speed_multiplier=scale, terrain_audit=audit,
                           pushes=pushes, physics_timestep_s=timestep)
            save_run(run_dir, log, summary, terrain)
        plot_run(log, summary, terrain, path, run_dir/'plots.png', tag+' / '+mode)
        summaries.append(summary)
        log['planned_xy'] = np.array([path(t)[0] for t in np.linspace(path.start_s, path.start_s+path.duration_s, 180)])
        runs.append((mode, log))
    replay = None
    if replays == 'all' or (replays == 'representative' and (seed == SEEDS[0] and phase == PHASES[0] or not summaries[0]['mobility']['passed'])):
        from .offroad_view import render_replay
        replay = str(render_replay(plant, runs, directory/'replay.mp4', fps=20, speed=1., width=640, height=480,
                                   elevation=-55., distance=2.5, title=name,
                                   fixed_lookat=[0., 0., terrain.height(0., 0.)]))
    result = dict(case=name, seed=seed, phase=phase, speed_multiplier=scale,
                  summaries=summaries, replay=replay)
    write_json(done, result)
    return result


def report(output, results, total, errors):
    rows = []
    for result in results:
        for summary in result['summaries']:
            rows.append((result, summary))
    aggregate = {}
    for case in sorted({result['case'] for result in results}):
        aggregate[case] = {}
        for mode in ('terrain_aware', 'flat_baseline'):
            selected = [s for r, s in rows if r['case'] == case and s['controller'] == mode]
            aggregate[case][mode] = dict(
                runs=len(selected), completed=sum(s['completed'] for s in selected),
                accepted=sum(s['mobility']['passed'] for s in selected),
                median_rms_m=float(np.median([s['tracking']['rms_error_while_moving_m'] for s in selected])),
                worst_max_error_m=max(s['tracking']['max_error_m'] for s in selected),
                worst_final_error_m=max(s['tracking']['final_error_m'] for s in selected))
    write_json(Path(output)/'aggregate.json', aggregate)
    lines = ['# Off-road mobility evaluation', '',
             f'Collected {len(results)}/{total} matched pairs; infrastructure errors: {len(errors)}.', '',
             'Current controller is unchanged. Acceptance thresholds were fixed before collection.',
             'Passing demonstrates these simulated trajectories, not unrestricted hardware mobility.', '',
             '| Case / seed / phase / speed | Controller | Completed | Passed | RMS mm | P95 mm | Max mm | Final mm | Heading excursion deg |',
             '|---|---|---|---|---|---|---|---|---|']
    for result, s in sorted(rows, key=lambda r: (r[0]['case'], r[0]['seed'], r[0]['phase'], r[0]['speed_multiplier'], r[1]['controller'])):
        m, t = s['mobility'], s['tracking']
        lines.append(f"| {result['case']} / {result['seed']} / {result['phase']} / {result['speed_multiplier']} | {s['controller']} | {s['completed']} | {m['passed']} | {1000*t['rms_error_while_moving_m']:.1f} | {1000*m['p95_moving_error_m']:.1f} | {1000*t['max_error_m']:.1f} | {1000*t['final_error_m']:.1f} | {m['max_heading_excursion_deg']:.1f} |")
    lines.extend(['', '## Replays', ''])
    for result in sorted(results, key=lambda r: (r['case'], r['seed'], r['phase'], r['speed_multiplier'])):
        if result['replay']:
            relative = Path(result['replay']).relative_to(output).as_posix()
            lines.append(f"- [{Path(relative).parent}]({relative})")
    if errors:
        lines.extend(['', '## Infrastructure errors', '', *[f'- {error}' for error in errors]])
    (Path(output)/'REPORT.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    write_json(Path(output)/'progress.json', dict(completed_pairs=len(results), total_pairs=total, errors=errors,
                                                finished=len(results)+len(errors)==total))
    write_json(Path(output)/'results.json', results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=('showcase', 'robustness', 'speed'), default='showcase')
    parser.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    parser.add_argument('--workers', type=int, default=3)
    parser.add_argument('--output', type=Path, default=REPO_ROOT/'results'/'offroad'/'mobility')
    parser.add_argument('--replays', choices=('all', 'representative', 'none'), default='all')
    parser.add_argument('--timestep', type=float, choices=(.001, .0005), default=.001)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    seeds = SEEDS if args.suite == 'robustness' else SEEDS[:1]
    phases = PHASES if args.suite == 'robustness' else PHASES[:1]
    scales = (.5, 1., 1.5, 2.) if args.suite == 'speed' else (1.,)
    sources = {str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in sorted((REPO_ROOT/'src'/'spherical_wheel').glob('*.py'))}
    config = load_config()
    config['simulation']['timestep_s'] = args.timestep
    manifest = dict(suite=args.suite, cases=args.cases, seeds=seeds, phases=phases, speed_multipliers=scales,
                    thresholds=TARGETS, source_sha256=sources, config=config, replays=args.replays,
                    git_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO_ROOT, text=True).strip())
    manifest_path = output/'manifest.json'
    if manifest_path.exists() and json.loads(manifest_path.read_text()) != json.loads(json.dumps(manifest)):
        raise ValueError('Output contains a different experiment. Choose a fresh --output directory.')
    write_json(manifest_path, manifest)
    print('Building shared flat-ground gain library...', flush=True)
    scope = hashlib.sha256(str(output).encode()).hexdigest()[:12]
    workspace = Workspace(config, REPO_ROOT/'generated'/'offroad'/'mobility'/scope/'design',
                          REPO_ROOT/'generated'/'offroad'/'mobility'/f'design_{args.timestep:g}'/'flat'/'control_schedule.npz')
    jobs = [(case, seed, phase, scale, str(output), str(workspace.cache_path), args.replays, args.timestep)
            for case in args.cases for seed in seeds for phase in phases for scale in scales]
    results, errors = [], []
    report(output, results, len(jobs), errors)
    started = time.time()
    with ProcessPoolExecutor(max_workers=max(1, args.workers)) as pool:
        pending = {pool.submit(run_pair, job): job for job in jobs}
        for future in as_completed(pending):
            try:
                result = future.result()
                results.append(result)
                print(f"[{len(results)}/{len(jobs)}] {result['case']} seed={result['seed']} phase={result['phase']} "
                      f"passes={[s['mobility']['passed'] for s in result['summaries']]} elapsed={time.time()-started:.0f}s", flush=True)
            except Exception as exc:
                errors.append(f'{pending[future][:4]}: {type(exc).__name__}: {exc}')
                print(errors[-1], flush=True)
            report(output, results, len(jobs), errors)
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
