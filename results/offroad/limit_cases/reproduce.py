"""Cases chosen to be beyond what the wheel can do, kept as evidence of where it stops.

Run from the repository root with the project environment:

    python results/offroad/limit_cases/reproduce.py

``slick_ramp_beyond_friction``: an 8 degree grade with a strip whose friction
coefficient (0.12) is below tan(8 deg) = 0.14, so standing on it needs 117% of
the friction available. No controller can hold there; the run shows what the
contact-force measurements report while it slides.
"""
import json
from pathlib import Path

from spherical_wheel.control import jsonable
from spherical_wheel.offroad_experiments import ReferencePath
from spherical_wheel.offroad_suite import run_jobs
from spherical_wheel.terrain import make_terrain

HERE = Path(__file__).resolve().parent


def main():
    terrain = make_terrain('slick_ramp', grade_deg=8., mu=.12)
    path = ReferencePath(1.0, 0., 12.)
    jobs = [{'terrain': terrain.spec(), 'path': path.spec(), 'params': {'mode': mode}, 'label': mode,
             'save': str(HERE/'slick_ramp_beyond_friction'/mode),
             'title': 'Slippery strip beyond the friction limit'}
            for mode in ('flat_baseline', 'terrain_aware')]
    report = {'scope': 'Deliberately infeasible cases. Simulated provisional model.',
              'terrain': terrain.description, 'runs': {}}
    for summary in run_jobs(jobs, workers=2):
        report['runs'][summary['label']] = {
            'completed': summary['completed'], 'failure': summary['failure'],
            'completed_duration_s': summary['completed_duration_s'],
            'distance_along_path_m': summary['distance_along_path_m'],
            'final_error_mm': summary['tracking']['final_error_m']*1000,
            'max_error_mm': summary['tracking']['max_error_m']*1000,
            'friction_use_p95': summary['contact']['utilization_p95'],
            'sliding_time_s': summary['contact']['sliding_time_s'],
            'sliding_distance_mm': summary['contact']['slip_distance_m']*1000,
            'lowest_friction_estimate': summary['contact']['estimated_friction_min'],
            'friction_under_wheel_range': summary['ground']['friction_range']}
    (HERE/'summary.json').write_text(json.dumps(jsonable(report), indent=2)+'\n')
    print(json.dumps(jsonable(report), indent=2))


if __name__ == '__main__':
    main()
