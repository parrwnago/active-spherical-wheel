"""I76 pure reference preflight, after root registration and before physics."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import traceback

import numpy as np

from route import ROUTE_IDS, make_route
from heading_reference import PlannedTangentReference


ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--stage', type=Path, required=True)
    args = parser.parse_args()
    protocol_path, stage = args.protocol.resolve(), args.stage.resolve()
    protocol = json.loads(protocol_path.read_text())
    if protocol.get('iteration') != 'I-76' or tuple(protocol.get('route_ids', [])) != ROUTE_IDS:
        raise ValueError('Root-fixed I76 route registration required')
    caps = protocol['heading_reference_parameters']
    if caps != dict(max_rate_rad_s=.35, max_acceleration_rad_s2=.5):
        raise ValueError('Root-fixed I76 heading guards are .35 rad/s and .5 rad/s²')
    sources = [Path(__file__).resolve(), BASE/'runtime/route.py',
               BASE/'runtime/heading_reference.py', BASE/'runtime/test_analytic_reference.py']
    for source in sources:
        relative = str(source.relative_to(ROOT))
        if protocol['source_sha256'].get(relative) != sha(source):
            raise ValueError('Reference source not frozen by root registration: '+relative)
    for relative, expected in protocol['source_sha256'].items():
        if sha(ROOT/relative) != expected:
            raise ValueError('Root-registered source differs: '+relative)
    stage.mkdir(parents=True, exist_ok=False)
    files = sources+[protocol_path]
    bindings = {str(p.relative_to(ROOT)): dict(sha256=sha(p), bytes=p.stat().st_size) for p in files}
    save(stage/'registered_before_reference_arithmetic.json',
         dict(utc=datetime.now(timezone.utc).isoformat(), files=bindings,
              no_physics=True, no_terrain=True, no_case_outputs=True,
              sampling=dict(dt_s=.005, intervals='Inclusive state grids through full final hold.')))
    for source in files:
        target = stage/'source'/source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    try:
        paths = {route_id: make_route(route_id) for route_id in ROUTE_IDS}
        if protocol['route_specs'] != {route_id: path.spec() for route_id, path in paths.items()}:
            raise ValueError('Analytic route specs differ from root registration')
        result = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', str(BASE/'runtime'),
                                 '-p', 'test_analytic_reference.py', '-v'], cwd=ROOT,
                                text=True, capture_output=True)
        (stage/'unittest.log').write_text(result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError('Focused reference contracts failed; see retained unittest.log')
        reports = {}
        for route_id, path in paths.items():
            ref = PlannedTangentReference(**caps, path=path)
            count = int(round(path.total_duration_s/.005))+1
            times = np.arange(count)*.005
            rows = [ref.sample(float(t)) for t in times]
            arrays = {key: np.asarray([r[key] for r in rows]) for key in rows[0]}
            arrays['time_s'] = times
            position, velocity, acceleration = (np.array([path(t)[k] for t in times]) for k in range(3))
            arrays.update(xy_m=position, velocity_m_s=velocity, acceleration_m_s2=acceleration)
            np.savez_compressed(stage/(route_id+'_reference_samples.npz'), **arrays)
            raw_command_equal = all(np.array_equal(arrays['raw_'+key], arrays['command_'+key])
                                    for key in ('heading_rad', 'heading_rate_rad_s', 'heading_acceleration_rad_s2'))
            reports[route_id] = dict(reference=ref.spec(), count=count,
                initial_heading_deg=float(np.rad2deg(arrays['raw_heading_rad'][0])),
                final_heading_deg=float(np.rad2deg(arrays['raw_heading_rad'][-1])),
                sampled_heading_range_deg=[float(np.rad2deg(arrays['raw_heading_rad'].min())),
                                           float(np.rad2deg(arrays['raw_heading_rad'].max()))],
                sampled_absolute_rate_peak_rad_s=float(abs(arrays['raw_heading_rate_rad_s']).max()),
                sampled_absolute_acceleration_peak_rad_s2=float(abs(arrays['raw_heading_acceleration_rad_s2']).max()),
                sampled_speed_peak_m_s=float(np.linalg.norm(velocity, axis=1).max()),
                sampled_linear_acceleration_peak_m_s2=float(np.linalg.norm(acceleration, axis=1).max()),
                raw_command_arrays_exact=raw_command_equal,
                rate_limit_active_samples=int(arrays['rate_limited'].sum()),
                acceleration_limit_active_samples=int(arrays['acceleration_limited'].sum()))
            if not raw_command_equal:
                raise AssertionError('Raw and commanded reference differ')
        unchanged = all(sha(ROOT/relative) == entry['sha256'] for relative, entry in bindings.items())
        if not unchanged:
            raise ValueError('Reference source or root registration changed during checks')
        report = dict(iteration='I-76', passed=True, physics_steps=0,
                      no_terrain_or_case_output_read=True, focused_test_functions=10,
                      routes=reports, source_and_inputs_unchanged=True,
                      limitation='Pure geometric/reference checks are not actuator feasibility or closed-loop validation.')
        save(stage/'report.json', report)
        save(stage/'artifact_manifest.json', dict(files={p.name: dict(sha256=sha(p), bytes=p.stat().st_size)
             for p in sorted(stage.iterdir()) if p.is_file()}))
        print(json.dumps(report, indent=2), flush=True)
    except Exception as error:
        save(stage/'failure.json', dict(error=repr(error), traceback=traceback.format_exc(),
             physics_steps=0, sources_and_registration_preserved=True))
        raise


if __name__ == '__main__':
    main()
