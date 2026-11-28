"""Launch one I86 unchanged baseline trace and preserve the observed exit."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import os
import subprocess

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def main():
    campaign = BASE / 'campaign'
    campaign.mkdir(exist_ok=False)
    script = BASE / 'runner/run_trace.py'
    protocol = BASE / 'protocol/frozen_stage01/protocol.json'
    out = campaign / 'native106p5_stage01'
    command = [str(ROOT / 'build/tools/bin/micromamba'), 'run', '-p',
               str(ROOT / 'build/gazebo-env'), 'python', str(script),
               '--protocol', str(protocol), '--output', str(out), '--run']
    env = os.environ.copy()
    for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'LD_PRELOAD'):
        env.pop(key, None)
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    save(campaign / 'registered_launch.json', dict(iteration='I-86',
        utc=datetime.now(timezone.utc).isoformat(), command=command, cwd=str(ROOT),
        protocol_sha256=hashlib.sha256(protocol.read_bytes()).hexdigest(),
        launcher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        environment_overrides={'unset': ['PYTHONPATH', 'LD_LIBRARY_PATH', 'LD_PRELOAD'],
                               'set': {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}},
        scope='One unchanged native B060 prefix through106.5s. No controller intervention.'))
    print('Launching unchanged B060106.5s diagnostic', flush=True)
    with (campaign / 'native106p5_stage01.stdout.log').open('xb') as stream:
        observed = subprocess.run(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
    save(campaign / 'native106p5_stage01.observed_exit.json', dict(iteration='I-86',
        utc=datetime.now(timezone.utc).isoformat(), actual_observed_exit_code=observed.returncode,
        command=command, source='subprocess.run returncode'))
    print(json.dumps(dict(observed_exit=observed.returncode, case=str(out.relative_to(ROOT)))), flush=True)
    raise SystemExit(observed.returncode)


if __name__ == '__main__':
    main()
