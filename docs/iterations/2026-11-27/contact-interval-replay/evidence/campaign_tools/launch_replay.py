"""Fresh, sanitized process for the single registered I68 replay."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import os
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]
stage = BASE/'launch_stage01'
stage.mkdir(exist_ok=False)
env = dict(os.environ)
for key in ('PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH'):
    env.pop(key, None)
env.update(MAMBA_ROOT_PREFIX=str(ROOT/'build/mamba'), MUJOCO_GL='egl',
    OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
cmd = [str(ROOT/'build/tools/bin/micromamba'), 'run', '-p', str(ROOT/'build/gazebo-env'),
    'python', str(BASE/'campaign_tools/run_interval_replay.py'), '--run']
registered = dict(iteration='I-68', registered_utc=datetime.now(timezone.utc).isoformat(),
    command=cmd, protocol_sha256=hashlib.sha256((BASE/'protocol.json').read_bytes()).hexdigest(),
    launcher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), scope='One fresh process; no retries or overwritten runs.')
(stage/'registered_before_execution.json').write_text(json.dumps(registered, indent=2)+'\n')
started = time.monotonic()
with (stage/'execution.log').open('xb') as log:
    completed = subprocess.run(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
result = dict(returncode=completed.returncode, elapsed_s=time.monotonic()-started)
(stage/'execution_result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result), flush=True)
sys.exit(completed.returncode)
