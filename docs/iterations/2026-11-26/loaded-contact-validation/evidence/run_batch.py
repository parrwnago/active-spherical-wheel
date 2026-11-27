"""Serial fresh-process orchestration of the frozen I57 cases."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

ROOT = Path.cwd()
BASE = ROOT/'results/loaded_contact_validation_20261126'
parser = argparse.ArgumentParser()
parser.add_argument('--phase', choices=('pilots', 'remaining'), required=True)
args = parser.parse_args()
protocol = json.loads((BASE/'protocol.json').read_text())
pilots = protocol['initial_pilots']
cases = pilots if args.phase == 'pilots' else [x['case_id'] for x in protocol['cases'] if x['case_id'] not in pilots]
prediction = BASE/'model_tools/predictions_stage01'
assert (prediction/'report.json').exists() and (prediction/'artifact_manifest.json').exists()
if args.phase == 'remaining':
    for case in pilots:
        d = json.loads((BASE/'campaign/runs'/case/'provenance.json').read_text())
        assert d['success'] and d['original_continuation_bit_exact']
stage = BASE/f'batch_{args.phase}'
stage.mkdir(exist_ok=False)
registration = dict(
    phase=args.phase, cases=cases, registered_utc=datetime.now(timezone.utc).isoformat(),
    protocol_sha256=hashlib.sha256((BASE/'protocol.json').read_bytes()).hexdigest(),
    predictions_manifest_sha256=hashlib.sha256((prediction/'artifact_manifest.json').read_bytes()).hexdigest(),
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    execution='Serial fresh child processes; stop immediately on any nonzero exit; never overwrite cases.')
(stage/'registered_before_execution.json').write_text(json.dumps(registration, indent=2)+'\n')
env = dict(os.environ)
for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
    env.pop(key, None)
env.update(MAMBA_ROOT_PREFIX=str(ROOT/'build/mamba'), MUJOCO_GL='egl',
           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
started = time.monotonic()
completed = []
for case in cases:
    assert not (BASE/'campaign/runs'/case).exists()
    cmd = [str(ROOT/'build/tools/bin/micromamba'), 'run', '-p', str(ROOT/'build/gazebo-env'),
           'python', str(BASE/'campaign_tools/run_probe.py'), '--output', str(BASE), '--run', '--case-id', case]
    t0 = time.monotonic()
    with (stage/(case+'.log')).open('wb') as log:
        result = subprocess.run(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    row = dict(case_id=case, returncode=result.returncode, elapsed_s=time.monotonic()-t0)
    completed.append(row)
    (stage/'execution_results.json').write_text(json.dumps(completed, indent=2)+'\n')
    print(json.dumps(dict(**row, completed=len(completed), planned=len(cases), total_elapsed_s=time.monotonic()-started)), flush=True)
    if result.returncode:
        sys.exit(result.returncode)
