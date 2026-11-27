"""Serial fresh-process execution of the frozen I58 cases."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

ROOT=Path.cwd();BASE=ROOT/'results/balance_priority_20261126'
parser=argparse.ArgumentParser();parser.add_argument('--phase',choices=('initial','conditional'),required=True)
args=parser.parse_args();protocol=json.loads((BASE/'protocol.json').read_text())
cases=protocol['initial_execution_order'] if args.phase=='initial' else protocol['conditional_execution_order']
if args.phase=='conditional':
    lead=json.loads((BASE/'analysis/stage01/report.json').read_text())
    assert lead['broader_campaign_eligible'], 'Do not expand a failed lead screen'
stage=BASE/('batch_'+args.phase);stage.mkdir(exist_ok=False)
(stage/'registered_before_execution.json').write_text(json.dumps(dict(
    registered_utc=datetime.now(timezone.utc).isoformat(),phase=args.phase,cases=cases,
    protocol_sha256=hashlib.sha256((BASE/'protocol.json').read_bytes()).hexdigest(),
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    scope='Serial freshprocesses; refuse existing case folders and stop nonzero execution. No source/settings changes.'),indent=2)+'\n')
env=dict(os.environ)
for key in ('PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH'):
    env.pop(key,None)
env.update(MAMBA_ROOT_PREFIX=str(ROOT/'build/mamba'),MUJOCO_GL='egl',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
rows=[];start=time.monotonic()
for case in cases:
    assert not (BASE/'campaign/runs'/case).exists()
    t0=time.monotonic()
    cmd=[str(ROOT/'build/tools/bin/micromamba'),'run','-p',str(ROOT/'build/gazebo-env'),
         'python',str(BASE/'campaign_tools/run_case.py'),'--output',str(BASE),'--run','--case-id',case]
    with (stage/(case+'.log')).open('wb') as log:
        result=subprocess.run(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
    row=dict(case_id=case,returncode=result.returncode,elapsed_s=time.monotonic()-t0)
    rows.append(row);(stage/'execution_results.json').write_text(json.dumps(rows,indent=2)+'\n')
    print(json.dumps(dict(**row,completed=len(rows),planned=len(cases),total_elapsed_s=time.monotonic()-start)),flush=True)
    if result.returncode:sys.exit(result.returncode)
