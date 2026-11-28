"""Run each frozen I76 case once in an isolated Gazebo process; seal outputs."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    protocol_path = BASE/'protocol/protocol.json'
    protocol = json.loads(protocol_path.read_text())
    logs = BASE/'campaign/process_logs'
    logs.mkdir(parents=True,exist_ok=False)
    env = os.environ.copy()
    for key in ('PYTHONPATH','LD_LIBRARY_PATH','LD_PRELOAD'):
        env.pop(key,None)
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONUNBUFFERED='1',
               PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',MUJOCO_GL='egl')
    def run(case):
        command = [str(ROOT/'build/tools/bin/micromamba'),'run','-p',str(ROOT/'build/gazebo-env'),
                   'python',str(BASE/'campaign_tools/run_case.py'),'--base',str(BASE),'--case',case['case_id']]
        print(json.dumps(dict(start=case['case_id'],utc=datetime.now(timezone.utc).isoformat())),flush=True)
        with (logs/(case['case_id']+'.log')).open('x') as handle:
            result = subprocess.run(command,cwd=ROOT,env=env,stdout=handle,stderr=subprocess.STDOUT)
        print(json.dumps(dict(finished=case['case_id'],returncode=result.returncode)),flush=True)
        return dict(case_id=case['case_id'],command=command,returncode=result.returncode)
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures = [pool.submit(run,case) for case in protocol['cases']]
        outcomes = [future.result() for future in as_completed(futures)]
    files = {}
    for case in protocol['cases']:
        directory = BASE/'campaign/runs'/case['case_id']
        for name in ('precision.npz','rollout.npz','summary.json','provenance_before.json',
                     'provenance.json','failure.json','artifact_manifest.json','runtime_before_physics.json'):
            path=directory/name
            if path.exists():
                files[str(path.relative_to(BASE))] = dict(sha256=sha(path),bytes=path.stat().st_size)
        path=logs/(case['case_id']+'.log')
        files[str(path.relative_to(BASE))] = dict(sha256=sha(path),bytes=path.stat().st_size)
    seal = dict(iteration='I-76',sealed_before_campaign_evaluation_utc=datetime.now(timezone.utc).isoformat(),
                protocol_sha256=sha(protocol_path),processes=sorted(outcomes,key=lambda x:x['case_id']),
                files=files,all_processes_successful=all(x['returncode']==0 for x in outcomes),
                scope='All eighteen process results retained; execution success does not mean acceptance.',
                no_new_campaign_metric_or_truth_analysis_by_launcher=True)
    (BASE/'campaign/outputs_sealed.json').write_text(json.dumps(seal,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(outputs_sealed=True,all_processes_successful=seal['all_processes_successful'])),flush=True)
    return 0 if seal['all_processes_successful'] else 1


if __name__=='__main__':
    sys.exit(main())
