"""Run three independent frozen shadow cases; seal all before truth evaluation."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,os,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[3];BASE=Path(__file__).resolve().parents[1]
CASES=('m100_B','p030_B','p090_B')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    stage=BASE/'inference/launch_stage01';stage.mkdir(parents=True,exist_ok=False)
    seal=BASE/'protocol/inference_source_input_seal.json'
    env=dict(os.environ)
    for key in ('PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_PRELOAD'):
        env.pop(key,None)
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    commands={case:[str(ROOT/'.venv/bin/python'),str(BASE/'campaign_tools/run_shadow_case.py'),
        '--case',case,'--input-root',str(BASE/'inputs/stage01'),'--output-root',str(BASE/'inference/stage01'),
        '--source-input-seal',str(seal)] for case in CASES}
    (stage/'registered_before_execution.json').write_text(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),
        source_input_seal_sha256=sha(seal),commands=commands,parallel_fresh_processes=3,threads_per_process=1),indent=2)+'\n')
    opened={};processes={};started=time.monotonic();results=[]
    try:
        for case,command in commands.items():
            opened[case]=(stage/(case+'.log')).open('xb')
            processes[case]=subprocess.Popen(command,cwd=ROOT,env=env,stdout=opened[case],stderr=subprocess.STDOUT)
        for case,process in processes.items():
            code=process.wait();results.append(dict(case=case,returncode=code))
            (stage/'execution_results.json').write_text(json.dumps(results,indent=2)+'\n')
            print(json.dumps(dict(case=case,returncode=code,elapsed_s=time.monotonic()-started)),flush=True)
    finally:
        for f in opened.values():f.close()
    if any(row['returncode'] for row in results):raise SystemExit(1)
    files=[]
    for case in CASES:
        directory=BASE/'inference/stage01'/case
        complete=json.loads((directory/'completed.json').read_text());assert complete['frames']==8801
        for name in ('predictions.npz','model_evidence.npz','completed.json','artifact_manifest.json'):
            p=directory/name;files.append(dict(case=case,path=str(p.relative_to(ROOT)),sha256=sha(p)))
    out=BASE/'inference/all_three_sealed.json'
    out.write_text(json.dumps(dict(iteration='I-70',utc=datetime.now(timezone.utc).isoformat(),
        all_three_inference_complete=True,frames=26403,source_input_seal_sha256=sha(seal),files=files,
        evaluation_truth_read=False,new_physics=False,elapsed_s=time.monotonic()-started),indent=2)+'\n')
    print(json.dumps(dict(all_three_sealed=True,sha256=sha(out))),flush=True)


if __name__=='__main__':main()
