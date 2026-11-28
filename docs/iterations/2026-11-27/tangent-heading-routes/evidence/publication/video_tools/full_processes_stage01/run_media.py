"""Dispatch fixed I76 media jobs from the root-released saved-state seal."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parents[2]
CASES=[(route,phase)for route in ('straight','arc','eight')for phase in ('m100','p030','p090')]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):
 with p.open('x')as f:json.dump(v,f,indent=2,sort_keys=True);f.write('\n')
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--kind',choices=('preview','full'),required=True);parser.add_argument('--released-seal',required=True);args=parser.parse_args()
 seal=BASE/'campaign/outputs_sealed.json'
 if sha(seal)!=args.released_seal:raise ValueError('Root release seal differs')
 work=BASE/'publication/video_tools'/('preview_processes_stage01'if args.kind=='preview'else'full_processes_stage01');work.mkdir(exist_ok=False)
 stage_root=BASE/'publication'/('video_preview_stage01'if args.kind=='preview'else'videos_stage01')
 render=BASE/'publication/video_tools/render.py'
 save(work/'registered_before_launch.json',dict(iteration='I-76',utc=datetime.now(timezone.utc).isoformat(),kind=args.kind,all_nine_pairs=CASES,renderer_sha256=sha(render),orchestrator_sha256=sha(Path(__file__)),root_released_output_seal_sha256=args.released_seal,protocol_sha256=sha(BASE/'protocol/protocol.json'),workers=3,no_physics=True,no_inference=True))
 shutil.copy2(Path(__file__),work/'run_media.py')
 env=os.environ.copy();env.update(PYTHONPATH=str(ROOT/'src'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MUJOCO_GL='egl')
 def run(pair):
  route,phase=pair;stage=stage_root/route/phase
  command=[str(ROOT/'.venv/bin/python'),str(render),'--route',route,'--phase',phase,'--stage',str(stage),'--input-seal',str(seal)]
  if args.kind=='preview':command.append('--preview')
  print(json.dumps(dict(start=route+'_'+phase,kind=args.kind)),flush=True)
  with (work/f'{route}_{phase}.log').open('x')as stream:r=subprocess.run(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
  result=dict(route=route,phase=phase,command=command,returncode=r.returncode,stage=str(stage.relative_to(BASE)))
  print(json.dumps(dict(finished=route+'_'+phase,returncode=r.returncode,kind=args.kind)),flush=True)
  return result
 with ThreadPoolExecutor(max_workers=3)as pool:
  rows=[f.result()for f in as_completed([pool.submit(run,pair)for pair in CASES])]
 save(work/'receipt.json',dict(iteration='I-76',passed=all(r['returncode']==0 for r in rows),kind=args.kind,jobs=sorted(rows,key=lambda r:(r['route'],r['phase'])),root_seal_unchanged=sha(seal)==args.released_seal,renderer_unchanged=sha(render)==json.loads((work/'registered_before_launch.json').read_text())['renderer_sha256'],no_physics=True))
 raise SystemExit(0 if all(r['returncode']==0 for r in rows)else 1)
if __name__=='__main__':main()
