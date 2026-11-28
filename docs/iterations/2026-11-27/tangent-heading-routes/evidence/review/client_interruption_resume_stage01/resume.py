"""Resume frozen I76 jobs; keep completed outputs and preserve interruptions."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess
ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parents[2]
STAGE=Path(__file__).resolve().parent
EXPECTED_PROTOCOL='53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2'
NAMES=('precision.npz','rollout.npz','summary.json','provenance_before.json','provenance.json','failure.json','artifact_manifest.json','runtime_before_physics.json')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def binding(p):return dict(sha256=sha(p),bytes=p.stat().st_size)
def save(p,v):
 with p.open('x')as f:json.dump(v,f,indent=2,sort_keys=True);f.write('\n')
def now():return datetime.now(timezone.utc).isoformat()
def verify_protocol():
 p=BASE/'protocol/protocol.json'
 if sha(p)!=EXPECTED_PROTOCOL:raise ValueError('Frozen corrected protocol differs')
 d=json.loads(p.read_text())
 for group in ('source_sha256','input_sha256'):
  for rel,digest in d[group].items():
   if sha(ROOT/rel)!=digest:raise ValueError('Frozen source/input changed:'+rel)
 if sha(ROOT/d['gain_cache']['path'])!=d['gain_cache']['sha256']:raise ValueError('Frozen gain changed')
 return d

def command(job):
 return [str(ROOT/'build/tools/bin/micromamba'),'run','-p',str(ROOT/'build/gazebo-env'),'python',str(BASE/'campaign_tools/run_case.py'),'--base',str(BASE),'--case',job['case_id']]

def prepare():
 protocol=verify_protocol()
 if (BASE/'campaign/outputs_sealed.json').exists():raise FileExistsError('Campaign already sealed')
 observed=[json.loads(line)for line in (STAGE/'observed_launcher_stdout.jsonl').read_text().splitlines()if line.strip()]
 expected={f'straight_{phase}_{arm}'for phase in ('m100','p030','p090')for arm in ('N','T')}
 if len(observed)!=6 or {x['finished']for x in observed}!=expected or any(x['returncode']!=0 for x in observed):raise ValueError('Exact six observed successful stdout records required')
 recovered=[];files={}
 for job in protocol['cases']:
  cid=job['case_id']
  if cid not in expected:continue
  directory=BASE/'campaign/runs'/cid
  summary=json.loads((directory/'summary.json').read_text());prov=json.loads((directory/'provenance.json').read_text())
  if not summary['completed'] or summary['failure']is not None or abs(summary['completed_duration_s']-job['duration_s'])>1e-8:raise ValueError('Incomplete recovered case:'+cid)
  if prov['protocol_sha256']!=EXPECTED_PROTOCOL or not prov['source_hashes_unchanged_after'] or not prov['completed']or prov['failure']is not None:raise ValueError('Recovered provenance mismatch:'+cid)
  manifest=json.loads((directory/'artifact_manifest.json').read_text())
  entries={entry['path']:entry for entry in manifest['files']}
  if not {'precision.npz','rollout.npz','summary.json','provenance.json'}<=set(entries):raise ValueError('Required completed artifacts absent:'+cid)
  for rel,entry in entries.items():
   p=directory/rel
   if binding(p)!={k:entry[k]for k in ('sha256','bytes')}:raise ValueError('Recovered artifact changed:'+str(p))
  log=BASE/'campaign/process_logs'/f'{cid}.log'
  finish=[json.loads(line)for line in log.read_text().splitlines()if line.startswith('{')and '"completed"'in line]
  if not finish or finish[-1].get('case')!=cid or not finish[-1].get('completed')or finish[-1].get('failure')is not None:raise ValueError('Final completed log missing:'+cid)
  for p in directory.iterdir():
   if p.is_file():files[str(p.relative_to(BASE))]=binding(p)
  files[str(log.relative_to(BASE))]=binding(log)
  recovered.append(dict(case_id=cid,command=command(job),returncode=0,returncode_source='Observed prior launcher stdout, preserved verbatim in review/client_interruption_resume_stage01/observed_launcher_stdout.jsonl',completed_artifacts_validated=True))
 save(STAGE/'registered_before_resume.json',dict(iteration='I-76',utc=now(),protocol_sha256=EXPECTED_PROTOCOL,source=binding(Path(__file__)),observed_stdout=binding(STAGE/'observed_launcher_stdout.jsonl'),validated_completed_files=files,source_input_bindings_unchanged=True,no_physics_or_case_array_reads_during_recovery=True,policy='Keep six observed-successful complete runs; archive interrupted directories/logs; start remaining twelve under identical frozen control source.'))
 interruption=[]
 for job in protocol['cases']:
  cid=job['case_id']
  if cid in expected:continue
  directory=BASE/'campaign/runs'/cid;log=BASE/'campaign/process_logs'/f'{cid}.log'
  row=dict(case_id=cid,original_case_directory=str(directory.relative_to(BASE)),started_before_interruption=directory.exists(),raw_state_arrays_saved=False)
  if directory.exists():
   before=directory/'provenance_before.json'
   if before.exists():
    pid=json.loads(before.read_text()).get('pid');proc=Path(f'/proc/{pid}/cmdline')
    if proc.exists()and str(BASE/'campaign_tools/run_case.py').encode()in proc.read_bytes():raise RuntimeError('Recorded experiment still active:'+cid)
   dst=STAGE/'interrupted_runs'/cid;dst.parent.mkdir(exist_ok=True);shutil.move(str(directory),str(dst));row['preserved_case_directory']=str(dst.relative_to(BASE))
   row['raw_state_arrays_saved']=any(dst.glob('*.npz'))
  if log.exists():
   dst=STAGE/'interrupted_logs'/log.name;dst.parent.mkdir(exist_ok=True);shutil.move(str(log),str(dst));row['preserved_process_log']=str(dst.relative_to(BASE))
  interruption.append(row)
 save(STAGE/'interruption_receipt.json',dict(iteration='I-76',utc=now(),reason='User reports accidental VSCode closure; no experiment processes remained active on resume.',cases=interruption,controller_failure_claimed=False,lost_in_memory_states_disclosed=True))
 save(STAGE/'recovery_receipt.json',dict(iteration='I-76',utc=now(),protocol_sha256=EXPECTED_PROTOCOL,recovered_processes=recovered,completed_file_bindings=files,returncodes_observed_not_inferred=True,evidence_provenance='Copied exact six finished/returncode0 records from prior assistant tool stdout, unified exec session83530 chunks9e4ed7 and d75ef1. These were actually observed before client interruption; summary/provenance/artifact/log checks independently validate completion.',case_array_contents_read=False))
 print(json.dumps(dict(recovered=len(recovered),remaining=18-len(recovered),preserved_interrupted=sum(x['started_before_interruption']for x in interruption))),flush=True)

def run():
 protocol=verify_protocol();registered=json.loads((STAGE/'registered_before_resume.json').read_text())
 if binding(Path(__file__))!=registered['source'] or binding(STAGE/'observed_launcher_stdout.jsonl')!=registered['observed_stdout']:raise ValueError('Registered recovery source/evidence changed')
 recovery=json.loads((STAGE/'recovery_receipt.json').read_text())
 for rel,v in recovery['completed_file_bindings'].items():
  if binding(BASE/rel)!=v:raise ValueError('Recovered completed artifact changed:'+rel)
 jobs=[x for x in protocol['cases']if x['case_id']not in {r['case_id']for r in recovery['recovered_processes']}]
 if len(jobs)!=12:raise ValueError('Exactly12 remaining jobs required')
 if (BASE/'campaign/outputs_sealed.json').exists():raise FileExistsError('Preserve existing seal')
 env=os.environ.copy()
 for key in ('PYTHONPATH','LD_LIBRARY_PATH','LD_PRELOAD'):env.pop(key,None)
 env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONUNBUFFERED='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',MUJOCO_GL='egl')
 def one(job):
  cid=job['case_id'];cmd=command(job);print(json.dumps(dict(start=cid,utc=now())),flush=True)
  with (BASE/'campaign/process_logs'/f'{cid}.log').open('x')as f:result=subprocess.run(cmd,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
  print(json.dumps(dict(finished=cid,returncode=result.returncode)),flush=True)
  return dict(case_id=cid,command=cmd,returncode=result.returncode,returncode_source='Observed resumed subprocess exit status')
 with ThreadPoolExecutor(max_workers=3)as pool:
  outcomes=[f.result()for f in as_completed([pool.submit(one,j)for j in jobs])]
 outcomes+=recovery['recovered_processes'];verify_protocol();files={}
 for job in protocol['cases']:
  directory=BASE/'campaign/runs'/job['case_id']
  for name in NAMES+('partial_precision.npz','partial_truth.npz'):
   p=directory/name
   if p.exists():files[str(p.relative_to(BASE))]=binding(p)
  p=BASE/'campaign/process_logs'/f"{job['case_id']}.log"
  files[str(p.relative_to(BASE))]=binding(p)
 for name in ('resume.py','observed_launcher_stdout.jsonl','registered_before_resume.json','recovery_receipt.json','interruption_receipt.json'):
  p=STAGE/name;files[str(p.relative_to(BASE))]=binding(p)
 success=all(x['returncode']==0 for x in outcomes)
 seal=dict(iteration='I-76',sealed_before_campaign_evaluation_utc=now(),protocol_sha256=EXPECTED_PROTOCOL,processes=sorted(outcomes,key=lambda x:x['case_id']),files=files,all_processes_successful=success,recovered_completed_processes=6,fresh_resumed_processes=12,scope='All18 corrected-protocol recordings; six observed successful exits recovered from prior tool stdout and byte-verified artifacts, twelve newly observed subprocess outcomes. No fabricated status or physics. Acceptance remains separate.',no_new_campaign_metric_or_truth_analysis_by_launcher=True)
 save(BASE/'campaign/outputs_sealed.json',seal);print(json.dumps(dict(outputs_sealed=True,all_processes_successful=success)),flush=True)
 raise SystemExit(0 if success else 1)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');p.add_argument('--run',action='store_true');a=p.parse_args()
 if a.prepare:prepare()
 if a.run:run()
