"""Fresh sanitized I69 process; LD_PRELOAD applies only to child Python."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib,json,os,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[3];BASE=Path(__file__).resolve().parents[1]
stage=BASE/'capture_launch_stage01';stage.mkdir(exist_ok=False)
protocol=json.loads((BASE/'phase_B_protocol.json').read_text())
env=dict(os.environ)
for key in ('PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_PRELOAD','ASW_I69_CAPTURE_ALL'):
    env.pop(key,None)
env.update(MAMBA_ROOT_PREFIX=str(ROOT/'build/mamba'),MUJOCO_GL='egl',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
cmd=[str(ROOT/'build/tools/bin/micromamba'),'run','-p',str(ROOT/'build/gazebo-env'),'env',
    'LD_PRELOAD='+str(ROOT/protocol['probe_library']),
    'ASW_I69_PROBE_PATH='+str(BASE/'campaign/native_solver_capture/solver_packets.bin'),
    'python',str(BASE/'campaign_tools/run_solver_capture.py'),'--run']
(stage/'registered_before_execution.json').write_text(json.dumps(dict(registered_utc=datetime.now(timezone.utc).isoformat(),command=cmd,protocol_sha256=hashlib.sha256((BASE/'phase_B_protocol.json').read_bytes()).hexdigest()),indent=2)+'\n')
start=time.monotonic()
with (stage/'execution.log').open('xb') as stream:result=subprocess.run(cmd,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
receipt=dict(returncode=result.returncode,elapsed_s=time.monotonic()-start)
(stage/'result.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt),flush=True)
sys.exit(result.returncode)
