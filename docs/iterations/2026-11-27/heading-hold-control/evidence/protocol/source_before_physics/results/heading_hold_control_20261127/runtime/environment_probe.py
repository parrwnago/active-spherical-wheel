"""Read-only interpreter/module/binary availability; never creates a simulator."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import os
import subprocess

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    stage = BASE/'runtime/environment_stage01'
    stage.mkdir(parents=True, exist_ok=False)
    files = [Path(__file__).resolve(), ROOT/'environment/gazebo.yml',
             ROOT/'build/gazebo/libSphericalWheelBridge.so',
             ROOT/'build/gazebo/libAswExactPhysics.so',
             ROOT/'generated/offroad/flat/control_schedule.npz']
    bindings = {str(p.relative_to(ROOT)): dict(sha256=sha(p), bytes=p.stat().st_size)
                for p in files}
    (stage/'registered_before_probe.json').write_text(json.dumps(dict(
        registered_utc=datetime.now(timezone.utc).isoformat(), paths=bindings,
        read_only=True, no_server_or_physics=True),indent=2)+'\n')
    env = dict(os.environ)
    for key in ('PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH',
                'COLCON_PREFIX_PATH','LD_PRELOAD','ASW_I69_CAPTURE_ALL'):
        env.pop(key,None)
    env.update(MAMBA_ROOT_PREFIX=str(ROOT/'build/mamba'),MUJOCO_GL='egl',
               OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    code = """import sys,json,importlib.metadata as md,importlib.util
out={'python':sys.version,'executable':sys.executable,'packages':{}}
for name in ['numpy','scipy','mujoco','Pillow','imageio','imageio-ffmpeg']:
 try: out['packages'][name]=md.version(name)
 except md.PackageNotFoundError: out['packages'][name]=None
try:
 import gz.sim8, gz.math7
 out['gazebo_import_available']=True
 out['gz_sim8_file']=gz.sim8.__file__
except ImportError as e:
 out['gazebo_import_available']=False
 out['gazebo_import_error']=str(e)
print(json.dumps(out))
"""
    commands = {
        'venv':[str(ROOT/'.venv/bin/python'),'-c',code],
        'gazebo_env':[str(ROOT/'build/tools/bin/micromamba'),'run','-p',
                      str(ROOT/'build/gazebo-env'),'python','-c',code],
    }
    results = {}
    for name, command in commands.items():
        result = subprocess.run(command,cwd=ROOT,env=env,text=True,capture_output=True)
        results[name] = dict(command=command[:-1]+['<recorded probe source>'],
                             returncode=result.returncode,stderr=result.stderr)
        if result.returncode == 0:
            results[name]['probe'] = json.loads(result.stdout)
        else:
            results[name]['stdout'] = result.stdout
    unchanged = all(sha(ROOT/p)==b['sha256'] for p,b in bindings.items())
    receipt = dict(no_server_created=True,no_physics=True,no_installation=True,
                   source_and_binaries_unchanged=unchanged,results=results)
    (stage/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt,indent=2))


if __name__=='__main__':
    main()
