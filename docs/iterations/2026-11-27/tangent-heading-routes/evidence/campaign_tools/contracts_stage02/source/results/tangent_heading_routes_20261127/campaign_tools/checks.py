"""Snapshot then run focused I76 adapter/runner contracts; no physics."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stage',type=Path,required=True);args=parser.parse_args()
    stage=args.stage.resolve();stage.mkdir(parents=True,exist_ok=False)
    sources=list((ROOT/'src/spherical_wheel').rglob('*.py'))+[ROOT/'config/model.yaml']
    for part in ('adapter','campaign_tools','runtime'):sources+=list((BASE/part).glob('*.py'))
    sources=sorted(set(sources));bindings={str(p.relative_to(ROOT)):sha(p) for p in sources}
    for p in sources:
        dst=stage/'source'/p.relative_to(ROOT);dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dst)
    env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join(str(p) for p in [ROOT/'src',BASE/'adapter',BASE/'campaign_tools',BASE/'runtime'])
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    command=[str(ROOT/'.venv/bin/python'),'-m','unittest','-v','test_tangent_heading_adapter','test_runner','test_native_identity']
    save(stage/'registered_before_tests.json',dict(iteration='I-76',utc=datetime.now(timezone.utc).isoformat(),source_sha256=bindings,
        command=command,environment={'PYTHONPATH':env['PYTHONPATH']},no_physics=True,
        count_policy='12 functions expected:8 reused adapter,3 generalized runner,1 reused realcontroller identity;4 included identity subtests not added.'))
    with (stage/'unittest.log').open('x') as stream:
        result=subprocess.run(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
    unchanged=all(sha(ROOT/rel)==digest for rel,digest in bindings.items())
    save(stage/'receipt.json',dict(iteration='I-76',passed=result.returncode==0 and unchanged,returncode=result.returncode,
        source_unchanged=unchanged,no_physics=True,scoped_functions=12,identity_subtests_included=4))
    print((stage/'unittest.log').read_text());raise SystemExit(result.returncode or int(not unchanged))
if __name__=='__main__':main()
