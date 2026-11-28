"""I75 reference-only checks after the root's separate registration.

Never constructs a physical robot, reads an outcome, or inspects terrain.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil,subprocess,sys
import numpy as np
from heading_reference import PlannedTangentReference

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--stage',type=Path,required=True);args=parser.parse_args()
    protocol_path=args.protocol.resolve();stage=args.stage.resolve()
    protocol=json.loads(protocol_path.read_text())
    settings=protocol['reference_bounds']
    caps=dict(max_rate_rad_s=float(settings['maximum_absolute_heading_rate_rad_s']),
              max_acceleration_rad_s2=float(settings['maximum_absolute_heading_acceleration_rad_s2']))
    if caps!={'max_rate_rad_s':.35,'max_acceleration_rad_s2':.5}:
        raise ValueError('Root-fixed I75 bounds are .35 rad/s and .5 rad/s²')
    sources=[Path(__file__).resolve(),BASE/'runtime/heading_reference.py',BASE/'runtime/route.py',BASE/'runtime/test_heading_reference.py']
    files=sources+[protocol_path]
    stage.mkdir(parents=True,exist_ok=False)
    bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in files}
    save(stage/'registered_before_reference_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),
         files=bindings,no_physics=True,no_terrain_arithmetic=True,no_case_outputs=True,
         sampling=dict(count=9001,clock_s=[0.,45.],dt_s=.005)))
    for p in sources:
        target=stage/'source'/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
    ref=PlannedTangentReference(**caps)
    times=np.arange(9001)*.005
    rows=[ref.sample(float(t)) for t in times]
    arrays={key:np.asarray([r[key] for r in rows]) for key in rows[0]}
    arrays['time_s']=times
    np.savez_compressed(stage/'reference_samples.npz',**arrays)
    result=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(BASE/'runtime'),
                           '-p','test_heading_reference.py','-v'],cwd=ROOT,text=True,capture_output=True)
    (stage/'unittest.log').write_text(result.stdout+result.stderr)
    unchanged=all(sha(ROOT/p)==v['sha256'] for p,v in bindings.items())
    if not unchanged:raise ValueError('Reference source/registration changed')
    report=dict(iteration='I-75',passed=result.returncode==0,physics_steps=0,
                no_terrain_or_case_output_read=True,reference=ref.spec(),
                initial_heading_deg=float(np.rad2deg(arrays['raw_heading_rad'][0])),
                final_heading_deg=float(np.rad2deg(arrays['raw_heading_rad'][-1])),
                sampled_heading_range_deg=[float(np.rad2deg(arrays['raw_heading_rad'].min())),float(np.rad2deg(arrays['raw_heading_rad'].max()))],
                sampled_absolute_rate_peak_rad_s=float(abs(arrays['raw_heading_rate_rad_s']).max()),
                sampled_absolute_acceleration_peak_rad_s2=float(abs(arrays['raw_heading_acceleration_rad_s2']).max()),
                raw_command_arrays_exact=all(np.array_equal(arrays['raw_'+name],arrays['command_'+name])
                    for name in ('heading_rad','heading_rate_rad_s','heading_acceleration_rad_s2')),
                rate_limit_active_samples=int(arrays['rate_limited'].sum()),
                acceleration_limit_active_samples=int(arrays['acceleration_limited'].sum()),
                test_returncode=result.returncode,source_and_inputs_unchanged=True,
                limitation='Reference-only analytic/sample checks; not actuator feasibility or closed-loop validation.')
    save(stage/'report.json',report)
    save(stage/'artifact_manifest.json',dict(files={p.name:dict(sha256=sha(p),bytes=p.stat().st_size)
         for p in sorted(stage.iterdir()) if p.is_file()}))
    print(json.dumps(report,indent=2),flush=True)
    if result.returncode:raise SystemExit(result.returncode)


if __name__=='__main__':main()
