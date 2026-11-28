"""Verify corrected route at archived fixture points; never query terrain."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,importlib.util,json
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parents[2]
STAGE=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):
 with p.open('x')as f:json.dump(v,f,indent=2,sort_keys=True);f.write('\n')
def main():
 files=[Path(__file__),BASE/'runtime/route.py',BASE/'runtime/fixture_stage01/map_stage02/report.json']
 for rid in ('straight','arc','eight'):
  folder=BASE/'runtime/fixture_stage01/map_stage02'/rid
  files += [folder/'map_samples.npz',folder/'ground_verification_fixture.json']
 bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
 save(STAGE/'registered_before_geometry_arithmetic.json',dict(iteration='I-76',files=bindings,utc=datetime.now(timezone.utc).isoformat(),no_terrain_queries=True,no_physics=True,no_controller_outcome_reads=True,tolerance_m_or_derivatives=1e-12))
 spec=importlib.util.spec_from_file_location('_i76_carry_route',BASE/'runtime/route.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 rows={}
 for rid in m.ROUTE_IDS:
  route=m.make_route(rid);folder=BASE/'runtime/fixture_stage01/map_stage02'/rid
  with np.load(folder/'map_samples.npz',allow_pickle=False)as z:
   expected={k:z[k].copy()for k in ('time_s','xy_m','velocity_m_s','acceleration_m_s2')}
  values=[route(float(t))for t in expected['time_s']]
  errors={key:float(np.max(np.abs(np.asarray([v[i]for v in values])-expected[key])))for i,key in enumerate(('xy_m','velocity_m_s','acceleration_m_s2'))}
  ground=json.loads((folder/'ground_verification_fixture.json').read_text())
  errors['seven_ground_points_m']=float(np.max(np.abs(np.asarray([route(t)[0]for t in ground['ground_check_times_s']])-ground['ground_check_points_xy_m'])))
  rows[rid]=dict(maximum_absolute_errors=errors,passed=all(v<=1e-12 for v in errors.values()),samples=len(values))
 unchanged=all(sha(ROOT/p)==v['sha256']for p,v in bindings.items())
 save(STAGE/'receipt.json',dict(iteration='I-76',passed=unchanged and all(v['passed']for v in rows.values()),routes=rows,input_source_unchanged=unchanged,no_terrain_queries=True,no_physics=True,no_controller_outcome_reads=True,meaning='Original terrain fixture values reused at identical planned coordinates; moderate screen still failed for rough_stress eight.'))
 print(json.dumps(rows));raise SystemExit(0 if unchanged and all(v['passed']for v in rows.values())else 1)
if __name__=='__main__':main()
