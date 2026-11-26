"""One-step handoff algebra on original saved inputs; not a closed-loop replay."""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
spec=importlib.util.spec_from_file_location('handoff_continuity',HERE/'handoff_continuity.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    out=HERE/'saved_array_checks/stage01';out.mkdir(parents=True,exist_ok=False)
    source=[Path(__file__).resolve(),HERE/'handoff_continuity.py']
    raw=ROOT/'results/belt_capacity_sweep_20261126/campaign/runs'
    files=source+[raw/case/'precision.npz'for case in ('mars_cap030','mars_cap060')]
    before={str(p.relative_to(ROOT)):sha(p)for p in files}
    for p in source:
        dest=out/'source_snapshot'/p.name;dest.parent.mkdir(exist_ok=True);shutil.copy2(p,dest)
    (out/'before_arithmetic.json').write_text(json.dumps({'input_sha256':before,'scope':__doc__,'selection':'Fixed previously audited31.650baseline and32.170/32.190raised switch samples. Each is an independent one-step envelope centered on original ACTUALLY APPLIED prior input, not a sequential modified trajectory.'},indent=2)+'\n')
    rows=[]
    for case,t in [('mars_cap030',31.65),('mars_cap060',32.17),('mars_cap060',32.19)]:
        # Read only current/past estimated support and native command fields.
        # No true contact, future measurement, predictor or controller is used.
        with np.load(raw/case/'precision.npz',allow_pickle=False)as z:
            d={k:z[k].copy()for k in ('time_s','exact_applied','exact_previous_applied','exact_estimated_support_loads_N','exact_input_bounds')}
        i=int(np.argmin(abs(d['time_s']-t)));h=module.BeltHandoffContinuity()
        h.apply(d['exact_applied'][i-1],d['exact_previous_applied'][i-1],d['exact_estimated_support_loads_N'][i-1],.005,d['exact_input_bounds'][i-1,3:],d['time_s'][i-1])
        final,diag=h.apply(d['exact_applied'][i],d['exact_previous_applied'][i],d['exact_estimated_support_loads_N'][i],.005,d['exact_input_bounds'][i,3:],d['time_s'][i])
        checks={'genuine_beltB_mode_transition':bool(diag['triggered'][1]),
            'first3_torques_unchanged':bool(np.array_equal(final[:3],d['exact_applied'][i,:3])),
            'inactive_beltA_unchanged':bool(final[3]==d['exact_applied'][i,3]),
            'physical_cap_satisfied':bool(np.all(abs(final[3:])<=d['exact_input_bounds'][i,3:])),
            'beltB_step_15mm_s_or_less':bool(abs(final[4]-d['exact_previous_applied'][i,4])<=.015+1e-12),
            'original_saved_previous_alignment':bool(np.array_equal(d['exact_previous_applied'][i],d['exact_applied'][i-1]))}
        rows.append({'case':case,'index':i,'time_s':float(d['time_s'][i]),'native_applied':d['exact_applied'][i].tolist(),'previous_actually_applied':d['exact_previous_applied'][i].tolist(),'one_step_candidate_applied':final.tolist(),'delta':(final-d['exact_applied'][i]).tolist(),'checks':checks})
    assert before=={str(p.relative_to(ROOT)):sha(p)for p in files}
    result={'passed':all(all(r['checks'].values())for r in rows),'scope':__doc__,'assumption_m_s2':3.,'rows':rows,'input_sha256':before,
        'not_a_performance_result':'Native requested command histories remain those of the original runs. These independent local projections do not predict the modified closed-loop state or future observer/contact history.',
        'truth_or_future_input':False,'new_physics':False,'native_controller_called':False}
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'artifact_manifest.json').write_text(json.dumps({'files':[{'path':str(p.relative_to(out)),'sha256':sha(p),'bytes':p.stat().st_size}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']},indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['passed']:raise AssertionError('Preserve failed contract stage')

if __name__=='__main__':main()
