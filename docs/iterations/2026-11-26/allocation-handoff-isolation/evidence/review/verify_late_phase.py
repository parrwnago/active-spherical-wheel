"""Independent NumPy-only late-phase matched-history and motion check."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
FAMILY=ROOT/'results/allocation_handoff_isolation_20261126'
OUT=FAMILY/'review/late_phase_stage01'
OUT.mkdir(exist_ok=False)
paths={'baseline':FAMILY/'campaign/runs/base030/precision.npz',
       'phase':FAMILY/'phase_schedule_isolation/campaign/runs/phase_late030/precision.npz',
       'protocol':FAMILY/'phase_schedule_isolation/protocol.json','source':Path(__file__)}
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
decl=dict(captured_utc=datetime.now(timezone.utc).isoformat(),inputs={k:dict(path=str(p.relative_to(ROOT)),sha256=sha(p)) for k,p in paths.items()},
    fixed_windows_s=[[38,42],[38,44]],endpoint_tolerance_s=1e-9,
    scope='Read-only direct arithmetic and exact prefix checks. No controller/model/physics calls; no parameter selection.')
(OUT/'before_arithmetic.json').write_text(json.dumps(decl,indent=2)+'\n')
(OUT/'verify_late_phase.py').write_bytes(Path(__file__).read_bytes())
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def same(a,b):return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
def up(q):
    q=q/np.linalg.norm(q,axis=1,keepdims=True)
    w,x,y,z=q.T
    return np.column_stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
b=load(paths['baseline']);p=load(paths['phase']);t=b['time_s'];before=t<38-1e-9
common=sorted(set(b)&set(p));prefix={k:same(b[k][before],p[k][before]) for k in common}
i=int(np.argmin(abs(t-38)))
at_switch={k:same(b[k][i],p[k][i]) for k in ['exact_qpos','exact_qvel','exact_previous_applied','exact_observer_wrench','exact_estimated_normal','exact_reference_qpos']}
refs={k:same(b[k],p[k]) for k in ['time_s','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2']}
flag=p['exact_phase_schedule_on_normal']
checks=dict(prefix_all_common_fields_exact=all(prefix.values()),switch_state_and_observer_exact=all(at_switch.values()),
            full_time_reference_program_exact=all(refs.values()),phase_flag_exact=np.array_equal(flag,t>=38-1e-9))
cases={}
for label,d in [('baseline',b),('phase',p)]:
    error=np.linalg.norm(d['exact_qpos'][:,:2]-d['exact_reference_xy'],axis=1)
    lean=np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i',up(d['exact_qpos'][:,3:7]),up(d['exact_reference_quat'])), -1,1)))
    windows={}
    for end in [42,44]:
        mask=(t>=38-1e-9)&(t<=end+1e-9)
        joint=d['exact_qvel'][mask,6:9]-d['exact_feedback_reference_qvel'][mask,6:9]
        windows[f'38_{end}']=dict(samples=int(mask.sum()),position_rms_m=float(np.sqrt(np.mean(error[mask]**2))),
            peak_error_m=float(error[mask].max()),end_error_m=float(error[np.argmin(abs(t-end))]),
            peak_reference_lean_deg=float(lean[mask].max()),
            joint_rate_error_rms_rad_s=float(np.sqrt(np.mean(np.sum(joint*joint,axis=1)))),
            yaw_body_rate_rms_rad_s=float(np.sqrt(np.mean(d['exact_qvel'][mask,5]**2))))
    moving=(t>=1)&(t<=41)
    cases[label]=dict(moving_rms_m=float(np.sqrt(np.mean(error[moving]**2))),peak_error_m=float(error.max()),
        final_error_m=float(error[-1]),peak_reference_lean_deg=float(lean.max()),windows=windows)
changed=np.any(b['exact_feedback_gain']!=p['exact_feedback_gain'],axis=(1,2))
out=dict(passed=all(bool(v) for v in checks.values()),checks={k:bool(v) for k,v in checks.items()},
    prefix_fields=prefix,prefix_samples=int(before.sum()),state_at38=at_switch,reference_program=refs,
    first_gain_change_s=None if not changed.any() else float(t[np.flatnonzero(changed)[0]]),cases=cases,
    limitation='Matched-history intervention can establish an effect of scheduling in this episode; no global tuning or universal remedy is demonstrated.')
(OUT/'report.json').write_text(json.dumps(out,indent=2)+'\n')
assert all(sha(paths[k])==v['sha256'] for k,v in decl['inputs'].items())
print(json.dumps(out))
if not out['passed']:raise SystemExit(1)
