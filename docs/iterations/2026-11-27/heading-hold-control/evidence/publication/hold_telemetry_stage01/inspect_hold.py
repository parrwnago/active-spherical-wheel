"""Posthoc descriptive telemetry at the fixed 35 s and 45 s hold times.

No gate/parameter change, new inference or physics. These observations cannot
establish the complete cause of a trajectory or the effect of a future change.
"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
import numpy as np

BASE=Path(__file__).resolve().parents[2]
PHASES=('m100','p030','p090')
TIMES=(35.,45.)


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def safe(value):
    if isinstance(value,dict):return {key:safe(v) for key,v in value.items()}
    if isinstance(value,(tuple,list)):return [safe(v) for v in value]
    if isinstance(value,np.ndarray):return safe(value.tolist())
    if isinstance(value,np.generic):return value.item()
    return value
def save(path,value):path.write_text(json.dumps(safe(value),indent=2,sort_keys=True,allow_nan=False)+'\n')


def main():
    output=BASE/'publication/hold_telemetry_stage01';output.mkdir(exist_ok=False)
    paths=[Path(__file__),BASE/'protocol/protocol.json',BASE/'campaign/outputs_sealed.json']
    paths += [BASE/f'campaign/runs/{phase}_{policy}/precision.npz' for phase in PHASES for policy in ('N','H')]
    bindings={str(path):dict(sha256=sha(path),bytes=path.stat().st_size) for path in paths}
    save(output/'registered_before_inspection.json',dict(utc=datetime.now(timezone.utc).isoformat(),
         files=bindings,fixed_times_s=TIMES,scope='Posthoc descriptive fixed-time source telemetry; no altered gates or causal intervention.'))
    shutil.copy2(Path(__file__),output/'inspect_hold.py')
    rows=[]
    keys=('heading__target_rad','heading__raw_error_rad','heading__error_used_rad',
          'heading__vertical_lever','heading__horizontal_lever','heading__free_shell_mask',
          'heading__eligible','heading__effective_motor_limit_Nm','heading__wanted_world_yaw_torque_Nm',
          'heading__world_z_rate_rad_s','exact_heading_command','exact_drive_fade',
          'exact_clipping_command_delta','exact_estimated_support_loads_N','exact_qvel')
    for phase in PHASES:
        for policy in ('N','H'):
            case=f'{phase}_{policy}'
            with np.load(BASE/f'campaign/runs/{case}/precision.npz',allow_pickle=False) as data:
                for moment in TIMES:
                    index=int(np.argmin(abs(data['time_s']-moment)))
                    row=dict(case=case,requested_time_s=moment,actual_time_s=float(data['time_s'][index]))
                    row.update({key:data[key][index] for key in keys})
                    row['axis_below_existing_lever_cutoff']=bool(abs(row['heading__vertical_lever'])<.12)
                    row['no_shell_meets_existing_unloaded_condition']=bool(not row['heading__free_shell_mask'].any())
                    row['helper_is_disabled_by_configuration']=policy=='N'
                    rows.append(row)
    if any(sha(Path(path))!=v['sha256'] for path,v in bindings.items()):raise ValueError('Inspection input changed')
    save(output/'report.json',dict(iteration='I-74',rows=rows,
        scope='Fixed hold-entry/end observations. Eligibility barriers are recorded conditions, not a complete causal explanation of motion.',
        no_new_physics=True,no_new_estimator=True,acceptance_gates_unchanged=True))
    save(output/'artifact_manifest.json',dict(files={p.name:dict(sha256=sha(p),bytes=p.stat().st_size)
         for p in sorted(output.iterdir()) if p.is_file()}))
    print(json.dumps({'rows':len(rows),'output':str(output)}))


if __name__=='__main__':main()
