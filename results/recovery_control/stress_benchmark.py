"""Deterministic repeated-push benchmark. Select implementation via PYTHONPATH."""
from __future__ import annotations
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import numpy as np

PROFILES={'light':(.1,.4),'medium':(.2,.25),'overlap':(.3,.15)}
PATTERNS={'same_y':[(0.,1.,0.)], 'alternating_y':[(0.,1.,0.),(0.,-1.,0.)],
          'cross_xy':[(1.,0.,0.),(0.,1.,0.),(-1.,0.,0.),(0.,-1.,0.)]}


def worker(scenario, output, amplitudes, patterns, profiles, duration, pulses, controller_mode):
    from spherical_wheel.lab_runtime import SimulationLab
    from spherical_wheel.model import generate_model
    import spherical_wheel.lab_runtime as runtime_module
    root=Path(output)/scenario;root.mkdir(parents=True,exist_ok=True)
    lab=SimulationLab(model_path=generate_model(output_dir=root/'model'), controller_mode=controller_mode)
    records=[]
    for profile in profiles:
      pulse_duration,spacing=PROFILES[profile]
      for amplitude in amplitudes:
       for pattern in patterns:
        lab.command({'action':'controller','enabled':True});lab.command({'action':'belt','enabled':True})
        lab.command({'action':'reset','scenario':scenario})
        name=f'{profile}_{amplitude:g}N_{pattern}'
        target=root/name;target.mkdir(exist_ok=True)
        events={round((1.+j*spacing)/lab.dt):amplitude*np.array(PATTERNS[pattern][j%len(PATTERNS[pattern])]) for j in range(pulses)}
        contact_t=[];normal=[];phase=[];contact_names=set()
        max_qfrc=0.;max_unexpected_xfrc=0.;pushes_applied=0
        wall=time.monotonic()
        for i in range(round(duration/lab.dt)):
          if i in events:
            lab.command({'action':'push','force_N':events[i].tolist(),'duration_s':pulse_duration});pushes_applied+=1
          lab.advance()
          max_qfrc=max(max_qfrc,float(np.max(abs(lab.plant.data.qfrc_applied))))
          unexpected=lab.plant.data.xfrc_applied.copy();unexpected[lab.plant.fork_id,:3]-=lab.samples[-1]['force_N']
          max_unexpected_xfrc=max(max_unexpected_xfrc,float(np.max(abs(unexpected))))
          if i%10==0 or lab.failed:
            loads=np.zeros(5)
            for contact in lab.plant.contact_summary():
              if 'floor' not in contact['geoms'] or contact['normal_force_N']<.001:continue
              for n in contact['geoms']:
                if n=='floor':continue
                contact_names.add(n)
                for k,prefix in enumerate(('shell_A','shell_B','belt_A','belt_B','fork')):
                  if n.startswith(prefix) or (k==4 and n=='platform'):loads[k]+=contact['normal_force_N'];break
            contact_t.append(float(lab.plant.data.time));normal.append(loads);phase.append(lab.selected['phase'])
          if lab.failed:break
        samples=list(lab.samples)
        arrays={k:np.asarray([s[k] for s in samples]) for k in ('time_s','interval_start_s','qpos','qvel','command','input','force_N','reference_xy','tilt_deg')}
        np.savez_compressed(target/'rollout.npz',**arrays,contact_time_s=np.asarray(contact_t),normal_forces_N=np.asarray(normal),selected_gain_phase_rad=np.asarray(phase))
        u=arrays['input'];command=arrays['command'];velocity=arrays['qvel'];sat=abs(command)>lab.plant.bounds+1e-12
        record={'controller_mode':controller_mode,'scenario':scenario,'case':name,'pattern':pattern,'per_push_force_N':amplitude,'pulse_duration_s':pulse_duration,'spacing_s':spacing,
                'requested_pushes':pulses,'applied_pushes':pushes_applied,'requested_duration_s':duration,'completed_duration_s':float(arrays['time_s'][-1]),
                'failed':bool(lab.failed),'fall_time_s':float(arrays['time_s'][-1]) if lab.failed else None,
                'max_tilt_deg':float(arrays['tilt_deg'].max()),'final_tilt_deg':float(arrays['tilt_deg'][-1]),
                'final_position_error_m':float(np.linalg.norm(arrays['qpos'][-1,:2]-arrays['reference_xy'][-1])),
                'max_applied_inputs':np.max(abs(u),axis=0).tolist(),'max_command_to_limit_ratio':np.max(abs(command)/lab.plant.bounds,axis=0).tolist(),
                'saturated_interval_fraction':float(np.mean(np.any(sat,axis=1))),'channel_saturated_fraction':np.mean(sat,axis=0).tolist(),
                'shaft_phase_range_rad':[float(arrays['qpos'][:,7].min()),float(arrays['qpos'][:,7].max())],
                'max_base_speed_m_s':float(np.max(np.linalg.norm(velocity[:,:3],axis=1))),
                'max_base_angular_speed_rad_s':float(np.max(np.linalg.norm(velocity[:,3:6],axis=1))),
                'max_drive_speeds_rad_s':np.max(abs(velocity[:,6:9]),axis=0).tolist(),
                'observed_floor_contact_names':sorted(contact_names),'total_push_impulse_Ns':(arrays['force_N'].sum(axis=0)*lab.dt).tolist(),
                'max_combined_push_N':float(np.max(np.linalg.norm(arrays['force_N'],axis=1))),
                'max_uncommanded_generalized_force':max_qfrc,'max_unexpected_world_force':max_unexpected_xfrc,
                'trajectory':lab.trajectory,'wall_time_s':time.monotonic()-wall,'source_file':runtime_module.__file__,
                'source_sha256':hashlib.sha256(Path(runtime_module.__file__).read_bytes()).hexdigest()}
        (target/'summary.json').write_text(json.dumps(record,indent=2));records.append(record)
        (root/'summaries.json').write_text(json.dumps(records,indent=2))
        print(f'{scenario:10s} {name:30s} {"FALL" if lab.failed else "ok":4s} t={record["completed_duration_s"]:5.2f} maxTilt={record["max_tilt_deg"]:7.2f} final={record["final_tilt_deg"]:6.2f} sat={record["saturated_interval_fraction"]:.3f}',flush=True)
    return records


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--controller-mode',choices=['transition_aware','local_lqr'],default='transition_aware');parser.add_argument('--scenarios',nargs='+',default=['shell','polar','transition']);parser.add_argument('--amplitudes',nargs='+',type=float,default=[.5,1.,2.]);parser.add_argument('--patterns',nargs='+',default=list(PATTERNS));parser.add_argument('--profiles',nargs='+',default=list(PROFILES));parser.add_argument('--duration',type=float,default=10.);parser.add_argument('--pulses',type=int,default=8);parser.add_argument('--workers',type=int,default=3)
    args=parser.parse_args();Path(args.output).mkdir(parents=True,exist_ok=True)
    (Path(args.output)/'invocation.json').write_text(json.dumps({'arguments':vars(args),'python':sys.executable,'PYTHONPATH':os.environ.get('PYTHONPATH')},indent=2))
    all_records=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
      futures=[pool.submit(worker,s,args.output,args.amplitudes,args.patterns,args.profiles,args.duration,args.pulses,args.controller_mode) for s in args.scenarios]
      for future in as_completed(futures):all_records.extend(future.result())
    (Path(args.output)/'summaries.json').write_text(json.dumps(all_records,indent=2))
    print(json.dumps({'cases':len(all_records),'failed':sum(r['failed'] for r in all_records),'output':args.output}),flush=True)
if __name__=='__main__':main()
