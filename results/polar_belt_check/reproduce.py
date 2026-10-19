"""Independent same-gain polar drive ablation; run from repository root."""
from pathlib import Path
import json
import numpy as np
import mujoco
from spherical_wheel.sim import Plant
from spherical_wheel.model import generate_model
from spherical_wheel.control import find_equilibrium, design_lqr, jsonable

out=Path(__file__).resolve().parent
plant=Plant(model_path=generate_model(output_dir=out/'model'))
eq=find_equilibrium(plant, np.pi/2)
design=design_lqr(plant,eq)
np.savez_compressed(out/'design.npz', **{k:v for k,v in {**eq,**design}.items() if isinstance(v,(np.ndarray,float,int))})
summary={'engine':mujoco.__version__,'phase_rad':np.pi/2,'dt_s':plant.dt,'duration_s':8.,'inputs':plant.input_names,'units':plant.input_units,
         'same_gains_all_cases':True,'belt_ablation':'set requested command entries [3:] to zero before Plant.step; belt geometry/friction unchanged',
         'initial_condition':'equilibrium plus pure free-body roll perturbation in tangent coordinate 3; pitch 0',
         'push':'world Fy on fork COM, applied on [1.0,1.1) seconds',
         'caveat':'Ideal belt surface-speed servos; assumed masses/friction/limits; not proof of belt-only or hardware performance',
         'equilibrium':jsonable(eq),'runs':[]}

for kind,sign in [('roll',1),('roll',-1),('push',1),('push',-1)]:
  for belts in (True,False):
    name=f'{kind}_{"positive" if sign>0 else "negative"}_{"belts" if belts else "no_belts"}'
    plant.reset(qpos=eq['qpos'],qvel=eq['qvel'])
    dx=np.zeros(18)
    if kind=='roll': dx[3]=np.deg2rad(2.*sign)
    plant.perturb(dx)
    rows=[]
    for i in range(round(8./plant.dt)+1):
      t=float(plant.data.time)
      err=plant.error(eq['qpos'],eq['qvel'])
      command=eq['u']-design['K']@err
      applied=command.copy()
      if not belts: applied[3:]=0.
      disturbance=np.zeros(6)
      if kind=='push' and 1.-1e-9<=t<1.1-1e-9: disturbance[1]=sign*.5
      snapshot=plant.snapshot()
      belt_load=0.
      for c in plant.contact_summary():
        if any(n.startswith('belt_') for n in c['geoms']): belt_load+=c['normal_force_N']
      rows.append([t,*plant.data.qpos,*plant.data.qvel,*command,*np.clip(applied,-plant.bounds,plant.bounds),*disturbance,snapshot['tilt_rad'],float(np.linalg.norm(err[3:5])),belt_load,float(plant.balance_failed)])
      if plant.balance_failed or snapshot['tilt_rad']>np.deg2rad(75.) or i==round(8./plant.dt): break
      plant.step(applied,disturbance)
    log=np.asarray(rows)
    columns=['time_s',*[f'qpos_{i}' for i in range(10)],*[f'qvel_{i}' for i in range(9)],*[f'command_{n}' for n in plant.input_names],*[f'input_{n}' for n in plant.input_names],*[f'push_{n}' for n in ['Fx','Fy','Fz','Tx','Ty','Tz']],'tilt_rad','relative_tilt_rad','belt_normal_N','balance_failed']
    np.savez_compressed(out/f'{name}.npz',log=log,columns=np.asarray(columns))
    index={n:i for i,n in enumerate(columns)}
    last=log[-1]
    item={'case':name,'belts_enabled':belts,'initial_roll_deg':2.*sign if kind=='roll' else 0.,'push_Fy_N':sign*.5 if kind=='push' else 0.,'completed_duration_s':float(last[0]),
      'final_relative_tilt_deg':float(np.rad2deg(last[index['relative_tilt_rad']])),'max_relative_tilt_deg':float(np.rad2deg(log[:,index['relative_tilt_rad']]).max()),
      'fork_floor_contact':bool(last[index['balance_failed']]),'max_applied_input':np.max(abs(log[:-1,[index[f'input_{n}'] for n in plant.input_names]]),axis=0).tolist(),
      'peak_belt_normal_N':float(log[:,index['belt_normal_N']].max()),'total_push_impulse_Ns':float(np.sum(log[:-1,index['push_Fy']])*plant.dt),
      'nonzero_disturbance_channels':[n for n in ['Fx','Fy','Fz','Tx','Ty','Tz'] if np.any(log[:,index[f'push_{n}']]!=0)]}
    summary['runs'].append(item)
    print(json.dumps(item),flush=True)
(out/'summary.json').write_text(json.dumps(summary,indent=2))
