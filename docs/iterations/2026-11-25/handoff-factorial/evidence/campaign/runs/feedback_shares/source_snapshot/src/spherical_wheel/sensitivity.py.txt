"""Small fixed-gain study of the unfinished payload mass and COM height."""
from pathlib import Path
import copy
import json
import numpy as np
from .model import load_config, generate_model
from .sim import Plant
from .control import find_equilibrium, design_lqr


def run_sensitivity(output='results/sensitivity',duration=5.):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    nominal=load_config()
    plant=Plant(nominal)
    equilibrium=find_equilibrium(plant)
    gain=design_lqr(plant,equilibrium)['K']
    cases=[]
    for label,mass,height in [('lighter_lower',.8,.18),('nominal',1.,.20),('heavier_higher',1.2,.22)]:
        config=copy.deepcopy(nominal)
        config['geometry']['platform']['mass_kg']=mass
        config['geometry']['platform']['center_xyz_m'][2]=height
        path=generate_model(config,output/label/'model')
        variant=Plant(config,path)
        eq=find_equilibrium(variant)
        variant.reset(qpos=eq['qpos'])
        dx=np.zeros(18);dx[3:5]=np.deg2rad([2.,2.]);variant.perturb(dx)
        history=[];failed=False
        for _ in range(round(duration/variant.dt)):
            u=eq['u']-gain@variant.error(eq['qpos'])
            state=variant.step(u)
            history.append(np.r_[state['time_s'],state['tilt_rad'],variant.data.qpos[:2],state['input']])
            if state['balance_failed'] or state['tilt_rad']>np.deg2rad(75):
                failed=True;break
        history=np.asarray(history)
        np.savez_compressed(output/label/'rollout.npz',time_s=history[:,0],tilt_rad=history[:,1],position_xy=history[:,2:4],input=history[:,4:])
        cases.append({'case':label,'platform_mass_kg':mass,'platform_center_above_sphere_m':height,
                      'controller':'same_nominal_gain_no_retuning','completed_duration_s':float(state['time_s']),
                      'failed':failed,'max_tilt_deg':float(np.rad2deg(history[:,1]).max()),
                      'final_tilt_deg':float(np.rad2deg(history[-1,1])),
                      'final_position_error_m':float(np.linalg.norm(history[-1,2:4]))})
    result={'study':'fixed_nominal_gain_mass_and_COM_sensitivity','initial_tilt_offset_deg':[2.,2.],
            'duration_s':duration,'cases':cases,'limitations':'Three illustrative payload cases; not a broad robustness guarantee.'}
    (output/'summary.json').write_text(json.dumps(result,indent=2))
    return result
