"""Reproduce matched recovery and incoming-contact synchronization experiments.

Run from the repository root. All modes use the same unmodified generated
plant, actuator bounds, initial conditions and scheduled external pushes.
The preserved historical runtime is loaded only after shared physical-source
hashes match its recorded dependencies. Results are design-study evidence.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
from importlib.machinery import SourceFileLoader
import json
from pathlib import Path
import types
import mujoco
import numpy as np
import yaml
from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.control import find_equilibrium
from spherical_wheel.model import REPO_ROOT, generate_model

HERE = Path(__file__).resolve().parent
SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
CASES = {
    'shell_overlap': {'scenario':'shell', 'strength_N':.5, 'duration_s':.3,
                      'spacing_s':.15, 'directions':[(0.,1.,0.)]},
    'polar_cross': {'scenario':'polar', 'strength_N':1., 'duration_s':.2,
                    'spacing_s':.25, 'directions':[(1.,0.,0.),(0.,1.,0.),(-1.,0.,0.),(0.,-1.,0.)]},
}


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _legacy():
    expected = json.loads((HERE/'baseline'/'plant_sha256.json').read_text())
    for name, digest in expected.items():
        if _sha(REPO_ROOT/name) != digest:
            raise RuntimeError(f'Historical comparison requires its unchanged physical dependency: {name}')
    source = HERE/'baseline'/'lab_runtime.py.txt'
    name = 'spherical_wheel.saved_baseline_runtime'
    spec = importlib.util.spec_from_loader(name, SourceFileLoader(name, str(source)))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.SimulationLab


def measure(lab):
    """Actual loaded-contact slip and candidate support-point material velocity."""
    model, data = lab.plant.model, lab.plant.data
    load, slip = np.zeros(4), np.zeros(4)
    for index, contact in enumerate(data.contact):
        if contact.efc_address < 0:
            continue
        ids = [int(g) for g in contact.geom]
        names = [mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '' for g in ids]
        if 'floor' not in names:
            continue
        side = 0 if names[1]=='floor' else 1
        category = next((i for i,prefix in enumerate(SUPPORTS) if names[side].startswith(prefix)),None)
        if category is None:
            continue
        geom = ids[side]
        force = np.zeros(6)
        mujoco.mj_contactForce(model,data,index,force)
        load[category] += max(0.,force[0])
        jacobian = np.zeros((3,model.nv))
        mujoco.mj_jac(model,data,jacobian,None,contact.pos,int(model.geom_bodyid[geom]))
        velocity = jacobian@data.qvel + data.geom_xmat[geom].reshape(3,3)@model.geom_surfacevel[geom,:3]
        slip[category] = max(slip[category],float(np.linalg.norm(contact.frame.reshape(3,3)[1:]@velocity)))
    point = data.qpos[:3]+[0.,0.,-lab.config['geometry']['sphere_radius_m']]
    candidate = []
    for name in SUPPORTS:
        geom = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name+'_smooth')
        jacobian = np.zeros((3,model.nv))
        mujoco.mj_jac(model,data,jacobian,None,point,int(model.geom_bodyid[geom]))
        velocity = jacobian@data.qvel + data.geom_xmat[geom].reshape(3,3)@model.geom_surfacevel[geom,:3]
        candidate.append(velocity[:2])
    return load, slip, np.asarray(candidate)


def physical_guards(lab, mass, inertia):
    model, data = lab.plant.model, lab.plant.data
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE and model.neq == 0
    np.testing.assert_array_equal(model.body_mass,mass)
    np.testing.assert_array_equal(model.body_inertia,inertia)
    np.testing.assert_array_equal(data.qfrc_applied,0.)
    unexpected = data.xfrc_applied.copy()
    unexpected[lab.plant.fork_id,:3] -= lab.samples[-1]['force_N']
    np.testing.assert_array_equal(unexpected,0.)
    assert np.all(abs(lab.samples[-1]['input']) <= lab.plant.bounds)


def _arrays(lab):
    samples = list(lab.samples)
    keys = ('time_s','interval_start_s','qpos','qvel','command','input','force_N','reference_xy','tilt_deg')
    arrays = {key:np.asarray([sample[key] for sample in samples]) for key in keys}
    for key in ('sync_weights','sync_commands'):
        if key in samples[-1]:
            arrays[key] = np.asarray([sample[key] for sample in samples])
    arrays['disturbance'] = np.c_[arrays['force_N'],np.zeros((len(samples),3))]
    arrays['tilt_rad'] = np.deg2rad(arrays['tilt_deg'])
    arrays['model_path'] = np.array('model/model.xml')
    return arrays


def run_recovery(lab, case, mode, output):
    settings = CASES[case]
    if mode != 'local_lqr':
        lab.command({'action':'synchronization','enabled':mode=='transition_sync'})
    lab.command({'action':'reset','scenario':settings['scenario']})
    mass, inertia = lab.plant.model.body_mass.copy(),lab.plant.model.body_inertia.copy()
    pulse_steps = {round((1.+j*settings['spacing_s'])/lab.dt):
                   settings['strength_N']*np.asarray(settings['directions'][j%len(settings['directions'])])
                   for j in range(8)}
    normal, slips, velocities, transfers = [],[],[],[]
    previous = None
    for index in range(round(12./lab.dt)):
        if index in pulse_steps:
            lab.command({'action':'push','force_N':pulse_steps[index].tolist(),'duration_s':settings['duration_s']})
        lab.advance()
        load, slip, candidate = measure(lab)
        normal.append(load);slips.append(slip);velocities.append(candidate)
        dominant = SUPPORTS[int(np.argmax(load))] if load.max()>.01 else None
        if dominant != previous:
            transfers.append({'time_s':float(lab.plant.data.time),'from':previous,'to':dominant,
                              'normal_N':load.tolist(),'slip_m_s':slip.tolist()})
            previous = dominant
        physical_guards(lab,mass,inertia)
        if lab.failed:
            break
    arrays = _arrays(lab)
    arrays.update(normal_forces_N=np.asarray(normal),contact_slip_m_s=np.asarray(slips),
                  candidate_xy_velocity_m_s=np.asarray(velocities))
    np.savez_compressed(output/f'{case}_{mode}.npz',**arrays)
    record = {'case':case,'mode':mode,'failed':lab.failed,'time_s':float(lab.plant.data.time),
              'max_tilt_deg':float(arrays['tilt_deg'].max()),'final_tilt_deg':float(arrays['tilt_deg'][-1]),
              'final_xy_error_m':float(np.linalg.norm(arrays['qpos'][-1,:2]-arrays['reference_xy'][-1])),
              'final_xy_speed_m_s':float(np.linalg.norm(arrays['qvel'][-1,:2])),
              'final_angular_speed_rad_s':float(np.linalg.norm(arrays['qvel'][-1,3:6])),
              'saturation_fraction':float(np.mean(np.any(abs(arrays['command'])>lab.plant.bounds+1e-12,axis=1))),
              'push_impulse_Ns':(arrays['force_N'].sum(axis=0)*lab.dt).tolist(),
              'max_contact_slip_m_s':np.max(slips,axis=0).tolist(),'transfers':transfers,
              'fixed_reference_xy_m':lab.anchor.tolist(),'physical_guards_passed':True}
    print(json.dumps({k:v for k,v in record.items() if k!='transfers'}),flush=True)
    return record


def run_probe(lab, sync, output):
    """Same initial state/reference; only incoming synchronization is toggled."""
    lab.command({'action':'synchronization','enabled':sync})
    lab.command({'action':'reset','scenario':'polar'})
    equilibrium = find_equilibrium(lab.designer,1.74)
    velocity = np.zeros(9);velocity[:2]=[.096,.1];velocity[6]=.8
    lab.plant.reset(qpos=equilibrium['qpos'],qvel=velocity)
    lab.nominal_joints = equilibrium['qpos'][7:10].copy()
    lab.anchor = equilibrium['qpos'][:2].copy()
    lab.plant.apply_input([0.,0.,0.,-.1,0.])
    planar_velocity, anchor = velocity[:2].copy(),lab.anchor.copy()
    original_reference = lab._reference
    lab._reference = types.MethodType(lambda self:(anchor+self.plant.data.time*planar_velocity,
                                                   planar_velocity,np.zeros(2)),lab)
    lab.trajectory['kind']='line'
    lab._last_ref = equilibrium['qpos'].copy()
    lab._last_input = lab.plant.last_input.copy()
    lab._last_command = lab.plant.last_command.copy()
    lab.samples.clear();lab.history.clear();lab._record(0.)
    mass,inertia = lab.plant.model.body_mass.copy(),lab.plant.model.body_inertia.copy()
    observations = []
    try:
        for _ in range(100):
            normal,slip,candidate = measure(lab)
            observations.append([lab.plant.data.time,np.linalg.norm(candidate[0]),normal[0],slip[0],
                                 lab.samples[-1].get('sync_weights',np.zeros(5))[1],lab.plant.data.qvel[7]])
            lab.advance();physical_guards(lab,mass,inertia)
            assert not lab.failed
    finally:
        lab._reference = original_reference
    observed = np.asarray(observations)
    loaded = np.flatnonzero(observed[:,2]>1.)
    assert loaded.size
    first = int(loaded[0])
    arrays = _arrays(lab)
    np.testing.assert_array_equal(arrays['qpos'][0],equilibrium['qpos'])
    np.testing.assert_array_equal(arrays['qvel'][0],velocity)
    assert abs(float(arrays['qpos'][0,7])-1.74)<1e-12
    arrays['incoming_observations'] = observed
    arrays['incoming_observation_columns'] = np.array(['time_s','candidate_shell_A_speed_m_s','shell_A_normal_N',
                                                        'shell_A_contact_slip_m_s','shell_A_sync_weight','shell_A_speed_rad_s'])
    name = 'probe_sync_on' if sync else 'probe_sync_off'
    np.savez_compressed(output/f'{name}.npz',**arrays)
    record = {'case':'controlled_cap_exit','synchronization_enabled':sync,'failed':lab.failed,
              'initial_carrier_phase_rad':1.74,'initial_planar_velocity_m_s':[.096,.1],
              'initial_shaft_speed_rad_s':.8,'initial_belt_A_speed_m_s':-.1,
              'reference':'Same constant planar velocity [.096,.1] m/s; no external push.',
              'incoming_first_1N_load_s':float(observed[first,0]),
              'pre5ms_mismatch_m_s':float(observed[max(0,first-1),1]),
              'pre10ms_mismatch_m_s':float(observed[max(0,first-2),1]),
              'pre20ms_mismatch_m_s':float(observed[max(0,first-4),1]),
              'slip_at_first_1N_load_m_s':float(observed[first,3]),
              'peak_incoming_contact_slip_m_s':float(observed[:,3].max()),
              'preload_sync_weight_max':float(observed[:first,4].max()),
              'final_tilt_deg':float(arrays['tilt_deg'][-1]),'physical_guards_passed':True}
    print(json.dumps(record),flush=True)
    return record


def plot(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,3,figsize=(14,7),constrained_layout=True)
    modes = [('local_lqr','Original LQR'),('transition_no_sync','Scheduled, sync off'),('transition_sync','Scheduled, sync on')]
    for col,case in enumerate(CASES):
        for mode,label in modes:
            with np.load(output/f'{case}_{mode}.npz') as data:
                axes[0,col].plot(data['time_s'],data['tilt_deg'],label=label)
                error = np.linalg.norm(data['qpos'][:,:2]-data['reference_xy'],axis=1)*1000
                axes[1,col].plot(data['time_s'],error,label=label)
        axes[0,col].set(title=case.replace('_',' '),ylabel='Platform tilt (deg)',yscale='symlog')
        axes[1,col].set(xlabel='Simulation time (s)',ylabel='Error to unchanged position target (mm)')
        axes[0,col].legend(fontsize=8)
    for sync,label in [(False,'Sync off'),(True,'Sync on')]:
        with np.load(output/('probe_sync_on.npz' if sync else 'probe_sync_off.npz')) as data:
            obs=data['incoming_observations']
            axes[0,2].plot(obs[:,0],obs[:,1],label=label)
            axes[1,2].plot(obs[:,0],obs[:,3],label=label)
            load=np.flatnonzero(obs[:,2]>1.)
            for row in (0,1):axes[row,2].axvline(obs[load[0],0],alpha=.3,linestyle='--')
    axes[0,2].set(title='Same initial state: prepare incoming shell',ylabel='Incoming candidate speed (m/s)',xlim=(0,.17))
    axes[1,2].set(xlabel='Simulation time (s); dashed = first 1 N load',ylabel='Actual incoming contact slip (m/s)',xlim=(0,.17))
    axes[0,2].legend(fontsize=8)
    for row in axes:
        for axis in row:axis.grid(alpha=.25)
    fig.suptitle('Matched provisional plant and actuator bounds — position bias remains in shell recovery')
    fig.savefig(output/'comparison.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=HERE);parser.add_argument('--plot-only',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if args.plot_only:
        plot(args.output);return
    model=generate_model(output_dir=args.output/'model')
    advanced=SimulationLab(model_path=model)
    baseline=_legacy()(model_path=model)
    assert advanced.config==baseline.config
    np.testing.assert_array_equal(advanced.plant.bounds,baseline.plant.bounds)
    (args.output/'configuration.yaml').write_text(yaml.safe_dump(advanced.config,sort_keys=False))
    summary={'baseline_commit':(HERE/'baseline'/'commit.txt').read_text().strip(),
             'baseline_runtime_sha256':_sha(HERE/'baseline'/'lab_runtime.py.txt'),
             'current_source_sha256':{name:_sha(REPO_ROOT/'src'/'spherical_wheel'/name) for name in
                                      ('lab_runtime.py','recovery_control.py','synchronization.py')},
             'input_names':advanced.plant.input_names,'input_units':advanced.plant.input_units,
             'unchanged_input_bounds':advanced.plant.bounds.tolist(),'same_configuration_all_modes':True,
             'gain_library':advanced.gain_library_metadata,'support_order':SUPPORTS,'pulse_settings':CASES,
             'recovery_cases':[],'controlled_probes':[],
             'limitations':'Provisional physical parameters and ideal belt speed servos. One shell-recovery case retains position bias. Local sampled evidence does not establish global recovery or hardware performance.'}
    for case in CASES:
        for mode in ('local_lqr','transition_no_sync','transition_sync'):
            summary['recovery_cases'].append(run_recovery(baseline if mode=='local_lqr' else advanced,case,mode,args.output))
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    for sync in (False,True):
        summary['controlled_probes'].append(run_probe(advanced,sync,args.output))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    plot(args.output)


if __name__=='__main__':
    main()
