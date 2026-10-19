"""Matched 60-second yaw/recovery experiments from preserved source snapshots.

This benchmark caps numerical-library threads before importing NumPy. It changes
no physical parameter or input bound. Histories are replayed from equilibrium;
there are no plant-state resets after the initial reset.
"""
from __future__ import annotations
import os
for _thread_variable in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_thread_variable] = '1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse, copy, hashlib, importlib, json, shutil, sys, types
import mujoco
import numpy as np
import yaml
from spherical_wheel.model import generate_model

HERE = Path(__file__).resolve().parent
SUPPORTS = ('shell_A','shell_B','belt_A','belt_B')
SPECS = {
 'shell_medium_0.5N_same_y': ('shell',.5,.2,.25,[(0.,1.,0.)]),
 'transition_light_2N_same_y': ('transition',2.,.1,.4,[(0.,1.,0.)]),
 'shell_medium_1N_same_y': ('shell',1.,.2,.25,[(0.,1.,0.)]),
 'stable_shell_light_0.5N_alternating_y': ('shell',.5,.1,.4,[(0.,1.,0.),(0.,-1.,0.)]),
 'polar_medium_1N_cross_xy': ('polar',1.,.2,.25,[(1.,0.,0.),(0.,1.,0.),(-1.,0.,0.),(0.,-1.,0.)]),
 'hard_polar_overlap_2N_same_y': ('polar',2.,.3,.15,[(0.,1.,0.)]),
 'shell_overlap_0.5N_same_y': ('shell',.5,.3,.15,[(0.,1.,0.)]),
 'strong_shell_overlap_2N_same_y': ('shell',2.,.3,.15,[(0.,1.,0.)]),
 'strong_transition_overlap_2N_same_y': ('transition',2.,.3,.15,[(0.,1.,0.)]),
}
DEFAULT_CASES = (
 'shell_medium_0.5N_same_y', 'transition_light_2N_same_y', 'shell_medium_1N_same_y',
 'stable_shell_light_0.5N_alternating_y', 'polar_medium_1N_cross_xy',
 'hard_polar_overlap_2N_same_y', 'shell_overlap_0.5N_same_y',
)
REUSE = {
 'shell_medium_0.5N_same_y': 'shell/medium_0.5N_same_y/exact/rollout.npz',
 'transition_light_2N_same_y': 'transition/light_2N_same_y/exact/rollout.npz',
 'shell_medium_1N_same_y': 'shell/medium_1N_same_y/exact/rollout.npz',
}


def implementation(variant):
    namespace = '_yaw_evidence_'+variant
    if namespace not in sys.modules:
        package = types.ModuleType(namespace)
        package.__path__ = [str(HERE/(variant+'_source')/'spherical_wheel')]
        sys.modules[namespace] = package
    return importlib.import_module(namespace+'.lab_runtime').SimulationLab


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def observe(lab):
 p=lab.plant;m,d=p.model,p.data
 R=d.xmat[p.fork_id].reshape(3,3);w_world=R@d.qvel[3:6]
 spatial=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,p.fork_id,spatial,0)
 Mv=np.zeros(m.nv);mujoco.mj_mulM(m,d,Mv,d.qvel);ke=.5*float(d.qvel@Mv)
 mujoco.mj_subtreeVel(m,d)
 com=d.subtree_com[p.fork_id];momentum=m.body_subtreemass[p.fork_id]*d.subtree_linvel[p.fork_id]
 angular_momentum_z=float((d.subtree_angmom[p.fork_id]+np.cross(com,momentum))[2])
 quat=d.qpos[3:7];qw,qx,qy,qz=quat;heading=float(np.arctan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)))
 normal=np.zeros(4);belt_power=0.;world_ground_yaw_torque=0.
 for index,c in enumerate(d.contact):
  if c.efc_address<0:continue
  ids=[int(g) for g in c.geom];names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g) or '' for g in ids]
  if 'floor' not in names:continue
  side=0 if names[1]=='floor' else 1;geom=ids[side];name=names[side]
  category=next((j for j,prefix in enumerate(SUPPORTS) if name.startswith(prefix)),None)
  force=np.zeros(6);mujoco.mj_contactForce(m,d,index,force)
  if category is not None:normal[category]+=max(force[0],0.)
  applied_force=(1. if side==1 else -1.)*(c.frame.reshape(3,3).T@force[:3])
  applied_moment=(1. if side==1 else -1.)*(c.frame.reshape(3,3).T@force[3:])
  world_ground_yaw_torque+=float((np.cross(c.pos,applied_force)+applied_moment)[2])
  surface=d.geom_xmat[geom].reshape(3,3)@m.geom_surfacevel[geom,:3]
  belt_power-=float(applied_force@surface)
 weights=np.asarray(lab._last_sync['blend_weights']);sync=np.asarray(lab._last_sync['commands'])
 primary=np.asarray(lab._last_command).copy();primary[:3]=(primary[:3]-weights[:3]*sync[:3])/np.maximum(1.-weights[:3],1e-9)
 delta_sync=p.last_input[:3]-np.clip(primary[:3],-p.bounds[:3],p.bounds[:3])
 raw=d.qpos[7:10]-lab._last_ref[7:10];wrapped=(raw+np.pi)%(2*np.pi)-np.pi
 carrier=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'carrier');Rc=d.xmat[carrier].reshape(3,3)
 return {'time_s':float(d.time),'qpos':d.qpos.copy(),'qvel':d.qvel.copy(),'input':p.last_input.copy(),'command':lab._last_command.copy(),
   'force_N':lab.samples[-1]['force_N'].copy(),'tilt_deg':lab.samples[-1]['tilt_deg'],'heading_rad':heading,
   'world_angular_velocity_rad_s':w_world.copy(),'object_velocity_angular_rad_s':spatial[:3].copy(),
   'omega_verification_error_rad_s':float(np.max(abs(w_world-spatial[:3]))),'kinetic_energy_J':ke,
   'total_world_z_angular_momentum_kg_m2_s':angular_momentum_z,'ground_world_z_torque_Nm':world_ground_yaw_torque,
   'joint_power_W':float(p.last_input[:3]@d.qvel[6:9]),'joint_power_channels_W':p.last_input[:3]*d.qvel[6:9],
   'belt_power_W':belt_power,'passive_power_W':float(d.qfrc_passive@d.qvel),
   'synchronization_delta_joint_power_W':float(delta_sync@d.qvel[6:9]),'sync_weights':weights.copy(),'sync_commands':sync.copy(),
   'normal_forces_N':normal,'raw_joint_error_rad':raw.copy(),'wrapped_joint_error_rad':wrapped.copy(),
   'effective_carrier_phase_rad':float(np.arctan2(-Rc[2,0],Rc[2,2])),'reference_xy':lab._last_ref[:2].copy()}




def summary(arrays, case, variant, failed, reused=False):
    t = arrays['time_s']; velocity = arrays['qvel']; omega = arrays['world_angular_velocity_rad_s']
    dt = float(np.median(np.diff(t))) if len(t)>1 else .005
    yaw = np.unwrap(arrays['heading_rad'])
    final_time = float(t[-1]); tail = t>=max(0.,final_time-10.)
    planar = np.linalg.norm(velocity[:,:2],axis=1)
    error = np.linalg.norm(arrays['qpos'][:,:2]-arrays['reference_xy'],axis=1)
    fast_upright = (t>5.)&(abs(omega[:,2])>1.)&(arrays['tilt_deg']<5.)&(planar<.05)
    longest = streak = 0
    for flag in fast_upright:
        streak = streak+1 if flag else 0
        longest = max(longest,streak)
    wrap = np.argwhere(abs(np.diff(arrays['wrapped_joint_error_rad'],axis=0))>np.pi)
    events=[]
    for i,j in wrap:
        events.append({'time_s':float(t[i+1]),'joint_index':int(j),
                       'command_jump_norm':float(np.linalg.norm(arrays['command'][i+1]-arrays['command'][i]))})
    events=sorted(events,key=lambda row:row['command_jump_norm'],reverse=True)
    force=arrays['force_N']
    bounds=np.array([1.5,.6,.6,.3,.3])
    settings=SPECS[case]
    return {'case':case,'variant':variant,'scenario':settings[0],'failed':bool(failed),
       'requested_duration_s':60.,'completed_duration_s':final_time,'reused_exact_baseline':reused,
       'max_abs_world_yaw_rate_rad_s':float(np.max(abs(omega[:,2]))),
       'final_world_yaw_rate_rad_s':float(omega[-1,2]),'final_yaw_rpm':float(omega[-1,2]*60/(2*np.pi)),
       'heading_turns':float((yaw[-1]-yaw[0])/(2*np.pi)),
       'final_heading_deg':float(np.rad2deg(yaw[-1])),
       'final_tilt_deg':float(arrays['tilt_deg'][-1]),'max_tilt_deg':float(arrays['tilt_deg'].max()),
       'final_planar_speed_m_s':float(planar[-1]),'max_planar_speed_m_s':float(planar.max()),
       'final_total_angular_speed_rad_s':float(np.linalg.norm(omega[-1])),
       'final_position_error_m':float(error[-1]),'max_position_error_m':float(error.max()),
       'longest_fast_upright_low_translation_interval_s':longest*dt,
       'tail_mean_yaw_rate_rad_s':float(np.mean(omega[tail,2])),
       'tail_max_abs_yaw_rate_rad_s':float(np.max(abs(omega[tail,2]))),
       'initial_kinetic_energy_J':float(arrays['kinetic_energy_J'][0]),
       'final_kinetic_energy_J':float(arrays['kinetic_energy_J'][-1]),
       'max_kinetic_energy_J':float(arrays['kinetic_energy_J'].max()),
       'tail_mean_joint_power_W':float(np.mean(arrays['joint_power_W'][tail])),
       'tail_mean_belt_power_W':float(np.mean(arrays['belt_power_W'][tail])),
       'tail_mean_sync_delta_joint_power_W':float(np.mean(arrays['synchronization_delta_joint_power_W'][tail])),
       'tail_mean_passive_power_W':float(np.mean(arrays['passive_power_W'][tail])),
       'max_angular_velocity_crosscheck_error_rad_s':float(arrays['omega_verification_error_rad_s'].max()),
       'max_applied_inputs':np.max(abs(arrays['input']),axis=0).tolist(),
       'saturated_command_fraction':float(np.mean(np.any(abs(arrays['command'])>bounds,axis=1))),
       'max_combined_push_N':float(np.max(np.linalg.norm(force,axis=1))),
       'actual_push_impulse_Ns':(np.sum(force[1:],axis=0)*dt).tolist(),
       'raw_wrapped_reference_crossings':len(wrap),'largest_reference_crossings':events[:8],
       'settled_at_end':bool(not failed and arrays['tilt_deg'][-1]<.5 and planar[-1]<.03 and np.linalg.norm(omega[-1])<.08),
       'actuator_bounds_unchanged':True,'physical_guards_passed':True,
       'world_angular_velocity_definition':'fork body rotation @ free-joint local angular qvel; cross-checked against mj_objectVelocity in world axes',
       'shell_reference_crossing_note':'Raw material-phase reference crossings are diagnostic only; candidate feedback ignores the two shell position errors.',
       'pulse_settings':{'strength_N':settings[1],'duration_s':settings[2],'spacing_s':settings[3],
                         'directions':settings[4],'count':8,'first_start_s':1.}}


def run_one(case,variant,output,reuse_path=None):
    root=Path(output);folder=root/case;folder.mkdir(parents=True,exist_ok=True)
    config=yaml.safe_load((HERE/'configuration.yaml').read_text())
    model_path=root/('model_'+variant)/'model.xml'
    lab=implementation(variant)(config=copy.deepcopy(config),model_path=model_path)
    scenario,strength,duration,spacing,directions=SPECS[case]
    lab.command({'action':'reset','scenario':scenario})
    initial_mass=lab.plant.model.body_mass.copy();initial_inertia=lab.plant.model.body_inertia.copy()
    bounds=lab.plant.bounds.copy()
    if reuse_path is not None and variant=='baseline' and case in REUSE:
        source=Path(reuse_path)/REUSE[case]
        if source.is_file():
            with np.load(source,allow_pickle=False) as old:
                arrays={key:np.array(old[key],copy=True) for key in old.files}
            assert abs(float(arrays['time_s'][-1])-60.)<1e-8
            assert np.all(abs(arrays['input'])<=bounds+1e-12)
            arrays['model_path']=np.array(os.path.relpath(model_path,folder))
            arrays['disturbance']=np.c_[arrays['force_N'],np.zeros((len(arrays['time_s']),3))]
            arrays['tilt_rad']=np.deg2rad(arrays['tilt_deg'])
            np.savez_compressed(folder/(variant+'.npz'),**arrays)
            report=summary(arrays,case,variant,False,True)
            (folder/(variant+'.json')).write_text(json.dumps(report,indent=2))
            print(json.dumps({k:report[k] for k in ('case','variant','completed_duration_s','final_world_yaw_rate_rad_s','final_tilt_deg','reused_exact_baseline')}),flush=True)
            return report
    pulses={round((1.+j*spacing)/lab.dt):strength*np.asarray(directions[j%len(directions)]) for j in range(8)}
    records=[observe(lab)]
    for index in range(round(60./lab.dt)):
        if index in pulses:
            lab.command({'action':'push','force_N':pulses[index].tolist(),'duration_s':duration})
        lab.advance();records.append(observe(lab))
        data=lab.plant.data
        if np.any(data.qfrc_applied):raise AssertionError('Unexpected applied generalized force')
        unexpected=data.xfrc_applied.copy();unexpected[lab.plant.fork_id,:3]-=lab.samples[-1]['force_N']
        if np.any(unexpected):raise AssertionError('Unexpected world force or torque')
        if np.any(abs(lab.plant.last_input)>bounds):raise AssertionError('Actuator input exceeded unchanged bounds')
        if lab.plant.model.neq!=0 or lab.plant.model.jnt_type[0]!=mujoco.mjtJoint.mjJNT_FREE:
            raise AssertionError('Free root/constraint topology changed')
        if index%200==0:
            np.testing.assert_array_equal(lab.plant.model.body_mass,initial_mass)
            np.testing.assert_array_equal(lab.plant.model.body_inertia,initial_inertia)
        if (index+1)%round(10./lab.dt)==0:
            r=records[-1];print(json.dumps({'progress':case,'variant':variant,'time_s':r['time_s'],
                'yaw_rad_s':float(r['world_angular_velocity_rad_s'][2]),'tilt_deg':r['tilt_deg'],
                'xy_speed_m_s':float(np.linalg.norm(r['qvel'][:2]))}),flush=True)
        if lab.failed:break
    arrays={key:np.asarray([row[key] for row in records]) for key in records[0]}
    arrays['unwrapped_heading_rad']=np.unwrap(arrays['heading_rad'])
    arrays['model_path']=np.array(os.path.relpath(model_path,folder))
    arrays['disturbance']=np.c_[arrays['force_N'],np.zeros((len(records),3))]
    arrays['tilt_rad']=np.deg2rad(arrays['tilt_deg'])
    np.savez_compressed(folder/(variant+'.npz'),**arrays)
    report=summary(arrays,case,variant,lab.failed)
    (folder/(variant+'.json')).write_text(json.dumps(report,indent=2))
    print(json.dumps({k:report[k] for k in ('case','variant','completed_duration_s','failed','max_abs_world_yaw_rate_rad_s','final_world_yaw_rate_rad_s','final_tilt_deg','final_position_error_m','settled_at_end')}),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=HERE)
    parser.add_argument('--workers',type=int,default=3);parser.add_argument('--cases',nargs='*',default=list(DEFAULT_CASES))
    parser.add_argument('--plot-only',action='store_true')
    parser.add_argument('--reuse-known-baselines',type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if args.plot_only:
        plot(args.output);return
    config=yaml.safe_load((HERE/'configuration.yaml').read_text())
    models=[]
    for variant in ('baseline','candidate'):
        path=generate_model(config,args.output/('model_'+variant));models.append(path)
        print('Preparing '+variant+' gain library',flush=True)
        lab=implementation(variant)(config=copy.deepcopy(config),model_path=path)
        assert lab.plant.model.neq==0
    assert sha(models[0])==sha(models[1])
    metadata={'baseline_commit':'3bfd7998ed72146521894f445c2907129ed9802c',
      'source_snapshots':{variant:json.loads((HERE/(variant+'_source')/'provenance.json').read_text()) for variant in ('baseline','candidate')},
      'configuration_sha256':sha(HERE/'configuration.yaml'),'model_xml_sha256':sha(models[0]),
      'thread_environment':{name:os.environ.get(name) for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
      'same_physics_and_input_bounds':True,'input_bounds':[1.5,.6,.6,.3,.3],
      'candidate_feedback_change':'Zero shell material-position error[7:9]; preserve shaft phase, all velocities, synchronization and actuator limits.',
      'cases':args.cases,'duration_s':60.,'records':[],
      'limits':'Sampled closed-loop evidence with provisional physical parameters and ideal belt speed servos. Heading is unregulated; finite heading change does not imply spinning. Failure cases stop at the unchanged fall guard.'}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(run_one,case,variant,str(args.output),args.reuse_known_baselines) for case in args.cases for variant in ('baseline','candidate')]
        for future in as_completed(futures):
            metadata['records'].append(future.result())
            (args.output/'summary.json').write_text(json.dumps(metadata,indent=2))
    plot(args.output)
    print(json.dumps({'finished_runs':len(metadata['records']),'output':str(args.output)}),flush=True)


def plot(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=[('yaw','World vertical angular velocity (rpm)'),('tilt','Platform tilt (deg)'),
          ('speed','Planar platform speed (m/s)'),('energy','System kinetic energy (J)')]
    cases=[('shell_medium_0.5N_same_y','Shell: 8 × 0.5 N, 0.2 s, every 0.25 s'),
           ('transition_light_2N_same_y','Transition: 8 × 2 N, 0.1 s, every 0.4 s')]
    if not all((Path(output)/case/'candidate.npz').is_file() for case,_ in cases):
        cases=[('strong_shell_overlap_2N_same_y','Shell: 8 × 2 N, 0.3 s, every 0.15 s'),
               ('strong_transition_overlap_2N_same_y','Transition: 8 × 2 N, 0.3 s, every 0.15 s')]
    fig,axes=plt.subplots(4,2,figsize=(14,11),constrained_layout=True,sharex=True)
    for col,(case,title) in enumerate(cases):
        final=[]
        for variant,label,color in [('baseline','Baseline 0.3','#2864a4'),('candidate','Revised','#dd751f')]:
            with np.load(Path(output)/case/(variant+'.npz')) as data:
                t=data['time_s'];yaw=data['world_angular_velocity_rad_s'][:,2]*60/(2*np.pi)
                values=[yaw,data['tilt_deg'],np.linalg.norm(data['qvel'][:,:2],axis=1),data['kinetic_energy_J']]
                final.append((float(yaw[-1]),float(t[-1])))
                shown_label=label if t[-1]>59.9 else f'{label} (fell at {t[-1]:.2f} s)'
                for row,value in enumerate(values):axes[row,col].plot(t,value,label=shown_label,color=color,linewidth=1.)
        for row,(_,ylabel) in enumerate(rows):
            axis=axes[row,col]
            axis.set(ylabel=ylabel,xlim=(0,60));axis.grid(alpha=.25)
            last=1.+7*SPECS[case][3]+SPECS[case][2]
            axis.axvspan(1.,last,color='gray',alpha=.12,label='8 finite pushes' if row==0 else None)
        axes[0,col].set_title(title)
        final_note=(f'Final: {final[0][0]:.2f} → {final[1][0]:.4f} rpm' if min(final[0][1],final[1][1])>59.9
                    else f'Baseline fell at {final[0][1]:.2f} s; revised final {final[1][0]:.4f} rpm')
        axes[0,col].text(.98,.96,final_note,
                        transform=axes[0,col].transAxes,ha='right',va='top',fontsize=10,
                        bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
        axes[0,col].legend(loc='upper left',fontsize=9)
        axes[3,col].set_xlabel('Simulation time (s)')
    fig.suptitle('60 s with the same pushes and physical limits — actual world-vertical yaw',fontsize=15)
    fig.savefig(Path(output)/'comparison.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
