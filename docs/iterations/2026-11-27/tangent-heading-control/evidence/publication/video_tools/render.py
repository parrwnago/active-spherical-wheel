"""I75 saved-Gazebo-pose comparison videos; no controller or physics replay.

All six complete paths determine one static world-up camera. Each phase page
shows both registered methods for the full 45s, including failed/truncated
states, which are frozen and visibly marked after their final recorded time.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import sys
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parents[2]
PHASES={'m100':-1.,'p030':.3,'p090':.9}
METHODS=('N','T')
LABELS={'N':'Native baseline (N)','T':'Baseline + trajectory-facing heading (T)'}
FPS=20
WIDTH,HEIGHT=1600,960
PANEL_WIDTH,RENDER_HEIGHT=800,500
HEADING_SETTINGS=dict(heading_reference='start',heading_actuation='shells',
    heading_gain=.2,heading_damping=.06,heading_torque_limit_Nm=.15,heading_roll_budget_Nm=.05)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def orientation_signals(qpos,qvel,reference_quat,heading_target_rad=None):
    """World tilt/yaw and physical world-z angular rate from recorded states."""
    quat=np.asarray(qpos)[:,3:7]
    quat=quat/np.linalg.norm(quat,axis=1,keepdims=True)
    w,x,y,z=quat.T
    zz=1.-2.*(x*x+y*y)
    tilt=np.rad2deg(np.arccos(np.clip(zz,-1.,1.)))
    yaw=np.rad2deg(np.unwrap(np.arctan2(2.*(x*y+w*z),1.-2.*(y*y+z*z))))
    row_z=np.column_stack((2.*(x*z-w*y),2.*(y*z+w*x),zz))
    omega_world_z=np.rad2deg(np.sum(row_z*np.asarray(qvel)[:,3:6],axis=1))
    # Independent Euler heading derivative from the projected forward axis.
    row_x=np.column_stack((1.-2.*(y*y+z*z),2.*(x*y-w*z),2.*(x*z+w*y)))
    row_y=np.column_stack((2.*(x*y+w*z),1.-2.*(x*x+z*z),2.*(y*z-w*x)))
    angular=np.asarray(qvel)[:,3:6]
    omega_x=np.sum(row_x*angular,axis=1);omega_y=np.sum(row_y*angular,axis=1)
    omega_z=np.deg2rad(omega_world_z)
    hx,hy,hz=row_x[:,0],row_y[:,0],row_z[:,0]
    den=hx*hx+hy*hy
    heading_rate=np.rad2deg(omega_z-np.divide(hz*(omega_x*hx+omega_y*hy),den,
        out=np.full_like(den,np.nan),where=den>1e-8))
    ref=np.asarray(reference_quat)
    ref=ref/np.linalg.norm(ref,axis=1,keepdims=True)
    ref_lean=np.rad2deg(np.arccos(np.clip(1.-2.*np.sum(ref[:,1:3]**2,axis=1),-1.,1.)))
    rw,rx,ry,rz=ref.T
    actual_up=np.column_stack((2.*(x*z+w*y),2.*(y*z-w*x),zz))
    reference_up=np.column_stack((2.*(rx*rz+rw*ry),2.*(ry*rz-rw*rx),1.-2.*(rx*rx+ry*ry)))
    reference_up_error=np.rad2deg(np.arccos(np.clip(np.sum(actual_up*reference_up,axis=1),-1.,1.)))
    target=(np.full(len(yaw),float(yaw[0])) if heading_target_rad is None else
            np.rad2deg(np.broadcast_to(np.asarray(heading_target_rad,float),yaw.shape)))
    heading_error=(yaw-target+180.)%360.-180.
    return dict(tilt_deg=tilt,yaw_deg=yaw,omega_world_z_deg_s=omega_world_z,
                reference_lean_deg=ref_lean,reference_up_error_deg=reference_up_error,
                heading_target_deg=target.copy(),heading_error_deg=heading_error,
                euler_heading_rate_deg_s=heading_rate,heading_rate_valid=den>1e-8)


def nearest(times,moment):
    i=int(np.searchsorted(times,moment))
    if i>=len(times):return len(times)-1
    if i>0 and abs(times[i-1]-moment)<=abs(times[i]-moment):return i-1
    return i


def inputs_and_register(stage,seal):
    stage.mkdir(parents=True,exist_ok=False)
    dependencies=[Path(__file__).resolve(),ROOT/'src/spherical_wheel/offroad_view.py',
                  ROOT/'src/spherical_wheel/gazebo/mars_compare.py',
                  ROOT/'src/spherical_wheel/gazebo/mars.py',
                  ROOT/'src/spherical_wheel/offroad.py',BASE/'runtime/route.py',
                  BASE/'runtime/heading_reference.py',
                  BASE/'protocol/protocol.json',seal]
    yard=ROOT/'assets/gazebo/third_party/leo_simulator-ros2/leo_gz_worlds/models/marsyard2021_terrain'
    dependencies += [yard/'dem/marsyard_terrain_hm.tif',*[yard/'meshes'/f'{n}.obj' for n in ('lava','rocks_big','rocks_medium')]]
    for phase in PHASES:
        for method in METHODS:
            folder=BASE/'campaign/runs'/f'{phase}_{method}'
            dependencies += [folder/name for name in ('precision.npz','rollout.npz','summary.json','artifact_manifest.json','source_snapshot/config/model.yaml')]
    bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in dependencies}
    for p in dependencies:
        if p.suffix=='.py':
            target=stage/'source'/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
    save(stage/'registered_before_render.json',dict(
        registered_utc=datetime.now(timezone.utc).isoformat(),files=bindings,
        recorded_Gazebo_states_only=True,no_physics_integration=True,
        selection=dict(phases=PHASES,methods=METHODS,full_recorded_window_s=[0.,45.],fps=FPS),
        camera_policy='One shared static camera from full actual+reference extent of all6cases. '
                      'No favorable crop, motion omission, follow camera or visual body stabilization.',
        truncated_policy='After last recorded state, visibly freeze pose and show run-ended time/failure.',
        method_scope='Only planned tangent heading/rate feedback is enabled for T. '
                     'Native LQR, momentum observer, contact estimation, feedforward and input caps remain.'))
    return bindings


def verify_root_seal(seal_path):
    seal=json.loads(seal_path.read_text())
    protocol_path=BASE/'protocol/protocol.json'
    if seal['protocol_sha256']!=sha(protocol_path):raise ValueError('Root output seal protocol differs')
    if not seal['all_processes_successful']:raise ValueError('Root output seal reports process failures')
    protocol=json.loads(protocol_path.read_text())
    if protocol.get('iteration')!='I-75' or protocol.get('heading_parameters')!=HEADING_SETTINGS:
        raise ValueError('Display heading settings differ from the executed protocol')
    expected={f'{phase}_{method}' for phase in PHASES for method in METHODS}
    if {job['case_id'] for job in protocol['cases']}!=expected:raise ValueError('Protocol cases differ')
    if {p['case_id'] for p in seal['processes']}!=expected:raise ValueError('Root process set differs')
    if any(p['returncode']!=0 for p in seal['processes']):raise ValueError('Root process returned failure')
    route_key=str((BASE/'runtime/route.py').relative_to(ROOT))
    if protocol['source_sha256'][route_key]!=sha(BASE/'runtime/route.py'):
        raise ValueError('Display route differs from the executed frozen route')
    reference_key=str((BASE/'runtime/heading_reference.py').relative_to(ROOT))
    if protocol['source_sha256'][reference_key]!=sha(BASE/'runtime/heading_reference.py'):
        raise ValueError('Display planned heading generator differs from executed source')
    files=seal['files']
    for name,binding in files.items():
        path=(BASE/name).resolve()
        if not path.is_relative_to(BASE):raise ValueError('Root seal path escapes experiment')
        if sha(path)!=binding['sha256'] or path.stat().st_size!=binding['bytes']:
            raise ValueError('Root sealed artifact changed: '+name)
    for case in expected:
        for name in ('precision.npz','rollout.npz','summary.json','artifact_manifest.json'):
            if f'campaign/runs/{case}/{name}' not in files:raise ValueError('Root seal missing required run artifact')
    return dict(passed=True,protocol_sha256=sha(protocol_path),root_seal_sha256=sha(seal_path),
                verified_root_file_count=len(files),cases=sorted(expected),route_source_matches_executed=True)


def verify_manifests():
    for phase in PHASES:
        for method in METHODS:
            folder=BASE/'campaign/runs'/f'{phase}_{method}'
            for record in json.loads((folder/'artifact_manifest.json').read_text())['files']:
                path=folder/record['path']
                if sha(path)!=record['sha256']:raise ValueError('Run artifact changed: '+str(path))


def load_all():
    protocol=json.loads((BASE/'protocol/protocol.json').read_text())
    spec=importlib.util.spec_from_file_location('_i75_media_heading_reference',BASE/'runtime/heading_reference.py')
    reference_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(reference_module)
    reference=reference_module.PlannedTangentReference(**protocol['heading_reference_parameters'])
    runs={}
    for phase in PHASES:
        for method in METHODS:
            case=f'{phase}_{method}';folder=BASE/'campaign/runs'/case
            with np.load(folder/'rollout.npz',allow_pickle=False) as z:
                log={k:z[k].copy() for k in ('time_s','qpos','qvel','reference_xy','ground_height_m')}
            with np.load(folder/'precision.npz',allow_pickle=False) as z:
                exact={k:z[k].copy() for k in ('time_s','exact_qpos','exact_qvel','exact_reference_quat','exact_reference_xy',
                    'exact_heading_reference_rad','heading__raw_error_rad','exact_heading_command',
                    'heading__raw_heading_rad','heading__raw_heading_rate_rad_s',
                    'heading__command_heading_rad','heading__command_heading_rate_rad_s',
                    'heading__actual_helper_target_rad','heading__actual_helper_rate_reference_rad_s',
                    'heading__euler_heading_rate_rad_s','heading__rate_valid')}
            format_checks={}
            for log_key,exact_key in (('time_s','time_s'),('qpos','exact_qpos'),
                                      ('qvel','exact_qvel'),('reference_xy','exact_reference_xy')):
                saved,precise=log[log_key],exact[exact_key]
                if saved.shape!=precise.shape or not np.array_equal(saved,precise.astype(saved.dtype)):
                    raise ValueError('Rollout does not equal original-dtype encoding of precision: '+case+'/'+log_key)
                format_checks[log_key]=dict(rollout_dtype=saved.dtype.str,precision_dtype=precise.dtype.str,
                                            exact_after_original_dtype_cast=True,
                                            maximum_original_encoding_difference=float(np.max(np.abs(saved-precise))))
                # Native save_run stores a float32 display rollout. Render the
                # corresponding full-precision recorded state, without changing it.
                log[log_key]=precise.copy()
            stride=max(1,len(log['time_s'])//400)
            planned=log['reference_xy'][::stride]
            if not np.array_equal(planned[-1],log['reference_xy'][-1]):planned=np.vstack([planned,log['reference_xy'][-1]])
            log['planned_xy']=planned
            latch=exact['exact_heading_reference_rad']
            if latch.shape!=(len(log['time_s']),) or not np.array_equal(latch,np.full_like(latch,latch[0])):
                raise ValueError('Native initial heading latch changed')
            target=exact['heading__raw_heading_rad']
            if target.shape!=(len(log['time_s']),):raise ValueError('Raw path-heading target has wrong shape')
            planned=[reference.sample(float(t)) for t in log['time_s']]
            for field in ('raw_heading_rad','raw_heading_rate_rad_s','command_heading_rad','command_heading_rate_rad_s'):
                expected=np.array([row[field] for row in planned])
                if not np.allclose(exact['heading__'+field],expected,atol=1e-12,rtol=1e-12):
                    raise ValueError('Recorded planned heading differs from the frozen generator: '+field)
            if not np.array_equal(target,exact['heading__command_heading_rad']):
                raise ValueError('Registered raw and command target streams should be identical')
            signals=orientation_signals(exact['exact_qpos'],exact['exact_qvel'],exact['exact_reference_quat'],target)
            computed=np.deg2rad(signals['heading_error_deg'])
            angle_difference=(computed-exact['heading__raw_error_rad']+np.pi)%(2*np.pi)-np.pi
            if not np.all(abs(angle_difference)<=1e-10):
                raise ValueError('Recorded heading error differs from full-precision recorded pose/target')
            if abs(computed[0])>np.deg2rad(1.)+1e-10:raise ValueError('Actual initial heading misses planned tangent by more than registered1degree')
            valid=signals['heading_rate_valid']&exact['heading__rate_valid'].astype(bool)
            if not np.allclose(np.deg2rad(signals['euler_heading_rate_deg_s'][valid]),
                               exact['heading__euler_heading_rate_rad_s'][valid],atol=1e-10,rtol=1e-10):
                raise ValueError('Media Euler heading-rate computation differs from logged derivative')
            signals['command_heading_target_deg']=np.rad2deg(exact['heading__command_heading_rad'])
            signals['path_heading_rate_deg_s']=np.rad2deg(exact['heading__raw_heading_rate_rad_s'])
            signals['helper_heading_target_deg']=np.rad2deg(exact['heading__actual_helper_target_rad'])
            signals['helper_heading_rate_deg_s']=np.rad2deg(exact['heading__actual_helper_rate_reference_rad_s'])
            signals['heading_torque_norm_Nm']=np.linalg.norm(exact['exact_heading_command'][:,:3],axis=1)
            runs[case]=dict(case=case,method=method,phase=phase,log=log,signals=signals,
                            summary=json.loads((folder/'summary.json').read_text()),folder=folder,
                            rollout_precision_roundtrip=format_checks)
    return runs


def font(size):
    from PIL import ImageFont
    for path in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf','/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf'):
        if Path(path).is_file():return ImageFont.truetype(path,size)
    return ImageFont.load_default()


def composite(runs,phase,moment,plant,renderer,camera,option,viewer,path,heading_reference):
    from PIL import Image,ImageDraw
    image=Image.new('RGB',(WIDTH,HEIGHT),(13,21,31));draw=ImageDraw.Draw(image)
    stage='START HOLD' if moment<3. else 'MOTION' if moment<35. else 'GOAL HOLD'
    draw.text((22,12),f'I-75  |  start phase {PHASES[phase]:+.1f} rad  |  t = {moment:05.2f} s  |  {stage}',font=font(25),fill=(245,246,248))
    draw.text((22,47),'Recorded Gazebo / DART poses; same moderate faceted Mars ground, reference path, gains, observer and limits.',font=font(19),fill=(186,199,211))
    display=[]
    for col,method in enumerate(METHODS):
        run=runs[f'{phase}_{method}'];log=run['log'];sig=run['signals']
        index=nearest(log['time_s'],moment);recorded=float(log['time_s'][index])
        ended=moment>float(log['time_s'][-1])+1e-8
        reference_xy=path(moment)[0]
        display_log={**log,'reference_xy':np.broadcast_to(reference_xy,log['reference_xy'].shape)}
        pixels=viewer._draw(plant,renderer,display_log,index,camera,option,True)
        left=col*PANEL_WIDTH;image.paste(Image.fromarray(pixels),(left,80));draw=ImageDraw.Draw(image)
        draw.rectangle((left,80,left+PANEL_WIDTH-1,115),fill=(15,24,34))
        draw.text((left+14,84),LABELS[method],font=font(24),fill=(242,245,247))
        if ended:
            draw.rectangle((left+8,180,left+PANEL_WIDTH-8,250),fill=(114,24,28))
            draw.text((left+18,190),f'RUN ENDED at {log["time_s"][-1]:.3f} s; pose frozen',font=font(20),fill='white')
            draw.text((left+18,219),str(run['summary'].get('failure','incomplete run'))[:61],font=font(16),fill='white')
        error=1000.*np.linalg.norm(log['qpos'][index,:2]-reference_xy)
        planned=heading_reference.sample(moment)
        raw_target=(float(np.rad2deg(planned['raw_heading_rad'])) if ended else sig['heading_target_deg'][index])
        command_target=(float(np.rad2deg(planned['command_heading_rad'])) if ended else sig['command_heading_target_deg'][index])
        path_rate=(float(np.rad2deg(planned['raw_heading_rate_rad_s'])) if ended else sig['path_heading_rate_deg_s'][index])
        raw_error=(sig['yaw_deg'][index]-raw_target+180.)%360.-180.
        helper_label=('OFF (native)' if method=='N' else f'{sig["helper_heading_target_deg"][index]:+7.1f} deg')
        if ended and method!='N':helper_label+=' (last recorded)'
        lines=[f'Body tilt / ref lean / up error: {sig["tilt_deg"][index]:.1f} / {sig["reference_lean_deg"][index]:.1f} / {sig["reference_up_error_deg"][index]:.1f} deg',
               f'Path heading raw / command: {raw_target:+6.1f} / {command_target:+6.1f} deg',
               f'Applied helper target: {helper_label}',
               f'Raw path-heading error: {raw_error:+7.1f} deg',
               f'World yaw (unwrapped): {sig["yaw_deg"][index]:+7.1f} deg',
               f'Euler heading rate / path rate: {sig["euler_heading_rate_deg_s"][index]:+6.1f} / {path_rate:+6.1f} deg/s',
               f'World-z rotation rate: {sig["omega_world_z_deg_s"][index]:+7.1f} deg/s',
               f'Shaft: {log["qpos"][index,7]:+7.2f} rad | Heading motor norm: {sig["heading_torque_norm_Nm"][index]:.3f} N m',
               f'Position error: {error:6.1f} mm  |  pose t={recorded:.3f}s']
        for j,line in enumerate(lines):draw.text((left+14,590+j*28),line,font=font(21),fill=(230,235,241))
        if col:draw.line((left,80,left,849),fill=(81,94,105),width=2)
        display.append(dict(case=run['case'],requested_time_s=moment,pose_index=index,
                            recorded_pose_time_s=recorded,run_ended_pose_frozen=ended))
    draw.text((22,860),'Orange: shared reference and current goal   |   Cyan: actual trace   |   Camera covers all six paths.',font=font(20),fill=(236,190,114))
    draw.text((22,894),'T: planned tangent + Euler turn-rate target; gain 0.2, damping 0.06, torque cap 0.15 N m, roll budget 0.05 N m.',font=font(19),fill=(186,199,211))
    draw.text((22,930),'N leaves heading free. Both start aligned to the path; raw path heading is the shared evaluation reference.',font=font(17),fill=(166,180,193))
    return np.asarray(image),display


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--phase',choices=PHASES,required=True)
    parser.add_argument('--stage',type=Path,required=True);parser.add_argument('--input-seal',type=Path,required=True)
    parser.add_argument('--preview',action='store_true');args=parser.parse_args()
    stage=args.stage.resolve();bindings=inputs_and_register(stage,args.input_seal.resolve())
    root_verification=verify_root_seal(args.input_seal.resolve())
    save(stage/'root_seal_verification.json',root_verification)
    verify_manifests();runs=load_all()
    from spherical_wheel.gazebo.mars import MarsYard
    from spherical_wheel.gazebo.mars_compare import replay_plant
    from spherical_wheel import offroad_view as viewer
    import mujoco,yaml,imageio.v2 as imageio
    route_source=BASE/'runtime/route.py'
    spec=importlib.util.spec_from_file_location('_i75_frozen_video_route',route_source)
    route_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(route_module)
    path=route_module.SmoothSPath();full_planned=route_module.samples()['xy_m']
    heading_spec=importlib.util.spec_from_file_location('_i75_video_heading_reference',BASE/'runtime/heading_reference.py')
    heading_module=importlib.util.module_from_spec(heading_spec);heading_spec.loader.exec_module(heading_module)
    heading_reference=heading_module.PlannedTangentReference(**json.loads((BASE/'protocol/protocol.json').read_text())['heading_reference_parameters'])
    for run in runs.values():run['log']['planned_xy']=full_planned[::4]
    allxy=np.concatenate([full_planned,*[r['log']['qpos'][:,:2] for r in runs.values()],
                          *[r['log']['reference_xy'] for r in runs.values()]])
    low,high=allxy.min(axis=0),allxy.max(axis=0);middle=.5*(low+high)
    bounds=(float(low[0]-.7),float(high[0]+.7),float(low[1]-.7),float(high[1]+.7))
    yard=MarsYard();span=float(np.max(high-low)+.5)
    camera_info=dict(lookat=[float(middle[0]),float(middle[1]),float(yard.height(*middle)+.08)],
                     azimuth_deg=112.,elevation_deg=-34.,distance_m=max(2.2,1.6*span),
                     actual_and_reference_xy_bounds_m=[float(low[0]),float(high[0]),float(low[1]),float(high[1])],
                     render_terrain_bounds_m=list(bounds),all_six_paths_included=True)
    camera=viewer._camera(camera_info['lookat'],camera_info['azimuth_deg'],camera_info['elevation_deg'],camera_info['distance_m'])
    # _camera adds6cm to its supplied lookat; record the actual final lookat too.
    camera_info['actual_camera_lookat_m']=camera.lookat.tolist()
    first=runs[f'{args.phase}_N']['folder']/'source_snapshot/config/model.yaml'
    config_hashes={sha(r['folder']/'source_snapshot/config/model.yaml') for r in runs.values()}
    if len(config_hashes)!=1:raise ValueError('Comparison scenes require identical model configurations')
    plant=replay_plant(yaml.safe_load(first.read_text()),yard,bounds,stage/'scene')
    option=mujoco.MjvOption();option.geomgroup[3:]=0
    moments=np.array([0.,3.,15.,25.,35.,45.]) if args.preview else np.arange(45*FPS+1)/FPS
    video=stage/f'{args.phase}_N_T.mp4';writer=None;schedule=[]
    try:
        with mujoco.Renderer(plant.model,height=RENDER_HEIGHT,width=PANEL_WIDTH) as renderer:
            if not args.preview:
                writer=imageio.get_writer(video,fps=FPS,codec='libx264',macro_block_size=2,
                                          quality=None,ffmpeg_log_level='error',
                                          ffmpeg_params=['-crf','21','-preset','fast'])
            for frame_number,moment in enumerate(moments):
                pixels,rows=composite(runs,args.phase,float(moment),plant,renderer,camera,option,viewer,path,heading_reference)
                if writer is not None:writer.append_data(pixels)
                if args.preview or frame_number in (0,3*FPS,15*FPS,35*FPS,45*FPS):
                    imageio.imwrite(stage/f'pose_{moment:06.2f}s.jpg',pixels,quality=95)
                schedule.append(dict(frame=frame_number,cases=rows))
    finally:
        if writer is not None:writer.close()
    metadata=dict(iteration='I-75',phase=args.phase,requested_start_phase_rad=PHASES[args.phase],
                  phase_definition='Requested ground-relative initial configuration; setup adjusts the actual shaft joint for terrain normal.',
                  methods=list(METHODS),frames=len(moments),fps=FPS,dimensions=[WIDTH,HEIGHT],
                  heading_parameters=HEADING_SETTINGS,
                  heading_reference_parameters=json.loads((BASE/'protocol/protocol.json').read_text())['heading_reference_parameters'],
                  full_recorded_window_s=[0.,45.],playback_speed=1.,shared_camera=camera_info,
                  frozen_complete_reference=path.spec(),root_output_seal_verification=root_verification,
                  rendered_state_source='Unmodified full-precision recorded exact_qpos/qvel/time from precision.npz.',
                  rollout_precision_format_checks={case:run['rollout_precision_roundtrip'] for case,run in runs.items()},
                  path_heading_initial_deg={case:float(run['signals']['heading_target_deg'][0]) for case,run in runs.items()},
                  reference_marker='Always frozen full raw path(t), including after a truncated actual run.',
                  no_physics_or_controller_execution=True,renderer='MuJoCo: saved Gazebo poses only',
                  display_terrain='Visual height field sampled from the original Mars DEM; '
                                  'original rock/lava meshes. Display interpolation is not '
                                  'a new collision or contact evaluation.',
                  absolute_tilt='acos(world vertical dot body up), from recorded quaternion',
                  reference_lean='acos(world vertical dot reference up), from recorded raw reference quaternion',
                  raw_reference_up_error='acos(recorded body up dot recorded raw reference up)',
                  heading_target='Recorded raw planned geometric tangent; bounded command stream is separate and verified equal for this C2 path. Actual helper target is N OFF or T planned stream.',
                  heading_error='Shortest wrapped angular difference between actual Euler yaw and raw desired path tangent; unwrapped yaw is also displayed.',
                  euler_heading_rate='Derivative of the projected forward-axis angle from recorded pose/body angular velocity; compared with logged exact Euler derivative.',
                  heading_motor_norm='Euclidean norm of helper motor-command torques; not a world-frame yaw torque measurement.',
                  physical_world_yaw_rate='world-z component of recorded angular velocity, not Euler yaw derivative',
                  both_methods_and_all_phases_preserved=True)
    if not args.preview:
        with imageio.get_reader(video) as reader:
            count=reader.count_frames();info=reader.get_meta_data()
            if count!=len(moments):raise ValueError('Encoded frame count differs')
            for i in (0,len(moments)//2,len(moments)-1):
                imageio.imwrite(stage/f'decoded_{i:04d}.jpg',reader.get_data(i),quality=95)
        metadata['video']=dict(path=str(video.relative_to(ROOT)),sha256=sha(video),bytes=video.stat().st_size,
                               decoded_frames=count,duration_s=info['duration'])
    if not all(sha(ROOT/p)==b['sha256'] for p,b in bindings.items()):raise ValueError('Render source/input changed')
    save(stage/'display_schedule.json',schedule);save(stage/'metadata.json',metadata)
    artifacts={str(p.relative_to(stage)):dict(sha256=sha(p),bytes=p.stat().st_size)
               for p in sorted(stage.rglob('*')) if p.is_file() and 'scene' not in p.relative_to(stage).parts}
    save(stage/'artifact_manifest.json',dict(files=artifacts,inputs_unchanged=True,no_physics=True))
    print(json.dumps(dict(phase=args.phase,preview=args.preview,frames=len(moments),output=str(video) if not args.preview else str(stage)),indent=2),flush=True)


if __name__=='__main__':main()
