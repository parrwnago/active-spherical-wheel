"""I53 fixed m100 video cue, saved actual native states and diagnostics only."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import shutil
import types

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
I52=ROOT/'results/contact_scheduling_joint_tracking_20261126'
VIEWER=ROOT/'results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py'
PAINT=ROOT/'results/allocation_handoff_isolation_20261126/videos/render_isolation.py'
EVENT=HERE.parent/'event_audit/compact_stage02/summary.json'
VARIANTS=('B','R','N','NR')
COLORS={'B':(230,174,87),'R':(92,200,220),'N':(176,135,218),'NR':(113,211,168)}
TITLES={'B':'B: original baseline','R':'R: joint-reference guard','N':'N: normal-relative lookup','NR':'NR: both factors'}
LABELS=('shell A','shell B','belt A','belt B')
START,END,FPS=28.,36.,20
WIDTH,HEIGHT=1800,1800

def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def vec(a,digits=2):return '['+', '.join(f'{x:+.{digits}f}'for x in a)+']'
def dominant(a):return LABELS[int(np.argmax(a))]if np.sum(a)>.05 else'none'

def load(case,folder):
    for row in read(folder/'artifact_manifest.json')['files']:assert sha(folder/row['path'])==row['sha256']
    with np.load(folder/'precision.npz')as z:e={k:z[k].copy()for k in z.files}
    with np.load(folder/'rollout.npz')as z:log={k:z[k].copy()for k in z.files}
    log.update(time_s=e['time_s'],qpos=e['exact_qpos'],qvel=e['exact_qvel'],input=e['exact_applied'],reference_xy=e['exact_reference_xy']);log['planned_xy']=log['reference_xy'][::40]
    q=e['exact_qpos'][:,3:7];log['absolute_tilt_deg']=np.degrees(np.arccos(np.clip(1-2*np.sum(q[:,1:3]**2,axis=1)/np.sum(q*q,axis=1),-1,1)))
    log['speed_m_s']=np.linalg.norm(e['exact_qvel'][:,:2],axis=1);log['body_up_deg_s']=np.degrees(np.linalg.norm(e['exact_qvel'][:,3:5],axis=1))
    log['timed_error_mm']=1000*np.linalg.norm(e['exact_qpos'][:,:2]-e['exact_reference_xy'],axis=1)
    a=np.full(len(e['time_s']),np.nan);a[10:]=np.linalg.norm((e['exact_qvel'][10:,:2]-e['exact_qvel'][:-10,:2])/(e['time_s'][10:]-e['time_s'][:-10])[:,None],axis=1);log['accel50ms_m_s2']=a
    return dict(case=case,folder=folder,exact=e,log=log)

def chart(paint,runs,key,title,high):
    p=Image.new('RGB',(900,200),paint.BG);d=ImageDraw.Draw(p);left,top,w,h=52,35,820,120
    paint.text(d,(16,0),title,20);d.rectangle((left,top,left+w,top+h),fill=(4,12,20),outline=(60,81,98))
    for value in (0,high):paint.text(d,(3,top+(h-14 if value==0 else 0)),f'{value:g}',12)
    for variant,run in zip(VARIANTS,runs):
        log=run['log'];m=(log['time_s']>=START-1e-8)&(log['time_s']<=END+1e-8);pts=[(left+(t-START)/(END-START)*w,top+h-v/high*h)for t,v in zip(log['time_s'][m],log[key][m])]
        d.line(pts,fill=COLORS[variant],width=2)
    for t in (28,30,32,34,36):paint.text(d,(left+(t-START)/(END-START)*w-8,top+h+8),str(t),13)
    return p

def card(paint,event,end=False):
    p=Image.new('RGB',(WIDTH,HEIGHT),paint.BG);d=ImageDraw.Draw(p)
    paint.text(d,(60,55),'I53: investigate the visible burst without changing the controller',38,paint.WARN)
    y=190
    if not end:
        lines=[
            'Original I52 overview playback 10.000 s = recorded 32.000 s in the first, −1.00 rad phase chapter.',
            'The fixed user-cue window, overview playback 9–11 s, maps to recorded 28–36 s. It is replayed here at 0.5×.',
            'All four original native histories are shown: B baseline, R joint-reference guard, N normal-relative lookup, NR both.',
            'Same original gain bank and .30 m/s belt cap. Per-state scheduled gains can differ; no new gains or native runs are produced.',
            'Actual joint rates, native/used rate targets, requested/applied torques, belt speeds and current support are labeled separately.',
            'Desired-reference slew bounds are not actual joint/body acceleration limits. The numerical limits are simulation assumptions.',
            'Pose frames sample 25 ms; trace strips retain every recorded 5 ms sample. Tiny 5 ms contact toggles can occur between frames.',
            'The old overview poses were 200 ms apart. Its fractional clock is a continuous 4× mapping; this replay uses actual saved states, not video interpolation.',
            'Commands computed at the current timestamp act over the NEXT 5 ms interval; they cannot cause motion already measured at that instant.',
            'Truth support is saved simulated endpoint evaluation only. No true contacts are supplied to a controller or estimator.',
        ]
        for line in lines:y=paint.wrap(d,(70,y),line,1660,29)+45
    else:
        paint.text(d,(65,175),'Fixed recorded 28–36 s context, from the separately sealed event audit:',28)
        for x,label in [(70,'Policy'),(480,'Max actual speed'),(1010,'50 ms acceleration RMS')]:paint.text(d,(x,290),label,27,(157,196,219))
        for i,v in enumerate(VARIANTS):
            c=event['cases']['m100_'+v]['windows']['playback9_11'];yy=380+i*105
            paint.text(d,(70,yy),v,30,COLORS[v]);paint.text(d,(480,yy),f'{c["actual_planar_speed_max_m_s"]:.3f} m/s',29);paint.text(d,(1010,yy),f'{c["planar_acceleration50ms_rms_m_s2"]:.3f} m/s²',29)
        y=920
        for line in [
            'R speed peaks at 1.025 m/s at recorded 32.305 s. Its used-reference slew bounds still hold: reference bounds do not cap actual acceleration.',
            'Near 32.250 s, R changes belt-B to shell-B support/gain routing with a large existing balance error and a saturated shell torque request.',
            'That current command acts during 32.250–32.255 s. It cannot explain the velocity increment already ending at 32.250 s.',
            'N is less fast and has lower 50 ms acceleration RMS in this window, but contact/command pulses remain at 5 ms scale.',
            'These are distinct closed-loop histories. The clip does not establish a unique physical cause or globally superior gains; all I52 full-policy screens failed.',
        ]:y=paint.wrap(d,(70,y),line,1660,29,paint.WARN if y==920 else paint.FG)+38
    paint.text(d,(70,1720),'ACTUAL SAVED NATIVE MOTION | no new physics, controller, observer or forecast execution.',27,paint.WARN)
    return np.asarray(p)

def frame(paint,viewer,runs,plants,renderers,charts,moment,camera,bounds,planned):
    p=Image.new('RGB',(WIDTH,HEIGHT),paint.BG);d=ImageDraw.Draw(p);original=2+moment/4;clip=2+(moment-START)/.5
    paint.text(d,(22,10),f'I53 SLOW REPLAY | recorded {moment:.3f} s | original 4× clock ≈ {original:.3f} s | clip {clip:.3f} s | 0.5×',28)
    for k,(v,run)in enumerate(zip(VARIANTS,runs)):
        left,top=(k%2)*900,60+(k//2)*700;e=run['exact'];log=run['log'];i=viewer.nearest(e['time_s'],moment)
        paint.text(d,(left+16,top),TITLES[v]+' | initial phase −1.00 rad',23,COLORS[v])
        option=mujoco.MjvOption();option.geomgroup[3:]=0;pixels=viewer._draw(plants[k],renderers[k],log,i,camera,option,False);pixels=paint.inset(pixels,run,i,bounds,planned);p.paste(Image.fromarray(pixels),(left,top+45));d=ImageDraw.Draw(p)
        lines=[
            f'Recorded {e["time_s"][i]:.3f} s | timed error {log["timed_error_mm"][i]:.1f} mm | speed {log["speed_m_s"][i]:.3f} m/s',
            f'Absolute/ref. lean {log["absolute_tilt_deg"][i]:.2f}/{np.degrees(log["attitude_error_rad"][i]):.2f}° | body-up speed {log["body_up_deg_s"][i]:.1f}°/s',
            f'Body ωxyz {vec(e["exact_qvel"][i,3:6])} rad/s | |aXY,50ms| {log["accel50ms_m_s2"][i]:.2f} m/s²',
            f'ACTUAL joints [shaft,A,B] {vec(e["exact_qvel"][i,6:9])} rad/s',
            f'NATIVE targets {vec(e["exact_native_joint_reference"][i])} | USED {vec(e["exact_guarded_joint_reference"][i])} rad/s',
            f'APPLIED τ {vec(e["exact_applied"][i,:3])} Nm | belts A/B {vec(e["exact_applied"][i,3:],3)} m/s',
            f'REQUEST τ {vec(e["exact_native_requested"][i,:3])} Nm | belts {vec(e["exact_native_requested"][i,3:],3)} m/s',
            f'Observer: {dominant(e["exact_estimated_support_loads_N"][i])} | truth endpoint: {dominant(e["truth_patch_loads_N"][i])}',
        ]
        for j,line in enumerate(lines):paint.text(d,(left+16,top+410+j*31),line,18)
    for col,c in enumerate(charts):
        q=c.copy();d=ImageDraw.Draw(q);x=52+(moment-START)/(END-START)*820;d.line((x,35,x,155),fill=(249,248,239),width=2);p.paste(q,(col*900,1470))
    d=ImageDraw.Draw(p)
    for x,v in zip((24,390,800,1220),VARIANTS):paint.text(d,(x,1685),TITLES[v],21,COLORS[v])
    paint.text(d,(24,1720),'Curves retain all 5 ms samples; poses sample 25 ms. Body-up speed = ||ωbody,xy||, not yaw speed.',20)
    paint.text(d,(24,1752),'Truth is evaluation-only. Current requested/applied commands act during the NEXT 5 ms interval.',20)
    paint.text(d,(24,1780),'Reference slew constraints do not bound actual acceleration. No new control or gain-improvement claim is made by this replay.',16,paint.WARN)
    return np.asarray(p)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');args=parser.parse_args();selection=read(HERE/'selection.json');event=read(EVENT)
    spec=importlib.util.spec_from_file_location('i53_paint',PAINT);paint=importlib.util.module_from_spec(spec);spec.loader.exec_module(paint)
    viewer=types.ModuleType('_i53_saved_viewer');viewer.__file__=str(VIEWER);exec(compile(VIEWER.read_text(),str(VIEWER),'exec'),viewer.__dict__)
    resolution=read(I52/'campaign/resolved_case_paths.json');runs=[load('m100_'+v,ROOT/resolution['cases']['m100_'+v])for v in VARIANTS]
    old=read(I52/'videos/video_metadata.json')['shared_camera'];camera=viewer._camera(np.asarray(old['lookat']),old['azimuth_deg'],old['elevation_deg'],old['distance_m']);bounds=old['world_xy_bounds'];mid=old['lookat'][:2];terrainbounds=(mid[0]-1.2,mid[0]+1.2,mid[1]-1.2,mid[1]+1.2);planned=runs[0]['log']['planned_xy']
    speed_high=np.ceil(max(r['log']['speed_m_s'][(r['log']['time_s']>=START-1e-8)&(r['log']['time_s']<=END+1e-8)].max()for r in runs)*10)/10
    body_high=max(25.,np.ceil(max(r['log']['body_up_deg_s'][(r['log']['time_s']>=START-1e-8)&(r['log']['time_s']<=END+1e-8)].max()for r in runs)/25)*25)
    charts=[chart(paint,runs,'speed_m_s','Actual planar speed (m/s), full 5 ms trace',speed_high),chart(paint,runs,'body_up_deg_s','Actual body-up rotation speed (deg/s), 5 ms',body_high)]
    source=Path(__file__).resolve();inputs=[source,PAINT,VIEWER,HERE/'selection.json',EVENT,I52/'campaign/resolved_case_paths.json',I52/'videos/video_metadata.json',I52/'videos/video_display_schedule.json']
    for run in runs:inputs += [run['folder']/name for name in ('artifact_manifest.json','precision.npz','rollout.npz','source_snapshot/config/model.yaml')]
    hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs};snap=HERE/'source_snapshot';snap.mkdir(exist_ok=True)
    for path in (source,PAINT,VIEWER):
        dest=snap/f'{path.stem}_{sha(path)[:12]}.py'
        if not dest.exists():shutil.copy2(path,dest)
    prefix='preview'if args.preview else'video';output=HERE/'m100_slow_replay.mp4';metadata=HERE/f'{prefix}_metadata.json'
    if metadata.exists()or(not args.preview and output.exists()):raise FileExistsError('Preserve earlier media attempt')
    plants=[];renderers=[];writer=None;count=0;schedule=[]
    try:
        for run in runs:
            config=viewer.yaml.safe_load((run['folder']/'source_snapshot/config/model.yaml').read_text());plant=viewer.replay_plant(config,viewer.MarsYard(),terrainbounds,HERE/'_render_scenes'/run['case']);plants.append(plant);renderers.append(mujoco.Renderer(plant.model,height=350,width=900))
        first,last=card(paint,event),card(paint,event,True);imageio.imwrite(HERE/f'{prefix}_intro.jpg',first,quality=95);imageio.imwrite(HERE/f'{prefix}_result.jpg',last,quality=95)
        if not args.preview:
            writer=imageio.get_writer(output,fps=FPS,codec='libx264',quality=8,macro_block_size=2,ffmpeg_log_level='error')
            for _ in range(40):writer.append_data(first);count+=1
        moments=(28.,32.,32.25,32.30,36.)if args.preview else START+np.arange(321)*.025
        for moment in moments:
            pixels=frame(paint,viewer,runs,plants,renderers,charts,float(moment),camera,bounds,planned)
            if writer is not None:writer.append_data(pixels)
            if args.preview:imageio.imwrite(HERE/f'{prefix}_recorded_{moment:.3f}s.jpg',pixels,quality=95)
            schedule.append(dict(frame=count,recorded_time_s=float(moment),original_overview_time_s=2+float(moment)/4,slow_clip_time_s=2+(float(moment)-START)/.5,case_state_times_s={r['case']:float(r['exact']['time_s'][viewer.nearest(r['exact']['time_s'],moment)])for r in runs}));count+=1
        if writer is not None:
            for _ in range(60):writer.append_data(last);count+=1
    finally:
        if writer is not None:writer.close()
        for r in renderers:r.close()
    assert hashes=={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    meta=dict(iteration='I-53',scope=__doc__,input_sha256=hashes,frames=count,fps=FPS,dimensions=[WIDTH,HEIGHT],recorded_window_s=[START,END],playback_speed=.5,pose_spacing_s=.025,full_signal_spacing_s=.005,cases=[r['case']for r in runs],shared_camera=old,
        original_fractional_clock_scope='Continuous4xprogramclockmapping; originalencodedposeswere200msapart.',truth_evaluation_only=True,no_new_physics=True,no_controller_observer_forecast=True,no_scientific_gate_reduction=True,no_unique_physical_cause_or_gain_improvement_claim=True,default_changed=False)
    if not args.preview:
        with imageio.get_reader(output)as movie:
            assert movie.count_frames()==count==421
            for i in (0,40,210,420):imageio.imwrite(HERE/f'decoded_{i:04d}.jpg',movie.get_data(i),quality=95)
            info=movie.get_meta_data()
        meta['video']=dict(path=str(output.relative_to(ROOT)),sha256=sha(output),bytes=output.stat().st_size,frames=count,fps=FPS,duration_s=info['duration'])
    metadata.write_text(json.dumps(meta,indent=2)+'\n');(HERE/f'{prefix}_display_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n');print(json.dumps({k:v for k,v in meta.items()if k!='input_sha256'},indent=2))

if __name__=='__main__':main()
