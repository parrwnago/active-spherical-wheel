"""I50: close saved-state replay of the I49 late shell/belt handoff; no dynamics."""
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
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SOURCE=ROOT/'results/belt_capacity_sweep_20261126'
VIEWER=ROOT/'results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py'
CASES=('mars_cap030','mars_cap060','mars_cap090')
CAPS=(.3,.6,.9)
LABELS=('shell A','shell B','belt A','belt B')
COLORS=((92,175,224),(111,207,171),(224,164,91))
BG,FG,ORANGE,CYAN,WARN=(10,20,31),(222,234,245),(238,180,92),(104,210,230),(246,193,108)
WIDTH,HEIGHT,FPS=1800,1200,25
START,END=30.5,33.5

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def font(size):return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',size)
def text(draw,xy,value,size=20,color=FG):draw.text(xy,str(value),font=font(size),fill=color)
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def chart(t,series,title,ylim,lines=()):
    pic=Image.new('RGB',(600,155),BG);d=ImageDraw.Draw(pic);left,top,w,h=45,30,530,95;lo,hi=ylim
    d.rectangle((left,top,left+w,top+h),fill=(4,12,20),outline=(60,81,98));text(d,(14,1),title,17)
    for val in (lo,hi):text(d,(2,top+(h-13 if val==lo else 0)),f'{val:g}',11)
    for y,color in lines:
        pos=top+h-(y-lo)/(hi-lo)*h
        if top<=pos<=top+h:
            for x in range(left,left+w,10):d.line((x,pos,min(x+5,left+w),pos),fill=color)
    for values,color in series:
        pts=[(left+(x-START)/(END-START)*w,top+h-(y-lo)/(hi-lo)*h)for x,y in zip(t,values)]
        if len(pts)>1:d.line(pts,fill=color,width=2)
    for val in np.arange(START,END+.01,.5):text(d,(left+(val-START)/(END-START)*w-12,top+h+6),f'{val:g}',11)
    return pic

def cursor(base,moment):
    p=base.copy();d=ImageDraw.Draw(p);x=45+(moment-START)/(END-START)*530;d.line((x,30,x,125),fill=(249,248,239),width=2);return p

def card(end=False):
    p=Image.new('RGB',(WIDTH,HEIGHT),BG);d=ImageDraw.Draw(p)
    text(d,(65,55),'I50: inspect the visible shell-to-belt handoff',42,WARN)
    lines=[
        'Original video playback18s = recorded32s (2s intro, then2× playback).',
        'This replay covers recorded30.50–33.50s at quarter speed, using the original saved states.',
        'All three operating caps are retained; .60/.90 motion is identical in these saved runs.',
        'Current true contact is evaluation-only. Estimated contact and preparation weights are shown separately.',
        'The controller was not modified, rerun or given simulator truth. No new physics occurs in this replay.',
        'Absolute tilt and body-up rotation speed expose platform motion separately from rotation about its up-axis.',
        'Belt-B command curves show requests and actual clipped inputs, not measured motor torque or current.',
    ]if not end else[
        'The late window includes shell-B / belt-B load transfer, contact-label chatter and visible platform motion.',
        'At recorded32.000s, .30 is belt-B supported; .60/.90 remain shell-B supported and prepare belt B.',
        'Platform motion is already present before the principal belt-supported phase in both trajectories.',
        'Truth and observer traces identify timing; they do not establish a unique cause from this one coupled run.',
        'Preparation weight is a blend coefficient. It does not prove the incoming surface speed matched the ground.',
        'The original controller, states, input histories and all I49 artifacts remain unchanged.',
    ]
    for i,line in enumerate(lines):text(d,(70,180+i*117),line,27,WARN if i in (0,3)else FG)
    text(d,(70,1105),'Actual saved native Gazebo/DART motion re-rendered; diagnostic replay only.',27,WARN)
    return np.asarray(p)

def render_frame(viewer,runs,plants,renderers,charts,moment,camera):
    p=Image.new('RGB',(WIDTH,HEIGHT),BG);d=ImageDraw.Draw(p)
    text(d,(24,10),f'I50 HANDOFF AUDIT | recorded t={moment:.3f}s | original playback {2+moment/2:.3f}s | quarter speed',29)
    for col,(run,cap)in enumerate(zip(runs,CAPS)):
        left=col*600;e=run['exact'];i=viewer.nearest(e['time_s'],moment);option=mujoco.MjvOption();option.geomgroup[3:]=0
        pixels=viewer._draw(plants[col],renderers[col],run['log'],i,camera,option,False)
        p.paste(Image.fromarray(pixels),(left,115));d=ImageDraw.Draw(p)
        text(d,(left+15,60),f'Both belts ±{cap:.2f}m/s | original .30 design',22,COLORS[col])
        text(d,(left+15,90),'Original saved native states; common fixed camera',15)
        truth=int(e['truth_dominant_patch'][i]);estimated=int(e['estimated_dominant_patch'][i])
        text(d,(left+15,477),f'Truth: {LABELS[truth]if truth>=0 else"none"} | observer: {LABELS[estimated]if estimated>=0 else"none"}',21)
        text(d,(left+15,513),f'Absolute tilt {e["absolute_tilt_deg"][i]:.2f}° | body-up speed {e["body_up_rotation_speed_deg_s"][i]:.1f}°/s',19)
        text(d,(left+15,548),f'Planar speed {e["planar_speed_m_s"][i]:.3f}m/s | timed error {e["timed_error_mm"][i]:.1f}mm',19)
        text(d,(left+15,583),f'Belt B req/applied {e["exact_native_requested"][i,4]:+.3f}/{e["exact_applied"][i,4]:+.3f}m/s',19)
        for row,c in enumerate(charts[col]):p.paste(cursor(c,moment),(left,633+row*160))
    d=ImageDraw.Draw(p)
    text(d,(22,1120),'Load fraction: cyan truth endpoint / orange observer. Blend: cyan incoming belt B / orange incoming shell B.',18)
    text(d,(22,1149),'Belt B: orange requested / cyan applied. Contact labels are instantaneous and can chatter. Truth is evaluation-only.',18)
    text(d,(22,1178),'Prep weights do not prove speed matching. Body-up speed is ||ωbody,xy||; no controller change or new physics.',17,WARN)
    return np.asarray(p)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');args=parser.parse_args()
    audit=module(HERE/'audit_event.py','event_audit');runs=[]
    for case in CASES:
        exact=audit.load(case)
        with np.load(SOURCE/'campaign/runs'/case/'rollout.npz')as z:log={k:z[k].copy()for k in z.files}
        log.update(time_s=exact['time_s'],qpos=exact['exact_qpos'],qvel=exact['exact_qvel'],reference_xy=exact['exact_reference_xy'])
        log['planned_xy']=log['reference_xy'][::40];runs.append(dict(case=case,exact=exact,log=log))
    viewer=types.ModuleType('_I50_saved_viewer');viewer.__file__=str(VIEWER);exec(compile(VIEWER.read_text(),str(VIEWER),'exec'),viewer.__dict__)
    source=Path(__file__).resolve();inputs=[source,HERE/'audit_event.py',HERE/'event_motion_report.json',VIEWER,SOURCE/'videos/video_metadata.json',SOURCE/'videos/video_display_schedule.json']
    for case in CASES:inputs += [SOURCE/'campaign/runs'/case/name for name in ('precision.npz','rollout.npz','source_snapshot/config/model.yaml','artifact_manifest.json')]
    hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    snap=HERE/'source_snapshot';snap.mkdir(exist_ok=True)
    for path in (source,VIEWER):
        dest=snap/f'{path.stem}_{sha(path)[:12]}.py'
        if not dest.exists():shutil.copy2(path,dest)
    points=[];charts=[]
    belt_max=0.
    for run in runs:
        e=run['exact'];m=(e['time_s']>=START-1e-8)&(e['time_s']<=END+1e-8);points.append(e['exact_qpos'][m,:3]);belt_max=max(belt_max,float(abs(e['exact_native_requested'][m,4]).max()))
    points=np.vstack(points);middle=.5*(points.min(axis=0)+points.max(axis=0));middle[2]+=.025
    camera=viewer._camera(middle,112.,-24.,1.12);bounds=(middle[0]-1.,middle[0]+1.,middle[1]-1.,middle[1]+1.)
    belt_high=max(.6,np.ceil(belt_max*10)/10)
    for run in runs:
        e=run['exact'];m=(e['time_s']>=START-1e-8)&(e['time_s']<=END+1e-8);t=e['time_s'][m]
        charts.append([
            chart(t,[(e['truth_belt_B_fraction'][m],CYAN),(e['estimated_belt_B_fraction'][m],ORANGE)],'Belt-B fraction of wheel support',(-.05,1.05)),
            chart(t,[(e['exact_sync_weights'][m,4],CYAN),(e['exact_sync_weights'][m,2],ORANGE)],'Incoming drive preparation blend',(-.05,1.05)),
            chart(t,[(e['exact_native_requested'][m,4],ORANGE),(e['exact_applied'][m,4],CYAN)],'Signed belt-B request / applied (m/s)',(-belt_high,belt_high),lines=[(-.3,(155,167,180)),(.3,(155,167,180))])])
    prefix='slow_preview'if args.preview else'slow_video';output=HERE/'belt_handoff_slow.mp4';metadata=HERE/f'{prefix}_metadata.json'
    if metadata.exists()or(not args.preview and output.exists()):raise FileExistsError('Preserve earlier media stage')
    plants=[];renderers=[];writer=None;count=0;schedule=[]
    try:
        for run in runs:
            config=viewer.yaml.safe_load((SOURCE/'campaign/runs'/run['case']/'source_snapshot/config/model.yaml').read_text())
            plant=viewer.replay_plant(config,viewer.MarsYard(),bounds,HERE/'_render_scenes'/run['case']);plants.append(plant);renderers.append(mujoco.Renderer(plant.model,height=350,width=600))
        first,last=card(),card(True)
        imageio.imwrite(HERE/f'{prefix}_intro.jpg',first,quality=94);imageio.imwrite(HERE/f'{prefix}_result.jpg',last,quality=94)
        if not args.preview:
            writer=imageio.get_writer(output,fps=FPS,codec='libx264',quality=8,macro_block_size=2,ffmpeg_log_level='error')
            for _ in range(2*FPS):writer.append_data(first);count+=1
        moments=(30.5,31.03,31.67,32.,32.375,33.5)if args.preview else START+np.arange(301)*.01
        for moment in moments:
            pixels=render_frame(viewer,runs,plants,renderers,charts,float(moment),camera)
            if writer is not None:writer.append_data(pixels)
            if args.preview:imageio.imwrite(HERE/f'{prefix}_recorded_{moment:.3f}s.jpg',pixels,quality=95)
            schedule.append(dict(frame=count,display_recorded_time_s=float(moment),case_state_times_s={run['case']:float(run['exact']['time_s'][viewer.nearest(run['exact']['time_s'],moment)])for run in runs}));count+=1
        if writer is not None:
            for _ in range(3*FPS):writer.append_data(last);count+=1
    finally:
        if writer is not None:writer.close()
        for renderer in renderers:renderer.close()
    assert hashes=={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    meta=dict(scope=__doc__,input_sha256=hashes,frames=count,fps=FPS,dimensions=[WIDTH,HEIGHT],recorded_window_s=[START,END],playback_speed=.25,shared_camera=dict(lookat=middle.tolist(),azimuth_deg=112,elevation_deg=-24,distance_m=1.12),motion_is_original_saved_states=True,new_physics=False,new_controller=False,truth_controller_input=False,causal_claim=False)
    if not args.preview:
        with imageio.get_reader(output)as movie:
            assert movie.count_frames()==count
            for index in (0,50,200,425):imageio.imwrite(HERE/f'slow_decoded_{index:04d}.jpg',movie.get_data(index),quality=95)
            info=movie.get_meta_data()
        meta['video']=dict(path=str(output.relative_to(ROOT)),bytes=output.stat().st_size,sha256=sha(output),frames=count,fps=FPS,duration_s=info['duration'])
    metadata.write_text(json.dumps(meta,indent=2)+'\n');(HERE/f'{prefix}_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
    print(json.dumps({k:v for k,v in meta.items()if k!='input_sha256'},indent=2))

if __name__=='__main__':main()
