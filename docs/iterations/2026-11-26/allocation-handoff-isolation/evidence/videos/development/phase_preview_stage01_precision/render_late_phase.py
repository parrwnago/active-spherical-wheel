"""Separate I51 late-only phase scheduling diagnostic; saved actual states only."""
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
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
VIEWER=ROOT/'results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py'
PHASE=FAMILY/'phase_schedule_isolation/campaign/runs/phase_late030'
REPORT=FAMILY/'review/late_phase_stage01/report.json'
START,END,FPS=36.,44.,20
WIDTH,HEIGHT=1800,1200

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())

def load(folder,case):
    for r in read(folder/'artifact_manifest.json')['files']:assert sha(folder/r['path'])==r['sha256']
    with np.load(folder/'precision.npz')as z:e={k:z[k].copy()for k in z.files}
    with np.load(folder/'rollout.npz')as z:log={k:z[k].copy()for k in z.files}
    log.update(time_s=e['time_s'],qpos=e['exact_qpos'],qvel=e['exact_qvel'],input=e['exact_applied'],reference_xy=e['exact_reference_xy']);log['planned_xy']=log['reference_xy'][::40]
    q=log['qpos'][:,3:7];log['absolute_tilt_deg']=np.degrees(np.arccos(np.clip(1-2*np.sum(q[:,1:3]**2,axis=1)/np.sum(q*q,axis=1),-1,1)))
    log['timed_error_mm']=1000*np.linalg.norm(log['qpos'][:,:2]-log['reference_xy'],axis=1)
    return dict(case=case,folder=folder,exact=e,log=log)

def card(paint,report,end=False):
    pic=Image.new('RGB',(WIDTH,HEIGHT),paint.BG);d=ImageDraw.Draw(pic)
    paint.text(d,(60,50),'I51 separate late-phase diagnostic: same native history until38s',37,paint.WARN)
    if not end:
        lines=[
            'Two original .30m/s-belt runs: unchanged baseline and one separately registered gain-phase diagnostic.',
            'At recorded38s, only gain scheduling switches from world-vertical phase to phase relative to the estimated support normal.',
            'The two runs have identical saved state, observer, reference and input history before the switch.',
            'Recorded36–44s is replayed at half speed, with a common fixed camera, worldXY scale and program clock.',
            'The38s time gate is an episode-isolation device. It is not a deployable policy, automatic detector or repository default.',
            'The normal is observer-derived; no solved contact or true terrain normal is provided to the controller.',
            'This movie re-renders saved actual Gazebo/DART states. No new physics/controller or scientific reduction occurs.',
        ]
        y=185
        for line in lines:y=paint.wrap(d,(70,y),line,1650,29)+39
    else:
        paint.text(d,(65,155),'Matched-history timing makes this a separate causal scheduling intervention for this episode.',28)
        headings=[('Run',65),('Late38–44 RMS',470),('Final error',850),('Full44s peak',1170),('Moving1–41 RMS',1460)]
        for label,x in headings:paint.text(d,(x,270),label,23,(157,196,219))
        for i,(key,title)in enumerate([('baseline','Baseline'),('phase','Phase ON at38s')]):
            c=report['cases'][key];y=355+i*112
            for x,value in [(65,title),(470,f'{1000*c["windows"]["38_44"]["position_rms_m"]:.2f}mm'),(850,f'{1000*c["final_error_m"]:.2f}mm'),(1170,f'{1000*c["peak_error_m"]:.2f}mm'),(1460,f'{1000*c["moving_rms_m"]:.2f}mm')]:paint.text(d,(x,y),value,27,paint.FG if i==0 else(113,211,168))
        y=650
        for line in [
            'The late episode and final error improve here; full moving RMS is essentially unchanged and slightly worse.',
            'This does not establish a full-route fix, universally correct scheduling rule or deployable time switch.',
            'Only one original route/phase and one late episode are tested. History after the switch is free to diverge physically.',
            'The primary four-arm comparison remains separate. No controller default or motor model is changed.',
        ]:y=paint.wrap(d,(65,y),line,1660,28,paint.WARN if y==650 else paint.FG)+33
    paint.text(d,(65,1120),'TIME-GATED DIAGNOSTIC ONLY |38s switch is not a deployable controller or default.',26,paint.WARN)
    return np.asarray(pic)

def chart(paint,runs,title,key,high):
    pic=Image.new('RGB',(900,230),paint.BG);d=ImageDraw.Draw(pic);left,top,w,h=55,42,815,145
    paint.text(d,(18,1),title,21);d.rectangle((left,top,left+w,top+h),fill=(4,12,20),outline=(60,81,98))
    for val in (0,high):paint.text(d,(3,top+(h-15 if val==0 else 0)),f'{val:g}',12)
    x38=left+(38-START)/(END-START)*w;d.line((x38,top,x38,top+h),fill=paint.WARN,width=2);paint.text(d,(x38+5,top+4),'38s switch',14,paint.WARN)
    for run,color in zip(runs,[(230,174,87),(100,216,232)]):
        log=run['log'];m=(log['time_s']>=START-1e-8)&(log['time_s']<=END+1e-8);pairs=[(left+(t-START)/(END-START)*w,top+h-e/high*h)for t,e in zip(log['time_s'][m],log[key][m])];d.line(pairs,fill=color,width=2)
    for t in (36,38,40,42,44):paint.text(d,(left+(t-START)/(END-START)*w-8,top+h+8),str(t),13)
    return pic

def frame(paint,viewer,runs,plants,renderers,charts,moment,camera,bounds,planned):
    p=Image.new('RGB',(WIDTH,HEIGHT),paint.BG);d=ImageDraw.Draw(p)
    paint.text(d,(23,12),f'I51 SEPARATE LATE DIAGNOSTIC | recorded t={moment:.3f}s | half speed | phase switch at38s',30)
    for col,run in enumerate(runs):
        left=900*col;log=run['log'];e=run['exact'];i=viewer.nearest(log['time_s'],moment);active=col==1 and moment>=38.-1e-8
        paint.text(d,(left+18,70),'Original baseline .30m/s'if col==0 else'Estimated-normal gain phase from38s',26,paint.COLORS[0]if col==0 else(113,211,168))
        status='Original world-vertical scheduling'if col==0 or not active else'Estimated-normal phase ON (time-gated diagnostic)'
        paint.text(d,(left+18,111),status,19,paint.WARN if active else paint.FG)
        option=mujoco.MjvOption();option.geomgroup[3:]=0;pixels=viewer._draw(plants[col],renderers[col],log,i,camera,option,False);pixels=paint.inset(pixels,run,i,bounds,planned);p.paste(Image.fromarray(pixels),(left,150));d=ImageDraw.Draw(p)
        paint.text(d,(left+18,625),f'Timed error {log["timed_error_mm"][i]:.1f}mm | ref.lean {np.degrees(log["attitude_error_rad"][i]):.2f}°',23)
        paint.text(d,(left+18,666),f'Absolute tilt {log["absolute_tilt_deg"][i]:.2f}° | planar speed {np.linalg.norm(log["qvel"][i,:2]):.3f}m/s',22)
        paint.text(d,(left+18,707),f'Actual saved time {log["time_s"][i]:.3f}s | original .30gain design andcaps',19)
    for col,c in enumerate(charts):
        copy=c.copy();d=ImageDraw.Draw(copy);x=55+(moment-START)/(END-START)*815;d.line((x,42,x,187),fill=(249,248,239),width=2);p.paste(copy,(col*900,780))
    d=ImageDraw.Draw(p);paint.text(d,(23,1040),'Orange baseline / cyan phase diagnostic. Same program, initial history and camera. Vertical marker=38s switch.',23)
    paint.wrap(d,(23,1092),'The38s gate isolates this observed episode. It is not a deployable controller policy, default setting or proof of a full-route fix.',1750,25,paint.WARN)
    return np.asarray(p)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');args=parser.parse_args()
    report=read(REPORT);assert report['passed'];spec=importlib.util.spec_from_file_location('i51_media',HERE/'render_isolation.py');paint=importlib.util.module_from_spec(spec);spec.loader.exec_module(paint)
    runs=[load(FAMILY/'campaign/runs/base030','base030'),load(PHASE,'phase_late030')]
    for run in runs:run['log']['reference_lean_deg']=np.degrees(run['log']['attitude_error_rad'])
    viewer=types.ModuleType('_I51_late_viewer');viewer.__file__=str(VIEWER);exec(compile(VIEWER.read_text(),str(VIEWER),'exec'),viewer.__dict__)
    planned=runs[0]['log']['reference_xy'][::40];mid=.5*(planned.min(axis=0)+planned.max(axis=0));span=max(float(np.ptp(planned,axis=0).max())+.65,1.4);bounds=(mid[0]-span/2,mid[0]+span/2,mid[1]-span/2,mid[1]+span/2);terrainbounds=(mid[0]-1.2,mid[0]+1.2,mid[1]-1.2,mid[1]+1.2);camera=viewer._camera(np.r_[mid,runs[0]['log']['qpos'][0,2]],112.,-27.,1.9)
    charts=[chart(paint,runs,'Timed-path error(mm), late36–44s','timed_error_mm',200.),chart(paint,runs,'Reference-relative lean(deg), late36–44s','reference_lean_deg',10.)]
    source=Path(__file__).resolve();inputs=[source,HERE/'render_isolation.py',VIEWER,REPORT,FAMILY/'phase_schedule_isolation/protocol.json']
    for run in runs:inputs += [run['folder']/n for n in ('artifact_manifest.json','precision.npz','rollout.npz','source_snapshot/config/model.yaml')]
    hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs};snap=HERE/'source_snapshot';snap.mkdir(exist_ok=True)
    for path in (source,HERE/'render_isolation.py',VIEWER):
        dest=snap/f'{path.stem}_{sha(path)[:12]}.py'
        if not dest.exists():shutil.copy2(path,dest)
    prefix='phase_preview'if args.preview else'phase_video';metadata=HERE/f'{prefix}_metadata.json';output=HERE/'late_phase_diagnostic.mp4'
    if metadata.exists()or(not args.preview and output.exists()):raise FileExistsError('Preserve old phase media stage')
    plants=[];renderers=[];writer=None;count=0;schedule=[]
    try:
        for run in runs:
            cfg=viewer.yaml.safe_load((run['folder']/'source_snapshot/config/model.yaml').read_text());plant=viewer.replay_plant(cfg,viewer.MarsYard(),terrainbounds,HERE/'_phase_render_scenes'/run['case']);plants.append(plant);renderers.append(mujoco.Renderer(plant.model,height=450,width=900))
        first,last=card(paint,report),card(paint,report,True);imageio.imwrite(HERE/f'{prefix}_intro.jpg',first,quality=95);imageio.imwrite(HERE/f'{prefix}_result.jpg',last,quality=95)
        if not args.preview:
            writer=imageio.get_writer(output,fps=FPS,codec='libx264',quality=8,macro_block_size=2,ffmpeg_log_level='error')
            for _ in range(2*FPS):writer.append_data(first);count+=1
        moments=(36.,38.,40.625,44.)if args.preview else START+np.arange(321)*.025
        for moment in moments:
            pixels=frame(paint,viewer,runs,plants,renderers,charts,float(moment),camera,bounds,planned)
            if writer is not None:writer.append_data(pixels)
            if args.preview:imageio.imwrite(HERE/f'{prefix}_recorded_{moment:.3f}s.jpg',pixels,quality=95)
            schedule.append(dict(frame=count,recorded_time_s=float(moment),phase_flag=bool(moment>=38.-1e-8),case_state_times_s={r['case']:float(r['log']['time_s'][viewer.nearest(r['log']['time_s'],moment)])for r in runs}));count+=1
        if writer is not None:
            for _ in range(3*FPS):writer.append_data(last);count+=1
    finally:
        if writer is not None:writer.close()
        for r in renderers:r.close()
    assert hashes=={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    meta=dict(scope=__doc__,cases=['base030','phase_late030'],input_sha256=hashes,frames=count,fps=FPS,dimensions=[WIDTH,HEIGHT],recorded_window_s=[START,END],playback_speed=.5,diagnostic_time_gate_s=38.,deployable_policy_claim=False,no_new_physics=True,no_controller_execution=True,no_scientific_reduction=True,default_changed=False,shared_camera=dict(lookat=np.r_[mid,runs[0]['log']['qpos'][0,2]].tolist(),azimuth_deg=112,elevation_deg=-27,distance_m=1.9,world_xy_bounds=list(bounds)))
    if not args.preview:
        with imageio.get_reader(output)as movie:
            assert movie.count_frames()==count
            for i in (0,40,120,260,420):imageio.imwrite(HERE/f'phase_decoded_{i:04d}.jpg',movie.get_data(i),quality=95)
            info=movie.get_meta_data()
        meta['video']=dict(path=str(output.relative_to(ROOT)),bytes=output.stat().st_size,sha256=sha(output),frames=count,fps=FPS,duration_s=info['duration'])
    metadata.write_text(json.dumps(meta,indent=2)+'\n');(HERE/f'{prefix}_display_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n');print(json.dumps({k:v for k,v in meta.items()if k!='input_sha256'},indent=2))

if __name__=='__main__':main()
