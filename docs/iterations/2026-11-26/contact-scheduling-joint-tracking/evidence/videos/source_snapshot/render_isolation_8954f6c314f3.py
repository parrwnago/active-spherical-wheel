"""I51 primary four actual saved trajectories; no controller or physics execution."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import types

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
VIEWER=ROOT/'results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py'
CASES=('base030','reference030','base060','handoff060')
TITLES=('Baseline | .30m/s belts','Joint-reference guard | .30m/s belts','Baseline | .60m/s belts','Handoff continuity | .60m/s belts')
COLORS=((230,174,87),(92,200,220),(230,174,87),(113,211,168))
BG,FG,WARN=(10,20,31),(222,234,245),(246,193,108)
WIDTH,HEIGHT,FPS=1800,1500,20

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def font(size):return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',size)
def text(d,xy,value,size=20,color=FG):d.text(xy,str(value),font=font(size),fill=color)
def wrap(d,xy,value,width,size=25,color=FG):
    y=xy[1];line=''
    for word in str(value).split():
        trial=(line+' '+word).strip()
        if line and d.textlength(trial,font=font(size))>width:text(d,(xy[0],y),line,size,color);y+=size+8;line=word
        else:line=trial
    if line:text(d,(xy[0],y),line,size,color)
    return y+size+8

def load(case):
    folder=FAMILY/'campaign/runs'/case
    manifest=read(folder/'artifact_manifest.json')
    for r in manifest['files']:assert sha(folder/r['path'])==r['sha256'],(case,r['path'])
    with np.load(folder/'precision.npz')as z:p={k:z[k].copy()for k in z.files}
    with np.load(folder/'rollout.npz')as z:log={k:z[k].copy()for k in z.files}
    log.update(time_s=p['time_s'],qpos=p['exact_qpos'],qvel=p['exact_qvel'],input=p['exact_applied'],reference_xy=p['exact_reference_xy'])
    log['planned_xy']=log['reference_xy'][::40]
    q=log['qpos'][:,3:7];log['absolute_tilt_deg']=np.degrees(np.arccos(np.clip(1-2*np.sum(q[:,1:3]**2,axis=1)/np.sum(q*q,axis=1),-1,1)))
    log['timed_error_mm']=1000*np.linalg.norm(log['qpos'][:,:2]-log['reference_xy'],axis=1)
    return dict(case=case,folder=folder,exact=p,log=log,summary=read(folder/'summary.json'),diagnostics=read(folder/'candidate_diagnostics.json'))

def inset(pixels,run,i,bounds,planned):
    p=Image.fromarray(pixels);d=ImageDraw.Draw(p,'RGBA');left,top,size=712,185,170;x0,x1,y0,y1=bounds
    d.rectangle((left,top,left+size,top+size),fill=(7,16,25,235))
    def pt(xy):return left+9+(xy[0]-x0)/(x1-x0)*(size-18),top+22+(y1-xy[1])/(y1-y0)*(size-42)
    for points,color in [(planned,(244,174,75,255)),(run['log']['qpos'][:i+1,:2],(100,216,232,255))]:
        pairs=[pt(x)for x in points[::max(1,len(points)//250)]]
        if len(pairs)>1:d.line(pairs,fill=color,width=2)
    x,y=pt(run['log']['qpos'][i,:2]);d.ellipse((x-3,y-3,x+3,y+3),fill=(100,216,232,255))
    text(d,(left+7,top+2),'Fixed world XY',13);text(d,(left+7,top+size-17),'Orange plan / cyan robot',10)
    return np.asarray(p)

def chart(runs,title,high):
    p=Image.new('RGB',(900,180),BG);d=ImageDraw.Draw(p);left,top,w,h=52,35,820,110
    text(d,(18,0),title,20);d.rectangle((left,top,left+w,top+h),fill=(4,12,20),outline=(60,81,98))
    for v in (0,high):text(d,(4,top+(h-14 if v==0 else 0)),f'{v:g}',12)
    for run,color in zip(runs,[(230,174,87),(100,216,232)]):
        pairs=[(left+t/44*w,top+h-e/high*h)for t,e in zip(run['log']['time_s'],run['log']['timed_error_mm'])]
        d.line(pairs,fill=color,width=2)
    for t in (0,11,22,33,44):text(d,(left+t/44*w-7,top+h+7),str(t),12)
    return p

def cursor(base,t):
    p=base.copy();d=ImageDraw.Draw(p);x=52+t/44*820;d.line((x,35,x,145),fill=(249,248,239),width=2);return p

def intro():
    p=Image.new('RGB',(WIDTH,HEIGHT),BG);d=ImageDraw.Draw(p)
    text(d,(65,60),'I51: separate joint-reference and belt-handoff experiments',39,WARN)
    y=185
    for line in [
        'Four fresh native Gazebo/DART runs, original Mars figure-eight, phase+0.90, initial yaw45°, full44s.',
        'Compare only matched-cap pairs: .30baseline versus joint-reference guard; .60baseline versus belt-handoff continuity.',
        'Every arm keeps the original .30design/gain library, controller mode, observer, path and nominal torque bounds.',
        'Joint-reference guard: only shaft/shell rate targets are boxed at20rad/s and100rad/s² before the one native feedback call.',
        'Handoff continuity: only total belt commands during estimated load-state transitions are projected at3m/s² around the previous ACTUALLY APPLIED input.',
        'Both modifiers are declared simulation assumptions, not measured motor limits. No combined candidate is shown.',
        'All panels use the same fixed camera, worldXY scale and recorded program clock. Full0–44s plays at2×.',
        'Mechanistic command bounds are separate from actual tracking, balance, settling and acceptance-gate success.',
        'This movie re-renders saved actual states. It performs no physics, controller or new scientific reduction.',
    ]:y=wrap(d,(75,y),line,1650,29)+42
    text(d,(75,1420),'Primary four arms only. Any separate late-phase test is outside this movie.',26,WARN)
    return np.asarray(p)

def resultcard(report):
    p=Image.new('RGB',(WIDTH,HEIGHT),BG);d=ImageDraw.Draw(p)
    text(d,(55,45),'I51 results: physical motion decides; bounded commands alone are insufficient',35,WARN)
    text(d,(55,118),'Two matched-cap comparisons, one original route/phase; original gain library/design unchanged.',26)
    for x,s in [(55,'Arm'),(650,'Moving RMS'),(920,'Peak / final'),(1300,'Peak ref. lean'),(1550,'Settled / gates')]:text(d,(x,220),s,22,(157,196,219))
    for i,case in enumerate(CASES):
        c=report['cases'][case];t=c['timed_tracking'];y=295+i*105
        text(d,(55,y),TITLES[i],25,COLORS[i]);text(d,(650,y),f'{1000*t["moving_rms_m"]:.2f}mm',25)
        text(d,(920,y),f'{1000*t["full_peak_m"]:.2f}/{1000*t["final_m"]:.2f}mm',25)
        text(d,(1300,y),f'{c["attitude"]["peak_reference_error_deg"]:.2f}°',25)
        text(d,(1550,y),f'{c["original_settling"]["passed"]}/{c["original_route_gates"]["passed"]}',23)
    y=790
    for case,label in [('reference030','Joint-reference candidate versus .30baseline'),('handoff060','Handoff candidate versus .60baseline')]:
        comparison=report['comparisons'][case]
        y=wrap(d,(55,y),f'{label}: provisional screen passes = {comparison["passes_all_conditions"]}.',1690,28,WARN)+37
    y=wrap(d,(55,y),'Completion, settling and original route gates remain distinct. Inspect the canonical report for all six fixed turn events, contact/load/slip and requested versus applied commands.',1690,26)+35
    y=wrap(d,(55,y),'These are independent native closed-loop experiments. A reference/command projection may shift later contact and observer history; same-state saved-array algebra does not predict those outcomes.',1690,26)+35
    y=wrap(d,(55,y),'No repository default is adopted by this movie. The ideal belt servo and declared reference/slew limits do not validate motor torque-speed, electrical or thermal response.',1690,26,WARN)
    return np.asarray(p)

def frame(viewer,runs,plants,renderers,charts,moment,camera,bounds,planned):
    p=Image.new('RGB',(WIDTH,HEIGHT),BG);d=ImageDraw.Draw(p)
    text(d,(24,12),f'I51 PRIMARY FOUR ARMS | recorded t={moment:05.2f}s | playback2× | matched-cap pairs by row',30)
    for k,run in enumerate(runs):
        left,top=(k%2)*900,60+(k//2)*600;e=run['exact'];log=run['log'];end=float(log['time_s'][-1]);i=viewer.nearest(log['time_s'],min(moment,end));ended=moment>end+1e-9
        text(d,(left+18,top),TITLES[k],26,COLORS[k]);text(d,(left+18,top+33),'Original .30gain design | actual saved native Gazebo/DART state',17)
        option=mujoco.MjvOption();option.geomgroup[3:]=0
        pixels=viewer._draw(plants[k],renderers[k],log,i,camera,option,False);pixels=inset(pixels,run,i,bounds,planned)
        p.paste(Image.fromarray(pixels),(left,top+55));d=ImageDraw.Draw(p)
        text(d,(left+18,top+463),f'Timed error {log["timed_error_mm"][i]:.1f}mm | abs.tilt {log["absolute_tilt_deg"][i]:.2f}° | ref.lean {np.degrees(log["attitude_error_rad"][i]):.2f}°',20)
        text(d,(left+18,top+493),f'Planar speed {np.linalg.norm(log["qvel"][i,:2]):.3f}m/s | body-up rotation {np.degrees(np.linalg.norm(log["qvel"][i,3:5])):.1f}°/s',20)
        if k<2:
            text(d,(left+18,top+523),f'Max|joint rate target| native / guarded {np.max(abs(e["exact_native_joint_reference"][i])):.2f}/{np.max(abs(e["exact_guarded_joint_reference"][i])):.2f}rad/s',20)
        else:
            text(d,(left+18,top+523),f'Belt B native box / final {e["exact_native_applied"][i,4]:+.3f}/{e["exact_applied"][i,4]:+.3f}m/s',20)
        if ended:text(d,(left+18,top+553),f'ENDED at{end:.3f}s; last actual state held, no fabricated continuation',19,WARN)
        elif k==3:
            hd=run['diagnostics'][i]['handoff'];text(d,(left+18,top+553),f'Handoff episode A/B {hd["episode_active"]} | limited A/B {hd["limited"]}',18)
        else:text(d,(left+18,top+553),f'Actual saved program time {log["time_s"][i]:.3f}s; unchanged timed XYreference',18)
    for col,c in enumerate(charts):p.paste(cursor(c,moment),(col*900,1270))
    d=ImageDraw.Draw(p);text(d,(22,1460),'Timed-error strips: orange matched-cap baseline / cyan candidate. Shared axes and original reference program.',21)
    text(d,(22,1480),'Simulation assumptions only; bounds do not establish improved motion or actual motor capability.',16,WARN)
    return np.asarray(p)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');args=parser.parse_args()
    stage=FAMILY/'analysis/stage01'
    if not(stage/'artifact_manifest.json').is_file():raise FileNotFoundError('Wait for canonical scientific analysis seal')
    qa=read(FAMILY/'review/native_qa/stage01/report.json');assert qa['integrity_passed']
    for row in read(stage/'artifact_manifest.json')['files']:
        target=stage/row['path']
        if not target.is_file():target=ROOT/row['path']
        assert sha(target)==row['sha256'],row['path']
    report=read(stage/'report.json');runs=[load(case)for case in CASES];plan=read(HERE/'presentation_plan.json')
    viewer=types.ModuleType('_I51_saved_viewer');viewer.__file__=str(VIEWER);exec(compile(VIEWER.read_text(),str(VIEWER),'exec'),viewer.__dict__)
    planned=runs[0]['log']['reference_xy'][::40];mid=.5*(planned.min(axis=0)+planned.max(axis=0));span=max(float(np.ptp(planned,axis=0).max())+.65,1.4);bounds=(mid[0]-span/2,mid[0]+span/2,mid[1]-span/2,mid[1]+span/2)
    camera=viewer._camera(np.r_[mid,runs[0]['log']['qpos'][0,2]],112.,-27.,1.9);terrainbounds=(mid[0]-1.2,mid[0]+1.2,mid[1]-1.2,mid[1]+1.2)
    high=max(25.,np.ceil(max(run['log']['timed_error_mm'].max()for run in runs)/25)*25)
    charts=[chart(runs[:2],'.30m/s matched pair: original timed-path error(mm)',high),chart(runs[2:],'.60m/s matched pair: original timed-path error(mm)',high)]
    source=Path(__file__).resolve();inputs=[source,VIEWER,HERE/'presentation_plan.json',FAMILY/'protocol.json',stage/'report.json',stage/'artifact_manifest.json',FAMILY/'review/native_qa/stage01/report.json']
    for run in runs:inputs += [run['folder']/name for name in ('artifact_manifest.json','precision.npz','rollout.npz','summary.json','candidate_diagnostics.json','source_snapshot/config/model.yaml')]
    hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs};snap=HERE/'source_snapshot';snap.mkdir(exist_ok=True)
    for path in (source,VIEWER):
        dest=snap/f'{path.stem}_{sha(path)[:12]}.py'
        if not dest.exists():shutil.copy2(path,dest)
    prefix='preview'if args.preview else'video';metadata=HERE/f'{prefix}_metadata.json';output=HERE/'allocation_handoff_comparison.mp4'
    if metadata.exists()or(not args.preview and output.exists()):raise FileExistsError('Preserve prior render stage')
    plants=[];renderers=[];writer=None;count=0;schedule=[]
    try:
        for run in runs:
            cfg=viewer.yaml.safe_load((run['folder']/'source_snapshot/config/model.yaml').read_text());plant=viewer.replay_plant(cfg,viewer.MarsYard(),terrainbounds,HERE/'_render_scenes'/run['case']);plants.append(plant);renderers.append(mujoco.Renderer(plant.model,height=400,width=900))
        first,last=intro(),resultcard(report);imageio.imwrite(HERE/f'{prefix}_intro.jpg',first,quality=95);imageio.imwrite(HERE/f'{prefix}_resultcard.jpg',last,quality=95)
        if not args.preview:
            writer=imageio.get_writer(output,fps=FPS,codec='libx264',quality=8,macro_block_size=2,ffmpeg_log_level='error')
            for _ in range(2*FPS):writer.append_data(first);count+=1
        moments=(0.,6.825,18.,31.65,32.17,40.625,44.)if args.preview else np.arange(0.,44.+1e-9,.1)
        for moment in moments:
            pixels=frame(viewer,runs,plants,renderers,charts,float(moment),camera,bounds,planned)
            if writer is not None:writer.append_data(pixels)
            if args.preview:imageio.imwrite(HERE/f'{prefix}_recorded_{moment:.3f}s.jpg',pixels,quality=95)
            schedule.append(dict(frame=count,recorded_time_s=float(moment),case_state_times_s={r['case']:float(r['log']['time_s'][viewer.nearest(r['log']['time_s'],min(moment,r['log']['time_s'][-1]))])for r in runs}));count+=1
        if writer is not None:
            for _ in range(4*FPS):writer.append_data(last);count+=1
    finally:
        if writer is not None:writer.close()
        for r in renderers:r.close()
    assert hashes=={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    meta=dict(iteration='I-51',scope=__doc__,primary_cases=list(CASES),input_sha256=hashes,frames=count,fps=FPS,dimensions=[WIDTH,HEIGHT],recorded_window_s=[0,44],playback_speed=2.,shared_camera=dict(lookat=np.r_[mid,runs[0]['log']['qpos'][0,2]].tolist(),azimuth_deg=112,elevation_deg=-27,distance_m=1.9,world_xy_bounds=list(bounds)),no_new_physics=True,no_controller_execution=True,no_scientific_reduction=True,default_changed=False,case_end_times_s={r['case']:float(r['log']['time_s'][-1])for r in runs})
    if not args.preview:
        with imageio.get_reader(output)as movie:
            assert movie.count_frames()==count
            for index in (0,40,280,560):imageio.imwrite(HERE/f'decoded_{index:04d}.jpg',movie.get_data(index),quality=95)
            info=movie.get_meta_data()
        meta['video']=dict(path=str(output.relative_to(ROOT)),sha256=sha(output),bytes=output.stat().st_size,frames=count,fps=FPS,duration_s=info['duration'])
    metadata.write_text(json.dumps(meta,indent=2)+'\n');(HERE/f'{prefix}_display_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n');print(json.dumps({k:v for k,v in meta.items()if k!='input_sha256'},indent=2))

if __name__=='__main__':main()
