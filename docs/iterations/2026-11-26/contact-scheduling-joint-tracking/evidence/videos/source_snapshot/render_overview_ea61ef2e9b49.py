"""I52 fixed twelve-cell actual saved-state overview; no new control/physics."""
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
PAINT=ROOT/'results/allocation_handoff_isolation_20261126/videos/render_isolation.py'
PHASES=('m100','p030','p090')
VARIANTS=('B','R','N','NR')
TITLES={'B':'B: original baseline','R':'R: joint-reference guard','N':'N: normal-relative gain phase','NR':'NR: both factors'}
COLORS={'B':(230,174,87),'R':(92,200,220),'N':(176,135,218),'NR':(113,211,168)}
WIDTH,HEIGHT,FPS=1800,1500,20

def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def load(case,folder):
    for r in read(folder/'artifact_manifest.json')['files']:assert sha(folder/r['path'])==r['sha256'],(case,r['path'])
    with np.load(folder/'precision.npz')as z:e={k:z[k].copy()for k in z.files}
    with np.load(folder/'rollout.npz')as z:log={k:z[k].copy()for k in z.files}
    log.update(time_s=e['time_s'],qpos=e['exact_qpos'],qvel=e['exact_qvel'],input=e['exact_applied'],reference_xy=e['exact_reference_xy']);log['planned_xy']=log['reference_xy'][::40]
    q=log['qpos'][:,3:7];log['absolute_tilt_deg']=np.degrees(np.arccos(np.clip(1-2*np.sum(q[:,1:3]**2,axis=1)/np.sum(q*q,axis=1),-1,1)))
    log['timed_error_mm']=1000*np.linalg.norm(log['qpos'][:,:2]-log['reference_xy'],axis=1);log['reference_lean_deg']=np.degrees(log['attitude_error_rad'])
    return dict(case=case,folder=folder,exact=e,log=log)

def chart(paint,runs,key,title,high):
    pic=Image.new('RGB',(900,180),paint.BG);d=ImageDraw.Draw(pic);left,top,w,h=52,35,820,110
    paint.text(d,(18,0),title,20);d.rectangle((left,top,left+w,top+h),fill=(4,12,20),outline=(60,81,98))
    for v in (0,high):paint.text(d,(3,top+(h-14 if v==0 else 0)),f'{v:g}',12)
    for variant,run in zip(VARIANTS,runs):
        pairs=[(left+t/44*w,top+h-value/high*h)for t,value in zip(run['log']['time_s'],run['log'][key])]
        if len(pairs)>1:d.line(pairs,fill=COLORS[variant],width=2)
    for t in (0,11,22,33,44):paint.text(d,(left+t/44*w-7,top+h+7),str(t),12)
    return pic

def intro(paint):
    p=Image.new('RGB',(WIDTH,HEIGHT),paint.BG);d=ImageDraw.Draw(p)
    paint.text(d,(65,55),'I52: whole-run scheduling × joint-reference tracking, three starting phases',36,paint.WARN)
    y=185
    for line in [
        'Twelve valid fresh native Gazebo/DART figure-eight runs: initial shaft phases−1.00,+0.30,+0.90rad; all belts±0.30m/s.',
        'Each fixed phase chapter shows the same2×2: B original, R joint-reference guard, N estimated-normal gain lookup, NR both.',
        'The same flat gain bank, observer, trajectory and motor-torque limits are retained. Ordinary local anchors differ by initial phase.',
        'R uses the exact I51 reference guard:20rad/s and100rad/s² assumptions. These are reference constraints, not measured motor limits.',
        'N changes the lookup angle in the existing flat gain bank from the first control call. It does not redesign dynamics or transform every error/input frame.',
        'All chapters replay the complete recorded0–44s at4×, with the original shared fixed camera, worldXY scale and plot axes.',
        'This is an overview.200ms pose sampling cannot resolve individual5ms twitches; full5ms scientific plots and earlier slow clips remain available.',
        'One technical process crash and its registered exact-source retry are preserved. Twelve valid scored cells come from thirteen process attempts.',
        'Saved actual states are re-rendered. No new physics, controller, observer, forecast or scientific reduction occurs here.',
    ]:y=paint.wrap(d,(75,y),line,1650,28)+38
    return np.asarray(p)

def phasecard(paint,report,phase):
    p=Image.new('RGB',(WIDTH,HEIGHT),paint.BG);d=ImageDraw.Draw(p);value=report['per_phase'][phase]['phase_rad']
    paint.text(d,(60,55),f'I52 starting phase {value:+.2f}rad: outcomes remain configuration dependent',37,paint.WARN)
    paint.text(d,(60,130),'All four completed44s. Compare only within this starting phase; same .30m/s cap and fixed gain bank.',26)
    for x,label in [(60,'Policy'),(440,'Moving RMS'),(760,'Peak / final'),(1190,'Peak ref. lean'),(1515,'Settled / route')]:paint.text(d,(x,260),label,24,(157,196,219))
    for i,v in enumerate(VARIANTS):
        c=report['cases'][phase+'_'+v];t=c['timed_tracking'];y=355+i*125
        for x,text in [(60,TITLES[v]),(440,f'{1000*t["moving_rms_m"]:.2f}mm'),(760,f'{1000*t["full_peak_m"]:.2f}/{1000*t["final_m"]:.2f}mm'),(1190,f'{c["attitude"]["peak_reference_error_deg"]:.2f}°'),(1515,f'{c["original_settling"]["passed"]}/{c["original_route_gates"]["passed"]}')]:paint.text(d,(x,y),text,25,COLORS[v]if x==60 else paint.FG)
    y=945
    for v in ('R','N','NR'):
        passed=report['per_phase'][phase]['comparisons'][v+'-B']['passes_all_conditions']
        paint.text(d,(65,y),f'{v} versus own-phase B: registered full screen passes = {passed}',28,paint.WARN);y+=80
    paint.wrap(d,(65,1270),'Original settling/route success is separate from stricter turn/regression requirements. All72 fixed turn outcomes remain in the report; no pooled average hides a failure.',1650,27)
    return np.asarray(p)

def finalcard(paint,report):
    p=Image.new('RGB',(WIDTH,HEIGHT),paint.BG);d=ImageDraw.Draw(p)
    paint.text(d,(60,55),'I52: useful local improvements, with regressions across starting configurations',35,paint.WARN)
    paint.text(d,(65,155),'All12 valid cells completed44s. No R/N/NR policy passes the full registered screen across allthree starts.',28)
    for x,label in [(70,'Policy'),(450,'Phase−1.00'),(850,'Phase+0.30'),(1250,'Phase+0.90')]:paint.text(d,(x,280),label,29,(157,196,219))
    for i,v in enumerate(('R','N','NR')):
        y=375+i*110;paint.text(d,(70,y),v,30,COLORS[v])
        for x,phase in zip((450,850,1250),PHASES):paint.text(d,(x,y),str(report['policy_screens'][v]['phase_passes'][phase]),30)
    y=805
    for line in [
        'N at−1.00rad and NR at+0.90rad pass original settling/route gates, but fail registered turn-regression checks.',
        'R at−1.00rad worsens motion and lean; normal scheduling and the combined policy also show mixed tradeoffs at+0.30rad.',
        'Native-versus-used reference errors and actual joint rates are reported separately. Smaller requested targets alone are not improved motor tracking.',
        'The reference guard introduces a current estimated-contact rolling-reference mismatch; this is separate from actual slip or body tracking.',
        'These are three deterministic configurations on one route, not statistical replication or a general terrain qualification. Defaults remain unchanged.',
    ]:y=paint.wrap(d,(65,y),line,1650,27,paint.WARN if y==805 else paint.FG)+35
    return np.asarray(p)

def frame(paint,viewer,runs,plants,renderers,charts,phase,phase_value,moment,playback,camera,bounds,planned):
    p=Image.new('RGB',(WIDTH,HEIGHT),paint.BG);d=ImageDraw.Draw(p)
    paint.text(d,(22,10),f'I52 OVERVIEW | startphase {phase_value:+.2f}rad | recorded t={moment:05.2f}s | playback4× | movie t={playback:05.2f}s',29)
    for k,(v,run)in enumerate(zip(VARIANTS,runs)):
        left,top=(k%2)*900,60+(k//2)*600;log=run['log'];e=run['exact'];i=viewer.nearest(log['time_s'],moment)
        paint.text(d,(left+18,top),TITLES[v],26,COLORS[v]);paint.text(d,(left+18,top+33),'Actual saved native state | whole-run policy | original .30gain bank/cap',16)
        option=mujoco.MjvOption();option.geomgroup[3:]=0;pixels=viewer._draw(plants[k],renderers[k],log,i,camera,option,False);pixels=paint.inset(pixels,run,i,bounds,planned);p.paste(Image.fromarray(pixels),(left,top+55));d=ImageDraw.Draw(p)
        paint.text(d,(left+18,top+463),f'Timed error {log["timed_error_mm"][i]:.1f}mm | abs.tilt {log["absolute_tilt_deg"][i]:.2f}° | ref.lean {log["reference_lean_deg"][i]:.2f}°',20)
        paint.text(d,(left+18,top+493),f'Max|joint target| native/used {np.max(abs(e["exact_native_joint_reference"][i])):.2f}/{np.max(abs(e["exact_guarded_joint_reference"][i])):.2f}rad/s | actual {np.max(abs(e["exact_qvel"][i,6:9])):.2f}',19)
        paint.text(d,(left+18,top+523),f'Current lookup phase {np.degrees(e["exact_effective_phase_rad"][i]):+.1f}° | source '+('estimated normal'if v in ('N','NR')else'world vertical'),20)
        changed=bool(np.any(e['exact_reference_guard_delta'][i]!=0));paint.text(d,(left+18,top+553),f'Actual recorded {log["time_s"][i]:.3f}s | reference modified now={changed}',18)
    for col,c in enumerate(charts):
        q=c.copy();d=ImageDraw.Draw(q);x=52+moment/44*820;d.line((x,35,x,145),fill=(249,248,239),width=2);p.paste(q,(col*900,1270))
    d=ImageDraw.Draw(p)
    for x,v in zip((24,390,800,1220),VARIANTS):paint.text(d,(x,1460),TITLES[v],21,COLORS[v])
    paint.text(d,(24,1480),'4× overview with200ms poses; scientific traces retain5ms samples. Assumed reference bounds are not physical motor limits.',16,paint.WARN)
    return np.asarray(p)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');args=parser.parse_args()
    stage=FAMILY/'analysis/stage01'
    if not(stage/'artifact_manifest.json').is_file():raise FileNotFoundError('Wait for sealed canonical analysis')
    report=read(stage/'report.json');assert report['integrity']['passed'];resolution=read(FAMILY/'campaign/resolved_case_paths.json');plan=read(HERE/'presentation_plan.json')
    spec=importlib.util.spec_from_file_location('i52_paint',PAINT);paint=importlib.util.module_from_spec(spec);spec.loader.exec_module(paint)
    viewer=types.ModuleType('_I52_saved_viewer');viewer.__file__=str(VIEWER);exec(compile(VIEWER.read_text(),str(VIEWER),'exec'),viewer.__dict__)
    allruns={case:load(case,ROOT/folder)for case,folder in resolution['cases'].items()};reference=allruns['p090_B'];planned=reference['log']['reference_xy'][::40];mid=.5*(planned.min(axis=0)+planned.max(axis=0));span=max(float(np.ptp(planned,axis=0).max())+.65,1.4)
    bounds=(mid[0]-span/2,mid[0]+span/2,mid[1]-span/2,mid[1]+span/2);terrainbounds=(mid[0]-1.2,mid[0]+1.2,mid[1]-1.2,mid[1]+1.2);lookat=np.r_[mid,reference['log']['qpos'][0,2]];camera=viewer._camera(lookat,112.,-27.,1.9)
    error_high=max(25.,np.ceil(max(r['log']['timed_error_mm'].max()for r in allruns.values())/25)*25);lean_high=max(2.,np.ceil(max(r['log']['reference_lean_deg'].max()for r in allruns.values())/2)*2)
    source=Path(__file__).resolve();inputs=[source,PAINT,VIEWER,HERE/'presentation_plan.json',FAMILY/'protocol.json',FAMILY/'campaign/resolved_case_paths.json',stage/'report.json',stage/'artifact_manifest.json',FAMILY/'review/native_qa/final_stage01/report.json']
    for run in allruns.values():inputs += [run['folder']/name for name in ('artifact_manifest.json','precision.npz','rollout.npz','source_snapshot/config/model.yaml')]
    hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs};snap=HERE/'source_snapshot';snap.mkdir(exist_ok=True)
    for path in (source,PAINT,VIEWER):
        dest=snap/f'{path.stem}_{sha(path)[:12]}.py'
        if not dest.exists():shutil.copy2(path,dest)
    prefix='preview'if args.preview else'video';output=HERE/'scheduling_joint_tracking_overview.mp4';metadata=HERE/f'{prefix}_metadata.json'
    if metadata.exists()or(not args.preview and output.exists()):raise FileExistsError('Preserve previous render stage')
    writer=None;count=0;schedule=[]
    try:
        first,last=intro(paint),finalcard(paint,report);imageio.imwrite(HERE/f'{prefix}_intro.jpg',first,quality=95);imageio.imwrite(HERE/f'{prefix}_final.jpg',last,quality=95)
        if not args.preview:
            writer=imageio.get_writer(output,fps=FPS,codec='libx264',quality=8,macro_block_size=2,ffmpeg_log_level='error')
            for _ in range(40):writer.append_data(first);count+=1
        for chapter,phase in enumerate(PHASES):
            runs=[allruns[phase+'_'+v]for v in VARIANTS];plants=[];renderers=[];charts=[chart(paint,runs,'timed_error_mm','Original timed XYerror(mm), common scale',error_high),chart(paint,runs,'reference_lean_deg','Reference-relative lean(deg), common scale',lean_high)]
            try:
                for run in runs:
                    config=viewer.yaml.safe_load((run['folder']/'source_snapshot/config/model.yaml').read_text());plant=viewer.replay_plant(config,viewer.MarsYard(),terrainbounds,HERE/'_render_scenes'/run['case']);plants.append(plant);renderers.append(mujoco.Renderer(plant.model,height=400,width=900))
                moments=(0.,18.,32.,44.)if args.preview else np.arange(221)*.2
                for moment in moments:
                    planned_frame=40+chapter*251+int(round(moment/.2));pixels=frame(paint,viewer,runs,plants,renderers,charts,phase,report['per_phase'][phase]['phase_rad'],float(moment),planned_frame/FPS,camera,bounds,planned)
                    if writer is not None:assert count==planned_frame;writer.append_data(pixels)
                    if args.preview:imageio.imwrite(HERE/f'{prefix}_{phase}_{moment:.1f}s.jpg',pixels,quality=95)
                    schedule.append(dict(frame=planned_frame,playback_time_s=planned_frame/FPS,phase_key=phase,recorded_time_s=float(moment),case_state_times_s={r['case']:float(r['log']['time_s'][viewer.nearest(r['log']['time_s'],moment)])for r in runs}));count+=1
                card=phasecard(paint,report,phase);imageio.imwrite(HERE/f'{prefix}_{phase}_result.jpg',card,quality=95)
                if writer is not None:
                    for _ in range(30):writer.append_data(card);count+=1
            finally:
                for renderer in renderers:renderer.close()
            print(json.dumps(dict(chapter=phase,rendered=True,preview=args.preview)),flush=True)
        if writer is not None:
            for _ in range(80):writer.append_data(last);count+=1
    finally:
        if writer is not None:writer.close()
    assert hashes=={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    meta=dict(iteration='I-52',scope=__doc__,input_sha256=hashes,frames=count,fps=FPS,dimensions=[WIDTH,HEIGHT],phase_chapters=list(PHASES),all_twelve_cases=sorted(allruns),recorded_window_per_chapter_s=[0,44],playback_speed=4.,overview_pose_interval_s=.2,
        shared_camera=dict(lookat=lookat.tolist(),distance_m=1.9,azimuth_deg=112,elevation_deg=-27,world_xy_bounds=list(bounds)),shared_plot_limits=dict(timed_error_mm=[0,error_high],reference_lean_deg=[0,lean_high]),
        no_new_physics=True,no_controller_observer_forecast=True,no_scientific_reduction=True,default_changed=False,explicit_resolver_used=True)
    if not args.preview:
        with imageio.get_reader(output)as movie:
            assert movie.count_frames()==count==873
            for i in (0,40,291,542,872):imageio.imwrite(HERE/f'decoded_{i:04d}.jpg',movie.get_data(i),quality=95)
            info=movie.get_meta_data()
        meta['video']=dict(path=str(output.relative_to(ROOT)),sha256=sha(output),bytes=output.stat().st_size,frames=count,fps=FPS,duration_s=info['duration'])
    metadata.write_text(json.dumps(meta,indent=2)+'\n');(HERE/f'{prefix}_display_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n');print(json.dumps({k:v for k,v in meta.items()if k!='input_sha256'},indent=2))

if __name__=='__main__':main()
