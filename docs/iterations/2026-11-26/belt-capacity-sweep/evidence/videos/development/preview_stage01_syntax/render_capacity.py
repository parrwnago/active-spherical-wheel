"""Saved native I49 belt-capacity sweep: same design, three operating limits.

Only saved states, recorded commands and canonical analysis are displayed.
MuJoCo forward kinematics render the states; no controller or physics is run.
"""
from pathlib import Path
from functools import lru_cache
import argparse
import hashlib
import json
import shutil
import types

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
VIEWER=ROOT/'results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py'
CASES=('mars_cap030','mars_cap060','mars_cap090')
CAPS=(.3,.6,.9)
WIDTH,HEIGHT,FPS=1800,1200,20
BG,FG,WARN=(10,20,31),(222,234,245),(246,193,108)
COLORS=((92,175,224),(111,207,171),(224,164,91))


def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@lru_cache(maxsize=24)
def font(size):return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',size)


def text(draw,xy,value,size=20,color=FG):draw.text(xy,str(value),font=font(size),fill=color)


def wrap(draw,xy,value,width,size=20,color=FG):
    y,line=xy[1],''
    for word in str(value).split():
        candidate=(line+' '+word).strip()
        if line and draw.textlength(candidate,font=font(size))>width:
            text(draw,(xy[0],y),line,size,color);y+=size+7;line=word
        else:line=candidate
    if line:text(draw,(xy[0],y),line,size,color)
    return y+size+7


def num(value,scale=1.,digits=2):return 'n/a' if value is None else f'{value*scale:.{digits}f}'


def load_case(case):
    folder=FAMILY/'campaign/runs'/case
    if not(folder/'artifact_manifest.json').is_file():raise FileNotFoundError('Wait for all sealed raw cases')
    for row in read(folder/'artifact_manifest.json')['files']:
        assert sha(folder/row['path'])==row['sha256'],row['path']
    with np.load(folder/'precision.npz')as z:exact={k:z[k].copy()for k in z.files}
    with np.load(folder/'rollout.npz')as z:log={k:z[k].copy()for k in z.files}
    log.update(time_s=exact['time_s'],qpos=exact['exact_qpos'],qvel=exact['exact_qvel'],input=exact['exact_applied'],reference_xy=exact['exact_reference_xy'])
    log['planned_xy']=log['reference_xy'][::max(1,len(log['time_s'])//200)]
    log['timed_error_mm']=1000*np.linalg.norm(log['qpos'][:,:2]-log['reference_xy'],axis=1)
    q=log['qpos'][:,3:7];log['absolute_tilt_deg']=np.degrees(np.arccos(np.clip(1-2*np.sum(q[:,1:3]**2,axis=1)/np.sum(q*q,axis=1),-1.,1.)))
    return dict(case=case,folder=folder,exact=exact,log=log,summary=read(folder/'summary.json'),provenance=read(folder/'provenance.json'))


def inset(pixels,run,index,bounds,planned):
    picture=Image.fromarray(pixels);draw=ImageDraw.Draw(picture,'RGBA')
    left,top,size=405,235,180;x0,x1,y0,y1=bounds
    draw.rectangle((left,top,left+size,top+size),fill=(7,16,25,230))
    def point(xy):return left+10+(xy[0]-x0)/(x1-x0)*(size-20),top+20+(y1-xy[1])/(y1-y0)*(size-40)
    for values,color in [(planned,(244,174,75,255)),(run['log']['qpos'][:index+1,:2],(100,216,232,255))]:
        points=[point(p)for p in values[::max(1,len(values)//250)]]
        if len(points)>1:draw.line(points,fill=color,width=2)
    x,y=point(run['log']['qpos'][index,:2]);draw.ellipse((x-3,y-3,x+3,y+3),fill=(100,216,232,255))
    text(draw,(left+8,top+3),'Fixed world XY',13)
    text(draw,(left+8,top+size-18),'Orange plan / cyan robot',11)
    return np.asarray(picture)


def chart_base(times,series,title,ylim,lines=()):
    picture=Image.new('RGB',(600,190),BG);draw=ImageDraw.Draw(picture)
    left,top,w,h=48,37,525,120;lo,hi=ylim
    draw.rectangle((left,top,left+w,top+h),fill=(4,12,20),outline=(60,81,98))
    text(draw,(18,4),title,18)
    for value in (lo,hi):text(draw,(5,top+(0 if value==hi else h-14)),f'{value:g}',12,(157,181,199))
    for value,color,label in lines:
        y=top+h-(value-lo)/(hi-lo)*h
        if top<=y<=top+h:
            for x in range(left,left+w,10):draw.line((x,y,min(x+5,left+w),y),fill=color,width=1)
            text(draw,(left+4,y-15),label,12,color)
    for values,color in series:
        points=[(left+t/44*w,top+h-(value-lo)/(hi-lo)*h)for t,value in zip(times,values)if np.isfinite(value)]
        if len(points)>1:draw.line(points,fill=color,width=2)
    for instant in (0,11,22,33,44):text(draw,(left+instant/44*w-7,top+h+5),str(instant),12)
    return picture


def cursor(base,moment):
    result=base.copy();draw=ImageDraw.Draw(result);x=48+np.clip(moment,0,44)/44*525
    draw.line((x,37,x,157),fill=(249,248,239),width=2)
    return result


def intro():
    picture=Image.new('RGB',(WIDTH,HEIGHT),BG);draw=ImageDraw.Draw(picture)
    text(draw,(65,55),'I49: higher assumed belt capacity, with the original gain design',40)
    y=165
    for message,color in [
        ('Three fresh native Mars figure-eight runs: both belt limits 0.30 / 0.60 / 0.90 m/s.',FG),
        ('The gain library and ordinary design remain normalized for the original 0.30 m/s setting.',FG),
        ('Shaft/shell torque limits stay 1.5 / 0.6 / 0.6 Nm. Same path, phase +0.90, initial yaw 45°, observer and contact model.',FG),
        ('Only physical and measured-state mirror operating belt bounds change. Incoming-belt preparation uses those same bounds.',WARN),
        ('The full recorded 0–44s program plays at 2×. All columns use the same fixed camera and world-XY scale.',FG),
        ('State-dependent gain scheduling still follows each run; the gain design is unchanged.',FG),
        ('This changes an ideal surface-velocity capacity assumption. It does not model an actual motor’s torque-speed/current/thermal response.',WARN),
        ('Higher capacity is an experimental assumption, not a recommended motor rating or a repository default change.',WARN),
    ]:
        y=wrap(draw,(75,y),message,1635,29,color)+35
    return np.asarray(picture)


def frame(viewer,runs,plants,renderers,charts,moment,camera,xybounds,planned):
    picture=Image.new('RGB',(WIDTH,HEIGHT),BG);draw=ImageDraw.Draw(picture)
    text(draw,(27,12),f'I49 IDEAL BELT-CAPACITY SWEEP | same original 0.30 design | recorded program t={moment:05.2f}s | playback 2×',28)
    for column,(run,cap)in enumerate(zip(runs,CAPS)):
        left=600*column;log=run['log'];end=float(log['time_s'][-1]);index=viewer.nearest(log['time_s'],min(moment,end))
        stopped=moment>end+1e-9
        option=mujoco.MjvOption();option.geomgroup[3:]=0
        pixels=viewer._draw(plants[column],renderers[column],log,index,camera,option,False)
        pixels=inset(pixels,run,index,xybounds,planned)
        picture.paste(Image.fromarray(pixels),(left,135));draw=ImageDraw.Draw(picture)
        text(draw,(left+18,66),f'Both belts: ±{cap:.2f} m/s',27,COLORS[column])
        text(draw,(left+18,105),'Actual saved native Gazebo/DART motion',16)
        requested=np.max(abs(run['exact']['exact_native_requested'][index,3:]))
        applied=np.max(abs(run['exact']['exact_applied'][index,3:]))
        text(draw,(left+18,585),f'Recorded t {log["time_s"][index]:.3f}s | timed error {log["timed_error_mm"][index]:.1f}mm',18)
        text(draw,(left+18,615),f'Absolute platform tilt {log["absolute_tilt_deg"][index]:.2f}°',19)
        text(draw,(left+18,645),f'Max|beltA/B| request/applied {requested:.3f}/{applied:.3f}m/s',17)
        if stopped:text(draw,(left+18,675),f'RUN ENDED at {end:.3f}s; image held, no further states',17,WARN)
        else:text(draw,(left+18,675),'Original timed reference; no altered goal or speed target',16)
        picture.paste(cursor(charts[column][0],moment),(left,725))
        picture.paste(cursor(charts[column][1],moment),(left,930))
    draw=ImageDraw.Draw(picture)
    text(draw,(25,1150),'Belt plots: orange requested / cyan applied; dotted gray=old 0.30 cap, green=own cap. Shared axes across all arms.',20)
    text(draw,(25,1180),'Higher ideal capacity does not model actual motor response. Saved states are re-rendered; no new physics/controller execution.',18,WARN)
    return np.asarray(picture)


def resultcard(report,motion_equal):
    picture=Image.new('RGB',(WIDTH,HEIGHT),BG);draw=ImageDraw.Draw(picture)
    text(draw,(55,35),'I49 result: higher capacity changes the tradeoff; no setting is adopted',37,WARN)
    text(draw,(55,100),'All three completed 44s. Fixed original design/gain library; higher operating limits include preparation clipping.',25)
    headings=[('Both-belt cap',55),('Moving RMS',400),('Peak / final error',710),('Peak ref. lean',1160),('Settled / route gates',1460)]
    for label,x in headings:text(draw,(x,205),label,23,(157,196,219))
    for index,(case,cap)in enumerate(zip(CASES,CAPS)):
        d=report['cases'][case];t=d['timed_tracking'];y=275+index*103
        text(draw,(55,y),f'±{cap:.2f} m/s',29,COLORS[index])
        text(draw,(400,y),f'{1000*t["moving_rms_m"]:.2f} mm',27)
        text(draw,(710,y),f'{1000*t["full_peak_m"]:.2f} / {1000*t["final_m"]:.2f} mm',27)
        text(draw,(1160,y),f'{d["attitude"]["peak_reference_error_deg"]:.2f}°',27)
        text(draw,(1460,y),f'{d["original_settling"]["passed"]} / {d["original_route_gates"]["passed"]}',25)
    base=report['cases'][CASES[0]];raised=report['cases'][CASES[1]]
    percent=100*(raised['timed_tracking']['moving_rms_m']/base['timed_tracking']['moving_rms_m']-1)
    text(draw,(55,625),f'Higher caps improve peak/final error and peak reference-relative lean, but moving RMS worsens {percent:.2f}%.',26,WARN)
    text(draw,(55,677),'Both raised-cap candidates fail the registered screen: settling/route gates and selected turn metrics regress.',25,WARN)
    text(draw,(55,747),f'0.60 / 0.90 recorded body motion identical: {motion_equal}. Applied belt commands differ; identical motion does not mean unused capacity.',24)
    for index,case in enumerate(CASES):
        d=report['cases'][case]['belts'];a=d['belt_A']['max_applied_abs_m_s'];b=d['belt_B']['max_applied_abs_m_s']
        text(draw,(55,800+index*40),f'Cap {CAPS[index]:.2f}: max applied belt A / B = {a:.3f} / {b:.3f} m/s',23)
    wrap(draw,(55,970),'One deterministic Mars route and one initial phase. The ideal surface-speed servo does not model actual motor response, torque-speed capability, current, power or thermal limits.',1690,25)
    text(draw,(55,1110),'Repository defaults remain unchanged. This is a simulation capacity experiment, not motor validation or a general controller result.',24,WARN)
    return np.asarray(picture)


def scientific_plots(report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=FAMILY/'plots'
    if list(folder.glob('*.png')):raise FileExistsError('Preserve existing scientific figures before rerendering')
    fig,axes=plt.subplots(2,2,figsize=(11.5,8),constrained_layout=True)
    specs=[('moving_rms_m','Moving timed RMS','mm',1000),('full_peak_m','Peak timed error','mm',1000),
           ('final_m','Final timed error','mm',1000),('peak_reference_error_deg','Peak reference-relative lean','deg',1)]
    for ax,(key,title,unit,scale)in zip(axes.flat,specs):
        values=[report['cases'][case]['attitude' if key=='peak_reference_error_deg' else 'timed_tracking'][key]*scale for case in CASES]
        bars=ax.bar(range(3),values,color=[np.array(c)/255 for c in COLORS])
        for bar,value in zip(bars,values):ax.annotate(f'{value:.2f}',(bar.get_x()+bar.get_width()/2,bar.get_height()),xytext=(0,4),textcoords='offset points',ha='center',fontsize=10)
        ax.set(title=title,ylabel=unit,xticks=range(3),xticklabels=['0.30','0.60','0.90'],xlabel='Operating belt cap (m/s)');ax.grid(axis='y',alpha=.2);ax.margins(y=.15)
    fig.suptitle('I49: unchanged original0.30 design; all three44s runs complete\nPeak/final improve, moving RMS worsens; raised caps fail original settling/route/turn screen',fontsize=13)
    for ext in ('png','pdf'):fig.savefig(folder/f'capacity_tracking.{ext}',dpi=190)
    plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4.8),constrained_layout=True);x=np.arange(6)
    for index,(case,cap)in enumerate(zip(CASES,CAPS)):
        turns=report['cases'][case]['turns'];color=np.array(COLORS[index])/255
        axes[0].bar(x+(index-1)*.25,[1000*r['geometric_component_overshoot_m'] for r in turns],width=.25,color=color,label=f'{cap:.2f} m/s')
        axes[1].bar(x+(index-1)*.25,[r['actual_reversal_lag_s'] if r['actual_reversal_lag_s'] is not None else np.nan for r in turns],width=.25,color=color,label=f'{cap:.2f} m/s')
    axes[0].set(title='Fixed six component-reversal events',ylabel='Geometric component overshoot (mm)')
    axes[1].set(title='Signed reversal lag; negative = early',ylabel='Time (s)');axes[1].axhline(0,color='black',lw=.8)
    for ax in axes:ax.set(xticks=x,xticklabels=['1 y','2 x','3 y','4 y','5 x','6 y']);ax.grid(axis='y',alpha=.2);ax.legend(fontsize=9)
    fig.suptitle('Higher capacity does not uniformly improve turning\nSame original timed reference and fixed gain library; one saved run per cap',fontsize=13)
    for ext in ('png','pdf'):fig.savefig(folder/f'capacity_turns.{ext}',dpi=190)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',action='store_true');parser.add_argument('--analysis-stage',default='stage01')
    parser.add_argument('--plots',action='store_true')
    args=parser.parse_args()
    stage=FAMILY/'analysis'/args.analysis_stage
    if not(stage/'artifact_manifest.json').is_file():raise FileNotFoundError('Wait for sealed canonical analysis')
    manifest=read(stage/'artifact_manifest.json')
    entries=manifest['files']
    for row in entries:assert sha(stage/row['path'])==row['sha256'],row['path']
    report=read(stage/'report.json');assert all(report['integrity']['baseline17_fields_byte_exact'].values())
    runs=[load_case(case)for case in CASES]
    viewer=types.ModuleType('_I49_saved_native_viewer');viewer.__file__=str(VIEWER)
    exec(compile(VIEWER.read_text(),str(VIEWER),'exec'),viewer.__dict__);viewer.HERE=HERE
    protocol=read(FAMILY/'protocol.json')
    with np.load(ROOT/protocol['inputs']['historical_precision'])as z:planned=z['exact_reference_xy'][::40].copy()
    middle=.5*(planned.min(axis=0)+planned.max(axis=0));span=max(float(np.ptp(planned,axis=0).max())+.65,1.4)
    xybounds=(middle[0]-span/2,middle[0]+span/2,middle[1]-span/2,middle[1]+span/2)
    camera=viewer._camera(np.r_[middle,runs[0]['log']['qpos'][0,2]],112.,-27.,1.9)
    terrainbounds=(middle[0]-1.2,middle[0]+1.2,middle[1]-1.2,middle[1]+1.2)
    source=Path(__file__).resolve();snapshot=HERE/'source_snapshot';snapshot.mkdir(exist_ok=True)
    for p in (source,VIEWER):
        dest=snapshot/f'{p.stem}_{sha(p)[:12]}.py'
        if not dest.exists():shutil.copy2(p,dest)
    prefix='preview'if args.preview else'video';metadata=HERE/f'{prefix}_metadata.json';output=HERE/'belt_capacity_comparison.mp4'
    if metadata.exists()or(not args.preview and output.exists()):raise FileExistsError('Preserve previous media attempt')
    inputs=[source,VIEWER,FAMILY/'protocol.json',stage/'report.json',stage/'artifact_manifest.json',stage/'case_metrics.csv',stage/'turn_metrics.csv',ROOT/protocol['inputs']['historical_precision']]
    for run in runs:inputs += [run['folder']/name for name in ('artifact_manifest.json','precision.npz','rollout.npz','summary.json','provenance.json','terrain.json','source_snapshot/config/model.yaml')]
    hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    motion_equal=np.array_equal(runs[1]['exact']['exact_qpos'],runs[2]['exact']['exact_qpos'])and np.array_equal(runs[1]['exact']['exact_qvel'],runs[2]['exact']['exact_qvel'])
    applied_equal=np.array_equal(runs[1]['exact']['exact_applied'],runs[2]['exact']['exact_applied'])
    if args.plots:scientific_plots(report)
    error_high=np.ceil(max(float(run['log']['timed_error_mm'].max())for run in runs)/25)*25
    belt_high=np.ceil(max(float(abs(run['exact']['exact_native_requested'][:,3:]).max())for run in runs)*10)/10
    charts=[]
    for run,cap in zip(runs,CAPS):
        t=run['log']['time_s'];d=run['exact']
        charts.append((chart_base(t,[(run['log']['timed_error_mm'],(238,180,92))],'Original timed-path error (mm)',(0,error_high)),
                       chart_base(t,[(np.max(abs(d['exact_native_requested'][:,3:]),axis=1),(238,180,92)),(np.max(abs(d['exact_applied'][:,3:]),axis=1),(104,210,230))],
                                  'Max |belt A/B| speed (m/s)',(0,belt_high),lines=[(.3,(164,178,191),'old0.30'),(cap,(137,223,176),f'own{cap:.2f}')]))))
    plants,renderers=[],[];writer=None;count=0;schedule=[]
    try:
        for run in runs:
            config=viewer.yaml.safe_load((run['folder']/'source_snapshot/config/model.yaml').read_text())
            plant=viewer.replay_plant(config,viewer.MarsYard(),terrainbounds,HERE/'_render_scenes'/run['case'])
            plants.append(plant);renderers.append(mujoco.Renderer(plant.model,height=430,width=600))
        first,last=intro(),resultcard(report,motion_equal)
        imageio.imwrite(HERE/f'{prefix}_intro.jpg',first,quality=94);imageio.imwrite(HERE/f'{prefix}_resultcard.jpg',last,quality=94)
        if not args.preview:
            writer=imageio.get_writer(output,fps=FPS,codec='libx264',quality=8,macro_block_size=2,ffmpeg_log_level='error')
            for _ in range(2*FPS):writer.append_data(first);count+=1
        moments=[0.,6.825,12.,31.65,44.]if args.preview else np.arange(0.,44.+1e-9,.1)
        for moment in moments:
            pixels=frame(viewer,runs,plants,renderers,charts,float(moment),camera,xybounds,planned)
            if writer is not None:writer.append_data(pixels)
            if args.preview or abs(moment-12.)<1e-9 or abs(moment-44.)<1e-9:imageio.imwrite(HERE/f'{prefix}_native_{moment:.3f}s.jpg',pixels,quality=94)
            schedule.append(dict(frame=count,display_time_s=float(moment),case_state_times_s={run['case']:float(run['log']['time_s'][viewer.nearest(run['log']['time_s'],min(moment,run['log']['time_s'][-1]))])for run in runs}))
            count+=1
        if writer is not None:
            for _ in range(4*FPS):writer.append_data(last);count+=1
    finally:
        if writer is not None:writer.close()
        for renderer in renderers:renderer.close()
    assert hashes=={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    info=dict(iteration='I-49',scope=__doc__,source_sha256=sha(source),input_sha256=hashes,frames=count,fps=FPS,dimensions=[WIDTH,HEIGHT],
        actual_window_s=[0.,44.],playback_speed=2.,intro_seconds=2.,resultcard_seconds=4.,
        shared_camera=dict(lookat=np.r_[middle,runs[0]['log']['qpos'][0,2]].tolist(),distance_m=1.9,azimuth_deg=112,elevation_deg=-27,world_xy_bounds_m=list(xybounds)),
        gain_library_and_design_unchanged=True,operating_caps_m_s=CAPS,capacity_scope='Final belt bounds and incoming belt preparation, not a final-clip-only ablation.',
        motion_060_090_identical=bool(motion_equal),applied_060_090_identical=bool(applied_equal),
        simulation_or_controller_execution=False,scientific_reduction_recomputed=False,default_changed=False,
        meaning='Ideal surface-speed capacity experiment; no actual motor-response or motor-rating validation.')
    if not args.preview:
        with imageio.get_reader(output)as movie:
            decoded=movie.count_frames();meta=movie.get_meta_data()
            for index in (0,40,count//2,count-1):imageio.imwrite(HERE/f'decoded_frame_{index:04d}.jpg',movie.get_data(index),quality=94)
        assert decoded==count and tuple(meta['size'])==(WIDTH,HEIGHT)and meta['fps']==FPS
        info['video']=dict(path=str(output.relative_to(ROOT)),sha256=sha(output),bytes=output.stat().st_size,frames=decoded,fps=FPS,duration_s=meta['duration'])
    metadata.write_text(json.dumps(info,indent=2)+'\n');(HERE/f'{prefix}_display_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
    print(json.dumps({k:v for k,v in info.items()if k!='input_sha256'},indent=2))


if __name__=='__main__':main()
