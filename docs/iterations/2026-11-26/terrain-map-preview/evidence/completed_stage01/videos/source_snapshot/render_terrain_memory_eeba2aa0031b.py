"""I44 saved robot motion, ideal depth and causal map/prediction presentation.

No depth is rendered, map recomputed, controller called or dynamics integrated.
Only the actual saved robot state is re-rendered for visual context. Map and
depth displays use completed, past-only anchor snapshots from the campaign.
"""
from pathlib import Path
import argparse
import hashlib
import json
import types

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
I40=ROOT/'results/lqr_contact_forecast_20261125'
VIEWER=I40/'videos/render_contact_forecasts.py'
ARMS=('observer_plane','latest_four','memory_four')
NAMES=('Current observer tangent','Latest delivered four views','Accumulated four-view memory')
WIDTH,HEIGHT,FPS=800,600,20
CHAPTERS=({'id':'early_bump','start_s':6.4,'stop_s':10.,'speed':.5},
          {'id':'late_revisit','start_s':30.,'stop_s':36.,'speed':1.})
CAMCOLORS=((247,175,78),(114,196,246),(204,146,242),(120,225,169))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())


def palette(values):
    """Fixed blue/cyan/yellow palette; display only, no geometry interpolation."""
    x=np.clip(np.asarray(values,float),0.,1.)
    return np.stack((np.clip(3*x-1.,0.,1.),np.clip(1.5-abs(3*x-1.5),0.,1.),
                     np.clip(2.-3*x,0.,1.)),axis=-1)*255


def text(draw,position,value,font,size=14,color=(207,227,242)):
    draw.text(position,str(value),font=font(size),fill=color)


def depth_panel(depth,batch,anchor,font):
    picture=Image.new('RGB',(WIDTH,HEIGHT),(9,19,29));draw=ImageDraw.Draw(picture)
    text(draw,(17,10),'FOUR IDEAL DEPTH VIEWS: axial optical Z',font,21)
    capture=float(depth['capture_time_s'][batch]);delivery=float(depth['delivered_at_s'][batch])
    text(draw,(17,42),f'Capture {capture:.3f}s | delivered {delivery:.3f}s | anchor {anchor:.3f}s',font,15)
    text(draw,(17,68),'20Hz capture / 50ms latency / exact pose / no physical sensor noise',font,13)
    for camera in range(4):
        values=depth['depth_m'][batch,camera];valid=depth['valid_mask'][batch,camera]
        colors=palette(np.nan_to_num(values,nan=0.)/2.).astype(np.uint8);colors[~valid]=0
        tile=Image.fromarray(colors).resize((288,216),Image.Resampling.NEAREST)
        left,top=50+(camera%2)*400,113+(camera//2)*241
        picture.paste(tile,(left,top));draw=ImageDraw.Draw(picture)
        text(draw,(left,top-22),str(depth['camera_names'][camera]),font,15,CAMCOLORS[camera])
    text(draw,(17,574),'Black = invalid, self-occluded, missed or outside radial range. Color: axial Z 0–2m.',font,12)
    return np.asarray(picture)


def map_base(memory,index,bounds,height_origin,font):
    image=Image.new('RGB',(WIDTH,HEIGHT),(9,19,29));draw=ImageDraw.Draw(image)
    text(draw,(17,10),'OBSERVED CELLS ONLY: accumulated past observations',font,19)
    now=float(memory['anchor_time_s'][index])
    text(draw,(17,40),f'Frozen map knowledge at anchor {now:.3f}s; unknown cells stay black',font,14)
    left,top,w,h=175,80,450,450
    image_array=np.asarray(image).copy();image_array[top:top+h,left:left+w]=(1,5,9)
    lo,hi=int(memory['offsets'][index]),int(memory['offsets'][index+1])
    points=memory['centers_m'][lo:hi];spread=memory['height_spread_m'][lo:hi]
    x0,x1,y0,y1=bounds
    keep=(points[:,0]>=x0)&(points[:,0]<=x1)&(points[:,1]>=y0)&(points[:,1]<=y1)
    points,spread=points[keep],spread[keep]
    if len(points):
        cols=np.clip(((points[:,0]-x0)/(x1-x0)*(w-1)).astype(int),0,w-1)
        rows=np.clip(((y1-points[:,1])/(y1-y0)*(h-1)).astype(int),0,h-1)
        colors=palette((points[:,2]-height_origin+.3)/.6).astype(np.uint8)
        colors[spread>.02]=(239,104,94)
        image_array[top+rows,left+cols]=colors
    image=Image.fromarray(image_array);draw=ImageDraw.Draw(image)
    text(draw,(17,546),'Cell color = observed height relative to initial centre−radius, fixed ±0.30m.',font,12)
    text(draw,(17,570),'Red = multiple-height cell; observations do not guarantee a supported query.',font,12)
    return image


def map_panel(base,anchor_record,predictions,bounds,font):
    image=base.copy();draw=ImageDraw.Draw(image,'RGBA');x0,x1,y0,y1=bounds
    def pixel(xy):return 175+(float(xy[0])-x0)/(x1-x0)*449,80+(y1-float(xy[1]))/(y1-y0)*449
    body=np.asarray(anchor_record['exact_qpos'])[:2];x,y=pixel(body)
    draw.ellipse((x-6,y-6,x+6,y+6),outline=(245,249,251,255),width=2)
    cv=np.asarray(predictions['observer_plane']['shared_cv_qpos'])[:2];x,y=pixel(cv)
    draw.line([(x-6,y-6),(x+6,y+6)],fill=(224,154,243,255),width=2)
    draw.line([(x-6,y+6),(x+6,y-6)],fill=(224,154,243,255),width=2)
    for arm,color in zip(ARMS,((177,186,197),(112,190,244),(86,224,188))):
        for candidate in predictions[arm]['contact_preview'].get('candidates',[]):
            x,y=pixel(candidate['point_m'][:2]);draw.ellipse((x-3,y-3,x+3,y+3),outline=color+(255,),width=1)
    text(draw,(52,64),'White circle: anchor body | purple X: shared +200ms CV centre',font,11)
    text(draw,(52,532),'Small rings: possible geometric contact points (observer/latest/memory)',font,11)
    return np.asarray(image)


def format_error(value,scale=1.,digits=2):
    return 'n/a' if value is None else f'{value*scale:.{digits}f}'


def prediction_panel(rows,anchor,font):
    image=Image.new('RGB',(WIDTH,HEIGHT),(9,19,29));draw=ImageDraw.Draw(image)
    text(draw,(17,10),'SHADOW GEOMETRY: same +200ms CV pose for all arms',font,19)
    text(draw,(17,42),f'Knowledge frozen {anchor:.3f}s | queried endpoint {anchor+.2:.3f}s',font,15)
    text(draw,(17,71),'Errors use private evaluator geometry/future contacts; not mapper inputs.',font,13,(243,193,111))
    y=115
    for arm,label in zip(ARMS,NAMES):
        row=rows[arm];score=row['surface_score'];query=row['surface_query']
        text(draw,(20,y),label,font,18)
        status='SUPPORTED'if score['supported']else'UNKNOWN'
        text(draw,(28,y+28),f'{status} | {score.get("reason")}',font,13)
        normal='ambiguous'if score['normal_ambiguous']else format_error(score['normal_error_deg'])
        text(draw,(28,y+51),f'EVAL height error (mm) {format_error(score["height_error_m"],1000)} | normal (deg) {normal}',font,14)
        patches=', '.join(row['candidate_patches'])or'none'
        complete=row['contact_preview'].get('contact_prediction_supported',False)
        text(draw,(28,y+75),f'Possible patches: {patches} | {"complete"if complete else"unknown/partial"}',font,13)
        y+=119
    reference=rows['observer_plane']
    text(draw,(20,486),f'Shared CV motion error: {1000*reference["cv_motion_position_error_m"]:.2f}mm (evaluation only)',font,14)
    text(draw,(20,513),'Candidate points/patches are geometry; no predicted forces, load shares or slip.',font,13)
    text(draw,(20,541),'Native LQR motion is unchanged. No sensor/map forecast commands the robot.',font,13,(243,193,111))
    text(draw,(20,572),'Snapshots are held between the119 stored evaluation anchors.',font,12)
    return np.asarray(image)


def endcard(report,font):
    image=Image.new('RGB',(WIDTH*2,HEIGHT*2),(10,20,31));draw=ImageDraw.Draw(image)
    text(draw,(35,34),'I44: four-camera terrain memory — offline shadow evaluation',font,35)
    text(draw,(35,95),'Actual robot states and commands are the original44s native LQR trace; tracking cannot improve in replay.',font,23)
    text(draw,(35,139),'Ideal depth, exact calibration/poses,20Hz,50ms latency. No SLAM, noise or hardware validation.',font,23)
    text(draw,(35,195),'Primary stratum:109 uniform anchors ×3 horizons =327 nested endpoint queries per arm.',font,23,(244,194,108))
    y=280
    for arm,label in zip(ARMS,NAMES):
        stats=report['summary']['uniform'][arm]['all_horizons']
        text(draw,(40,y),label,font,29)
        text(draw,(60,y+45),f'CV-centre surface support {100*stats["coverage"]:.1f}% ({stats["supported"]}/{stats["query_slots"]})',font,24)
        text(draw,(60,y+85),f'Covered height RMSE {format_error(stats["height_error_m"]["rmse"],1000)}mm (N{stats["height_error_m"]["n"]}) | geometric-normal P90 {format_error(stats["normal_error_deg"]["p90_abs"])}deg (N{stats["normal_error_deg"]["n"]})',font,22)
        y+=165
    text(draw,(35,y+15),f'Predeclared UNIFORM geometry-utility screen: {"PASS"if report["utility_passed"]else"FAIL"}. This is not a contact/control success gate.',font,24,(244,194,108))
    text(draw,(35,y+70),'Geometric normals are mesh-face proxies, not force-weighted physical contact normals.',font,23)
    text(draw,(35,y+115),'One reused trajectory, no new physics or controller commands. Unknown/ambiguous regions remain explicit.',font,23)
    text(draw,(35,y+160),'Possible shell/belt geometry does not establish load-transfer prediction or closed-loop benefit.',font,23)
    dense=report['summary']['dense_bump']['memory_four']['all_horizons']
    text(draw,(35,y+205),f'Early dense bump: memory surface {dense["supported"]}/{dense["query_slots"]}; contact candidate {dense["contact_candidate_available"]}/{dense["query_slots"]}. Blindness remains.',font,24,(244,194,108))
    text(draw,(35,y+250),'Latest views are blind at these underbody query locations; the four depth images still see other terrain.',font,22)
    return np.asarray(image)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',action='store_true')
    parser.add_argument('--analysis-stage',default='analyze_stage01')
    parser.add_argument('--depth-stage',default='depth_stage01')
    args=parser.parse_args()
    prefix='preview'if args.preview else'video';target=HERE/'terrain_memory_shadow.mp4';metadata=HERE/f'{prefix}_metadata.json'
    if target.exists()or metadata.exists():raise FileExistsError('Preserve prior rendering stage')
    analysis=FAMILY/'campaign'/args.analysis_stage;depth_stage=FAMILY/'campaign'/args.depth_stage
    sentinel=analysis/'artifact_manifest.json'
    required=[analysis/name for name in ('report.json','predictions.json','anchor_records.json','memory_snapshots.npz')]
    if not sentinel.is_file():raise FileNotFoundError('Wait for final analysis completion sentinel before opening NPZ')
    if not all(p.is_file()for p in required):raise FileNotFoundError('Wait for sealed complete analysis outputs')
    for name,entry in read(sentinel).items():
        assert sha(analysis/name)==entry['sha256'],name
    source=Path(__file__).resolve();snapshot=HERE/'source_snapshot';snapshot.mkdir(exist_ok=True)
    for p in (source,VIEWER):
        saved=snapshot/f'{p.stem}_{sha(p)[:12]}.py'
        if not saved.exists():saved.write_bytes(p.read_bytes())
        assert sha(saved)==sha(p)
    viewer=types.ModuleType('i44_actual_saved_state_viewer');viewer.__file__=str(VIEWER)
    exec(compile(VIEWER.read_text(),str(VIEWER),'exec'),viewer.__dict__)
    assert sha(VIEWER)==read(I40/'videos/overview_video_metadata.json')['render_source_sha256']
    viewer.HERE=HERE;run=viewer.load_case('mars_figure8_p090');log=run['log']
    report=read(analysis/'report.json');anchors=read(analysis/'anchor_records.json');rows=read(analysis/'predictions.json')
    table={}
    for row in rows:
        if abs(row['horizon_s']-.2)<1e-8:table.setdefault(row['anchor_index'],{})[row['arm']]=row
    with np.load(depth_stage/'depth_batches.npz')as z:depth={k:z[k].copy()for k in z.files if k!='render_wall_s'}
    with np.load(analysis/'memory_snapshots.npz')as z:memory={k:z[k].copy()for k in ('anchor_time_s','offsets','centers_m','height_spread_m')}
    anchor_times=np.array([row['anchor_time_s']for row in anchors])
    np.testing.assert_array_equal(anchor_times,memory['anchor_time_s'])
    paths=[source,VIEWER,FAMILY/'protocol.json',FAMILY/'protocol_clarifications.json',sentinel,*required,depth_stage/'depth_batches.npz',depth_stage/'report.json',depth_stage/'scene_metadata.json']
    paths += [run['folder']/name for name in ('precision.npz','rollout.npz','summary.json','provenance.json','terrain.json','source_snapshot/config/model.yaml')]
    provenance={'iteration':'I-44','scope':__doc__,'chapters':list(CHAPTERS),'fps':FPS,'dimensions':[1600,1200],
        'knowledge_display':'Map/prediction/depth held at latest stored anchor<=displaytime; depth is latest delivered batch at that anchor. Robot moves continuously through actual saved states.',
        'map_boundary':'Observed cell means only; no true terrain heatmap, future cells or future actual contact marker is inserted.',
        'input_sha256':{str(p.relative_to(ROOT)):sha(p)for p in paths}}
    (HERE/f'{prefix}_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    plant=viewer.scene(run,'_I44_actual_replay');saved_state=plant.capture();renderer=mujoco.Renderer(plant.model,height=HEIGHT,width=WIDTH)
    option=mujoco.MjvOption();option.geomgroup[3:]=0
    writer=None if args.preview else imageio.get_writer(target,fps=FPS,codec='libx264',quality=8,macro_block_size=2,ffmpeg_log_level='error')
    frame_count=0;view_records=[];cache={};height_origin=float(log['qpos'][0,2]-.12)
    try:
        for chapter in CHAPTERS:
            mask=(log['time_s']>=chapter['start_s'])&(log['time_s']<=chapter['stop_s'])
            positions=log['qpos'][mask,:3];center=.5*(positions.min(axis=0)+positions.max(axis=0));distance=max(1.3,2*float(np.max(positions.max(axis=0)-positions.min(axis=0))))
            xy=positions[:,:2];span=float(np.max(xy.max(axis=0)-xy.min(axis=0))+2.4);middle=.5*(xy.min(axis=0)+xy.max(axis=0))
            bounds=[float(middle[0]-span/2),float(middle[0]+span/2),float(middle[1]-span/2),float(middle[1]+span/2)]
            view_records.append(dict(chapter,lookat_xyz_m=center.tolist(),camera_distance_m=distance,map_bounds_xy_m=bounds))
            moments=[chapter['start_s'],chapter['stop_s']]if args.preview else chapter['start_s']+viewer.frame_times(chapter['stop_s']-chapter['start_s'],chapter['speed'])
            for moment in moments:
                index=viewer.nearest(log['time_s'],moment);ai=int(np.searchsorted(anchor_times,moment+1e-9,side='right')-1)
                if ai<0:raise RuntimeError('No prior stored anchor for this window')
                anchor=anchors[ai];now=float(anchor['anchor_time_s']);batch=int(anchor['latest_batch_index'])
                assert now<=moment+1e-9 and float(depth['delivered_at_s'][batch])<=now+1e-9
                selected=table[anchor['anchor_index']]
                frame=viewer._draw(plant,renderer,log,index,viewer._camera(center,112.,-25.,distance),option,False)
                frame=viewer._caption(frame,'ACTUAL SAVED ROBOT: native LQR + observer unchanged',[
                    f'Recorded t{log["time_s"][index]:.3f}s | playback{chapter["speed"]:g}x | {chapter["id"]}',
                    f'Timed tracking error {log["goal_error_mm"][index]:.2f}mm | platform tilt {log["absolute_tilt_deg"][index]:.2f}deg',
                    f'Sensor/map/prediction display frozen at previous stored anchor {now:.3f}s',
                    'Ideal sensing and memory are SHADOW ONLY; they never command this motion'],
                    'Saved Gazebo/DART states re-rendered for context. No new physics, controller call or tracking improvement.')
                key=(chapter['id'],ai)
                if key not in cache:
                    cache[key]=(depth_panel(depth,batch,now,viewer._font),map_panel(map_base(memory,ai,bounds,height_origin,viewer._font),anchor,selected,bounds,viewer._font),prediction_panel(selected,now,viewer._font))
                d,m,p=cache[key]
                combined=np.concatenate([np.concatenate([frame,d],axis=1),np.concatenate([m,p],axis=1)],axis=0)
                if writer is not None:writer.append_data(combined)
                frame_count+=1
                if args.preview or abs(moment-moments[-1])<1e-8:imageio.imwrite(HERE/f'{prefix}_{chapter["id"]}_{moment:.3f}s.jpg',combined,quality=92)
        if writer is not None:
            card=endcard(report,viewer._font)
            for _ in range(3*FPS):writer.append_data(card);frame_count+=1
            imageio.imwrite(HERE/'video_resultcard.jpg',card,quality=92)
    finally:
        if writer is not None:writer.close()
        renderer.close();plant.restore(saved_state)
    result=dict(provenance,frames=frame_count,chapter_views=view_records)
    if not args.preview:
        with imageio.get_reader(target)as video:info=video.get_meta_data();count=video.count_frames();last=video.get_data(count-1)
        assert count==frame_count and tuple(info['size'])==(1600,1200)and info['fps']==FPS
        imageio.imwrite(HERE/'video_decoded_last.jpg',last,quality=92)
        result['video']=dict(path=str(target.relative_to(ROOT)),sha256=sha(target),bytes=target.stat().st_size,frames=count,fps=FPS,dimensions=list(info['size']),duration_s=info['duration'],decoded_last=True)
    metadata.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items()if k!='input_sha256'},indent=2))


if __name__=='__main__':main()
