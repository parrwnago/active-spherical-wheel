"""Descriptive saved-data audit of I49 playback18s; no physics or controller call."""
from pathlib import Path
import csv
import hashlib
import json
import shutil

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SOURCE=ROOT/'results/belt_capacity_sweep_20261126'
CASES=('mars_cap030','mars_cap060','mars_cap090')
LABELS=('shell_A','shell_B','belt_A','belt_B')
WINDOWS={'playback18_context':(30.5,33.5),'recorded18_alternative':(17.,19.)}

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())

def load(case):
    folder=SOURCE/'campaign/runs'/case
    with np.load(folder/'precision.npz')as z:d={k:z[k].copy()for k in z.files}
    q=d['exact_qpos'][:,3:7];q=q/np.linalg.norm(q,axis=1)[:,None]
    d['absolute_tilt_deg']=np.degrees(np.arccos(np.clip(1-2*np.sum(q[:,1:3]**2,axis=1),-1,1)))
    d['body_angular_speed_deg_s']=np.degrees(np.linalg.norm(d['exact_qvel'][:,3:6],axis=1))
    d['body_up_rotation_speed_deg_s']=np.degrees(np.linalg.norm(d['exact_qvel'][:,3:5],axis=1))
    d['planar_speed_m_s']=np.linalg.norm(d['exact_qvel'][:,:2],axis=1)
    d['timed_error_mm']=1000*np.linalg.norm(d['exact_qpos'][:,:2]-d['exact_reference_xy'],axis=1)
    for kind,key in [('truth','truth_patch_loads_N'),('estimated','exact_estimated_support_loads_N')]:
        values=d[key];total=values.sum(axis=1)
        d[f'{kind}_belt_B_fraction']=np.divide(values[:,3],total,out=np.zeros_like(total),where=total>.05)
        d[f'{kind}_dominant_patch']=np.where(total>.05,np.argmax(values,axis=1),-1)
    return d

def main():
    for name in ('event_motion_report.json','event_motion_samples.csv','event_motion.png'):
        if(HERE/name).exists():raise FileExistsError('Preserve earlier audit before a new stage')
    source=Path(__file__).resolve();snap=HERE/'source_snapshot';snap.mkdir(exist_ok=True)
    shutil.copy2(source,snap/f'{source.stem}_{sha(source)[:12]}.py')
    inputs=[source,SOURCE/'videos/belt_capacity_comparison.mp4',SOURCE/'videos/video_display_schedule.json',ROOT/'src/spherical_wheel/gazebo/plant.py']
    for case in CASES:inputs += [SOURCE/'campaign/runs'/case/name for name in ('precision.npz','rollout.npz','provenance.json','artifact_manifest.json')]
    hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    runs={case:load(case)for case in CASES};schedule=read(SOURCE/'videos/video_display_schedule.json')
    mapping=[r for r in schedule if r['frame']in(220,340,350,360,370,380)]
    result={'scope':__doc__,'input_sha256':hashes,'video_mapping':mapping,'windows':{},
        'conventions':{
            'playback_mapping':'20fps video:2s intro then2x; frame360/playback18s displays recorded32s.',
            'qvel':'GazeboPlant._pull stores linear world velocity and angular BODY velocity, then shaft/shell joint rates.',
            'body_up_rotation_speed':'||omega_body[:2]|| in deg/s; rate of the platform up-vector, independent of rotation about that up-vector. Not an Euler-roll/pitch derivative.',
            'absolute_tilt':'Angle between platform up and world vertical; not lean relative to the terrain or controller reference.',
            'truth':'Stored current evaluator contact endpoint only; dominant argmax labels may chatter, and are not stable transfer estimates.',
            'causality':'Descriptive timing associations only. No counterfactual controller or terrain run; no unique cause established.'},
        'cases':{},'observations':[
            'Playback18s is recorded32s, not recorded18s. The alternative recorded18s sample has zero applied belts and shell support in every arm.',
            'The raised .60/.90 trajectories are identical; the slow replay retains three columns for correspondence to the original video.',
            'Shell_B/belt_B support changes occur in the mapped late window; body-up motion is already present before the principal belt-supported phase.',
            'Estimated support, preparation weights and actual input are plotted separately from evaluator truth; truth is never sent to a controller.'
        ]}
    fields=['case','time_s','absolute_tilt_deg','body_up_rotation_speed_deg_s','body_angular_speed_deg_s','planar_speed_m_s','timed_error_mm','truth_dominant_patch','estimated_dominant_patch','truth_belt_B_fraction','estimated_belt_B_fraction','incoming_belt_B_weight','incoming_shell_B_weight','belt_B_requested_m_s','belt_B_applied_m_s']
    rows=[]
    for case,d in runs.items():
        t=d['time_s'];case_result={}
        for window,(a,b)in WINDOWS.items():
            ix=np.flatnonzero((t>=a-1e-8)&(t<=b+1e-8));j=ix[np.argmax(d['body_up_rotation_speed_deg_s'][ix])];k=ix[np.argmax(d['body_angular_speed_deg_s'][ix])]
            case_result[window]={'window_s':[a,b],'samples':len(ix),'absolute_tilt_range_deg':[float(d['absolute_tilt_deg'][ix].min()),float(d['absolute_tilt_deg'][ix].max())],
                'body_up_rotation_peak':{'time_s':float(t[j]),'deg_s':float(d['body_up_rotation_speed_deg_s'][j]),'truth_loads_N':d['truth_patch_loads_N'][j].tolist()},
                'body_angular_speed_peak':{'time_s':float(t[k]),'deg_s':float(d['body_angular_speed_deg_s'][k]),'body_components_deg_s':np.degrees(d['exact_qvel'][k,3:6]).tolist(),'truth_loads_N':d['truth_patch_loads_N'][k].tolist()},
                'planar_speed_range_m_s':[float(d['planar_speed_m_s'][ix].min()),float(d['planar_speed_m_s'][ix].max())],
                'first_instantaneous_belt_B_dominant_s':next((float(t[i])for i in ix if d['truth_dominant_patch'][i]==3),None)}
            for i in ix:
                row={key:float(d[key][i])for key in fields if key in d and np.ndim(d[key])==1}
                row.update(case=case,truth_dominant_patch=LABELS[int(d['truth_dominant_patch'][i])]if d['truth_dominant_patch'][i]>=0 else'none',estimated_dominant_patch=LABELS[int(d['estimated_dominant_patch'][i])]if d['estimated_dominant_patch'][i]>=0 else'none',incoming_belt_B_weight=float(d['exact_sync_weights'][i,4]),incoming_shell_B_weight=float(d['exact_sync_weights'][i,2]),belt_B_requested_m_s=float(d['exact_native_requested'][i,4]),belt_B_applied_m_s=float(d['exact_applied'][i,4]))
                rows.append(row)
        result['cases'][case]=case_result
    with(HERE/'event_motion_samples.csv').open('w')as file:
        writer=csv.DictWriter(file,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    result['raised_motion_identity']=bool(np.array_equal(runs[CASES[1]]['exact_qpos'],runs[CASES[2]]['exact_qpos'])and np.array_equal(runs[CASES[1]]['exact_qvel'],runs[CASES[2]]['exact_qvel']))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(5,2,figsize=(12,12),sharex=True,constrained_layout=True)
    for col,case in enumerate(CASES[:2]):
        d=runs[case];t=d['time_s'];m=(t>=30.5-1e-8)&(t<=33.5+1e-8);x=t[m]
        axes[0,col].plot(x,d['absolute_tilt_deg'][m],color='black');axes[0,col].set(ylabel='Absolute tilt (deg)',title='.30 operating cap'if col==0 else'.60/.90: identical recorded motion')
        axes[1,col].plot(x,d['body_up_rotation_speed_deg_s'][m],color='#9566a7');axes[1,col].set(ylabel='Body-up rotation (deg/s)')
        axes[2,col].plot(x,d['truth_belt_B_fraction'][m],label='Truth endpoint',color='#1b9eae');axes[2,col].plot(x,d['estimated_belt_B_fraction'][m],label='Observer',color='#d9902b',ls='--');axes[2,col].set(ylabel='Belt-B load fraction',ylim=(-.05,1.05));axes[2,col].legend(fontsize=8)
        axes[3,col].plot(x,d['exact_sync_weights'][m,4],label='Incoming belt B',color='#1b9eae');axes[3,col].plot(x,d['exact_sync_weights'][m,2],label='Incoming shell B',color='#d9902b');axes[3,col].set(ylabel='Prep blend weight',ylim=(-.05,1.05));axes[3,col].legend(fontsize=8)
        axes[4,col].plot(x,d['exact_native_requested'][m,4],label='Requested',color='#d9902b');axes[4,col].plot(x,d['exact_applied'][m,4],label='Applied',color='#1b9eae');axes[4,col].set(ylabel='Belt B (m/s)',xlabel='Recorded simulation time (s)');axes[4,col].legend(fontsize=8)
        for ax in axes[:,col]:ax.axvline(32.,color='gray',ls=':',lw=1);ax.grid(alpha=.2)
    fig.suptitle('I49 playback18s = recorded32s: actual saved motion and handoff signals\nCurrent truth is evaluator-only; instantaneous contact labels chatter; this is not a causal experiment.',fontsize=13)
    for ext in ('png','pdf'):fig.savefig(HERE/f'event_motion.{ext}',dpi=180)
    plt.close(fig)
    assert hashes=={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    (HERE/'event_motion_report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'mapping':mapping,'cases':result['cases'],'inputs_unchanged':True},indent=2))

if __name__=='__main__':main()
