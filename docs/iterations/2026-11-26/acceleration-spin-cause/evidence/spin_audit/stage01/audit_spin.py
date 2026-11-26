"""I54: saved-state platform/material spin audit. No model/controller calls."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import csv
import numpy as np

H = Path(__file__).resolve().parent
ROOT = H.parents[2]
OLD = ROOT / 'results/contact_scheduling_joint_tracking_20261126'
CASES = tuple('m100_' + x for x in ('B', 'R', 'N', 'NR'))
WINDOWS = {'full_trace': [0., 44.], 'after28': [28., 44.], 'burst': [31.6, 32.5], 'after_gain_switch': [32.25, 44.], 'after_speed_peak': [32.305, 44.]}
FIXED = [0, 1, 10, 20, 28, 30, 31, 31.5, 31.6, 31.715, 31.765, 31.77, 32, 32.2, 32.24, 32.245, 32.25, 32.255, 32.26, 32.265, 32.27, 32.275, 32.28, 32.285, 32.29, 32.295, 32.3, 32.305, 32.35, 32.5, 33, 34, 36, 38, 40, 42, 44]
THRESHOLDS = (.5, 1., 2.)
TERMS = ('equilibrium_command', 'rolling_feedforward', 'support_command', 'lqr_command', 'transition_command_delta', 'heading_command', 'fade_command_delta')
GROUPS = {'position': slice(0,3), 'attitude': slice(3,6), 'joint_position': slice(6,9), 'linear_velocity': slice(9,12), 'angular_velocity': slice(12,15), 'joint_velocity': slice(15,18)}
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
    if isinstance(v, dict): return {str(k): safe(x) for k,x in v.items()}
    if isinstance(v, (tuple, list)): return [safe(x) for x in v]
    if isinstance(v, np.ndarray): return safe(v.tolist())
    if isinstance(v, np.generic): return safe(v.item())
    if isinstance(v, float) and not np.isfinite(v): return None
    return v
def save(p,v): p.write_text(json.dumps(safe(v), indent=2, allow_nan=False)+'\n')
def load(p):
    with np.load(p, allow_pickle=False) as z: return {k:z[k].copy() for k in z.files}
def rotate(q, v):
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    c = np.cross(q[...,1:], v)
    return v + 2*q[...,:1]*c + 2*np.cross(q[...,1:], c)
def multiply(a,b):
    return np.r_[a[0]*b[0]-a[1:]@b[1:], a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:],b[1:])]
def unwrap_valid(angle, valid):
    result = np.full(len(angle), np.nan)
    for ids in np.split(np.flatnonzero(valid), np.flatnonzero(np.diff(np.flatnonzero(valid))>1)+1):
        if len(ids): result[ids] = np.unwrap(angle[ids])
    return result
def heading_signals(q, omega_world):
    x = rotate(q, np.broadcast_to([1.,0,0], (len(q),3)))
    h2 = np.sum(x[:,:2]**2, axis=1)
    valid = h2 > .2**2
    xdot = np.cross(omega_world, x)
    rate = (x[:,0]*xdot[:,1]-x[:,1]*xdot[:,0])/np.maximum(h2, 1e-30)
    rate[~valid] = np.nan
    angle = unwrap_valid(np.arctan2(x[:,1],x[:,0]), valid)
    return angle, rate, valid, np.sqrt(h2)
def episodes(t, signal, lo, hi, threshold, dwell=.1):
    inwindow = (t>=lo-1e-9)&(t<=hi+1e-9)
    active = inwindow & (abs(signal)>=threshold)
    ii = np.flatnonzero(active)
    cuts = np.flatnonzero((np.diff(ii)>1)|(np.sign(signal[ii[1:]])!=np.sign(signal[ii[:-1]])))+1
    out = []
    for ids in np.split(ii,cuts):
        if not len(ids): continue
        a,b = int(ids[0]),int(ids[-1])
        if t[b]-t[a] < dwell-1e-9: continue
        out.append(dict(start_s=t[a], end_s=t[b], sampled_span_s=t[b]-t[a], direction_sign=int(np.sign(signal[a])), samples=len(ids), left_censored=bool(a==0 or (t[a-1]<lo-1e-9 and abs(signal[a-1])>=threshold)), right_censored=bool(b==len(t)-1 or (t[b+1]>hi+1e-9 and abs(signal[b+1])>=threshold))))
    return out


def pure_checks():
    results = []
    q = np.array([[1.,0,0,0],[np.cos(.4),0,0,np.sin(.4)]])
    angle,rate,valid,_ = heading_signals(q,np.array([[0.,0,2.],[0.,0,2.]]))
    assert np.max(abs(angle-[0.,.8]))<1e-12 and np.max(abs(rate-2))<1e-12 and valid.all()
    results.append('known yaw angle and world-z heading derivative')
    q0=np.array([.82,.2,-.3,.4]);q0/=np.linalg.norm(q0);w=np.array([.7,-.4,1.3]);dt=1e-7
    a=np.linalg.norm(w)*dt;dq=np.r_[np.cos(a/2),np.sin(a/2)*w/np.linalg.norm(w)];q1=multiply(dq,q0)
    ang,r,valid,_=heading_signals(np.array([q0,q1]),np.array([w,w]));assert abs((ang[1]-ang[0])/dt-r[0])<2e-7
    results.append('tilted quaternion finite-difference heading derivative')
    q90=np.array([[np.sqrt(.5),0,np.sqrt(.5),0]])
    _,_,valid,_=heading_signals(q90,np.zeros((1,3)));assert not valid[0]
    results.append('vertical projected heading abstains rather than Euler artifact')
    assert np.allclose(rotate(np.array([[1.,0,0,0]]),np.array([[1.,0,0]])),[[1,0,0]])
    phi=np.pi/2;assert np.allclose([np.cos(phi),0,-np.sin(phi)],[0,0,-1],atol=1e-15)
    results.append('shaft-y and carrier-x shell-axis convention')
    t=np.arange(24)*.005;assert not episodes(t[:20],np.ones(20),0,.095,1.)
    assert len(episodes(t,np.ones(24),0,.115,1.))==1
    results.append('100ms dwell requires duration, not a sample-count shortcut')
    r=episodes(t,np.r_[np.ones(12),-np.ones(12)],0,.115,1.);assert not r
    results.append('opposite spin signs do not join into one sustained episode')
    return {'passes':True,'checks':results}


def main():
    out=H/'stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/Path(__file__).name)
    mapping=OLD/'campaign/resolved_case_paths.json';paths=json.loads(mapping.read_text())['cases'];folders={c:ROOT/paths[c] for c in CASES}
    inputs=[mapping,OLD/'protocol.json',OLD/'analysis/stage01/report.json',OLD/'videos/video_display_schedule.json',ROOT/'src/spherical_wheel/model.py',ROOT/'src/spherical_wheel/sim.py',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'results/teammate_handoff_20261126/event_audit/final_report.json']
    for c in CASES: inputs.extend(folders[c]/f for f in ('precision.npz','rollout.npz','provenance.json','summary.json','artifact_manifest.json'))
    hashes={str(p.relative_to(ROOT)):sha(p) for p in inputs}
    save(out/'registered_before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=hashes,windows_s=WINDOWS,fixed_snapshots_s=FIXED,spin_thresholds_rad_s=THRESHOLDS,dwell_s=.1,selection='All four m100 arms. Full-trace and fixed28–44s heading/world-z/material-spin summaries; first and all sustained .5/1/2rad/s episodes, per-window maxima; no favorable-run selection. Fixed31.6–32.5 burst window and32.25–44 after-gain window set before calculation.',heading='Fork local-x direction projected to XY; valid iff horizontal norm>.2. Unwrapped only within contiguous valid segments, never across undefined heading. Its analytic derivative uses xdot=omega_world cross x. World-z angular velocity is separately R(q)*freejoint body omega, not Euler yaw rate.',material_spin='Relative shaft/shell encoder rates are distinct from platform heading. Absolute carrier omega=platformomega+worldshaftaxis*shaft_rate; absolute shell omega adds worldcarrierxaxis*shell_rate. This kinematic identity is not angular-momentum or contact-force attribution.',clock='Backward measured change ending rowi was driven by applied input at rowi−1. Current command acts next. Endpointcontacttruth is last1ms sample, not a full5ms hold mean.',limits='No simulator/controller/model evaluation; no logged computational-latency inference. Thresholds are diagnostic only; source/current algebra cannot identify unique physical cause.'))
    save(out/'pure_checks.json',pure_checks())
    result={'registered_windows_s':WINDOWS,'cases':{},'limits':['Heading is a geometric projected direction; validity is checked separately from world-z angular velocity.','Relative shell/shaft spin is not platform heading rotation. Absolute shell rates are kinematic, not a torque or momentum balance.','No heading-position hold is active; the native reference carries measured yaw. This does not imply all angular-rate feedback is absent.','A sampled currentcommand cannot cause the measured increment already ending at that row.','Endpoint contacts do not resolve within-hold contact impulses; the unique physical cause of the complete coupled event remains unisolated.']};csvrows=[]
    for case in CASES:
        p=load(folders[case]/'precision.npz');l=load(folders[case]/'rollout.npz');t=p['time_s'];q=p['exact_qpos'];v=p['exact_qvel'];n=len(t);quat=q[:,3:7];K=p['exact_feedback_gain'];e=p['exact_error'];req=p['exact_native_requested'];u=p['exact_applied']
        assert q.shape==(n,10) and v.shape==(n,9)
        world=rotate(quat,v[:,3:6]);heading,headingrate,valid,hproj=heading_signals(quat,world)
        shaftaxis=rotate(quat,np.broadcast_to([0.,1.,0.],(n,3)));shellaxis=rotate(quat,np.column_stack([np.cos(q[:,7]),np.zeros(n),-np.sin(q[:,7])]))
        carrier=world+shaftaxis*v[:,6,None];shellA=carrier+shellaxis*v[:,7,None];shellB=carrier+shellaxis*v[:,8,None]
        up=rotate(quat,np.broadcast_to([0.,0.,1.],(n,3)));refup=rotate(p['exact_reference_quat'],np.broadcast_to([0.,0.,1.],(n,3)))
        lean=np.degrees(np.arccos(np.clip(np.sum(up*refup,axis=1),-1,1)));tilt=np.degrees(np.arccos(np.clip(up[:,2],-1,1)));speed=np.linalg.norm(v[:,:2],axis=1);timed=np.linalg.norm(q[:,:2]-p['exact_reference_xy'],axis=1)
        groups={name:-np.einsum('nij,nj->ni',K[:,:,sl],e[:,sl]) for name,sl in GROUPS.items()}
        assert np.max(abs(sum(groups.values())-p['exact_lqr_command']))<1e-9
        assert np.max(abs(sum(p['exact_'+k] for k in TERMS)-req))<1e-9
        gainjump=np.zeros((n,5));errorjump=gainjump.copy();gainjump[1:]=-np.einsum('nij,nj->ni',np.diff(K,axis=0),(e[1:]+e[:-1])/2);errorjump[1:]=-np.einsum('nij,nj->ni',(K[1:]+K[:-1])/2,np.diff(e,axis=0))
        assert np.max(abs(gainjump[1:]+errorjump[1:]-np.diff(p['exact_lqr_command'],axis=0)))<1e-9
        weight=json.loads((folders[case]/'provenance.json').read_text())['weight_N']
        def dominant(a): return LABELS[int(np.argmax(a))] if np.sum(a)>=max(.05,.05*weight) else 'none'
        def control(i): return dict(time_s=t[i],requested=req[i],applied=u[i],LQRgroups={k:x[i] for k,x in groups.items()},terms={k:p['exact_'+k][i] for k in TERMS},gain_jump=gainjump[i],error_jump=errorjump[i],estimated_loads_N=p['exact_estimated_support_loads_N'][i],truth_loads_N=p['truth_patch_loads_N'][i],estimated_patch=dominant(p['exact_estimated_support_loads_N'][i]),truth_patch=dominant(p['truth_patch_loads_N'][i]),used_joint_reference_rad_s=p['exact_guarded_joint_reference'][i],native_joint_reference_rad_s=p['exact_native_joint_reference'][i],angular_velocity_reference_body_rad_s=p['exact_feedback_reference_qvel'][i,3:6],phase_rad=p['exact_effective_phase_rad'][i],sync_weights=p['exact_sync_weights'][i])
        def snapshot(i):
            b=max(0,i-1);dt=t[i]-t[b]
            return dict(index=int(i),time_s=t[i],interval_start_s=t[b],heading_unwrapped_rad=heading[i],heading_valid=valid[i],horizontal_projection_norm=hproj[i],heading_derivative_rad_s=headingrate[i],platform_world_omega_rad_s=world[i],platform_body_omega_rad_s=v[i,3:6],world_omega_increment_rad_s=world[i]-world[b],world_angular_acceleration_interval_rad_s2=(world[i]-world[b])/dt if dt else np.zeros(3),body_linear_velocity_m_s=v[i,:3],planar_speed_m_s=speed[i],planar_velocity_increment_m_s=v[i,:2]-v[b,:2],relative_joint_positions_rad=q[i,7:10],relative_joint_rates_rad_s=v[i,6:9],shaft_axis_world=shaftaxis[i],shell_axis_world=shellaxis[i],absolute_carrier_omega_world_rad_s=carrier[i],absolute_shellA_omega_world_rad_s=shellA[i],absolute_shellB_omega_world_rad_s=shellB[i],relative_lean_deg=lean[i],absolute_tilt_deg=tilt[i],timed_error_m=timed[i],reference_xy=p['exact_reference_xy'][i],reference_velocity_m_s=p['exact_reference_velocity_m_s'][i],estimated_normal=p['exact_estimated_normal'][i],slip_speed_m_s=l['slip_speed_m_s'][i],contact_force_N=l['contact_force_N'][i],current_control=control(i),previous_hold_control=control(b))
        selected={int(np.argmin(abs(t-at))) for at in FIXED};windows={}
        for name,(lo,hi) in WINDOWS.items():
            ids=np.flatnonzero((t>=lo-1e-9)&(t<=hi+1e-9));a,b=int(ids[0]),int(ids[-1]);pairs=ids[1:][valid[ids[1:]]&valid[ids[:-1]]];dtheta=heading[pairs]-heading[pairs-1]
            peaks={}
            for label,signal in {'max_abs_world_z_rate':abs(world[:,2]),'max_abs_heading_derivative':abs(headingrate),'max_planar_speed':speed,'max_abs_relative_joint_rate':np.max(abs(v[:,6:9]),axis=1),'max_lean':lean}.items():
                i=int(ids[np.nanargmax(signal[ids])]);selected.add(i);peaks[label]={'time_s':t[i],'value':signal[i]}
            eps={str(th):episodes(t,world[:,2],lo,hi,th) for th in THRESHOLDS}
            for ep in eps.values():
                for row in ep:selected.update((int(np.argmin(abs(t-row['start_s']))),int(np.argmin(abs(t-row['end_s'])))))
            windows[name]=dict(window_s=[lo,hi],samples=len(ids),valid_heading_samples=int(valid[ids].sum()),minimum_heading_projection_norm=hproj[ids].min(),heading_net_turns=(heading[b]-heading[a])/(2*np.pi) if valid[ids].all() else None,heading_absolute_travel_turns=np.sum(abs(dtheta))/(2*np.pi),world_z_rate_peak_abs_rad_s=np.max(abs(world[ids,2])),world_z_rate_rms_rad_s=np.sqrt(np.mean(world[ids,2]**2)),heading_rate_peak_abs_rad_s=np.nanmax(abs(headingrate[ids])),end_world_z_rate_rad_s=world[b,2],end_heading_derivative_rad_s=headingrate[b],material_relative_net_turns=(q[b,7:10]-q[a,7:10])/(2*np.pi),relative_joint_rate_max_abs_rad_s=np.max(abs(v[ids,6:9]),axis=0),absolute_shell_world_z_rate_max_abs_rad_s=np.max(abs(np.column_stack([shellA[ids,2],shellB[ids,2]])),axis=0),world_z_sustained_episodes=eps,peaks=peaks)
        signals=dict(time_s=t,heading_unwrapped_rad=heading,heading_derivative_rad_s=headingrate,heading_valid=valid,horizontal_heading_projection_norm=hproj,platform_omega_world_rad_s=world,absolute_carrier_omega_world_rad_s=carrier,absolute_shellA_omega_world_rad_s=shellA,absolute_shellB_omega_world_rad_s=shellB,shaft_axis_world=shaftaxis,shell_axis_world=shellaxis,relative_lean_deg=lean,absolute_tilt_deg=tilt,planar_speed_m_s=speed,timed_error_m=timed)
        np.savez_compressed(out/(case+'_signals.npz'),**signals)
        result['cases'][case]=dict(windows=windows,snapshots=[snapshot(i) for i in sorted(selected)],heading_command_max_abs=np.max(abs(p['exact_heading_command']),axis=0),angular_velocity_reference_body_max_abs_rad_s=np.max(abs(p['exact_feedback_reference_qvel'][:,3:6]),axis=0),feedback_quaternion_yaw_tangent_error_max_abs_rad=np.max(abs(e[:,5])),heading_derivative_minus_world_z_max_abs_rad_s=np.max(abs(headingrate-world[:,2])))
        for i in np.flatnonzero(t>=28-1e-9):
            row=dict(case=case,time_s=t[i],heading_rad=heading[i],heading_derivative_rad_s=headingrate[i],heading_valid=bool(valid[i]),world_omega_z_rad_s=world[i,2],body_omega_z_rad_s=v[i,5],relative_lean_deg=lean[i],speed_m_s=speed[i],timed_error_m=timed[i],estimated_patch=dominant(p['exact_estimated_support_loads_N'][i]),truth_patch=dominant(p['truth_patch_loads_N'][i]))
            for label,values in [('joint_rate',v[i,6:9]),('applied',u[i]),('requested',req[i])]: row.update({label+'_'+str(j):x for j,x in enumerate(values)})
            csvrows.append(safe(row))
    save(out/'report.json',result)
    with (out/'spin_samples28_44.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=csvrows[0].keys());w.writeheader();w.writerows(csvrows)
    for rel,h in hashes.items(): assert sha(ROOT/rel)==h,rel
    save(out/'artifact_manifest.json',{'files':[dict(path=str(p.relative_to(out)),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(out.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']})
    print('I54 saved-spin reduction complete; all source/input hashes unchanged.')


if __name__=='__main__': main()
