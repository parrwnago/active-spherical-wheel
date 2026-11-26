"""Recover logged blend targets and current-pose contact velocities, no preview.

Only mj_kinematics, mj_comPos and mj_jac operate on private data. No integration,
forward dynamics, contact queries, observer/controller calls or forecast occur.
"""
from datetime import datetime,timezone
from pathlib import Path
from unittest.mock import patch
from contextlib import ExitStack
import csv
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
INPUT=ROOT/'results/belt_capacity_sweep_20261126'
CASES=('mars_cap030','mars_cap060','mars_cap090')
LABELS=('shell_A','shell_B','belt_A','belt_B')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [safe(v)for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float)and not np.isfinite(x):return None
    return x
def save(path,x):path.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def forbidden(*args,**kwargs):raise AssertionError('Dynamics, integration, forecast and contact queries forbidden')


def main():
    output=HERE/'stage03_current_kinematics';output.mkdir(exist_ok=False)
    sources=[Path(__file__).resolve(),ROOT/'src/spherical_wheel/synchronization.py',ROOT/'src/spherical_wheel/offroad_control.py']
    records=[]
    for source in sources:
        dest=output/'source_snapshot'/source.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(source.read_bytes())
        records.append(dict(origin=str(source.relative_to(ROOT)),snapshot=str(dest.relative_to(output)),sha256=sha(source)))
    inputs={};models={}
    for case in CASES:
        for name in ('precision.npz','rollout.npz','provenance.json'):
            p=INPUT/'campaign/runs'/case/name;inputs[str(p.relative_to(ROOT))]=sha(p)
        matches=list((INPUT/'campaign/_workspaces'/case).glob('worker_*/mirror/model.xml'));assert len(matches)==1
        models[case]=matches[0]
        inputs[str(matches[0].relative_to(ROOT))]=sha(matches[0])
        for mesh in ET.parse(matches[0]).getroot().findall('asset/mesh'):
            p=matches[0].parent/mesh.attrib['file'];inputs[str(p.relative_to(ROOT))]=sha(p)
    save(output/'before_arithmetic.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),sources=records,input_sha256=inputs,
        fixed_windows_s=[[17.,19.],[30.,34.]],zoom_window_s=[31.5,32.5],
        scope='Saved-row algebra plus private CURRENT-pose kinematics only. Recover targetcommand=pre_sync+transition_delta/weight for weight>=1e-6. No future geometry or preview reproduced.',
        point_precision='Loaded estimated patch points are savedfloat32 load-weighted means; unloaded points are current virtualspheresupports. Jacobian affine-in-point averaging applies within each rigidpatch. Normal is filtered estimated plane, not physical collisionnormal.'))
    assert len({sha(p)for p in models.values()})==1
    model=mujoco.MjModel.from_xml_path(str(models[CASES[0]]));data=mujoco.MjData(model)
    assert model.nq==10 and model.nv==9
    patch=[]
    for label in LABELS:
        geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,label+'_smooth')
        patch.append(dict(geom=geom,body=int(model.geom_bodyid[geom]),radius=float(model.geom_user[geom,4])))
    reports={}
    with ExitStack()as stack:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_integratePos','mj_contactForce'):
            stack.enter_context(patch_tool(mujoco,name,forbidden))
        for case in CASES:
            p=load(INPUT/'campaign/runs'/case/'precision.npz');log=load(INPUT/'campaign/runs'/case/'rollout.npz');t=p['time_s']
            weight=float(json.loads((INPUT/'campaign/runs'/case/'provenance.json').read_text())['weight_N'])
            selected=((t>=17.-1e-9)&(t<=19.+1e-9))|((t>=30.-1e-9)&(t<=34.+1e-9));ii=np.flatnonzero(selected)
            loads=p['exact_estimated_support_loads_N'];weights=p['exact_sync_weights'];delta=p['exact_transition_command_delta']
            before=sum(p[key]for key in ('exact_equilibrium_command','exact_rolling_feedforward','exact_support_command','exact_lqr_command'))
            target=np.full_like(before,np.nan);active=weights>=1e-6;target[active]=before[active]+delta[active]/weights[active]
            rows=[]
            for i in ii:
                data.qpos[:]=p['exact_qpos'][i];data.qvel[:]=p['exact_qvel'][i]
                mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
                n=p['exact_estimated_normal'][i].copy();n/=np.linalg.norm(n);projection=np.eye(3)-np.outer(n,n)
                velocities=[];base=[];axes=[];origins=[]
                for j,meta in enumerate(patch):
                    point=log['estimated_contact_points_m'][i,j].astype(float)
                    loaded=loads[i,j]>1e-8 and np.isfinite(point).all()
                    if not loaded:point=data.xpos[meta['body']]-meta['radius']*n
                    jac=np.zeros((3,model.nv));mujoco.mj_jac(model,data,jac,None,point,meta['body'])
                    material=projection@(jac@p['exact_qvel'][i]);axis=projection@data.geom_xmat[meta['geom']].reshape(3,3)[:,1]
                    velocity=material.copy()
                    if j>=2:velocity+=axis*p['exact_previous_applied'][i,j+1]
                    base.append(material);velocities.append(velocity);axes.append(axis);origins.append('savedfloat32estimatedpoint'if loaded else'currentvirtualpoint')
                outgoing=np.sum(np.asarray(velocities)*loads[i,:,None],axis=0)/max(loads[i].sum(),1e-12)
                row=dict(index=int(i),time_s=t[i],outgoing_current_speed_m_s=np.linalg.norm(outgoing),
                    outgoing_current_velocity_x_m_s=outgoing[0],outgoing_current_velocity_y_m_s=outgoing[1],outgoing_current_velocity_z_m_s=outgoing[2])
                for j,label in enumerate(LABELS):
                    col=j+1
                    row[label+'_point_source']=origins[j];row[label+'_weight']=weights[i,col]
                    row[label+'_recovered_target_command']=target[i,col]
                    row[label+'_current_contact_speed_m_s']=np.linalg.norm(velocities[j])
                    row[label+'_current_mismatch_to_outgoing_m_s']=np.linalg.norm(velocities[j]-outgoing)
                    if j>=2:
                        row[label+'_current_pose_residual_if_recovered_target_m_s']=(np.linalg.norm(base[j]+axes[j]*target[i,col])if np.isfinite(target[i,col])else np.nan)
                        row[label+'_effective_surface_axis_norm']=np.linalg.norm(axes[j])
                rows.append(safe(row))
            with(output/(case+'_current_kinematics.csv')).open('w',newline='')as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
            zoom=(t>=31.5-1e-9)&(t<=32.5+1e-9);prep=zoom&(weights[:,4]>1e-12)
            strong_truth=zoom&(p['truth_patch_loads_N'][:,3]>=.05*weight)
            meaningful=zoom&active[:,4]
            fractions=loads[:,3]/np.maximum(loads.sum(axis=1),1e-12)
            changes=np.flatnonzero(zoom&np.r_[False,(fractions[1:]>=.05)!=(fractions[:-1]>=.05)])
            peak=ii[np.argmax(np.where(zoom[ii],abs(p['exact_applied'][ii,4]-p['exact_previous_applied'][ii,4]),-1))]
            times_to_report=[31.6,31.8,31.95,31.975,32.,32.1,32.15,32.16,32.165,32.17,32.18,32.19,32.23,32.30,32.325,32.375,32.4,32.5]
            nearest={int(np.argmin(abs(t-now)))for now in times_to_report};nearest.add(int(peak))
            reports[case]=dict(zoom_window_s=[31.5,32.5],first_nonzero_beltB_preparation_within_zoom_s=float(t[np.flatnonzero(prep)[0]])if prep.any()else None,
                preparation_at_zoom_start_left_censored=bool(weights[np.flatnonzero(zoom)[0],4]>1e-12),
                first_strong_beltB_truth_within_zoom_s=float(t[np.flatnonzero(strong_truth)[0]])if strong_truth.any()else None,
                recovered_beltB_target_range_m_s=[np.nanmin(target[meaningful,4]),np.nanmax(target[meaningful,4])]if meaningful.any()else None,
                max_beltB_applied_step_in_zoom=dict(index=int(peak),time_s=t[peak],current_applied_m_s=p['exact_applied'][peak,4],previous_applied_m_s=p['exact_previous_applied'][peak,4],
                    sync_weight=weights[peak,4],transition_delta=delta[peak,4],observer_loads_N=loads[peak],truth_loads_N=p['truth_patch_loads_N'][peak]),
                inferred5pct_beltB_threshold_crossings=[dict(index=int(i),time_s=t[i],previous_share=fractions[i-1],current_share=fractions[i],
                    previous_weight=weights[i-1,4],current_weight=weights[i,4],previous_recovered_target_m_s=target[i-1,4],current_recovered_target_m_s=target[i,4],
                    previous_pre_sync_m_s=before[i-1,4],current_pre_sync_m_s=before[i,4],previous_transition_m_s=delta[i-1,4],current_transition_m_s=delta[i,4],
                    previous_requested_m_s=p['exact_native_requested'][i-1,4],current_requested_m_s=p['exact_native_requested'][i,4],
                    applied_step_m_s=p['exact_applied'][i,4]-p['exact_applied'][i-1,4],
                    lqr_step_m_s=p['exact_lqr_command'][i,4]-p['exact_lqr_command'][i-1,4],
                    rolling_step_m_s=p['exact_rolling_feedforward'][i,4]-p['exact_rolling_feedforward'][i-1,4],
                    truth_beltB_load_N=p['truth_patch_loads_N'][i,3],observer_beltB_load_N=loads[i,3])for i in changes],
                selected_rows=[row for row in rows if row['index']in nearest])
    save(output/'report.json',dict(cases=reports,source='Current pose only; conditional algebraic target recovery from logged pre_sync/delta/weights.',
        caveats=['Recovered targets are actual already-bounded commandscalars, not unsaturated demands or full future targetrecords.',
            'Current-pose residual using a future-selected target is a descriptive consistency check, not the missing predicted future residual.',
            'Direct copying of outgoing slip is not necessarily a better target; this audit does not prescribe that change.',
            'Preparation becomeszero whenestimatedloadfractionreaches5%; native LQR/feedforward then carries the loadeddrive. There is no guaranteedbumpless matching at that boundary.']))
    for rel,expected in inputs.items():assert sha(ROOT/rel)==expected
    for row in records:assert sha(ROOT/row['origin'])==row['sha256']
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(output.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
    print(json.dumps(safe({case:{key:value for key,value in info.items()if key!='selected_rows'}for case,info in reports.items()}),indent=2))


patch_tool=patch.object
if __name__=='__main__':main()
