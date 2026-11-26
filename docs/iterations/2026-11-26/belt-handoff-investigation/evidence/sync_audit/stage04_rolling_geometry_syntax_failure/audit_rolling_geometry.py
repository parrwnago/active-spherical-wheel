"""Independent saved single-patch rolling matrix, private current kinematics only."""
from datetime import datetime,timezone
from pathlib import Path
from contextlib import ExitStack
from unittest.mock import patch
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
SCALE=np.array([5.,5.,5.,.3,.3])


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v)for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float)and not np.isfinite(x):return None
    return x
def save(path,x):path.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def forbidden(*args,**kwargs):raise AssertionError('Dynamics/integration/solvedcontacts forbidden')


def main():
    output=HERE/'stage04_rolling_geometry';output.mkdir(exist_ok=False)
    sources=[Path(__file__).resolve(),ROOT/'src/spherical_wheel/experiments.py',ROOT/'src/spherical_wheel/contact_sensing.py']
    records=[]
    for source in sources:
        dest=output/'source_snapshot'/source.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(source.read_bytes())
        records.append(dict(origin=str(source.relative_to(ROOT)),snapshot=str(dest.relative_to(output)),sha256=sha(source)))
    inputs={};models={}
    for case in CASES:
        source=INPUT/'campaign/runs'/case/'precision.npz';inputs[str(source.relative_to(ROOT))]=sha(source)
        matches=list((INPUT/'campaign/_workspaces'/case).glob('worker_*/mirror/model.xml'));assert len(matches)==1
        models[case]=matches[0];inputs[str(matches[0].relative_to(ROOT))]=sha(matches[0])
        for mesh in ET.parse(matches[0]).getroot().findall('asset/mesh'):
            source=matches[0].parent/mesh.attrib['file'];inputs[str(source.relative_to(ROOT))]=sha(source)
    save(output/'before_arithmetic.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),sources=records,input_sha256=inputs,
        selection='Fixed31.645/31.650/31.655/31.660 and perarm maximum raw normalized torque and maximum successive raw normalized torquejump within recorded30–34. No selection by resulting matrixcondition.',
        scope='Pure currentpose kinematics and independent NumPySVD/lstsq; no source allocator, controller, observer, forecast or physics calls. Exactglobalestimatedpoint usedonlywithonepositiveestimatedpatch.'))
    model=mujoco.MjModel.from_xml_path(str(models[CASES[0]]));data=mujoco.MjData(model)
    geom={label:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,label+'_smooth')for label in LABELS}
    result={}
    with ExitStack()as stack:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_integratePos','mj_contactForce'):
            stack.enter_context(patch.object(mujoco,name,forbidden))
        for case in CASES:
            p=load(INPUT/'campaign/runs'/case/'precision.npz');t=p['time_s'];window=(t>=30.-1e-9)&(t<=34.+1e-9);ii=np.flatnonzero(window)
            raw=p['exact_native_requested'][:,:3];scale=np.array([1.5,.6,.6]);jump=np.zeros(len(t));jump[1:]=np.max(abs(np.diff(raw,axis=0))/scale,axis=1)
            selections={int(np.argmin(abs(t-now))):['fixedtime']for now in (31.645,31.650,31.655,31.660)}
            for label,index in [('max_raw_torque',ii[np.argmax(np.max(abs(raw[ii])/scale,axis=1))]),('max_raw_torque_jump',ii[np.argmax(jump[ii])])]:
                selections.setdefault(int(index),[]).append(label)
            rows=[]
            for i,kinds in sorted(selections.items()):
                loads=p['exact_estimated_support_loads_N'][i];loaded=np.flatnonzero(loads>0)
                row=dict(index=i,time_s=t[i],selection=kinds,estimated_loads_N=loads,truth_loads_N=p['truth_patch_loads_N'][i],
                    saved_drive_references=p['exact_allocation_drive_reference'][i],raw_requested=p['exact_native_requested'][i],applied=p['exact_applied'][i],
                    sync_weights=p['exact_sync_weights'][i],transition_delta=p['exact_transition_command_delta'][i],
                    lqr=p['exact_lqr_command'][i],actual_joint_rates_rad_s=p['exact_qvel'][i,6:9])
                if len(loaded)!=1:
                    row['eligible_single_patch_reconstruction']=False;row['reason']='Multiple/zero estimatedpatches; no exactperrecordpoint reconstruction fromglobalmean';rows.append(row);continue
                label=LABELS[int(loaded[0])];g=geom[label];body=int(model.geom_bodyid[g]);point=p['exact_estimated_contact_point_m'][i]
                data.qpos[:]=p['exact_qpos'][i];mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
                jac=np.zeros((3,model.nv));mujoco.mj_jac(model,data,jac,None,point,body)
                full=np.zeros((2,5));full[:,:3]=jac[:2,6:9]
                if label.startswith('belt'):
                    other=3 if label.endswith('A')else4;full[:,other]=data.geom_xmat[g].reshape(3,3)[:2,1]
                else:other=1 if label.endswith('A')else2
                columns=[0,other];matrix=full[:,columns];normalized=full*SCALE
                rhs=-p['exact_reference_velocity_m_s'][i]
                singular=np.linalg.svd(matrix*SCALE[columns],compute_uv=False)
                reconstructed=SCALE*np.linalg.lstsq(normalized,rhs,rcond=1e-7)[0]
                saved=p['exact_allocation_drive_reference'][i]
                norms=np.linalg.norm(matrix,axis=0);cosine=float(matrix[:,0]@matrix[:,1]/np.prod(norms))if np.prod(norms)>0 else np.nan
                row.update(eligible_single_patch_reconstruction=True,loaded_patch=label,point_m=point,active_command_columns=columns,
                    D_world_xy=matrix,D_native_normalized=matrix*SCALE[columns],
                    singular_values_native_normalized_m_s=singular,condition_native_normalized=float(singular[0]/singular[-1]),
                    normalized_direction_cosine=cosine,acute_drive_direction_angle_deg=np.degrees(np.arccos(np.clip(abs(cosine),0,1))),
                    physical_column_levers_or_axis_norm=norms,rhs_m_s=rhs,reconstructed_drive_references=reconstructed,
                    saved_minus_reconstructed=saved-reconstructed,max_abs_saved_minus_reconstructed=np.max(abs(saved-reconstructed)),
                    independent_residual_norm_m_s=np.linalg.norm(full@reconstructed-rhs),saved_residual_norm_m_s=np.linalg.norm(full@saved-rhs),
                    exact_point_vs_savedfloat32_patchpoint_m='Exactprecisionglobalpointused; oneSupportLocatorrecord for this solepatch.')
                rows.append(row)
            result[case]=rows
    save(output/'report.json',dict(cases=result,scope='Current estimatedsinglepatchrolling allocation matrix only; real contacttruth appears solelyasevaluation metadata.',
        convention='Normalizedcolumns use nativepreferences[5,5,5,.3,.3], which are allocation scales, not joint/motor limits. Shellcolumns have unitsm/rad; beltcolumn dimensionless; normalizedSVD avoidsmixedunitconditionclaim.',
        limitation='SVD/least-squares arithmetic independentlyreproduces nativeform fromexactq/point; futurecontact/rimrecords/dynamicresponse andcauseofallpathdeviation are notreconstructed.'))
    for rel,expected in inputs.items():assert sha(ROOT/rel)==expected
    for row in records:assert sha(ROOT/row['origin'])==row['sha256']
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(f.relative_to(output)),bytes=f.stat().st_size,sha256=sha(f))for f in sorted(output.rglob('*'))if f.is_file()and f.name!='artifact_manifest.json']))
    print(json.dumps(safe({case:[{k:row.get(k)for k in ('time_s','selection','loaded_patch','condition_native_normalized','acute_drive_direction_angle_deg','saved_drive_references','reconstructed_drive_references','max_abs_saved_minus_reconstructed')}for row in rows]for case,rows in result.items()}),indent=2))


if __name__=='__main__':main()
