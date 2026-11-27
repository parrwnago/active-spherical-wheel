"""Post-primary independent geometry/direct-point-Jacobian checks; no helper."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import patch
import mujoco
import numpy as np

FAMILIES=('shell_A','shell_B','belt_A','belt_B')
SAMPLE_TIMES=(0.,1.,6.4,7.4,31.4,32.6,35.4,36.4,44.)


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    with np.load(path,allow_pickle=False)as z:return {key:z[key].copy()for key in z.files}


def direction(force,moment,radius):
    size=float(force@force)
    if size<1e-12:return None
    lateral=-np.cross(force,moment)/(radius*size)
    excess=float(lateral@lateral)
    if excess>=1.:return lateral/math.sqrt(excess)
    normal=math.sqrt(1.-excess)*force/math.sqrt(size)+lateral
    return normal/np.linalg.norm(normal)


def basis(normal):
    reference=np.eye(3)[int(np.argmin(abs(normal)))]
    first=np.cross(normal,reference);first/=np.linalg.norm(first)
    return first,np.cross(normal,first)


def independently_evaluate(model,data,centre_body,qpos,qvel,previous,wrench,cone,minimum):
    radius=float(model.geom_user[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'shell_A_smooth'),4])
    force,moment=wrench[:3],wrench[3:]
    normal=direction(force,moment,radius)
    zero=dict(valid=np.zeros(102,bool),frame_valid=False,
              normal_angle_rad=np.zeros(102),velocity_residual_m_s=np.zeros((102,3)),
              velocity_residual_norm_m_s=np.zeros(102),moment_residual_norm=np.zeros(102))
    if (normal is None or not np.isfinite(qpos).all()or not np.isfinite(qvel).all()
            or not np.isfinite(previous).all()or not np.isfinite(wrench).all()
            or np.linalg.norm(qpos[3:7])<1e-12 or np.linalg.norm(force)<minimum
            or force@normal<minimum):return zero,normal
    data.qpos[:]=qpos;data.qpos[3:7]/=np.linalg.norm(data.qpos[3:7]);data.qvel[:]=qvel
    mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
    centre=data.xpos[centre_body].copy()
    first,second=basis(normal)
    seeds=[normal]
    for theta in (.5*cone,cone):
        for i in range(8):
            tangent=math.cos(i*math.pi/4)*first+math.sin(i*math.pi/4)*second
            seeds.append(math.cos(theta)*normal+math.sin(theta)*tangent)
    for seed_number,seed in enumerate(seeds):
        for kind,family in enumerate((0,1,2,3,0,1)):
            index=6*seed_number+kind
            geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,FAMILIES[family]+'_smooth')
            sign,lower,upper,_,R=model.geom_user[geom,:5]
            if kind>=4:upper=lower
            body=int(model.geom_bodyid[geom]);axis=sign*data.xmat[body].reshape(3,3)[:,0]
            proposal=-R*seed;axial=float(axis@proposal)
            radial=proposal-axial*axis
            length=float(np.linalg.norm(radial))
            radial=radial/length if length>1e-12 else basis(axis)[0]
            clipped=float(np.clip(axial,lower,upper))
            offset=clipped*axis+math.sqrt(max(float(R*R-clipped*clipped),0.))*radial
            n=-offset/R
            angle=float(np.arctan2(np.linalg.norm(np.cross(normal,n)),np.clip(normal@n,-1.,1.)))
            valid=(lower-1e-12<=axis@offset<=upper+1e-12 and angle<=cone+1e-12 and force@n>=minimum)
            if not valid:continue
            # Direct candidate-point Jacobian, unlike the tested centre-plus-twist formula.
            jac=np.zeros((3,model.nv));mujoco.mj_jac(model,data,jac,None,centre+offset,body)
            material=jac@qvel
            if family>=2:material+=data.geom_xmat[geom].reshape(3,3)[:,1]*previous[family+1]
            residual=material-n*float(n@material)
            zero['valid'][index]=True
            zero['normal_angle_rad'][index]=angle
            zero['velocity_residual_m_s'][index]=residual
            zero['velocity_residual_norm_m_s'][index]=np.linalg.norm(residual)
            zero['moment_residual_norm'][index]=np.linalg.norm(moment-np.cross(offset,force))/(R*np.linalg.norm(force))
    zero['frame_valid']=True
    return zero,normal


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--features-root',type=Path,required=True)
    parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    protocol=json.loads(args.protocol.read_text());cases={}
    for case in protocol['datasets']:
        folder=args.features_root/case
        context=load(folder/'measurement_context.npz');saved=load(folder/'projected_features.npz')
        registration=json.loads((folder/'registered_before_arithmetic.json').read_text())
        metadata=json.loads((folder/'metadata.json').read_text())
        xml=next(Path(item['path'])for item in registration['inputs']if Path(item['path']).name=='model.xml')
        assert sha(xml)==metadata['model_xml_sha256']
        model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
        model.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        centre=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'fork')
        errors={key:0. for key in ('normal_angle_rad','velocity_residual_m_s','velocity_residual_norm_m_s','moment_residual_norm')}
        rows=[]
        def forbidden(*a,**k):raise RuntimeError('No physics, contact or dynamics query allowed')
        with ExitStack()as guards:
            for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_collision','mj_contactForce'):
                guards.enter_context(patch.object(mujoco,name,side_effect=forbidden))
            for seconds in SAMPLE_TIMES:
                row=int(round(seconds/.005));assert abs(context['time_s'][row]-seconds)<1e-8
                inputs=[context[key][row].copy()for key in ('exact_qpos','exact_qvel','exact_previous_applied','exact_observer_wrench')]
                original=[x.copy()for x in inputs]
                expected,normal=independently_evaluate(model,data,centre,*inputs,
                    math.radians(protocol['candidate_search']['normal_cone_deg']),protocol['candidate_search']['minimum_compressive_force_N'])
                assert all(np.array_equal(a,b,equal_nan=True)for a,b in zip(inputs,original))
                np.testing.assert_allclose(normal,context['intrinsic_normal_hint'][row],rtol=1e-12,atol=1e-12)
                np.testing.assert_array_equal(expected['valid'],saved['valid'][row])
                assert expected['frame_valid']==saved['frame_valid'][row]
                for key in errors:
                    difference=float(np.max(abs(expected[key]-saved[key][row])))
                    errors[key]=max(errors[key],difference)
                    np.testing.assert_allclose(expected[key],saved[key][row],rtol=1e-10,atol=1e-10)
                rows.append(dict(row=row,time_s=float(context['time_s'][row]),valid_candidates=int(expected['valid'].sum())))
        cases[case]=dict(samples=rows,max_abs_difference=errors,model_sha256=sha(xml),
                        candidate_validity_and_frame_flags_exact=True)
    report=dict(iteration='I-60',completed_utc=datetime.now(timezone.utc).isoformat(),cases=cases,
                sampled_rows_per_case=len(SAMPLE_TIMES),new_physics_runs=0,all_checks_pass=True,
                scope='Post-primary independent geometry/direct-point-Jacobian sample check; no helper import; fixed times from protocol boundaries, no outcome-based sample selection.',
                limits='Finite recorded samples and radial normal/rim assumptions; not an independent dynamic/contact solver or validation of actual physical truth.')
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
