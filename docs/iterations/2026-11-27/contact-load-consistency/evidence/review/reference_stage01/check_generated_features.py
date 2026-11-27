"""Independent full-rate I-61 force/lever/axis, veto and policy audit."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from unittest.mock import patch
import mujoco
import numpy as np
from analytic_load_reference import point_load, joint_gate

FAMILIES=('shell_A','shell_B','belt_A','belt_B')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    with np.load(path,allow_pickle=False)as z:return {key:z[key].copy()for key in z.files}


def compare(a,b):
    np.testing.assert_allclose(a,b,atol=1e-10,rtol=1e-12)
    finite=np.isfinite(a)&np.isfinite(b)
    return float(np.max(abs(np.asarray(a)[finite]-np.asarray(b)[finite])))if finite.any()else 0.


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--features-root',type=Path,required=True)
    p.add_argument('--i60-features-root',type=Path,required=True)
    p.add_argument('--i60-point-root',type=Path,required=True)
    p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    protocol=json.loads(a.protocol.read_text());cases={}
    abs_tol=protocol['primary_veto']['absolute_tolerance_Nm'];rel_tol=protocol['primary_veto']['relative_force_lever_tolerance']
    for case in protocol['datasets']:
        folder=a.features_root/case
        for item in json.loads((folder/'artifact_manifest.json').read_text())['files']:
            assert sha(folder/item['path'])==item['sha256'],item['path']
        f=load(folder/'load_features_and_policies.npz');old=a.i60_features_root/case
        q=load(old/'measurement_context.npz');scores=load(old/'scores.npz')
        # Explicit whitelist even in this after-feature-seal parity control.
        with np.load(a.i60_point_root/(case+'_selected_points.npz'),allow_pickle=False)as z:
            old_points=z['candidate_point_m'][:,3].copy()
        n=len(f['time_s']);np.testing.assert_array_equal(f['time_s'],q['time_s'])
        np.testing.assert_array_equal(f['i60_accepted'],scores['fused__confident'])
        np.testing.assert_array_equal(f['proposed_family'],scores['fused__chosen_family'])
        np.testing.assert_array_equal(f['observer_generalized'],q['exact_observer_generalized'])
        np.testing.assert_array_equal(f['observer_wrench'],q['exact_observer_wrench'])
        oldpoint_difference=compare(f['proposed_point_m'],old_points)
        registration=json.loads((folder/'registered_before_arithmetic.json').read_text())
        xml=next(Path(item['path'])for item in registration['inputs']if Path(item['path']).name=='model.xml')
        model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
        model.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        centre_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'fork')
        joint_ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,key)for key in ('shaft','shell_A_joint','shell_B_joint')]
        predictions={key:np.full((n,9),np.nan)for key in ('proposed','native_alternative')}
        validity={key:np.zeros(n,bool)for key in predictions}
        map_base=np.full((n,4,3),np.nan);map_moment=np.full((n,4,3,3),np.nan);centres=np.full((n,3),np.nan)
        map_valid=np.zeros(n,bool)
        def forbidden(*x,**kw):raise RuntimeError('No point Jacobian, dynamics, contact or physics call permitted')
        with ExitStack()as guards:
            for name in ('mj_jac','mj_step','mj_step1','mj_step2','mj_forward','mj_collision','mj_contactForce'):
                guards.enter_context(patch.object(mujoco,name,side_effect=forbidden))
            for i in range(n):
                pose=q['exact_qpos'][i];force=f['observer_wrench'][i,:3];gamma=f['observer_generalized'][i]
                finite=(np.isfinite(pose).all()and np.linalg.norm(pose[3:7])>1e-12
                        and np.isfinite(f['observer_wrench'][i]).all()and np.linalg.norm(force)>=1.)
                if not finite:continue
                data.qpos[:]=pose;data.qpos[3:7]/=np.linalg.norm(data.qpos[3:7])
                mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
                centre=data.xpos[centre_id].copy();centres[i]=centre;map_valid[i]=True
                for family in range(4):
                    axes=np.stack([data.xaxis[j]for j in joint_ids]);anchors=np.stack([data.xanchor[j]for j in joint_ids])
                    active=np.array([True,family==0,family==1])
                    map_moment[i,family]=np.where(active[:,None],axes,0.)
                    scalar=np.sum(axes*np.cross(centre-anchors,force),axis=1)
                    map_base[i,family]=np.where(active,scalar,0.)
                for prefix,point_key,label_key in (('proposed','proposed_point_m','proposed_family'),
                                                  ('native_alternative','native_alternative_point_m','native_family')):
                    label=int(f[label_key][i]);point=f[point_key][i]
                    if not 0<=label<4 or not np.isfinite(point).all()or not np.isfinite(gamma).all():continue
                    if not np.array_equal(f['observer_wrench'][i],gamma[:6]):continue
                    geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,FAMILIES[label]+'_smooth')
                    sign,lower,upper,_,R=model.geom_user[geom,:5]
                    offset=point-centre;axis=sign*data.xmat[int(model.geom_bodyid[geom])].reshape(3,3)[:,0]
                    if abs(np.linalg.norm(offset)-R)>1e-9 or not lower-1e-9<=axis@offset<=upper+1e-9:continue
                    validity[prefix][i]=True
                    predictions[prefix][i]=point_load(model,data,point,label,force)
        differences={}
        for prefix in predictions:
            np.testing.assert_array_equal(validity[prefix],f[prefix+'__valid'])
            differences[prefix+'_prediction']=compare(predictions[prefix],f[prefix+'__prediction'])
            residual=f['observer_generalized']-predictions[prefix]
            differences[prefix+'_residual']=compare(residual,f[prefix+'__residual'])
            norm=np.linalg.norm(f['observer_wrench'][:,:3],axis=1);radius=f[prefix+'__radius_m']
            tau=abs_tol+rel_tol*radius*norm
            pass_flag=np.array([validity[prefix][i]and joint_gate(predictions[prefix][i],f['observer_generalized'][i],norm[i],radius[i],absolute_tolerance_Nm=abs_tol,relative_tolerance=rel_tol)for i in range(n)])
            np.testing.assert_array_equal(pass_flag,f[prefix+'_gate__passes'])
            np.testing.assert_array_equal(validity[prefix]&~pass_flag,f[prefix+'_gate__vetoed'])
            np.testing.assert_array_equal(~validity[prefix],f[prefix+'_gate__unknown'])
            differences[prefix+'_threshold']=compare(tau,f[prefix+'_gate__threshold_Nm'])
            differences[prefix+'_ratio']=compare(residual[:,6:9]/tau[:,None],f[prefix+'_gate__normalized_joint_residual'])
        np.testing.assert_array_equal(map_valid,f['map__valid'])
        differences['map_centre']=compare(centres,f['map__centre_m'])
        differences['map_base']=compare(map_base,f['map__joint_at_centre'])
        differences['map_moment']=compare(map_moment,f['map__joint_moment_map'])
        use_a=f['i60_accepted'];use_c=use_a&f['proposed_gate__passes']
        for prefix,use in [('A',use_a),('C',use_c)]:
            np.testing.assert_array_equal(np.where(use,f['proposed_family'],f['native_family']),f[prefix+'_family'])
            np.testing.assert_array_equal(np.where(use[:,None],f['proposed_point_m'],f['native_point_m']),f[prefix+'_point_m'])
            np.testing.assert_array_equal(use,f[prefix+'_uses_proposal'])
        np.testing.assert_array_equal(f['N_family'],f['native_family'])
        np.testing.assert_array_equal(f['N_point_m'],f['native_point_m'])
        cases[case]=dict(rows=n,max_abs_differences=differences,all_validity_gate_and_policy_flags_exact=True,
                         frozen_I60_F_point_max_difference_m=oldpoint_difference,
                         source_features_sha256=sha(folder/'load_features_and_policies.npz'))
    report=dict(iteration='I-61',completed_utc=datetime.now(timezone.utc).isoformat(),cases=cases,all_checks_pass=True,
                new_physics_runs=0,scope='All-row independent analytic force/lever/hinge-axis loads, componentwise veto, affine oracle maps and exact N/A/C policy selection; no point Jacobian or new helper imports.')
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
