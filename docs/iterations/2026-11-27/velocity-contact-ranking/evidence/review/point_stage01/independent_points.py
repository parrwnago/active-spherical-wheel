"""Post-primary independent point statistics and selected-coordinate checks."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import patch
import numpy as np

MODES=('geometry','wrench','velocity','fused')
TARGETS=('equivalent_raw','dominant_patch_raw','equivalent_radial_R_proxy','dominant_patch_radial_R_proxy')
SUBSETS=('all_matched','accepted','correct_family','strict_one_physical_point')
FAMILIES=('shell_A','shell_B','belt_A','belt_B')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    with np.load(path,allow_pickle=False)as z:return {key:z[key].copy()for key in z.files}


def summarize_values(distance):
    if len(distance)==0:return dict(count=0,mean_mm=None,RMS_mm=None,median_mm=None,p95_mm=None,maximum_mm=None)
    d=1000.*np.asarray(distance)
    return dict(count=len(d),mean_mm=float(np.mean(d)),RMS_mm=float(math.sqrt(float(d@d)/len(d))),
                median_mm=float(np.quantile(d,.5)),p95_mm=float(np.quantile(d,.95)),maximum_mm=float(max(d)))


def independent_statistics(points):
    n=points['native_point_m'];p=points['candidate_point_m'];target=points['targets_m']
    primary=points['primary_eligible'];out={}
    nvalid=np.isfinite(n).all(axis=1)
    for m,mode in enumerate(MODES):
        out[mode]={};pvalid=np.isfinite(p[:,m]).all(axis=1)
        for j,name in enumerate(TARGETS):
            tvalid=np.isfinite(target[:,j]).all(axis=1);matched=primary&nvalid&pvalid&tvalid
            masks=[matched,matched&points['accepted'][:,m],
                   matched&(points['chosen_family'][:,m]==points['truth_family']),
                   matched&points['strict_one_physical_point']]
            out[mode][name]={}
            for subset,mask in zip(SUBSETS,masks):
                nd=np.sqrt(np.sum((n[mask]-target[mask,j])**2,axis=1))
                pd=np.sqrt(np.sum((p[mask,m]-target[mask,j])**2,axis=1))
                move=np.sqrt(np.sum((p[mask,m]-n[mask])**2,axis=1))
                out[mode][name][subset]=dict(primary_eligible_count=int(primary.sum()),matched_count=int(mask.sum()),
                    missing_native_on_primary=int(np.sum(primary&~nvalid)),missing_candidate_on_primary=int(np.sum(primary&~pvalid)),
                    missing_target_on_primary=int(np.sum(primary&~tvalid)),native=summarize_values(nd),candidate=summarize_values(pd),
                    movement_from_native=summarize_values(move),
                    candidate_minus_native_mean_mm=float(1000.*(pd.mean()-nd.mean()))if mask.any()else None)
    return out


def compare(actual,expected,path=''):
    if isinstance(expected,dict):
        assert set(actual)==set(expected),path
        return max((compare(actual[k],expected[k],path+'/'+k)for k in expected),default=0.)
    if isinstance(expected,float):
        assert math.isclose(actual,expected,rel_tol=1e-12,abs_tol=1e-10),(path,actual,expected)
        return abs(actual-expected)
    assert actual==expected,(path,actual,expected)
    return 0.


def tangent_basis(n):
    e=np.eye(3)[np.argmin(abs(n))];a=np.cross(n,e);a/=np.linalg.norm(a)
    return a,np.cross(n,a)


def projection(proposal,axis,R,lower,upper):
    component=float(axis@proposal);axial=float(np.clip(component,lower,upper))
    radial=proposal-component*axis;length=float(np.linalg.norm(radial))
    direction=radial/length if length>1e-12 else tangent_basis(axis)[0]
    return axial*axis+math.sqrt(max(R*R-axial*axial,0.))*direction


def selected_point(model,data,centre,normal,index,label,mujoco):
    n=normal/np.linalg.norm(normal);a,b=tangent_basis(n);seed,kind=divmod(index,6)
    if seed==0:hint=n
    else:
        ring=1 if seed<=8 else 2;theta=math.radians(5.*ring)
        angle=((seed-1)%8)*math.pi/4
        hint=math.cos(theta)*n+math.sin(theta)*(math.cos(angle)*a+math.sin(angle)*b)
    assert (0,1,2,3,0,1)[kind]==label
    geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,FAMILIES[label]+'_smooth')
    sign,lower,upper,_,R=model.geom_user[geom,:5]
    if kind>=4:upper=lower
    body=int(model.geom_bodyid[geom]);axis=sign*data.xmat[body].reshape(3,3)[:,0]
    return centre+projection(-R*hint,axis,R,lower,upper)


def native_equivalent(model,data,centre,normal,mujoco):
    entries=[]
    for family in FAMILIES:
        geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,family+'_smooth')
        sign,lower,upper,_,R=model.geom_user[geom,:5]
        axis=data.xmat[int(model.geom_bodyid[geom])].reshape(3,3)[:,0]
        entries.append((sign,lower,upper,R,axis))
    support=-entries[0][3]*normal
    for sign,lower,upper,R,axis in entries:
        if lower-1e-12<=sign*(axis@support)<=upper+1e-12:return centre+support
    axis=entries[0][4];axial=float(axis@support);radial=support-axial*axis
    direction=radial/np.linalg.norm(radial)
    xs=[sign*lower*float(each_axis@axis)for sign,lower,upper,R,each_axis in entries[:2]]
    share=float(np.clip((axial-xs[1])/(xs[0]-xs[1]),0.,1.))
    points=[centre+x*axis+math.sqrt(max(entries[0][3]**2-x*x,0.))*direction for x in xs]
    return share*points[0]+(1.-share)*points[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--point-root',type=Path,required=True)
    parser.add_argument('--features-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    saved=json.loads((args.point_root/'report.json').read_text());all_points=[];checks={};geometry={}
    for case in saved['cases']:
        p=load(args.point_root/(case+'_selected_points.npz'));all_points.append(p)
        checks[case]={'statistics_max_abs_difference':compare(independent_statistics(p),saved['cases'][case])}
        # Coordinate controls are explicitly post-primary: fixed 1 s plus the
        # largest fused/raw target-error row, not a new performance screen.
        distance=np.linalg.norm(p['candidate_point_m'][:,3]-p['targets_m'][:,0],axis=1)
        worst=int(np.argmax(np.where(p['primary_eligible'],distance,-np.inf)))
        rows=sorted({200,worst});context=load(args.features_root/case/'measurement_context.npz')
        registration=json.loads((args.features_root/case/'registered_before_arithmetic.json').read_text())
        xml=next(Path(x['path'])for x in registration['inputs']if Path(x['path']).name=='model.xml')
        import mujoco
        model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
        centre_body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'fork');g=[]
        def forbidden(*a,**k):raise RuntimeError('No dynamics, contact or physics allowed')
        with ExitStack()as guards:
            for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_collision','mj_contactForce'):
                guards.enter_context(patch.object(mujoco,name,side_effect=forbidden))
            for row in rows:
                data.qpos[:]=context['exact_qpos'][row];data.qpos[3:7]/=np.linalg.norm(data.qpos[3:7])
                mujoco.mj_kinematics(model,data);centre=data.xpos[centre_body].copy()
                hint=context['intrinsic_normal_hint'][row]
                np.testing.assert_allclose(centre,p['centre_m'][row],atol=1e-12,rtol=1e-12)
                errors=[]
                for m in range(4):
                    candidate=selected_point(model,data,centre,hint,int(p['original_candidate_index'][row,m]),int(p['chosen_family'][row,m]),mujoco)
                    np.testing.assert_allclose(candidate,p['candidate_point_m'][row,m],atol=1e-12,rtol=1e-12)
                    errors.append(float(np.max(abs(candidate-p['candidate_point_m'][row,m]))))
                native=native_equivalent(model,data,centre,hint,mujoco)
                np.testing.assert_allclose(native,p['native_point_m'][row],atol=1e-10,rtol=1e-10)
                g.append(dict(row=row,time_s=float(p['time_s'][row]),candidate_max_abs_m=max(errors),native_max_abs_m=float(np.max(abs(native-p['native_point_m'][row])))))
        geometry[case]=g
    pooled={key:np.concatenate([p[key]for p in all_points])for key in all_points[0]}
    aggregate=independent_statistics(pooled);aggregate_error=compare(aggregate,saved['aggregate'])
    output=dict(iteration='I-60',completed_utc=datetime.now(timezone.utc).isoformat(),new_physics_runs=0,
        case_checks=checks,aggregate_max_abs_statistics_difference=aggregate_error,coordinate_controls=geometry,
        fused_raw_equivalent=aggregate['fused']['equivalent_raw'],all_checks_pass=True,
        source_review='Point source reconstructs original selected candidate IDs with frozen private geometry, not a new rank or truth-generated candidate.',
        crucial_qualification='The historical key strict_one_physical_point actually means one resolved >0.05N solver record. Weaker contacts may remain in the load-weighted target. No exact single-physical-point claim.',
        original_truth_capture_source='I58 frozen scripts/run-lqr-contact-forecast.py: counts[k] += int(force > .05)',
        limits='Post-primary descriptive point statistics and selected-coordinate controls, not a closed-loop improvement or global contact-geometry guarantee.')
    args.output.write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output))


if __name__=='__main__':main()
