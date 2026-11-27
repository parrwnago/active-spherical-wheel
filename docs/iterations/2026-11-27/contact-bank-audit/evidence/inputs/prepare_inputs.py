"""I67 lossless ROI projection of previously frozen signals/features/results."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,numpy as np

CASES=('m100_B','p030_B','p090_B')


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}


def run(protocol,i60,i64,i65,i66,output):
    specification=json.loads(protocol.read_text())
    if specification['iteration']!='I-67':raise ValueError('Wrong protocol')
    output.mkdir(parents=True,exist_ok=False)
    science65=json.loads((i65/'analysis/stage01/science.json').read_text())
    science66=json.loads((i66/'analysis/stage01/science.json').read_text())
    files=[protocol,Path(__file__).resolve(),i65/'analysis/stage01/science.json',i66/'analysis/stage01/science.json']
    case_receipts={}
    for case in CASES:
        p60=i60/case
        context_path=p60/'measurement_context.npz';projected_path=p60/'projected_features.npz';scores_path=p60/'scores.npz'
        primitive_path=i64/'features/portable_stage01'/case/'primitives.npz';base_path=i64/'portable_inputs/i62_base'/f'{case}.npz'
        pose_path=i66/'features/stage01'/case/'pose_maps.npz';frames_path=i66/'analysis/stage01'/f'{case}_frames.npz'
        oof_path=i65/'analysis/stage01'/f'{case}_oof.npz'
        files += [context_path,projected_path,scores_path,primitive_path,base_path,pose_path,frames_path,oof_path]
        # Context projection accepts ONLY the old sensor normal and clock.
        with np.load(context_path,allow_pickle=False) as z:context={k:z[k].copy() for k in ('time_s','intrinsic_normal_hint')}
        primitive=load(primitive_path);base=load(base_path);pose=load(pose_path);frames=load(frames_path);oof=load(oof_path)
        t=base['time_s'];n=len(t)
        if n!=8801 or any(not np.array_equal(t,x['time_s']) for x in (context,primitive,pose,frames,oof)):raise ValueError('Original full clocks differ')
        opportunity=frames['cohort_useful']|frames['cohort_harmful'];roi=opportunity.copy();anchor=np.zeros(n,bool)
        events=[]
        for event in science66['events']:
            if event['case']!=case:continue
            i=event['index']
            if abs(t[i]-event['time_s'])>1e-9:raise ValueError('Old event clock differs')
            anchor[i]=True;roi |= abs(t-t[i])<=.100000001;events.append(event)
        indices=np.flatnonzero(roi)
        packet=dict(source_index=indices,time_s=t[indices],event_anchor=anchor[indices],
            intrinsic_normal_hint=context['intrinsic_normal_hint'][indices],
            observer_generalized=primitive['observer_generalized'][indices],observer_interval_mean=primitive['observer_interval_mean'][indices],
            fixed_scale_Nm=frames['fixed_scale_Nm'][indices],truth_family=base['truth_family'][indices],
            sampled_reference_point_m=frames['reference_point_m'][indices],
            primary_eligible=base['primary_eligible'][indices],i60_accepted=base['i60_accepted'][indices],
            N_family=base['N_family'][indices],A_family=base['A_family'][indices],
            correct_branch=frames['correct_branch'][indices],
            old_I65_code=oof['M1__shared_covariance_code'][indices],old_I65_keep=oof['M1__shared_covariance_keep'][indices],
            old_I65_e_F=oof['M1__e_F'][indices],old_I65_e_Nalt=oof['M1__e_Nalt'][indices],
            reference_cf_code=frames['cf_sampled_correct_point_code'][indices],
            reference_cf_F_score=frames['cf_sampled_correct_point_F_score'][indices],
            reference_cf_Nalt_score=frames['cf_sampled_correct_point_Nalt_score'][indices],
            reference_cf_applicable=frames['cf_applicable'][indices],
            reference_residual_Nm=frames['T_sampled_point_residual_Nm'][indices])
        for name in ('centre_m','linear_at_centre','angular_world','valid','patch_axis_world','hinge_axis_world','hinge_anchor_m'):
            packet[name]=pose[name][indices]
        for name in ('radius_m','patch_lower_m','patch_upper_m','ancestor_active'):packet[name]=pose[name]
        for name in ('focus','useful','harmful','retained_useful'):packet['cohort_'+name]=frames['cohort_'+name][indices]
        for branch in ('F','Nalt'):
            family_key='proposed_family' if branch=='F' else 'native_family'
            id_key='proposed_candidate_index' if branch=='F' else 'native_candidate_index'
            packet['saved_'+branch+'_family']=primitive[family_key][indices]
            packet['saved_'+branch+'_index']=primitive[id_key][indices]
            for name in ('point_'+branch+'_m','H_'+branch,'H_'+branch+'_valid'):packet['saved_'+name]=primitive[name][indices]
        with np.load(projected_path,allow_pickle=False) as z:
            for name in z.files:
                v=z[name];packet['old__'+name]=v[indices] if v.ndim and len(v)==n else v.copy()
        with np.load(scores_path,allow_pickle=False) as z:
            for name in ('fused__family_scores','fused__candidate_index','fused__chosen_family','fused__confident'):
                packet['old_'+name]=z[name][indices]
        model=science65['models'][case]['M1']['fitted']
        for name in ('mean','covariance','cholesky'):packet['frozen_'+name]=np.asarray(model[name],float)
        case_path=output/(case+'.npz');np.savez_compressed(case_path,**packet)
        case_receipts[case]=dict(roi_rows=len(indices),full_rows=n,opportunities=int(opportunity.sum()),
            useful=int(frames['cohort_useful'].sum()),harmful=int(frames['cohort_harmful'].sum()),focus=int(frames['cohort_focus'].sum()),
            event_count=len(events),events=[dict(index=e['index'],time_s=e['time_s'],reasons=e['reasons']) for e in events],
            packet_sha256=sha(case_path))
    receipt=dict(iteration='I-67',utc=datetime.now(timezone.utc).isoformat(),scope='Lossless known-array/ROI projection only; no new bank geometry, distance, capacity score or fitting.',
                 files=[dict(path=str(p),sha256=sha(p)) for p in files],cases=case_receipts)
    (output/'manifest.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(case_receipts,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('protocol','i60','i64','i65','i66','output'):p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();run(a.protocol,a.i60,a.i64,a.i65,a.i66,a.output)
