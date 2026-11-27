"""I72 truth/baseline projection only after all three rankings are sealed.

Inference never reads this output.  Original native equivalent points remain
distinct from every newly selected physical candidate point.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
import numpy as np

CASES=('m100_B','p030_B','p090_B')
BASE_FIELDS=('time_s','truth_patch_loads_N','truth_patch_points_m','N_family','F_family','A_family',
    'i60_accepted','N_points_m','F_points_m','A_points_m')
OLD_FIELDS=('time_s','primary_mask','truth_eligible','truth_family','native_family',
    'fused__chosen_family','fused__accepted_family','valid_family_count','measured_excitation_m_s')
RANK_FIELDS=('family_scores','candidate_index','score_margin','valid_family_count','confident','score_tied',
    'geometry_forced','best_velocity_residual_m_s','residual_separation_m_s','excitation_m_s')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path,fields=None):
    with np.load(path,allow_pickle=False) as p:return {k:p[k].copy() for k in (p.files if fields is None else fields)}


def selected(points,family):
    out=np.full((len(family),3),np.nan)
    valid=(family>=0)&(family<4)
    out[valid]=points[np.flatnonzero(valid),family[valid].astype(int)]
    return out


def project(base,old,pred):
    time=base['time_s']
    if not np.array_equal(time,old['time_s']) or not np.array_equal(time,pred['time_s']):
        raise ValueError('Baseline, old fixed-domain and new inference clocks differ')
    for key,other in [('N_family','native_family'),('F_family','fused__chosen_family')]:
        if not np.array_equal(base[key],old[other]):raise ValueError('Frozen baseline family mismatch')
    if not np.array_equal(base['i60_accepted'],old['fused__accepted_family']>=0):
        raise ValueError('Frozen acceptance mismatch')
    if not np.array_equal(base['A_family'],np.where(base['i60_accepted'],base['F_family'],base['N_family'])):
        raise ValueError('Old A fallback semantics changed')
    output=dict(time_s=time.copy(),truth_patch_loads_N=base['truth_patch_loads_N'].copy(),
        truth_patch_points_m=base['truth_patch_points_m'].copy(),old_accepted=base['i60_accepted'].copy(),
        old_primary_dominant=(old['primary_mask']&old['truth_eligible']).copy(),
        old_truth_family=old['truth_family'].copy(),
        old_ambiguous_motion=(old['primary_mask']&old['truth_eligible']&(old['valid_family_count']>=2)
                              &(old['measured_excitation_m_s']>=.02)),
        old_valid_family_count=old['valid_family_count'].copy(),old_excitation_m_s=old['measured_excitation_m_s'].copy())
    for new,old_name in [('N','N'),('oldF','F'),('A','A')]:
        family=base[old_name+'_family'];output[new+'_family']=family.copy()
        output[new+'_point_m']=selected(base[old_name+'_points_m'],family)
    # Check the actual point as well as family fallback, including same-family updates.
    expected=np.where(base['i60_accepted'][:,None],output['oldF_point_m'],output['N_point_m'])
    if not np.array_equal(expected,output['A_point_m'],equal_nan=True):
        raise ValueError('Old A must retain F physical point or N equivalent point exactly')
    for name,prefix in [('newF','fused'),('newVG','velocity')]:
        family=np.asarray(pred[prefix+'__chosen_family'])
        accepted_family=np.asarray(pred[prefix+'__accepted_family'])
        points=np.asarray(pred[prefix+'__point_m'])
        if family.shape!=(len(time),) or accepted_family.shape!=family.shape or points.shape!=(len(time),3):
            raise ValueError('Expected chosen/accepted families N and new physical point N by3')
        accepted=accepted_family>=0
        if np.any(accepted&(accepted_family!=family)) or np.any(accepted&~np.isfinite(points).all(axis=1)):
            raise ValueError('Accepted candidate must have its chosen family and finite physical point')
        output[name+'_family']=family.copy();output[name+'_point_m']=points.copy();output[name+'_accepted']=accepted
        for key in RANK_FIELDS:
            source=prefix+'__'+key
            if source in pred:output[name+'__'+key]=pred[source].copy()
    for key in ('anchor_available','selected_candidate_id','selected_row_index','family_distance_squared_m2'):
        if key in pred:output[key]=pred[key].copy()
    for alias in ('N_family','native_family'):
        if alias in pred and not np.array_equal(pred[alias],output['N_family']):
            raise ValueError('Runner nativefallback family differs from frozenN')
    known=output['N_family']>=0
    for alias in ('N_point_m','native_point_m'):
        if alias in pred and not np.array_equal(pred[alias][known],output['N_point_m'][known],equal_nan=True):
            raise ValueError('Runner nativefallback point differs from frozenN equivalentpoint')
    cfam=np.where(output['newF_accepted'],output['newF_family'],output['N_family'])
    cpoint=np.where(output['newF_accepted'][:,None],output['newF_point_m'],output['N_point_m'])
    for alias in ('C_family','primaryC_family'):
        if alias in pred and not np.array_equal(pred[alias],cfam):
            raise ValueError('RunnerC family differs from registeredfallback policy')
    if 'C_point_m' in pred and not np.array_equal(pred['C_point_m'][cfam>=0],cpoint[cfam>=0],equal_nan=True):
        raise ValueError('RunnerC point differs from registeredphysical/native-equivalent policy')
    if 'C_used_proposal' in pred and not np.array_equal(pred['C_used_proposal'],output['newF_accepted']):
        raise ValueError('RunnerC proposal-use flag differs from frozenacceptance')
    vgfam=np.where(output['newVG_accepted'],output['newVG_family'],output['N_family'])
    vgpoint=np.where(output['newVG_accepted'][:,None],output['newVG_point_m'],output['N_point_m'])
    if 'secondaryC_VG_family' in pred and not np.array_equal(pred['secondaryC_VG_family'],vgfam):
        raise ValueError('Runner secondaryVG family differs from registeredfallback')
    if 'C_VG_point_m' in pred and not np.array_equal(pred['C_VG_point_m'][vgfam>=0],vgpoint[vgfam>=0],equal_nan=True):
        raise ValueError('Runner secondaryVG point differs from registeredfallback')
    if 'C_VG_used_proposal' in pred and not np.array_equal(pred['C_VG_used_proposal'],output['newVG_accepted']):
        raise ValueError('Runner secondaryVG proposal-use flag differs from frozenacceptance')
    return output


def run(baseline_root,old_evaluation_root,prediction_root,inference_seal,output):
    if not inference_seal.is_file():raise ValueError('All-three-inference-sealed receipt required')
    paths=[Path(__file__),inference_seal]
    for case in CASES:paths += [baseline_root/case/'evaluation_packet.npz',old_evaluation_root/(case+'_evaluation.npz'),prediction_root/case/'predictions.npz']
    bindings=[dict(path=str(p),sha256=sha(p)) for p in paths]
    output.mkdir(parents=True,exist_ok=False)
    (output/'registered_before_truth_read.json').write_text(json.dumps(dict(iteration='I-72',utc=datetime.now(timezone.utc).isoformat(),
        files=bindings,baseline_fields=BASE_FIELDS,old_domain_fields=OLD_FIELDS,
        boundary='Evaluation-only truth; old ambiguous-motion domain fixed before newpoint selection. Newpoints solely from sealedI72outputs.'),indent=2)+'\n')
    shutil.copy2(Path(__file__),output/'prepare_evaluation.py')
    files=[]
    for case in CASES:
        packet=project(load(baseline_root/case/'evaluation_packet.npz',BASE_FIELDS),
                       load(old_evaluation_root/(case+'_evaluation.npz'),OLD_FIELDS),load(prediction_root/case/'predictions.npz'))
        folder=output/case;folder.mkdir();path=folder/'evaluation_packet.npz'
        np.savez_compressed(path,**packet);files.append(dict(case=case,path=str(path.relative_to(output)),sha256=sha(path)))
    for item in bindings:
        if sha(item['path'])!=item['sha256']:raise ValueError('Frozen projection source/input changed')
    (output/'artifact_manifest.json').write_text(json.dumps(dict(files=files,no_new_scoring=True,no_new_physics=True),indent=2)+'\n')
    print(json.dumps(dict(cases=len(files),evaluation_only=True)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('baseline-root','old-evaluation-root','prediction-root','inference-seal','output'):p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();run(a.baseline_root.resolve(),a.old_evaluation_root.resolve(),a.prediction_root.resolve(),a.inference_seal.resolve(),a.output.resolve())
