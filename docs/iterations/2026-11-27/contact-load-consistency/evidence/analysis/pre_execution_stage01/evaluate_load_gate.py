"""I-61 pure-NumPy truth evaluation, after all inference policies are sealed."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import math
import shutil
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
CASES=('m100_B','p030_B','p090_B')
POLICIES=('N','A','C')
TARGETS=('equivalent_raw','dominant_patch_raw','equivalent_radial_R_proxy','dominant_patch_radial_R_proxy')
SUBSETS=('all','native_correct','A_accepted','C_retained','stable','one_resolved_contact_gt005N')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v) for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float) and not math.isfinite(x):return None
    return x
def save(path,x):Path(path).write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False) as z:return {name:z[name].copy() for name in z.files}
def exact(a,b):return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
def ratio(a,b):return float(a/b) if b else None


def mask_window(t,window):return (t>=window[0]-1e-9)&(t<window[1]-1e-9)


def dominant(loads,minimum_total,minimum_share):
    finite=np.isfinite(loads).all(axis=1)&(loads>=0).all(axis=1)
    clean=np.where(np.isfinite(loads),loads,0.);total=clean.sum(axis=1);maximum=clean.max(axis=1)
    unique=(clean==maximum[:,None]).sum(axis=1)==1
    share=np.divide(maximum,total,out=np.zeros_like(total),where=total>0)
    loaded=finite&(total>=minimum_total)&(maximum>0)
    eligible=loaded&unique&(share>=minimum_share)
    return np.where(eligible,np.argmax(clean,axis=1),-1),eligible,loaded,share


def stable_mask(label,eligible):
    mask=np.zeros(len(label),bool)
    for i in range(2,len(label)-2):
        sl=slice(i-2,i+3);mask[i]=eligible[sl].all() and (label[sl]==label[i]).all()
    return mask


def radial_proxy(points,centre,radius):
    offset=points-centre;length=np.linalg.norm(offset,axis=1)
    good=np.isfinite(points).all(axis=1)&np.isfinite(centre).all(axis=1)&(length>1e-12)
    result=np.full_like(points,np.nan);result[good]=centre[good]+radius[good,None]*offset[good]/length[good,None]
    return result


def statistics(distance_m):
    if not len(distance_m):return dict(count=0,mean_mm=None,RMS_mm=None,median_mm=None,p95_mm=None,maximum_mm=None)
    x=distance_m*1000.
    return dict(count=len(x),mean_mm=float(x.mean()),RMS_mm=float(np.sqrt(np.mean(x*x))),
        median_mm=float(np.median(x)),p95_mm=float(np.percentile(x,95)),maximum_mm=float(x.max()))


def verify_policies(pack,protocol):
    p=protocol['primary_veto']
    threshold=p['absolute_tolerance_Nm']+p['relative_force_lever_tolerance']*pack['proposed__radius_m']*pack['proposed__force_norm_N']
    residual=pack['proposed__residual'];valid=pack['proposed__valid']&np.isfinite(residual).all(axis=1)
    valid &= np.isfinite(threshold)&(threshold>0)
    passes=valid&(np.max(abs(residual[:,6:9])/threshold[:,None],axis=1)<=1.)
    if not np.array_equal(passes,pack['proposed_gate__passes']):raise ValueError('Gate flags differ from sealed primary formula')
    if not np.array_equal(threshold,pack['proposed_gate__threshold_Nm']):raise ValueError('Gate tolerance differs')
    A=pack['i60_accepted'];C=A&passes
    for policy,selected in (('A',A),('C',C)):
        label=np.where(selected,pack['proposed_family'],pack['native_family'])
        point=np.where(selected[:,None],pack['proposed_point_m'],pack['native_point_m'])
        if not exact(label,pack[policy+'_family']) or not exact(point,pack[policy+'_point_m']):
            raise ValueError('Frozen policy formula mismatch: '+policy)
    return True


def make_frames(pack,truth,protocol):
    e=protocol['evaluation'];t=pack['time_s']
    if not exact(t,truth['truth_time_s']):raise ValueError('Truth/feature time mismatch')
    family,eligible,loaded,share=dominant(truth['truth_patch_loads_N'],e['truth_total_min_N'],e['truth_dominant_fraction_min'])
    safe_family=np.maximum(family,0);n=len(t);rows=np.arange(n)
    dominant_point=truth['truth_patch_points_m'][rows,safe_family].copy()
    dominant_point[~eligible]=np.nan
    force=pack['observer_wrench'][:,:3]
    base=pack['map__joint_at_centre'][rows,safe_family]
    angular=pack['map__joint_moment_map'][rows,safe_family]
    moment=np.cross(dominant_point-pack['map__centre_m'],force)
    oracle_prediction=base+np.einsum('nij,nj->ni',angular,moment)
    oracle_residual=pack['observer_generalized'][:,6:9]-oracle_prediction
    oracle_valid=eligible&pack['map__valid']&np.isfinite(dominant_point).all(axis=1)
    oracle_valid &= np.isfinite(oracle_prediction).all(axis=1)&np.isfinite(oracle_residual).all(axis=1)
    tolerance=pack['proposed_gate__threshold_Nm']
    oracle_ratio=oracle_residual/tolerance[:,None]
    oracle_pass=oracle_valid&(np.max(abs(oracle_ratio),axis=1)<=1.)
    frames=dict(time_s=t.copy(),truth_family=family,truth_eligible=eligible,truth_loaded=loaded,
        truth_dominant_fraction=share,truth_patch_loads_N=truth['truth_patch_loads_N'],
        truth_resolved_contact_counts_gt005N=truth['truth_patch_contact_counts'],
        truth_equivalent_point_m=truth['truth_equivalent_point_m'],truth_dominant_patch_point_m=dominant_point,
        primary_mask=mask_window(t,e['primary_window_s']),stable_truth_mask=stable_mask(family,eligible),
        oracle_geometry_valid=oracle_valid,oracle_geometry_pass=oracle_pass,
        oracle_joint_prediction_Nm=oracle_prediction,oracle_joint_residual_Nm=oracle_residual,
        oracle_joint_normalized_residual=oracle_ratio,
        oracle_max_normalized_joint_residual=np.max(abs(oracle_ratio),axis=1),
        i60_accepted=pack['i60_accepted'],load_passes=pack['proposed_gate__passes'],
        load_valid=pack['proposed_gate__valid'],load_unknown=pack['proposed_gate__unknown'],
        proposed_joint_residual_Nm=pack['proposed_gate__extra_joint_residual_Nm'],
        proposed_normalized_joint_residual=pack['proposed_gate__normalized_joint_residual'],
        proposed_max_normalized_joint_residual=pack['proposed_gate__max_normalized_joint_residual'],
        native_alternative_joint_residual_Nm=pack['native_alternative_gate__extra_joint_residual_Nm'],
        native_relative_max_ratio_difference=pack['native_relative_max_ratio_difference'],
        threshold_Nm=tolerance,force_norm_N=pack['proposed__force_norm_N'],
        proposed_candidate_index=pack['proposed_candidate_index'],native_candidate_index=pack['native_candidate_index'])
    for policy in POLICIES:
        for suffix in ('family','point_m'):frames[policy+'_'+suffix]=pack[policy+'_'+suffix]
    frames['A_uses_proposal']=pack['A_uses_proposal'];frames['C_uses_proposal']=pack['C_uses_proposal']
    frames['harmful_A']=eligible&(frames['N_family']==family)&(frames['A_family']!=family)
    frames['harmful_C']=eligible&(frames['N_family']==family)&(frames['C_family']!=family)
    frames['useful_A']=eligible&(frames['N_family']!=family)&(frames['A_family']==family)
    frames['retained_useful']=frames['useful_A']&(frames['C_family']==family)
    frames['equivalent_radial_R_proxy']=radial_proxy(truth['truth_equivalent_point_m'],pack['map__centre_m'],pack['proposed__radius_m'])
    frames['dominant_patch_radial_R_proxy']=radial_proxy(dominant_point,pack['map__centre_m'],pack['proposed__radius_m'])
    return frames


def metrics(frames,window):
    selected=window&frames['truth_eligible'];count=int(selected.sum());truth=frames['truth_family']
    A_harm=int(np.sum(window&frames['harmful_A']));C_harm=int(np.sum(window&frames['harmful_C']))
    useful=int(np.sum(window&frames['useful_A']));retained=int(np.sum(window&frames['retained_useful']))
    oracle_mask=selected&frames['stable_truth_mask']
    result=dict(counts=dict(window_rows=int(window.sum()),dominant_eligible=count,
        loaded=int(np.sum(window&frames['truth_loaded'])),loaded_nondominant=int(np.sum(window&frames['truth_loaded']&~frames['truth_eligible'])),
        A_proposals=int(np.sum(selected&frames['A_uses_proposal'])),C_retained_proposals=int(np.sum(selected&frames['C_uses_proposal'])),
        vetoed_proposals=int(np.sum(selected&frames['A_uses_proposal']&~frames['C_uses_proposal'])),
        unknown_load_proposals=int(np.sum(selected&frames['A_uses_proposal']&frames['load_unknown'])),
        A_changed_family=int(np.sum(selected&frames['A_uses_proposal']&(frames['A_family']!=frames['N_family']))),
        C_changed_family=int(np.sum(selected&frames['C_uses_proposal']&(frames['C_family']!=frames['N_family'])))),
        policies={},harmful_overrides=dict(A=A_harm,C=C_harm,relative_reduction=ratio(A_harm-C_harm,A_harm)),
        useful_corrections=dict(A=useful,C_retained=retained,retention=ratio(retained,useful)),
        oracle_geometry=dict(denominator=int(oracle_mask.sum()),valid_count=int(np.sum(oracle_mask&frames['oracle_geometry_valid'])),
            pass_count=int(np.sum(oracle_mask&frames['oracle_geometry_pass'])),
            valid_fraction=ratio(int(np.sum(oracle_mask&frames['oracle_geometry_valid'])),int(oracle_mask.sum())),
            pass_fraction=ratio(int(np.sum(oracle_mask&frames['oracle_geometry_pass'])),int(oracle_mask.sum()))),points={})
    for policy in POLICIES:
        label=frames[policy+'_family'];errors=int(np.sum(selected&(label!=truth)))
        adjacent=window[:-1]&window[1:]&frames['truth_eligible'][:-1]&frames['truth_eligible'][1:]&(truth[:-1]==truth[1:])
        result['policies'][policy]=dict(error_count=errors,error_fraction=ratio(errors,count),
            unknown_count=int(np.sum(selected&(label<0))),sampled_disagreement_equivalent_s=.005*errors,
            false_switch_count=int(np.sum(adjacent&(label[:-1]!=label[1:]))))
    masks=(selected,selected&(frames['N_family']==truth),selected&frames['A_uses_proposal'],
        selected&frames['C_uses_proposal'],selected&frames['stable_truth_mask'],
        selected&(frames['truth_resolved_contact_counts_gt005N'].sum(axis=1)==1))
    target_arrays=(frames['truth_equivalent_point_m'],frames['truth_dominant_patch_point_m'],
                   frames['equivalent_radial_R_proxy'],frames['dominant_patch_radial_R_proxy'])
    policy_finite={p:np.isfinite(frames[p+'_point_m']).all(axis=1) for p in POLICIES}
    for target_name,target in zip(TARGETS,target_arrays):
        target_finite=np.isfinite(target).all(axis=1);result['points'][target_name]={}
        for subset,subset_mask in zip(SUBSETS,masks):
            matched=subset_mask&target_finite
            for finite in policy_finite.values():matched&=finite
            result['points'][target_name][subset]=dict(eligible_count=int(subset_mask.sum()),matched_count=int(matched.sum()),
                matched_fraction=ratio(int(matched.sum()),int(subset_mask.sum())),
                missing_target=int(np.sum(subset_mask&~target_finite)),
                missing_policy={p:int(np.sum(subset_mask&~policy_finite[p])) for p in POLICIES},
                policies={p:statistics(np.linalg.norm(frames[p+'_point_m'][matched]-target[matched],axis=1)) for p in POLICIES})
    return result


def gate_report(aggregate,cases,protocol):
    cfg=protocol['gates'];gates=[]
    def add(name,actual,threshold,passes,group='performance'):
        gates.append(dict(name=name,actual=actual,threshold=threshold,scorable=actual is not None,
            passed=bool(actual is not None and passes),group=group))
    value=aggregate['harmful_overrides']['relative_reduction'];limit=cfg['harmful_override_relative_reduction_min']
    add('harmful_override_reduction',value,limit,value is not None and value>=limit)
    value=aggregate['useful_corrections']['retention'];limit=cfg['useful_correction_retention_min']
    add('useful_correction_retention',value,limit,value is not None and value>=limit)
    a=aggregate['policies']['A']['error_fraction'];c=aggregate['policies']['C']['error_fraction'];n=aggregate['policies']['N']['error_fraction']
    difference=None if a is None or c is None else c-a;limit=cfg['pooled_C_error_above_A_allowance']
    add('pooled_C_vs_A_error',difference,limit,difference is not None and difference<=limit)
    for case,result in cases.items():
        p=result['windows']['primary'];a=p['policies']['A']['error_fraction'];c=p['policies']['C']['error_fraction']
        difference=None if a is None or c is None else c-a;limit=cfg['per_phase_C_error_above_A_allowance']
        add(case+'_C_vs_A_error',difference,limit,difference is not None and difference<=limit)
    c=aggregate['policies']['C']['error_fraction'];improvement=None if n is None or c is None else n-c
    limit=None if n is None else max(cfg['C_native_error_reduction_absolute_min'],cfg['C_native_error_reduction_relative_min']*n)
    add('C_improves_native',improvement,limit,improvement is not None and limit is not None and improvement>=limit)
    point=aggregate['points']['equivalent_raw']['all']['policies']
    for stat,key in (('RMS_mm','point_C_above_native_RMS_m_allowance'),('p95_mm','point_C_above_native_p95_m_allowance'),
                     ('maximum_mm','point_C_above_native_maximum_m_allowance')):
        native=point['N'][stat];candidate=point['C'][stat]
        difference=None if native is None or candidate is None else .001*(candidate-native);limit=cfg[key]
        add('point_'+stat+'_C_vs_native_m',difference,limit,difference is not None and difference<=limit)
    for case,result in cases.items():
        value=result['windows']['primary']['oracle_geometry']['pass_fraction'];limit=cfg['per_phase_oracle_geometry_pass_fraction_min']
        add(case+'_oracle_geometry_compatibility',value,limit,value is not None and value>=limit,'model_validity')
    return gates


def fixed_events(frames,case,i60_root):
    existing=json.loads((i60_root/'analysis/events_stage01/events.json').read_text())['cases'][case]
    specs={name:None if event is None else event['time_s'] for name,event in existing.items()}
    specs['i60_worst_selected_point']={'m100_B':35.205,'p030_B':38.380,'p090_B':17.765}[case]
    events={}
    for name,time_s in specs.items():
        if time_s is None:events[name]=None;continue
        i=int(np.argmin(abs(frames['time_s']-time_s)))
        if abs(float(frames['time_s'][i])-time_s)>1e-8:raise ValueError('Fixed event timestamp absent')
        events[name]=dict(time_s=float(frames['time_s'][i]),truth_family=int(frames['truth_family'][i]),
            N_family=int(frames['N_family'][i]),A_family=int(frames['A_family'][i]),C_family=int(frames['C_family'][i]),
            A_uses_proposal=bool(frames['A_uses_proposal'][i]),C_uses_proposal=bool(frames['C_uses_proposal'][i]),
            proposed_joint_residual_Nm=frames['proposed_joint_residual_Nm'][i],
            proposed_normalized_joint_residual=frames['proposed_normalized_joint_residual'][i],
            native_alternative_joint_residual_Nm=frames['native_alternative_joint_residual_Nm'][i],
            oracle_joint_residual_Nm=frames['oracle_joint_residual_Nm'][i],oracle_geometry_pass=bool(frames['oracle_geometry_pass'][i]),
            threshold_Nm=float(frames['threshold_Nm'][i]),
            distances_to_equivalent_reference_mm={p:1000*float(np.linalg.norm(frames[p+'_point_m'][i]-frames['truth_equivalent_point_m'][i])) for p in POLICIES})
    return events


def evaluate_all(packs,truths,protocol,i60_root=None):
    cases,frames={},{}
    for case in CASES:
        verify_policies(packs[case],protocol);value=make_frames(packs[case],truths[case],protocol);frames[case]=value
        windows={'primary':protocol['evaluation']['primary_window_s'],**protocol['evaluation']['secondary_windows_s']}
        report=dict(windows={name:metrics(value,mask_window(value['time_s'],window)) for name,window in windows.items()})
        report['windows']['stable_primary']=metrics(value,value['primary_mask']&value['stable_truth_mask'])
        if int(value['primary_mask'].sum())!=8000:raise ValueError('Expected8000half-openmovingrows')
        if i60_root is not None:report['fixed_events']=fixed_events(value,case,i60_root)
        cases[case]=report
    combined={key:np.concatenate([frames[c][key] for c in CASES],axis=0) for key in frames[CASES[0]]}
    aggregate=metrics(combined,combined['primary_mask']);gates=gate_report(aggregate,cases,protocol)
    science=dict(iteration='I-61',new_physics_steps=0,live_controller_changed=False,cases=cases,
        aggregate_primary=aggregate,gates=gates,all_gates_passed=all(g['passed'] for g in gates),
        performance_gates_passed=all(g['passed'] for g in gates if g['group']=='performance'),
        model_validity_gates_passed=all(g['passed'] for g in gates if g['group']=='model_validity'),
        failed_gates=[g['name'] for g in gates if not g['passed']],
        qualification='Filtered9-residual/currentgeometry single-resultant acceptanceveto only. Oraclegeometry uses OBSERVEDtotalF andrawdominantpoint, nottrueforcevalidation; multipleweakcontacts/clock/modelerrorsremain. Countsarecorrelatedsavedstates, notindependentrobottrials.',
        point_count_qualification='Exactlyone loggedresolvedrecordabove0.05N doesnotexcludeweakercontributors toloadweightedmeans.')
    return frames,safe(science)


def run(protocol_path,feature_root,output):
    protocol=json.loads(protocol_path.read_text());output.mkdir(parents=True,exist_ok=False)
    if protocol['iteration']!='I-61':raise ValueError('ExpectedI-61protocol')
    mapping=json.loads((protocol_path.parent/'api_mapping_before_execution.json').read_text())
    if sha(protocol_path)!=mapping['protocol_sha256']:raise ValueError('Protocolchangedfrompreexecutionsourcebinding')
    i60_root=ROOT/protocol['frozen_i60_root'];event_path=i60_root/'analysis/events_stage01/events.json'
    event_manifest=json.loads((event_path.parent/'artifact_manifest.json').read_text())
    expected_event=next(x['sha256'] for x in event_manifest['files'] if x['path']=='events.json')
    if sha(event_path)!=expected_event:raise ValueError('FrozenI60eventselectionchanged')
    bindings=[]
    for case in CASES:
        directory=feature_root/case;manifest=directory/'artifact_manifest.json';saved=json.loads(manifest.read_text())
        if not saved.get('sealed_before_any_truth_read_utc'):raise ValueError('Featuresealnotpresent')
        for item in saved['files']:
            if sha(directory/item['path'])!=item['sha256']:raise ValueError('Changedsealedfeatureartifact')
        precision=ROOT/'results/balance_priority_20261126/campaign/runs'/case/'precision.npz'
        bindings.append(dict(case=case,feature_manifest_sha256=sha(manifest),precision_path=str(precision),precision_sha256=sha(precision)))
    root_seal=json.loads((protocol_path.parent/'input_source_seal.json').read_text())
    expected={str((Path(x['path']) if Path(x['path']).is_absolute() else ROOT/x['path']).resolve()):x['sha256'] for x in root_seal['inputs']}
    for binding in bindings:
        if expected.get(binding['precision_path'])!=binding['precision_sha256']:raise ValueError('Truthinputdiffersfromrootseal')
    save(output/'registered_before_truth_read.json',dict(iteration='I-61',registered_utc=datetime.now(timezone.utc).isoformat(),
        evaluator_source_sha256=sha(Path(__file__)),protocol_sha256=sha(protocol_path),features_and_truth=bindings,
        frozen_event_selection=dict(path=str(event_path),sha256=expected_event)))
    shutil.copy2(Path(__file__),output/'evaluate_load_gate.py');shutil.copy2(protocol_path,output/'protocol.json')
    packs,truths={},{}
    for binding in bindings:
        case=binding['case'];packs[case]=load(feature_root/case/'load_features_and_policies.npz')
        with np.load(binding['precision_path'],allow_pickle=False) as raw:
            truths[case]={name:raw[name].copy() for name in ('truth_time_s','truth_patch_loads_N','truth_patch_points_m',
                'truth_patch_contact_counts','truth_equivalent_point_m')}
    frames,science=evaluate_all(packs,truths,protocol,i60_root)
    for binding in bindings:
        if sha(binding['precision_path'])!=binding['precision_sha256']:raise ValueError('Truthinputchangedwhileevaluating')
    if sha(event_path)!=expected_event:raise ValueError('FrozenI60events changedwhileevaluating')
    for case,values in frames.items():np.savez_compressed(output/(case+'_evaluation.npz'),**values)
    save(output/'science.json',science)
    save(output/'artifact_manifest.json',dict(iteration='I-61',files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(all_gates_passed=science['all_gates_passed'],failed_gates=science['failed_gates'],
        counts=science['aggregate_primary']['counts'],policies=science['aggregate_primary']['policies'],
        harmful=science['aggregate_primary']['harmful_overrides'],useful=science['aggregate_primary']['useful_corrections'])))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--feature-root',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.protocol.resolve(),args.feature_root.resolve(),args.output.resolve())
