"""Independent cached-coordinate, hinge-load and eigen-score I67 review.

No bank_audit or candidate helper imports. Uses frozen cached coordinates;
does not regenerate or expand the bank, refit scores, or query contact physics.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json,math
import numpy as np

CASES=('m100_B','p030_B','p090_B')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}


def min_indices(value,allowed):
    ids=np.full(len(value),-1,int);minimum=np.full(len(value),np.nan)
    for i in range(len(value)):
        eligible=np.flatnonzero(allowed[i]&np.isfinite(value[i]))
        if len(eligible):
            j=min(eligible,key=lambda j:(float(value[i,j]),int(j)))
            ids[i]=j;minimum[i]=value[i,j]
    return ids,minimum


def take(value,index):
    out=np.full((len(index),)+value.shape[2:],np.nan)
    for i,j in enumerate(index):
        if j>=0:out[i]=value[i,j]
    return out


def hinge_load(packet,points,families):
    """J^T F via axis dot((point-hinge origin) cross force)."""
    n,c,_=points.shape;out=np.full((n,c,3),np.nan)
    family=np.broadcast_to(families,(n,c));valid=(family>=0)&(family<4)&np.isfinite(points).all(axis=2)
    force=packet['observer_interval_mean'][:,:3]
    for i in range(n):
        for k in range(c):
            if not valid[i,k] or not np.isfinite(force[i]).all():continue
            f=int(family[i,k])
            for j in range(3):
                if not packet['ancestor_active'][f,j]:out[i,k,j]=0.
                else:out[i,k,j]=packet['hinge_axis_world'][i,j]@np.cross(points[i,k]-packet['hinge_anchor_m'][i,j],force[i])
    return out


def eigen_score(e,packet):
    eigenvalue,eigenvector=np.linalg.eigh(packet['frozen_covariance'])
    if np.any(eigenvalue<=0):raise ValueError('Frozen covariance is not positive definite')
    flat=e.reshape(-1,3);good=np.isfinite(flat).all(axis=1);s=np.full(len(flat),np.nan)
    q=(flat[good]-packet['frozen_mean'])@eigenvector
    s[good]=np.einsum('ij,ij->i',q,q/eigenvalue)
    return s.reshape(e.shape[:-1])


def capacity(e,score,wrong,wrongscore,available):
    known=available&np.isfinite(score)&np.isfinite(wrongscore[:,None])&np.isfinite(e).all(axis=2)&np.isfinite(wrong).all(axis=1)[:,None]
    difference=e-wrong[:,None,:];distance=np.sqrt(np.sum(difference*difference,axis=2))
    ambiguous=known&(distance<=.01);eps=1e-9*np.maximum(1.,np.maximum(abs(score),abs(wrongscore[:,None])))
    tied=known&~ambiguous&(abs(score-wrongscore[:,None])<=eps)
    win=known&~ambiguous&~tied&(score<wrongscore[:,None])
    return known,ambiguous,tied,win


def compare(a,b,atol,rtol=0):
    a=np.asarray(a);b=np.asarray(b)
    if a.shape!=b.shape:return dict(passed=False,reason='shape mismatch')
    finite=np.isfinite(a)&np.isfinite(b);pattern=np.array_equal(np.isfinite(a),np.isfinite(b))
    difference=abs(a[finite]-b[finite]);maximum=float(difference.max()) if difference.size else 0.
    return dict(passed=bool(pattern and np.all(difference<=atol+rtol*abs(b[finite]))),
        finite_patterns_exact=bool(pattern),maximum_absolute_difference=maximum,atol=atol,rtol=rtol)


def group(a,mask):
    new=mask&a['reference_correct']&~a['baseline_correct']
    return dict(denominator=int(mask.sum()),known_bank=int(np.sum(mask&(a['usable_correct_count']>0))),
        original_correct=int(np.sum(mask&a['baseline_correct'])),sampled_reference_correct=int(np.sum(mask&a['reference_correct'])),
        any_existing_candidate_correct=int(np.sum(mask&a['any_bank_correct'])),
        newly_correct_reference_denominator=int(new.sum()),reference_repairs_supported=int(np.sum(new&a['any_bank_correct'])),reference_repairs_unsupported=int(np.sum(new&~a['any_bank_correct'])),
        capacity_matrix=dict(reference_yes_bank_yes=int(np.sum(mask&a['reference_correct']&a['any_bank_correct'])),
            reference_yes_bank_no=int(np.sum(mask&a['reference_correct']&~a['any_bank_correct'])),
            reference_no_bank_yes=int(np.sum(mask&~a['reference_correct']&a['any_bank_correct'])),
            reference_no_bank_no=int(np.sum(mask&~a['reference_correct']&~a['any_bank_correct']))))


def case_review(p,b,expected,protocol):
    cfg=protocol['verification'];checks=[];N=len(p['time_s']);row=np.arange(N);family=b['family_index'];out={}
    active=p['old__valid'].copy()
    for key in ('normal_angle_rad','velocity_residual_norm_m_s','moment_residual_norm'):
        active &= np.isfinite(p['old__'+key])&(p['old__'+key]>=0)
    active &= np.isfinite(p['old__velocity_residual_m_s']).all(axis=2)&p['old__frame_valid'][:,None]&np.isfinite(p['old__excitation_m_s'])[:,None]
    cost=(p['old__normal_angle_rad']/math.radians(10.))**2+p['old__moment_residual_norm']**2+(p['old__velocity_residual_norm_m_s']/.02)**2
    fused_ids=[];fused_score=[]
    for f in range(4):
        ids,values=min_indices(cost,active&(family==f)[None,:]);fused_ids.append(ids);fused_score.append(values)
    fused_ids=np.stack(fused_ids,axis=1);fused_score=np.stack(fused_score,axis=1)
    out['original_fused_ids_exact']=bool(np.array_equal(fused_ids,p['old_fused__candidate_index']));checks.append(out['original_fused_ids_exact'])
    out['original_fused_score']=compare(fused_score,p['old_fused__family_scores'],cfg['original_cost_atol']);checks.append(out['original_fused_score']['passed'])
    same=(family[None,:]==p['truth_family'][:,None])&(p['truth_family']>=0)[:,None]
    usable=active&same&b['reconstructed'];raw=same&b['reconstructed'];points=b['points_m']
    delta=points-p['sampled_reference_point_m'][:,None,:];distance=np.sqrt(np.sum(delta*delta,axis=2))
    nearest,distance_min=min_indices(distance,usable);pre,pre_distance=min_indices(distance,raw)
    branch=p['correct_branch'];selected=np.where(branch==0,p['saved_F_index'],np.where(branch==1,p['saved_Nalt_index'],-1))
    safe=np.maximum(selected,0);member=(selected>=0)&usable[row,safe]
    opportunity=p['cohort_useful']|p['cohort_harmful'];desired=np.where(branch==0,-1,1)
    prediction=hinge_load(p,points,family[None,:]);residual=p['observer_interval_mean'][:,None,6:9]-prediction
    e=residual/p['fixed_scale_Nm'][:,None,None];score=eigen_score(e,p);score[branch<0]=np.nan;min_id,min_score=min_indices(score,usable)
    # Recompute the fixed wrong branch from point/body/hinges, not old scores.
    wrong_point=np.where((branch==0)[:,None],p['saved_point_Nalt_m'],p['saved_point_F_m'])
    wrong_family=np.where(branch==0,p['saved_Nalt_family'],p['saved_F_family'])
    wrong_prediction=hinge_load(p,wrong_point[:,None,:],wrong_family[:,None])[:,0]
    wrong=(p['observer_interval_mean'][:,6:9]-wrong_prediction)/p['fixed_scale_Nm'][:,None];wrong[~opportunity]=np.nan
    wrongscore=eigen_score(wrong,p)
    known,ambiguous,tied,win=capacity(e,score,wrong,wrongscore,usable&opportunity[:,None])
    anywin=win.any(axis=1);selected_code=np.full(N,3,np.int8)
    for i in np.flatnonzero(opportunity&(selected>=0)):
        j=selected[i]
        if ambiguous[i,j]:selected_code[i]=2
        elif tied[i,j]:selected_code[i]=0
        elif known[i,j]:selected_code[i]=desired[i] if win[i,j] else -desired[i]
    baseline=opportunity&(p['old_I65_code']==desired);reference=opportunity&(p['reference_cf_code']==desired)
    reference_pred=p['observer_interval_mean'][:,6:9]-p['reference_residual_Nm']
    torque_distance=np.sqrt(np.sum((prediction-reference_pred[:,None,:])**2,axis=2));torque_distance[branch<0]=np.nan;torque_id,torque_gap=min_indices(torque_distance,usable)
    selected_distance=take(distance,selected);selected_score=take(score,selected)
    selected_distance[~member]=np.nan;selected_score[~member]=np.nan
    reference_score=np.where(branch==0,p['reference_cf_F_score'],p['reference_cf_Nalt_score'])
    arrays=dict(selected_id=selected,selected_member=member,opportunity=opportunity,selected_original_code=selected_code,
        baseline_correct=baseline,reference_correct=reference,any_bank_correct=anywin,usable_correct_count=usable.sum(axis=1),pre_admission_correct_count=raw.sum(axis=1),
        selected_distance_m=selected_distance,nearest_distance_m=distance_min,pre_admission_distance_m=pre_distance,
        geometric_selection_gap_m=selected_distance-distance_min,geometric_admission_gap_m=distance_min-pre_distance,
        selected_load_score=selected_score,min_load_score=min_score,reference_load_score=reference_score,
        load_selection_gap=selected_score-min_score,min_minus_reference_score=min_score-reference_score,
        minimum_reference_torque_distance_Nm=torque_gap,wrong_load_score=wrongscore)
    for name,value in arrays.items():
        if value.dtype.kind in 'biu':check=dict(passed=bool(np.array_equal(value,expected[name])),exact=bool(np.array_equal(value,expected[name])))
        else:
            is_score='score' in name or name=='load_selection_gap'
            atol=cfg['score_atol'] if is_score else cfg['load_prediction_atol_Nm'] if name.endswith('_Nm') else cfg['selected_point_atol_m']
            check=compare(value,expected[name],atol,cfg['score_rtol'] if is_score else 0)
        out[name]=check;checks.append(check['passed'])
    index_reports={}
    for name,ids,values,bank_values,tolerance in (('nearest_id',nearest,distance_min,distance,cfg['selected_point_atol_m']),
        ('pre_admission_nearest_id',pre,pre_distance,distance,cfg['selected_point_atol_m']),
        ('minscore_id',min_id,min_score,score,cfg['score_atol']),
        ('nearest_load_reference_id',torque_id,torque_gap,torque_distance,cfg['load_prediction_atol_Nm'])):
        primary=expected[name];equal=np.array_equal(ids,primary);primary_value=take(bank_values,primary)
        equivalent=compare(values,primary_value,tolerance,cfg['score_rtol'] if name=='minscore_id' else 0)
        index_reports[name]=dict(exact=bool(equal),mismatch_count=int(np.sum(ids!=primary)),
            equivalent_minimum_value=equivalent,mismatches=[dict(source_index=int(p['source_index'][i]),independent=int(ids[i]),primary=int(primary[i])) for i in np.flatnonzero(ids!=primary)])
        checks.append(equivalent['passed'])
    # Check the declared primary descriptor points independently even if an
    # algebraically equivalent minimum has a different last-bit argmin.
    for descriptor,key in (('nearest_geometry_correct','nearest_id'),('minscore_correct','minscore_id')):
        ids=expected[key];actual=win[row,np.maximum(ids,0)]&(ids>=0)
        out[descriptor]=dict(passed=bool(np.array_equal(actual,expected[descriptor])),exact=bool(np.array_equal(actual,expected[descriptor])))
        checks.append(out[descriptor]['passed'])
    original_point=take(points,selected);saved=np.where((branch==0)[:,None],p['saved_point_F_m'],p['saved_point_Nalt_m'])
    out['selected_coordinates']=compare(original_point[member],saved[member],cfg['selected_point_atol_m']);checks.append(out['selected_coordinates']['passed'])
    out['selected_normalized_load']=compare(take(e,selected)[opportunity],np.where((branch==0)[:,None],p['old_I65_e_F'],p['old_I65_e_Nalt'])[opportunity],1e-10)
    checks.append(out['selected_normalized_load']['passed'])
    out['selected_original_codes_vs_I65']=bool(np.array_equal(selected_code[opportunity],p['old_I65_code'][opportunity]));checks.append(out['selected_original_codes_vs_I65'])
    out['inclusion_nonregression']=bool(np.all(anywin[baseline]));checks.append(out['inclusion_nonregression'])
    out['argmin_indices']=index_reports;out['all_checks_passed']=all(checks)
    return out,arrays


def run(args):
    args.output.mkdir(parents=True,exist_ok=False);protocol=read(args.protocol);canonical=read(args.canonical/'science.json')
    write(args.output/'registered_before_independent_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(Path(__file__)),
        protocol_sha256=sha(args.protocol),canonical_report_sha256=sha(args.canonical/'science.json'),numpy_version=np.__version__,
        status='Post-result independent verification source sealed before execution; primary29/29result already communicated, no science rule changes',
        independence='Cached coordinates shared, independent active/cost/minimum/hinge cross-product loads/eigen scores/classification; no bank_audit import or fitting.'))
    reports={};all_arrays=[];all_packets=[]
    for case in CASES:
        p=load(args.packets/(case+'.npz'));b=load(args.banks/(case+'.npz'));expected=load(args.canonical/(case+'_frames.npz'))
        report,arrays=case_review(p,b,expected,protocol);reports[case]=report;all_arrays.append(arrays);all_packets.append(p)
    merged={k:np.concatenate([a[k] for a in all_arrays]) for k in all_arrays[0]};groups={};group_exact=True
    for name in ('focus','useful','harmful','retained_useful'):
        mask=np.concatenate([p['cohort_'+name] for p in all_packets]);actual=group(merged,mask);expected=canonical['cohorts'][name]
        same=all(actual[k]==expected[k] for k in actual);groups[name]=dict(exact=bool(same),independent=actual);group_exact &= same
    repaired=np.concatenate([(p['cohort_focus']|p['cohort_harmful']) for p in all_packets])&merged['reference_correct']&~merged['baseline_correct']
    denominator=int(repaired.sum());supported=int(np.sum(repaired&merged['any_bank_correct']))
    count_exact=denominator==canonical['primary_capacity']['reference_repaired_denominator'] and supported==canonical['primary_capacity']['supported']
    result=dict(iteration='I-67',numpy_version=np.__version__,all_checks_passed=all(r['all_checks_passed'] for r in reports.values()) and group_exact and count_exact,
        per_case=reports,cohorts=groups,reference_repairs=dict(denominator=denominator,supported=supported,unsupported=denominator-supported,exact=bool(count_exact)),
        scope='Independent verification of retrospective truth-assisted finite-bank capacity; no new estimator, bank expansion, refit or physics.')
    write(args.output/'comparison.json',result)
    print(json.dumps(dict(all_checks_passed=result['all_checks_passed'],reference_repairs=result['reference_repairs'],numpy_version=np.__version__)))
    return result['all_checks_passed']


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('protocol','packets','banks','canonical','output'):parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args();raise SystemExit(0 if run(args) else 1)
