"""Independent I59 raw-log startup-anchor algebra. No project/physics imports."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,argparse
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];STUDY=ROOT/'results/startup_anchor_audit_20261126';I58=ROOT/'results/balance_priority_20261126'
CASES=('m100_B','m100_P','p030_B','p030_P','p090_B','p090_P');ANCHORS=('m100_B','p030_B','p090_B');MODES=('row_only','trim_only','combined');TOL=1e-10;ABS=np.array([.001,.001,.001,.0001,.0001]);NORMAL=.001


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def exact(a,b):
    a,b=np.asarray(a),np.asarray(b);return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
def safe(x):
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {str(k):safe(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v) for v in x]
    if isinstance(x,float) and not np.isfinite(x):return None
    return x

def save(p,x):Path(p).write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def maxabs(x):return float(np.max(abs(x),initial=0))
def absolute_bounds(low,high):
    return np.where((low<=0)&(high>=0),0,np.minimum(abs(low),abs(high))),np.maximum(abs(low),abs(high))

def fade_bounds(prefade,requested,fade_delta,damping,logged):
    q=np.asarray(logged,np.float32);q64=q.astype(float)
    before=np.nextafter(q,np.float32(-np.inf),dtype=np.float32).astype(float);after=np.nextafter(q,np.float32(np.inf),dtype=np.float32).astype(float)
    low=np.maximum(0,(q64+before)/2);high=np.minimum(1,(q64+after)/2)
    den=prefade[:,:3]-damping;num=requested[:,:3]-damping
    for k in range(3):
        a=den[:,k];b=num[:,k];active=a!=0
        lower=np.full(len(a),-np.inf);upper=np.full(len(a),np.inf)
        with np.errstate(divide='ignore',invalid='ignore'):
            left=(b-TOL)/a;right=(b+TOL)/a
        lower[active]=np.minimum(left[active],right[active]);upper[active]=np.maximum(left[active],right[active]);low=np.maximum(low,lower);high=np.minimum(high,upper)
    chosen=np.argmax(abs(den),axis=1);scale=np.max(abs(den),axis=1);identifiable=scale>0
    estimate=q64.copy();ids=np.flatnonzero(identifiable);estimate[ids]=num[ids,chosen[ids]]/den[ids,chosen[ids]]
    empty=low>high
    if np.any(empty):
        raise ValueError('Fade interval inconsistent at indices '+str(np.flatnonzero(empty).tolist()))
    point=np.clip(estimate,low,high);reconstructed=point[:,None]*prefade[:,:3]+(1-point[:,None])*damping
    residual=maxabs(reconstructed-requested[:,:3]);delta_residual=maxabs((point[:,None]-1)*den-fade_delta[:,:3])
    return point,low,high,dict(structurally_identifiable=int(identifiable.sum()),structurally_unidentifiable=int((~identifiable).sum()),small_denominator_le1e_minus12=int((scale<=1e-12).sum()),maximum_factor_interval_width=float(np.max(high-low)),maximum_request_reclosure_error=residual,maximum_fade_delta_reclosure_error=delta_residual,float32_recorded_dtype=str(np.asarray(logged).dtype)),den


def summarize(point,low,high,caps,selection,held):
    p=point[selection];lo,hi=low[selection],high[selection];cc=caps[selection];h=held[selection];lower,upper=absolute_bounds(lo,hi);normp=np.max(abs(p)/cc,axis=1);normlo=np.max(lower/cc,axis=1);normhi=np.max(upper/cc,axis=1)
    return dict(states=len(p),held_states=int(h.sum()),maximum_abs_point_by_input=np.max(abs(p),axis=0),maximum_abs_possible_by_input=np.max(upper,axis=0),maximum_abs_uncertainty_width_by_input=np.max(hi-lo,axis=0),maximum_cap_normalized_point=float(normp.max()),maximum_cap_normalized_possible=float(normhi.max()),point_absolute_material_states_by_input=np.sum(abs(p)>=ABS,axis=0),guaranteed_absolute_material_states_by_input=np.sum(lower>=ABS,axis=0),possible_absolute_material_states_by_input=np.sum(upper>=ABS,axis=0),point_normalized_material_states=int(np.sum(normp>=NORMAL)),guaranteed_normalized_material_states=int(np.sum(normlo>=NORMAL)),possible_normalized_material_states=int(np.sum(normhi>=NORMAL)),point_absolute_material_held_seconds_by_input=np.sum((abs(p)>=ABS)&h[:,None],axis=0)*.005,point_normalized_material_held_seconds=float(np.sum((normp>=NORMAL)&h)*.005))


def run():
    if (HERE/'report.json').exists():raise FileExistsError('Preserve prior independent numerical audit')
    prereg=json.loads((HERE/'registered_before_arithmetic.json').read_text());assert sha(Path(__file__))==prereg['script_sha256']
    for path,digest in prereg['input_sha256'].items():assert sha(ROOT/path)==digest,path
    protocol=json.loads((STUDY/'protocol/protocol.json').read_text());raws={};rolls={};provs={};anchor={};constancy={};control_failures=[]
    required=('time_s','exact_feedback_gain','exact_error','exact_feedback_error','exact_equilibrium_command','exact_rolling_feedforward','exact_support_command','exact_lqr_command','exact_transition_command_delta','exact_heading_command','exact_fade_command_delta','exact_sync_weights','exact_native_requested','exact_native_applied','exact_native_clipping_command_delta','exact_input_bounds','exact_qvel','exact_reference_qvel')
    for cid in CASES:
        folder=I58/'campaign/runs'/cid;r=load(folder/'precision.npz');roll=load(folder/'rollout.npz');p=json.loads((folder/'provenance.json').read_text());assert all(k in r for k in required);N=len(r['time_s']);assert N==8801 and np.all(abs(np.diff(r['time_s'])-.005)<1e-9);assert r['exact_feedback_gain'].shape==(N,5,18) and r['exact_error'].shape==(N,18);assert np.isfinite(np.concatenate([r[k].ravel() for k in required])).all();assert exact(r['exact_error'],r['exact_feedback_error']);assert len(roll['drive_fade'])==N
        row=r['exact_feedback_gain'][:,0,:];trim=r['exact_equilibrium_command'];constancy[cid]=dict(row0_byte_constant=exact(row,np.broadcast_to(row[0],row.shape).copy()),row0_max_deviation=maxabs(row-row[0]),trim_byte_constant=exact(trim,np.broadcast_to(trim[0],trim.shape).copy()),trim_max_deviation=maxabs(trim-trim[0]))
        if cid in ANCHORS:anchor[cid]=dict(row0=row[0].copy(),trim=trim[0].copy())
        raws[cid]=r;rolls[cid]=roll;provs[cid]=p
    coefficient_pairs={}
    for a in ANCHORS:
        for b in ANCHORS:
            coefficient_pairs[a+'__'+b]=dict(row0_byte_equal=exact(anchor[a]['row0'],anchor[b]['row0']),row0_difference=anchor[a]['row0']-anchor[b]['row0'],row0_max_abs_difference=maxabs(anchor[a]['row0']-anchor[b]['row0']),trim_byte_equal=exact(anchor[a]['trim'],anchor[b]['trim']),trim_difference=anchor[a]['trim']-anchor[b]['trim'],trim_max_abs_difference=maxabs(anchor[a]['trim']-anchor[b]['trim']))
    report=dict(iteration='I-59 independent raw-log audit',completed_utc=datetime.now(timezone.utc).isoformat(),physics_calls=0,native_boundary_only=True,anchor_coefficients=anchor,coefficient_pairs=coefficient_pairs,within_trace_constancy=constancy,case_controls={},comparisons={},all_controls_pass=False,control_failures=[],inference='Frozen state/error/refs/othergainrows/synchronizationtargetsandweights/heading/fadeconditions. No fullPallocator, reset, model ortrajectoryprediction. Engineeringmaterialityclassifications separate.')
    global_point=np.zeros((3,2));global_possible=np.zeros((3,2));material_counts=np.zeros((3,2),dtype=int);possible_counts=np.zeros((3,2),dtype=int)
    for cid,r in raws.items():
        t=r['time_s'];N=len(t);caps=r['exact_input_bounds'];K=r['exact_feedback_gain'];e=r['exact_error'];eq=r['exact_equilibrium_command'];native=r['exact_native_requested'];applied=r['exact_native_applied'];w=r['exact_sync_weights'];lqr=-np.einsum('nij,nj->ni',K,e);pre=eq+r['exact_rolling_feedforward']+r['exact_support_command']+r['exact_lqr_command'];prefade=pre+r['exact_transition_command_delta']+r['exact_heading_command'];damping=-float(provs[cid]['parameters']['airborne_damping_Nms'])*(r['exact_qvel'][:,6:9]-r['exact_reference_qvel'][:,6:9]);f,fl,fh,fade,den=fade_bounds(prefade,native,r['exact_fade_command_delta'],damping,rolls[cid]['drive_fade']);own=cid.rsplit('_',1)[0]+'_B'
        checks=dict(lqr_recomposition=maxabs(lqr-r['exact_lqr_command']),native_request_stage_sum=maxabs(prefade+r['exact_fade_command_delta']-native),native_clip_recomposition=maxabs(np.clip(native,-caps,caps)-applied),native_clipping_delta=maxabs(applied-native-r['exact_native_clipping_command_delta']),zero_blend_transition=maxabs(np.where(w==0,r['exact_transition_command_delta'],0)),fade_belt_delta_zero=maxabs(r['exact_fade_command_delta'][:,3:]),fade_request=fade['maximum_request_reclosure_error'],fade_delta=fade['maximum_fade_delta_reclosure_error'])
        own_row_delta=anchor[own]['row0'][None,:]-K[:,0,:];own_trim_delta=anchor[own]['trim'][None,:]-eq;checks['own_B_anchor_row_command_reproduction']=maxabs(np.einsum('ni,ni->n',own_row_delta,e));checks['own_B_anchor_trim_reproduction']=maxabs(own_trim_delta)
        if not all(c['row0_byte_constant'] and c['trim_byte_constant'] for c in [constancy[cid]]):control_failures.append(dict(case=cid,kind='nonconstantanchor'))
        if not all(value<=TOL for value in checks.values()):control_failures.append(dict(case=cid,kind='numericalclosure',checks=checks))
        if np.any(w<0) or np.any(w>1):control_failures.append(dict(case=cid,kind='invalidsyncweights'))
        controls=dict(closure=checks,fade=fade,samephase_B_row_byte_equal=exact(K[:,0,:],np.broadcast_to(anchor[own]['row0'],K[:,0,:].shape).copy()),samephase_B_trim_byte_equal=exact(eq,np.broadcast_to(anchor[own]['trim'],eq.shape).copy()))
        report['case_controls'][cid]=controls
        windows=dict(protocol['windows_s']);windows.update(protocol['event_windows_s']);held=np.arange(N)<N-1;stored=dict(time_s=t,fade_point=f,fade_low=fl,fade_high=fh,base_native_request=native,base_native_applied=applied)
        for ai,a in enumerate(ANCHORS):
            rowdelta=-np.einsum('ni,ni->n',anchor[a]['row0'][None,:]-K[:,0,:],e);trimdelta=anchor[a]['trim'][None,:]-eq
            for mi,mode in enumerate(MODES):
                d=np.zeros((N,5))
                if mode in ('row_only','combined'):d[:,0]=rowdelta
                if mode in ('trim_only','combined'):d+=trimdelta
                ds=(1-w)*d;point=ds.copy();low=ds.copy();high=ds.copy();point[:,:3]*=f[:,None];p1=ds[:,:3]*fl[:,None];p2=ds[:,:3]*fh[:,None];low[:,:3]=np.minimum(p1,p2);high[:,:3]=np.maximum(p1,p2)
                request=native+point;clipped=np.clip(request,-caps,caps);clipdelta=clipped-applied;cliplo=np.clip(native+low,-caps,caps)-applied;cliphi=np.clip(native+high,-caps,caps)-applied
                key=cid+'__'+mode+'__'+a;result=dict(case=cid,mode=mode,anchor=a,native_requested={},native_clipped={},native_anchor_identity_max_abs=max(maxabs(point),maxabs(clipdelta)) if a==own else None)
                for win,(left,right) in windows.items():
                    select=(t>=left-1e-9)&(t<=right+1e-9)
                    result['native_requested'][win]=summarize(point,low,high,caps,select,held);result['native_clipped'][win]=summarize(clipdelta,cliplo,cliphi,caps,select,held)
                result['baseline_native_clip_state_fraction']=float(np.mean(np.any(abs(native)>caps+1e-12,axis=1)));result['counterfactual_native_clip_state_fraction']=float(np.mean(np.any(abs(request)>caps+1e-12,axis=1)));result['raw_nonzero_masked_by_clip_states']=int(np.sum(np.any(abs(point)>TOL,axis=1)&np.all(abs(clipdelta)<=TOL,axis=1)))
                if a==own and result['native_anchor_identity_max_abs']>TOL:control_failures.append(dict(case=cid,mode=mode,kind='actual_anchor_identity',error=result['native_anchor_identity_max_abs']))
                report['comparisons'][key]=result
                for bi,name in enumerate(('native_requested','native_clipped')):
                    s=result[name]['full'];global_point[mi,bi]=max(global_point[mi,bi],s['maximum_cap_normalized_point']);global_possible[mi,bi]=max(global_possible[mi,bi],s['maximum_cap_normalized_possible']);material_counts[mi,bi]+=s['point_normalized_material_states'];possible_counts[mi,bi]+=s['possible_normalized_material_states']
                prefix=mode+'__'+a;stored[prefix+'__requested_delta']=point;stored[prefix+'__clipped_delta']=clipdelta;stored[prefix+'__requested_delta_lower']=low;stored[prefix+'__requested_delta_upper']=high;stored[prefix+'__clipped_delta_lower']=cliplo;stored[prefix+'__clipped_delta_upper']=cliphi
        np.savez_compressed(HERE/(cid+'_independent_counterfactuals.npz'),**stored)
    report['global_summary']=dict(modes=MODES,boundaries=('native_requested','native_clipped'),maximum_normalized_point=global_point,maximum_normalized_possible=global_possible,normalized_material_state_counts=material_counts,normalized_possible_material_state_counts=possible_counts,warning='Countsarecorrelatedstaticstateevaluationsincludingownanchoridentity andterminalstate; notindependenttestsornewphysicaltrials.')
    report['control_failures']=control_failures;report['all_controls_pass']=not control_failures
    save(HERE/'report.json',report)
    for path,digest in prereg['input_sha256'].items():assert sha(ROOT/path)==digest,path
    assert sha(Path(__file__))==prereg['script_sha256'];save(HERE/'input_hashes_unchanged.json',dict(passed=True))
    print(json.dumps(safe(dict(all_controls_pass=report['all_controls_pass'],constancy=constancy,coefficient_pairs=coefficient_pairs,global_summary=report['global_summary'])),indent=2))

if __name__=='__main__':run()
