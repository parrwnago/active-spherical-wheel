"""Independent I58 saved-array formulas; no imports of canonical metrics/physics.

Run only after parent confirms initial B/A/P and canonical analysis are sealed.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,gzip
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];STUDY=ROOT/'results/balance_priority_20261126'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {str(k):safe(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v) for v in x]
    if isinstance(x,float) and not np.isfinite(x):return None
    return x

def save(p,x):Path(p).write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def rotation(q):
    q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]]).transpose(2,0,1)
def exact(a,b):
    a,b=np.asarray(a),np.asarray(b)
    return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
def covered(t,lo,hi):return bool(len(t) and t[0]<=lo+1e-9 and t[-1]>=hi-1e-9)

def main(stage):
    if (HERE/'audit_result.json').exists():raise FileExistsError('Preserve prior audit attempt')
    prereg=json.loads((HERE/'registered_before_outcomes.json').read_text())
    for path,digest in prereg['input_source_sha256'].items():assert sha(ROOT/path)==digest,path
    protocol=json.loads((STUDY/'protocol.json').read_text());canonical=json.loads((stage/'report.json').read_text());index=json.loads((stage/'portable_inputs.json').read_text());events=json.loads((stage/index['events']).read_text());inputhash={};metrics={};metadata={};comparisons={};agreement={};raw_by_case={};missing=[]
    def read(p):inputhash[str(p.relative_to(ROOT))]=sha(p);return json.loads(p.read_text())
    def arrays(p):
        inputhash[str(p.relative_to(ROOT))]=sha(p)
        with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
    for cid,entry in index['cases'].items():
        raw=arrays(stage/entry['projection']);raw_by_case[cid]=raw;md=read(stage/entry['metadata']);metadata[cid]=md;t=raw['time_s'];q=raw['exact_qpos'];v=raw['exact_qvel'];R=rotation(q[:,3:7]);Rr=rotation(raw['exact_reference_qpos'][:,3:7]);up=R[:,:,2];upr=Rr[:,:,2]
        xy=np.linalg.norm(q[:,:2]-raw['exact_reference_xy'],axis=1);moving=(t>=1)&(t<=41);late=(t>=28-1e-9)&(t<=44+1e-9);angular=np.einsum('nij,nj->ni',R,v[:,3:6]);headingvalid=np.sum(R[:,:2,0]**2,axis=1)>.04
        # Canonical data never bridges an invalid projected heading sample.
        heading=np.full(len(t),np.nan);ids=np.flatnonzero(headingvalid)
        for run in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
            if len(run):heading[run]=np.unwrap(np.arctan2(R[run,1,0],R[run,0,0]))
        increments=np.diff(heading)[late[1:]&late[:-1]&headingvalid[1:]&headingvalid[:-1]]
        lean=np.rad2deg(np.arccos(np.clip(np.sum(up*upr,axis=1),-1,1)));tilt=np.rad2deg(np.arccos(np.clip(up[:,2],-1,1)));speed=np.linalg.norm(v[:,:2],axis=1);acc50=np.linalg.norm((v[10:,:3]-v[:-10,:3])/(t[10:]-t[:-10])[:,None],axis=1)
        u=raw['exact_applied'];prev=raw['exact_previous_applied'];caps=raw['exact_input_bounds'];requested=raw['exact_requested'];steps=u-prev;normsteps=np.linalg.norm((steps/caps)[:-1],axis=1);interval=np.r_[.005,np.diff(t)];excess=np.maximum(abs(steps)-np.array([60,60,60,6,6])[None,:]*interval[:,None],0)
        dgpath=stage/entry['diagnostics'];inputhash[str(dgpath.relative_to(ROOT))]=sha(dgpath)
        with gzip.open(dgpath,'rt') as stream:governor=json.load(stream)
        total=np.array([np.array(r['diagnostics']['first_stage_balance_delta'])+np.array(r['diagnostics']['secondary_balance_delta']) for r in governor if r['diagnostics']]);eq=np.array([r['diagnostics']['equality_residual_max'] for r in governor if r['diagnostics']]);secondary=np.array([r['diagnostics']['secondary_balance_delta'] for r in governor if r['diagnostics']]);candidate=md['job']['variant']!='native'
        if candidate:
            assert len(total)==len(t);assert all(row['time_s']==ti for row,ti in zip(governor,t))
            loss=dict(maxabs=float(abs(total).max()),peaknorm=float(np.linalg.norm(total,axis=1).max()),heldRMS=float(np.sqrt(np.mean(np.sum(total[:-1]**2,axis=1)))),finite=bool(np.isfinite(total).all()),eqmax=float(eq.max()),eqfinite=bool(np.isfinite(eq).all()),secondarymax=float(abs(secondary).max()),secondaryfinite=bool(np.isfinite(secondary).all()))
        else:loss=None
        tail=t>=t[-1]-1;summary=md['summary'];settled=bool(summary['completed'] and np.all(lean[tail]<1) and np.all(speed[tail]<.03) and np.all(np.linalg.norm(v[tail,3:6],axis=1)<.08));turns=[]
        for ev in events:
            select=(t>=ev['window_s'][0]-1e-9)&(t<=ev['window_s'][1]+1e-9);signed=ev['incoming_sign']*(q[:,ev['axis_index']]-ev['reference_component_extremum_m']);turns.append(dict(event_id=ev['event_id'],coverage=covered(t,*ev['window_s']),overshoot=float(max(0,signed[select].max())) if select.any() else None))
        m=dict(available=True,completed=bool(summary['completed']),failure=summary['failure'],full=covered(t,0,44),movingcoverage=covered(t,1,41),latecoverage=covered(t,28,44),headingvalid=bool(headingvalid[late].all()),rms=float(np.sqrt(np.mean(xy[moving]**2))) if moving.any() else None,peak=float(xy.max()),final=float(xy[-1]),leanpeak=float(lean.max()),leanRMS=float(np.sqrt(np.mean(lean**2))),tiltpeak=float(tilt.max()),yawpeak=float(abs(angular[late,2]).max()) if late.any() else None,headingtravel=float(abs(increments).sum()) if late.any() else None,speedpeak=float(speed.max()),acc50peak=float(acc50.max()) if len(acc50) else None,capexcess=float(np.maximum(abs(u)-caps,0).max()),requestexcess=float(np.maximum(abs(requested)-caps,0).max()),rateexcess=float(excess.max()),inputfinite=bool(np.isfinite(np.r_[u.ravel(),prev.ravel(),requested.ravel()]).all()),history=bool(exact(prev[1:],u[:-1])),continuity=dict(RMS=float(np.sqrt(np.mean(normsteps**2))),p99=float(np.quantile(normsteps,.99)),peak=float(normsteps.max())),priority=loss,turns=turns,settled=settled,bank=md['provenance']['fixed_bank_numeric_sha256'],source_unchanged=md['provenance']['source_hashes_unchanged_after'],historical_exact=None)
        if cid in protocol['historical_baseline_paths']:
            # Independently compare full shared original precision fields, not just recorded parity.
            fresh=arrays(ROOT/md['original_precision_path']);old=arrays(ROOT/protocol['historical_baseline_paths'][cid]);shared=sorted(set(fresh)&set(old));m['historical_exact']=all(exact(fresh[k],old[k]) for k in shared);m['historical_shared_fields']=len(shared)
        expected=canonical['cases'][cid];checks=[]
        def compare(path,a,b):
            ok=bool(np.isclose(a,b,rtol=1e-10,atol=1e-10));checks.append((path,ok));assert ok,(cid,path,a,b)
        for path,a,b in [('rms',m['rms'],expected['timed_tracking']['moving_rms_m']),('peak',m['peak'],expected['timed_tracking']['full_peak_m']),('final',m['final'],expected['timed_tracking']['final_m']),('leanpeak',m['leanpeak'],expected['attitude']['peak_raw_reference_error_deg']),('leanRMS',m['leanRMS'],expected['attitude']['rms_raw_reference_error_deg']),('tiltpeak',m['tiltpeak'],expected['attitude']['peak_absolute_deg']),('yawpeak',m['yawpeak'],expected['motion']['late']['world_z_peak_abs_rad_s']),('headingtravel',m['headingtravel'],expected['motion']['late']['heading_absolute_travel_rad']),('speedpeak',m['speedpeak'],expected['motion']['full']['planar_speed_peak_m_s']),('acc50peak',m['acc50peak'],expected['motion']['full']['linear_acceleration50ms']['peak_norm']),('caps',m['capexcess'],expected['commands']['applied_box_max_violation']),('requests',m['requestexcess'],expected['commands']['requested_box_max_violation']),('continuityRMS',m['continuity']['RMS'],expected['continuity']['normalized_step_norm_RMS']),('continuityp99',m['continuity']['p99'],expected['continuity']['normalized_step_norm_p99']),('continuitymax',m['continuity']['peak'],expected['continuity']['normalized_step_norm_max'])]:compare(path,a,b)
        if candidate:
            for key,outkey in [('maxabs','total_response_max_abs'),('peaknorm','total_response_peak_norm'),('heldRMS','total_response_held_RMS_norm')]:compare('priority_'+key,loss[key],expected['priority'][outkey])
        assert m['settled']==expected['original_settling']['passed'];assert [x['overshoot'] for x in turns]==[x['geometric_component_overshoot_m'] for x in expected['turns']]
        if m['historical_exact'] is not None:assert m['historical_exact']==expected['historical_parity']['all_shared_fields_exact']
        metrics[cid]=m;agreement[cid]=dict(independent_core_metrics_agree=True,quantities=len(checks),scope='Core scalar metrics independently recomputed; turn reversallag not rederived, geometricovershoots are. Gresponsevalues read savedmodeldiagnostics; bank/execution assertions remain provenance.')
    limits=protocol['acceptance']
    def le(a,b):return bool(a is not None and b is not None and np.isfinite(a) and np.isfinite(b) and a<=b+1e-12)
    for phase in ('m100','p030','p090'):
        c,b=metrics.get(phase+'_P'),metrics.get(phase+'_B')
        if c is None or b is None:missing.append(phase);continue
        a=metrics.get('m100_A') if phase=='m100' else None
        checks=dict(full44_no_failure=bool(c['completed'] and c['failure'] is None and c['full']),baseline_full44_no_failure=bool(b['completed'] and b['failure'] is None and b['full']),baseline_historical_exact=bool(b['historical_exact']),moving_and_late_coverage=bool(c['movingcoverage'] and b['movingcoverage'] and c['latecoverage'] and b['latecoverage']),late_heading_valid=bool(c['headingvalid'] and b['headingvalid']),same_frozen_bank=c['bank']==b['bank'],finite_actual_history=bool(c['inputfinite'] and c['history']),hard_total_amplitude=bool(c['capexcess']<=1e-10 and c['requestexcess']<=1e-10),exact_nominal_transverse_response=bool(c['priority']['finite'] and c['priority']['maxabs']<=limits['balance_response_tolerance']),secondary_equality=bool(c['priority']['eqfinite'] and c['priority']['secondaryfinite'] and c['priority']['eqmax']<=limits['balance_response_tolerance'] and c['priority']['secondarymax']<=limits['balance_response_tolerance']),moving_timed_XY_RMS=le(c['rms'],limits['timed_xy_rms_ratio_max']*b['rms']+limits['timed_xy_rms_add_m']),full_timed_XY_peak=le(c['peak'] if c['full'] else None,b['peak']+limits['timed_xy_peak_add_m'] if b['full'] else None),final_timed_XY=le(c['final'] if c['full'] else None,b['final']+limits['final_xy_add_m'] if b['full'] else None),raw_lean_peak=le(c['leanpeak'],b['leanpeak']+limits['relative_lean_peak_add_deg']),raw_lean_RMS=le(c['leanRMS'],limits['relative_lean_rms_ratio_max']*b['leanRMS']+limits['relative_lean_rms_add_deg']),absolute_tilt_peak=le(c['tiltpeak'],b['tiltpeak']+limits['absolute_tilt_peak_add_deg']),late_yaw_peak=le(c['yawpeak'],limits['yaw_peak_abs_ratio_max']*b['yawpeak']+limits['yaw_peak_abs_add_rad_s']),late_absolute_heading_travel=le(c['headingtravel'],limits['heading_absolute_travel_ratio_max']*b['headingtravel']+limits['heading_absolute_travel_add_rad']),full_planar_speed_peak=le(c['speedpeak'],limits['planar_speed_peak_ratio_max']*b['speedpeak']+limits['planar_speed_peak_add_m_s']),full_50ms_linear_acceleration_peak=le(c['acc50peak'],limits['linear_acceleration50ms_peak_ratio_max']*b['acc50peak']+limits['linear_acceleration50ms_peak_add_m_s2']),all_six_turn_overshoots=bool(len(c['turns'])==6 and all(x['coverage'] and y['coverage'] and le(x['overshoot'],y['overshoot']+limits['each_turn_overshoot_add_m']) for x,y in zip(c['turns'],b['turns']))),conditional_original_settling=bool(not b['settled'] or c['settled']))
        if phase=='m100':
            checks['hard_slew_comparator_available_and_exact']=bool(a is not None and a['completed'] and a['failure'] is None and a['full'] and a['historical_exact'])
            if a is not None:
                checks['hard_slew_comparator_rate_box']=a['rateexcess']<=1e-10;checks['nontrivial_old_priority_loss']=bool(a['priority']['finite'] and a['priority']['peaknorm']>=limits['old_A_priority_peak_min_rad_s']);checks['primary_priority_restoration_vs_A']=le(c['priority']['heldRMS'],limits['priority_RMS_ratio_vs_A_max']*a['priority']['heldRMS'])
            else:checks['primary_priority_restoration_vs_A']=False
        expected=canonical['comparisons'][phase+'_P_vs_'+phase+'_B'];assert checks==expected['checks'],(phase,checks,expected['checks'])
        comparisons[phase]=dict(all_gate_booleans_agree=True,checks=checks,failed=[k for k,v in checks.items() if not v],passes_all=all(checks.values()))
    result=dict(completed_utc=datetime.now(timezone.utc).isoformat(),source_stage=str(stage.relative_to(ROOT)),case_metrics=metrics,metric_agreement=agreement,gate_comparisons=comparisons,unavailable_phase_pairs=missing,scope='Independent saved-array core formula/gate audit; unavailable phases remain untested. Nativephysics not rerun. Priority uses recorded modeldiagnostics and does not establish physicalbalance.',physics_executions=0,input_sha256=inputhash)
    save(HERE/'audit_result.json',result)
    assert all(sha(ROOT/p)==h for p,h in inputhash.items())
    for path,digest in prereg['input_source_sha256'].items():assert sha(ROOT/path)==digest,path
    print(json.dumps(safe(dict(cases=list(metrics),comparisons=comparisons,unavailable_phase_pairs=missing)),indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--stage',type=Path,required=True);args=parser.parse_args();main(args.stage.resolve())
