"""I54 frozen-state gain algebra only; never executes/integrates a controller."""
from pathlib import Path
import datetime,hashlib,json,shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
NAMES=('shell_A','shell_B','belt_A','belt_B')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def safe(v):
    if isinstance(v,dict):return {str(k):safe(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [safe(x) for x in v]
    if isinstance(v,np.ndarray):return safe(v.tolist())
    if isinstance(v,np.generic):return safe(v.item())
    if isinstance(v,float) and not np.isfinite(v):return None
    return v

def main():
    out=HERE/'stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'analyze_gain.py')
    mapping=ROOT/'results/contact_scheduling_joint_tracking_20261126/campaign/resolved_case_paths.json'
    resolver=json.loads(mapping.read_text())['cases']
    cache=ROOT/'generated/offroad/flat/control_schedule.npz'
    files=[mapping,cache,ROOT/'src/spherical_wheel/recovery_control.py',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/contact_sensing.py']
    cases=('m100_B','m100_R','m100_N','m100_NR')
    files += [ROOT/resolver[c]/'precision.npz' for c in cases]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in files}
    write(out/'registered_before_arithmetic.json',{'iteration':'I-54','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_sha256':sha(Path(__file__)),'input_sha256':hashes,'scope':'No integration, controller execution or model/observer mutation. Reimplement exact cached gain lookup and compare all loaded samples; descriptive fixed-state algebra only.','events':'Prior I53 events31.770 and32.250, plus largest gain-induced normalized command change over fixed28–36 and32.25–44 windows, all fourm100 arms.','decomposition':'Symmetric phase/load Shapley split of K change; symmetric K/error split of LQR change. Unchanged shaft anchor. These are algebraic decompositions, not trajectory counterfactuals.','blend_feasibility':'At selected rows retain current non-LQR command terms and current error; blend previous/currentK overlambda0..1. Intersect all5 native input boxes. No inference that a feasible blend ensures balance/contact or that an infeasible blend excludes nonlinear solutions.'})
    bank=load(cache);groups={}
    for name in NAMES:
        regime=int(name.startswith('belt'));side=int(name.endswith('B'))
        idx=np.flatnonzero((bank['regime_code']==regime)&(bank['side_code']==side)&(np.abs(bank['K'][:,0,4])<=100.))
        idx=idx[np.argsort(bank['phase_rad'][idx])]
        groups[name]=(bank['phase_rad'][idx],bank['K'][idx])
    def gain(phase,loads,anchor):
        result=np.zeros((5,18));total=float(loads.sum())
        for name,force in zip(NAMES,loads):
            if force<.001:continue
            phases,gains=groups[name]
            def distance(x):return max(phases[0]-x,0.,x-phases[-1])
            p=min((phase,phase-2*np.pi,phase+2*np.pi),key=distance)
            i=int(np.searchsorted(phases,p));lo,hi=max(0,i-1),min(len(phases)-1,i)
            f=np.clip((p-phases[lo])/max(phases[hi]-phases[lo],1e-9),0.,1.)
            result += force/total*((1-f)*gains[lo]+f*gains[hi])
        result[0]=anchor
        return result
    result={'cases':{},'limits':'Fixed-state command algebra; full contact dynamics and closed-loop counterfactuals not inferred.'}
    for case in cases:
        p=load(ROOT/resolver[case]/'precision.npz');t=p['time_s'];K=p['exact_feedback_gain'];e=p['exact_error'];loads=p['exact_estimated_support_loads_N'];phase=p['exact_effective_phase_rad'];anchor=K[0,0].copy();n=len(t)
        loaded=np.flatnonzero(loads.sum(axis=1)>.001)
        assert np.max(np.abs(K[:,0]-anchor))<1e-12
        reproduced=np.array([gain(phase[i],loads[i],anchor) for i in loaded])
        maxgap=float(np.max(abs(reproduced-K[loaded])));assert maxgap<1e-10,(case,maxgap)
        mean_e=(e[1:]+e[:-1])/2
        dK=K[1:]-K[:-1]
        kg=-np.einsum('nij,nj->ni',dK,mean_e)
        eg=-np.einsum('nij,nj->ni',(K[1:]+K[:-1])/2,e[1:]-e[:-1])
        assert np.max(abs(kg+eg-np.diff(p['exact_lqr_command'],axis=0)))<1e-9
        bounds=p['exact_input_bounds']
        ids={int(np.argmin(abs(t-x))) for x in (31.770,32.250)}
        selections={}
        for label,(a,b) in {'burst28_36':(28,36),'after32p25_44':(32.25,44)}.items():
            ii=np.flatnonzero((t[1:]>=a-1e-9)&(t[1:]<=b+1e-9))
            j=int(ii[np.argmax(np.max(abs(kg[ii])/bounds[ii+1],axis=1))]+1)
            ids.add(j);selections[label]=float(t[j])
        events=[]
        for i in sorted(ids):
            j=i-1;cur=K[i];prev=K[j];error=e[i];me=(error+e[j])/2
            a=gain(phase[i],loads[i],anchor);b=gain(phase[j],loads[j],anchor)
            if max(np.max(abs(a-cur)),np.max(abs(b-prev)))>=1e-10:
                events.append({'time_s':float(t[i]),'unavailable':'Current or previous gain held through no-load; not an instantaneous lookup pair.'});continue
            phase_oldload=gain(phase[i],loads[j],anchor)
            oldphase_newload=gain(phase[j],loads[i],anchor)
            phase_change=((a-oldphase_newload)+(phase_oldload-b))/2
            load_change=((a-phase_oldload)+(oldphase_newload-b))/2
            assert np.max(abs(phase_change+load_change-(cur-prev)))<1e-10
            dphase=-phase_change@me;dload=-load_change@me
            assert np.max(abs(dphase+dload-kg[j]))<1e-9
            other=p['exact_native_requested'][i]-p['exact_lqr_command'][i]
            u0=other-prev@error;u1=other-cur@error;du=u1-u0
            assert np.max(abs(u1-p['exact_native_requested'][i]))<1e-9
            intervals=[];lo,hi=0.,1.
            for k in range(5):
                lim=bounds[i,k]
                if abs(du[k])<1e-14:
                    l,h=(0.,1.) if abs(u0[k])<=lim+1e-12 else (1.,0.)
                else:
                    x,y=(-lim-u0[k])/du[k],(lim-u0[k])/du[k]
                    l,h=max(0.,min(x,y)),min(1.,max(x,y))
                intervals.append([float(l),float(h)]);lo=max(lo,l);hi=min(hi,h)
            # Exhaustive scalar grid is independent numerical cross-check of closed-form intersection.
            lambdas=np.linspace(0.,1.,10001)
            grid=np.max(abs(u0[None,:]+lambdas[:,None]*du[None,:])/bounds[i],axis=1)
            best=int(np.argmin(grid))
            if lo>hi:assert float(np.min(grid))>1.-1e-8
            events.append(safe({'index':i,'time_s':t[i],'previous_time_s':t[j],'phase_step_rad':phase[i]-phase[j],'loads_previous_N':loads[j],'loads_current_N':loads[i],'loadshares_previous':loads[j]/loads[j].sum(),'loadshares_current':loads[i]/loads[i].sum(),'gain_relative_change':np.linalg.norm(cur-prev)/np.linalg.norm(prev),'LQR_step_due_phase':dphase,'LQR_step_due_support_mix':dload,'LQR_step_due_error':eg[j],'requested_step':p['exact_native_requested'][i]-p['exact_native_requested'][j],'nonLQR_terms_step':(p['exact_native_requested'][i]-p['exact_lqr_command'][i])-(p['exact_native_requested'][j]-p['exact_lqr_command'][j]),'actual_applied_before':p['exact_applied'][j],'actual_applied_current':p['exact_applied'][i],'frozen_current_error_previous_gain_request':u0,'actual_current_request':u1,'blend_lambda_input_intervals':intervals,'all_inputs_feasible_blend':bool(lo<=hi),'all_inputs_intersection':[lo,hi],'grid_minimax_lambda':lambdas[best],'grid_minimax_worst_input_fraction':grid[best],'heading_command':p['exact_heading_command'][i]}))
        result['cases'][case]=dict(loaded_rows_verified=len(loaded),maximum_gain_reconstruction_error=maxgap,largest_jump_event_times_s=selections,all_trace_heading_command_max_abs=float(np.max(abs(p['exact_heading_command']))),events=events)
    write(out/'report.json',safe(result))
    for rel,h in hashes.items():assert sha(ROOT/rel)==h
    write(out/'manifest.json',{'files':[{'path':p.name,'bytes':p.stat().st_size,'sha256':sha(p)} for p in sorted(out.iterdir()) if p.is_file()]})
    print(json.dumps({c:{'gain_rows':v['loaded_rows_verified'],'gain_gap':v['maximum_gain_reconstruction_error'],'events':[(a['time_s'],a.get('all_inputs_feasible_blend'),a.get('grid_minimax_worst_input_fraction')) for a in v['events']]} for c,v in result['cases'].items()},indent=2))
if __name__=='__main__':main()
