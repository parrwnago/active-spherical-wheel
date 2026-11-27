"""Exploratory, teacher-forced I56H one-step audit. No integrated physics.

Every prediction starts from the exact measured state and known applied input.
This is neither a free-running rollout nor a forecast of future contacts.
"""
from pathlib import Path
from datetime import datetime,timezone
from contextlib import ExitStack
from unittest.mock import patch
import importlib.util,hashlib,json,shutil,sys,time,csv
import numpy as np
import mujoco,yaml

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
RUN=ROOT/'results/constrained_handoff_20261126/campaign/runs/m100_H'
EVAL=ROOT/'results/loaded_contact_validation_20261126/model_tools/evaluate_frozen_model.py'
GROUPS={'planar':((0,1),'m/s'),'transverse':((3,4),'rad/s'),'world_yaw':((9,),'rad/s'),'shaft':((6,),'rad/s'),'shell_A':((7,),'rad/s'),'shell_B':((8,),'rad/s')}
SUPPORTS=('shell_A','shell_B','belt_A','belt_B')


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
    w,x,y,z=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def forbidden(*args,**kwargs):raise AssertionError('Teacher-forced static audit attempted physics/observer step')
def stats(values,times,indices):
    x=np.asarray(values,float);valid=np.isfinite(x);x=x[valid];ii=np.asarray(indices)[valid]
    if not len(x):return dict(samples=0)
    peak=int(np.argmax(x));return dict(samples=len(x),rms=float(np.sqrt(np.mean(x*x))),median=float(np.median(x)),p90=float(np.quantile(x,.90)),p95=float(np.quantile(x,.95)),p99=float(np.quantile(x,.99)),peak=float(x[peak]),peak_index=int(ii[peak]),peak_time_s=float(times[ii[peak]]))

def main():
    if (HERE/'registered_before_arithmetic.json').exists():raise FileExistsError('Preserve prior audit attempt')
    provenance=json.loads((RUN/'provenance.json').read_text());modelpath=Path(provenance['models']['mirror_xml']);configpath=RUN/'source_snapshot/config/model.yaml'
    inputs=[Path(__file__),EVAL,ROOT/'src/spherical_wheel/handoff_model.py',ROOT/'src/spherical_wheel/contact_sensing.py',RUN/'precision.npz',RUN/'governor_diagnostics.json',RUN/'provenance.json',configpath,modelpath]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in inputs}
    save(HERE/'registered_before_arithmetic.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),iteration='I-57 post-primary exploratory',input_sha256=hashes,
        question='Is the frozen affine offset+G5actualcommand an informative one-step baseline across the already-recorded I56H44s trajectory?',samples='EVERY held interval index0..8799; retain every reconstruction failure.',
        teacher_forcing='Each5ms prediction receives exact current simulated q/v, saved observerwrench, prior actualbeltcommand and known actualappliedinput. No futurestate/truth entersmodel.',
        baselines=['Frozen affine offset+G5actualinput','Constant current velocity in world/commoninitialfork axes'],groups=GROUPS,
        frame='Observed endpointangularvelocity rotated toworld with own endpointquaternion, then R_initial.T; affine andconstantcurrent predictions use initialframe. Worldyaw separately projected.',
        evaluation_only_truth='Stratify by endpoint fourpatch supportmask(load>.05N), same/change; not continuouscontactmode inside5ms and no body-support category.',
        no_fitting_no_gates=True,no_new_physics=True,inference='Post-primary exploratory teacher-forced one-step diagnostic. Not multi-step forecast, calibrated controller cost or fullI56cause.'))
    for p in inputs:
        if p.suffix in ('.py','.yaml','.xml'):
            target=HERE/'source_snapshot'/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
    spec=importlib.util.spec_from_file_location('_i57_one_step_frozen_evaluator',EVAL);module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    from spherical_wheel.handoff_model import MeasuredContactVelocityModel
    with np.load(RUN/'precision.npz',allow_pickle=False) as z:
        fields={k:z[k].copy() for k in module.FIELDS};truth=z['truth_patch_loads_N'].copy()
    diagnostics=json.loads((RUN/'governor_diagnostics.json').read_text());config=yaml.safe_load(configpath.read_text())
    n=len(fields['time_s'])-1
    if n!=8800 or not np.all(abs(np.diff(fields['time_s'])-.005)<1e-9):raise ValueError('Expected exactly original8800held5ms intervals')
    predicted=np.full((n,10),np.nan);current=np.full_like(predicted,np.nan);observed=np.full_like(predicted,np.nan)
    mismatch=np.full(n,np.nan);confidence=np.full(n,np.nan);estimated_loads=np.full((n,4),np.nan);failures=[];names=[]
    start=time.monotonic()
    with ExitStack() as stack:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_inverse','mj_collision','mj_integratePos'):stack.enter_context(patch.object(mujoco,name,forbidden))
        model=mujoco.MjModel.from_xml_path(str(modelpath));data=mujoco.MjData(model);helper=MeasuredContactVelocityModel(model,contact_strength=provenance['candidate_settings']['contact_strength'])
        for i in range(n):
            try:
                records,measured,c=module.reconstruct(model,data,fields,i,config,provenance)
                r=helper.evaluate(fields['exact_qpos'][i],fields['exact_qvel'][i],records,horizon_s=.005,tangent_confidence=c)
                d=diagnostics[i]['diagnostics'];np.testing.assert_equal(list(r.contact_names),d['contact_names']);np.testing.assert_allclose(c,d['tangent_confidence_proxy'],atol=1e-12,rtol=0)
                np.testing.assert_allclose(r.contact_loads_N,d['modeled_contact_loads_N'],atol=1e-9,rtol=0)
                np.testing.assert_allclose(measured['slip_speed_m_s'],d['estimated_material_velocity_mismatch_m_s'],atol=1e-11,rtol=0)
                np.testing.assert_allclose([measured['loads_N'][name] for name in SUPPORTS],fields['exact_estimated_support_loads_N'][i],atol=1e-9,rtol=0)
                np.testing.assert_allclose(measured['point_m'],fields['exact_estimated_contact_point_m'][i],atol=1e-11,rtol=0)
                R0=r.fork_rotation_world;R1=rotation(fields['exact_qpos'][i+1,3:7]);actual=fields['exact_qvel'][i+1];v=fields['exact_qvel'][i];p=r.offset+r.response@fields['exact_applied'][i]
                omega_world=R1@actual[3:6];observed[i]=np.r_[actual[:3],R0.T@omega_world,actual[6:],omega_world[2]]
                predicted[i]=np.r_[p,(R0@p[3:6])[2]];current[i]=np.r_[v,(R0@v[3:6])[2]]
                mismatch[i]=measured['slip_speed_m_s'];confidence[i]=c;estimated_loads[i]=[measured['loads_N'][name] for name in SUPPORTS];names.append(list(r.contact_names))
                if not np.isfinite(np.r_[observed[i],predicted[i],current[i]]).all():raise ValueError('Nonfinite velocity output')
            except Exception as error:
                predicted[i]=np.nan;current[i]=np.nan;observed[i]=np.nan;names.append([]) if len(names)<=i else None;failures.append(dict(index=i,time_s=float(fields['time_s'][i]),exception_type=type(error).__name__,message=str(error)))
            if (i+1)%1000==0:print(json.dumps(dict(completed_intervals=i+1,failures=len(failures),elapsed_s=time.monotonic()-start)),flush=True)
    same=np.all((truth[:-1]>.05)==(truth[1:]>.05),axis=1);valid=np.isfinite(predicted).all(axis=1)&np.isfinite(observed).all(axis=1);indices=np.arange(n)
    error_model=predicted-observed;error_current=current-observed
    np.savez_compressed(HERE/'full_rate_predictions_errors.npz',time_s=fields['time_s'][:-1],next_time_s=fields['time_s'][1:],source_indices=indices,affine_prediction=predicted,current_velocity_prediction=current,observed_next_velocity=observed,affine_error=error_model,current_velocity_error=error_current,valid=valid,endpoint_fourpatch_support_same=same,truth_patch_loads_current_N=truth[:-1],truth_patch_loads_next_N=truth[1:],estimated_patch_loads_N=estimated_loads,tangent_confidence=confidence,estimated_material_velocity_mismatch_m_s=mismatch)
    report=dict(iteration='I-57 post-primary exploratory',completed_utc=datetime.now(timezone.utc).isoformat(),physics_executions=0,teacher_forced_intervals=n,successful_intervals=int(valid.sum()),failures=failures,
        horizon_s=.005,model_contact_strength=10.,model_mismatch_scale_m_s=.02,elapsed_seconds=time.monotonic()-start,
        evaluation_strata=dict(all=int(valid.sum()),endpoint_fourpatch_support_same=int((valid&same).sum()),endpoint_fourpatch_support_changed=int((valid&~same).sum())),metrics={},
        frame='Row0:9 worldtranslation3 + initialforkangular3 + relativehinges3; row9 worldyaw. Physicalnextangularvelocity transformed with actualnextpose thencommoninitialframe.',
        limitations=['Teacher-forced exact currentstate and known appliedinput at every step; no propagated predictionstate or future reference.', 'Uses original observerwrench/knownpriorbeltcommand; no truthcontact, nextstate or futureloads feed model.', 'Absolute affine prediction can fail even if paired G differences are accurate, because common offset/drift and heldcontact approximations differ.', 'Endpoint fourpatch masks do not certify continuous support inside5ms or include additional body contact.', 'No fit, acceptancegate, stability guarantee, motorcapacity claim or fulltrajectorycause.'],no_controller_model_observer_source_edits=True)
    rows=[]
    for group,(ix,units) in GROUPS.items():
        error_a=np.linalg.norm(error_model[:,list(ix)],axis=1);error_b=np.linalg.norm(error_current[:,list(ix)],axis=1);report['metrics'][group]={}
        for stratum,selection in [('all',valid),('endpoint_same',valid&same),('endpoint_changed',valid&~same)]:
            modelstat=stats(error_a[selection],fields['time_s'],indices[selection]);basestat=stats(error_b[selection],fields['time_s'],indices[selection]);ratio=modelstat['rms']/basestat['rms'] if modelstat.get('samples') and basestat.get('rms',0)>0 else None
            report['metrics'][group][stratum]=dict(units=units,affine=modelstat,constant_current=basestat,rms_ratio=ratio)
            for method,s in [('affine',modelstat),('constant_current',basestat)]:rows.append(dict(group=group,stratum=stratum,method=method,units=units,**s))
    save(HERE/'report.json',report);save(HERE/'reconstructed_contact_names.json',names)
    with (HERE/'error_statistics.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['group','stratum','method','units','samples','rms','median','p90','p95','p99','peak','peak_index','peak_time_s']);writer.writeheader();writer.writerows(rows)
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(11,9),constrained_layout=True)
    for ax,(group,(ix,units)) in zip(axes.ravel(),GROUPS.items()):
        a=np.linalg.norm(error_model[:,list(ix)],axis=1);b=np.linalg.norm(error_current[:,list(ix)],axis=1)
        ax.plot(fields['time_s'][:-1],b,lw=.6,color='#888888',label='Constant currentvelocity');ax.plot(fields['time_s'][:-1],a,lw=.6,color='#b2182b',label='Affine offset+G5u')
        ax.set_yscale('symlog',linthresh=1e-4 if units=='m/s' else 1e-3);ax.set_title(group);ax.set_xlabel('Source time, s');ax.set_ylabel('Absolute errornorm, '+units);ax.legend(fontsize=7)
    fig.suptitle('Exploratory I56H teacher-forced5ms absolute velocity prediction\nExact currentstate and known appliedinput at EVERYstep; not a multi-step forecast',fontsize=12)
    fig.savefig(HERE/'one_step_errors.png',dpi=180);fig.savefig(HERE/'one_step_errors.pdf');plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(10,9),constrained_layout=True)
    for ax,(group,(ix,units)) in zip(axes.ravel(),GROUPS.items()):
        for errors,label,color in [(error_model,'Affine','#b2182b'),(error_current,'Constant current','#888888')]:
            x=np.sort(np.linalg.norm(errors[valid][:,list(ix)],axis=1));ax.plot(x,np.arange(1,len(x)+1)/len(x),label=label,color=color)
        ax.set_xscale('symlog',linthresh=1e-4 if units=='m/s' else 1e-3);ax.set_title(group);ax.set_xlabel('Absolute errornorm, '+units);ax.set_ylabel('Empirical cumulative fraction');ax.legend(fontsize=7)
    fig.suptitle('Exploratory one-step error distributions, all recorded I56H intervals\nTeacher-forced; no calibration or acceptance gates',fontsize=12)
    fig.savefig(HERE/'one_step_error_distributions.png',dpi=180);fig.savefig(HERE/'one_step_error_distributions.pdf');plt.close(fig)
    if hashes!={str(p.relative_to(ROOT)):sha(p) for p in inputs}:raise ValueError('Source/input changed duringaudit')
    save(HERE/'input_hashes_unchanged.json',dict(passed=True,input_sha256=hashes))
    save(HERE/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(HERE)),sha256=sha(p),bytes=p.stat().st_size) for p in sorted(HERE.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(intervals=n,successful=int(valid.sum()),failures=len(failures),metrics=report['metrics']),indent=2))

if __name__=='__main__':main()
