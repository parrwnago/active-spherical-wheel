"""I55 exact-prefix short causal diagnostics using the unchanged I52 runner."""
from pathlib import Path
from contextlib import ExitStack
from unittest.mock import patch
import argparse,datetime,hashlib,importlib.util,json,os,shutil,sys,traceback
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
PARENT=ROOT/'results/contact_scheduling_joint_tracking_20261126'
CASES=('native_batch','native_substeps','zero_free_shellA','hold_attitude')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
    if isinstance(v,dict):return {str(k):safe(x) for k,x in v.items()}
    if isinstance(v,(tuple,list)):return [safe(x) for x in v]
    if isinstance(v,np.ndarray):return safe(v.tolist())
    if isinstance(v,np.generic):return safe(v.item())
    if isinstance(v,Path):return str(v)
    if isinstance(v,float) and not np.isfinite(v):return None
    return v

def save(p,v):Path(p).write_text(json.dumps(safe(v),indent=2,allow_nan=False)+'\n')
def digest(v):return hashlib.sha256(json.dumps(safe(v),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def module(p,name):
    spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m

def history(controller):
    # Selected stateful numeric history; physical solver state is preserved by replaying every native prefix step.
    names=('nominal_joints','_vref','_held','_sync','heading_reference','_slow_normal','_fall_end','_yaw_error','_plane')
    h={k:getattr(controller,k) for k in names if hasattr(controller,k)}
    h['estimator']=vars(controller.estimator).copy()
    h['observer']={k:getattr(controller.observer,k) for k in ('generalized','interval_mean','_previous')}
    h['synchronizer_plane']=controller.synchronizer._plane
    guard=getattr(controller,'_i51_joint_reference_guard',None)
    h['guard']=None if guard is None else guard.state_dict()
    return digest(h)

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--case',choices=CASES,required=True);ap.add_argument('--run',action='store_true');args=ap.parse_args()
    if not args.run:raise SystemExit('Explicit --run is required for new Gazebo integration')
    protocol=json.loads((HERE/'protocol.json').read_text());case=args.case;out=HERE/'campaign'/case
    out.mkdir(parents=True,exist_ok=False)
    for rel in ('protocol.json','campaign/jobs.json'):
        dst=out/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(PARENT/rel,dst)
    executed=[Path(__file__),HERE/'protocol.json']
    if case!='native_batch':executed.append(HERE/'substep_trace.py')
    for p in executed:
        target=out/'i55_source'/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
    provenance={'iteration':'I-55','case':case,'registered_protocol_sha256':sha(HERE/'protocol.json'),'captured_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'executed_source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in executed},'source_semantics':'Original I52 runner is imported from its original path without editing. Its inherited exact_native_* fields describe returned final values; sidecar pre_output_* records native computation before actuator override, which already includes attitude override in that arm.','actual_requested_horizon_s':protocol['duration_s'],'original_job_horizon_s':44.,'parent_path_and_conditions_unchanged':True,'no_branch_state_restore':True,'internal_history_fingerprint':'Selected numerical controller state, not an independent dump of all DART solver state. Full native prefix is replayed.'}
    save(out/'i55_provenance_before.json',provenance)
    parent=module(PARENT/'run_factorial_case.py','_i55_original_i52_runner')
    import spherical_wheel.gazebo.experiments as gx
    from spherical_wheel.offroad_control import OffroadController
    true_command=OffroadController.command;true_feedback=OffroadController._terrain_feedback
    true_plant=gx.GazeboWorkspace.plant;true_run=gx.run_gazebo
    with np.load(PARENT/'campaign/runs/m100_R/precision.npz',allow_pickle=False) as z:
        old={k:z[k].copy() for k in ('time_s','exact_qpos','exact_qvel','exact_applied','exact_native_requested','exact_observer_wrench','exact_estimated_normal','exact_nominal_joints_before','exact_feedback_gain','exact_feedback_error')}
    branch_index={'zero_free_shellA':6450,'hold_attitude':6316}.get(case,None)
    rows=[];feedback_capture={};frozen=None;traces=[];contexts=[];workspace_model_paths={};prefix_max=0.
    def plant(workspace,terrain):
        physical=true_plant(workspace,terrain)
        if not workspace_model_paths:
            workspace_model_paths.update(mirror_xml=str(workspace.mirror().xml_path) if hasattr(workspace.mirror(),'xml_path') else str(workspace.directory/'mirror/model.xml'),physical_sdf=str(physical.world_path.parent/'models/active_spherical_wheel/model.sdf'),world=str(physical.world_path))
            # The actual mirror/model.xml is generated by StateMirror in workspace.directory/mirror.
            workspace_model_paths['mirror_xml']=str(workspace.directory/'mirror/model.xml')
        if case!='native_batch' and not traces:
            instrumentation=module(HERE/'substep_trace.py','_i55_substep_trace')
            trace=instrumentation.SubstepTrace(physical,start_s=31.4,end_s=32.5,physics_dt_s=.001)
            context=trace.instrument();context.__enter__();traces.append(trace);contexts.append(context)
        return physical
    def run(workspace,terrain,path,params=None,**kwargs):
        kwargs['duration_s']=protocol['duration_s']
        return true_run(workspace,terrain,path,params,**kwargs)
    def feedback(controller,qref,vref,feedforward,normal,measurement,touching):
        nonlocal frozen
        index=int(round(float(controller.plant.data.time)/controller.plant.dt))
        native=qref[3:7].copy();active=case=='hold_attitude' and 6316<=index<6349
        if active:
            if frozen is None:frozen=native.copy()
            qref[3:7]=frozen
        feedback_capture.update(native_qref_quat=native,used_qref_quat=qref[3:7].copy(),reference_override_active=active)
        return true_feedback(controller,qref,vref,feedforward,normal,measurement,touching)
    def command(controller,xy,velocity,acceleration):
        nonlocal prefix_max
        t=float(controller.plant.data.time);index=int(round(t/controller.plant.dt));before=history(controller)
        # Abort BEFORE intervention if the native physical prefix does not reproduce the original state.
        if branch_index is not None and index<=branch_index:
            gap=max(float(np.max(abs(controller.plant.data.qpos-old['exact_qpos'][index]))),float(np.max(abs(controller.plant.data.qvel-old['exact_qvel'][index]))))
            prefix_max=max(prefix_max,gap)
            if gap!=0.:raise AssertionError(f'Original-state prefix mismatch before treatment at {t}: {gap}')
        applied,info=true_command(controller,xy,velocity,acceleration)
        pre_req=np.array(info['requested'],copy=True);pre_applied=np.array(applied,copy=True);pre_clip=np.array(info['clipping_command_delta'],copy=True)
        req=pre_req.copy();output_active=case=='zero_free_shellA' and 6450<=index<6455
        if output_active:
            req[1]=0.
            applied=np.clip(req,-controller.plant.bounds,controller.plant.bounds)
            info=dict(info);info['requested']=req.copy();info['clipping_command_delta']=applied-req
        if feedback_capture['reference_override_active']:
            info=dict(info);info['reference_quat']=feedback_capture['used_qref_quat'].copy()
        row={'time_s':t,'control_index':index,'history_before_sha256':before,
            'pre_output_requested':pre_req,'pre_output_applied':pre_applied,'pre_output_clipping_delta':pre_clip,
            'final_requested':np.array(info['requested'],copy=True),'final_applied':np.array(applied,copy=True),
            'output_request_delta':np.asarray(info['requested'])-pre_req,'output_applied_delta':np.asarray(applied)-pre_applied,
            'output_override_active':output_active,
            **feedback_capture}
        rows.append(safe(row))
        # Complete measured native history must match before the first intervention; t0 native calculation too for output-only arm.
        if branch_index is not None and (index<branch_index or (case=='zero_free_shellA' and index==branch_index)):
            pairs=[(pre_req,old['exact_native_requested'][index]),(pre_applied,old['exact_applied'][index]),(info['feedback_gain'],old['exact_feedback_gain'][index]),(info['error'],old['exact_feedback_error'][index]),(np.r_[controller.observer.wrench()],old['exact_observer_wrench'][index])]
            for a,b in pairs:np.testing.assert_array_equal(a,b)
        return applied,info
    success=False
    try:
        with patch.object(gx.GazeboWorkspace,'plant',plant),patch.object(gx,'run_gazebo',run),patch.object(OffroadController,'command',command),patch.object(OffroadController,'_terrain_feedback',feedback):
            parent.run_case(out,'m100_R',.3)
        success=True
    except BaseException:
        save(out/'i55_failure.json',{'traceback':traceback.format_exc(),'rows':len(rows),'prefix_max_state_gap':prefix_max})
        raise
    finally:
        for context in reversed(contexts):context.__exit__(None,None,None)
        if rows:
            np.savez_compressed(out/'i55_sidecar.npz',**{k:np.asarray([r[k] for r in rows]) for k in rows[0]})
        for trace in traces:trace.save(out/'substeps.npz')
        native_manifest=out/'campaign/runs/m100_R/artifact_manifest.json'
        if native_manifest.exists():shutil.copyfile(native_manifest,out/'i52_runner_artifact_manifest_before_i55_metadata.json')
        provenance.update(success=success,sidecar_rows=len(rows),original_state_prefix_max_abs_gap=prefix_max,models=workspace_model_paths)
        provenance['model_sha256']={k:sha(p) for k,p in workspace_model_paths.items() if Path(p).is_file()}
        for name in ('mirror_xml','physical_sdf'):
            if name in workspace_model_paths:
                p=Path(workspace_model_paths[name]);dst=out/'i55_models'/name/ p.name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dst)
        for rel,h in provenance['executed_source_sha256'].items():assert sha(ROOT/rel)==h,rel
        save(out/'i55_provenance.json',provenance)
        save(out/'i55_capture_manifest.json',{'iteration':'I-55','case':case,'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)} for p in sorted(out.rglob('*')) if p.is_file() and p.name!='i55_capture_manifest.json']})
    if success:
        # Baseline comparisons include every overlapping5ms precision field and the complete observed horizon.
        path=out/'campaign/runs/m100_R/precision.npz'
        with np.load(path,allow_pickle=False) as new,np.load(PARENT/'campaign/runs/m100_R/precision.npz',allow_pickle=False) as previous:
            count=len(new['time_s']);equal={k:bool(np.array_equal(new[k],previous[k][:count],equal_nan=True)) for k in new.files if k in previous.files}
        save(out/'historical_precision_comparison.json',{'exact_fields':sum(equal.values()),'common_fields':len(equal),'all_common_fields_exact':all(equal.values()),'fields':equal,'scope':'Fullhorizoncomparison meaningfulfornativecases; intervenedcasesexpectedtodivergeaftertreatment.'})
        save(out/'i55_artifact_manifest.json',{'iteration':'I-55','case':case,'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)} for p in sorted(out.rglob('*')) if p.is_file() and p.name!='i55_artifact_manifest.json']})
        print(json.dumps({'I55_case':case,'success':success,'rows':len(rows),'full_historical_exact':all(equal.values()),'models':workspace_model_paths}),flush=True)
if __name__=='__main__':main()
