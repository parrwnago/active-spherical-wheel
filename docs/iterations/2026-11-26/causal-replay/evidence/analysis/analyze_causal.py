"""Registered I55 saved-data gates and momentum/contact accounting."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,csv,hashlib,json,shutil,traceback,xml.etree.ElementTree as ET
import numpy as np
import budget_math as bm

H=Path(__file__).resolve().parent;FAMILY=H.parent;ROOT=FAMILY.parents[1]
OLD=ROOT/'results/contact_scheduling_joint_tracking_20261126/campaign/runs/m100_R'
CASES=('native_batch','native_substeps','zero_free_shellA','hold_attitude')
START={'zero_free_shellA':32.250,'hold_attitude':31.580}
INTERVALS=((31.4,31.58),(31.58,31.745),(32.245,32.275),(32.250,32.275),(32.250,32.305),(32.250,32.500))
ENDPOINTS=(31.580,31.745,32.245,32.250,32.255,32.275,32.305,32.500)
SUBSTEP_ENDPOINTS=(32.251,32.255)
TERMS=('equilibrium_command','rolling_feedforward','support_command','lqr_command','transition_command_delta','heading_command','fade_command_delta')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v)for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float)and not np.isfinite(x):return None
    return x
def save(p,x):p.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def exact(a,b):
    a,b=np.asarray(a),np.asarray(b)
    return a.shape==b.shape and bool(np.array_equal(a,b,equal_nan=True)if a.dtype.kind in'fc'and b.dtype.kind in'fc'else np.array_equal(a,b))
def index(t,at):
    i=int(np.argmin(abs(t-at)))
    if abs(t[i]-at)>1e-8:raise ValueError(f'Unavailable exact endpoint {at}; nearest {t[i]}')
    return i
def degrees_between(a,b):return np.degrees(np.arccos(np.clip(np.sum(a*b,axis=-1),-1,1)))
def up(quat):return bm.rotation(quat)[...,2]
def prefix_compare(a,b,mask=None):
    n=len(a['time_s']);m=len(b['time_s']);rows=np.arange(n)if mask is None else np.flatnonzero(mask);result={}
    for k in sorted(set(a)&set(b)):
        x,y=a[k],b[k]
        if x.ndim and y.ndim and len(x)==n and len(y)==m:result[k]=exact(x[rows],y[rows])
        else:result[k]=exact(x,y)
    return {'all_shared_fields_exact':bool(result and all(result.values())),'shared_field_count':len(result),'compared_rows':len(rows),'fields':result}
def copy_source(out,paths):
    for p in paths:
        dst=out/'source_snapshot'/p.relative_to(ROOT);dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dst)


def selected_scope(case,p,s):
    t=s['time_s'];bounds=p['exact_input_bounds'];bounds=np.broadcast_to(bounds,s['final_requested'].shape)
    activeA=(t>=32.25-1e-9)&(t<32.275-1e-9)if case=='zero_free_shellA'else np.zeros(len(t),bool)
    activeQ=(t>=31.58-1e-9)&(t<31.745-1e-9)if case=='hold_attitude'else np.zeros(len(t),bool)
    wanted=s['pre_output_requested'].copy();wanted[activeA,1]=0
    qgood=exact(s['native_qref_quat'][~activeQ],s['used_qref_quat'][~activeQ])
    if activeQ.any():qgood=qgood and exact(s['used_qref_quat'][activeQ],np.tile(s['native_qref_quat'][np.flatnonzero(activeQ)[0]],(activeQ.sum(),1)))
    checks={'output_active_flags_exact':exact(s['output_override_active'],activeA),'reference_active_flags_exact':exact(s['reference_override_active'],activeQ),'only_authorized_request_index_changed':exact(wanted,s['final_requested']),'final_clip_exact':exact(np.clip(wanted,-bounds,bounds),s['final_applied']),'request_delta_identity':np.max(abs(s['pre_output_requested']+s['output_request_delta']-s['final_requested']))<1e-12,'applied_delta_identity':np.max(abs(s['pre_output_applied']+s['output_applied_delta']-s['final_applied']))<1e-12,'quaternion_override_scope_exact':qgood,'precision_final_requested_exact':exact(p['exact_native_requested'],s['final_requested']),'precision_final_applied_exact':exact(p['exact_applied'],s['final_applied']),'time_and_control_index_exact':exact(p['time_s'],t)and exact(np.arange(len(t)),s['control_index'])}
    keys=['exact_'+k for k in TERMS]
    if all(k in p for k in keys):checks['stored_native_terms_equal_pre_actuator_request']=bool(np.max(abs(sum(p[k]for k in keys)-s['pre_output_requested']))<1e-9)
    if 'exact_previous_applied'in p:checks['observer_known_previous_applied_exact']=exact(p['exact_previous_applied'][1:],s['final_applied'][:-1])
    return {'passes':all(checks.values()),'checks':checks,'masked_holds':int(activeA.sum()),'held_reference_holds':int(activeQ.sum()),'semantics':'Pre-output request already includes frozen-reference feedback in hold_attitude; it is not an untreated counterfactual command.'}


def clock_gate(p,z):
    before,after=z['bridge_state_before'],z['bridge_state'];dt=after[:,0]-before[:,0];n=len(dt);hi=z['hold_index'];sub=z['substep_index'];checks={}
    checks['state_schema_exact']=exact(z['state_fields'],bm.STATE_FIELDS)
    checks['window_has_220_holds_1100_steps']=n==1100 and exact(hi,np.repeat(np.arange(6280,6500),5))
    checks['substep_index_1to5']=exact(sub,np.tile(np.arange(1,6),n//5))and n%5==0
    checks['hold_completed_all']=bool(z['hold_completed'].all())
    checks['before_after_chain_exact']=exact(before[1:],after[:-1])
    checks['one_ms_physics_dt']=bool(np.max(abs(dt-.001))<=1e-8 and np.max(abs(after[:,21]-.001))<=1e-12)
    checks['plugin_iterations_consecutive']=bool(np.all(after[:,1]-before[:,1]==1)and np.all(np.diff(after[:,1])==1))
    checks['resets_unchanged']=bool(np.all(after[:,22]==before[:,22])and np.all(after[:,23]==0))
    checks['relative_endpoint_clock']=bool(np.max(abs(z['time_s']-(z['control_start_time_s']+sub*.001)))<=1e-9 and np.max(abs(z['control_end_time_s']-z['control_start_time_s']-.005))<=1e-9)
    checks['actual_held_input_equals_issued']=exact(z['held_input'],p['exact_applied'][hi])
    checks['world_disturbance_zero']=bool(np.max(abs(z['disturbance_world']))==0)
    checks['ragged_contact_arrays_exact']=exact(z['contact_tags'],z['contact_rows'][:,0].astype(np.int64))and exact(z['contact_admitted'],z['contact_rows'][:,12]>=.5)
    checks['plugin_clock_fields_exact']=exact(z['bridge_time_s'],after[:,0])and exact(z['bridge_iterations'],after[:,1].astype(np.int64))
    checks['plugin_clock_offset_stable']=bool(np.ptp(after[:,0]-z['time_s'])<1e-8)
    q,v=bm.state_qv(after);ends=np.flatnonzero(sub==5);j=hi[ends]+1
    qgap=float(np.max(abs(q[ends]-p['exact_qpos'][j])));vgap=float(np.max(abs(v[ends]-p['exact_qvel'][j])))
    checks['precision_endpoints_match']=qgap<=1e-12 and vgap<=1e-12
    return {'passes':all(checks.values()),'checks':checks,'substep_rows':n,'maximum_qpos_endpoint_gap':qgap,'maximum_qvel_transport_endpoint_gap':vgap,'plugin_minus_relative_clock_offset_s':float(np.median(after[:,0]-z['time_s'])),'plugin_minus_relative_clock_offset_range_s':float(np.ptp(after[:,0]-z['time_s']))}


def budgets(z,mb,ma):
    centers={'prior':mb['systemCOM_world_m'],'midpoint':.5*(mb['systemCOM_world_m']+ma['systemCOM_world_m']),'post':ma['systemCOM_world_m']}
    contacts={name:bm.contact_moments(z['contact_rows'],z['contact_offsets'],center)for name,center in centers.items()}
    dt=z['bridge_state'][:,0]-z['bridge_state_before'][:,0];axis=.5*(mb['shell_axis_world']+ma['shell_axis_world']);u=z['held_input'];motor=np.zeros((len(dt),4,3));motor[:,1]=-(u[:,1]+u[:,2])[:,None]*axis;motor[:,2]=u[:,1,None]*axis;motor[:,3]=u[:,2,None]*axis
    shaftaxis=.5*(mb['shaft_axis_world']+ma['shaft_axis_world']);shaftmotor=np.zeros_like(motor);shaftmotor[:,0]=-u[:,0,None]*shaftaxis;shaftmotor[:,1]=u[:,0,None]*shaftaxis
    rmid=.5*(mb['bodyCOM_world_m']+ma['bodyCOM_world_m'])-.5*(mb['systemCOM_world_m']+ma['systemCOM_world_m'])[:,None,:]
    transport=-ma['body_masses_kg'][None,:,None]*np.cross(rmid,(ma['systemCOM_velocity_world_m_s']-mb['systemCOM_velocity_world_m_s'])[:,None,:])
    assert np.max(abs(transport.sum(axis=1)))<1e-12
    contactF=contacts['midpoint']['force_by_body_world_N'].sum(axis=1)
    linear_state=ma['mass_kg']*(ma['systemCOM_velocity_world_m_s']-mb['systemCOM_velocity_world_m_s'])
    linear_force=dt[:,None]*(contactF+ma['mass_kg']*ma['gravity_world_m_s2']+z['disturbance_world'][:,:3])
    linear_residual=linear_force-linear_state
    out=[]
    for a,b in INTERVALS:
        ids=np.flatnonzero((z['time_s']>a+1e-9)&(z['time_s']<=b+1e-9));first,last=ids[0],ids[-1]
        assert abs((z['time_s'][first]-.001)-a)<1e-8 and abs(z['time_s'][last]-b)<1e-8
        delta=ma['H_by_body_world'][last,:,2]-mb['H_by_body_world'][first,:,2];total=delta.sum();ci={name:np.sum(c['torque_by_body_world_Nm'][ids,:,2]*dt[ids,None],axis=0)for name,c in contacts.items()};mi=np.sum(motor[ids,:,2]*dt[ids,None],axis=0);si=np.sum(shaftmotor[ids,:,2]*dt[ids,None],axis=0);ti=transport[ids,:,2].sum(axis=0)
        out.append({'start_s':a,'end_s':b,'substep_count':len(ids),'delta_H_by_body_z_kg_m2_s':dict(zip(bm.NAMES,delta)),'delta_H_total_z_kg_m2_s':total,'delta_H_fork_carrier_z_kg_m2_s':delta[:2].sum(),'delta_H_shells_z_kg_m2_s':delta[2:].sum(),'contact_by_body_yaw_impulse_Nm_s':{name:dict(zip(bm.NAMES,v))for name,v in ci.items()},'closure_by_center':{name:bm.closure(total,v.sum())for name,v in ci.items()},'primary_center':'midpoint','known_shell_motor_couple_impulse_by_body_Nm_s':dict(zip(bm.NAMES,mi)),'known_shaft_motor_couple_impulse_by_body_Nm_s':dict(zip(bm.NAMES,si)),'moving_systemCOM_transport_impulse_by_body_Nm_s':dict(zip(bm.NAMES,ti)),'body_delta_minus_contact_motor_transport_remainder_Nm_s':dict(zip(bm.NAMES,delta-ci['midpoint']-mi-si-ti)),'linear_momentum_convention_QA':{'state_delta_p_N_s':linear_state[ids].sum(axis=0),'all_contact_gravity_push_impulse_N_s':linear_force[ids].sum(axis=0),'force_minus_state_residual_N_s':linear_residual[ids].sum(axis=0),'per_step_residual_rms_vector_N_s':np.sqrt(np.mean(linear_residual[ids]**2,axis=0)),'per_step_residual_max_norm_N_s':np.linalg.norm(linear_residual[ids],axis=1).max(),'scope':'Raw exported force sign used unchanged; no posthoc sign selection. Additional convention QA, not altered registered yaw acceptance rule.'},'remainder_scope':'Includes joint-constraint/passive transfer and any model/force-accounting/discretization residual; not isolated motor effect or separately measured bearing torque. Moving-COM transport is approximated explicitly using midpoint relative COM positions and measured COM-velocity increment; it cancels in the total. Gravity has zero world-z moment. All exported contact forces included; no admission/positive-load exclusion.'})
    mask=(z['control_start_time_s']>=32.25-1e-9)&(z['control_start_time_s']<32.275-1e-9);c=contacts['midpoint'];Amax=float(c['shellA_sum_force_norm_N'][mask].max());bodymax=float(c['body_sum_force_norm_N'][mask].max())
    coverage={'mask_substeps':int(mask.sum()),'shellA_sum_force_norm_max_N':Amax,'shellA_positive_normal_load_max_N':float(c['shellA_positive_normal_load_N'][mask].max()),'body_sum_force_norm_max_N':bodymax,'clean_unloaded_comparison_force_coverage':Amax<=.05 and bodymax<=1e-12,'rejected_force_norm_max_N':float(c['rejected_sum_force_norm_N'].max()),'all_solver_rows':c['all_rows_count'],'rejected_rows':c['rejected_rows_count'],'linear_convention_QA_context':{'gravity_world_m_s2':ma['gravity_world_m_s2'],'residual_rms_vector_N_s':np.sqrt(np.mean(linear_residual**2,axis=0)),'maximum_residual_norm_N_s':np.linalg.norm(linear_residual,axis=1).max(),'net_force_minus_state_residual_N_s':linear_residual.sum(axis=0)}}
    return out,coverage


def endpoint(p,s,at,baseline_side):
    i=index(p['time_s'],at);q=p['exact_qpos'][i];v=p['exact_qvel'][i];r=bm.rotation(q[3:7]);actual=r[:,2];native=up(s['native_qref_quat'][i]);used=up(s['used_qref_quat'][i]);bup=up(baseline_side['native_qref_quat'][index(baseline_side['time_s'],at)]);held=up(baseline_side['native_qref_quat'][index(baseline_side['time_s'],31.580)])
    world=r@v[3:6];x=r[:,0];heading=float(np.arctan2(x[1],x[0]));t=p['time_s'];allworld=np.einsum('nij,nj->ni',bm.rotation(p['exact_qpos'][:,3:7]),p['exact_qvel'][:,3:6]);unwrapped,headrate,valid=bm.heading(p['exact_qpos'],allworld)
    return {'time_s':t[i],'actual_up':actual,'absolute_tilt_deg':degrees_between(actual,np.array([0.,0.,1.])),'relative_to_branch_native_target_deg':degrees_between(actual,native),'relative_to_branch_used_target_deg':degrees_between(actual,used),'relative_to_held_target_before_release_deg':degrees_between(actual,held),'relative_to_untreated_same_clock_target_deg':degrees_between(actual,bup),'branch_native_target_up':native,'used_target_up':used,'untreated_same_clock_target_up':bup,'platform_world_omega_rad_s':world,'heading_unwrapped_rad':unwrapped[i],'heading_derivative_rad_s':headrate[i],'heading_valid':valid[i],'relative_joint_rates_rad_s':v[6:9],'actual_xy':q[:2],'linear_velocity_m_s':v[:3],'requested':s['final_requested'][i],'applied':s['final_applied'][i],'pre_actuator_override_requested':s['pre_output_requested'][i],'LQR_error':p['exact_error'][i],'state_at_endpoint_precedes_current_command':True}


def substep_endpoints(p,s,z,mb,ma):
    q=ma['qpos'];omega=ma['omega_by_body_world_rad_s'][:,0]
    seq=np.r_[mb['qpos'][:1],q];wseq=np.r_[mb['omega_by_body_world_rad_s'][:1,0],omega]
    h,hr,hvalid=bm.heading(seq,wseq)
    fullomega=np.einsum('nij,nj->ni',bm.rotation(p['exact_qpos'][:,3:7]),p['exact_qvel'][:,3:6]);fh,_,_=bm.heading(p['exact_qpos'],fullomega)
    offset=fh[index(p['time_s'],31.4)]-h[0];h=h[1:]+offset;hr=hr[1:];hvalid=hvalid[1:]
    out={}
    for at in SUBSTEP_ENDPOINTS:
        i=index(z['time_s'],at);j=int(z['hold_index'][i]);r=bm.rotation(q[i,3:7]);actual=r[:,2]
        out[str(at)]={'time_s':z['time_s'][i],'interval_start_s':z['time_s'][i]-.001,'control_start_s':z['control_start_time_s'][i],'platform_world_omega_rad_s':omega[i],'heading_unwrapped_rad':h[i],'heading_derivative_rad_s':hr[i],'heading_valid':hvalid[i],'actual_up':actual,'absolute_tilt_deg':degrees_between(actual,np.array([0.,0.,1.])),'relative_to_current_hold_target_deg':degrees_between(actual,up(s['used_qref_quat'][j])),'actual_xy':q[i,:2],'linear_velocity_m_s':ma['qvel'][i,:3],'relative_joint_rates_rad_s':ma['qvel'][i,6:9],'H_by_body_world_kg_m2_s':ma['H_by_body_world'][i],'H_total_world_kg_m2_s':ma['H_total_world'][i],'actual_held_input':z['held_input'][i],'sample_precedes_next_controller_update':True,'is_control_hold_end':bool(z['substep_index'][i]==5)}
    return out


def main(output):
    output.mkdir(exist_ok=False)
    proof=FAMILY/'review/budget_source_review_stage01/report.json'
    source=[Path(__file__),H/'budget_math.py',FAMILY/'protocol.json',FAMILY/'protocol_clarifications.json',FAMILY/'run_diagnostic.py',FAMILY/'substep_trace.py',proof]
    source=[p for p in source if p.exists()]
    paths=source+[OLD/'precision.npz']
    provenance={};casefolders={c:FAMILY/'campaign'/c for c in CASES}
    for case,f in casefolders.items():
        if (f/'i55_failure.json').exists():raise RuntimeError(f'Case execution failure retained: {f}')
        for name in ['i55_artifact_manifest.json','i55_provenance.json','i55_sidecar.npz','campaign/runs/m100_R/precision.npz','campaign/runs/m100_R/summary.json']:
            if not(f/name).exists():raise FileNotFoundError(f/name)
            paths.append(f/name)
        if case!='native_batch':paths.append(f/'substeps.npz')
        provenance[case]=json.loads((f/'i55_provenance.json').read_text())
        for k in ['mirror_xml','physical_sdf']:
            p=Path(provenance[case]['models'][k]);paths.append(p)
            if sha(p)!=provenance[case]['model_sha256'][k]:raise AssertionError('Model differs from case source')
        xml=Path(provenance[case]['models']['mirror_xml'])
        for node in ET.parse(xml).getroot().findall('./asset/mesh'):paths.append((xml.parent/node.attrib['file']).resolve())
    paths=list(dict.fromkeys(paths));hashes={str(p.relative_to(ROOT)):sha(p)for p in paths};copy_source(output,source)
    save(output/'registered_before_reduction.json',{'utc':datetime.now(timezone.utc).isoformat(),'input_sha256':hashes,'cases':CASES,'intervals_s':INTERVALS,'endpoints_s':ENDPOINTS,'additional_mechanistic_1ms_endpoints_s':SUBSTEP_ENDPOINTS,'first_hold_comparison':'All non-A held inputs, disturbances and friction exact over32.250–32.255; later other-input differences are mediated feedback and reported separately.','primary_contact_center':'midpoint systemCOM; same exported point, no reprojection','contact_integration':'Right-end last-solve force × actual1msdt. Captured versioned DART/gz-physics source supports force=lambda/dt; reconstruction remains conditional on sign/order/pass-through/completeness. Raw force sign unchanged. Prior/post COM sensitivities mandatory, not selected by closure. All solver rows included.','closure_gate':'abs(contactImpulse−DeltaHz)<=max(1e−4Nms,.1absDeltaHz), diagnostic not a control adoption gate.','prefix_gate':'All shared native5ms fields exact; branch state/observer/history through t0 and command fields strictly before t0 exact.','endpoint_transport_tolerance':1e-12,'attitude_primary':'31.745 measuredstate before first releasecommand; compare actualup against common held31.580 target, branch current native target AND baseline same-clock native target.','moving_COM_transport':'Per-body/group impulse budget includes −mi rrel_mid cross DeltaV_systemCOM; transport sums to zero. All known shell and shaft motor couples are separately included. Remainder still includes joint-constraint/passive/discretization and model/force-accounting differences.','source_claim':'Definitions fixed before new branch outcomes. Root preliminary zero-A state differences and the sealed early native/zero-A reduction were known before this final consolidation; hold_attitude NPZ outcomes remain unread. No thresholds, windows, interventions, or endpoint definitions changed.'})
    p={c:load(casefolders[c]/'campaign/runs/m100_R/precision.npz')for c in CASES};s={c:load(casefolders[c]/'i55_sidecar.npz')for c in CASES};old=load(OLD/'precision.npz')
    report={'study':'I-55 causal diagnostic; no controller candidate or adoption screen','gates':{},'cases':{},'paired_outcomes':{}}
    report['gates']['native_batch_historical']=prefix_compare(p['native_batch'],old)
    report['gates']['native_substeps_batch_equivalence']=prefix_compare(p['native_substeps'],p['native_batch'])
    traces={}
    for c in CASES:
        f=casefolders[c];summary=json.loads((f/'campaign/runs/m100_R/summary.json').read_text());record={'selective_intervention':selected_scope(c,p[c],s[c]),'completion':{'diagnostic_horizon_s':32.5,'recorded_end_s':p[c]['time_s'][-1],'complete_diagnostic_horizon':abs(p[c]['time_s'][-1]-32.5)<1e-8,'summary_completed':summary['completed'],'summary_failure':summary['failure'],'original_route_duration_s':40.,'not_a_complete44s_route':True}}
        if c in START:
            t0=START[c];m=p[c]['time_s']<t0-1e-9;pre=prefix_compare(p[c],p['native_substeps'],m);i=index(p[c]['time_s'],t0);statekeys=['exact_qpos','exact_qvel','exact_observer_wrench','exact_estimated_normal','exact_nominal_joints_before','exact_previous_applied']
            measured={k:exact(p[c][k][i],p['native_substeps'][k][i])for k in statekeys if k in p[c]and k in p['native_substeps']}
            hmask=s[c]['time_s']<=t0+1e-9;hist=exact(s[c]['history_before_sha256'][hmask],s['native_substeps']['history_before_sha256'][hmask]);record['prefix']={'strict_pre_t0_all_fields':pre,'t0_measured_fields':measured,'selected_history_fingerprints_through_t0_exact':hist,'passes':pre['all_shared_fields_exact']and all(measured.values())and hist,'solver_scope':'Full native prefix replayed, selected controller-state fingerprint checked; DART hidden solver state is not independently dumped.'}
        record['endpoints']={str(at):endpoint(p[c],s[c],at,s['native_substeps'])for at in ENDPOINTS}
        if c!='native_batch':
            z=load(f/'substeps.npz');traces[c]=z;record['clock']=clock_gate(p[c],z);mp=provenance[c]['models'];mb=bm.momenta(mp['mirror_xml'],mp['physical_sdf'],z['bridge_state_before']);ma=bm.momenta(mp['mirror_xml'],mp['physical_sdf'],z['bridge_state']);record['momentum_intervals'],record['unloaded_shellA_coverage']=budgets(z,mb,ma);record['inertia_checks']=ma['inertia_checks'];record['max_chain_omega_gap_rad_s']=max(mb['max_chain_omega_gap_rad_s'],ma['max_chain_omega_gap_rad_s']);record['substep_endpoints']=substep_endpoints(p[c],s[c],z,mb,ma)
            np.savez_compressed(output/(c+'_momentum.npz'),time_s=z['time_s'],H_before_by_body_world=mb['H_by_body_world'],H_after_by_body_world=ma['H_by_body_world'],systemCOM_before_world_m=mb['systemCOM_world_m'],systemCOM_after_world_m=ma['systemCOM_world_m'],shell_axis_before_world=mb['shell_axis_world'],shell_axis_after_world=ma['shell_axis_world'])
        report['cases'][c]=record
    nativeOK=report['gates']['native_batch_historical']['all_shared_fields_exact']and report['gates']['native_substeps_batch_equivalence']['all_shared_fields_exact']
    z,native=traces['zero_free_shellA'],traces['native_substeps'];first=(z['time_s']>32.25+1e-9)&(z['time_s']<=32.255+1e-9);later=(z['time_s']>32.255+1e-9)&(z['time_s']<=32.275+1e-9);notA=[0,2,3,4]
    firstgood=exact(z['held_input'][first][:,notA],native['held_input'][first][:,notA])and exact(z['disturbance_world'][first],native['disturbance_world'][first])and exact(z['friction'][first],native['friction'][first])
    report['gates']['zero_A_first_hold_other_inputs_exact']={'passes':firstgood,'first_hold_substeps':int(first.sum()),'later_mediated_max_other_input_delta':np.max(abs(z['held_input'][later][:,notA]-native['held_input'][later][:,notA]),axis=0),'other_input_indices':notA,'scope':'Only the first5ms has matched other held inputs. Later feedback naturally responds to altered state/history; full25ms effect is a closed-loop mediated intervention effect.'}
    report['gates']['technical_causal_prerequisites_pass']=nativeOK and firstgood and all(d['selective_intervention']['passes']and d['completion']['complete_diagnostic_horizon']and d.get('prefix',{'passes':True})['passes']and d.get('clock',{'passes':True})['passes']for d in report['cases'].values())
    for c in START:
        rows={}
        for at in ENDPOINTS:
            a,b=report['cases'][c]['endpoints'][str(at)],report['cases']['native_substeps']['endpoints'][str(at)]
            rows[str(at)]={'world_z_rate_delta_rad_s':a['platform_world_omega_rad_s'][2]-b['platform_world_omega_rad_s'][2],'heading_delta_deg':np.degrees(a['heading_unwrapped_rad']-b['heading_unwrapped_rad']),'absolute_tilt_delta_deg':a['absolute_tilt_deg']-b['absolute_tilt_deg'],'branch_current_native_relative_error_delta_deg':a['relative_to_branch_native_target_deg']-b['relative_to_branch_native_target_deg'],'fixed_untreated_target_error_delta_deg':a['relative_to_untreated_same_clock_target_deg']-b['relative_to_untreated_same_clock_target_deg'],'actual_up_separation_deg':degrees_between(a['actual_up'],b['actual_up']),'XY_delta_m':np.asarray(a['actual_xy'])-b['actual_xy'],'relative_joint_rate_delta_rad_s':np.asarray(a['relative_joint_rates_rad_s'])-b['relative_joint_rates_rad_s']}
        report['paired_outcomes'][c]=rows
    report['paired_substep_outcomes']={}
    for c in START:
        report['paired_substep_outcomes'][c]={}
        for at in SUBSTEP_ENDPOINTS:
            a,b=report['cases'][c]['substep_endpoints'][str(at)],report['cases']['native_substeps']['substep_endpoints'][str(at)]
            report['paired_substep_outcomes'][c][str(at)]={'world_z_rate_delta_rad_s':a['platform_world_omega_rad_s'][2]-b['platform_world_omega_rad_s'][2],'heading_delta_deg':np.degrees(a['heading_unwrapped_rad']-b['heading_unwrapped_rad']),'H_by_body_z_delta_kg_m2_s':np.asarray(a['H_by_body_world_kg_m2_s'])[:,2]-np.asarray(b['H_by_body_world_kg_m2_s'])[:,2],'H_total_z_delta_kg_m2_s':a['H_total_world_kg_m2_s'][2]-b['H_total_world_kg_m2_s'][2],'actual_held_input_delta':np.asarray(a['actual_held_input'])-b['actual_held_input']}
    report['causal_interpretation_allowed']=report['gates']['technical_causal_prerequisites_pass']
    report['clean_unloaded_A_effect_interpretation_allowed']=report['causal_interpretation_allowed']and all(report['cases'][c]['unloaded_shellA_coverage']['clean_unloaded_comparison_force_coverage']for c in ['native_substeps','zero_free_shellA'])
    report['force_source_proof_sha256']=sha(proof)
    report['limits']=['Any branch effects include the natural feedback/contact response after treatment; direct hook scope is separate from later mediated changes.','Known internal motor torques cancel in total angularmomentum balance. Per-body remainder includes the explicitly approximated moving-COM transport correction and is not uniquely a motor, contact or passive cause.','Versioned DART/gz-physics source supports force=solver impulse/dt, conditional on actual force sign/order/pass-through/completeness; the raw field remains forceN. Closure failure means incomplete force accounting, not no motor reaction.','The interventions can worsen balance or tracking and are not deployable controller recommendations.','Pre-actuator-output request in hold_attitude already includes the held attitude target.']
    save(output/'report.json',report)
    with(output/'paired_outcomes.csv').open('w',newline='')as stream:
        writer=csv.DictWriter(stream,fieldnames=['case','time_s','world_z_rate_delta_rad_s','heading_delta_deg','absolute_tilt_delta_deg','fixed_untreated_target_error_delta_deg','actual_up_separation_deg']);writer.writeheader()
        for c,rows in report['paired_outcomes'].items():
            for at,d in rows.items():writer.writerow({'case':c,'time_s':at,**{k:d[k]for k in writer.fieldnames[2:]}})
    for rel,h in hashes.items():assert sha(ROOT/rel)==h,rel
    save(output/'artifact_manifest.json',{'files':[{'path':str(f.relative_to(output)),'bytes':f.stat().st_size,'sha256':sha(f)}for f in sorted(output.rglob('*'))if f.is_file()and f.name!='artifact_manifest.json']})
    print(json.dumps({'report':str(output/'report.json'),'technical_causal_prerequisites_pass':report['gates']['technical_causal_prerequisites_pass']}))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    try:main(args.output.resolve())
    except BaseException:
        if args.output.exists():save(args.output/'analysis_failure.json',{'traceback':traceback.format_exc()})
        raise
