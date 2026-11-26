"""Compact I54 saved-array follow-up and figures; no new experiment."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib, json, shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent;ROOT=H.parents[2]
OLD=ROOT/'results/contact_scheduling_joint_tracking_20261126'
CASES=tuple('m100_'+x for x in ('B','R','N','NR'))
COLORS={'B':'#bd7d2c','R':'#219dba','N':'#8860af','NR':'#38a076'}
CHOSEN=[32.245,32.250,32.275,32.290,32.305,44.]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [safe(v)for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    return x
def save(p,x):p.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p)as z:return {k:z[k].copy()for k in z.files}
def main():
    out=H/'compact_stage02';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/Path(__file__).name)
    mapping=OLD/'campaign/resolved_case_paths.json';mp=json.loads(mapping.read_text())['cases']
    inputs=[H/'stage01/report.json',mapping]+[H/'stage01'/(c+'_signals.npz')for c in CASES]+[ROOT/mp['m100_R']/'precision.npz']
    hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
    save(out/'registered_before_consolidation.json',{'utc':datetime.now(timezone.utc).isoformat(),'source_sha256':sha(__file__),'input_sha256':hashes,'scope':'Compact report/plots derived from sealed primary stage. Fixed chosen rows requested for timing clarity; not another experiment or new causal test.','chosen_R_times_s':CHOSEN,'figures':'All four28–44s platformheading/world-z rates; R materialrates and appliedtorques. Detail31.6–32.5s. No threshold/selection changes.'})
    primary=json.loads((H/'stage01/report.json').read_text());r=primary['cases']['m100_R'];summary={'scope':'Saved I52 four-arm m100 spin audit; no new simulator, model, controller or runtime evaluation.','video_mapping':'Overview playback10s = recorded32s; playback9–11s = recorded28–36s.','registered_windows_s':primary['registered_windows_s'],'cases':{},'R_chosen_rows':[],'R_world_z_sustained_episodes_after28':r['windows']['after28']['world_z_sustained_episodes'],'limits':primary['limits']}
    for c,d in primary['cases'].items():
        windows={}
        for key in ('full_trace','after28','after_gain_switch','after_speed_peak'):
            row=d['windows'][key];windows[key]={k:v for k,v in row.items()if k!='world_z_sustained_episodes'}
        summary['cases'][c]={'windows':windows,'heading_command_max_abs':d['heading_command_max_abs'],'angular_velocity_reference_body_max_abs_rad_s':d['angular_velocity_reference_body_max_abs_rad_s'],'feedback_quaternion_yaw_tangent_error_max_abs_rad':d['feedback_quaternion_yaw_tangent_error_max_abs_rad']}
    for at in CHOSEN:
        row=min(r['snapshots'],key=lambda x:abs(x['time_s']-at));assert abs(row['time_s']-at)<1e-7
        summary['R_chosen_rows'].append(row)
    row=lambda t:min(r['snapshots'],key=lambda x:abs(x['time_s']-t))
    a,b,c=row(32.245),row(32.25),row(32.305)
    summary['R_interval_comparison']={'preceding_hold_32_245_to32_250':{'applied':a['current_control']['applied'],'world_z_rate_change_rad_s':b['platform_world_omega_rad_s'][2]-a['platform_world_omega_rad_s'][2],'planar_velocity_increment_m_s':b['planar_velocity_increment_m_s'],'heading_change_deg':np.degrees(b['heading_unwrapped_rad']-a['heading_unwrapped_rad']),'meaning':'Measured increment ending32.250 precedes newlysampled shellgaincommand.'},'subsequent_32_250_to32_305':{'duration_s':c['time_s']-b['time_s'],'world_z_rate_change_rad_s':c['platform_world_omega_rad_s'][2]-b['platform_world_omega_rad_s'][2],'heading_change_deg':np.degrees(c['heading_unwrapped_rad']-b['heading_unwrapped_rad']),'planar_speed_change_m_s':c['planar_speed_m_s']-b['planar_speed_m_s'],'shellB_relative_rate_change_rad_s':c['relative_joint_rates_rad_s'][2]-b['relative_joint_rates_rad_s'][2],'meaning':'Multiple heldcommands/contactintervals; association, not isolated causal torque response.'}}
    summary['interpretation']={'what_spins':'The platform heading itself turns+2.748353 revolutions after32.250; shell/shaft encoder spin is separately measured. All headingprojections remain wellconditioned in this interval.','onset':'R exceeds sustained1rad/s already29.555s and31.095s; the episode associated withthe mainburst starts32.095s, before32.250gainchange.','persistence':'R remainspositiveheadingrotation through44s, including42s when desiredXYvelocityiszero.','control_scope':'All four arms havezero headingcommand andzero bodyangularvelocityreference. The referencecarriesactualyaw, so accumulatedheadingangleisnotheld. Existing LQR angularratefeedback remains; commonfreeheadingalonecannotexplaincrossarmdifferences.','causality':'Incomingcontacttransitionalreadyhasameasuredvelocitychangebeforethenewgaincommand; subsequentspinboostoccursundersaturatedtorques. Endpointlogs do not uniquelysplitcontactimpulses,motorreaction,andcoupledcontrollerhistory.'}
    save(out/'summary.json',summary)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42,'savefig.dpi':165})
    sig={c:load(H/'stage01'/(c+'_signals.npz'))for c in CASES};raw=load(ROOT/mp['m100_R']/'precision.npz');t=raw['time_s'];s=sig['m100_R']
    fig,ax=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
    for case,values in sig.items():
        tt=values['time_s'];m=(tt>=28-1e-9)&(tt<=44+1e-9);i=np.argmin(abs(tt-32.25));key=case.split('_')[1];color=COLORS[key]
        ax[0].plot(tt[m],(values['heading_unwrapped_rad'][m]-values['heading_unwrapped_rad'][i])/(2*np.pi),label=key,color=color,lw=1.2)
        ax[1].plot(tt[m],values['platform_omega_world_rad_s'][m,2],color=color,lw=1.0)
    m=(t>=28-1e-9)&(t<=44+1e-9)
    for j,name,color in [(6,'Shaft','#777777'),(7,'Shell A','#bd7d2c'),(8,'Shell B','#8860af')]:ax[2].plot(t[m],raw['exact_qvel'][m,j],label=name,color=color,lw=.9)
    for j,name,color in [(0,'Shaft','#777777'),(1,'Shell A','#bd7d2c'),(2,'Shell B','#8860af')]:ax[3].plot(t[m],raw['exact_applied'][m,j],label=name,color=color,lw=.8)
    for a in ax:a.grid(alpha=.2);a.axvline(32.25,color='#333333',ls='--',lw=.8);a.set_xlim(28,44)
    ax[0].set_ylabel('Platform heading change\nfrom 32.250 s (turns)');ax[0].legend(ncol=4,loc='upper left');ax[1].set_ylabel('Platform world-z\nangular velocity (rad/s)');ax[1].axhline(0,color='k',lw=.5);ax[2].set_ylabel('R relative joint\nrates (rad/s)');ax[2].legend(ncol=3,loc='upper right');ax[3].set_ylabel('R applied motor\ntorques (N m)');ax[3].set_xlabel('Recorded simulation time (s)');ax[3].legend(ncol=3,loc='upper right');fig.suptitle('I54: the platform itself keeps spinning after the burst\nAll four −1 rad starts; heading is valid throughout the shown interval')
    for ext in ('png','pdf'):fig.savefig(out/('platform_and_material_spin.'+ext),bbox_inches='tight')
    plt.close(fig)
    fig,ax=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained');m=(t>=31.6-1e-9)&(t<=32.5+1e-9)
    ax[0].plot(t[m],s['platform_omega_world_rad_s'][m,2],label='World-z angular velocity',color=COLORS['R']);ax[0].plot(t[m],s['heading_derivative_rad_s'][m],label='Geometric heading derivative',color='#333333',ls='--');ax[0].set_ylabel('Platform rate (rad/s)');ax[0].legend(fontsize=9)
    ax[1].plot(t[m],s['planar_speed_m_s'][m],color=COLORS['R'],label='Actual planar speed');ax[1].plot(t[m],np.linalg.norm(raw['exact_reference_velocity_m_s'][m],axis=1),color='#333333',ls='--',label='Desired planar speed');ax[1].set_ylabel('Speed (m/s)');ax[1].legend(fontsize=9)
    ax[2].plot(t[m],raw['exact_native_requested'][m,2],color='#bd7d2c',label='Requested shell-B torque');ax[2].plot(t[m],raw['exact_applied'][m,2],color=COLORS['R'],label='Applied shell-B torque');ax[2].set_ylabel('Shell B (N m)');ax[2].legend(fontsize=9)
    ax[3].plot(t[m],raw['exact_applied'][m,4],color='#bd7d2c',label='Applied belt-B speed');ax[3].set_ylabel('Belt B (m/s)');ax[3].legend(fontsize=9);ax[3].set_xlabel('Recorded simulation time (s)')
    for a in ax:a.grid(alpha=.2);a.axvline(32.25,color='#333333',ls='--',lw=.8);a.set_xlim(31.6,32.5)
    ax[0].annotate('Already 3.739 rad/s\nat 32.245 s',xy=(32.245,3.738786),xytext=(31.97,5.8),arrowprops={'arrowstyle':'->'},fontsize=9)
    fig.suptitle('R spin predates the 32.250 s gain switch, then increases\nCommands at each endpoint act during the next 5 ms hold')
    for ext in ('png','pdf'):fig.savefig(out/('R_spin_onset_detail.'+ext),bbox_inches='tight')
    plt.close(fig)
    for rel,h in hashes.items():assert sha(ROOT/rel)==h
    save(out/'artifact_manifest.json',{'files':[dict(path=str(p.relative_to(out)),bytes=p.stat().st_size,sha256=sha(p))for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
    print('Compact spin summary and figures complete; summary bytes', (out/'summary.json').stat().st_size)
if __name__=='__main__':main()
