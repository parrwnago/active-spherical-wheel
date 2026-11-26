"""I55 fixed-window descriptive plots and compact report consolidation."""
from pathlib import Path
from datetime import datetime,timezone
import json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import analyze_causal as ac
import budget_math as bm
H=Path(__file__).resolve().parent;FAMILY=H.parent;ROOT=FAMILY.parents[1]
def main():
    out=H/'presentation_stage01';out.mkdir(exist_ok=False)
    paths=[Path(__file__),H/'stage01/report.json',H/'analyze_causal.py',H/'budget_math.py']
    for c in ['native_substeps','zero_free_shellA','hold_attitude']:
        f=FAMILY/'campaign'/c;paths.extend([f/'campaign/runs/m100_R/precision.npz',f/'i55_sidecar.npz',f/'substeps.npz',H/'stage01'/(c+'_momentum.npz')])
    hashes={str(p.relative_to(ROOT)):ac.sha(p)for p in paths};ac.copy_source(out,[Path(__file__)])
    ac.save(out/'before_consolidation.json',{'utc':datetime.now(timezone.utc).isoformat(),'input_sha256':hashes,'scope':'No new scientific thresholds, endpoints, cases or acceptance decisions. Fixed registered trace windows only; all intervention and negative outcomes retained.','source_disclosure':'All canonical outcomes already known before figure generation.'})
    report=json.loads((H/'stage01/report.json').read_text());summary={'scope':'Causal diagnostics, no controller candidate or adoption claim','technical_causal_prerequisites_pass':report['gates']['technical_causal_prerequisites_pass'],'clean_unloaded_A_effect_interpretation_allowed':report['clean_unloaded_A_effect_interpretation_allowed'],'baseline_gates':{k:report['gates'][k]for k in ['native_batch_historical','native_substeps_batch_equivalence','zero_A_first_hold_other_inputs_exact']},'zero_A':{'first_1ms_and5ms':report['paired_substep_outcomes']['zero_free_shellA'],'later_endpoints':report['paired_outcomes']['zero_free_shellA']},'hold_attitude':{'primary_time_s':31.745,'baseline':report['cases']['native_substeps']['endpoints']['31.745'],'treated':report['cases']['hold_attitude']['endpoints']['31.745'],'paired_endpoints':report['paired_outcomes']['hold_attitude']},'contact_closure':{},'limits':report['limits']}
    for c,d in report['cases'].items():
        if'momentum_intervals'in d:summary['contact_closure'][c]=[{'start_s':r['start_s'],'end_s':r['end_s'],**r['closure_by_center']['midpoint']}for r in d['momentum_intervals']]
    native0=report['cases']['native_substeps']['endpoints']['32.25']['platform_world_omega_rad_s'][2]
    native5=report['cases']['native_substeps']['substep_endpoints']['32.255']['platform_world_omega_rad_s'][2]
    zero5=report['cases']['zero_free_shellA']['substep_endpoints']['32.255']['platform_world_omega_rad_s'][2]
    summary['zero_A']['first_5ms_native_yaw_rate_rise_rad_s']=native5-native0;summary['zero_A']['first_5ms_zeroA_yaw_rate_rise_rad_s']=zero5-native0;summary['zero_A']['first_5ms_rise_reduction_fraction']=1-(zero5-native0)/(native5-native0)
    ac.save(out/'summary.json',summary)
    raw={};side={};z={};moment={}
    for c in ['native_substeps','zero_free_shellA','hold_attitude']:
        f=FAMILY/'campaign'/c;raw[c]=ac.load(f/'campaign/runs/m100_R/precision.npz');side[c]=ac.load(f/'i55_sidecar.npz');z[c]=ac.load(f/'substeps.npz');moment[c]=ac.load(H/'stage01'/(c+'_momentum.npz'))
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42,'savefig.dpi':170})
    fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True,layout='constrained');colors={'native_substeps':'#bd7d2c','zero_free_shellA':'#219dba'}
    for c,label in [('native_substeps','Native'),('zero_free_shellA','Only shell A torque zeroed')]:
        t=z[c]['time_s'];m=(t>=32.245-1e-9)&(t<=32.32+1e-9);axes[0].plot(t[m],z[c]['bridge_state'][m,14],color=colors[c],label=label)
        axes[1].step(t[m]-.001,z[c]['held_input'][m,1],where='post',color=colors[c],label=label+' — shell A')
        axes[1].step(t[m]-.001,z[c]['held_input'][m,2],where='post',color=colors[c],ls='--',lw=.8,label=label+' — shell B')
    t=moment['native_substeps']['time_s'];m=(t>=32.25-1e-9)&(t<=32.32+1e-9);delta=moment['zero_free_shellA']['H_after_by_body_world']-moment['native_substeps']['H_after_by_body_world']
    axes[2].plot(t[m],delta[m,:2,2].sum(axis=1)*1000,label='Fork + carrier',color='#444444');axes[2].plot(t[m],delta[m,2,2]*1000,label='Shell A',color='#219dba');axes[2].plot(t[m],delta[m,3,2]*1000,label='Shell B',color='#8860af');axes[2].plot(t[m],delta[m,:,2].sum(axis=1)*1000,label='Total',color='#b63232',ls='--')
    axes[0].set_ylabel('Platform world-z rate\n(rad/s)');axes[0].legend(fontsize=9);axes[1].set_ylabel('Held shell torques\n(N m)');axes[1].legend(fontsize=8,ncol=2);axes[2].set_ylabel('Treated − native vertical H\n(10⁻³ kg m²/s)');axes[2].set_xlabel('Recorded simulation time (s)');axes[2].legend(ncol=4,fontsize=8)
    for a in axes:a.grid(alpha=.2);a.axvline(32.25,color='#333333',ls=':',lw=.8);a.axvline(32.255,color='#888888',ls=':',lw=.8);a.axvline(32.275,color='#333333',ls='--',lw=.8);a.set_xlim(32.245,32.32)
    fig.suptitle('I55: removing unloaded shell A torque reduces the initial yaw surge\nOther held inputs match for the first 5 ms; later feedback is allowed to respond')
    for ext in ['png','pdf']:fig.savefig(out/('shell_A_causal_response.'+ext),bbox_inches='tight')
    plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True,layout='constrained');native_target=ac.up(side['native_substeps']['native_qref_quat'])
    for c,label,color in [('native_substeps','Native','#bd7d2c'),('hold_attitude','Attitude held','#219dba')]:
        p=raw[c];t=p['time_s'];m=(t>=31.4-1e-9)&(t<=31.85+1e-9);actual=ac.up(p['exact_qpos'][:,3:7]);world=np.einsum('nij,nj->ni',bm.rotation(p['exact_qpos'][:,3:7]),p['exact_qvel'][:,3:6]);axes[0].plot(t[m],ac.degrees_between(actual[m],np.array([0.,0.,1.])),label=label,color=color);axes[1].plot(t[m],ac.degrees_between(actual[m],native_target[m]),color=color);axes[2].plot(t[m],world[m,2],color=color)
    axes[0].set_ylabel('Absolute platform tilt\nfrom world vertical (°)');axes[0].legend();axes[1].set_ylabel('Error to SAME untreated\ndesired up direction (°)');axes[2].set_ylabel('Platform world-z rate\n(rad/s)');axes[2].set_xlabel('Recorded simulation time (s)')
    for a in axes:a.grid(alpha=.2);a.axvline(31.58,color='#333333',ls=':',lw=.8);a.axvline(31.745,color='#333333',ls='--',lw=.8);a.set_xlim(31.4,31.85)
    fig.suptitle('I55: holding the old attitude changes the entering state, with a tradeoff\nAt release: less yaw, greater absolute tilt and greater untreated-reference error')
    for ext in ['png','pdf']:fig.savefig(out/('attitude_causal_response.'+ext),bbox_inches='tight')
    plt.close(fig)
    for rel,h in hashes.items():assert ac.sha(ROOT/rel)==h,rel
    ac.save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':ac.sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
    print('Causal diagnostic summary and fixed-window plots sealed',out)
if __name__=='__main__':main()
