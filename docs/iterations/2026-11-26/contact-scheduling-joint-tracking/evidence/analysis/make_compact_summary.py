"""Present existing sealed I52 metrics only; no raw-data reduction or physics."""
from pathlib import Path
import hashlib
import json
import shutil

import numpy as np

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
PHASES=('m100','p030','p090')
VARIANTS=('B','R','N','NR')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):p.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')

def main():
    stage=HERE/'stage01'
    if not(stage/'artifact_manifest.json').is_file():raise FileNotFoundError('Wait for canonical report seal')
    out=HERE/'compact_stage01';out.mkdir(exist_ok=False)
    source=Path(__file__).resolve();shutil.copy2(source,out/'source.py')
    paths=[source,stage/'report.json',stage/'artifact_manifest.json',FAMILY/'campaign/resolved_case_paths.json']
    before={str(p.relative_to(ROOT)):sha(p)for p in paths};save(out/'before_presentation.json',{'input_sha256':before,'scope':__doc__})
    report=json.loads((stage/'report.json').read_text());cases={}
    for key,c in report['cases'].items():
        if not c['available']:cases[key]=c;continue
        j=c['joint_tracking'];full=j['windows']['full'];mechanical=c['mechanistic'];phase=mechanical['phase']
        cases[key]={'phase_rad':c['phase_rad'],'variant':c['variant'],'completed':c['completed'],'failure':c['failure'],'duration_s':c['completed_duration_s'],'fixed_horizon_coverage':c['fixed_horizon_coverage'],
            'tracking':c['timed_tracking'],'attitude':c['attitude'],'settled':c['original_settling'],'route_gates':c['original_route_gates'],
            'departure':c['departure'],'turns':c['turns'],'commands':c['commands'],'belts':c['belts'],
            'joint_tracking':{'units':j['units'],'interpretation':j['interpretation'],'actual':full['actual'],'native_reference':full['native_reference'],'used_reference':full['guarded_reference'],
                'actual_minus_native_ref':full['actual_minus_native_ref'],'actual_minus_used_ref':full['actual_minus_guarded_ref'],
                'modified_held_duration_s':j['modified_held_duration_s'],'modified_held_samples':j['modified_held_samples'],
                'max_native_reference_rad_s':j['max_abs_native_reference_rad_s'],'max_used_reference_rad_s':j['max_abs_guarded_reference_rad_s'],
                'max_actual_rate_rad_s':j['max_abs_actual_joint_rate_rad_s'],'max_reference_acceleration_all_emissions_rad_s2':j['max_abs_guarded_reference_acceleration_rad_s2']},
            'phase':phase,'introduced_rolling_reference_residual':mechanical.get('introduced_rolling_reference_residual'),
            'support_comparison':c['support_modes']['comparison'],'support_definitions':c['support_modes']['definitions'],
            'support_family_duration_s':{s:c['support_modes'][s]['family_duration_s']for s in ('estimated','truth')},'slip_contact':c['slip_contact']}
    compact={'iteration':'I-52','source_report_sha256':sha(stage/'report.json'),'execution_resolution':report['execution_resolution'],
        'integrity_passed':report['integrity']['passed'],'integrity_checks':report['integrity']['checks'],
        'historical_parity_summary':{k:{'required':v['required'],'available':v['available'],'passed':v['passed']}for k,v in report['integrity']['historical_parity'].items()},
        'cases':cases,'per_phase':report['per_phase'],'policy_screens':report['policy_screens'],'conventions':report['conventions'],'limitations':report['limitations'],
        'scope':__doc__}
    save(out/'summary.json',compact)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(14,8),sharey='row',constrained_layout=True)
    x=np.arange(4)
    for col,phase in enumerate(PHASES):
        rows=[cases[phase+'_'+v]for v in VARIANTS]
        def number(row,key):
            if 'joint_tracking'not in row:return np.nan
            return row['joint_tracking'][key]['rms_norm']
        native=[number(r,'actual_minus_native_ref')for r in rows];used=[number(r,'actual_minus_used_ref')for r in rows]
        axes[0,col].bar(x-.17,native,.34,label='Actual−native target',color='#d99035');axes[0,col].bar(x+.17,used,.34,label='Actual−used target',color='#269fc5')
        axes[0,col].set_yscale('log');axes[0,col].set_title(f'{phase}: phase {report["per_phase"][phase]["phase_rad"]:+.2f}rad')
        actual=[r.get('joint_tracking',{}).get('max_actual_rate_rad_s',np.nan)for r in rows];target=[r.get('joint_tracking',{}).get('max_used_reference_rad_s',np.nan)for r in rows]
        axes[1,col].bar(x-.17,actual,.34,label='Max actual joint rate',color='#d99035');axes[1,col].bar(x+.17,target,.34,label='Max used joint-rate target',color='#269fc5')
        for ax in axes[:,col]:ax.set_xticks(x,VARIANTS);ax.grid(axis='y',alpha=.2);ax.legend(fontsize=8)
    axes[0,0].set_ylabel('RMS joint-error vector norm(rad/s), log scale');axes[1,0].set_ylabel('Max across three joints(rad/s)')
    fig.suptitle('I52: actual joint motion and native/used reference targets are distinct\nEach policy has its own closed-loop history; reference bounds are not physical motor limits',fontsize=12)
    for ext in ('png','pdf'):fig.savefig(out/f'joint_tracking_all_phases.{ext}',dpi=170)
    plt.close(fig)
    assert before=={str(p.relative_to(ROOT)):sha(p)for p in paths}
    save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'sha256':sha(p),'bytes':p.stat().st_size}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
    print(json.dumps({'summary_sha256':sha(out/'summary.json'),'source_report_sha256':sha(stage/'report.json'),'scope':__doc__},indent=2))

if __name__=='__main__':main()
