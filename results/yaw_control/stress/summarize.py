"""Summarize the matched 81-case, 10-second yaw regression comparison."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
BASELINE_COMMIT='3bfd7998ed72146521894f445c2907129ed9802c'

def load(name):
    rows=json.loads((ROOT/name/'summaries.json').read_text())
    if len(rows)!=81:
        raise ValueError(f'{name}: expected81 cases, got{len(rows)}')
    mapping={(row['scenario'],row['case']):row for row in rows}
    if len(mapping)!=81:
        raise ValueError(f'{name}: duplicate case identifiers')
    return rows,mapping

def stats(values):
    a=np.asarray(values,dtype=float)
    return {'median':float(np.median(a)), 'p90':float(np.percentile(a,90)), 'max':float(np.max(a))}

def main():
    old,oldmap=load('baseline_3bfd799');new,newmap=load('phasefree')
    if oldmap.keys()!=newmap.keys():raise ValueError('Case grids differ')
    pairs=[]
    for key in sorted(oldmap):
        before,after=oldmap[key],newmap[key]
        pairs.append({'scenario':key[0],'case':key[1],
          'baseline_failed':before['failed'],'phasefree_failed':after['failed'],
          'baseline_final_yaw_rad_s':before['final_world_yaw_rate_rad_s'],
          'phasefree_final_yaw_rad_s':after['final_world_yaw_rate_rad_s'],
          'baseline_peak_yaw_rad_s':before['max_abs_world_yaw_rate_rad_s'],
          'phasefree_peak_yaw_rad_s':after['max_abs_world_yaw_rate_rad_s'],
          'baseline_yaw_settling_after_last_push_s':before['yaw_settling_after_last_push_s'],
          'phasefree_yaw_settling_after_last_push_s':after['yaw_settling_after_last_push_s'],
          'baseline_final_xy_error_m':before['final_position_error_m'],
          'phasefree_final_xy_error_m':after['final_position_error_m'],
          'baseline_max_tilt_deg':before['max_tilt_deg'],'phasefree_max_tilt_deg':after['max_tilt_deg'],
          'baseline_final_tilt_deg':before['final_tilt_deg'],'phasefree_final_tilt_deg':after['final_tilt_deg']})
    common=[pair for pair in pairs if not pair['baseline_failed'] and not pair['phasefree_failed']]
    aggregate={}
    for name,rows in [('baseline_3bfd799',old),('phasefree',new)]:
        survivors=[row for row in rows if not row['failed']]
        common_rows=[row for row in rows if (row['scenario'],row['case']) in {(p['scenario'],p['case']) for p in common}]
        aggregate[name]={'cases':len(rows),'failed':sum(row['failed'] for row in rows),
          'survived':len(survivors),'survivor_final_abs_yaw_rad_s':stats([abs(row['final_world_yaw_rate_rad_s']) for row in survivors]),
          'common_survivor_final_abs_yaw_rad_s':stats([abs(row['final_world_yaw_rate_rad_s']) for row in common_rows]),
          'common_survivor_peak_abs_yaw_rad_s':stats([row['max_abs_world_yaw_rate_rad_s'] for row in common_rows]),
          'common_survivor_final_xy_error_m':stats([row['final_position_error_m'] for row in common_rows]),
          'common_survivor_yaw_above_0_1_rad_s_at_end':sum(abs(row['final_world_yaw_rate_rad_s'])>.1 for row in common_rows),
          'common_survivor_yaw_above_1_rad_s_at_end':sum(abs(row['final_world_yaw_rate_rad_s'])>1. for row in common_rows),
          'common_survivor_yaw_settled_before_end':sum(row['yaw_settling_time_s'] is not None for row in common_rows),
          'maximum_uncommanded_generalized_force':max(row['max_uncommanded_generalized_force'] for row in rows),
          'maximum_unexpected_world_force':max(row['max_unexpected_world_force'] for row in rows),
          'source_hashes':{key:sorted({row['source_hashes_at_worker_start'][key] for row in rows}) for key in rows[0]['source_hashes_at_worker_start']},
          'failures':[{'scenario':row['scenario'],'case':row['case'],'time_s':row['fall_time_s']} for row in rows if row['failed']]}
    config_paths=[ROOT/mode/scenario/'model/model_config.json' for mode in ('baseline_3bfd799','phasefree') for scenario in ('shell','polar','transition')]
    configurations=[json.loads(path.read_text()) for path in config_paths]
    protocol_matches=True
    reference_matches=True
    initial_state_matches=True
    for key in sorted(oldmap):
        scenario,case=key
        with np.load(ROOT/'baseline_3bfd799'/scenario/case/'rollout.npz',allow_pickle=False) as before, np.load(ROOT/'phasefree'/scenario/case/'rollout.npz',allow_pickle=False) as after:
            count=min(len(before['time_s']),len(after['time_s']))
            protocol_matches &= bool(np.allclose(before['force_N'][:count],after['force_N'][:count],atol=1e-12,rtol=0))
            reference_matches &= bool(np.allclose(before['reference_xy'][:count],after['reference_xy'][:count],atol=1e-12,rtol=0))
            initial_state_matches &= bool(np.allclose(before['qpos'][0],after['qpos'][0],atol=1e-12,rtol=0) and np.allclose(before['qvel'][0],after['qvel'][0],atol=1e-12,rtol=0))
    summary={'baseline_commit':BASELINE_COMMIT,'cases_per_mode':81,'common_survivors':len(common),
      'requested_duration_s':10.,'thread_caps':{'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'},
      'change':'Zero shell material-angle position errors7:9 only; shaft phase and every velocity feedback term retained.',
      'same_configuration_all_models':all(config==configurations[0] for config in configurations),
      'same_applied_push_history_until_first_fall':protocol_matches,
      'same_world_xy_reference_until_first_fall':reference_matches,
      'same_initial_state_all_pairs':initial_state_matches,
      'metrics_note':'Yaw is angular velocity projected on world vertical, rad/s. Settling means |yaw|<0.1rad/s for every remaining sample after the last pulse, with at least0.5s observed. Null means not demonstrated within10s. Falling runs are excluded from common-survivor yaw/XY comparisons.',
      'aggregate':aggregate,
      'survival_regressions':[p for p in pairs if not p['baseline_failed'] and p['phasefree_failed']],
      'survival_improvements':[p for p in pairs if p['baseline_failed'] and not p['phasefree_failed']],
      'largest_xy_regressions':sorted(common,key=lambda p:p['phasefree_final_xy_error_m']-p['baseline_final_xy_error_m'],reverse=True)[:10],
      'paired_cases':pairs,
      'limitations':'These are10s sampled simulations of the provisional model. Long-tail evidence is separate. No global recovery, absolute-heading, zero-position-bias, or hardware guarantee is established.'}
    (ROOT/'comparison.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('paired_cases','largest_xy_regressions')},indent=2))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(11,4.7),constrained_layout=True)
        colors={'shell':'#168375','polar':'#ca8d32','transition':'#49678e'}
        for scenario,color in colors.items():
            rows=[p for p in common if p['scenario']==scenario]
            axes[0].scatter([max(abs(p['baseline_final_yaw_rad_s']),1e-6) for p in rows],
                            [max(abs(p['phasefree_final_yaw_rad_s']),1e-6) for p in rows],label=scenario,s=25,color=color,alpha=.75)
        axes[0].plot([1e-6,30],[1e-6,30],'--',color='#a7afb3',lw=1)
        axes[0].set(xscale='log',yscale='log',xlabel='Baseline final |world yaw rate| (rad/s)',ylabel='Free shell phase final |yaw rate| (rad/s)',title=f'Common survivors at10s (n={len(common)})')
        axes[0].legend(frameon=False,fontsize=8);axes[0].grid(alpha=.16)
        for scenario,color in colors.items():
            rows=[p for p in common if p['scenario']==scenario]
            axes[1].scatter([p['baseline_final_xy_error_m']*1000 for p in rows],
                            [p['phasefree_final_xy_error_m']*1000 for p in rows],s=25,color=color,alpha=.75)
        limit=max([p['baseline_final_xy_error_m']*1000 for p in common]+[p['phasefree_final_xy_error_m']*1000 for p in common])*1.05
        axes[1].plot([0,limit],[0,limit],'--',color='#a7afb3',lw=1)
        axes[1].set(xlabel='Baseline final XY error (mm)',ylabel='Free shell phase final XY error (mm)',title='Position error remains a separate metric');axes[1].grid(alpha=.16)
        fig.suptitle('Matched physical model and limits ·8 pushes per run · no absolute-heading lock\nYaw plot display floor:1e−6rad/s',fontsize=10)
        fig.savefig(ROOT/'comparison.png',dpi=160)
    except ImportError:
        pass

if __name__=='__main__':main()
