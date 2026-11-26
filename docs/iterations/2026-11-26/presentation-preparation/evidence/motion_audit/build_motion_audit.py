"""Saved-evidence synthesis for presentation; NumPy algebra and plots, no simulator."""
from pathlib import Path
import csv
import hashlib
import json
import shutil

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
INPUTS = {
    'grid_summary':'results/configuration_study_20261125/map/summary.json',
    'grid_provenance':'results/configuration_study_20261125/map/provenance.json',
    'grid_states':'results/configuration_study_20261125/map/configurations.csv',
    'same_attitude':'results/configuration_study_20261125/map/same_tilt_example.json',
    'early_event':'results/contact_diagnostics_20261125/allocation_event_evidence.json',
    'early_rollout':'results/contact_diagnostics_20261125/baseline/plain_figure8/phase_+0.90/rollout.npz',
    'turn_report':'results/figure8_event_audit_20261125/events/full_audit/report.json',
    'turn_definition':'results/figure8_event_audit_20261125/events/selection_stage02_corrected/selection_source/event_metrics.py',
    'mars_precision':'results/lqr_contact_forecast_20261125/campaign/runs/mars_figure8_p090/precision.npz',
    'flat_precision':'results/figure8_event_audit_20261125/terrain/flat_control/campaign/runs/flat_figure8/precision.npz',
    'I42_turns':'results/handoff_factorial_20261125/analysis/turn_addendum/report.json',
    'I42_primary':'results/handoff_factorial_20261125/analysis/stage01/report.json',
    'I43_report':'results/tilt_reference_20261125/analysis/stage01/report.json',
    'model_source':'src/spherical_wheel/model.py',
    'rolling_source':'src/spherical_wheel/experiments.py',
    'allocation_source':'src/spherical_wheel/rolling_allocation.py',
    'grid_source':'results/configuration_study_20261125/map/source_snapshot/scripts/map-rolling-configurations.py',
    'configuration_notes':'docs/iterations/2026-11-25/configuration-study/README.md',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(key):
    return json.loads((ROOT/INPUTS[key]).read_text())


def load(key):
    with np.load(ROOT/INPUTS[key]) as z:
        return {k:z[k].copy() for k in z.files}


def save(path, value):
    def safe(x):
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        if isinstance(x, dict): return {k:safe(v) for k,v in x.items()}
        if isinstance(x, (list,tuple)): return [safe(v) for v in x]
        return x
    path.write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def rotation_wxyz(q):
    w,x,y,z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def body_velocity_matrix(rotation, phase_rad, contact_offset):
    shaft = rotation@np.array([0.,1.,0.])
    hemisphere = rotation@np.array([np.cos(phase_rad),0.,-np.sin(phase_rad)])
    # The repository stores contact-speed J and solves J*d=-v.  B=-J maps
    # shaft/loaded-shell rates directly to no-slip horizontal body velocity.
    return -np.column_stack((np.cross(shaft,contact_offset)[:2],np.cross(hemisphere,contact_offset)[:2]))


def main():
    if (HERE/'motion_audit.json').exists(): raise FileExistsError('Preserve previous synthesis before a new stage')
    snapshot = HERE/'source_snapshot'; snapshot.mkdir(exist_ok=False)
    shutil.copy2(Path(__file__),snapshot/Path(__file__).name)
    before = {value:sha(ROOT/value) for value in INPUTS.values()}
    save(HERE/'inputs_before.json',before)
    summary, provenance, same = read('grid_summary'),read('grid_provenance'),read('same_attitude')
    params = provenance['parameters']
    grid_count = len(params['roll_deg'])*len(params['pitch_deg'])*len(params['shaft_phase_deg'])*len(params['normal_vectors'])
    assert grid_count == summary['configurations'] == 17739
    assert grid_count*len(params['travel_direction_deg']) == summary['directional_samples'] == 141912
    with (ROOT/INPUTS['grid_states']).open() as stream: grid=list(csv.DictReader(stream))
    assert len(grid)==17739
    selected=[r for r in grid if r['normal']=='flat' and float(r['roll_deg'])==-20. and float(r['pitch_deg'])==0. and float(r['shaft_phase_deg'])==-90.]
    assert len(selected)==1 and selected[0]['support']=='shell_B' and int(selected[0]['support_count'])==1
    examples=[]
    for case in same['cases']:
        assert case['roll_deg']==-20. and case['pitch_deg']==0. and case['support_count']==1
        roll=np.radians(case['roll_deg']);rotation=np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
        matrix=body_velocity_matrix(rotation,np.radians(case['shaft_phase_deg']),np.array([0.,0.,-.12]))
        singular=np.linalg.svd(matrix*5,compute_uv=False)
        np.testing.assert_allclose(singular,[case['sigma1_m_s'],case['sigma2_m_s']],atol=3e-15,rtol=0)
        directions=[]
        for direction in (0.,90.):
            target=.06*np.array([np.cos(np.radians(direction)),np.sin(np.radians(direction))])
            rates=5*np.linalg.lstsq(matrix*5,target,rcond=1e-7)[0]
            saved=next(x for x in case['directions'] if x['travel_direction_deg']==direction)
            np.testing.assert_allclose(rates,[saved['bounded_shaft_reference'],saved['bounded_shell_B_reference']],atol=3e-14,rtol=0)
            np.testing.assert_allclose(np.linalg.norm(matrix@rates-target),saved['speed_box_residual_m_s'],atol=2e-15,rtol=0)
            directions.append(dict(direction_deg=direction,desired_velocity_m_s=target,rates_rad_s=rates,
                                   achieved_velocity_m_s=matrix@rates,residual_m_s=float(np.linalg.norm(matrix@rates-target))))
        examples.append(dict(roll_deg=-20.,pitch_deg=0.,yaw_deg=0.,phase_deg=case['shaft_phase_deg'],normal=[0.,0.,1.],
            support='shell_B',contact_count=1,B_body_velocity_per_joint_rate_m=matrix,repository_J_contact_m=-matrix,
            physical_singular_values_m=np.linalg.svd(matrix,compute_uv=False),scaled_singular_values_m_s=singular,
            normalization_joint_rad_s=[5.,5.],directions=directions))
    bad, early=read('early_event'),load('early_rollout');i=bad['index'];q=early['qpos'][i].astype(float)
    point=early['estimated_contact_points_m'][i,1].astype(float)
    matrix=body_velocity_matrix(rotation_wxyz(q[3:7]),q[7],point-q[:3]);target=early['reference_velocity_m_s'][i].astype(float)
    singular=np.linalg.svd(matrix*5,compute_uv=False);rates=5*np.linalg.lstsq(matrix*5,target,rcond=1e-7)[0]
    ridge=5*np.linalg.lstsq(np.vstack((matrix*5,.01*np.eye(2))),np.r_[target,0.,0.],rcond=1e-7)[0]
    np.testing.assert_allclose(singular,bad['scaled_allocation_singular_values_m_s'],atol=1e-12,rtol=0)
    np.testing.assert_allclose(rates,np.asarray(bad['reconstructed_least_squares_drive_rates'])[[0,2]],atol=1e-7,rtol=0)
    np.testing.assert_allclose(ridge,np.asarray(bad['damped_drive_rates_lambda_0_01'])[[0,2]],atol=1e-10,rtol=0)
    np.testing.assert_allclose(np.linalg.norm(matrix@ridge-target),bad['damped_xy_residual_m_s'],atol=1e-12,rtol=0)
    early_example=dict(saved_time_s=bad['time_s'],source_float32=True,support='shell_B',estimated_loads_N=bad['estimated_loads_N'],
        B_body_velocity_per_joint_rate_m=matrix,scaled_singular_values_m_s=singular,condition_number=float(singular[0]/singular[1]),
        desired_velocity_m_s=target,unregularized_rates_rad_s=rates,unregularized_residual_m_s=float(np.linalg.norm(matrix@rates-target)),
        normalized_speed_penalty_lambda_m_s=.01,relaxed_rates_rad_s=ridge,relaxed_achieved_velocity_m_s=matrix@ridge,
        relaxed_residual_m_s=float(np.linalg.norm(matrix@ridge-target)),logged_requested_command=bad['requested_command'],
        logged_applied_command=bad['applied_command'],transition_delta_at_sample=bad['transition_delta'],
        limitation=bad['saved_precision'])
    turns=read('turn_report');i42turns=read('I42_turns');i42=read('I42_primary');i43=read('I43_report')
    mars,flat=load('mars_precision'),load('flat_precision')
    for name in ('time_s','exact_reference_xy','exact_reference_velocity_m_s'):
        np.testing.assert_array_equal(mars[name],flat[name])
    turn_rows=turns['turn_comparison'];assert len(turn_rows)==6
    assert all(not e['reference_component_reversal_is_full_stop'] for e in turns['mars']['turn_events'])
    baseline=np.array([row['mars_geometric_overshoot_mm'] for row in turn_rows])
    no_sync=np.array([row['arms']['sync_off']['geometric_component_overshoot_mm'] for row in i42turns['turn_comparison']])
    shaped=np.array([row['consistent_rate_mm'] for row in i43['main_screen']['turn_nonregression']])
    assert np.sum(no_sync>baseline+1e-9)==4 and np.sum(shaped>baseline+1e-9)==4
    audit=dict(scope=__doc__,new_simulations=0,new_model_forward_calls=0,
        grid=dict(configurations=grid_count,direction_cases=summary['directional_samples'],clear_patch_interiors=summary['patch_interiors_clear_of_plane'],
                  example_grid_row=selected[0],fixed_case_pair_note='Rank-loss phase−90° is a grid sample; well-conditioned phase−.6rad is the same-study saved supplementary example, not on the5° grid.'),
        equations=dict(contact_point_velocity='v_body_xy + P_xy(a_shaft × r)*shaft_rate + P_xy(a_shell × r)*shell_rate = 0 (frozen base angular rate)',
            direct_body_map='B = −J_contact = −P_xy[a_shaft × r, a_shell × r]',axes='a_shaft = R_fork e_y; a_shell = R_fork R_y(shaft_phase)e_x',
            normalized_map='d=S*z, S=diag(5,5)rad/s for these two joints; singular values of B*S have m/s units.',
            speed_cost='min_z ||B*S*z−v_des||²+λ²||z||²; this is a normalized speed-demand penalty, not a torque/power cost.'),
        examples=examples,early_saved_event=early_example,
        turns=dict(reference_time_position_velocity_exactly_equal=True,rows=turn_rows,first_mars=turns['mars']['turn_events'][0],first_flat=turns['flat']['turn_events'][0],
            onset_mars_s=turns['mars']['primary_deviation']['start_s'],onset_flat_s=turns['flat']['primary_deviation']['start_s'],
            metrics=dict(timed_error='||p(t)−p_ref(t)||',local_path_distance='Closest point on reference polyline within±2s, branch constrained; distinct from nominal-time Frenet components.',
                component_overshoot='Positive incoming-direction excess beyond reference component extremum in the declared event window.',
                reversal_lag='First sustained new-sign actual velocity zero crossing minus reference component zero; negative means earlier. Nonzero vector speed is allowed.')),
        interventions=dict(I42_sync_off=dict(window_s=[4.,10.],turns_after_restoration=True,overshoot_mm=no_sync,worse_turns=4,
            moving_timed_RMS_mm=[1000*i42['runs']['baseline']['full_summary_tracking']['rms_error_while_moving_m'],1000*i42['runs']['sync_off']['full_summary_tracking']['rms_error_while_moving_m']],
            first_turn_lag_s=[turn_rows[0]['mars_reversal_lag_s'],i42turns['first_turn']['arms']['sync_off']['actual_reversal_lag_s']]),
            I43_shaped_consistent_rate=dict(overshoot_mm=shaped,worse_turns=4,accepted=i43['main_screen']['passes_all_conditions'],screen=i43['main_screen'])),
        causal_boundaries=['Single loaded patch suffices for rank loss; two hemispheres touching are not required.',
            'Flat-ground overshoots show terrain relief is not necessary for overshoot; they do not identify a sole controller cause.',
            'FirstMars departure6.86s precedes all six reference component reversals; generic turn overshoot cannot explain its onset.',
            'Same-contact kinematic rank loss, discontinuous support selection, and feedback handover are separate mechanisms.',
            'Unregularized references are rates; actuator torque commands and clipping are separate.',
            'A cost cannot restore a missing instantaneous direction; it trades tracking error for finite references.',
            'Turning comparisons are one trace per arm, separate interventions with carried state/history, not independent trials or causal isolation.'],
        source_files=INPUTS)
    figures=HERE/'figures';figures.mkdir(exist_ok=False)
    plt.rcParams.update({'font.size':11,'axes.titlesize':12,'figure.titlesize':14})
    fig,axes=plt.subplots(1,2,figsize=(11.5,4.7),constrained_layout=True)
    for ax,example in zip(axes,examples):
        for column,color,label in [(0,'#337eaf','Shaft, +1rad/s'),(1,'#df9b3a','Loaded shell B, +1rad/s')]:
            v=example['B_body_velocity_per_joint_rate_m'][:,column]
            ax.arrow(0,0,v[0],v[1],width=.0015,head_width=.007,length_includes_head=True,color=color,label=label,zorder=3)
        ax.axhline(0,color='#999',lw=.6);ax.axvline(0,color='#999',lw=.6)
        ax.set(xlim=(-.015,.14),ylim=(-.12,.09),aspect='equal',xlabel='Body x velocity contribution (m/s)',ylabel='Body y velocity contribution (m/s)',
            title=f'Shaft phase {example["phase_deg"]:.3f}°\nScaled singular values: {example["scaled_singular_values_m_s"][0]:.3f}, {example["scaled_singular_values_m_s"][1]:.3f}m/s')
        ax.grid(alpha=.18);ax.legend(fontsize=8,loc='upper left')
    axes[1].annotate('x request feasible',xy=(.06,0),xytext=(.06,.045),arrowprops=dict(arrowstyle='->',color='#396b46'),fontsize=10,color='#396b46')
    axes[1].annotate('y request unavailable',xy=(0,.06),xytext=(.025,.075),arrowprops=dict(arrowstyle='->',color='#ad4943'),fontsize=10,color='#ad4943')
    fig.suptitle('One contact, same −20° platform roll: changing shaft phase can align drive directions\nFlat normal; shell B remains selected. Instantaneous geometry, not a balance test.')
    for ext in ('png','pdf'):fig.savefig(figures/f'drive_direction_example.{ext}',dpi=200)
    plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11.5,8),constrained_layout=True);x=np.arange(6);width=.36
    for shift,case,color in [(-width/2,'mars','#b57b40'),(width/2,'flat','#468db1')]:
        axes[0,0].bar(x+shift,[r[case+'_geometric_overshoot_mm'] for r in turn_rows],width=width,color=color,label=case.capitalize())
        axes[0,1].bar(x+shift,[r[case+'_reversal_lag_s'] for r in turn_rows],width=width,color=color,label=case.capitalize())
    labels=[r['event_id'].replace('turn_','') for r in turn_rows]
    axes[0,0].set(title='Geometric component overshoot',ylabel='Beyond planned extremum (mm)',xticks=x,xticklabels=labels);axes[0,0].legend()
    axes[0,1].set(title='Velocity-component reversal timing',ylabel='Lag (s); negative = early',xticks=x,xticklabels=labels);axes[0,1].axhline(0,color='black',lw=.8)
    first=turns['mars']['turn_events'][0];when=first['time_s'];extremum=first['reference_component_extremum_m']
    for data,color,label in [(mars,'#b57b40','Mars actual'),(flat,'#468db1','Flat actual')]:
        mask=(data['time_s']>=first['window_s'][0])&(data['time_s']<=first['window_s'][1]);tau=data['time_s'][mask]-when
        axes[1,0].plot(tau,1000*(data['exact_qpos'][mask,1]-extremum),color=color,label=label)
        axes[1,1].plot(tau,1000*data['exact_qvel'][mask,1],color=color,label=label)
    mask=(mars['time_s']>=first['window_s'][0])&(mars['time_s']<=first['window_s'][1]);tau=mars['time_s'][mask]-when
    axes[1,0].plot(tau,1000*(mars['exact_reference_xy'][mask,1]-extremum),'--',color='#333',label='Reference')
    axes[1,1].plot(tau,1000*mars['exact_reference_velocity_m_s'][mask,1],'--',color='#333',label='Reference')
    axes[1,0].set(title='First turn: lag without geometric overshoot on Mars',ylabel='y − planned maximum (mm)',xlabel='Time from first reference y reversal (s)')
    axes[1,1].set(title='First turn: different actual zero-crossing times',ylabel='y velocity (mm/s)',xlabel='Time from first reference y reversal (s)')
    for ax in axes[1]:ax.axvline(0,color='gray',lw=.8);ax.axhline(0,color='gray',lw=.8);ax.legend(fontsize=9)
    for ax in axes.flat:ax.grid(axis='y',alpha=.2)
    fig.suptitle('Six saved figure-eight component reversals: overshoot, timing and path error are different\nIdentical timed reference; nonzero forward speed at all events; flat and Mars initialization differ.')
    for ext in ('png','pdf'):fig.savefig(figures/f'turn_geometry_and_timing.{ext}',dpi=190)
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(11.5,4.8),constrained_layout=True)
    for shift,values,color,label in [(-.25,baseline,'#667b8d','Native baseline'),(0.,no_sync,'#4a94a4','I42 sync off only4–10s'),(.25,shaped,'#d99a4c','I43 shaped tilt + consistent rate')]:
        ax.bar(x+shift,values,width=.25,color=color,label=label)
    ax.set(xticks=x,xticklabels=labels,ylabel='Geometric component overshoot (mm)',xlabel='Fixed six reference events',
           title='Local improvements did not produce uniform turn improvement\nBoth candidates worsen4 of6 overshoots; different experiments, not one combined factorial.')
    ax.grid(axis='y',alpha=.2);ax.legend(fontsize=9)
    for ext in ('png','pdf'):fig.savefig(figures/f'intervention_turn_tradeoffs.{ext}',dpi=190)
    plt.close(fig)
    after={value:sha(ROOT/value) for value in INPUTS.values()};assert before==after
    audit['integrity']=dict(inputs_unchanged=True,numeric_assertions_passed=True,source_sha256=sha(Path(__file__)),input_sha256=before)
    save(HERE/'motion_audit.json',audit)
    save(HERE/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(HERE)),sha256=sha(p),bytes=p.stat().st_size) for p in sorted(HERE.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(passed=True,examples=len(examples),early_condition_number=early_example['condition_number'],turns=len(turn_rows),simulations=0),indent=2))


if __name__=='__main__':main()
