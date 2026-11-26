"""Saved-array presentation audit; no controller, observer, model or simulator run."""
from pathlib import Path
from datetime import datetime, timezone
import csv
import hashlib
import json

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TIMES = [6.66, 6.70, 6.725, 6.76, 6.825, 6.855, 6.86, 6.90, 7.10]
GROUPS = dict(position=(0,3), attitude_tangent=(3,6), joint_position=(6,9),
              linear_velocity=(9,12), angular_velocity=(12,15), joint_velocity=(15,18))


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def load(path):
    with np.load(path) as z:
        return {k:z[k].copy() for k in z.files}


def safe(value):
    if isinstance(value, dict):return {str(k):safe(v)for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [safe(v)for v in value]
    if isinstance(value,np.ndarray):return safe(value.tolist())
    if isinstance(value,np.generic):return value.item()
    return value


def main():
    declaration = json.loads((HERE/'inputs_before_execution.json').read_text())
    for row in declaration['inputs']:
        assert sha(ROOT/row['path']) == row['sha256']
    historical = load(ROOT/'results/support_routing_timing_20261126/campaign/historical_stage01/precision.npz')
    contexts, checks = [], []
    for arm in ('baseline','feedback_shares','sync_off','both'):
        folder = ROOT/'results/handoff_factorial_20261125/campaign/runs'/arm
        exact, log = load(folder/'precision.npz'), load(folder/'rollout.npz')
        for desired in TIMES:
            i = int(np.argmin(abs(exact['time_s']-desired)))
            assert abs(exact['time_s'][i]-desired)<1e-8
            K, error = exact['exact_feedback_gain'][i], exact['exact_feedback_error'][i]
            groups = {name:-(K[:,a:b]@error[a:b])for name,(a,b)in GROUPS.items()}
            total = -K@error
            cast_equal = np.array_equal(total.astype(log['lqr_command'].dtype),log['lqr_command'][i])
            assert cast_equal
            terms = {name:log[name][i].astype(float)for name in
                ('equilibrium_command','rolling_feedforward','support_command','transition_command_delta','heading_command','fade_command_delta')}
            reconstructed = total+sum(terms.values())
            bounds = log['input_bounds'][i].astype(float)
            residual = reconstructed-exact['exact_native_requested'][i]
            group_residual = sum(groups.values())-total
            row = dict(arm=arm,time_s=float(exact['time_s'][i]),index=i,
                input_order=['shaft','shell_A','shell_B','belt_A','belt_B'],
                input_units=['Nm','Nm','Nm','m/s','m/s'], exact_grouped_LQR=groups,
                exact_LQR=total, logged_float32_terms=terms,
                exact_requested=exact['exact_native_requested'][i], applied=exact['exact_applied'][i],
                reconstruction_residual_due_to_float32_components=residual,
                normalized_max_abs_reconstruction_residual=float(np.max(abs(residual)/bounds)),
                grouped_sum_max_abs_residual=float(np.max(abs(group_residual))),
                exact_LQR_cast_matches_saved_float32=cast_equal,
                native_feedback_loads_N=exact['native_feedback_loads_N'][i],
                truth_patch_loads_N=exact['truth_patch_loads_N'][i],
                exact_error=error,
                feedback_reference_angular_velocity=log['feedback_reference_qvel'][i,3:6])
            contexts.append(row)
            if arm=='baseline':
                np.testing.assert_array_equal(exact['exact_qpos'][i],historical['exact_qpos'][i])
                np.testing.assert_array_equal(exact['exact_native_requested'][i],historical['exact_native_requested'][i])
                np.testing.assert_array_equal(K,historical['exact_feedback_gain'][i])
                np.testing.assert_array_equal(error,historical['exact_feedback_error'][i])
    example = next(r for r in contexts if r['arm']=='baseline' and abs(r['time_s']-6.825)<1e-8)
    i=example['index']
    exact_terms = dict(LQR=historical['exact_feedback_gain'][i]@(-historical['exact_feedback_error'][i]),
        rolling_feedforward=historical['exact_rolling_feedforward'][i],
        holding=historical['exact_support_torque'][i],
        synchronization_delta=historical['exact_transition_command_delta'][i],
        heading_delta=historical['exact_heading_command'][i],fade_delta=historical['exact_fade_command_delta'][i])
    equilibrium = example['logged_float32_terms']['equilibrium_command']
    assert np.array_equal(equilibrium[3:],np.zeros(2))
    exact_sum = equilibrium+sum(exact_terms.values())
    np.testing.assert_allclose(exact_sum,historical['exact_native_requested'][i],atol=1e-12,rtol=0)
    belt = 3
    example_verified = dict(time_s=example['time_s'], LQR_groups_m_s={k:float(v[belt])for k,v in example['exact_grouped_LQR'].items()},
        exact_terms_m_s={k:float(v[belt])for k,v in exact_terms.items()},
        equilibrium_m_s=float(equilibrium[belt]),requested_m_s=float(historical['exact_native_requested'][i,belt]),
        applied_m_s=float(historical['exact_applied'][i,belt]),bound_m_s=float(historical['input_bounds'][i,belt]),
        over_bound_ratio=float(historical['exact_native_requested'][i,belt]/historical['input_bounds'][i,belt]),
        attitude_plus_angular_share_of_LQR=float((example['exact_grouped_LQR']['attitude_tangent'][belt]+example['exact_grouped_LQR']['angular_velocity'][belt])/exact_terms['LQR'][belt]),
        exact_sum_residual=float(exact_sum[belt]-historical['exact_native_requested'][i,belt]),
        requested_reference_body_velocity_m_s=historical['exact_reference_velocity_m_s'][i],
        zero_desired_angular_rate=bool(np.array_equal(historical['exact_feedback_reference_qvel'][i,3:6],np.zeros(3))),
        explanation='This command requests local belt surface speed, not robot body speed. The large positive demand comes predominantly from attitude/angular-rate feedback; nominal rolling FF is small and negative.')
    candidate_tables={}
    for name in ('initial','effort005','adaptive'):
        rel=f'docs/iterations/2026-11-25/comparison_{name}.json'
        report=json.loads((ROOT/rel).read_text())
        keys=('route','phase_rad','variant','completed','rms_moving_mm','max_error_mm','final_error_mm',
              'max_reference_joint_speed_rad_s','max_measured_joint_speed_rad_s','passed_all_gates',
              'reference_speed_violation_samples','reference_acceleration_violation_steps')
        candidate_tables['W20_'+name]=dict(source=rel,rows=[{k:r[k]for k in keys}for r in report['rows']])
    with (ROOT/'docs/iterations/2026-11-25/diagnostic_comparison.csv').open()as stream:
        rows=list(csv.DictReader(stream))
    candidate_tables['D10']=dict(source='docs/iterations/2026-11-25/diagnostic_comparison.csv',rows=rows,
        limitation='D10 andW20 have separatelyexecutedbaselines; do not mix them to claim an algorithm effect.')
    result=dict(scope='Saved-array/code audit for presentation. No new simulations, control, observer or dynamics execution.',
        state_group_indices=GROUPS,selected_times_s=TIMES,verified_contexts=len(contexts),contexts=contexts,
        maximum_normalized_float32_component_reconstruction_residual=max(r['normalized_max_abs_reconstruction_residual']for r in contexts),
        example_6_825=example_verified,candidate_tables=candidate_tables,
        all_36_LQR_float32_cast_comparisons_exact=True,
        all_36_desired_angular_rates_zero=all(np.array_equal(r['feedback_reference_angular_velocity'],np.zeros(3))for r in contexts),
        numerical_caveats=['Grouped error uses quaternion tangent attitude coordinates, not Euler roll/pitch.',
            'I42 arms share program times but physical/controller histories diverge afterintervention.',
            'Most original rollout components arefloat32. Exact LQR gains/errors andI46historical exactcomponentlogging isolate rounding.',
            'The decomposition identifies command origins, not a unique physical cause of the departure.'])
    for row in declaration['inputs']:
        assert sha(ROOT/row['path']) == row['sha256']
    (HERE/'saved_decomposition.json').write_text(json.dumps(safe(result),indent=2)+'\n')
    print(json.dumps(safe(example_verified),indent=2))


if __name__=='__main__':main()
