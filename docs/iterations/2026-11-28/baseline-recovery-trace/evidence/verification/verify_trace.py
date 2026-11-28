"""Independent I-86 saved-array and loaded-support audit; no physics steps.

MuJoCo is used only for kinematics and subtree COM velocity on recorded poses.
Spatial groups are descriptive contact clusters, not an assertion of independent
rolling constraints. All impulses below are force-sample quadrature, not exact
solver impulse exports.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil

import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[3]
BASELINE = ROOT/'results/current_belt_capacity_20261128/campaign/runs/B060/child/campaign/runs/eight_m100_N/precision.npz'
OLD_SEAL = ROOT/'results/current_belt_capacity_20261128/campaign/outputs_sealed.json'
DEFAULT_MODEL = ROOT/'results/current_belt_capacity_20261128/campaign/runs/B060/child/campaign/_workspaces/eight_m100_N/mirror/model.xml'
WINDOW = (105., 106.3)
FOCUS = (105.4, 106.1)
SELECTED = (105.4, 105.5, 105.525, 105.530, 105.590, 105.725, 105.9, 106., 106.1)
SEPARATION_M = .010
SHARE_LEVELS = (.01, .05, .10)
EPS_N = 1e-9
FAMILIES = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
TAG_TO_FAMILY = np.array([0, 1, 2, 3, 0, 1, -1])


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def clean(x):
    if isinstance(x, dict): return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)): return [clean(v) for v in x]
    if isinstance(x, np.ndarray): return clean(x.tolist())
    if isinstance(x, np.generic): return clean(x.item())
    if isinstance(x, float) and not np.isfinite(x): return None
    return x


def save(path, obj):
    with Path(path).open('x') as stream:
        json.dump(clean(obj), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def read_arrays(path):
    with np.load(path, allow_pickle=False) as z:
        return {key: z[key].copy() for key in z.files}


def stats(x):
    x = np.asarray(x, float)
    return dict(count=len(x), mean=float(np.mean(x)), rms=float(np.sqrt(np.mean(x*x))),
                max_abs=float(np.max(np.abs(x))))


def components(points, separation=SEPARATION_M):
    """Connected components of the <=10 mm graph; no tag-dependent splitting."""
    points = np.asarray(points, float).reshape(-1, 3)
    unseen = set(range(len(points))); result = []
    while unseen:
        seed = min(unseen); unseen.remove(seed); group = [seed]; queue = [seed]
        while queue:
            i = queue.pop()
            neighbours = [j for j in sorted(unseen) if np.linalg.norm(points[j]-points[i]) <= separation]
            unseen.difference_update(neighbours); queue.extend(neighbours); group.extend(neighbours)
        result.append(np.array(sorted(group), dtype=int))
    return result


def contact_summary(rows):
    rows = np.asarray(rows, float).reshape(-1, 16)
    if not np.isfinite(rows).all(): raise ValueError('Nonfinite contact row')
    tags = rows[:, 0].astype(int)
    if not np.array_equal(tags, rows[:, 0]) or np.any((tags < 0) | (tags > 6)):
        raise ValueError('Unknown contact tag')
    fn = np.einsum('ij,ij->i', rows[:, 4:7], rows[:, 7:10])
    loaded = (rows[:, 12] >= .5) & (fn > EPS_N)
    c, weights = rows[loaded], fn[loaded]
    loaded_tags = tags[loaded]
    total = float(weights.sum())
    pressure_tag = np.bincount(loaded_tags, weights=weights, minlength=7)
    family_pressure = np.array([pressure_tag[TAG_TO_FAMILY == i].sum() for i in range(4)])
    family_shares = family_pressure/total if total > EPS_N else np.zeros(4)
    groups = []
    for ids in components(c[:, 1:4]):
        p = float(weights[ids].sum())
        point = np.average(c[ids, 1:4], axis=0, weights=weights[ids])
        normal = np.average(c[ids, 4:7], axis=0, weights=weights[ids])
        norm = np.linalg.norm(normal)
        normal = normal/norm if norm > 1e-12 else np.zeros(3)
        diam = max((np.linalg.norm(c[i, 1:4]-c[j, 1:4]) for i in ids for j in ids), default=0.)
        groups.append(dict(positive_pressure_N=p, share=p/total, point_m=point,
                           pressure_weighted_normal=normal, solver_points=len(ids),
                           tags=sorted(set(loaded_tags[ids].tolist())),
                           signed_force_N=c[ids, 7:10].sum(axis=0), diameter_m=diam))
    groups.sort(key=lambda x: -x['positive_pressure_N'])
    same_family_groups = []
    for i in range(4):
        mask = TAG_TO_FAMILY[loaded_tags] == i
        counts = components(c[mask, 1:4])
        fam_weights = weights[mask]
        shares = sorted([float(fam_weights[k].sum()/total) for k in counts], reverse=True) if total > EPS_N else []
        same_family_groups.append(shares)
    admitted = rows[:, 12] >= .5
    native = admitted & ((rows[:, 10] > 0) | (fn > 0))
    total_force = rows[:, 7:10].sum(axis=0)
    max_separation = max((np.linalg.norm(c[i, 1:4]-c[j, 1:4]) for i in range(len(c)) for j in range(len(c))), default=0.)
    return dict(raw_solver_points=len(rows), loaded_solver_points=len(c), total_pressure_N=total,
                signed_force_N=total_force, admitted_force_N=rows[admitted, 7:10].sum(axis=0),
                native_force_N=rows[native, 7:10].sum(axis=0),
                rejected_force_N=rows[~admitted, 7:10].sum(axis=0), pressure_by_tag_N=pressure_tag,
                pressure_by_family_N=family_pressure, family_shares=family_shares,
                body_pressure_share=float(pressure_tag[6]/total) if total > EPS_N else 0.,
                spatial_groups=groups, same_family_spatial_group_shares=same_family_groups,
                max_loaded_point_separation_m=max_separation,
                group_counts_at_share=[sum(g['share'] >= x for g in groups) for x in SHARE_LEVELS],
                family_counts_at_share=[int(np.count_nonzero(family_shares >= x)) for x in SHARE_LEVELS],
                second_spatial_group_share=groups[1]['share'] if len(groups) > 1 else 0.,
                second_family_share=float(np.sort(family_shares)[-2]))


def rotation(q):
    q = np.asarray(q, float)/np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def qv(state):
    q = np.r_[state[2:9], state[15:18]]
    q[3:7] /= np.linalg.norm(q[3:7])
    return q, np.r_[state[9:12], rotation(q[3:7]).T@state[12:15], state[18:21]]


def com(model, data, state):
    data.qpos[:], data.qvel[:] = qv(state)
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_comVel(model, data)
    mujoco.mj_subtreeVel(model, data)
    return data.subtree_com[0].copy(), data.subtree_linvel[0].copy()


def check_holds(p, s):
    holds = np.arange(21000, 21260)
    assert len(s['time_s']) == 1300 and np.all(s['hold_completed'])
    assert np.array_equal(s['hold_index'], np.repeat(holds, 5))
    assert np.array_equal(s['substep_index'], np.tile(np.arange(1, 6), len(holds)))
    assert np.array_equal(s['held_input'], p['exact_applied'][s['hold_index']])
    assert np.max(np.abs(s['time_s']-(p['time_s'][s['hold_index']]+s['substep_index']*.001))) < 1e-9
    assert np.array_equal(p['exact_previous_applied'][1:], p['exact_applied'][:-1])
    assert np.array_equal(s['bridge_state_before'][1:], s['bridge_state'][:-1])
    assert np.max(np.abs(s['bridge_state'][:, 0]-s['bridge_state_before'][:, 0]-.001)) < 1e-8


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    args = ap.parse_args()
    run, out, model_path = args.run.resolve(), args.output.resolve(), args.model.resolve()
    out.mkdir(parents=True, exist_ok=False)
    pp = run/'child/campaign/runs/eight_m100_N/precision.npz'
    sp = run/'substeps.npz'
    metadata_path = run/'diagnostic_metadata.json'
    paths = [Path(__file__).resolve(), BASELINE, OLD_SEAL, model_path, pp, sp, metadata_path]
    bound = {str(p.relative_to(ROOT)): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in paths}
    shutil.copyfile(Path(__file__), out/'verify_trace.py')
    save(out/'registered_before_array_reads.json', dict(
        iteration='I-86', utc=datetime.now(timezone.utc).isoformat(), source_and_inputs=bound,
        window_s=WINDOW, focus_s=FOCUS, selected_times_s=SELECTED,
        spatial_group_rule='Connected components of loaded contact points joined at <=10mm; different tags do not force a split. This is not proof of independent rolling constraints.',
        separation_m=SEPARATION_M, positive_pressure_epsilon_N=EPS_N, relative_share_sensitivities=SHARE_LEVELS,
        numerator='Admitted positive normal pressure fn=n dot f. Signed whole-COM force includes all exported rows.',
        force_scope='Force samples integrated with dt; not exported exact solver impulses. Independent kinematics only; no dynamics simulation or observer update.',
        checks='All79baselineprefix;states/commands/timing;1ms and5ms COM momentum closure;separated spatial groups and motor-family loads;no causal intervention.'))
    seal = json.loads(OLD_SEAL.read_text())
    expected = seal['files'][str(BASELINE.relative_to(OLD_SEAL.parent.parent))]
    assert bound[str(BASELINE.relative_to(ROOT))] == expected
    p, old, s = read_arrays(pp), read_arrays(BASELINE), read_arrays(sp)
    metadata = json.loads(metadata_path.read_text())
    assert metadata['native_run_returned'] is True
    assert metadata['controller_intervention'] is False
    counts = metadata['command_preview_counts']
    assert counts == dict(commands=21301, previews=21301, observer_updates=21301)
    assert set(p) == set(old) and len(p) == 79
    n = len(p['time_s']); assert n == 21301 and abs(p['time_s'][-1]-106.5) < 1e-9
    fields = []
    for key in sorted(old):
        e = old[key][:n].copy()
        if key == 'command_held_after_row': e[-1] = False
        okay = p[key].dtype == e.dtype and p[key].shape == e.shape and p[key].tobytes() == e.tobytes()
        fields.append(dict(field=key, bit_exact=okay))
    assert all(row['bit_exact'] for row in fields)
    check_holds(p, s)
    for key in ('native_feedback_boundary_verified', 'mirror_previous_applied_matches_physical', 'no_truth_or_terrain_read_during_command'):
        assert np.all(p[key]), key
    assert np.all(p['native_observer_update_count'] == 1)
    assert np.array_equal(p['exact_applied'], np.clip(p['exact_requested'], -p['exact_input_bounds'], p['exact_input_bounds']))
    lqr_error = float(np.max(abs(-np.einsum('nij,nj->ni', p['exact_feedback_gain'], p['exact_error'])-p['exact_lqr_command'])))
    requested = sum(p[k] for k in ('exact_equilibrium_command', 'exact_rolling_feedforward', 'exact_support_command', 'exact_lqr_command', 'exact_transition_command_delta', 'exact_heading_command', 'exact_fade_command_delta'))
    request_error = float(np.max(abs(requested-p['exact_requested'])))
    assert max(lqr_error, request_error) < 1e-10
    model = mujoco.MjModel.from_xml_path(str(model_path)); data = mujoco.MjData(model)
    mass = float(model.body_mass.sum()); gravity = model.opt.gravity.copy()
    assert not np.any(model.dof_damping[:6]) and not np.any(model.dof_frictionloss[:6])
    assert not np.any(model.body_gravcomp) and model.opt.viscosity == 0 and model.opt.density == 0
    assert len(s['contact_offsets']) == 1301 and s['contact_offsets'][0] == 0
    assert s['contact_offsets'][-1] == len(s['contact_rows']) and np.all(np.diff(s['contact_offsets']) >= 0)
    assert np.array_equal(s['contact_tags'], s['contact_rows'][:, 0].astype(int))
    assert np.array_equal(s['contact_admitted'], s['contact_rows'][:, 12] >= .5)
    summaries, positions_pre, positions_post, velocities_pre, velocities_post = [], [], [], [], []
    q_error, v_error = [], []
    for j in range(1300):
        c = s['contact_rows'][s['contact_offsets'][j]:s['contact_offsets'][j+1]]
        summaries.append(contact_summary(c))
        pre = com(model, data, s['bridge_state_before'][j]); post = com(model, data, s['bridge_state'][j])
        positions_pre.append(pre[0]); velocities_pre.append(pre[1]); positions_post.append(post[0]); velocities_post.append(post[1])
        if s['substep_index'][j] == 5:
            row = s['hold_index'][j]+1; q, v = qv(s['bridge_state'][j])
            q_error.append(float(np.max(abs(q-p['exact_qpos'][row])))); v_error.append(float(np.max(abs(v-p['exact_qvel'][row]))))
    assert max(q_error) < 1e-12 and max(v_error) < 1e-12
    vp, va = np.array(velocities_pre), np.array(velocities_post)
    force = np.array([x['signed_force_N'] for x in summaries])
    admitted_force = np.array([x['admitted_force_N'] for x in summaries])
    native_force = np.array([x['native_force_N'] for x in summaries])
    external = force+mass*gravity+s['disturbance_world'][:, :3]
    required = mass*(va-vp)/.001
    residual = required-external
    admitted_residual = required-(admitted_force+mass*gravity+s['disturbance_world'][:, :3])
    native_residual = required-(native_force+mass*gravity+s['disturbance_world'][:, :3])
    pressure = np.array([x['pressure_by_family_N'] for x in summaries])
    ptag = np.array([x['pressure_by_tag_N'] for x in summaries])
    total_pressure = np.array([x['total_pressure_N'] for x in summaries])
    group_counts = np.array([x['group_counts_at_share'] for x in summaries])
    family_counts = np.array([x['family_counts_at_share'] for x in summaries])
    intervals = []
    for k, hold in enumerate(range(21000, 21260)):
        lo, hi = 5*k, 5*k+5
        F = force[lo:hi].mean(axis=0)
        momentum = mass*(va[hi-1]-vp[lo])/.005
        net = external[lo:hi].mean(axis=0)
        mean_pressure = pressure[lo:hi].mean(axis=0)
        end = hold+1
        intervals.append(dict(time_s=float(p['time_s'][end]), hold_start_s=float(p['time_s'][hold]),
            source_row=int(end), held_input=p['exact_applied'][hold],
            new_command_after_end_state=p['exact_applied'][end], contact_force_N=F,
            admitted_contact_force_N=admitted_force[lo:hi].mean(axis=0),
            native_contact_force_N=native_force[lo:hi].mean(axis=0),
            gravity_force_N=mass*gravity, COM_momentum_rate_N=momentum,
            force_balance_residual_N=momentum-net, family_mean_pressure_N=mean_pressure,
            family_last_pressure_N=pressure[hi-1], inferred_end_loads_N=p['exact_estimated_support_loads_N'][end],
            no_resolved_positive_pressure_substeps=int(np.count_nonzero(total_pressure[lo:hi] <= EPS_N))))
    res5 = np.array([x['force_balance_residual_N'] for x in intervals])
    focus_sub = (s['time_s'] >= FOCUS[0]-1e-9) & (s['time_s'] <= FOCUS[1]+1e-9)
    pressure_impulse = ptag.sum(axis=0)*.001
    scalar_total_impulse = float(pressure_impulse.sum())
    selected_rows = []
    for target in SELECTED:
        j = int(np.argmin(abs(s['time_s']-target)))
        item = dict(requested_time_s=target, time_s=float(s['time_s'][j]),
                    held_from_time_s=float(s['control_start_time_s'][j]),
                    held_input=s['held_input'][j], COM_velocity_pre_m_s=vp[j],
                    COM_velocity_post_m_s=va[j], required_net_force_N=required[j],
                    sampled_net_force_N=external[j], residual_N=residual[j], **summaries[j])
        selected_rows.append(item)
    no_pressure = np.flatnonzero(total_pressure <= EPS_N)
    sensitivity = []
    for i, level in enumerate(SHARE_LEVELS):
        sensitivity.append(dict(minimum_fraction_of_total_pressure=level,
            two_or_more_spatial_groups_substeps=int(np.count_nonzero(group_counts[:, i] >= 2)),
            two_or_more_spatial_groups_duration_s=float(np.count_nonzero(group_counts[:, i] >= 2)*.001),
            two_or_more_motor_families_substeps=int(np.count_nonzero(family_counts[:, i] >= 2)),
            two_or_more_motor_families_duration_s=float(np.count_nonzero(family_counts[:, i] >= 2)*.001),
            focus_two_or_more_spatial_groups_substeps=int(np.count_nonzero((group_counts[:, i] >= 2)&focus_sub)),
            focus_two_or_more_motor_families_substeps=int(np.count_nonzero((family_counts[:, i] >= 2)&focus_sub))))
    report = dict(iteration='I-86', passed=True, source_and_input_bindings_unchanged=True,
        no_new_physics=True, precision_rows=n, precision_fields=fields, trace_substeps=1300, completed_holds=260,
        qpos_endpoint_max_abs_error=max(q_error), qvel_endpoint_max_abs_error=max(v_error),
        lqr_ledger_max_abs_error=lqr_error, requested_ledger_max_abs_error=request_error,
        total_mass_kg=mass, gravity_m_s2=gravity, mujoco_version=mujoco.__version__,
        all_substeps_force_residual_norm_N=stats(np.linalg.norm(residual, axis=1)),
        admitted_substeps_force_residual_norm_N=stats(np.linalg.norm(admitted_residual, axis=1)),
        native_substeps_force_residual_norm_N=stats(np.linalg.norm(native_residual, axis=1)),
        rejected_force_norm_N=stats(np.linalg.norm(force-admitted_force, axis=1)),
        all_holds_force_residual_norm_N=stats(np.linalg.norm(res5, axis=1)),
        sampled_contact_world_impulse_Ns=force.sum(axis=0)*.001,
        sampled_net_world_impulse_Ns=external.sum(axis=0)*.001,
        whole_window_COM_momentum_change_Ns=mass*(va[-1]-vp[0]),
        whole_window_impulse_residual_Ns=mass*(va[-1]-vp[0])-external.sum(axis=0)*.001,
        pressure_sample_integral_by_tag_Ns=pressure_impulse,
        pressure_sample_integral_tag_fractions=pressure_impulse/scalar_total_impulse if scalar_total_impulse > 0 else np.zeros(7),
        scalar_positive_pressure_sample_integral_Ns=scalar_total_impulse,
        no_resolved_positive_pressure_times_s=s['time_s'][no_pressure],
        maximum_second_spatial_group_pressure_share=max(x['second_spatial_group_share'] for x in summaries),
        maximum_second_motor_family_pressure_share=max(x['second_family_share'] for x in summaries),
        maximum_loaded_point_separation_m=max(x['max_loaded_point_separation_m'] for x in summaries),
        spatial_group_sensitivities=sensitivity,
        selected_substep_rows=selected_rows,
        selected_5ms_intervals=[min(intervals, key=lambda x: abs(x['time_s']-t)) for t in SELECTED],
        rows=intervals,
        qualifications=[
            'Raw contact-point counts and 10mm spatial components are not demonstrated independent kinematic supports.',
            'Different shell/belt motor families can matter even at nearby contact locations; continuous load shares are retained.',
            'An almost unloaded contact can be negligible in present force balance while still mattering at a future load transfer.',
            'Signed total contact force includes every exported row; positive-pressure clustering excludes rejected and noncompressive rows.',
            'Force sample times are interval ends; multiplying by1ms is sampled quadrature, not an exact solver impulse export.',
            'New command at an interval end acts after that state. Its preceding held command may have contributed.',
            'No intervention here identifies a controller term as causal or proves that multi-contact explains the recovery failure.'])
    for relative, binding in bound.items():
        path = ROOT/relative
        assert path.stat().st_size == binding['bytes'] and sha(path) == binding['sha256']
    save(out/'report.json', report)
    np.savez_compressed(out/'independent_signals.npz', time_s=s['time_s'],
        COM_pre_position_m=positions_pre, COM_post_position_m=positions_post,
        COM_pre_velocity_m_s=vp, COM_post_velocity_m_s=va,
        contact_force_N=force, sampled_net_force_N=external, COM_required_net_force_N=required,
        admitted_contact_force_N=admitted_force, native_contact_force_N=native_force,
        force_balance_residual_N=residual, pressure_by_tag_N=ptag, pressure_by_family_N=pressure,
        total_pressure_N=total_pressure, spatial_group_counts=group_counts, motor_family_counts=family_counts,
        second_spatial_group_share=[x['second_spatial_group_share'] for x in summaries],
        second_motor_family_share=[x['second_family_share'] for x in summaries],
        raw_solver_point_count=[x['raw_solver_points'] for x in summaries],
        loaded_solver_point_count=[x['loaded_solver_points'] for x in summaries])
    save(out/'artifact_manifest.json', dict(files={p.name: dict(bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(out.iterdir()) if p.is_file()}))
    print(json.dumps(clean(dict(passed=True, precision_fields=len(fields), substeps=1300,
        force_residual_5ms_N=report['all_holds_force_residual_norm_N'],
        max_second_spatial_group_share=report['maximum_second_spatial_group_pressure_share'],
        max_second_motor_family_share=report['maximum_second_motor_family_pressure_share']))))


if __name__ == '__main__':
    main()
