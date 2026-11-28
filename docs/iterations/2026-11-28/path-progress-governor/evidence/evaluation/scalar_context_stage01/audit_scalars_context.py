"""Posthoc independent scalar audit and fixed +0.3 peak contexts; no rescore.

Does not import either metric/evaluator module or their saved signal arrays.
Registers its exact source and closed raw inputs before opening NPZ arrays.
The original report, frozen gates, experiment and decision remain unchanged.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]
DT = .005


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write(p, value):
    with Path(p).open('x') as f:
        json.dump(value, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')


def stats(a):
    a = np.asarray(a)
    return dict(complete=bool(len(a) and np.isfinite(a).all()), count=len(a),
                finite_count=int(np.isfinite(a).sum()), maximum=float(np.max(a)),
                mean=float(np.mean(a)), rms=float(np.linalg.norm(a)/np.sqrt(len(a))),
                p95=float(np.percentile(a, 95)))


def axes(q):
    q = q/np.sqrt(np.sum(q*q, axis=1))[:, None]
    w, x, y, z = q.T
    # Independently form only R's third column and third row, not full matrices.
    up = np.column_stack((2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)))
    row_z = np.column_stack((2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)))
    return up, row_z


def field(row, keys):
    for key in keys:
        row = row[key]
    return row


def finite_value(a):
    a = np.asarray(a)
    if a.ndim == 0:
        return float(a) if np.isfinite(a) else None
    return [finite_value(x) for x in a]


def independent_case(raw, side, outcome, job):
    t, q, v = raw['time_s'], raw['exact_qpos'], raw['exact_qvel']
    arrival = float(outcome['first_complete_sample_s'])
    end = arrival+10.
    episode = t <= end+1e-9
    travel = (t >= 3.-1e-9) & (t <= arrival+1e-9)
    final = (t >= end-2.-1e-9) & (t <= end+1e-9)
    direction = np.array([np.cos(113.*np.pi/180.), np.sin(113.*np.pi/180.)])
    origin = np.array([-2.4, 10.])
    goal = origin+1.2*direction
    delta = q[:, :2]-origin
    along = delta[:, 0]*direction[0]+delta[:, 1]*direction[1]
    cross = delta[:, 0]*direction[1]-delta[:, 1]*direction[0]
    # Finite line: perpendicular distance in the interior, endpoint norms outside.
    error = np.abs(cross)
    error[along < 0.] = np.linalg.norm(delta[along < 0.], axis=1)
    error[along > 1.2] = np.linalg.norm(q[along > 1.2, :2]-goal, axis=1)
    goal_error = np.linalg.norm(q[:, :2]-goal, axis=1)
    speed = np.sqrt(v[:, 0]**2+v[:, 1]**2)
    up, row_z = axes(q[:, 3:7])
    target_up, _ = axes(raw['exact_reference_quat'])
    lean = np.arccos(np.clip(np.sum(up*target_up, axis=1), -1., 1.))*180./np.pi
    tilt = np.arccos(np.clip(up[:, 2], -1., 1.))*180./np.pi
    wz = np.sum(row_z*v[:, 3:6], axis=1)
    request, applied, prior, caps = (raw[k] for k in (
        'exact_requested', 'exact_applied', 'exact_previous_applied', 'exact_input_bounds'))
    held = raw['command_held_after_row'].astype(bool) & (t < end-1e-9)
    clipped = np.abs(request) > caps+1e-12
    overflow = request-np.minimum(np.maximum(request, -caps), caps)
    excess = np.sqrt(np.sum((overflow/caps)**2, axis=1))
    step = np.sqrt(np.sum(((applied-prior)/caps)**2, axis=1))
    u = side['gov__progress_u']
    ut, et = u[travel], error[travel]
    du = np.maximum(np.diff(ut), 0.)
    prms = np.sqrt(np.dot(du, (et[:-1]**2+et[1:]**2)/2.)/np.sum(du))
    acceleration = np.linalg.norm((v[10:, :3]-v[:-10, :3])/.05, axis=1)
    accel_mask = episode[10:] & episode[:-10]
    final_position = bool(final.sum() == 401 and np.all(goal_error[final] <= .02+1e-12)
                          and np.all(speed[final] <= .02+1e-12))
    four = bool(final_position and np.all(np.abs(wz[final]) <= .05+1e-12)
                and np.all(lean[final] <= 2.+1e-12))
    tau = side['gov__virtual_time_s']
    stall_windows = []
    for i in range(len(t)-1000):
        j = i+1000
        if t[i] >= 3.-1e-9 and t[j] < arrival-1e-9 and tau[j]-tau[i] < .05-1e-12:
            if np.min(goal_error[i:j+1]) > .02:
                stall_windows.append([i, j])
    hold = bool(t[-1] >= end-1e-9 and end <= job['maximum_duration_s']+1e-9)
    arrival_idx = np.flatnonzero(np.abs(t-arrival) <= 1e-9)
    near_terminal = np.flatnonzero(tau >= 35.-1e-9)
    arrival_ok = bool(len(arrival_idx) == 1 and len(near_terminal) and
        tau[arrival_idx[0]] >= 35.-1e-9 and arrival-t[near_terminal[0]] <= DT+1e-9 and
        np.all(np.abs(u[arrival_idx[0]:]-1.) <= 1e-10) and
        np.all(np.abs(raw['exact_reference_xy'][arrival_idx[0]:]-goal) <= 1e-10) and
        np.all(np.abs(raw['exact_reference_velocity_m_s'][arrival_idx[0]:]) <= 1e-10) and
        np.all(np.abs(raw['exact_reference_acceleration_m_s2'][arrival_idx[0]:]) <= 1e-10))
    complete = bool(outcome['reference_complete'] and arrival <= t[-1]+1e-9 and arrival_ok)
    finite = bool(all(np.isfinite(a).all() for a in (q, v, request[held], applied[held], prior[held], caps[held])))
    raw_held = raw['command_held_after_row'].astype(bool)
    row = dict(
        declared_reference_arrival_s=arrival, actual_last_time_s=float(t[-1]),
        arrival_added_time_s=arrival-35., reference_path_completed=complete,
        full_ten_second_hold=hold, physical_run_completed=bool(outcome['survived_without_native_failure']),
        grid_ok=bool(len(t) >= 2 and abs(t[0]) <= 1e-9 and np.all(np.abs(np.diff(t)-DT) <= 1e-9)),
        held_mask_ok=bool(np.array_equal(raw_held, np.arange(len(t)) < len(t)-1)),
        finite_state_commands=finite,
        cap_ok=bool(finite and np.all(caps[held] > 0) and np.all(np.abs(applied[held]) <= caps[held]+1e-12)),
        clip_identity=bool(np.array_equal(applied[raw_held], np.minimum(np.maximum(request[raw_held], -caps[raw_held]), caps[raw_held]))),
        prior_input_identity=bool(np.array_equal(prior[1:], applied[:-1])),
        finite_reference=bool(all(np.isfinite(a).all() for a in (u, side['gov__progress_rate_s_inv'], side['gov__progress_acceleration_s_inv2'], raw['exact_reference_quat'], raw['exact_reference_xy'], raw['exact_reference_velocity_m_s'], raw['exact_reference_acceleration_m_s2']))),
        progress_valid=bool(np.isfinite(u).all() and np.all(u >= -1e-12) and np.all(u <= 1.+1e-12) and np.all(np.diff(u) >= -1e-12)),
        virtual_clock_valid=bool(np.isfinite(tau).all() and abs(tau[0]-t[0]) <= 1e-9 and np.all(np.diff(tau) >= -1e-12) and np.all(np.diff(tau) <= np.diff(t)+1e-9) and np.all(tau <= t+1e-9)),
        native_measurement_boundary_ok=bool(np.all(raw['native_observer_update_count'] == 1) and np.all(raw['no_truth_or_terrain_read_during_command']) and np.all(raw['mirror_previous_applied_matches_physical']) and not np.any(raw['exact_heading_command'])),
        tracking=dict(geometric_traversal=stats(error[travel]),
            geometric_progress_rms=dict(rms=float(prms), covered_progress=float(ut[-1]-ut[0])),
            own_reference_xy=stats(np.linalg.norm(q[:, :2]-raw['exact_reference_xy'], axis=1)[travel]),
            goal_error_at_primary_end_m=float(goal_error[np.flatnonzero(episode)[-1]]),
            physical_goal_position_converged=final_position),
        attitude=dict(own_raw_target_peak_deg=float(np.max(lean[episode])),
            own_raw_target_rms_deg=float(np.linalg.norm(lean[episode])/np.sqrt(episode.sum())),
            absolute_tilt_peak_deg=float(np.max(tilt[episode]))),
        motion=dict(planar_speed_peak_m_s=float(np.max(speed[episode])),
            world_z_rate_peak_rad_s=float(np.max(np.abs(wz[episode]))),
            acceleration_50ms_peak_m_s2=float(np.max(acceleration[accel_mask]))),
        settling=dict(final_window_complete=bool(final.sum() == 401), final_samples=int(final.sum()),
            physical_position_passed=final_position, four_condition_passed=four,
            goal=stats(goal_error[final]), speed=stats(speed[final]),
            world_z_rate=stats(np.abs(wz[final])), own_reference_up_error=stats(lean[final])),
        commands=dict(held_intervals=int(held.sum()),
            clipped_any_seconds=float(np.count_nonzero(np.any(clipped[held], axis=1))*.005),
            clipped_channel_seconds=(np.count_nonzero(clipped[held], axis=0)*.005).tolist(),
            clipped_any_fraction=float(np.mean(np.any(clipped[held], axis=1))),
            clipped_channel_fraction=np.mean(clipped[held], axis=0).tolist(),
            normalized_clipping_excess=stats(excess[held]), normalized_applied_step=stats(step[held]),
            requested_peak_abs=np.max(np.abs(request[held]), axis=0).tolist()),
        no_progress=dict(passed=not stall_windows, flagged_windows=len(stall_windows)))
    signals = dict(time_s=t, tilt=tilt, speed=speed, lean=lean, wz=wz, episode=episode,
                   geometric=error, excess=excess, reference_speed=np.linalg.norm(raw['exact_reference_velocity_m_s'], axis=1))
    return row, signals


def compare_leaves(a, b, prefix=''):
    checks = []
    for key, value in a.items():
        name = prefix+'.'+key if prefix else key
        if isinstance(value, dict):
            checks += compare_leaves(value, b[key], name)
        else:
            other = b[key]
            if isinstance(value, bool):
                okay = value == other
                maximum_error = None
            else:
                va, vb = np.asarray(value), np.asarray(other)
                maximum_error = float(np.max(np.abs(va-vb)))
                okay = bool(va.shape == vb.shape and np.allclose(va, vb, atol=5e-10, rtol=2e-12))
            checks.append(dict(name=name, passed=bool(okay), maximum_error=maximum_error))
    return checks


def snapshot(raw, side, signals, idx):
    result = dict(row=int(idx), physical_time_s=float(raw['time_s'][idx]),
        speed_m_s=float(signals['speed'][idx]), absolute_tilt_deg=float(signals['tilt'][idx]),
        own_target_up_error_deg=float(signals['lean'][idx]), world_z_rate_rad_s=float(signals['wz'][idx]),
        geometric_error_m=float(signals['geometric'][idx]),
        reference_speed_m_s=float(signals['reference_speed'][idx]),
        normalized_clipping_excess=float(signals['excess'][idx]))
    names = ('gov__virtual_time_s', 'gov__clock_rate', 'gov__clock_acceleration_s_inv',
        'gov__progress_u', 'gov__progress_rate_s_inv', 'gov__progress_acceleration_s_inv2',
        'gov__authority_factor', 'gov__joint_reference_factor', 'gov__command_factor',
        'gov__raw_target_rate', 'gov__authority_sigma_min_m_s',
        'gov__maximum_nominal_joint_reference_rad_s', 'gov__nominal_command_ratio',
        'probe_joint_reference', 'gov__retiming_ever_active')
    for name in names:
        if name in side:
            result[name] = finite_value(side[name][idx])
    for name in ('exact_qpos', 'exact_qvel', 'exact_requested', 'exact_applied', 'exact_input_bounds',
                 'exact_reference_xy', 'exact_reference_velocity_m_s', 'exact_reference_acceleration_m_s2',
                 'exact_reference_quat'):
        result[name] = finite_value(raw[name][idx])
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--protocol', type=Path, default=BASE/'protocol/protocol.json')
    ap.add_argument('--evaluation', type=Path, default=BASE/'evaluation/completed_stage01')
    ap.add_argument('--output', type=Path, default=BASE/'evaluation/scalar_context_stage01')
    args = ap.parse_args()
    protocol = json.loads(args.protocol.read_text())
    canonical_file = args.evaluation/'report.json'
    canonical = json.loads(canonical_file.read_text())
    out = args.output.resolve()
    files = [Path(__file__).resolve(), args.protocol.resolve(), canonical_file.resolve(), BASE/'campaign/outputs_sealed.json']
    paths = {}
    for job in protocol['cases']:
        cid = job['case_id']
        case = BASE/protocol['run_root']/cid
        paths[cid] = (case/'child/campaign/runs'/(cid.rsplit('_', 1)[0]+'_N')/'precision.npz',
                      case/'governor_diagnostics.npz', case/'i78_outcome.json')
        files.extend(paths[cid])
    bindings = {str(p.relative_to(ROOT)): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in files}
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, out/Path(__file__).name)
    write(out/'registered_before_arrays.json', dict(utc=datetime.now(timezone.utc).isoformat(), files=bindings,
        posthoc=True, purpose='Independent scalar/gate audit and observational +0.3 peak context; no canonical rescore.',
        no_npz_arrays_opened_in_this_stage_before_registration=True, no_new_physics=True,
        context_rule='Each N/G +0.3 full-primary-episode planar-speed and absolute-tilt argmax; fixed +/-0.25s; also traversal slowdown-factor minima and first active rate. No gate or window changes.'))
    rows, bundles, checks = {}, {}, []
    for job in protocol['cases']:
        cid = job['case_id']; p, s, o = paths[cid]
        with np.load(p, allow_pickle=False) as z:
            raw = {k:z[k].copy() for k in z.files}
        with np.load(s, allow_pickle=False) as z:
            side = {k:z[k].copy() for k in z.files}
        outcome = json.loads(o.read_text())
        row, signals = independent_case(raw, side, outcome, job)
        rows[cid] = row; bundles[cid] = (raw, side, signals)
        checks += [dict(case_id=cid, **c) for c in compare_leaves(row, canonical['cases'][cid])]
    gates = []
    cfg = protocol['evaluation']
    for phase in (-1., .3, .9):
        ids = {j['policy']:j['case_id'] for j in protocol['cases'] if j['phase_rad'] == phase}
        n, g = rows[ids['N']], rows[ids['G']]
        physical_keys = ('physical_run_completed', 'grid_ok', 'held_mask_ok', 'finite_state_commands', 'cap_ok',
            'clip_identity', 'prior_input_identity', 'finite_reference', 'reference_path_completed',
            'full_ten_second_hold', 'progress_valid', 'virtual_clock_valid', 'native_measurement_boundary_ok')
        gates.append(dict(name=f'{phase}_physical_completion_and_bounds', passed=bool(all(n[k] and g[k] for k in physical_keys))))
        for name, definition in cfg['nonregression'].items():
            nv, gv = field(n, definition['path']), field(g, definition['path'])
            limit = nv*definition['ratio']+definition['add']
            gates.append(dict(name=f'{phase}_{name}', native=nv, actual=gv, limit=limit, passed=bool(gv <= limit+1e-12)))
        for key, tolerance in (('clipped_channel_fraction', cfg['channel_clipping_fraction_add']),
                               ('clipped_channel_seconds', cfg['channel_clipping_seconds_add'])):
            for i, (nv, gv) in enumerate(zip(n['commands'][key], g['commands'][key])):
                gates.append(dict(name=f'{phase}_{key}_{i}', native=nv, actual=gv, limit=nv+tolerance, passed=bool(gv <= nv+tolerance+1e-12)))
        gates.append(dict(name=f'{phase}_preserve_native_four_condition_settling', native=n['settling']['four_condition_passed'], actual=g['settling']['four_condition_passed'], passed=bool(not n['settling']['four_condition_passed'] or g['settling']['four_condition_passed'])))
        gates.append(dict(name=f'{phase}_physical_goal_position_convergence', passed=g['settling']['physical_position_passed']))
        gates.append(dict(name=f'{phase}_no_prolonged_clock_stall', passed=g['no_progress']['passed']))
        if phase == -1.:
            nv, gv = n['tracking']['geometric_traversal']['maximum'], g['tracking']['geometric_traversal']['maximum']
            gates.append(dict(name=f'{phase}_minus_one_geometric_peak_benefit', native=nv, actual=gv, limit=cfg['lead_peak_ratio']*nv, passed=bool(gv <= cfg['lead_peak_ratio']*nv+1e-12)))
    canonical_gates = {g['name']:g for g in canonical['decision']['checks']}
    gate_agreement = [dict(name=g['name'], passed=g['passed'] == canonical_gates[g['name']]['passed']) for g in gates]
    contexts = {}
    for policy in ('N', 'G'):
        cid = 'straight_p030_'+policy; raw, side, signals = bundles[cid]
        t = raw['time_s']; episode_indices = np.flatnonzero(signals['episode'])
        events = {}
        for name, signal in (('absolute_tilt_peak', signals['tilt']), ('planar_speed_peak', signals['speed'])):
            idx = int(episode_indices[np.argmax(signal[episode_indices])])
            window = (t >= t[idx]-.25-1e-9) & (t <= t[idx]+.25+1e-9) & signals['episode']
            events[name] = dict(peak=snapshot(raw, side, signals, idx),
                fixed_context_start_s=float(t[window][0]), fixed_context_end_s=float(t[window][-1]),
                context_rows=int(window.sum()),
                samples=[snapshot(raw, side, signals, j) for j in np.flatnonzero(window)])
        contexts[policy] = events
    cid = 'straight_p030_G'; raw, side, signals = bundles[cid]; t = raw['time_s']
    traversal = (t >= 3.-1e-9) & (t < rows[cid]['declared_reference_arrival_s']-1e-9)
    eligible = np.flatnonzero(traversal)
    factor_events = {}
    for key in ('gov__authority_factor', 'gov__joint_reference_factor', 'gov__command_factor', 'gov__clock_rate', 'gov__raw_target_rate'):
        idx = int(eligible[np.nanargmin(side[key][eligible])])
        factor_events[key+'_minimum'] = snapshot(raw, side, signals, idx)
    active = eligible[side['gov__clock_rate'][eligible] < 1.-1e-12]
    factor_events['first_actual_slowdown'] = snapshot(raw, side, signals, int(active[0])) if len(active) else None
    report = dict(iteration='I-78', no_new_physics=True, canonical_unchanged=True,
        independent_implementation='No metrics/evaluator imports or saved signals; explicit quaternion axes and finite-line cases.',
        scalar_checks=checks, all_scalar_checks_passed=all(c['passed'] for c in checks),
        reconstructed_gates=gates, gate_boolean_agreement=gate_agreement,
        all_gate_booleans_match=all(c['passed'] for c in gate_agreement),
        exactly_failed_gates=[g for g in gates if not g['passed']],
        independent_rows=rows, p030_context=contexts, p030_trigger_context=factor_events,
        causal_qualification='Posthoc observational contexts. Retiming changes the physical state history, own raw attitude target and future contact evolution. These synchronized logs alone do not isolate which mechanism caused the +0.3 peak regressions.')
    write(out/'report.json', report)
    lines = ['# I-78 independent audit and +0.3 peak contexts', '',
        f"Independent scalar leaves: {len(checks)}; all agree with unchanged canonical report: {report['all_scalar_checks_passed']}.",
        f"Reconstructed performance/goal/stall checks: {len(gates)}; all decisions agree: {report['all_gate_booleans_match']}.", '',
        'This is a posthoc audit of the existing six episodes, not additional simulations or a repository test suite.', '',
        '| +0.3 policy/event | Time (s) | Virtual time (s) | Clock rate | Tilt (deg) | Speed (m/s) | Own-target up error (deg) | Ref speed (m/s) |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for policy, events in contexts.items():
        for name, event in events.items():
            p = event['peak']
            lines.append(f"| {policy} {name} | {p['physical_time_s']:.3f} | {p['gov__virtual_time_s']:.6f} | {p['gov__clock_rate']:.6f} | {p['absolute_tilt_deg']:.6f} | {p['speed_m_s']:.6f} | {p['own_target_up_error_deg']:.6f} | {p['reference_speed_m_s']:.6f} |")
    lines += ['', 'Failed frozen checks:']
    for g in report['exactly_failed_gates']:
        lines.append(f"- {g['name']}: {g['actual']:.12g}; limit {g['limit']:.12g}; native {g['native']:.12g}.")
    lines += ['', report['causal_qualification'], '']
    (out/'summary.md').write_text('\n'.join(lines))
    for p, binding in bindings.items():
        if sha(ROOT/p) != binding['sha256']:
            raise AssertionError('Source/input changed: '+p)
    write(out/'artifact_manifest.json', dict(files={str(p.relative_to(out)):dict(bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(out.rglob('*')) if p.is_file()}))
    print(json.dumps(dict(scalar_leaves=len(checks), scalar_audit_pass=report['all_scalar_checks_passed'], gate_checks=len(gates), gate_agreement=report['all_gate_booleans_match'], failed_performance_gates=report['exactly_failed_gates'])), flush=True)
    if not report['all_scalar_checks_passed'] or not report['all_gate_booleans_match']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
