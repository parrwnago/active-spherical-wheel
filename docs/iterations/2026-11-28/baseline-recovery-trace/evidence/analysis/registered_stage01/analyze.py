"""I86 registered unchanged-baseline recovery diagnostic.

Register source/rules first. Execute only after root confirms the new physical
output seal. No simulator/controller is instantiated by this analysis.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from core import (ROOT, LEGACY_PATH, legacy, model_properties, reduce_trace,
    interval_arrays, window_balance, window_indices, vector_stats)

HERE = Path(__file__).resolve().parent
RECOVERY_CORE = ROOT/'results/recovery_assessment_20261128/evaluation/core.py'
RECOVERY_DEFS = ROOT/'results/recovery_assessment_20261128/protocol/definitions.json'
MODEL = ROOT/'results/tangent_heading_routes_20261127/campaign/_workspaces/eight_m100_N/mirror/model.xml'
BLOCKS = [('XY_position', 0, 2), ('height', 2, 3), ('roll_pitch_attitude', 3, 5),
    ('heading_position', 5, 6), ('derived_rolling_shaft_error', 6, 7),
    ('cyclic_shell_position', 7, 9), ('body_linear_velocity', 9, 12),
    ('body_angular_velocity', 12, 15), ('shaft_velocity', 15, 16), ('shell_velocities', 16, 18)]
COMMANDS = ['exact_equilibrium_command', 'exact_rolling_feedforward', 'exact_support_command',
    'exact_lqr_command', 'exact_transition_command_delta', 'exact_heading_command',
    'exact_fade_command_delta', 'exact_clipping_command_delta']


def binding(path):
    p = Path(path)
    return dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())


def save(path, value):
    with Path(path).open('x') as f:
        json.dump(legacy.clean(value), f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')


def arrays(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() for k in z.files}


def source_bindings():
    paths = [Path(__file__), HERE/'core.py', HERE/'test_core.py', HERE/'definitions.json',
        LEGACY_PATH, RECOVERY_CORE, RECOVERY_DEFS]
    return {str(p.relative_to(ROOT)): binding(p) for p in paths}


def register():
    out = HERE/'registered_stage01'; out.mkdir(exist_ok=False)
    save(out/'registration.json', dict(iteration='I-86', utc=datetime.now(timezone.utc).isoformat(),
        source_bindings=source_bindings(), model_input={str(MODEL.relative_to(ROOT)): binding(MODEL)},
        reductions=json.loads((HERE/'definitions.json').read_text()),
        known_outcomes='I83/I85 old baseline event already known; all fixed new1ms reductions prospective.',
        no_new_array_reads=True,
        planned_new_inputs=['closed I86 substeps.npz', 'same-case child precision.npz',
            'parity_receipt.json', 'root physical output seal'],
        change_policy='Source/rule changes require a new preserved stage. No outcome-dependent tuning.'))
    for path in [Path(__file__), HERE/'core.py', HERE/'test_core.py', HERE/'definitions.json']:
        shutil.copy2(path, out/path.name)
    print(str(out/'registration.json'))


def check_registration():
    p = HERE/'registered_stage01/registration.json'
    registration = json.loads(p.read_text())
    if registration['source_bindings'] != source_bindings():
        raise AssertionError('Registered primary analysis source changed')
    if registration['model_input'] != {str(MODEL.relative_to(ROOT)): binding(MODEL)}:
        raise AssertionError('Registered analytic COM model changed')
    return p


def control_reduction(raw, definitions):
    t = raw['time_s']; lo, hi = definitions['trace_window_s']
    ids = np.flatnonzero((t >= lo-1e-9) & (t <= hi+1e-9))
    r = {key: raw[key][ids] for key in ['time_s', 'exact_qpos', 'exact_qvel',
        'exact_reference_quat', 'exact_reference_xy', 'exact_reference_velocity_m_s',
        'exact_estimated_normal', 'exact_estimated_support_loads_N', 'exact_feedback_reference_qvel',
        'exact_requested', 'exact_applied', 'exact_input_bounds', 'exact_previous_applied',
        'exact_feedback_gain', 'exact_error', *COMMANDS]}
    spec = importlib.util.spec_from_file_location('i86_registered_recovery', RECOVERY_CORE)
    recovery = importlib.util.module_from_spec(spec); spec.loader.exec_module(recovery)
    s = recovery.signals(r, json.loads(RECOVERY_DEFS.read_text()))
    blocks = {name: -np.einsum('nij,nj->ni', r['exact_feedback_gain'][:, :, lo:hi], r['exact_error'][:, lo:hi])
        for name, lo, hi in BLOCKS}
    ledger = sum(r[key] for key in COMMANDS[:-1])
    checks = dict(requested_ledger_max_error=float(np.max(abs(ledger-r['exact_requested']))),
        applied_ledger_max_error=float(np.max(abs(ledger+r[COMMANDS[-1]]-r['exact_applied']))),
        lqr_blocks_max_error=float(np.max(abs(sum(blocks.values())-r['exact_lqr_command']))),
        box_clipping_identity=bool(np.array_equal(np.clip(r['exact_requested'], -r['exact_input_bounds'],
            r['exact_input_bounds']), r['exact_applied'])))
    if max(checks[k] for k in ['requested_ledger_max_error', 'applied_ledger_max_error', 'lqr_blocks_max_error']) > 1e-7:
        raise AssertionError('Exact command reconstruction fails')
    if not checks['box_clipping_identity']:
        raise AssertionError('Actual command clipping identity fails')
    up = np.asarray([legacy.quat_matrix(q)[:, 2] for q in r['exact_reference_quat']])
    r['reference_up'] = up
    r['reference_up_rate_deg_s'] = np.r_[np.nan,
        np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i', up[1:], up[:-1]), -1., 1.)))/.005]
    return r, s, blocks, checks


def plot_report(out, sub, intervals, control, signals, metadata, definitions):
    lo, hi = definitions['focus_window_s']
    t = sub['time_s']; tc = control['time_s']
    fig, axes = plt.subplots(6, 1, figsize=(12, 13), sharex=True)
    axes[0].plot(tc, 1000*signals['spatial_velocity_error_m_s'], label='Root spatial velocity error')
    axes[0].axhline(20, color='k', ls='--', label='Unchanged recovery velocity band')
    axes[0].set_ylabel('Velocity error\n(mm/s)'); axes[0].legend(fontsize=8)
    for j, name in enumerate(['World X', 'World Y']):
        axes[1].plot(t, sub['root_velocity_after'][:, j], label='Root '+name)
        axes[1].plot(t, sub['COM_velocity_after'][:, j], ls=':', label='Whole COM '+name)
    axes[1].set_ylabel('Velocity\n(m/s)'); axes[1].legend(ncol=2, fontsize=8)
    axes[2].plot(t, sub['force_all_fixed_basis'][:, 0], label='Ground contact, fixed downhill')
    gravity_down = metadata['mass_kg']*np.asarray(metadata['gravity_m_s2'])@np.asarray(metadata['fixed_basis_rows_down_cross_normal'])[0]
    axes[2].axhline(gravity_down, color='gray', ls=':', label='Gravity, fixed downhill')
    axes[2].plot(t, sub['COM_required_force_fixed_basis'][:, 0], alpha=.7, label='Measured COM momentum rate')
    axes[2].set_ylabel('Force (N)'); axes[2].legend(ncol=2, fontsize=8)
    for j, name in enumerate(legacy.FAMILIES):
        axes[3].plot(t, sub['pressure_family'][:, j], label=name+' actual1ms')
    axes[3].plot(tc, control['exact_estimated_support_loads_N'][:, 0], ls='--', color='k', label='Inferred shell A')
    axes[3].set_ylabel('Normal load\n(N)'); axes[3].legend(ncol=3, fontsize=7)
    axes[4].plot(tc, signals['absolute_tilt_deg'], label='Actual world tilt')
    axes[4].plot(tc, np.degrees(np.arccos(np.clip(control['reference_up'][:, 2], -1., 1.))), label='Desired world tilt')
    axes[4].plot(tc, signals['own_up_error_deg'], label='Native desired-up error')
    axes[4].set_ylabel('Angle (deg)'); axes[4].legend(ncol=3, fontsize=8)
    for j, name in enumerate(['Shaft', 'Shell A', 'Shell B', 'Belt A', 'Belt B']):
        axes[5].plot(tc, control['exact_applied'][:, j]/control['exact_input_bounds'][:, j], label=name)
    axes[5].axhline(1, ls='--', color='gray'); axes[5].axhline(-1, ls='--', color='gray')
    axes[5].set_ylabel('Applied / limit'); axes[5].legend(ncol=3, fontsize=8)
    axes[-1].set_xlabel('Physical time (s); commands shown act AFTER their state rows')
    for ax in axes:
        ax.grid(alpha=.25); ax.set_xlim(lo, hi)
        ax.axvspan(105.52, 105.53, color='orange', alpha=.15)
        ax.axvline(105.59, color='gray', ls='--', lw=.8)
    fig.suptitle('I86 native baseline recovery:1ms contact forces and held commands\nOrange: fixed105.520–105.530s probe; unchanged observer/LQR/feedforward,0.60m/s caps')
    fig.tight_layout(rect=(0, 0, 1, .95)); fig.savefig(out/'recovery_trace.png', dpi=150)
    with PdfPages(out/'recovery_trace.pdf') as pdf:
        pdf.savefig(fig)
    plt.close(fig)


def analyze(args):
    registration = check_registration()
    run, precision_path, seal = args.run.resolve(), args.precision.resolve(), args.seal.resolve()
    if not seal.is_file():
        raise AssertionError('Root closed physical output seal required before arrays')
    inputs = [run/'substeps.npz', precision_path, run/'parity_receipt.json', seal, MODEL]
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    save(out/'registered_before_arrays.json', dict(iteration='I-86', utc=datetime.now(timezone.utc).isoformat(),
        preregistration=str(registration.relative_to(ROOT)), preregistration_binding=binding(registration),
        sources=source_bindings(), inputs={str(p.relative_to(ROOT)): binding(p) for p in inputs},
        scope='No source/rule/threshold tuning. Arrays are opened only after this receipt.'))
    parity = json.loads((run/'parity_receipt.json').read_text())
    if not parity['passed']:
        raise AssertionError('Unchanged baseline prefix parity required')
    trace, precision = arrays(run/'substeps.npz'), arrays(precision_path)
    definitions = json.loads((HERE/'definitions.json').read_text())
    properties, gravity = model_properties(MODEL)
    sub, metadata = reduce_trace(trace, precision, properties, gravity, definitions)
    intervals = interval_arrays(sub, trace, precision, definitions)
    control, signals, blocks, ledger_checks = control_reduction(precision, definitions)
    focus_sub = window_indices(sub['time_s'], *definitions['focus_window_s'])
    focus_int = window_indices(intervals['time_s'], *definitions['focus_window_s'])
    tc = control['time_s']; focus_control = np.flatnonzero((tc >= definitions['focus_window_s'][0]-1e-9) & (tc <= definitions['focus_window_s'][1]+1e-9))
    closure = {}
    for key in ['all', 'admitted', 'native']:
        closure[key] = dict(substeps=vector_stats(sub['residual_'+key][focus_sub]),
            completed5ms=vector_stats(intervals['residual_'+key][focus_int]))
    snapshots = []
    for target in definitions['snapshot_times_s']:
        i = int(np.argmin(abs(tc-target))); absolute_row = int(round(tc[i]/.005))
        h = absolute_row-1
        j = int(np.argmin(abs(intervals['time_s']-tc[i])))
        snapshots.append(dict(time_s=float(tc[i]),
            state_before_new_command=dict(root_velocity_m_s=control['exact_qvel'][i, :3],
                spatial_velocity_error_m_s=signals['spatial_velocity_error_m_s'][i],
                path_distance_m=signals['intended_distance_m'][i], own_up_error_deg=signals['own_up_error_deg'][i],
                normal=control['exact_estimated_normal'][i], desired_up=control['reference_up'][i]),
            preceding_hold=dict(start_s=float(precision['time_s'][h]), applied=precision['exact_applied'][h],
                command_blocks={key: precision[key][h] for key in COMMANDS},
                contact_force_N=intervals['force_all'][j], net_force_N=intervals['net_all'][j],
                COM_required_force_N=intervals['COM_required_force'][j], actual_mean_loads_N=intervals['pressure_family'][j]),
            new_command_acts_after_state=dict(applied=control['exact_applied'][i],
                command_blocks={key: control[key][i] for key in COMMANDS},
                lqr_error_blocks={key: value[i] for key, value in blocks.items()},
                supplied_angular_velocity_reference_rad_s=control['exact_feedback_reference_qvel'][i, 3:6])))
    changes = np.linalg.norm(intervals['root_velocity_after']-intervals['root_velocity_before'], axis=1)
    j = focus_int[int(np.argmax(changes[focus_int]))]
    maxima = dict(largest5ms_root_velocity_change=dict(interval_start_s=float(intervals['time_s'][j]-.005),
        interval_end_s=float(intervals['time_s'][j]), delta_m_s=intervals['root_velocity_after'][j]-intervals['root_velocity_before'][j],
        COM_delta_m_s=intervals['COM_velocity_after'][j]-intervals['COM_velocity_before'][j],
        held_input=intervals['held_input'][j], contact_force_N=intervals['force_all'][j],
        net_force_N=intervals['net_all'][j], COM_required_force_N=intervals['COM_required_force'][j]))
    report = dict(iteration='I-86', status='unchanged diagnostic completed; no controller intervention',
        metadata=metadata, baseline_parity=parity, command_ledger=ledger_checks,
        fixed_window_sampled_momentum_balance=window_balance(sub, definitions),
        focus_force_closure=closure, maxima=maxima,
        focus_motion=dict(path_distance_max_m=float(signals['intended_distance_m'][focus_control].max()),
            spatial_velocity_error_max_m_s=float(signals['spatial_velocity_error_m_s'][focus_control].max()),
            native_up_error_max_deg=float(signals['own_up_error_deg'][focus_control].max()),
            absolute_tilt_max_deg=float(signals['absolute_tilt_deg'][focus_control].max()),
            any_requested_clipping_rows=int(np.sum(np.any(abs(control['exact_requested'][focus_control]) > control['exact_input_bounds'][focus_control]+1e-12, axis=1)))),
        observer=dict(interval_force_error=vector_stats(intervals['observer_force_error'][focus_int]),
            interval_moment_error=vector_stats(intervals['observer_moment_error'][focus_int]),
            filtered_force_error=vector_stats(intervals['observer_filter_force_error'][focus_int]),
            same_filter_normal_error_deg=legacy.stats(intervals['observer_filter_normal_error_deg'][focus_int]),
            controller_vs_interval_pressure_normal_deg=legacy.stats(intervals['controller_pressure_normal_error_deg'][focus_int])),
        snapshots=snapshots, limitations=definitions['causal_limit'],
        impulse_qualification=definitions['force_accounting'], no_full_route_recovery_claim=True)
    save(out/'report.json', report)
    np.savez_compressed(out/'substep_signals.npz', **sub)
    np.savez_compressed(out/'interval_signals.npz', **intervals)
    np.savez_compressed(out/'control_signals.npz', **control,
        **{'derived__'+k:v for k,v in signals.items()}, **{'lqr_block__'+k:v for k,v in blocks.items()})
    plot_report(out, sub, intervals, control, signals, metadata, definitions)
    check_registration()
    save(out/'artifact_manifest.json', dict(files={p.name: binding(p) for p in sorted(out.iterdir()) if p.is_file()}))
    print(json.dumps(legacy.clean(dict(output=str(out), closure=closure,
        focus=report['focus_motion'], maxima=maxima, windows=report['fixed_window_sampled_momentum_balance'])), indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--register-only', action='store_true')
    ap.add_argument('--run', type=Path)
    ap.add_argument('--precision', type=Path)
    ap.add_argument('--seal', type=Path)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    if args.register_only:
        register(); return
    if any(getattr(args, key) is None for key in ['run', 'precision', 'seal', 'output']):
        ap.error('Analysis requires --run --precision --seal --output')
    analyze(args)


if __name__ == '__main__':
    main()
