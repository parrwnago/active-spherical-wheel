"""Separately registered post-run comparison of independent I86 reductions."""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT/'results/baseline_recovery_trace_20261128'
WINDOWS = dict(pre=[105.4, 105.52], kick=[105.52, 105.53], response=[105.53, 106.],
               focus=[105.4, 106.1], whole_trace=[105., 106.3])
TOL = 1e-9


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def save(p, x):
    with Path(p).open('x') as f:
        json.dump(x, f, indent=2, sort_keys=True, allow_nan=False); f.write('\n')


def arrays(p):
    with np.load(p, allow_pickle=False) as z: return {k: z[k].copy() for k in z.files}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(); out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    primary = BASE/'analysis/completed_stage01'
    independent = BASE/'verification/completed_stage01'
    raw = BASE/'campaign/native106p5_stage01/substeps.npz'
    paths = [Path(__file__).resolve(), primary/'report.json', primary/'substep_signals.npz',
             primary/'interval_signals.npz', independent/'report.json', independent/'independent_signals.npz', raw]
    bindings = {str(p.relative_to(ROOT)): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in paths}
    shutil.copyfile(Path(__file__), out/'compare_primary.py')
    save(out/'registered_before_primary_array_reads.json', dict(
        iteration='I-86', utc=datetime.now(timezone.utc).isoformat(), source_and_inputs=bindings,
        scope='Post-run separately registered comparison, not new physics or acceptance tuning.',
        tolerance_absolute=TOL, windows_s=WINDOWS,
        residual_convention='Primary net-minus-required; independent required-minus-net, so negate for comparison.',
        planned='Compare independent recorded-state MuJoCo COM against primary analytic COM; all force/pressure/held/time arrays; fixed-window impulse and signed root/COM velocity decomposition.'))
    a = arrays(primary/'substep_signals.npz'); h = arrays(primary/'interval_signals.npz')
    b = arrays(independent/'independent_signals.npz'); raw_arrays = arrays(raw)
    pr = json.loads((primary/'report.json').read_text()); ir = json.loads((independent/'report.json').read_text())
    comparisons = []
    def check(label, left, right):
        left, right = np.asarray(left), np.asarray(right)
        assert left.shape == right.shape, (label, left.shape, right.shape)
        error = float(np.max(np.abs(left-right))) if left.size else 0.
        comparisons.append(dict(label=label, shape=list(left.shape), max_absolute_error=error))
        assert error <= TOL, (label, error)
    for pa, ib in [('time_s', 'time_s'), ('force_all', 'contact_force_N'),
                   ('force_admitted', 'admitted_contact_force_N'), ('force_native', 'native_contact_force_N'),
                   ('COM_required_force', 'COM_required_net_force_N'), ('net_all', 'sampled_net_force_N'),
                   ('COM_position_before', 'COM_pre_position_m'), ('COM_position_after', 'COM_post_position_m'),
                   ('COM_velocity_before', 'COM_pre_velocity_m_s'), ('COM_velocity_after', 'COM_post_velocity_m_s'),
                   ('pressure_family', 'pressure_by_family_N')]:
        check('substep:'+pa, a[pa], b[ib])
    check('substep:residual_all', a['residual_all'], -b['force_balance_residual_N'])
    rows = ir['rows']
    for pa, ib in [('time_s', 'time_s'), ('force_all', 'contact_force_N'),
                   ('force_admitted', 'admitted_contact_force_N'), ('force_native', 'native_contact_force_N'),
                   ('COM_required_force', 'COM_momentum_rate_N'), ('pressure_family', 'family_mean_pressure_N'),
                   ('last_pressure_family', 'family_last_pressure_N'), ('held_input', 'held_input')]:
        check('interval:'+pa, h[pa], [x[ib] for x in rows])
    check('interval:residual_all', h['residual_all'], -np.array([x['force_balance_residual_N'] for x in rows]))
    check('metadata:mass_kg', pr['metadata']['mass_kg'], ir['total_mass_kg'])
    checks_by_window = {}
    for name, (lo, hi) in WINDOWS.items():
        ids = np.flatnonzero((b['time_s'] > lo+1e-9)&(b['time_s'] <= hi+1e-9))
        dv = b['COM_post_velocity_m_s'][ids[-1]]-b['COM_pre_velocity_m_s'][ids[0]]
        rootdv = raw_arrays['bridge_state'][ids[-1], 9:12]-raw_arrays['bridge_state_before'][ids[0], 9:12]
        row = pr['fixed_window_sampled_momentum_balance'][name]
        own = dict(root_velocity_change_m_s=rootdv,
                   COM_velocity_change_m_s=dv,
                   root_relative_to_COM_velocity_change_contribution_m_s=rootdv-dv,
                   contact_sampled_impulse_Ns=b['contact_force_N'][ids].sum(axis=0)*.001,
                   net_sampled_impulse_Ns=b['sampled_net_force_N'][ids].sum(axis=0)*.001,
                   actual_COM_momentum_change_Ns=dv*ir['total_mass_kg'])
        for key, val in own.items(): check(name+':'+key, row[key], val)
        checks_by_window[name] = {k: v.tolist() for k, v in own.items()}
    for rel, binding in bindings.items():
        assert sha(ROOT/rel) == binding['sha256'] and (ROOT/rel).stat().st_size == binding['bytes']
    save(out/'report.json', dict(iteration='I-86', passed=True, no_new_physics=True,
        array_and_vector_comparisons=len(comparisons), compared_scalar_values=sum(int(np.prod(x['shape'])) if x['shape'] else 1 for x in comparisons),
        largest_absolute_error=max(x['max_absolute_error'] for x in comparisons), comparisons=comparisons,
        independent_fixed_window_ledger=checks_by_window, residual_sign_convention_explicit=True,
        inputs_and_sources_unchanged=True,
        qualification='Agreement is an analysis consistency check on one physical replay, not a separate trial or proof of controller causality.'))
    save(out/'artifact_manifest.json', dict(files={p.name: dict(bytes=p.stat().st_size, sha256=sha(p)) for p in out.iterdir() if p.is_file()}))
    print(json.dumps(dict(passed=True, comparisons=len(comparisons), largest_absolute_error=max(x['max_absolute_error'] for x in comparisons))))


if __name__ == '__main__': main()
