"""Audit preserved post-pulse MPC rest targets before another closed-loop trial."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from spherical_wheel.mirror import StateMirror
from spherical_wheel.model import REPO_ROOT
from spherical_wheel.predictive_equilibrium import EquilibriumSettings, solve_goal_equilibrium
from spherical_wheel.predictive_model import EstimatedPlanePredictor


OUTPUT = REPO_ROOT/'results/mpc_equilibrium_20261125/audit'
SOURCE = REPO_ROOT/'results/mpc_balance_20261125/trajectory/runs'


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    settings = EquilibriumSettings()
    paths = [Path(__file__), REPO_ROOT/'src/spherical_wheel/predictive_equilibrium.py',
             REPO_ROOT/'src/spherical_wheel/predictive_model.py']
    snapshot = OUTPUT/'declared_source'
    snapshot.mkdir(exist_ok=True)
    files = []
    for path in paths:
        data = path.read_bytes();target = snapshot/(path.name+'.txt')
        if target.exists():
            raise FileExistsError('Preserve completed audit source; choose a fresh stage before rerunning')
        target.write_bytes(data)
        files.append({'path': str(path.relative_to(REPO_ROOT)), 'snapshot': target.name,
                      'sha256': hashlib.sha256(data).hexdigest()})
    (snapshot/'manifest.json').write_text(json.dumps({'stage': 'Captured before five-anchor equilibrium audit',
        'settings': asdict(settings), 'files': files}, indent=2)+'\n')
    mirror = StateMirror(model_dir=OUTPUT/'goal_mirror')
    predictor = EstimatedPlanePredictor(mirror)
    rows = []
    for direction, anchors in [('world_y', (1.4, 2., 3., 5.)), ('world_x', (2.,))]:
        folder = SOURCE/direction/'mpc'
        plans = json.loads((folder/'plans.json').read_text())
        with np.load(folder/'precision.npz') as data:
            exact = {key: data[key] for key in data}
        with np.load(folder/'rollout.npz') as data:
            log = {key: data[key] for key in data}
        for at in anchors:
            plan = min(plans, key=lambda row: abs(row['time_s']-at))
            index = int(np.argmin(abs(exact['time_s']-plan['time_s'])))
            q0, v0 = np.array(plan['qpos']), np.array(plan['qvel'])
            previous = exact['exact_applied'][index-1]
            mirror.set_state(q0, v0, plan['time_s'], previous)
            calibration = predictor.update(q0, v0, plan['normal'], plan['point_m'], plan['load_N'],
                friction=float(log['estimated_friction'][index]), command=previous)
            reference, reference_velocity = predictor.perturb(q0, v0, np.array(plan['target_states'][-1]))
            np.testing.assert_allclose(reference_velocity, 0., atol=1e-14)
            result = solve_goal_equilibrium(predictor, reference, settings=settings)
            row = {'direction': direction, 'anchor_time_s': plan['time_s'],
                   'estimated_plane_calibration': calibration, 'analytic_reference_qpos': reference.tolist(),
                   'qpos': result.qpos.tolist(), 'qvel': result.qvel.tolist(), 'inputs': result.inputs.tolist(),
                   'verified': result.verified, 'diagnostics': result.diagnostics,
                   'input_hashes': {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                   for path in [folder/'plans.json', folder/'precision.npz', folder/'rollout.npz']}}
            rows.append(row)
            print(json.dumps({'direction': direction, 'time_s': plan['time_s'], 'verified': result.verified,
                'selected': result.diagnostics['selected_label'],
                'hold': result.diagnostics['selected_audit']['hold_peak_metrics'],
                'max_qacc': result.diagnostics['selected_audit']['max_acceleration_residual'],
                'failed_gates': [key for key, value in result.diagnostics['selected_audit']['hold_gates'].items() if not value]}), flush=True)
            (OUTPUT/'report.json').write_text(json.dumps({'status': 'audit_in_progress', 'settings': asdict(settings),
                'scope': 'Frozen estimated-plane model audits only. No new Gazebo run, live reset, new holding constraint or terminal controller.',
                'rows': rows}, indent=2)+'\n')
    report = {'status': 'verified_equilibrium_available' if any(row['verified'] for row in rows) else 'no_verified_equilibrium_found',
              'scope': 'Five declared frozen estimated-plane audits. Failure of local searches is not proof of physical goal infeasibility.',
              'settings': asdict(settings), 'rows': rows,
              'verified_count': sum(row['verified'] for row in rows),
              'terminal_derivation': 'Skipped unless a stationary forward-hold target passes verification.'}
    (OUTPUT/'report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
