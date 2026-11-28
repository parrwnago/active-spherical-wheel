"""I77 desired-up source attribution: saved measurements, no dynamics integration."""
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import patch
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
OLD = ROOT/'results/tangent_heading_routes_20261127'
OUT = HERE/'reference_source_stage02'
WINDOW = (96., 97.5)
SNAPSHOTS = (96., 96.035, 96.15, 96.16, 96.355, 96.4, 96.425, 96.5, 96.8, 97., 97.5)


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def clean(v):
    if isinstance(v, dict): return {str(k): clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return [clean(x) for x in v]
    if isinstance(v, np.ndarray): return clean(v.tolist())
    if isinstance(v, np.generic): return clean(v.item())
    if isinstance(v, float) and not np.isfinite(v): return None
    return v


def write(p, v):
    with p.open('x') as f:
        json.dump(clean(v), f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')


def up(q):
    w, x, y, z = q
    return np.array([2*(x*z+w*y), 2*(y*z-w*x), 1.-2*(x*x+y*y)])


def angle(a, b):
    return np.degrees(np.arctan2(np.linalg.norm(np.cross(a, b), axis=-1),
                                np.sum(a*b, axis=-1)))


def main():
    protocol_path = OLD/'protocol/protocol.json'
    seal_path = OLD/'campaign/outputs_sealed.json'
    run = OLD/'campaign/runs/eight_m100_N'
    precision = run/'precision.npz'
    summary_path = run/'summary.json'
    config_path = ROOT/'config/model.yaml'
    model_path = OLD/'campaign/_workspaces/eight_m100_N/mirror/model.xml'
    if sha(protocol_path) != '53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2':
        raise ValueError('Original protocol changed')
    if sha(seal_path) != 'e5e65236947d7b1945c90b3a74a133f055495851d1938b42dacd0ac26602fc2a':
        raise ValueError('Original release changed')
    protocol = json.loads(protocol_path.read_text())
    for group in ('source_sha256', 'input_sha256'):
        for rel, expected in protocol[group].items():
            if sha(ROOT/rel) != expected: raise ValueError('Frozen source/input changed: '+rel)
    sealed = json.loads(seal_path.read_text())['files']
    for p in (precision, summary_path):
        b = sealed[str(p.relative_to(OLD))]
        if p.stat().st_size != b['bytes'] or sha(p) != b['sha256']:
            raise ValueError('Original output changed: '+str(p))
    model_tree = ET.parse(model_path)
    assets = [model_path.parent/n.attrib['file'] for n in model_tree.iter() if 'file' in n.attrib]
    sources = sorted((ROOT/'src/spherical_wheel').rglob('*.py'))+[Path(__file__).resolve(), config_path]
    inputs = [protocol_path, seal_path, precision, summary_path, model_path,
              HERE/'reference_source_stage01/registered_before_array_reads.json',
              HERE/'reference_source_stage01/setup_failure.json',
              HERE/'reference_source_stage01/source'/Path(__file__).resolve().relative_to(ROOT)]+assets
    bindings = {str(p.relative_to(ROOT)): dict(bytes=p.stat().st_size, sha256=sha(p))
                for p in sorted(set(sources+inputs))}
    OUT.mkdir(parents=True, exist_ok=False)
    for p in sources:
        dst = OUT/'source'/p.relative_to(ROOT)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dst)
    write(OUT/'registered_before_array_reads.json', dict(
        iteration='I-77', utc=datetime.now(timezone.utc).isoformat(), files=bindings,
        window_s=WINDOW, fixed_snapshots_s=SNAPSHOTS, no_new_physics=True,
        no_controller_command=True, no_parameter_changes=True,
        reconstruction='Bind unchanged OffroadController.lean_vector/reference_attitude to saved parameters and known model mass/COM. Recover constant equilibrium quaternion by algebraically removing the known first-row lean quaternion from the first saved raw reference. Require every window raw quaternion reconstructed within1e-12.',
        prior_setup_failure='reference_source_stage01 guard correctly rejected a direct equilibrium anchor because the first estimated normal is not vertical. That sealed stage is preserved without an attribution result.',
        comparisons=['Hold estimated normal at96s while retaining actual reference acceleration.',
                     'Hold reference acceleration at96s while retaining actual estimated normal.',
                     'Per-step exact uncapped lean increments from acceleration and normal; report cap activity.',
                     'Sequential desired-up vector split with normal change evaluated at current acceleration; vectors add, angular magnitudes do not.'],
        limitations=['Saved-state algebra, not executed closed-loop counterfactuals or independent physical causal effects.',
                     'Force-derived normal can represent contact geometry and tangential wrench; this attribution does not establish normal correctness.',
                     'Only desired-up motion is considered; native angular-rate target remains zero.']))
    sys.path.insert(0, str(ROOT/'src'))
    import mujoco
    from spherical_wheel.model import load_config
    from spherical_wheel.offroad_control import OffroadController, OffroadParams
    config = load_config(config_path)
    params = OffroadParams(**json.loads(summary_path.read_text())['parameters'])
    forbidden_calls = []
    def forbidden(*a, **kw):
        forbidden_calls.append(True)
        raise AssertionError('Dynamics/contact API prohibited')
    with ExitStack() as guards:
        for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_collision', 'mj_contactForce'):
            guards.enter_context(patch.object(mujoco, name, forbidden))
        model = mujoco.MjModel.from_xml_path(str(model_path))
        fork = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'fork')
        factor = float(model.body_subtreemass[fork])*float(config['geometry']['sphere_radius_m'])/(float(model.body_mass[fork])*float(model.body_ipos[fork, 2]))
        with np.load(precision, allow_pickle=False) as z:
            data = {key: z[key].copy() for key in ('time_s', 'exact_estimated_normal',
                'exact_reference_acceleration_m_s2', 'exact_reference_quat', 'exact_reference_qvel')}
        qeq = np.zeros(model.nq); qeq[3] = 1.
        carrier = SimpleNamespace(params=params, gravity=abs(float(config['simulation']['gravity_m_s2'][2])),
            lean_factor=factor, ordinary={'equilibrium': {'qpos': qeq}})
        carrier.lean_vector = MethodType(OffroadController.lean_vector, carrier)
        carrier.reference_attitude = MethodType(OffroadController.reference_attitude, carrier)
        first_tilt = carrier.reference_attitude(data['exact_estimated_normal'][0], data['exact_reference_acceleration_m_s2'][0])
        inverse_first_tilt = first_tilt*np.array([1., -1., -1., -1.])
        mujoco.mju_mulQuat(qeq[3:7], data['exact_reference_quat'][0], inverse_first_tilt)
        ids = np.flatnonzero((data['time_s'] >= WINDOW[0]-1e-9)&(data['time_s'] <= WINDOW[1]+1e-9))
        assert len(ids) == 301
        # Include the preceding row so the96s first-row rate has native timing.
        ext = np.r_[ids[0]-1, ids]
        t = data['time_s'][ext]
        np.testing.assert_allclose(np.diff(t), .005, atol=1e-10, rtol=0.)
        n = data['exact_estimated_normal'][ext]
        a = data['exact_reference_acceleration_m_s2'][ext]
        g = carrier.gravity; scale = params.lean_gain*factor
        rawlean = a/g-scale*n[:, :2]
        lean = np.array([carrier.lean_vector(ni, ai) for ni, ai in zip(n, a)])
        quat = np.array([carrier.reference_attitude(ni, ai) for ni, ai in zip(n, a)])
        max_qerr = float(np.max(abs(quat-data['exact_reference_quat'][ext])))
        if max_qerr > 1e-12: raise AssertionError('Desired raw quaternion reconstruction differs')
        np.testing.assert_array_equal(data['exact_reference_qvel'][ids, 3:6], np.zeros((len(ids), 3)))
        actualup = np.array([up(q) for q in quat])
        n0, a0 = n[1], a[1]
        nheld = np.array([up(carrier.reference_attitude(n0, ai)) for ai in a[1:]])
        aheld = np.array([up(carrier.reference_attitude(ni, a0)) for ni in n[1:]])
        # Deterministic sequence: first change acceleration, then normal.
        middle = np.array([up(carrier.reference_attitude(ni, ai)) for ni, ai in zip(n[:-1], a[1:])])
        dnormal = actualup[1:]-middle
        dacc = middle-actualup[:-1]
        closure = float(np.max(abs(dnormal+dacc-np.diff(actualup, axis=0))))
        dlean_normal = -scale*np.diff(n[:, :2], axis=0)
        dlean_acc = np.diff(a, axis=0)/g
        leanclosure = float(np.max(abs(dlean_normal+dlean_acc-np.diff(rawlean, axis=0))))
        rates = angle(actualup[1:], actualup[:-1])/.005
        normalrates = angle(actualup[1:], middle)/.005
        accr = angle(middle, actualup[:-1])/.005
        cap = np.linalg.norm(rawlean[1:], axis=1) > np.sin(np.deg2rad(params.max_lean_deg))
        if closure > 1e-14 or leanclosure > 1e-14: raise AssertionError('Algebraic split failed')
    arrays = dict(time_s=t[1:], source_row=ids, estimated_normal=n[1:], reference_acceleration_m_s2=a[1:],
        raw_lean=rawlean[1:], capped_lean=lean[1:], cap_active=cap, desired_up=actualup[1:],
        normal_held96_desired_up=nheld, acceleration_held96_desired_up=aheld,
        desired_up_rate_deg_s=rates, sequential_normal_rate_magnitude_deg_s=normalrates,
        sequential_acceleration_rate_magnitude_deg_s=accr,
        desired_up_delta_from_normal=dnormal, desired_up_delta_from_acceleration=dacc,
        uncapped_lean_delta_from_normal=dlean_normal, uncapped_lean_delta_from_acceleration=dlean_acc)
    np.savez_compressed(OUT/'attribution.npz', **arrays)
    snapshots = []
    for time in SNAPSHOTS:
        i = int(np.argmin(abs(arrays['time_s']-time)))
        if abs(arrays['time_s'][i]-time) > 1e-9: raise AssertionError('Snapshot clock differs')
        snapshots.append({key: value[i] for key, value in arrays.items()})
    def extrema(v):
        i = int(np.argmax(v))
        return dict(max=float(v[i]), at_s=float(arrays['time_s'][i]), rms=float(np.sqrt(np.mean(v*v))))
    for rel, b in bindings.items():
        p = ROOT/rel
        if p.stat().st_size != b['bytes'] or sha(p) != b['sha256']: raise AssertionError('Source/input changed')
    report = dict(iteration='I-77', window_s=WINDOW, rows=len(ids), lean_factor=factor,
        gravity_m_s2=g, lean_gain=params.lean_gain, max_lean_deg=params.max_lean_deg,
        reconstructed_quaternion_max_abs_error=max_qerr, zero_angular_rate_reference_verified=True,
        forbidden_dynamics_contact_api_calls=len(forbidden_calls), source_inputs_unchanged=True,
        sequential_desired_up_vector_max_closure=closure, uncapped_lean_max_closure=leanclosure,
        cap_active_rows=int(cap.sum()), desired_up_rate_deg_s=extrema(rates),
        normal_change_rate_magnitude_deg_s=extrema(normalrates),
        acceleration_change_rate_magnitude_deg_s=extrema(accr),
        normal_held96_up_motion_deg=extrema(angle(nheld, np.broadcast_to(actualup[1], nheld.shape))),
        acceleration_held96_up_motion_deg=extrema(angle(aheld, np.broadcast_to(actualup[1], aheld.shape))),
        normal_held96_vs_actual_up_deg=extrema(angle(nheld, actualup[1:])),
        acceleration_held96_vs_actual_up_deg=extrema(angle(aheld, actualup[1:])),
        snapshots=snapshots, limitations=[
            'These are same-state reference algebra comparisons, not executed physical interventions.',
            'Sequential3D vector increments add exactly; angular magnitudes are not additive and depend on sequence.',
            'No conclusion is made here about correctness of contact normal or its physical cause.'])
    write(OUT/'report.json', report)
    (OUT/'findings.md').write_text(
        '# I-77 desired-up source attribution\n\n'
        f'Unchanged reference_attitude reconstructs all 301 rows from 96.0–97.5 s with maximum quaternion component error {max_qerr:.3g}. '
        f'No dynamics/contact API is called. Lean factor is {factor:.6g}; the lean cap is active on {int(cap.sum())} rows.\n\n'
        f'Maximum actual desired-up rate is {rates.max():.6g} deg/s. In the exact sequential split, '
        f'the normal-change angular magnitude peaks at {normalrates.max():.6g} deg/s, '
        f'while acceleration-change magnitude peaks at {accr.max():.6g} deg/s. '
        'Angular magnitudes do not add; the 3D vector increments do.\n\n'
        'Holding each source at 96 s is saved-state reference algebra, not a physical closed-loop intervention. '
        'This identifies what moves the target, not whether the estimated normal is correct or what physically caused its motion.\n')
    write(OUT/'artifact_manifest.json', dict(files={str(p.relative_to(OUT)): dict(bytes=p.stat().st_size, sha256=sha(p))
        for p in sorted(OUT.rglob('*')) if p.is_file()}))
    print(json.dumps(clean({k: report[k] for k in ('rows', 'lean_factor', 'reconstructed_quaternion_max_abs_error',
        'cap_active_rows', 'desired_up_rate_deg_s', 'normal_change_rate_magnitude_deg_s', 'acceleration_change_rate_magnitude_deg_s',
        'normal_held96_up_motion_deg', 'acceleration_held96_up_motion_deg')})), flush=True)


if __name__ == '__main__':
    main()
