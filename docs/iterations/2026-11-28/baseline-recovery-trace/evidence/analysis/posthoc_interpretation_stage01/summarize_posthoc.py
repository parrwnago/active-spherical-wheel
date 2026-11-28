"""I86 separate posthoc interpretation of already-completed primary signals.

No original rules/results are changed. This addendum selects the maximum
spatial velocity residual within the already registered focus window and
compares it with105.530s. Its question arose after the primary plot was seen.
"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import numpy as np
from core import legacy

HERE = Path(__file__).resolve().parent
PRIMARY = HERE/'completed_stage01'


def binding(path):
    b = path.read_bytes()
    return dict(bytes=len(b), sha256=hashlib.sha256(b).hexdigest())


def save(path, value):
    with path.open('x') as f:
        json.dump(legacy.clean(value), f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')


def main():
    out = HERE/'posthoc_interpretation_stage01'; out.mkdir(exist_ok=False)
    inputs = [PRIMARY/x for x in ['report.json', 'substep_signals.npz', 'interval_signals.npz', 'control_signals.npz']]
    save(out/'registered_before_addendum_arrays.json', dict(iteration='I-86 posthoc addendum',
        utc=datetime.now(timezone.utc).isoformat(), source=binding(Path(__file__)),
        inputs={str(p):binding(p) for p in inputs},
        known_outcomes='Primary analysis/plot and some descriptive snapshot queries already viewed. This is posthoc, not prospective physics registration.',
        selection='Maximum primary spatial velocity residual within fixed105.4–106.1focus; compare105.530 and105.400. Report signed root/COM/relative contributions and actual/desired-up rates. Initial1msdrop/reload rows105.524–.527 separately. No tuning or intervention.'))
    shutil.copy2(Path(__file__), out/Path(__file__).name)
    with np.load(PRIMARY/'control_signals.npz', allow_pickle=False) as z:
        control = {k:z[k].copy() for k in z.files}
    with np.load(PRIMARY/'substep_signals.npz', allow_pickle=False) as z:
        sub = {k:z[k].copy() for k in z.files}
    t = control['time_s']; ids = np.flatnonzero((t >= 105.4-1e-8) & (t <= 106.1+1e-8))
    peak = ids[np.argmax(control['derived__spatial_velocity_error_m_s'][ids])]
    anchor = int(np.argmin(abs(t-105.53)))
    points = {}
    for name, i in [('focus_start', int(np.argmin(abs(t-105.4)))), ('anchor', anchor), ('residual_peak', peak)]:
        j = int(np.argmin(abs(sub['time_s']-t[i])))
        np.testing.assert_allclose(sub['time_s'][j], t[i], rtol=0, atol=1e-8)
        root = control['exact_qvel'][i, :3]
        com = sub['COM_velocity_after'][j]
        quat = control['exact_qpos'][i, 3:7]
        rotation = legacy.quat_matrix(quat)
        omega = rotation@control['exact_qvel'][i, 3:6]
        up = rotation[:, 2]
        reference_up = control['reference_up'][i]
        desired_up_derivative = (reference_up-control['reference_up'][i-1])/.005
        desired_minimal_angular_rate = np.cross(reference_up, desired_up_derivative)
        actual_up_derivative = np.cross(omega, up)
        points[name] = dict(time_s=float(t[i]), root_velocity_m_s=root,
            COM_velocity_m_s=com, root_relative_to_COM_velocity_m_s=root-com,
            spatial_nominal_velocity_m_s=control['derived__spatial_nominal_velocity_m_s'][i],
            spatial_velocity_error_m_s=control['derived__spatial_velocity_error_m_s'][i],
            root_spatial_error_xy_m_s=root[:2]-control['derived__spatial_nominal_velocity_m_s'][i],
            COM_minus_same_nominal_velocity_xy_m_s=com[:2]-control['derived__spatial_nominal_velocity_m_s'][i],
            nominal_COM_qualification='Same root-path velocity is used only as a descriptive common subtraction, not as an independently specified desired whole-COM trajectory.',
            actual_world_angular_rate_rad_s=omega,
            actual_up_change_rate_deg_s=float(np.degrees(np.linalg.norm(actual_up_derivative))),
            desired_up_change_rate_deg_s=float(control['reference_up_rate_deg_s'][i]),
            desired_minimum_angular_rate_world_rad_s=desired_minimal_angular_rate,
            supplied_body_angular_reference_rad_s=control['exact_feedback_reference_qvel'][i, 3:6],
            actual_world_tilt_deg=float(control['derived__absolute_tilt_deg'][i]),
            desired_world_tilt_deg=float(np.degrees(np.arccos(np.clip(reference_up[2],-1.,1.)))),
            native_up_error_deg=float(control['derived__own_up_error_deg'][i]),
            normal=control['exact_estimated_normal'][i],
            command_components={k:control[k][i] for k in ['exact_support_command', 'exact_lqr_command',
                'lqr_block__roll_pitch_attitude', 'lqr_block__body_linear_velocity', 'lqr_block__body_angular_velocity', 'exact_applied']})
    a, p = points['anchor'], points['residual_peak']
    delta = {key: np.asarray(p[key])-np.asarray(a[key]) for key in [
        'root_velocity_m_s', 'COM_velocity_m_s', 'root_relative_to_COM_velocity_m_s', 'spatial_nominal_velocity_m_s']}
    closure = delta['root_velocity_m_s']-delta['COM_velocity_m_s']-delta['root_relative_to_COM_velocity_m_s']
    initial_rows = []
    for target in [105.524,105.525,105.526,105.527]:
        j = int(np.argmin(abs(sub['time_s']-target)))
        initial_rows.append(dict(time_s=float(sub['time_s'][j]), contact_force_N=sub['force_all'][j],
            pressure_family_N=sub['pressure_family'][j],
            COM_delta_velocity_m_s=sub['COM_velocity_after'][j]-sub['COM_velocity_before'][j],
            root_delta_velocity_m_s=sub['root_velocity_after'][j]-sub['root_velocity_before'][j],
            held_input=sub['held_input'][j]))
    rate = control['reference_up_rate_deg_s']; j = ids[np.nanargmax(rate[ids])]
    report = dict(scope='Posthoc interpretation only; no new physics or primary-rule change.',
        points=points, anchor_to_peak_signed_velocity_change=delta,
        signed_decomposition_closure_m_s=closure,
        initial_drop_reload=initial_rows,
        maximum_desired_up_rate=dict(time_s=float(t[j]), deg_s=float(rate[j])),
        interpretation_limits=['The root/COM difference is a kinematic identity and includes internal motion/reorientation, not a separately measured disturbance force.',
            'Current feedback blocks act together; their large opposing terms do not prove which one is harmful.',
            'Zero angular reference during changing desired-up direction is a design convention; this replay alone does not prove that changing it improves recovery.',
            'One unloaded1msstep and immediate reload are observed; no5msairborne claim.',
            'I43 whole-target smoothing plus desired-rate correction already had mixed/failed broader results; it is not an untested cure.'])
    save(out/'report.json', report)
    save(out/'artifact_manifest.json', {p.name:binding(p) for p in sorted(out.iterdir()) if p.is_file()})
    print(json.dumps(legacy.clean(report),indent=2))


if __name__ == '__main__':
    main()
