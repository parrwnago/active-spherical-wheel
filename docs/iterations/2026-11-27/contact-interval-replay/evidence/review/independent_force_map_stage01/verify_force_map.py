"""I68 fixed-window independent MuJoCo Jacobian check, never a physics run.

Raw bridge states and reported post-solve point forces are authoritative.
MuJoCo provides private pose/Jacobian arithmetic for the generated physical
mirror, while the candidate implementation is the portable analytical core.
No contact torque or unreported solver force is reconstructed or assumed.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import mujoco as mj
import numpy as np


EVENTS_S = (7.875, 32.110, 35.055, 35.205)
TOLERANCE = 1e-10


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def module(path):
    spec = importlib.util.spec_from_file_location('_i68_analytical_core_review', path)
    out = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(out)
    return out


def qv(raw):
    """Mirror the native scalar bridge normalization; use MuJoCo rotation."""
    raw = np.asarray(raw, float)
    q = np.r_[raw[2:5], raw[5:9]/np.linalg.norm(raw[5:9]), raw[15:18]]
    rotation = np.zeros(9)
    mj.mju_quat2Mat(rotation, q[3:7])
    rotation = rotation.reshape(3, 3)
    v = np.r_[raw[9:12], rotation.T @ raw[12:15], raw[18:21]]
    return q, v, rotation


def maximum(a, b):
    a, b = np.asarray(a), np.asarray(b)
    if a.shape != b.shape or not np.array_equal(np.isfinite(a), np.isfinite(b)):
        raise AssertionError('Shape or finite-mask mismatch')
    mask = np.isfinite(a)
    return float(np.max(np.abs(a[mask]-b[mask]))) if mask.any() else 0.


def independent_frame(model, data, q, v, rows, body_ids, held):
    data.qpos[:] = q
    data.qvel[:] = v
    mj.mj_kinematics(model, data)
    mj.mj_comPos(model, data)
    loads, velocities = [], []
    virtual_work = []
    slip, pressure = [], []
    jac = np.zeros((3, model.nv))
    for row in rows:
        tag = int(row[0])
        body = int(body_ids[tag])
        point, force = row[1:4], row[7:10]
        jac[:] = 0.
        mj.mj_jac(model, data, jac, None, point, body)
        load, velocity = jac.T @ force, jac @ v
        loads.append(load)
        velocities.append(velocity)
        virtual_work.append(float(force @ velocity-v @ load))
        normal = row[4:7]/np.linalg.norm(row[4:7])
        pressure.append(max(0., float(force @ normal)))
        material = velocity.copy()
        if tag in (2, 3):
            # Belt body world-y, not a reused analytical fork rotation.
            axis = data.xmat[body].reshape(3, 3)[:, 1]
            direction = axis-normal*float(axis @ normal)
            material += held[3 if tag == 2 else 4]*direction
        slip.append(material-normal*float(material @ normal))
    return (np.asarray(loads).reshape(-1, 9), np.asarray(velocities).reshape(-1, 3),
            np.asarray(virtual_work), np.asarray(slip).reshape(-1, 3), np.asarray(pressure))


def run(args):
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    frozen = json.loads(Path(args.seal).read_text())
    for item in frozen['files']:
        if sha(item['path']) != item['sha256']:
            raise AssertionError('Frozen reviewer dependency changed: '+item['path'])
    provenance = json.loads(Path(args.model_provenance).read_text())
    entry = next(x for x in provenance['files'] if x['name'] == 'physical_mirror_xml')
    model_path = Path(entry['origin'])
    if sha(model_path) != entry['sha256']:
        raise AssertionError('Generated physical mirror differs from pre-control record')
    model = mj.MjModel.from_xml_path(str(model_path.resolve()))
    if (model.nq, model.nv) != (10, 9):
        raise AssertionError('Unexpected physical model dimensions')
    data = mj.MjData(model)
    core = module(args.core)
    with np.load(args.substeps, allow_pickle=False) as source:
        pack = {key: source[key] for key in source.files}
    ids = pack['tag_mirror_body_ids']
    for tag in range(7):
        geom = int(pack['tag_mirror_geom_ids'][tag])
        if (int(model.geom_bodyid[geom]) != ids[tag]
                or model.geom(geom).name != pack['tag_mirror_geom_names'][tag]
                or model.body(int(ids[tag])).name != pack['tag_mirror_body_names'][tag]):
            raise AssertionError('Saved tag-to-physical-mirror identity mismatch')
    report = dict(scope='Four preregistered control-end times, all five post-solve forces, before/after pose sensitivity.',
                  reference='Reported-point-force generalized load only; no exported contact couple or total solver constraint wrench.',
                  source_sha256=sha(__file__), core_sha256=sha(args.core),
                  protocol_sha256=sha(args.protocol), substeps_sha256=sha(args.substeps),
                  model_sha256=sha(model_path), numpy_version=np.__version__, mujoco_version=mj.__version__,
                  tolerance=TOLERANCE, events=[])
    all_gaps = {}
    for event in EVENTS_S:
        index = np.flatnonzero(np.abs(pack['control_end_time_s']-event) <= 1e-9)
        if len(index) != 5 or not np.array_equal(pack['substep_index'][index], np.arange(1, 6)):
            raise AssertionError('Missing exact five-solve control interval at '+str(event))
        if not pack['hold_completed'][index].all():
            raise AssertionError('Incomplete held control interval')
        event_report = dict(control_end_time_s=event, control_end_index=int(pack['control_end_index'][index[0]]),
                            sample_count=5, reported_contact_rows=0, rejected_contact_rows=0, gaps={}, per_pose={})
        native_by_pose = {'before': [], 'after': []}
        analytic_by_pose = {'before': [], 'after': []}
        def gap(name, value):
            event_report['gaps'][name] = max(event_report['gaps'].get(name, 0.), value)
            all_gaps[name] = max(all_gaps.get(name, 0.), value)
        for i in index:
            rows = pack['contact_rows'][pack['contact_offsets'][i]:pack['contact_offsets'][i+1]]
            event_report['reported_contact_rows'] += len(rows)
            event_report['rejected_contact_rows'] += int(np.count_nonzero(rows[:, 12] < .5))
            for pose in ('before', 'after'):
                raw = pack['bridge_state_before' if pose == 'before' else 'bridge_state'][i]
                q, v, _ = qv(raw)
                gap(pose+'_cached_qpos', maximum(q, pack['qpos_'+pose][i]))
                gap(pose+'_cached_qvel', maximum(v, pack['qvel_'+pose][i]))
                loads, velocity, work, slip, pressure = independent_frame(model, data, q, v, rows, ids, pack['held_input'][i])
                analytic = core.contact_frame(q, v, rows, pack['held_input'][i])
                analytic_velocity, valid = core.point_velocity(q, v, rows[:, 1:4], rows[:, 0])
                if not valid.all() or not analytic['row_valid'].all():
                    raise AssertionError('Unexpected invalid finite captured force rows')
                gap(pose+'_row_generalized', maximum(loads, analytic['row_generalized']))
                gap(pose+'_sum_generalized', maximum(np.sum(loads, axis=0), analytic['generalized']))
                gap(pose+'_rigid_point_velocity_m_s', maximum(velocity, analytic_velocity))
                gap(pose+'_belt_material_slip_m_s', maximum(slip, analytic['row_slip_velocity_m_s']))
                gap(pose+'_normal_pressure_N', maximum(pressure, analytic['row_pressure_N']))
                gap(pose+'_virtual_work_W', float(np.max(np.abs(work))) if len(work) else 0.)
                gap(pose+'_total_virtual_work_W', abs(float(np.sum(loads, axis=0) @ v-np.sum(rows[:, 7:10]*velocity))))
                native_by_pose[pose].append(np.sum(loads, axis=0))
                analytic_by_pose[pose].append(analytic['generalized'])
        q0, _, _ = qv(pack['bridge_state_before'][index[0]])
        q1, _, _ = qv(pack['bridge_state'][index[-1]])
        middle = q0[3:7]+(q1[3:7] if q0[3:7] @ q1[3:7] >= 0 else -q1[3:7])
        middle /= np.linalg.norm(middle)
        rmid = np.zeros(9)
        mj.mju_quat2Mat(rmid, middle)
        rmid = rmid.reshape(3, 3)
        dt = pack['step_dt_s'][index]
        if abs(float(np.sum(dt))-.005) > 1e-12:
            raise AssertionError('Actual solve durations do not fill one control hold')
        for pose in ('before', 'after'):
            mean = np.sum(np.asarray(native_by_pose[pose])*dt[:, None], axis=0)/np.sum(dt)
            mean[3:6] = rmid @ mean[3:6]
            analytic_mean = core.reduce_interval(np.asarray(analytic_by_pose[pose]), dt, q0, q1)
            gap(pose+'_interval_observer_frame_load', maximum(mean, analytic_mean))
            event_report['per_pose'][pose] = dict(independent_interval_load9=mean.tolist(), analytic_interval_load9=analytic_mean.tolist())
        event_report['passed'] = all(x <= TOLERANCE for x in event_report['gaps'].values())
        report['events'].append(event_report)
    report['maximum_gaps'] = all_gaps
    report['all_checks_passed'] = all(x['passed'] for x in report['events'])
    report['qualification'] = 'Verifies analytical mapping/arithmetic for reported point forces, not missing solver couples, contact-force accuracy, or model-to-DART dynamics.'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    if not report['all_checks_passed']:
        raise AssertionError('Independent Jacobian force-map check failed; see preserved report')
    print(json.dumps(dict(all_checks_passed=True, events=len(report['events']), maximum_gaps=all_gaps), sort_keys=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('substeps', 'model-provenance', 'core', 'protocol', 'seal', 'output'):
        parser.add_argument('--'+name, required=True)
    run(parser.parse_args())
