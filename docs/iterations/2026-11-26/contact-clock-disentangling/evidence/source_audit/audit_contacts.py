"""Registered I48 algebraic contact/pose convention audit; saved NumPy only."""
from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
I46 = ROOT / 'results/support_routing_timing_20261126'
CASES = [('mars', 'mars_compact_outback', 'mars_compact_outback'),
         ('flat', 'flat_compact_outback_setup_corrected', 'flat_compact_outback')]
PLUGIN = ROOT / 'gazebo/plugin/SphericalWheelBridge.cc'
BINARY = ROOT / 'build/gazebo/libSphericalWheelBridge.so'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def axis_from_state(state):
    q = state[:, 5:9].copy()
    norm = np.linalg.norm(q, axis=1)
    q /= norm[:, None]
    local = np.column_stack([np.cos(state[:, 15]), np.zeros(len(state)), -np.sin(state[:, 15])])
    first = np.cross(q[:, 1:], local)
    axis = local + 2 * q[:, :1] * first + 2 * np.cross(q[:, 1:], first)
    return axis


def main():
    if (HERE / 'report.json').exists():
        raise FileExistsError('Preserve existing registered audit')
    inputs = [PLUGIN, BINARY]
    for _, folder, workspace in CASES:
        inputs += [I46 / 'campaign/runs' / folder / 'truth_substeps.npz',
                   I46 / 'campaign/_workspaces' / workspace / 'gazebo/world/models/active_spherical_wheel/model.sdf',
                   I46 / 'campaign/_workspaces' / workspace / 'flat/model.xml']
    source = HERE / 'source_before_execution'
    source.mkdir(exist_ok=False)
    shutil.copyfile(__file__, source / 'audit_contacts.py')
    shutil.copyfile(PLUGIN, source / PLUGIN.name)
    before = {str(p.relative_to(ROOT)): sha(p) for p in inputs}
    registration = dict(registered_utc=datetime.now(timezone.utc).isoformat(),
        conditions=['prior_published_pose_raw_vector', 'post_published_pose_raw_vector',
                    'prior_published_pose_radial_R', 'post_published_pose_radial_R'],
        primary_records='Individual raw contact rows tagged smooth0..3, admitted row12>.5, Fn=max(F dot n,0)>.05N; first substep excluded.',
        pose_proxy='Prior state_i-1 is preceding published pose proxy for cached PreUpdate geometry, NOT a proved force timestamp.',
        exact_band_tolerance_m=1e-12, weighting='Record fraction and normal-load-weighted violating fraction, separately per case.',
        worst_table='Worst10 distinct substeps ranked by smallest post_raw individual-point margin; keep worst point within each substep.',
        source_sha256=sha(Path(__file__)), input_sha256=before,
        scope='No observer/filter/controller/physics invocation; no clock fitting or source/data changes.')
    save(HERE / 'protocol_before_execution.json', registration)
    reports, summary_rows, worst_rows = {}, [], []
    for case_id, folder, workspace in CASES:
        sdf = I46 / 'campaign/_workspaces' / workspace / 'gazebo/world/models/active_spherical_wheel/model.sdf'
        plugin = next(p for p in ET.parse(sdf).getroot().iter('plugin') if p.find('sphere_radius') is not None)
        radius, gap, polar = (float(plugin.find(n).text) for n in ('sphere_radius', 'equatorial_gap_half', 'polar_rim'))
        xml = I46 / 'campaign/_workspaces' / workspace / 'flat/model.xml'
        for name in ('shell_A', 'shell_B', 'belt_A', 'belt_B'):
            user = ET.parse(xml).getroot().find(".//geom[@name='" + name + "_smooth']").get('user').split()
            assert float(user[4]) == radius
            assert float(user[1]) == (gap if name.startswith('shell') else polar)
            assert float(user[2]) == (polar if name.startswith('shell') else radius)
        path = I46 / 'campaign/runs' / folder / 'truth_substeps.npz'
        with np.load(path) as z:
            times, state, contacts, offsets = (z[k].copy() for k in ('time_s', 'bridge_state', 'contacts', 'contact_offsets'))
            fields = z['bridge_state_fields'].tolist()
        assert fields[0] == 'time' and fields[15] == 'q_shaft' and fields[22:] == ['resets', 'reset_pending']
        assert np.all(abs(np.diff(times) - .001) < 1e-9)
        assert np.all(abs(np.diff(state[:, 0]) - .001) < 1e-9)
        assert np.all(np.diff(state[:, 1]) == 1)
        assert np.all(abs(state[:, 21] - .001) < 1e-12)
        assert np.all(state[:, 22] == state[0, 22]) and np.all(state[:, 23] == 0)
        assert offsets[0] == 0 and offsets[-1] == len(contacts)
        sample = np.repeat(np.arange(len(times)), np.diff(offsets))
        force = np.maximum(np.einsum('ij,ij->i', contacts[:, 7:10], contacts[:, 4:7]), 0.)
        eligible = (sample > 0) & (contacts[:, 0] >= 0) & (contacts[:, 0] <= 3) & (contacts[:, 12] > .5) & (force > .05)
        raw_indices = np.flatnonzero(eligible)
        row, index, weight = contacts[eligible], sample[eligible], force[eligible]
        tags = row[:, 0].astype(int)
        lower = np.where(tags < 2, gap, polar)
        upper = np.where(tags < 2, polar, radius)
        sign = np.where((tags == 0) | (tags == 2), 1., -1.)
        axis = axis_from_state(state)
        margins, lengths = {}, {}
        for pose, shift in (('prior', -1), ('post', 0)):
            vector = row[:, 1:4] - state[index + shift, 2:5]
            length = np.linalg.norm(vector, axis=1)
            assert np.all(length > 1e-12)
            lengths[pose] = length
            coordinate = sign * np.einsum('ij,ij->i', axis[index + shift], vector)
            for representation, value in (('raw', coordinate), ('radial_R', coordinate * radius / length)):
                key = pose + '_' + representation
                margin = np.minimum(value - lower, upper - value)
                margins[key] = margin
                bad = margin < -1e-12
                result = dict(case_id=case_id, condition=key, records=len(row), violating_records=int(sum(bad)),
                    violating_record_fraction=float(np.mean(bad)) if len(row) else None,
                    minimum_margin_m=float(margin.min()) if len(row) else None,
                    minimum_margin_um=float(margin.min() * 1e6) if len(row) else None,
                    normal_load_weighted_violating_fraction=float(weight[bad].sum() / weight.sum()) if weight.sum() else None)
                summary_rows.append(result)
        order = np.lexsort((raw_indices, margins['post_raw']))
        chosen, used = [], set()
        for n in order:
            if int(index[n]) not in used:
                chosen.append(int(n)); used.add(int(index[n]))
            if len(chosen) == 10:
                break
        for rank, n in enumerate(chosen, 1):
            worst_rows.append(dict(case_id=case_id, rank=rank, substep_index=int(index[n]), raw_contact_row_index=int(raw_indices[n]),
                recorded_relative_time_s=float(times[index[n]]), recorded_bridge_time_s=float(state[index[n], 0]),
                tag=int(tags[n]), normal_force_N=float(weight[n]),
                point_x_m=float(row[n, 1]), point_y_m=float(row[n, 2]), point_z_m=float(row[n, 3]),
                prior_radius_m=float(lengths['prior'][n]), post_radius_m=float(lengths['post'][n]),
                **{key + '_margin_um': float(value[n] * 1e6) for key, value in margins.items()}))
        reports[case_id] = dict(substeps=len(times), all_raw_contact_rows=len(contacts), eligible_records=len(row),
            first_substep_rows_excluded=int(offsets[1]), smooth_record_tags=[0, 1, 2, 3],
            metadata=dict(radius_m=radius, gap_half_m=gap, polar_rim_m=polar),
            clock_checks=dict(relative_time_continuous=True, bridge_time_continuous=True, iteration_increments_one=True,
                constant1ms_step=True, no_reset_or_pending_reset=True,
                bridge_minus_relative_time_range_s=[float((state[:, 0] - times).min()), float((state[:, 0] - times).max())]),
            conditions=[r for r in summary_rows if r['case_id'] == case_id])
    for name, values in [('conditions.csv', summary_rows), ('worst10_per_case.csv', worst_rows)]:
        with (HERE / name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(values[0])); writer.writeheader(); writer.writerows(values)
    report = dict(iteration='I-48', scope=registration['scope'], cases=reports,
        source_findings=[
            'PreUpdate caches center/carrier axes; OnContact admits using that cache.',
            'Admit radially normalizes each smooth contact vector to R before band projection.',
            'PostUpdate publishes updated state and contact messages; normal sign/fallback Admit still use cached geometry.',
            'Rows do not serialize per-contact producer timestamp or decision-match flag; force epoch remains unproved.',
            'A2 projected a load-weighted equivalent raw vector, which differs from an individual solver point and plugin radial normalization.'],
        limits=['Prior published pose is a source-backed proxy, not a calibrated sensor/force delay.',
                'This tests contact/admission conventions only; no observer error, filter benefit or closed-loop fix is established.',
                'Rims/body and first substep are excluded from this smooth-point band diagnostic.'])
    save(HERE / 'report.json', report)
    for path in inputs:
        assert sha(path) == before[str(path.relative_to(ROOT))]
    files = [p for p in HERE.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    save(HERE / 'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(HERE)), bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(files)]))


if __name__ == '__main__':
    main()
