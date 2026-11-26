"""I-44: causal four-camera terrain memory over a saved native Gazebo trace.

This is offline shadow evaluation. No controller command is computed or changed.
Only the range adapter/evaluator receive scene geometry; the mapper receives
depth frames and exact acquisition poses. Each execution has a fresh stage dir.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
from time import perf_counter
import traceback

import mujoco
import numpy as np

from spherical_wheel.model import REPO_ROOT, load_config


ARMS = ('observer_plane', 'latest_four', 'memory_four')
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def save_json(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def make_anchors(times):
    """Fixed uniform+dense evaluation strata, deduplicated by saved index."""
    times = np.asarray(times, float)
    if times.ndim != 1 or not len(times) or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError('Finite strictly increasing saved times are required')
    groups = {}
    for kind, values in (
        ('uniform', np.arange(1, 110)*.4),
        ('dense_bump', 6.7+np.arange(11)*.02),
    ):
        for target in values:
            if target < times[0]-1e-9 or target > times[-1]-.2+1e-9:
                continue
            index = int(np.argmin(abs(times-target)))
            row = groups.setdefault(index, dict(anchor_index=index, anchor_time_s=float(times[index]), strata=[]))
            if kind not in row['strata']:
                row['strata'].append(kind)
    return [groups[i] for i in sorted(groups)]


def cv_state(qpos, qvel, horizon_s, model):
    """Independent MuJoCo integration oracle for the shared kinematic preview."""
    qpos, qvel = np.asarray(qpos, float), np.asarray(qvel, float)
    if qpos.shape != (model.nq,) or qvel.shape != (model.nv,) or not np.all(np.isfinite(qpos)) or not np.all(np.isfinite(qvel)):
        raise ValueError('Finite complete robot state is required')
    if not np.isfinite(horizon_s) or horizon_s < 0:
        raise ValueError('A finite nonnegative horizon is required')
    predicted = qpos.copy()
    mujoco.mj_integratePos(model, predicted, qvel, float(horizon_s))
    return predicted


def available_batch_indices(delivery_times, anchor_time_s):
    """Delivery, rather than capture, controls causal information availability."""
    delivery_times = np.asarray(delivery_times, float)
    if not np.isfinite(anchor_time_s) or not np.all(np.isfinite(delivery_times)) or np.any(np.diff(delivery_times) < 0):
        raise ValueError('Finite ordered deliveries and anchor are required')
    return np.flatnonzero(delivery_times <= anchor_time_s+1e-9)


class ObserverPlane:
    """Current observer tangent, constructed here; not the cached synchronizer."""
    def __init__(self, center, normal, radius_m, anchor_time_s):
        self.normal = np.asarray(normal, float).copy()
        self.normal /= np.linalg.norm(self.normal)
        self.point = np.asarray(center, float)-float(radius_m)*self.normal
        self.anchor_time_s = float(anchor_time_s)

    def query(self, xy, *, at_time_s):
        if abs(at_time_s-self.anchor_time_s) > 1e-8:
            raise ValueError('Knowledge must stay frozen at the anchor')
        xy = np.asarray(xy, float)
        if self.normal[2] <= 1e-6:
            return dict(supported=False, reason='vertical_observer_plane')
        z = self.point[2]-self.normal[:2]@(xy-self.point[:2])/self.normal[2]
        return dict(supported=True, reason='current_observer_tangent',
                    point_m=np.r_[xy, z], normal=self.normal.copy(), height_m=float(z),
                    first_capture_s=None, last_capture_s=None, age_s=None,
                    plane_rms_m=0., support_cells=None)


def score_surface(query, truth):
    """Conditional geometry error; unknowns keep an explicit coverage indicator."""
    supported = bool(query.get('supported', False))
    result = dict(supported=supported, reason=query.get('reason'),
                  truth_valid=bool(truth.get('valid', False)),
                  normal_ambiguous=bool(truth.get('normal_ambiguous', False)),
                  height_error_m=None, normal_error_deg=None)
    if supported and result['truth_valid']:
        result['height_error_m'] = float(query['height_m']-truth['point_m'][2])
        if not result['normal_ambiguous']:
            a, b = np.asarray(query['normal'], float), np.asarray(truth['normal'], float)
            result['normal_error_deg'] = float(np.rad2deg(np.arccos(np.clip(a@b/(np.linalg.norm(a)*np.linalg.norm(b)), -1., 1.))))
    return result


def candidate_point_error(candidates, actual_point, loaded_patches):
    """An unloaded finite fallback point is not an actual contact target."""
    point = np.asarray(actual_point, float)
    if not loaded_patches or not candidates or point.shape != (3,) or not np.all(np.isfinite(point)):
        return None
    return min(float(np.linalg.norm(np.asarray(c['point_m'], float)-point)) for c in candidates)


def source_snapshot(stage, output):
    paths = [Path(__file__).resolve(), REPO_ROOT/'config/model.yaml',
             REPO_ROOT/'pyproject.toml', output/'protocol.json', output/'protocol_clarifications.json']
    paths += sorted((REPO_ROOT/'src/spherical_wheel').rglob('*.py'))
    paths += sorted((REPO_ROOT/'tests').glob('test_terrain*.py'))
    entries = []
    for source in paths:
        rel = source.relative_to(REPO_ROOT)
        target = stage/'source'/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        entries.append(dict(path=str(rel), sha256=sha(source), snapshot=str(target.relative_to(stage))))
    save_json(stage/'source_manifest.json', dict(captured_utc=datetime.now(timezone.utc).isoformat(), sources=entries))


def load_inputs(output):
    protocol = json.loads((output/'protocol.json').read_text())
    for name, expected in protocol['inputs_sha256'].items():
        if sha(REPO_ROOT/name) != expected:
            raise RuntimeError(f'Input changed: {name}')
    with np.load(REPO_ROOT/protocol['precision']) as z:
        exact = {key: z[key].copy() for key in z.files}
    provenance = json.loads((REPO_ROOT/protocol['provenance']).read_text())
    return protocol, exact, provenance


def _metadata(scene):
    value = getattr(scene, 'metadata', {})
    return value() if callable(value) else value


def render_depth(output, stage, protocol, exact):
    from spherical_wheel.terrain_depth import FourCameraDepthAdapter, TerrainRayScene, four_camera_rig
    qpos, times = exact['exact_qpos'], exact['time_s']
    # Covers max range plus the largest full 3D camera offset under any tilt.
    margin = 2.+.30
    xy = qpos[:, :2]
    bounds = (float(xy[:, 0].min()-margin), float(xy[:, 0].max()+margin),
              float(xy[:, 1].min()-margin), float(xy[:, 1].max()+margin))
    # This exact mesh was written for the original Gazebo collision world.
    matches = sorted((REPO_ROOT/'results/lqr_contact_forecast_20261125/campaign/_workspaces/mars_figure8_p090').rglob('marsyard2021_dem.obj'))
    if len(matches) != 1:
        raise RuntimeError(f'Expected one saved collision DEM, found {len(matches)}')
    scene = TerrainRayScene.from_mars(load_config(), stage/'scene', bounds, dem_obj=matches[0])
    cameras = four_camera_rig()
    adapter = FourCameraDepthAdapter(scene, cameras)
    dt = float(np.median(np.diff(times)))
    stride = int(round(1./protocol['cameras']['capture_hz']/dt))
    indices = np.arange(0, len(times), stride, dtype=int)
    depths, masks, transforms, timings = [], [], [], []
    maximum_depth_encoding_error = 0.
    started = perf_counter()
    for count, index in enumerate(indices):
        capture = float(times[index])
        before = perf_counter()
        batch = adapter.render_batch(qpos[index], capture, capture+protocol['cameras']['latency_s'])
        timings.append(perf_counter()-before)
        if len(batch) != 4:
            raise AssertionError('Four depth frames are required')
        full_depth = np.stack([f.depth_m for f in batch])
        encoded_depth = full_depth.astype(np.float32)
        valid_depth = np.isfinite(full_depth)
        if np.any(valid_depth):
            maximum_depth_encoding_error = max(maximum_depth_encoding_error,
                float(np.max(abs(full_depth[valid_depth]-encoded_depth[valid_depth]))))
        depths.append(encoded_depth)
        masks.append(np.stack([f.valid_mask for f in batch]))
        transforms.append(np.stack([f.world_from_camera for f in batch]))
        if count % 40 == 0:
            print(json.dumps(dict(stage='depth', batches=count+1, total=len(indices), capture_time_s=capture, elapsed_s=perf_counter()-started)), flush=True)
    np.savez_compressed(stage/'depth_batches.npz', capture_time_s=times[indices],
        delivered_at_s=times[indices]+protocol['cameras']['latency_s'], source_indices=indices,
        depth_m=np.stack(depths), valid_mask=np.stack(masks),
        world_from_camera=np.stack(transforms),
        body_from_camera=np.stack([c.body_from_camera for c in cameras]),
        intrinsics=np.stack([c.intrinsics for c in cameras]),
        camera_names=np.array([c.name for c in cameras]), render_wall_s=np.array(timings))
    save_json(stage/'scene_metadata.json', _metadata(scene))
    save_json(stage/'report.json', dict(iteration='I-44', batches=len(indices), cameras_per_batch=4,
        rays_per_batch=4*64*48, elapsed_s=perf_counter()-started,
        render_wall_s=dict(median=float(np.median(timings)), p95=float(np.percentile(timings, 95)), max=max(timings)),
        encoding=dict(depth_dtype='float32', pose_dtype='float64', max_axial_depth_encoding_error_m=maximum_depth_encoding_error,
                      interpretation='No physical sensor noise; finite precision encoding is explicitly retained.'),
        measurement_boundary='Visible axial depth only; robot first-hits invalid, no see-through. Exact capture pose. No mapper or controller access to scene.',
        saved_collision_dem=dict(path=str(matches[0].relative_to(REPO_ROOT)), sha256=sha(matches[0])),
        outputs_sha256={name: sha(stage/name) for name in ('depth_batches.npz', 'scene_metadata.json')}))


def evaluate_surface(scene, xy):
    """Evaluator-only top-surface geometry, with a small face-edge probe."""
    xy = np.asarray(xy, float)
    offsets = np.array([[0., 0.], [1e-5, 0.], [-1e-5, 0.], [0., 1e-5], [0., -1e-5]])
    origins = np.c_[xy+offsets, np.full(5, 10.)]
    hit = scene.environment_hit(origins, np.tile([0., 0., -1.], (5, 1)))
    if not hit['valid'][0]:
        return dict(valid=False, normal_ambiguous=False)
    normals = hit['normal_world']
    valid = np.asarray(hit['valid'], bool)
    angles = np.arccos(np.clip(normals[valid]@normals[0], -1., 1.))
    return dict(valid=True, point_m=hit['point_world'][0], normal=normals[0],
                normal_ambiguous=bool(not np.all(valid) or np.max(angles) > 1e-4),
                normal_kind='Compiled collision-mesh geometric face proxy; not a force-weighted normal')


def statistics(values):
    values = np.asarray([v for v in values if v is not None and np.isfinite(v)], float)
    if not len(values):
        return dict(n=0, rmse=None, median_abs=None, p90_abs=None, p95_abs=None, max_abs=None)
    return dict(n=len(values), rmse=float(np.sqrt(np.mean(values**2))),
                median_abs=float(np.median(abs(values))), p90_abs=float(np.percentile(abs(values), 90)),
                p95_abs=float(np.percentile(abs(values), 95)), max_abs=float(np.max(abs(values))))


def summarize_rows(rows):
    result = {}
    for stratum in ('all_unique', 'uniform', 'dense_bump'):
        selected = [r for r in rows if stratum == 'all_unique' or stratum in r['strata']]
        result[stratum] = {}
        for arm in ARMS:
            group = [r for r in selected if r['arm'] == arm]
            horizon_results = {}
            for horizon in (None, .05, .1, .2):
                sub = [r for r in group if horizon is None or abs(r['horizon_s']-horizon) < 1e-8]
                counts = {}
                for r in sub:
                    reason = r['surface_score']['reason']
                    counts[reason] = counts.get(reason, 0)+1
                covered = sum(r['surface_score']['supported'] for r in sub)
                actual_loaded = [r for r in sub if r['true_loaded_patches']]
                proposed_changes = [r for r in actual_loaded if r['true_anchor_family'] is not None and
                                    any(p.split('_')[0] != r['true_anchor_family'] for p in r['candidate_patches'])]
                horizon_results['all_horizons' if horizon is None else f'{horizon:.2f}s'] = dict(
                    query_slots=len(sub), supported=covered,
                    coverage=covered/len(sub) if sub else None,
                    reasons=counts,
                    truth_normal_ambiguous=sum(r['surface_score']['normal_ambiguous'] for r in sub),
                    height_error_m=statistics([r['surface_score']['height_error_m'] for r in sub]),
                    normal_error_deg=statistics([r['surface_score']['normal_error_deg'] for r in sub]),
                    contact_candidate_available=sum(bool(r['candidate_patches']) for r in sub),
                    complete_contact_previews=sum(r['contact_preview_complete'] for r in sub),
                    partial_contact_previews=sum(bool(r['candidate_patches']) and not r['contact_preview_complete'] for r in sub),
                    ambiguous_candidates=sum(r['candidate_ambiguous'] for r in sub),
                    candidate_cardinality=statistics([len(r['candidate_patches']) for r in sub]),
                    nearest_candidate_to_future_equivalent_point_m=statistics([r['nearest_candidate_error_m'] for r in sub]),
                    future_equivalent_point_eligible=sum(r['future_point_eligible'] for r in sub),
                    future_loaded_queries=len(actual_loaded),
                    any_future_loaded_patch_in_candidates=sum(r['future_patch_overlap'] for r in actual_loaded),
                    future_dominant_patch_in_candidates=sum(r['future_dominant_in_candidates'] for r in actual_loaded),
                    family_change_warning_slots=len(proposed_changes),
                    endpoint_false_family_change_warnings=sum(r['true_dominant_patch'].split('_')[0] == r['true_anchor_family'] for r in proposed_changes),
                    warning_definition='Candidate contains another family than actual anchor support; endpoint false means actual future dominant family still equals anchor family. Evaluation-only labels.',
                    first_cell_observation_delivery_lead_s=statistics([r['first_cell_delivery_lead_s'] for r in sub]),
                    observation_age_s=statistics([r['observation_age_s'] for r in sub]))
            result[stratum][arm] = horizon_results
    return result


def contact_warning_report(rows, exact, weight_N):
    """Score warnings at fixed anchors, never use events to generate forecasts."""
    t, loads = exact['time_s'], exact['truth_patch_loads_N']
    threshold = max(.05, .05*weight_N)
    dominant = np.where(loads.max(axis=1) >= threshold, loads.argmax(axis=1), -1)
    family = np.where(dominant < 0, -1, dominant//2)
    cuts = np.r_[0, np.flatnonzero(np.diff(family))+1, len(t)]
    stable = []
    for first, stop in zip(cuts[:-1], cuts[1:]):
        if family[first] >= 0 and t[stop-1]-t[first] >= .015-1e-9:
            stable.append((first, stop, int(family[first])))
    events = []
    for before, after in zip(stable[:-1], stable[1:]):
        if before[2] == after[2]:
            continue
        index, _, new_family = after
        event = dict(time_s=float(t[index]), incoming_family=('shell', 'belt')[new_family],
                     incoming_patch=LABELS[int(dominant[index])], warnings={})
        for arm in ARMS:
            eligible = [r for r in rows if r['arm'] == arm and
                        r['anchor_time_s'] < t[index]-1e-9 and
                        r['anchor_time_s']+r['horizon_s'] >= t[index]-1e-9]
            warned = [r for r in eligible if any(p.startswith(event['incoming_family']) for p in r['candidate_patches'])]
            first = min(warned, key=lambda r: r['anchor_time_s']) if warned else None
            event['warnings'][arm] = dict(eligible_query_slots=len(eligible), warned_query_slots=len(warned),
                complete_warned_query_slots=sum(r['contact_preview_complete'] for r in warned),
                ambiguous_warned_query_slots=sum(r['candidate_ambiguous'] for r in warned),
                first_warning_anchor_s=None if first is None else first['anchor_time_s'],
                lead_s=None if first is None else float(t[index]-first['anchor_time_s']))
        events.append(event)
    return dict(load_threshold_N=threshold, stable_dwell_s=.015, events=events,
                interpretation='Geometric incoming-family warnings at fixed sparse anchors; not exact force transfer time, causal validation or force prediction.')


def analyze(output, stage, depth_stage, protocol, exact, provenance):
    from spherical_wheel.terrain_depth import DepthFrame, TerrainRayScene
    from spherical_wheel.terrain_memory import MemorySettings, TerrainMemory, WheelGeometry, preview_contacts
    if not (depth_stage/'report.json').is_file():
        raise RuntimeError('Use a completed preserved depth stage')
    depth_report = json.loads((depth_stage/'report.json').read_text())
    for name, expected in depth_report['outputs_sha256'].items():
        if sha(depth_stage/name) != expected:
            raise RuntimeError(f'Depth input changed: {name}')
    with np.load(depth_stage/'depth_batches.npz') as z:
        depth = {k: z[k].copy() for k in z.files}
    # Scene is only used by the evaluation branch below, never passed to maps.
    qpos, times = exact['exact_qpos'], exact['time_s']
    margin, xy = 2.30, qpos[:, :2]
    bounds = (float(xy[:, 0].min()-margin), float(xy[:, 0].max()+margin),
              float(xy[:, 1].min()-margin), float(xy[:, 1].max()+margin))
    scene = TerrainRayScene.from_mars(load_config(), stage/'evaluation_scene', bounds,
                                     dem_obj=REPO_ROOT/depth_report['saved_collision_dem']['path'])
    # A separate kinematic model verifies the shared CV without using scene data.
    model = mujoco.MjModel.from_xml_path(str(REPO_ROOT/'generated/offroad/flat/model.xml'))
    settings = MemorySettings(**{k: v for k, v in protocol['map'].items()
                                 if k not in ('unknown_rule', 'static_world_assumption')})
    memory = TerrainMemory(settings)
    geometry = WheelGeometry.from_config(load_config())
    anchors = make_anchors(times)
    if len(anchors) != protocol['anchors']['expected_unique']:
        raise AssertionError('Anchor count differs from registered protocol')
    rows, anchor_rows, cell_groups = [], [], []
    pointer, latest_batch = 0, None
    map_wall, query_wall = [], []
    max_cv_difference = 0.
    started = perf_counter()
    for number, anchor in enumerate(anchors):
        index, now = anchor['anchor_index'], anchor['anchor_time_s']
        map_start = perf_counter()
        while pointer < len(depth['capture_time_s']) and depth['delivered_at_s'][pointer] <= now+1e-9:
            batch = tuple(DepthFrame(
                camera_name=str(depth['camera_names'][k]), depth_m=depth['depth_m'][pointer, k],
                valid_mask=depth['valid_mask'][pointer, k], intrinsics=depth['intrinsics'][k],
                body_from_camera=depth['body_from_camera'][k], world_from_camera=depth['world_from_camera'][pointer, k],
                capture_time_s=float(depth['capture_time_s'][pointer]), delivered_at_s=float(depth['delivered_at_s'][pointer]))
                for k in range(4))
            memory.ingest(batch, available_at_s=now)
            latest_batch = batch
            pointer += 1
        latest = TerrainMemory(settings)
        if latest_batch is not None:
            latest.ingest(latest_batch, available_at_s=now)
        map_wall.append(perf_counter()-map_start)
        frozen = memory.snapshot(at_time_s=now)
        cells = frozen.observed_cells(at_time_s=now)
        cell_groups.append(cells)
        anchor_rows.append(dict(**anchor, latest_batch_index=pointer-1,
            maximum_ingested_capture_s=None if not pointer else float(depth['capture_time_s'][pointer-1]),
            maximum_ingested_delivery_s=None if not pointer else float(depth['delivered_at_s'][pointer-1]),
            ingested_batches=pointer, memory_cell_count=len(cells['centers_m']),
            exact_qpos=qpos[index].copy(), estimated_normal=exact['exact_estimated_normal'][index].copy()))
        providers = dict(observer_plane=ObserverPlane(qpos[index, :3], exact['exact_estimated_normal'][index],
                                                    load_config()['geometry']['sphere_radius_m'], now),
                         latest_four=latest, memory_four=frozen)
        # Every arm receives exactly the same current state and known geometry.
        arm_previews = {}
        for arm, provider in providers.items():
            before = perf_counter()
            arm_previews[arm] = preview_contacts(provider, geometry, qpos[index], exact['exact_qvel'][index],
                now, horizons_s=tuple(protocol['horizons_s']),
                band_tolerance_m=protocol['contact_geometry']['band_tolerance_m'],
                max_gap_m=protocol['contact_geometry']['max_geometric_gap_m'])
            query_wall.append(perf_counter()-before)
        for hi, horizon in enumerate(protocol['horizons_s']):
            cv = cv_state(qpos[index], exact['exact_qvel'][index], horizon, model)
            future_index = int(np.argmin(abs(times-(now+horizon))))
            target_qpos = qpos[future_index]
            motion_position_error = float(np.linalg.norm(cv[:3]-target_qpos[:3]))
            motion_orientation_error = float(2*np.arccos(np.clip(abs(cv[3:7]@target_qpos[3:7]), 0., 1.)))
            truth_surface = evaluate_surface(scene, cv[:2])
            threshold = max(.05, .05*provenance['weight_N'])
            loads = exact['truth_patch_loads_N'][future_index]
            loaded_patches = [LABELS[k] for k in np.flatnonzero(loads >= threshold)]
            dominant = LABELS[int(loads.argmax())] if loads.max() >= threshold else None
            actual_point = exact['truth_equivalent_point_m'][future_index]
            point_eligible = bool(loaded_patches and np.all(np.isfinite(actual_point)))
            anchor_loads = exact['truth_patch_loads_N'][index]
            anchor_dominant = LABELS[int(anchor_loads.argmax())] if anchor_loads.max() >= threshold else None
            for arm, provider in providers.items():
                preview = arm_previews[arm]['rows'][hi]
                difference = float(np.max(abs(np.asarray(preview['predicted_qpos'])-cv)))
                max_cv_difference = max(max_cv_difference, difference)
                if difference > 1e-10:
                    raise AssertionError('Arms do not share the independent constant-velocity pose')
                query = provider.query(cv[:2], at_time_s=now)
                candidates = preview.get('candidates', [])
                patches = sorted(set(c['patch'] for c in candidates))
                first_capture = query.get('first_capture_s')
                lead = None if first_capture is None else now+horizon-float(first_capture)-protocol['cameras']['latency_s']
                rows.append(dict(**anchor, arm=arm, horizon_s=horizon, future_index=future_index,
                    shared_cv_qpos=cv, cv_motion_position_error_m=motion_position_error,
                    cv_motion_orientation_error_rad=motion_orientation_error,
                    surface_query=query, surface_truth=truth_surface,
                    surface_score=score_surface(query, truth_surface),
                    contact_preview=preview, candidate_patches=patches,
                    contact_preview_complete=bool(preview.get('contact_prediction_supported', False)),
                    candidate_ambiguous=bool(preview.get('ambiguous', len(candidates) > 1)),
                    true_loaded_patches=loaded_patches, true_dominant_patch=dominant,
                    true_anchor_family=None if anchor_dominant is None else anchor_dominant.split('_')[0],
                    true_future_equivalent_point_m=actual_point,
                    future_point_eligible=point_eligible,
                    future_patch_overlap=bool(set(patches)&set(loaded_patches)),
                    future_dominant_in_candidates=dominant in patches if dominant else False,
                    nearest_candidate_error_m=candidate_point_error(candidates, actual_point, loaded_patches),
                    first_cell_delivery_lead_s=lead, observation_age_s=query.get('age_s')))
        if number % 15 == 0:
            print(json.dumps(dict(stage='analyze', anchors=number+1, total=len(anchors), time_s=now,
                                  ingested_batches=pointer, memory_cells=len(cells['centers_m']), elapsed_s=perf_counter()-started)), flush=True)
    summary = summarize_rows(rows)
    uniform = summary['uniform']
    memory_stats = uniform['memory_four']['all_horizons']
    latest_stats = uniform['latest_four']['all_horizons']
    limits = protocol['predeclared_gates']['memory_utility_uniform']
    utility = dict(
        coverage=memory_stats['coverage'] >= limits['coverage_min'],
        coverage_gain_vs_latest=memory_stats['coverage']-latest_stats['coverage'] >= limits['coverage_gain_vs_latest_min'],
        height_rmse=memory_stats['height_error_m']['rmse'] is not None and memory_stats['height_error_m']['rmse'] <= limits['covered_height_rmse_m_max'],
        normal_p90=memory_stats['normal_error_deg']['p90_abs'] is not None and memory_stats['normal_error_deg']['p90_abs'] <= limits['covered_normal_p90_deg_max'])
    common = {}
    for stratum in ('uniform', 'dense_bump'):
        keyed = {}
        for r in rows:
            if stratum in r['strata']:
                keyed.setdefault((r['anchor_index'], r['horizon_s']), {})[r['arm']] = r
        common_rows = [g for g in keyed.values() if all(g[a]['surface_score']['supported'] for a in ARMS)]
        common[stratum] = dict(query_slots=len(common_rows), arms={a:dict(
            height_error_m=statistics([g[a]['surface_score']['height_error_m'] for g in common_rows]),
            normal_error_deg=statistics([g[a]['surface_score']['normal_error_deg'] for g in common_rows])) for a in ARMS})
    report = dict(iteration='I-44', scope=protocol['scope'], anchors=len(anchors),
        unique_forecast_query_slots=len(anchors)*len(protocol['horizons_s']), arms=ARMS,
        strata_overlap='6.8s belongs to both strata; counts must not be added as independent samples.',
        summary=summary, common_covered=common,
        utility_gates=utility, utility_passed=all(utility.values()),
        integrity=dict(max_cv_difference_vs_mujoco=max_cv_difference,
            kinematic_model=dict(path='generated/offroad/flat/model.xml',
                                 sha256=sha(REPO_ROOT/'generated/offroad/flat/model.xml')),
            causal_frames=all(a['maximum_ingested_delivery_s'] is None or a['maximum_ingested_delivery_s'] <= a['anchor_time_s']+1e-9 for a in anchor_rows),
            controller_invoked=False, new_physics_runs=0, commands='Unchanged reused native trace, verified by input hash; no new control policy executed.'),
        shared_cv_motion_position_error_m=statistics([r['cv_motion_position_error_m'] for r in rows if r['arm'] == 'observer_plane']),
        incoming_family_warnings=contact_warning_report(rows, exact, provenance['weight_N']),
        runtime=dict(analysis_elapsed_s=perf_counter()-started, map_update_wall_s=statistics(map_wall),
                     three_horizon_preview_wall_s=statistics(query_wall), interpretation='Offline wall time, not a real-time claim.'),
        limitations=['Ideal depth and exact pose; no SLAM, sensor noise or hardware validation.',
                     '2.5D local plane memory cannot represent general multiple-surface contacts.',
                     'Geometry candidates do not predict actual forces, load sharing or slip.',
                     'Geometric normals use private compiled collision meshes, not saved contact forces.',
                     'Same collision-ray backend supplies ideal sensing and geometric evaluation; analytic fixtures and saved actual Gazebo contacts provide separate checks.',
                     'Trajectory and overshoot cannot improve in shadow replay.'])
    offsets = np.r_[0, np.cumsum([len(c['centers_m']) for c in cell_groups])]
    np.savez_compressed(stage/'memory_snapshots.npz', anchor_time_s=np.array([a['anchor_time_s'] for a in anchors]),
        offsets=offsets, **{k:np.concatenate([c[k] for c in cell_groups], axis=0)
                            for k in ('centers_m', 'count', 'first_capture_s', 'last_capture_s', 'age_s', 'height_spread_m')})
    save_json(stage/'anchor_records.json', anchor_rows)
    save_json(stage/'predictions.json', rows)
    save_json(stage/'report.json', report)
    save_json(stage/'artifact_manifest.json', {name:dict(sha256=sha(stage/name), bytes=(stage/name).stat().st_size)
                                             for name in ('report.json', 'predictions.json', 'anchor_records.json', 'memory_snapshots.npz')})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=('depth', 'analyze'), required=True)
    parser.add_argument('--tag', default='stage01')
    parser.add_argument('--depth-tag', default='stage01')
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum() or not args.depth_tag.replace('_', '').isalnum():
        parser.error('Stage tags must be alphanumeric with optional underscores')
    output = args.output.resolve()
    protocol, exact, provenance = load_inputs(output)
    stage = output/'campaign'/f'{args.stage}_{args.tag}'
    stage.mkdir(parents=True, exist_ok=False)
    source_snapshot(stage, output)
    try:
        if args.stage == 'depth':
            render_depth(output, stage, protocol, exact)
        else:
            analyze(output, stage, output/'campaign'/f'depth_{args.depth_tag}', protocol, exact, provenance)
    except Exception as error:
        save_json(stage/'failure.json', dict(error=str(error), traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
