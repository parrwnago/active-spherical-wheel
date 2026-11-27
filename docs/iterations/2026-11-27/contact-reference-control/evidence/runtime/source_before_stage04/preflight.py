"""Preregistered map-only I73 route inspection. Never constructs a simulator.

The DEM and feature meshes are an offline fixture for selecting the experiment
site. They are never supplied to the controller or contact-selector adapter.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil
import sys
import numpy as np
from route import SmoothSPath, samples, world_bounds

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]
YARD = ROOT/'assets/gazebo/third_party/leo_simulator-ros2/leo_gz_worlds/models/marsyard2021_terrain'
SOURCES = [Path(__file__).resolve(), BASE/'runtime/route.py',
           ROOT/'src/spherical_wheel/gazebo/mars.py',
           ROOT/'src/spherical_wheel/gazebo/worlds.py']
INPUTS = [YARD/'dem/marsyard_terrain_hm.tif',
          *[YARD/'meshes'/f'{name}.obj' for name in ('lava', 'rocks_big', 'rocks_medium')]]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def register(stage, protocol):
    stage.mkdir(parents=True, exist_ok=False)
    paths = SOURCES+INPUTS+[protocol]
    records = {str(path.relative_to(ROOT)): dict(sha256=sha(path), bytes=path.stat().st_size)
               for path in paths}
    for path in SOURCES:
        target = stage/'source'/path.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
    value = dict(registered_utc=datetime.now(timezone.utc).isoformat(),
                 registered_before_map_arithmetic=True, paths=records,
                 no_physics=True, no_controller_or_estimator_execution=True,
                 fixture_only=True,
                 fixed_sampling=dict(route_count=1601, corridor_radius_m=.15,
                                     circumference_samples=25, path_included=True),
                 fixed_screen=dict(max_corridor_facet_slope_deg=8.,
                                   max_corridor_height_range_m=.08,
                                   min_route_feature_xy_clearance_m=.25),
                 feature_screen='Conservative XY distance to every original mesh triangle, '
                                'including buried geometry; no obstacle raster threshold.')
    save(stage/'registered_before_arithmetic.json', value)
    return value


def triangle_xy_distance(points, triangles):
    """Exact 2D point-to-closed-triangle distance, including degenerate facets."""
    answer = np.full(len(points), np.inf)
    for first in range(0, len(points), 80):
        p = points[first:first+80, None, :]
        distance = np.full((len(p), len(triangles)), np.inf)
        cross = []
        for k in range(3):
            a, b = triangles[None, :, k, :], triangles[None, :, (k+1)%3, :]
            edge, delta = b-a, p-a
            length2 = np.sum(edge*edge, axis=-1)
            ratio = np.sum(delta*edge, axis=-1)/np.where(length2>0, length2, 1.)
            closest = a+np.clip(ratio, 0., 1.)[..., None]*edge
            distance = np.minimum(distance, np.sum((p-closest)**2, axis=-1))
            cross.append(edge[..., 0]*delta[..., 1]-edge[..., 1]*delta[..., 0])
        signed = np.stack(cross, axis=-1)
        area2 = ((triangles[:, 1, 0]-triangles[:, 0, 0])*(triangles[:, 2, 1]-triangles[:, 0, 1])-
                 (triangles[:, 1, 1]-triangles[:, 0, 1])*(triangles[:, 2, 0]-triangles[:, 0, 0]))
        inside = ((np.all(signed>=0., axis=-1)|np.all(signed<=0., axis=-1)) & (area2[None, :]!=0.))
        distance[inside] = 0.
        answer[first:first+len(p)] = np.sqrt(np.min(distance, axis=1))
    return answer


def execute(stage, protocol):
    registered = json.loads((stage/'registered_before_arithmetic.json').read_text())
    for relative, binding in registered['paths'].items():
        if sha(ROOT/relative) != binding['sha256']:
            raise ValueError(f'Registered map source/input changed: {relative}')
    if (stage/'report.json').exists():
        raise FileExistsError('Preserve the existing map inspection stage')
    from spherical_wheel.gazebo.mars import MarsYard, feature_triangles
    path = SmoothSPath()
    fixture = samples()
    yard = MarsYard()
    corridor = fixture['corridor_xy_m']
    route = fixture['xy_m']
    planes = np.array([yard._plane(x, y) for x, y in corridor])
    route_planes = np.array([yard._plane(x, y) for x, y in route])
    slopes = np.rad2deg(np.arctan(np.linalg.norm(planes[:, 1:], axis=1)))
    route_slopes = np.rad2deg(np.arctan(np.linalg.norm(route_planes[:, 1:], axis=1)))
    minimum_clearance, feature_details = np.inf, {}
    # To avoid expensive all-yard products, an exact bounding-box exclusion:
    # triangles outside this route AABB expanded by 1m cannot come within1m.
    # All retained triangle coordinates are the original world-frame geometry.
    low, high = route.min(axis=0)-1., route.max(axis=0)+1.
    for name, triangles3 in feature_triangles().items():
        tri = triangles3[:, :, :2]
        keep = np.all(tri.max(axis=1)>=low, axis=1)&np.all(tri.min(axis=1)<=high, axis=1)
        if keep.any():
            distance = triangle_xy_distance(route, tri[keep])
            value = float(distance.min())
        else:
            distance = np.full(len(route), 1.)
            value = 1.
        minimum_clearance = min(minimum_clearance, value)
        feature_details[name] = dict(total_triangles=len(tri), nearby_triangles=int(keep.sum()),
                                     minimum_sampled_route_xy_clearance_m=value,
                                     lower_bound_m=1. if value>=1. else value,
                                     exact_below_one_m=value<1.)
        fixture[f'{name}_xy_clearance_lower_bound_m'] = np.minimum(distance, 1.)
    bounds = world_bounds()
    screen = registered['fixed_screen']
    height_range = float(np.ptp(planes[:, 0]))
    gates = dict(corridor_within_dem=bool((np.abs(corridor)<=yard.size/2).all()),
                 max_corridor_facet_slope=bool(slopes.max()<=screen['max_corridor_facet_slope_deg']),
                 corridor_height_range=height_range<=screen['max_corridor_height_range_m'],
                 feature_xy_clearance=minimum_clearance>=screen['min_route_feature_xy_clearance_m'])
    fixture.update(corridor_height_m=planes[:, 0], corridor_slope_deg=slopes,
                   route_height_m=route_planes[:, 0], route_slope_deg=route_slopes)
    np.savez_compressed(stage/'map_samples.npz', **fixture)
    report = dict(iteration='I-73', no_physics=True, fixture_only=True,
                  route=path.spec(), passed=all(gates.values()), gates=gates,
                  dem=dict(path=str(INPUTS[0].relative_to(ROOT)), sha256=sha(INPUTS[0]),
                           shape=list(yard.heights.shape), bounds_xy_m=[-yard.size/2,yard.size/2]*2,
                           cell_m=yard.spacing, full_height_range_m=[float(yard.heights.min()), float(yard.heights.max())]),
                  route_xy_bounds_m=[float(route[:,0].min()), float(route[:,0].max()), float(route[:,1].min()), float(route[:,1].max())],
                  world_visual_bounds_m=list(bounds),
                  route_height_range_m=[float(route_planes[:,0].min()),float(route_planes[:,0].max())],
                  corridor_height_range_m=[float(planes[:,0].min()),float(planes[:,0].max())],
                  corridor_relief_m=height_range,
                  route_slope_deg=dict(maximum=float(route_slopes.max()), p95=float(np.percentile(route_slopes,95))),
                  corridor_slope_deg=dict(maximum=float(slopes.max()), p95=float(np.percentile(slopes,95))),
                  maximum_sampled_reference_speed_m_s=float(np.linalg.norm(fixture['velocity_m_s'],axis=1).max()),
                  maximum_sampled_reference_acceleration_m_s2=float(np.linalg.norm(fixture['acceleration_m_s2'],axis=1).max()),
                  sampled_polyline_length_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()),
                  features=feature_details,
                  limitations=['Sampled route/circumference corridor, not a continuous safety certificate.',
                               'Feature clearance is a conservative 2D projection, capped at a 1m lower bound.',
                               'Known terrain is used only to select an offline experiment fixture; control gets no map.',
                               'No simulator outcome or actuator feasibility is implied by these geometric screens.'])
    save(stage/'report.json', report)
    for relative, binding in registered['paths'].items():
        if sha(ROOT/relative) != binding['sha256']:
            raise ValueError(f'Map source/input changed during inspection: {relative}')
    artifacts = {name:dict(sha256=sha(stage/name), bytes=(stage/name).stat().st_size)
                 for name in ('registered_before_arithmetic.json','map_samples.npz','report.json')}
    save(stage/'artifact_manifest.json', dict(files=artifacts, inputs_unchanged=True))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--register', action='store_true')
    parser.add_argument('--run', action='store_true')
    args=parser.parse_args()
    if args.register:
        register(args.stage.resolve(), args.protocol.resolve())
    if args.run:
        execute(args.stage.resolve(), args.protocol.resolve())
