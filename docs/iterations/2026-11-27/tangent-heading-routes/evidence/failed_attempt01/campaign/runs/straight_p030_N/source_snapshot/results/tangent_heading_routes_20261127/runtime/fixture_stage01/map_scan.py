"""One preregistered finite I76 map-only eight-placement scan.

All six candidates are evaluated once in a fixed order. The first passing one
is selected. No dynamic outcome, active-route mutation or controller call.
"""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import shutil

import numpy as np

ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parents[2]
YARD=ROOT/'assets/gazebo/third_party/leo_simulator-ros2/leo_gz_worlds/models/marsyard2021_terrain'
OFFSETS=[[.5,.4],[.5,-.4],[.5,.8],[.5,-.8],[.5,1.2],[.5,-1.2]]
SOURCES=[Path(__file__).resolve(),Path(__file__).with_name('map_preflight.py'),BASE/'runtime/route.py',
    ROOT/'src/spherical_wheel/gazebo/mars.py',ROOT/'src/spherical_wheel/gazebo/worlds.py']
INPUTS=[YARD/'dem/marsyard_terrain_hm.tif',
    *[YARD/'meshes'/f'{name}.obj' for name in ('lava','rocks_big','rocks_medium')]]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x') as stream:json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False);stream.write('\n')


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


def validate(value):
    if value['route_id']!='eight' or value['anchor_m']!=[-2.4,10.0] or value['geometry_heading_deg']!=113.:
        raise ValueError('Frozen shape, anchor and rotation required')
    if value['candidate_local_offsets_m']!=OFFSETS:raise ValueError('Only the six root-frozen candidates, in order, are allowed')
    if value['selection_policy']!='first_passing_in_declared_order':raise ValueError('First-pass policy required')
    if value['sampling']!=dict(route_count=1601,corridor_radius_m=.15,circumference_samples=25):
        raise ValueError('Unchanged route/corridor sampling required')
    if value['screens']!=dict(maximum_corridor_facet_slope_deg=12.,
            maximum_corridor_total_height_relief_m=.08,minimum_route_feature_xy_clearance_m=1.):
        raise ValueError('Unchanged fixed screens required')


def register(stage,protocol):
    stage.mkdir(parents=True,exist_ok=False)
    declared=json.loads(protocol.read_text());validate(declared)
    paths=SOURCES+INPUTS+[protocol,BASE/'runtime/fixture_stage01/historical_I75_protocol.json']
    records={str(path.relative_to(ROOT)):dict(sha256=sha(path),bytes=path.stat().st_size) for path in paths}
    for group in ('source_sha256','input_sha256'):
        for name,digest in declared[group].items():
            if sha(ROOT/name)!=digest:raise ValueError('Root-frozen scan input changed: '+name)
    for path in SOURCES+[protocol]:
        out=stage/'source'/path.relative_to(ROOT);out.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,out)
    save(stage/'registered_before_arithmetic.json',dict(iteration='I-76',utc=datetime.now(timezone.utc).isoformat(),
        files=records,protocol=declared,registered_before_map_arithmetic=True,
        candidate_count=6,evaluate_all_once=True,no_active_route_mutation=True,no_physics=True,
        no_controller_or_estimator_execution=True,no_dynamic_outcomes_read=True))


def verify(registered):
    for name,binding in registered['files'].items():
        path=ROOT/name
        if sha(path)!=binding['sha256'] or path.stat().st_size!=binding['bytes']:
            raise ValueError('Registered scan source/input changed: '+name)
    for group in ('source_sha256','input_sha256'):
        for name,digest in registered['protocol'][group].items():
            if sha(ROOT/name)!=digest:raise ValueError('Root-frozen scan binding changed: '+name)


def execute(stage):
    registered=json.loads((stage/'registered_before_arithmetic.json').read_text());verify(registered)
    if (stage/'report.json').exists():raise FileExistsError('Preserve the previous finite scan')
    declared=registered['protocol'];validate(declared)
    routes=module(BASE/'runtime/route.py','_i76_scan_frozen_routes')
    helper=module(Path(__file__).with_name('map_preflight.py'),'_i76_scan_frozen_map_helper')
    from spherical_wheel.gazebo.mars import MarsYard,feature_triangles
    yard=MarsYard();features=feature_triangles()
    original=routes.make_route('eight')
    if not np.array_equal(original.anchor_origin,declared['anchor_m']):raise ValueError('Source anchor differs')
    sampling=declared['sampling'];screens=declared['screens']
    base_fixture=routes.samples('eight',count=sampling['route_count'],
        corridor_radius_m=sampling['corridor_radius_m'],corridor_count=sampling['circumference_samples'])
    reports=[]
    for index,offset in enumerate(OFFSETS,start=1):
        # These are isolated in-memory offline fixtures; active source bytes are
        # never edited, and derivatives/timing remain those of the frozen eight.
        path=deepcopy(original);path.planned_local_offset=np.asarray(offset,float)
        path.origin=path.anchor_origin+path.rotation@path.planned_local_offset
        shift=path.origin-original.origin
        fixture={name:value.copy() for name,value in base_fixture.items()}
        fixture['xy_m']+=shift;fixture['corridor_xy_m']+=shift
        route,corridor=fixture['xy_m'],fixture['corridor_xy_m']
        planes=np.array([yard._plane(x,y) for x,y in corridor])
        route_planes=np.array([yard._plane(x,y) for x,y in route])
        slopes=np.rad2deg(np.arctan(np.linalg.norm(planes[:,1:],axis=1)))
        route_slopes=np.rad2deg(np.arctan(np.linalg.norm(route_planes[:,1:],axis=1)))
        low,high=route.min(axis=0)-1.,route.max(axis=0)+1.
        minimum_clearance=np.inf;details={}
        for name,triangles in features.items():
            tri=triangles[:,:,:2]
            keep=np.all(tri.max(axis=1)>=low,axis=1)&np.all(tri.min(axis=1)<=high,axis=1)
            distance=helper.triangle_xy_distance(route,tri[keep]) if keep.any() else np.full(len(route),1.)
            clearance=float(distance.min());minimum_clearance=min(minimum_clearance,clearance)
            fixture[name+'_xy_clearance_lower_bound_m']=np.minimum(distance,1.)
            details[name]=dict(total_triangles=len(tri),nearby_triangles=int(keep.sum()),
                minimum_sampled_route_xy_clearance_m=clearance,lower_bound_m=min(clearance,1.),exact_below_one_m=clearance<1.)
        relief=float(np.ptp(planes[:,0]))
        gates=dict(corridor_within_dem=bool((np.abs(corridor)<=yard.size/2).all()),
            max_corridor_facet_slope=bool(slopes.max()<=screens['maximum_corridor_facet_slope_deg']),
            corridor_height_range=relief<=screens['maximum_corridor_total_height_relief_m'],
            feature_xy_clearance=minimum_clearance>=screens['minimum_route_feature_xy_clearance_m'])
        fixture.update(corridor_height_m=planes[:,0],corridor_slope_deg=slopes,
            route_height_m=route_planes[:,0],route_slope_deg=route_slopes)
        folder=stage/f'candidate_{index:02d}';folder.mkdir()
        np.savez_compressed(folder/'map_samples.npz',**fixture)
        report=dict(iteration='I-76',candidate_index_1based=index,planned_local_offset_m=offset,
            planned_origin_m=path.origin.tolist(),anchor_m=declared['anchor_m'],route=path.spec(),
            passed=all(gates.values()),gates=gates,fixed_screens=screens,
            corridor_relief_m=relief,corridor_slope_deg=dict(maximum=float(slopes.max()),p95=float(np.percentile(slopes,95))),
            route_slope_deg=dict(maximum=float(route_slopes.max()),p95=float(np.percentile(route_slopes,95))),
            corridor_height_range_m=[float(planes[:,0].min()),float(planes[:,0].max())],
            route_height_range_m=[float(route_planes[:,0].min()),float(route_planes[:,0].max())],
            route_xy_bounds_m=[float(route[:,0].min()),float(route[:,0].max()),float(route[:,1].min()),float(route[:,1].max())],
            features=details,no_physics=True,no_dynamic_outcomes_read=True,no_active_route_mutation=True,
            limitations=['Sampled route/circumference corridor, not a continuous certificate.',
                'Mesh feature clearance is conservative XY projection, capped at1m lower bound.',
                'Map-only fixture selection gives no dynamic performance or actuator-feasibility evidence.'])
        save(folder/'report.json',report);reports.append(report)
    first=next((report for report in reports if report['passed']),None)
    verify(registered)
    selection=dict(policy='first_passing_in_declared_order',passed=first is not None,
        selected_index_1based=None if first is None else first['candidate_index_1based'],
        selected_local_offset_m=None if first is None else first['planned_local_offset_m'],
        selected_planned_origin_m=None if first is None else first['planned_origin_m'],
        all_six_evaluated_once=True,no_best_value_selection=True,no_outcome_selection=True)
    save(stage/'selection.json',selection)
    save(stage/'report.json',dict(iteration='I-76',candidate_count=6,candidates=reports,selection=selection,
        passed=first is not None,no_physics=True,no_active_route_mutation=True,no_dynamic_outcomes_read=True,
        source_and_inputs_unchanged=True,dem_loaded_once=True,features_loaded_once=True,
        next_step='Root must freeze the selected active route and run canonical full-route map preflight before physics.'))
    save(stage/'artifact_manifest.json',dict(files={str(p.relative_to(stage)):dict(sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(stage.rglob('*')) if p.is_file()},no_physics=True,inputs_unchanged=True))
    print(json.dumps(dict(selection=selection,candidates=[dict(index=r['candidate_index_1based'],
        offset=r['planned_local_offset_m'],passed=r['passed'],relief_m=r['corridor_relief_m'],
        maximum_facet_slope_deg=r['corridor_slope_deg']['maximum']) for r in reports]),indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',type=Path,required=True);parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--register',action='store_true');parser.add_argument('--run',action='store_true')
    args=parser.parse_args()
    if args.register:register(args.stage.resolve(),args.protocol.resolve())
    if args.run:execute(args.stage.resolve())
