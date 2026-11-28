"""I76 fixed-site map-only screens for all three preregistered route corridors.

No simulator, controller, estimator or case outcome is constructed/read. The
known map is an offline experiment fixture and is never supplied to control.
"""
from datetime import datetime, timezone
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
ROUTE_IDS=('straight','arc','eight')
SOURCES=[Path(__file__).resolve(),BASE/'runtime/route.py',
    ROOT/'src/spherical_wheel/gazebo/mars.py',ROOT/'src/spherical_wheel/gazebo/worlds.py']
INPUTS=[YARD/'dem/marsyard_terrain_hm.tif',
    *[YARD/'meshes'/f'{name}.obj' for name in ('lava','rocks_big','rocks_medium')]]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x') as stream:json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False);stream.write('\n')


def validate_protocol(value):
    if tuple(value['route_ids'])!=ROUTE_IDS:raise ValueError('All three fixed routes required')
    if value['anchor_m']!=[-2.4,10.0] or value['geometry_heading_deg']!=113.0:
        raise ValueError('Site anchor/rotation must remain fixed')
    if set(value['origin_by_route_m'])!=set(ROUTE_IDS):raise ValueError('Every route origin must be explicit')
    for route_id,origin in value['origin_by_route_m'].items():
        if np.asarray(origin).shape!=(2,) or not np.isfinite(origin).all():raise ValueError('Finite planned XY origin required')
        if route_id!='eight' and origin!=value['anchor_m']:raise ValueError('Straight/arc origin must retain original anchor')
    expected_sampling=dict(route_count=1601,corridor_radius_m=.15,circumference_samples=25)
    if value['sampling']!=expected_sampling:raise ValueError('Map sampling differs from root freeze')
    expected_screens=dict(maximum_corridor_facet_slope_deg=12.,
        maximum_corridor_total_height_relief_m=.08,minimum_route_feature_xy_clearance_m=1.)
    if value['screens']!=expected_screens:raise ValueError('Map screens differ from root freeze')


def register(stage,protocol):
    stage.mkdir(parents=True,exist_ok=False)
    declared=json.loads(protocol.read_text());validate_protocol(declared)
    paths=SOURCES+INPUTS+[protocol,BASE/'runtime/fixture_stage01/historical_I75_protocol.json']
    bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in paths}
    for group in ('source_sha256','input_sha256'):
        for relative,digest in declared[group].items():
            if sha(ROOT/relative)!=digest:raise ValueError('Root-registered input differs: '+relative)
    for path in SOURCES+[protocol]:
        destination=stage/'source'/path.relative_to(ROOT)
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,destination)
    value=dict(iteration='I-76',utc=datetime.now(timezone.utc).isoformat(),files=bindings,
        protocol=declared,registered_before_map_arithmetic=True,no_physics=True,
        no_controller_or_estimator_execution=True,no_case_outcomes_read=True,
        policy='Evaluate every frozen route origin once; retain every failed screen. No automatic site/threshold search.',
        feature_screen='Original mesh triangles, including buried geometry; conservative XY distance lower bound capped at1m.')
    save(stage/'registered_before_arithmetic.json',value)
    return value


def verify_bindings(registered):
    for relative,binding in registered['files'].items():
        path=ROOT/relative
        if sha(path)!=binding['sha256'] or path.stat().st_size!=binding['bytes']:
            raise ValueError('Frozen map source/input changed: '+relative)
    for group in ('source_sha256','input_sha256'):
        for relative,digest in registered['protocol'][group].items():
            if sha(ROOT/relative)!=digest:raise ValueError('Registered root source/input changed: '+relative)


def triangle_xy_distance(points,triangles):
    """Retained I73 exact point-to-closed-triangle XY distance calculation."""
    answer=np.full(len(points),np.inf)
    for first in range(0,len(points),80):
        p=points[first:first+80,None,:]
        distance=np.full((len(p),len(triangles)),np.inf);cross=[]
        for k in range(3):
            a,b=triangles[None,:,k,:],triangles[None,:,(k+1)%3,:]
            edge,delta=b-a,p-a;length2=np.sum(edge*edge,axis=-1)
            ratio=np.sum(delta*edge,axis=-1)/np.where(length2>0,length2,1.)
            closest=a+np.clip(ratio,0.,1.)[...,None]*edge
            distance=np.minimum(distance,np.sum((p-closest)**2,axis=-1))
            cross.append(edge[...,0]*delta[...,1]-edge[...,1]*delta[...,0])
        signed=np.stack(cross,axis=-1)
        area2=((triangles[:,1,0]-triangles[:,0,0])*(triangles[:,2,1]-triangles[:,0,1])-
               (triangles[:,1,1]-triangles[:,0,1])*(triangles[:,2,0]-triangles[:,0,0]))
        inside=(np.all(signed>=0.,axis=-1)|np.all(signed<=0.,axis=-1))&(area2[None,:]!=0.)
        distance[inside]=0.
        answer[first:first+len(p)]=np.sqrt(np.min(distance,axis=1))
    return answer


def execute(stage):
    registered=json.loads((stage/'registered_before_arithmetic.json').read_text())
    verify_bindings(registered)
    if (stage/'report.json').exists():raise FileExistsError('Preserve the existing map stage')
    protocol=registered['protocol'];validate_protocol(protocol)
    spec=importlib.util.spec_from_file_location('_i76_frozen_map_routes',BASE/'runtime/route.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    from spherical_wheel.gazebo.mars import MarsYard,feature_triangles
    yard=MarsYard();features=feature_triangles();reports={};ground_fixtures={}
    sampling=protocol['sampling'];screens=protocol['screens']
    for route_id in ROUTE_IDS:
        path=module.make_route(route_id)
        if not np.array_equal(path.origin,protocol['origin_by_route_m'][route_id]):raise ValueError('Route origin differs')
        if not np.array_equal(path.anchor_origin,protocol['anchor_m']):raise ValueError('Route anchor differs')
        fixture=module.samples(route_id,count=sampling['route_count'],
            corridor_radius_m=sampling['corridor_radius_m'],corridor_count=sampling['circumference_samples'])
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
            distance=triangle_xy_distance(route,tri[keep]) if keep.any() else np.full(len(route),1.)
            clearance=float(distance.min());minimum_clearance=min(minimum_clearance,clearance)
            fixture[name+'_xy_clearance_lower_bound_m']=np.minimum(distance,1.)
            details[name]=dict(total_triangles=len(tri),nearby_triangles=int(keep.sum()),
                minimum_sampled_route_xy_clearance_m=clearance,lower_bound_m=min(clearance,1.),
                exact_below_one_m=clearance<1.)
        relief=float(np.ptp(planes[:,0]))
        gates=dict(corridor_within_dem=bool((np.abs(corridor)<=yard.size/2).all()),
            max_corridor_facet_slope=bool(slopes.max()<=screens['maximum_corridor_facet_slope_deg']),
            corridor_height_range=relief<=screens['maximum_corridor_total_height_relief_m'],
            feature_xy_clearance=minimum_clearance>=screens['minimum_route_feature_xy_clearance_m'])
        fixture.update(corridor_height_m=planes[:,0],corridor_slope_deg=slopes,
            route_height_m=route_planes[:,0],route_slope_deg=route_slopes)
        route_stage=stage/route_id;route_stage.mkdir()
        np.savez_compressed(route_stage/'map_samples.npz',**fixture)
        times=np.linspace(path.start_s,path.start_s+path.duration_s,7)
        ground=dict(route_id=route_id,route=path.spec(),requested_duration_s=path.total_duration_s,
            control_interval_s=.005,ground_check_times_s=times.tolist(),
            ground_check_points_xy_m=[path(t)[0].tolist() for t in times],
            visual_bounds_xy_m=list(module.world_bounds(route_id)),
            unchanged_ground_check='Original Gazebo verify_ground median<=200um and at least50% points within200um; no tolerance change.',
            classification='Planned map points only; no Gazebo ground verification has run yet.')
        save(route_stage/'ground_verification_fixture.json',ground);ground_fixtures[route_id]=ground
        report=dict(iteration='I-76',route_id=route_id,no_physics=True,fixture_only=True,
            route=path.spec(),passed=all(gates.values()),gates=gates,fixed_screen=screens,
            corridor_radius_m=sampling['corridor_radius_m'],
            route_xy_bounds_m=[float(route[:,0].min()),float(route[:,0].max()),float(route[:,1].min()),float(route[:,1].max())],
            world_visual_bounds_m=list(module.world_bounds(route_id)),
            corridor_relief_m=relief,corridor_height_range_m=[float(planes[:,0].min()),float(planes[:,0].max())],
            route_height_range_m=[float(route_planes[:,0].min()),float(route_planes[:,0].max())],
            route_slope_deg=dict(maximum=float(route_slopes.max()),p95=float(np.percentile(route_slopes,95))),
            corridor_slope_deg=dict(maximum=float(slopes.max()),p95=float(np.percentile(slopes,95))),
            maximum_sampled_reference_speed_m_s=float(np.linalg.norm(fixture['velocity_m_s'],axis=1).max()),
            maximum_sampled_reference_acceleration_m_s2=float(np.linalg.norm(fixture['acceleration_m_s2'],axis=1).max()),
            sampled_polyline_length_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()),features=details,
            limitations=['Sampled route/circumference corridor, not a continuous safety certificate.',
                'Feature clearance is conservative2D projection, capped at1m lower bound.',
                'Map selects a fixed offline fixture; the controller receives no map.',
                'Geometric screens imply no actuation feasibility or dynamic result.'])
        save(route_stage/'report.json',report);reports[route_id]=report
    verify_bindings(registered)
    report=dict(iteration='I-76',passed=all(r['passed'] for r in reports.values()),routes=reports,
        no_physics=True,no_case_outcomes_read=True,no_site_search=True,source_and_inputs_unchanged=True,
        anchor_origin_m=protocol['anchor_m'],route_origin_by_id_m=protocol['origin_by_route_m'],geometry_heading_deg=113.,
        dem=dict(path=str(INPUTS[0].relative_to(ROOT)),sha256=sha(INPUTS[0]),shape=list(yard.heights.shape),
            bounds_xy_m=[-yard.size/2,yard.size/2]*2,cell_m=yard.spacing),
        all_routes_visual_bounds_xy_m=list(module.world_bounds()),
        failure_policy='Preserve every failed route and await root decision before physics; do not relocate or relax screens.')
    save(stage/'report.json',report)
    save(stage/'ground_verification_fixtures.json',ground_fixtures)
    save(stage/'artifact_manifest.json',dict(files={str(p.relative_to(stage)):dict(sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(stage.rglob('*')) if p.is_file()},inputs_unchanged=True,no_physics=True))
    print(json.dumps({r:dict(passed=v['passed'],gates=v['gates'],corridor_relief_m=v['corridor_relief_m'],
        maximum_facet_slope_deg=v['corridor_slope_deg']['maximum']) for r,v in reports.items()},indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',type=Path,required=True);parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--register',action='store_true');parser.add_argument('--run',action='store_true')
    args=parser.parse_args()
    if args.register:register(args.stage.resolve(),args.protocol.resolve())
    if args.run:execute(args.stage.resolve())
