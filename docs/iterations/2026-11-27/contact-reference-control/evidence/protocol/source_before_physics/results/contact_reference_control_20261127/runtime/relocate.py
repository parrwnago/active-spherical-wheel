"""I73 fixed-order, first-passing map-only relocation; no control outcomes."""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil
import numpy as np
import preflight
from route import SmoothSPath, samples

ROOT, BASE = preflight.ROOT, preflight.BASE


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--stage',type=Path,required=True)
    args=parser.parse_args()
    protocol_path=args.protocol.resolve();stage=args.stage.resolve()
    stage.mkdir(parents=True,exist_ok=False)
    paths=[Path(__file__).resolve(),*preflight.SOURCES,*preflight.INPUTS,
           protocol_path,BASE/'protocol/map_stage01.json']
    bindings={str(p.relative_to(ROOT)):dict(sha256=preflight.sha(p),bytes=p.stat().st_size) for p in paths}
    for p in [Path(__file__).resolve(),*preflight.SOURCES]:
        dest=stage/'source'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
    registration=dict(registered_utc=datetime.now(timezone.utc).isoformat(),
                      before_map_arithmetic=True,paths=bindings,
                      no_physics=True,no_control_outcomes=True)
    preflight.save(stage/'registered_before_arithmetic.json',registration)
    protocol=json.loads(protocol_path.read_text())
    from spherical_wheel.gazebo.mars import MarsYard,feature_triangles
    yard=MarsYard();features=feature_triangles()
    source_origin=SmoothSPath().origin
    base=samples()
    screens=protocol['screens']
    evaluated=[];selected=None
    for ordinal,origin in enumerate(protocol['origin_order']):
        delta=np.asarray(origin,float)-source_origin
        route=base['xy_m']+delta;corridor=base['corridor_xy_m']+delta
        planes=np.array([yard._plane(x,y) for x,y in corridor])
        slopes=np.rad2deg(np.arctan(np.linalg.norm(planes[:,1:],axis=1)))
        relief=float(np.ptp(planes[:,0]));clearance=np.inf;feature_summary={}
        low,high=route.min(axis=0)-1.,route.max(axis=0)+1.
        for name,tri3 in features.items():
            tri=tri3[:,:,:2]
            keep=np.all(tri.max(axis=1)>=low,axis=1)&np.all(tri.min(axis=1)<=high,axis=1)
            value=float(preflight.triangle_xy_distance(route,tri[keep]).min()) if keep.any() else 1.
            lower_bound=min(value,1.)
            feature_summary[name]=dict(xy_clearance_lower_bound_m=lower_bound,
                                       nearby_triangle_count=int(keep.sum()),
                                       exact_below_one_m=value<1.)
            clearance=min(clearance,lower_bound)
        gates=dict(corridor_within_dem=bool((np.abs(corridor)<=yard.size/2).all()),
                   max_corridor_facet_slope=bool(slopes.max()<=screens['maximum_corridor_facet_slope_deg']),
                   corridor_height_range=bool(relief<=screens['maximum_corridor_total_height_relief_m']),
                   feature_xy_clearance=bool(clearance>=screens['minimum_route_feature_xy_clearance_m']))
        row=dict(ordinal=ordinal,origin_m=origin,
                 maximum_corridor_facet_slope_deg=float(slopes.max()),
                 p95_corridor_facet_slope_deg=float(np.percentile(slopes,95)),
                 corridor_height_range_m=[float(planes[:,0].min()),float(planes[:,0].max())],
                 corridor_relief_m=relief,feature_clearance_lower_bound_m=clearance,
                 features=feature_summary,gates=gates,passed=all(gates.values()))
        evaluated.append(row)
        print(json.dumps(row),flush=True)
        if row['passed']:
            selected=dict(ordinal=ordinal,origin_m=origin)
            break
    unchanged=all(preflight.sha(ROOT/p)==b['sha256'] for p,b in bindings.items())
    if not unchanged:raise ValueError('Registered relocation inputs changed')
    report=dict(iteration='I-73',no_physics=True,fixture_only=True,
                selected=selected,evaluated=evaluated,
                total_registered_origins=len(protocol['origin_order']),
                evaluated_count=len(evaluated),stopped_at_first_pass=selected is not None,
                unchanged_screens=screens,registered_inputs_unchanged=unchanged,
                no_controller_map_input=True,no_controller_performance_selection=True,
                prior_failed_stage_preserved='runtime/map_stage01',
                limitations=['Sampled corridor, not continuous clearance certification.',
                             'Projected feature distances conservatively include buried geometry.',
                             'No simulator or actuator performance is inferred.'])
    preflight.save(stage/'report.json',report)
    artifacts={n:dict(sha256=preflight.sha(stage/n),bytes=(stage/n).stat().st_size)
               for n in ['registered_before_arithmetic.json','report.json']}
    preflight.save(stage/'artifact_manifest.json',dict(files=artifacts,inputs_unchanged=True))
    print(json.dumps(dict(selected=selected,evaluated_count=len(evaluated)),indent=2),flush=True)


if __name__=='__main__':main()
