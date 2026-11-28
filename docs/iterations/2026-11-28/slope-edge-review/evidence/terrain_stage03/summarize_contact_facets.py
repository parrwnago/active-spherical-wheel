"""Supplement closed terrain_stage02 using existing output, no physics."""
from pathlib import Path
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SOURCE = OUT.parent/'terrain_stage02/frames.json'
MODEL = ROOT/'results/tangent_heading_routes_20261127/campaign/_workspaces/eight_m100_N/mirror/model.xml'
INPUTS = [SOURCE,MODEL]


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(name,value):(OUT/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def main():
    assert not (OUT/'registered_before_arrays.json').exists()
    bound=[dict(path=str(p.relative_to(ROOT)),sha256=sha(p),bytes=p.stat().st_size) for p in INPUTS]
    save('registered_before_arrays.json',dict(iteration='I-80',scope='Posthoc narrow contact-facet/coordinate closure using closed stage02 geometry, no new physics.',inputs=bound,source_sha256=sha(Path(__file__)),selected_times_s=[100.5,100.8,100.835,100.85,101.,101.5,101.65]))
    frames=json.loads(SOURCE.read_text());tree=ET.parse(MODEL)
    fork=tree.find(".//worldbody/body[@name='fork']");carrier=fork.find("body[@name='carrier']")
    assert fork.find("freejoint[@name='base']") is not None
    assert carrier.attrib.get('pos','0 0 0')=='0 0 0'
    def context(row):
        p=max(row['pressure_patch_means'],key=lambda x:x['load_N'])
        gradient=np.asarray(p['dem']['gradient']);vel=np.asarray(row['linear_velocity_m_s']);denom=np.linalg.norm(gradient)*np.linalg.norm(vel[:2])
        angle=None if denom==0 else math.degrees(math.acos(float(np.clip((-gradient)@vel[:2]/denom,-1.,1.))))
        return dict(time_s=row['time_s'],root_position_m=row['center_m'],world_linear_velocity_m_s=row['linear_velocity_m_s'],
                    primary_pressure_patch=p['index'],primary_pressure_load_N=p['load_N'],
                    pressure_mean_point_m=p['point_m'],pressure_mean_minus_dem_height_m=p['point_minus_dem_m'],
                    pressure_mean_dem_facet=p['dem']['facet'],pressure_mean_dem_normal=p['dem']['normal'],
                    pressure_mean_dem_slope_deg=p['dem']['slope_deg'],
                    actual_xy_direction_from_downhill_deg=angle,
                    dem_tangent_height_rate_for_actual_xy_m_s=float(gradient@vel[:2]),
                    actual_vz_minus_dem_tangent_rate_m_s=float(vel[2]-gradient@vel[:2]))
    all_rows=[context(x) for x in frames if x['pressure_patch_means']]
    transitions=[]
    for a,b in zip(all_rows,all_rows[1:]):
        if a['pressure_mean_dem_facet']!=b['pressure_mean_dem_facet']:
            normal0=np.array(a['pressure_mean_dem_normal']);normal1=np.array(b['pressure_mean_dem_normal'])
            transitions.append(dict(time_s=b['time_s'],from_facet=a['pressure_mean_dem_facet'],to_facet=b['pressure_mean_dem_facet'],
                    from_slope_deg=a['pressure_mean_dem_slope_deg'],to_slope_deg=b['pressure_mean_dem_slope_deg'],
                    normal_change_deg=math.degrees(math.acos(float(np.clip(normal0@normal1,-1,1))))))
    selected=[min(all_rows,key=lambda x:abs(x['time_s']-when)) for when in [100.5,100.8,100.835,100.85,101.,101.5,101.65]]
    report=dict(iteration='I-80',scope='Loaded patch-mean geometry and model-coordinate review, not individual collision reconstruction or causal attribution.',
        coordinate_proof=dict(model=str(MODEL.relative_to(ROOT)),free_joint_body='fork',free_joint='base',
            fork_body_attributes=fork.attrib,carrier_body_attributes=carrier.attrib,
            carrier_offset_in_fork_m=[0,0,0],
            interpretation='The freejoint body origin and carrier/spherical-shell centre coincide; platform is offset above this origin. World root-position changes are wheel-centre changes, not platform-height changes.'),
        fixed_selected_rows=selected,pressure_mean_facet_transitions=transitions,
        observations=dict(nearest_facet_transition_to_100p835=min(transitions,key=lambda x:abs(x['time_s']-100.835)),
            transitions_within_100p5_101p65=[x for x in transitions if 100.5<=x['time_s']<=101.65]),
        qualifications=['Loaded per-patch pressure mean can be an aggregate. Its DEM facet is not identification of every individual collision facet.',
                        'A grade break in a shared-vertex heightfield is continuous height, not a cliff/unsupported freefall.',
                        'Actual XY velocity alignment with downhill supports terrain-following characterization; gravity causality or motor fault is not isolated by this geometry calculation.'])
    save('report.json',report)
    assert all(sha(ROOT/x['path'])==x['sha256'] for x in bound)
    save('artifact_manifest.json',dict(inputs_unchanged=True,no_new_physics=True,files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size) for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps({'selected':selected,'transitions':transitions,'coordinate':report['coordinate_proof']}))


if __name__=='__main__':main()
