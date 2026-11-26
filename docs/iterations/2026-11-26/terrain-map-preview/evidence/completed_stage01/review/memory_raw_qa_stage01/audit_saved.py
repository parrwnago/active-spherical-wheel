"""Independent I44 saved-output checks; no render, controller or physics calls."""
from __future__ import annotations
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from spherical_wheel.terrain_memory import MemorySettings,TerrainMemory

HERE=Path(__file__).resolve().parent;FAMILY=HERE.parents[1];REPO=FAMILY.parents[1]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def load_npz(path):
    with np.load(path) as z:return {k:z[k].copy() for k in z.files}
def save(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def stats(values):
    values=np.array([x for x in values if x is not None and np.isfinite(x)],float)
    if not len(values):return {'n':0,'rmse':None,'p90_abs':None}
    return {'n':len(values),'rmse':float(np.sqrt(np.mean(values*values))),'p90_abs':float(np.quantile(abs(values),.9))}


def batch(depth,index):
    return tuple(SimpleNamespace(camera_name=str(depth['camera_names'][k]),depth_m=depth['depth_m'][index,k],
        valid_mask=depth['valid_mask'][index,k],intrinsics=depth['intrinsics'][k],world_from_camera=depth['world_from_camera'][index,k],
        capture_time_s=float(depth['capture_time_s'][index]),delivered_at_s=float(depth['delivered_at_s'][index])) for k in range(4))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--analysis',type=Path,required=True);parser.add_argument('--depth',type=Path,required=True)
    args=parser.parse_args();analysis=args.analysis.resolve();depth_stage=args.depth.resolve()
    if (HERE/'report.json').exists():raise FileExistsError('Preserve existing raw QA stage')
    for path in [analysis/'artifact_manifest.json',analysis/'report.json',analysis/'predictions.json',depth_stage/'report.json']:
        if not path.is_file():raise FileNotFoundError(path)
    source=HERE/'source';source.mkdir(exist_ok=False)
    copied=[]
    for path in [Path(__file__),REPO/'src/spherical_wheel/terrain_memory.py',REPO/'scripts/run-terrain-memory.py',FAMILY/'protocol.json']:
        target=source/path.name;shutil.copyfile(path,target);copied.append({'origin':str(path),'snapshot':str(target.relative_to(HERE)),'sha256':sha(path)})
    save(HERE/'declaration_before_qa.json',{'declared_utc':datetime.now(timezone.utc).isoformat(),
        'scope':'Post-primary independent saved-data integrity/arithmetic QA, no parameter changes or robot physics.',
        'bounded_reingestion':'First0.4s anchor accumulated prefix; latest-four batches at0.4,6.8,43.6s. No full-map or depth-render repeat.',
        'sources':copied,'analysis_report_sha256':sha(analysis/'report.json'),'depth_report_sha256':sha(depth_stage/'report.json')})
    manifest=json.loads((analysis/'artifact_manifest.json').read_text())
    for name,item in manifest.items():assert sha(analysis/name)==item['sha256']
    protocol=json.loads((FAMILY/'protocol.json').read_text());report=json.loads((analysis/'report.json').read_text())
    rows=json.loads((analysis/'predictions.json').read_text());anchors=json.loads((analysis/'anchor_records.json').read_text())
    depth_report=json.loads((depth_stage/'report.json').read_text())
    for name,expected in depth_report['outputs_sha256'].items():assert sha(depth_stage/name)==expected
    depth=load_npz(depth_stage/'depth_batches.npz');cells=load_npz(analysis/'memory_snapshots.npz')
    exact=load_npz(REPO/protocol['precision']);provenance=json.loads((REPO/protocol['provenance']).read_text())
    for name,expected in protocol['inputs_sha256'].items():assert sha(REPO/name)==expected
    assert len(anchors)==119 and len(rows)==119*3*3
    settings=MemorySettings(**{k:v for k,v in protocol['map'].items() if k not in ('unknown_rule','static_world_assumption')})
    # Independently reconstruct every capture transform using the registered rigid mount.
    capture_error=0.
    for i,index in enumerate(depth['source_indices']):
        q=exact['exact_qpos'][index];body=np.eye(4);body[:3,:3]=Rotation.from_quat(q[[4,5,6,3]]).as_matrix();body[:3,3]=q[:3]
        for k in range(4):capture_error=max(capture_error,float(abs(body@depth['body_from_camera'][k]-depth['world_from_camera'][i,k]).max()))
        assert depth['capture_time_s'][i]==exact['time_s'][index]
    assert capture_error<1e-12
    pointer_checks=[];snapshot_checks=[];prefix_cells=None
    for ai,anchor in enumerate(anchors):
        now=anchor['anchor_time_s'];available=np.flatnonzero(depth['delivered_at_s']<=now+1e-9)
        count=len(available);assert anchor['ingested_batches']==count and anchor['latest_batch_index']==count-1
        assert anchor['maximum_ingested_delivery_s']==float(depth['delivered_at_s'][available[-1]])
        assert anchor['maximum_ingested_capture_s']==float(depth['capture_time_s'][available[-1]])
        assert anchor['maximum_ingested_delivery_s']<=now+1e-9
        a,b=cells['offsets'][ai:ai+2];part={k:cells[k][a:b] for k in ('centers_m','count','first_capture_s','last_capture_s','age_s','height_spread_m')}
        assert len(part['centers_m'])==anchor['memory_cell_count']
        assert np.all(part['first_capture_s']<=part['last_capture_s']) and np.all(part['last_capture_s']<=anchor['maximum_ingested_capture_s']+1e-9)
        np.testing.assert_allclose(part['age_s'],now-part['last_capture_s'],atol=1e-12,rtol=0.)
        assert np.all(part['count']>0) and np.all(part['height_spread_m']>=0)
        pointer_checks.append(count);snapshot_checks.append(len(part['centers_m']))
        if ai==0:prefix_cells=part
    # Reingest only the first prefix: stored snapshots cannot come from a future-complete map.
    first=anchors[0];memory=TerrainMemory(settings)
    for i in range(first['ingested_batches']):memory.ingest(batch(depth,i),available_at_s=first['anchor_time_s'])
    reconstructed=memory.observed_cells(first['anchor_time_s'])
    for key,value in prefix_cells.items():np.testing.assert_array_equal(reconstructed[key],value)
    # Latest-four maps at three fixed anchors verify correct batch selection and capture registration.
    latest_checks=0
    for stamp in (.4,6.8,43.6):
        anchor=min(anchors,key=lambda x:abs(x['anchor_time_s']-stamp));latest=TerrainMemory(settings)
        latest.ingest(batch(depth,anchor['latest_batch_index']),available_at_s=anchor['anchor_time_s'])
        for row in rows:
            if row['anchor_index']!=anchor['anchor_index'] or row['arm']!='latest_four':continue
            query=latest.query(np.asarray(row['shared_cv_qpos'])[:2],anchor['anchor_time_s']);saved=row['surface_query']
            assert query['supported']==saved['supported'] and query['reason']==saved['reason']
            if query['supported']:
                assert abs(query['height_m']-saved['height_m'])<1e-12
                np.testing.assert_allclose(query['normal'],saved['normal'],atol=1e-12,rtol=0.)
            latest_checks+=1
    # Every row shares CV, and future labels remain confined to scoring.
    max_cv_rotation=0.;max_candidate_radius=0.;max_height_error_gap=0.;max_normal_error_gap=0.;truth_invalid=0
    threshold=max(.05,.05*provenance['weight_N']);radius=.12
    groups={};loaded_multi=0;geometry_error_values={};macro_recall={arm:[] for arm in report['arms']}
    for row in rows:
        index=row['anchor_index'];h=row['horizon_s'];q=exact['exact_qpos'][index];v=exact['exact_qvel'][index];cv=np.asarray(row['shared_cv_qpos'])
        np.testing.assert_allclose(cv[:3],q[:3]+h*v[:3],atol=1e-14,rtol=0.);np.testing.assert_allclose(cv[7:],q[7:]+h*v[6:],atol=1e-14,rtol=0.)
        expected=Rotation.from_quat(q[[4,5,6,3]])*Rotation.from_rotvec(v[3:6]*h)
        actual=Rotation.from_quat(cv[[4,5,6,3]])
        max_cv_rotation=max(max_cv_rotation,float(abs(expected.as_matrix()-actual.as_matrix()).max()))
        groups.setdefault((index,h),[]).append(row)
        future=int(np.argmin(abs(exact['time_s']-(row['anchor_time_s']+h))));assert row['future_index']==future
        loads=exact['truth_patch_loads_N'][future];labels=[('shell_A','shell_B','belt_A','belt_B')[k] for k in np.flatnonzero(loads>=threshold)]
        assert row['true_loaded_patches']==labels
        point=exact['truth_equivalent_point_m'][future];eligible=bool(labels and np.isfinite(point).all())
        assert row['future_point_eligible']==eligible
        candidates=row['contact_preview']['candidates'];patches=sorted({c['patch'] for c in candidates});assert patches==row['candidate_patches']
        distances=[float(np.linalg.norm(np.asarray(c['point_m'])-point)) for c in candidates]
        distance=min(distances) if eligible and distances else None
        assert distance==row['nearest_candidate_error_m']
        for candidate in candidates:
            max_candidate_radius=max(max_candidate_radius,abs(float(np.linalg.norm(np.asarray(candidate['point_m'])-cv[:3]))-radius))
            assert candidate['contact_surface_query']['at_time_s']==row['anchor_time_s']
            assert candidate['clearance_m']<=.005+1e-12
        if labels:
            macro_recall[row['arm']].append(len(set(labels)&set(patches))/len(labels))
            loaded_multi+=int(len(labels)>1)
        score=row['surface_score'];query=row['surface_query'];truth=row['surface_truth']
        assert score['supported']==query['supported'];truth_invalid+=not truth['valid']
        if query['supported'] and truth['valid']:
            error=query['height_m']-truth['point_m'][2];max_height_error_gap=max(max_height_error_gap,abs(error-score['height_error_m']))
            if not truth['normal_ambiguous']:
                a=np.asarray(query['normal']);b=np.asarray(truth['normal']);angle=float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b)),a@b)))
                max_normal_error_gap=max(max_normal_error_gap,abs(angle-score['normal_error_deg']))
            else:assert score['normal_error_deg'] is None
        else:assert score['height_error_m'] is None and score['normal_error_deg'] is None
    assert max_cv_rotation<1e-12 and max_candidate_radius<1e-12 and max_height_error_gap<1e-12 and max_normal_error_gap<1e-5
    for group in groups.values():
        assert len(group)==3
        for row in group[1:]:np.testing.assert_array_equal(row['shared_cv_qpos'],group[0]['shared_cv_qpos'])
    # Recompute registered terrain utility gates from all uniform query slots.
    independently={}
    for arm in report['arms']:
        selected=[r for r in rows if r['arm']==arm and 'uniform' in r['strata']]
        heights=[r['surface_score']['height_error_m'] for r in selected];normals=[r['surface_score']['normal_error_deg'] for r in selected]
        independently[arm]={'slots':len(selected),'supported':sum(r['surface_score']['supported'] for r in selected),
            'height':stats(heights),'normal':stats(normals)}
        saved=report['summary']['uniform'][arm]['all_horizons'];assert saved['query_slots']==len(selected)
        assert saved['supported']==independently[arm]['supported']
        for key,value in [('height_error_m',independently[arm]['height']),('normal_error_deg',independently[arm]['normal'])]:
            assert saved[key]['n']==value['n']
            for metric in ('rmse','p90_abs'):
                assert (saved[key][metric] is None and value[metric] is None) or abs(saved[key][metric]-value[metric])<1e-12
    limit=protocol['predeclared_gates']['memory_utility_uniform'];m=independently['memory_four'];l=independently['latest_four']
    gates={'coverage':m['supported']/m['slots']>=limit['coverage_min'],
        'coverage_gain_vs_latest':m['supported']/m['slots']-l['supported']/l['slots']>=limit['coverage_gain_vs_latest_min'],
        'height_rmse':m['height']['rmse'] is not None and m['height']['rmse']<=limit['covered_height_rmse_m_max'],
        'normal_p90':m['normal']['p90_abs'] is not None and m['normal']['p90_abs']<=limit['covered_normal_p90_deg_max']}
    assert gates==report['utility_gates'] and all(gates.values())==report['utility_passed']
    output={'iteration':'I-44','scope':'Post-primary independent saved-data QA;0 rendering/controller/physics calls.',
        'passed':True,'analysis_report_sha256':sha(analysis/'report.json'),'raw_query_rows':len(rows),'unique_shared_query_slots':len(groups),
        'anchor_prefixes_checked':len(anchors),'capture_transform_max_gap':capture_error,'first_prefix_snapshot_exact':True,
        'latest_four_fixed_spot_queries':latest_checks,'snapshot_cell_counts_range':[min(snapshot_checks),max(snapshot_checks)],
        'max_CV_rotation_matrix_gap':max_cv_rotation,'max_candidate_radius_gap_m':max_candidate_radius,
        'max_saved_height_error_gap_m':max_height_error_gap,'max_saved_normal_angle_gap_deg':max_normal_error_gap,
        'truth_invalid_rows':truth_invalid,'independent_uniform_summary':independently,'utility_gates':gates,
        'all_query_macro_loaded_patch_recall':{a:float(np.mean(v)) if v else None for a,v in macro_recall.items()},
        'multi_loaded_rows_including_repeated_arms':loaded_multi,
        'limitations':['Normal truth remains the shared compiled-mesh face proxy, not a force-weighted normal.',
            'Partial confirmed candidate lists do not establish complete contact-set support; overlap is not force prediction.',
            'First-cell observation lead is not first complete plane/normal availability.','No parameters or primary results changed.']}
    save(HERE/'report.json',output)
    files=sorted(p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    save(HERE/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(HERE)),'sha256':sha(p),'bytes':p.stat().st_size} for p in files]})
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
