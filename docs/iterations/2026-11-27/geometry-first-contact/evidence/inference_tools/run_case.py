"""I72 shadow consumer; native family is available only for final fallback."""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json
import sys
import time
import numpy as np

from geometry_first_contact import geometry_first_rank
import frozen_i60_rank


ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'inputs'))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path,value):Path(path).write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def chosen_point(rank,world_points,candidate_ids,family_index,selected_mask):
    n=len(world_points);family=np.asarray(rank['chosen_family'],int)
    if family.shape!=(n,) or not np.isin(family,np.arange(-1,4)).all():raise ValueError('Invalid raw ranked families')
    points=np.full((n,3),np.nan);ids=np.full(n,-1,np.int64);rows=np.full(n,-1,np.int64)
    known=np.flatnonzero(family>=0)
    if len(known):
        row=rank['candidate_index'][known,family[known]]
        if np.any(row<0) or np.any(row>=world_points.shape[1]):raise ArithmeticError('Ranked family has no physical row')
        if not selected_mask[known,row].all() or not np.array_equal(family_index[row],family[known]):
            raise ArithmeticError('Ranked point changed selected physical family')
        points[known]=world_points[known,row];rows[known]=row;ids[known]=candidate_ids[row]
        if not np.isfinite(points[known]).all():raise ArithmeticError('Ranked physical point unavailable')
    return points,ids,rows


def infer_case(data):
    """Current-frame independent batch; fallback fields never enter ranker."""
    times=np.asarray(data['time_s']);n=len(times)
    if times.shape!=(n,) or n==0 or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
        raise ValueError('Nonempty finite increasing recorded times required')
    # No native_fallback member is accessed until geometry/ranking completes.
    selection=geometry_first_rank(data['projected_features'],data['world_points'],
        data['native_anchor_point_m'],data['candidate_ids'],rank_hypotheses=frozen_i60_rank.rank_hypotheses)
    output=dict(time_s=times.copy())
    for key in ('selected_candidate_id','selected_row_index','selected_mask','family_distance_squared_m2',
                'anchor_available','legacy_active_mask'):
        output[key]=selection[key]
    points=np.asarray(data['world_points'],float);ids=np.asarray(data['candidate_ids'],np.int64)
    family=np.asarray(data['projected_features']['family_index'],int)
    for mode,rank in selection['ranking'].items():
        for key,value in rank.items():output[mode+'__'+key]=value
        point,ident,row=chosen_point(rank,points,ids,family,selection['selected_mask'])
        output[mode+'__point_m']=point;output[mode+'__chosen_candidate_id']=ident;output[mode+'__chosen_row_index']=row
        output[mode+'__accepted']=np.asarray(rank['accepted_family'])>=0
    fallback=data['native_fallback']
    if not np.array_equal(times,fallback['time_s']):raise ValueError('Native fallback time differs')
    native_family=np.asarray(fallback['native_family']);native_point=np.asarray(fallback['native_point_m'],float)
    if (native_family.shape!=(n,) or native_point.shape!=(n,3) or not np.isfinite(native_family).all()
            or not np.equal(native_family,np.floor(native_family)).all() or not np.isin(native_family,np.arange(-1,4)).all()):
        raise ValueError('Invalid native fallback shape/family')
    output['native_family']=native_family.astype(np.int64,copy=True)
    output['native_point_m']=native_point.copy()
    for prefix,mode in (('C','fused'),('C_VG','velocity')):
        accepted=output[mode+'__accepted']
        output[prefix+'_family']=np.where(accepted,output[mode+'__accepted_family'],native_family).astype(np.int64)
        output[prefix+'_point_m']=np.where(accepted[:,None],output[mode+'__point_m'],native_point)
        output[prefix+'_used_proposal']=accepted.copy()
    return output


def verify_seal(path):
    seal=json.loads(Path(path).read_text())
    for group in ('sources','input_files'):
        for relative,digest in seal[group].items():
            if sha(ROOT/relative)!=digest:raise AssertionError('Frozen source/input changed: '+relative)
    return seal


def run(case,input_root,output_root,seal_path):
    verify_seal(seal_path)
    from packet_io import load_case
    directory=Path(output_root)/case;directory.mkdir(parents=True,exist_ok=False)
    source=Path(input_root)/case
    save(directory/'registered_before_inference.json',dict(iteration='I-72',case=case,
        utc=datetime.now(timezone.utc).isoformat(),source_input_seal_sha256=sha(seal_path),
        input_files={p.name:sha(p) for p in source.iterdir() if p.is_file()},
        frozen_rank_sha256=sha(frozen_i60_rank.__file__),
        primary='C=fused accepted family+raw physical point, otherwise exact native family+equivalent point.',
        secondary=['Raw fused','Raw velocity+geometry','C_VG velocity accepted/native fallback','Geometry/wrench diagnostics'],
        boundary='Native family/load fields are consumed only after geometry_first_rank returns. No labels, future data or new geometry.',
        ranking='Exact frozen I60rank_hypotheses default scales/confidence; no options changed.'))
    data=load_case(input_root,case)
    if len(data['time_s'])!=8801 or np.shape(data['world_points'])!=(8801,102,3):raise ValueError('Registered original full-case dimensions required')
    started=time.monotonic();arrays=infer_case(data)
    np.savez_compressed(directory/'predictions.npz',**arrays)
    verify_seal(seal_path)
    save(directory/'completed.json',dict(case=case,frames=len(arrays['time_s']),elapsed_s=time.monotonic()-started,
        source_input_seal_unchanged=True,truth_read=False,physics_run=False,force_refit=False,
        temporal_history=False,default_changed=False))
    save(directory/'artifact_manifest.json',dict(files=[dict(path=p.name,bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(directory.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(case=case,completed=True,frames=len(arrays['time_s'])),sort_keys=True),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',choices=['m100_B','p030_B','p090_B'],required=True)
    for key in ('input-root','output-root','source-input-seal'):p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args();run(args.case,args.input_root,args.output_root,args.source_input_seal)
