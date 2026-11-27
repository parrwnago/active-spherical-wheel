"""I71 shadow-only consumer of sealed I70 costs and endpoint points.

No physics, model fitting, NNLS, labels, native routing or new measurements.
"""
from dataclasses import asdict
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json
import time
import numpy as np

import frozen_temporal_contact_filter as frozen
from restricted_filter import ALLOWED_MODES,PAIR_MODES,RestrictedTemporalFilter,frame_update


ROOT=Path(__file__).resolve().parents[3]
PAIR_SETS=tuple(frozen.SUPPORT_SETS[4:10])
SLOT_FAMILY=np.array([0,1,2,3,0,1])


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    with np.load(path,allow_pickle=False) as z:return {key:z[key] for key in z.files}


def save(path,value):Path(path).write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def transition_key(previous,current):
    if previous not in range(8) or current not in range(8) or previous//2==current//2:return None
    union=tuple(sorted((previous//2,current//2)))
    return (PAIR_SETS.index(union),union.index(current//2),current%2)


def edge_evidence(node_cost,edge_cost):
    output=np.broadcast_to(-np.asarray(node_cost,float),(21,21)).copy()
    output[:,PAIR_MODES]=-np.inf
    for previous in range(8):
        for current in range(8):
            key=transition_key(previous,current)
            if key is not None and np.isfinite(edge_cost[key]):output[previous,current]=-edge_cost[key]
    return output


def validate_packet(packet):
    times=np.asarray(packet['time_s']);n=len(times)
    if times.shape!=(n,) or n==0 or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
        raise ValueError('Nonempty finite strictly increasing measured times required')
    shapes=dict(node_cost=(n,21),edge_cost=(n,6,2,2),node_valid=(n,21),edge_valid=(n,6,2,2),
        node_candidate_index=(n,21,2),edge_candidate_index=(n,6,2,2,2),
        node_endpoint_points_m=(n,21,3),edge_endpoint_points_m=(n,6,2,2,3))
    for key,shape in shapes.items():
        if np.shape(packet[key])!=shape:raise ValueError('Unexpected packet shape: '+key)
    for key in ('node_cost','edge_cost'):
        if np.isnan(packet[key]).any() or np.isneginf(packet[key]).any():raise ValueError('Invalid profile costs')
    if not np.isfinite(packet['node_cost'][:,20]).all():raise ValueError('Finite unknown potential required')
    if not np.array_equal(np.isfinite(packet['node_cost'][:,:8]),packet['node_valid'][:,:8]):
        raise ValueError('Singleton cost/fit validity differs')
    if not np.array_equal(np.isfinite(packet['edge_cost']),packet['edge_valid']):raise ValueError('Edge cost/fit validity differs')
    for mode in range(8):
        mask=packet['node_valid'][:,mode];ids=packet['node_candidate_index'][mask,mode,0]
        if np.any(ids<0) or np.any(ids>=102) or not np.equal(ids,np.floor(ids)).all() or np.any(SLOT_FAMILY[ids.astype(int)%6]!=mode//2):
            raise ValueError('Singleton candidate identity/family differs')
        if not np.isfinite(packet['node_endpoint_points_m'][mask,mode]).all():raise ValueError('Missing valid singleton endpoint')
    for pair,families in enumerate(PAIR_SETS):
        for endpoint in range(2):
            for motion in range(2):
                mask=packet['edge_valid'][:,pair,endpoint,motion]
                ids=packet['edge_candidate_index'][mask,pair,endpoint,motion,endpoint]
                if np.any(ids<0) or np.any(ids>=102) or not np.equal(ids,np.floor(ids)).all() or np.any(SLOT_FAMILY[ids.astype(int)%6]!=families[endpoint]):
                    raise ValueError('Directed edge endpoint identity/family differs')
                if not np.isfinite(packet['edge_endpoint_points_m'][mask,pair,endpoint,motion]).all():raise ValueError('Missing directed endpoint point')


def selected_geometry(packet,index,decision,*,use_edges):
    current=int(decision['selected_current_mode']);previous=int(decision['selected_previous_mode'])
    points=np.full((4,3),np.nan);ids=np.full(4,-1,np.int64);edge_index=-1
    if current==20:return dict(points_m=points,candidate_index=ids,selected_edge_index=edge_index)
    if current not in range(8):raise ArithmeticError('Disabled endpoint selected')
    family=current//2;key=transition_key(previous,current) if use_edges else None
    edge=key is not None and np.isfinite(packet['edge_cost'][index][key])
    if edge:
        points[family]=packet['edge_endpoint_points_m'][index][key]
        ids[family]=packet['edge_candidate_index'][index][key][key[1]]
        edge_index=int(np.ravel_multi_index(key,(6,2,2)))
    else:
        points[family]=packet['node_endpoint_points_m'][index,current]
        ids[family]=packet['node_candidate_index'][index,current,0]
    if not np.isfinite(points[family]).all() or ids[family]<0:raise ArithmeticError('Selected endpoint has no physical candidate')
    return dict(points_m=points,candidate_index=ids,selected_edge_index=edge_index)


def infer_packet(packet,progress=None):
    validate_packet(packet);state_filter=RestrictedTemporalFilter();edge_filter=RestrictedTemporalFilter();rows=[]
    for index,t in enumerate(packet['time_s']):
        cost=packet['node_cost'][index];edges=packet['edge_cost'][index]
        emission=frame_update(-cost);state=state_filter.update(float(t),-cost)
        hmm=edge_filter.update(float(t),-cost,edge_evidence(cost,edges))
        row=dict(time_s=t,emission_probability21=emission['probability'],
            state_hmm_probability21=state['probability'],posterior21=hmm['probability'],
            evidence_weight=hmm['evidence_weight'],history_reset=hmm['history_reset'])
        for prefix,decision,use_edges in (('emission',emission,False),('state_hmm',state,False),('hmm',hmm,True)):
            for key in ('support_index','confidence','selected_previous_mode','selected_current_mode'):
                row[prefix+'_'+key]=decision[key]
            for key,value in selected_geometry(packet,index,decision,use_edges=use_edges).items():row[prefix+'_'+key]=value
        rows.append(row)
        if progress is not None and index%500==0:progress(index+1,len(packet['time_s']))
    output={key:np.asarray([row[key] for row in rows]) for key in rows[0]}
    for key in ('emission_probability21','state_hmm_probability21','posterior21'):
        if np.any(output[key][:,PAIR_MODES]!=0.):raise ArithmeticError('Disabled probability leakage')
        np.testing.assert_allclose(output[key].sum(axis=1),1.,atol=1e-12,rtol=0.)
    return output


def verify_seal(path):
    seal=json.loads(Path(path).read_text())
    for group in ('sources','input_files'):
        for rel,digest in seal[group].items():
            if sha(ROOT/rel)!=digest:raise AssertionError('Frozen source/input changed: '+rel)
    return seal


def run(case,input_root,output_root,seal_path):
    verify_seal(seal_path);source=Path(input_root)/case/'inference_packet.npz'
    directory=Path(output_root)/case;directory.mkdir(parents=True,exist_ok=False)
    save(directory/'registered_before_inference.json',dict(iteration='I-71',case=case,
        utc=datetime.now(timezone.utc).isoformat(),source_input_seal_sha256=sha(seal_path),
        input_packet_sha256=sha(source),filter_settings=asdict(frozen.FilterSettings()),
        allowed_endpoint_modes=ALLOWED_MODES.tolist(),
        boundary='Sealed I70 costs and selected endpoint points only; no NNLS, labels, raw solver data or live calls.',
        transition='Original diagonal retention exact; off-diagonal mass redistributed among allowed destinations preserving original relative odds.',
        initial_prior='Original prior conditioned on allowed endpoints; pair endpoints unavailable, mixed-interval single-to-single edges retained.'))
    started=time.monotonic()
    def progress(done,total):print(json.dumps(dict(case=case,frames=done,total=total,elapsed_s=time.monotonic()-started)),flush=True)
    arrays=infer_packet(load(source),progress=progress)
    np.savez_compressed(directory/'predictions.npz',**arrays)
    verify_seal(seal_path)
    save(directory/'completed.json',dict(case=case,frames=len(arrays['time_s']),elapsed_s=time.monotonic()-started,
        disabled_pair_probability_exact_zero=True,truth_read=False,nnls_refit=False,physics_run=False,
        source_input_seal_unchanged=True))
    save(directory/'artifact_manifest.json',dict(files=[dict(path=p.name,bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(directory.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',choices=['m100_B','p030_B','p090_B'],required=True)
    for key in ('input-root','output-root','source-input-seal'):p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args();run(args.case,args.input_root,args.output_root,args.source_input_seal)
