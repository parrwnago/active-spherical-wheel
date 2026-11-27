"""Frozen I70 one-case shadow inference; no evaluation/truth files accepted."""
from pathlib import Path
from dataclasses import asdict
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,sys,time,traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'inputs'))
from input_features import frame_features
from spherical_wheel.shadow_contact_models import evaluate_frame,ModelConfig,PAIR_SETS,MODE_FAMILIES
from spherical_wheel.temporal_contact_filter import (TemporalContactFilter,FilterSettings,
    MODE_SUPPORTS,SUPPORT_SETS,MODE_NAMES,frame_sequence)


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):
    with np.load(p,allow_pickle=False) as z:return {key:z[key].copy() for key in z.files}
def save(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')


def transition_key(previous,current):
    if previous<0:return None
    old,new=MODE_SUPPORTS[previous],MODE_SUPPORTS[current]
    union=tuple(sorted(set(old)|set(new)))
    if len(new)!=1 or not old or old==new or len(union)!=2:return None
    return (PAIR_SETS.index(union),union.index(new[0]),current%2)


def edge_evidence(node_cost,edge_cost):
    output=np.broadcast_to(-np.asarray(node_cost,float),(21,21)).copy()
    for previous in range(21):
        for current in range(21):
            key=transition_key(previous,current)
            if key is not None and np.isfinite(edge_cost[key]):output[previous,current]=-edge_cost[key]
    return output


def selected_geometry(model,decision,*,use_edges):
    """One complete fit; endpoint support and interval forces stay distinct."""
    current=int(decision['selected_current_mode']);previous=int(decision['selected_previous_mode'])
    key=transition_key(previous,current) if use_edges else None
    edge=key is not None and np.isfinite(model['edge_cost'][key])
    storage=model['edge'] if edge else model['node'];index=key if edge else current
    families=PAIR_SETS[key[0]] if edge else MODE_SUPPORTS[current]
    point=np.full((4,3),np.nan);cov=np.full((4,3,3),np.nan);interval_force=np.full((4,3),np.nan)
    ids=np.full(4,-1,np.int64)
    if families and storage['valid'][index]:
        for slot,family in enumerate(families):
            interval_force[family]=storage['estimated_force_N'][index][slot]
            if family in MODE_SUPPORTS[current]:
                point[family]=storage['selected_point_m'][index][slot]
                cov[family]=storage['conditional_point_covariance_m2'][index][slot]
                ids[family]=storage['selected_candidate_index'][index][slot]
    return dict(points_m=point,conditional_point_covariance_m2=cov,interval_force_by_family_N=interval_force,
        candidate_index=ids,selected_edge_index=np.ravel_multi_index(key,(6,2,2)) if edge else -1)


class ShadowInference:
    def __init__(self):
        self.edge_filter=TemporalContactFilter();self.state_filter=TemporalContactFilter()

    def update(self,t,model):
        emission={k:v[0] for k,v in frame_sequence(-model['node_cost'][None]).items()}
        state=self.state_filter.update(t,-model['node_cost'])
        hmm=self.edge_filter.update(t,-model['node_cost'],edge_evidence(model['node_cost'],model['edge_cost']))
        row=dict(time_s=t,emission_probability21=emission['probability'],posterior21=hmm['probability'],
            state_hmm_probability21=state['probability'],evidence_weight=hmm['evidence_weight'],
            history_reset=hmm['history_reset'],hmm_support_index=hmm['support_index'],
            hmm_confidence=hmm['confidence'],hmm_selected_previous_mode=hmm['selected_previous_mode'],
            hmm_selected_current_mode=hmm['selected_current_mode'],force_fit_count=model['force_fit_count'],
            failed_force_fit_count=model['failed_force_fit_count'],shortlist_count=model['shortlist_count'],
            observation_valid=model['observation_valid'])
        for prefix,decision,use_edges in [('emission',emission,False),('state_hmm',state,False),('hmm',hmm,True)]:
            geo=selected_geometry(model,decision,use_edges=use_edges)
            for key,value in geo.items():row[prefix+'_'+key]=value
        evidence=dict(node_cost=model['node_cost'],edge_cost=model['edge_cost'],
            shortlist_candidate_index=model['shortlist_candidate_index'],
            node_candidate_index=model['node']['selected_candidate_index'],
            edge_candidate_index=model['edge']['selected_candidate_index'],
            node_valid=model['node']['valid'],edge_valid=model['edge']['valid'])
        return row,evidence


def run(case,input_root,output_root,seal_path):
    seal=json.loads(seal_path.read_text())
    for group in ('sources','input_files'):
        for rel,digest in seal[group].items():assert sha(ROOT/rel)==digest,(group,rel)
    directory=output_root/case;directory.mkdir(parents=True,exist_ok=False)
    source=input_root/case
    primitives,cached=load(source/'primitives.npz'),load(source/'features.npz')
    n=len(primitives['time_s']);assert n==8801
    for index,support in enumerate(MODE_SUPPORTS):
        assert tuple(MODE_FAMILIES[index][MODE_FAMILIES[index]>=0])==support,'Model/filter mode order differs'
    save(directory/'registered_before_inference.json',dict(iteration='I-70',case=case,
        registered_utc=datetime.now(timezone.utc).isoformat(),source_input_seal_sha256=sha(seal_path),
        inputs={str(p.relative_to(ROOT)):sha(p) for p in (source/'primitives.npz',source/'features.npz')},
        measurement_boundary='Whitelisted measured-state/observer/known-geometry arrays only; no evaluation packet, labels, terrain or references.',
        model_config={key:float(value) if isinstance(value,np.floating) else value for key,value in asdict(ModelConfig()).items()},
        filter_settings=asdict(FilterSettings()),mode_names=list(MODE_NAMES),
        scope='Offline shadow predictions; no robot motion or controller commands changed.'))
    rows=[];evidence=[];estimator=ShadowInference();started=time.monotonic()
    try:
        for index,t in enumerate(primitives['time_s']):
            features=frame_features(primitives,index,cached)
            duration=.005 if index==0 else float(t-primitives['time_s'][index-1])
            model=evaluate_frame(features['observation9'],features['points_m'],features['force_normals'],
                features['material_velocity_m_s'],features['force_maps'],features['family_index'],
                features['valid'],features['normal_hint'],features['candidate_index'],
                dt_s=duration,observation_valid=features['interval_valid'])
            row,ev=estimator.update(float(t),model);rows.append(row);evidence.append(ev)
            if index%500==0:print(json.dumps(dict(case=case,frames=index+1,total=n,elapsed_s=time.monotonic()-started)),flush=True)
        arrays={key:np.asarray([row[key] for row in rows]) for key in rows[0]}
        np.testing.assert_array_equal(arrays['time_s'],primitives['time_s'])
        for key in ('emission_probability21','posterior21','state_hmm_probability21'):
            assert np.isfinite(arrays[key]).all();np.testing.assert_allclose(arrays[key].sum(axis=1),1.,atol=1e-12)
        np.savez_compressed(directory/'predictions.npz',**arrays)
        np.savez_compressed(directory/'model_evidence.npz',**{key:np.asarray([row[key] for row in evidence]) for key in evidence[0]})
        for group in ('sources','input_files'):
            for rel,digest in seal[group].items():assert sha(ROOT/rel)==digest,(group,rel)
        save(directory/'completed.json',dict(case=case,frames=n,elapsed_s=time.monotonic()-started,
            total_force_fits=int(arrays['force_fit_count'].sum()),failed_force_fits=int(arrays['failed_force_fit_count'].sum()),
            truth_read=False,physics_run=False,source_input_seal_unchanged=True))
    except BaseException:
        save(directory/'failure.json',dict(frames=len(rows),traceback=traceback.format_exc()))
        if rows:np.savez_compressed(directory/'partial_predictions.npz',**{key:np.asarray([row[key] for row in rows]) for key in rows[0]})
        raise
    save(directory/'artifact_manifest.json',dict(files=[dict(path=p.name,bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(directory.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(case=case,success=True,frames=n,seconds=time.monotonic()-started)),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument('--case',choices=['m100_B','p030_B','p090_B'],required=True)
    for key in ('input-root','output-root','source-input-seal'):ap.add_argument('--'+key,type=Path,required=True)
    args=ap.parse_args();run(args.case,args.input_root.resolve(),args.output_root.resolve(),args.source_input_seal.resolve())
