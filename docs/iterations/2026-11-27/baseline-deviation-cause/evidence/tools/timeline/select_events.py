"""I77 fixed baseline event selection; no simulation, tuning or causal claim."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
DATA=ROOT/'results/tangent_heading_routes_20261127'
OUT=HERE/'events_stage01'
DT=.005
THRESHOLDS=(('geometric_30mm_100ms','geometric_cross_track_m',.030),('timed_50mm_100ms','timed_xy_error_m',.050))

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):
    with p.open('x')as f:json.dump(v,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def context(t,index,span):
    selected=np.flatnonzero((t>=t[index]-span-1e-10)&(t<=t[index]+span+1e-10))
    return dict(half_width_s=span,first_source_row=int(selected[0]),last_source_row=int(selected[-1]),first_s=float(t[selected[0]]),last_s=float(t[selected[-1]]),samples=len(selected))
def event(t,index,kind,signal,value,confirmation=None):
    return dict(kind=kind,source_row=int(index),time_s=float(t[index]),signal=signal,value_m=float(value),confirmation_source_row=None if confirmation is None else int(confirmation),confirmation_time_s=None if confirmation is None else float(t[confirmation]),sustained_duration_s=None if confirmation is None else float(t[confirmation]-t[index]),contexts=[context(t,index,s)for s in (1.,.25)])
def first_sustained(t,y,moving,bound):
    good=moving&np.isfinite(y)&(y>=bound);start=None
    for i,okay in enumerate(good):
        if not okay:start=None;continue
        if start is None or (i and abs((t[i]-t[i-1])-DT)>1e-9):start=i
        if t[i]-t[start]>=.1-1e-9:return start,i
    return None

def main():
    protocol_path=DATA/'protocol/protocol.json';seal_path=DATA/'campaign/outputs_sealed.json';science=DATA/'evaluation/completed_stage01/science.json';manifest=DATA/'evaluation/completed_stage01/artifact_manifest.json'
    protocol=json.loads(protocol_path.read_text());jobs=[j for j in protocol['cases']if j['policy']=='N'];assert len(jobs)==9
    files=[Path(__file__),protocol_path,seal_path,science,manifest]
    for j in jobs:files += [DATA/f"evaluation/completed_stage01/{j['case_id']}_signals.npz",DATA/f"campaign/runs/{j['case_id']}/precision.npz",DATA/f"campaign/runs/{j['case_id']}/summary.json",DATA/f"campaign/runs/{j['case_id']}/provenance.json"]
    bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
    seal=json.loads(seal_path.read_text());assert seal['protocol_sha256']==sha(protocol_path)
    for j in jobs:
        rel=f"campaign/runs/{j['case_id']}/precision.npz";binding=seal['files'][rel];assert sha(DATA/rel)==binding['sha256']
    OUT.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),OUT/'select_events.py')
    write(OUT/'registered_before_array_reads.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),files=bindings,no_new_physics=True,no_control_changes=True,selection=dict(thresholds=list(THRESHOLDS),sustain_s=.1,moving_window='Registered half-open moving intervals; actual time and contiguous5ms samples.',peak='Earliest maximum moving geometric error.',contexts_s=[1.,.25],fixed_audit_time_s=20.585,matched_control='straight_p090_N evaluated at all selected straight_m100_N event times.'),time_convention='Recorded state is before current-row command; current command acts after that state. Pressure truth is only last1ms sample, not the observer previous5ms interval.'))
    rows={};times={}
    for j in jobs:
        cid=j['case_id'];folder=DATA/'evaluation/completed_stage01'
        with np.load(folder/f'{cid}_signals.npz',allow_pickle=False)as z:s={k:z[k].copy()for k in ('time_s','timed_xy_error_m','geometric_cross_track_m')}
        t=s['time_s'];times[cid]=t;clock=protocol['evaluation']['clock_by_route'][j['route_id']];low,high=clock['moving_s'];moving=(t>=low-1e-9)&(t<high-1e-9)
        with np.load(DATA/f'campaign/runs/{cid}/precision.npz',allow_pickle=False)as z:
            np.testing.assert_array_equal(t,z['time_s']);xy=z['exact_qpos'][:,:2].copy();ref=z['exact_reference_xy'].copy()
        first=int(np.flatnonzero(moving)[0]);events=[]
        for name,key,bound in THRESHOLDS:
            found=first_sustained(t,s[key],moving,bound)
            events.append(dict(kind=name,found=False,reason='No contiguous moving interval at/above threshold for100ms')if found is None else dict(found=True,**event(t,found[0],name,key,s[key][found[0]],found[1])))
        ids=np.flatnonzero(moving&np.isfinite(s['geometric_cross_track_m']));peak=int(ids[np.argmax(s['geometric_cross_track_m'][ids])]);events.append(dict(found=True,**event(t,peak,'largest_moving_geometric_peak','geometric_cross_track_m',s['geometric_cross_track_m'][peak])))
        if cid in ('straight_m100_N','straight_p090_N'):
            i=int(np.argmin(abs(t-20.585)));events.append(dict(found=True,requested_time_s=20.585,**event(t,i,'fixed_prior_audit_time','geometric_cross_track_m',s['geometric_cross_track_m'][i])))
        rows[cid]=dict(case=j,moving_s=[low,high],initial=dict(time_s=float(t[0]),timed_error_m=float(s['timed_xy_error_m'][0]),offset_xy_m=(xy[0]-ref[0]).tolist()),moving_start=dict(time_s=float(t[first]),timed_error_m=float(s['timed_xy_error_m'][first]),geometric_error_m=float(s['geometric_cross_track_m'][first]),offset_xy_m=(xy[first]-ref[first]).tolist()),events=events)
    source=rows['straight_m100_N'];control=rows['straight_p090_N'];t=times['straight_p090_N']
    with np.load(DATA/'evaluation/completed_stage01/straight_p090_N_signals.npz',allow_pickle=False)as z:y=z['geometric_cross_track_m']
    for e in source['events']:
        if not e['found']or e['kind']=='fixed_prior_audit_time':continue
        i=int(np.argmin(abs(t-e['time_s'])));control['events'].append(dict(found=True,matched_source_case='straight_m100_N',matched_source_event=e['kind'],requested_time_s=e['time_s'],**event(t,i,'matched_control_'+e['kind'],'geometric_cross_track_m',y[i])))
    if not all(sha(ROOT/p)==v['sha256']for p,v in bindings.items()):raise ValueError('Sealed source/input changed')
    write(OUT/'report.json',dict(iteration='I-77',cases=rows,source_inputs_unchanged=True,no_new_physics=True,no_control_changes=True,observations_not_causal_proof=True))
    write(OUT/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in sorted(OUT.iterdir())if p.is_file()]))
    for cid,row in rows.items():print(cid,'initial_mm',round(1000*row['initial']['timed_error_m'],3),'moving_start_mm',round(1000*row['moving_start']['timed_error_m'],3),'events',[(e['kind'],e.get('time_s'),None if not e['found']else round(e['value_m']*1000,3))for e in row['events']])
if __name__=='__main__':main()
