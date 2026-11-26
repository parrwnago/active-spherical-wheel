"""Secondary saved-data audit of equal states under unequal .60/.90 inputs.

No simulation, model, observer or controller imports/calls. Current truth is the
latest completed 1ms solve; next-row truth is the last1ms solve after the held
5ms interval. Neither proves contact history throughout that interval.
"""
from datetime import datetime, timezone
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

FAMILY = Path(__file__).resolve().parents[1]
ROOT = FAMILY.parents[1]
OUTPUT = FAMILY/'review/cap060_cap090_equivalence'
DT = .005


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, dict):return {str(k):safe(v)for k,v in value.items()}
    if isinstance(value, (list,tuple)):return [safe(v)for v in value]
    if isinstance(value, np.ndarray):return safe(value.tolist())
    if isinstance(value, np.generic):return safe(value.item())
    if isinstance(value, float)and not np.isfinite(value):return None
    return value


def save(path,value):
    path.write_text(json.dumps(safe(value),indent=2,allow_nan=False)+'\n')


def load(path):
    with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}


def same(a,b):
    return a.shape==b.shape and a.dtype==b.dtype and a.tobytes()==b.tobytes()


def stats(values):
    values=np.asarray(values,float)
    return dict(count=values.size,min=float(values.min())if values.size else None,
        max=float(values.max())if values.size else None,mean=float(values.mean())if values.size else None)


def runs(times,mask):
    indices=np.flatnonzero(mask)
    if not len(indices):return []
    splits=np.flatnonzero(np.diff(indices)>1)+1
    return [dict(start_index=int(group[0]),last_held_index=int(group[-1]),
        start_s=float(times[group[0]]),last_command_s=float(times[group[-1]]),
        hold_end_s=float(times[group[-1]+1]),held_samples=len(group),held_duration_s=len(group)*DT)
        for group in np.split(indices,splits)]


def main():
    OUTPUT.mkdir(exist_ok=False)
    directories={name:FAMILY/'campaign/runs'/name for name in ('mars_cap060','mars_cap090')}
    sources=[Path(__file__).resolve(),FAMILY/'protocol.json',FAMILY/'run_capacity_case.py',
        ROOT/'src/spherical_wheel/contact_sensing.py',ROOT/'src/spherical_wheel/gazebo/plant.py',
        ROOT/'src/spherical_wheel/gazebo/bridge.py',ROOT/'gazebo/plugin/SphericalWheelBridge.cc']
    source_records=[]
    for path in sources:
        target=OUTPUT/'source_snapshot'/path.relative_to(ROOT)
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(path.read_bytes())
        source_records.append(dict(origin=str(path.relative_to(ROOT)),snapshot=str(target.relative_to(OUTPUT)),sha256=sha(path)))
    inputs={str((folder/name).relative_to(ROOT)):sha(folder/name)
        for folder in directories.values()for name in ('precision.npz','rollout.npz','provenance.json','artifact_manifest.json')}
    save(OUTPUT/'before_arithmetic.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),
        sources=source_records,input_sha256=inputs,
        question='When .60/.90 applied belts differ, are equal saved states explained by no loaded endpoint contact, or compatible with a loaded friction-limited response?',
        scope='Read saved arrays only. Current and next endpoints distinguished; no within5ms contact history or physics-response causality inferred.'))
    for folder in directories.values():
        for entry in json.loads((folder/'artifact_manifest.json').read_text())['files']:
            assert sha(folder/entry['path'])==entry['sha256']
    data={name:load(folder/'precision.npz')for name,folder in directories.items()}
    logs={name:load(folder/'rollout.npz')for name,folder in directories.items()}
    a,b=data.values();la,lb=logs.values();times=a['time_s']
    assert same(times,b['time_s'])
    assert np.all(np.abs(np.diff(times)-DT)<1e-9)
    weight=float(json.loads((directories['mars_cap060']/'provenance.json').read_text())['weight_N'])
    threshold=max(.05,.05*weight)
    equal_precision={key:same(a[key],b[key])for key in a if key in b}
    equal_rollout={key:same(la[key],lb[key])for key in la if key in lb}
    differences=a['exact_applied'][:-1]!=b['exact_applied'][:-1]
    combined=np.any(differences,axis=1);indices=np.flatnonzero(combined)
    csv_rows=[];components={}
    for col,label,patch in ((3,'belt_A',2),(4,'belt_B',3)):
        mask=differences[:,col];ii=np.flatnonzero(mask);following=ii+1
        cur=a['truth_patch_loads_N'][ii,patch];nxt=a['truth_patch_loads_N'][following,patch]
        cur_loaded=cur>=threshold;next_loaded=nxt>=threshold
        components[label]=dict(held_different_samples=len(ii),held_different_duration_s=len(ii)*DT,
            max_absolute_applied_difference_m_s=float(np.max(abs(a['exact_applied'][ii,col]-b['exact_applied'][ii,col])))if len(ii)else 0.,
            max_applied060_m_s=float(np.max(abs(a['exact_applied'][:-1,col]))),
            max_applied090_m_s=float(np.max(abs(b['exact_applied'][:-1,col]))),
            intervals=runs(times,mask),current_patch_load_N=stats(cur),next_endpoint_patch_load_N=stats(nxt),
            current_positive_contact_samples=int(np.sum(cur>1e-12)),next_positive_contact_samples=int(np.sum(nxt>1e-12)),
            current_loaded_ge5pct_weight_samples=int(cur_loaded.sum()),next_loaded_ge5pct_weight_samples=int(next_loaded.sum()),
            both_endpoints_loaded_samples=int(np.sum(cur_loaded&next_loaded)),
            neither_endpoint_loaded_samples=int(np.sum(~cur_loaded&~next_loaded)),
            no_positive_contact_at_either_endpoint_samples=int(np.sum((cur<=1e-12)&(nxt<=1e-12))),
            current_utilization=stats(la['utilization'][ii]),next_endpoint_utilization=stats(la['utilization'][following]),
            next_endpoint_loaded_and_utilization_ge099_samples=int(np.sum(next_loaded&(la['utilization'][following]>=.99))),
            next_endpoint_slip060_m_s=stats(la['slip_speed_m_s'][following]),next_endpoint_slip090_m_s=stats(lb['slip_speed_m_s'][following]))
        for i in ii:
            row=dict(index=int(i),time_s=float(times[i]),hold_end_s=float(times[i+1]),belt=label,
                requested_m_s=float(a['exact_native_requested'][i,col]),applied060_m_s=float(a['exact_applied'][i,col]),
                applied090_m_s=float(b['exact_applied'][i,col]),current_patch_load_N=float(a['truth_patch_loads_N'][i,patch]),
                next_patch_load_N=float(a['truth_patch_loads_N'][i+1,patch]),
                current_loaded_ge5pct_weight=bool(a['truth_patch_loads_N'][i,patch]>=threshold),
                next_loaded_ge5pct_weight=bool(a['truth_patch_loads_N'][i+1,patch]>=threshold),
                current_contact_count_gt005N=int(a['truth_patch_contact_counts'][i,patch]),
                next_contact_count_gt005N=int(a['truth_patch_contact_counts'][i+1,patch]))
            for prefix,log in (('060',la),('090',lb)):
                for suffix,j in (('current',i),('next',i+1)):
                    for key in ('slip_speed_m_s','utilization','normal_force_N','tangential_force_N','ground_friction'):
                        row[prefix+'_'+suffix+'_'+key]=float(log[key][j])
                    for axis,force in zip('xyz',log['contact_force_N'][j]):
                        row[prefix+'_'+suffix+'_force_'+axis+'_N']=float(force)
            csv_rows.append(row)
    if csv_rows:
        with(OUTPUT/'different_held_samples.csv').open('w',newline='')as stream:
            writer=csv.DictWriter(stream,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
    evaluator_gaps={key:dict(equal_bytes=equal_rollout[key],max_abs_gap=float(np.nanmax(abs(la[key].astype(float)-lb[key].astype(float)))))
        for key in ('contact_force_N','contact_moment_Nm','normal_force_N','support_loads_N','utilization','slip_speed_m_s','tangential_force_N','ground_friction')}
    report=dict(iteration='I-49',scope='Secondary saved-data equivalence audit, no new dynamics or controller/observer calls.',
        samples=len(times),held_samples=len(times)-1,dt_s=DT,weight_N=weight,loaded_endpoint_threshold_N=threshold,
        time_alignment='Command at rowi is held for[i,i+1]. Rowi truth is the latest completed1ms solve before that hold; rowi+1 is the latest1ms solve after it, not a5ms mean or full substep record.',
        equality=dict(precision=equal_precision,rollout=equal_rollout),
        differing_applied_components=[int(i)for i in np.flatnonzero(np.any(differences,axis=0))],
        union_different_held_samples=len(indices),union_different_held_duration_s=len(indices)*DT,
        components=components,evaluator_field_comparison=evaluator_gaps,
        physics_source=dict(plugin_belt_target='SphericalWheelBridge.cc449–456 passes commanded speed times tangential-axis norm as contactSurfaceMotionVelocity; frictionCoeff and secondaryFrictionCoeff set at439–440. No .30 cap in callback.',
            held_command='GazeboPlant.step173–183 applies operating cap, sends last_input belt values, runs5physicssteps, then pulls state/contact snapshot.',
            slip_diagnostic='contact_sensing.measure_records computes load-weighted slip including each belts commanded surface speed; differing slip with identical state can therefore be a kinematic consequence of different prior commands, not an independent sensor or motor response.'),
        interpretation_limits=['Equal exact saved states/wrenches are an observed equality, not a claim that every simulator-internal field is identical.',
            'Endpoint unloaded samples cannot prove the belt had no contact during any of the five intervening1ms solves.',
            'Loaded endpoints with near-unit utilization and unequal commanded slip are compatible with Coulomb friction saturation; saved endpoint data do not establish that mechanism uniquely.',
            'No force/current/surface target tracking telemetry or motor load dynamics were collected; ideal belt speed capacity is not hardware capability.'])
    save(OUTPUT/'report.json',report)
    for path,expected in inputs.items():assert sha(ROOT/path)==expected
    for entry in source_records:assert sha(ROOT/entry['origin'])==entry['sha256']
    save(OUTPUT/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(OUTPUT)),bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(OUTPUT.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
    print(json.dumps(safe(dict(components=components,evaluator_field_comparison=evaluator_gaps,
        equal_states=equal_precision['exact_qpos']and equal_precision['exact_qvel'])),indent=2))


if __name__=='__main__':main()
