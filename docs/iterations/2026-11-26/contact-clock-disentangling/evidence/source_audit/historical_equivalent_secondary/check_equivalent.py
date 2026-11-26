"""Secondary fixed41-row equivalent-mean representation check, no prepose claim."""
from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RAW = ROOT / 'results/support_routing_timing_20261126/campaign/historical_stage01/precision.npz'
META = ROOT / 'results/lqr_contact_forecast_20261125/campaign/runs/mars_figure8_p090/provenance.json'
XML = ROOT / 'results/support_routing_timing_20261126/campaign/historical_stage01/_workspace/flat/model.xml'
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path, obj):path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def main():
    (HERE / 'source').mkdir(exist_ok=False)
    shutil.copyfile(__file__, HERE / 'source/check_equivalent.py')
    before = {str(p.relative_to(ROOT)): sha(p) for p in (RAW, META, XML)}
    save(HERE / 'protocol_before_execution.json', dict(registered_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)), inputs_sha256=before,
        fixed_window='All41 nearest saved5ms samples6.700–6.900inclusive; no clock alignment/tuning.',
        eligibility='Total wheel load>=max(.05,.05*weight), dominant normalized share>=.8 and finite global equivalent mean.',
        comparison='Same CURRENT integrated pose; raw versus radial-R projection into dominant patch band. Equivalent mean is NOT an individual collision point.',
        limits='No pre-step inference from preceding5ms state; no observer/filter/controller/model/physics invocation. Primary I48 individual-row result unchanged.'))
    with np.load(RAW) as z:data={k:z[k] for k in z.files}
    weight=json.loads(META.read_text())['weight_N']
    root=ET.parse(XML).getroot(); bands={}
    for name in LABELS:
        user=list(map(float,root.find(".//geom[@name='"+name+"_smooth']").get('user').split()))
        bands[name]=user
    rows=[]
    for target in np.arange(1340,1381)*.005:
        i=int(np.argmin(abs(data['time_s']-target)));assert abs(data['time_s'][i]-target)<1e-8
        loads=data['truth_patch_loads_N'][i];total=float(loads.sum());p=data['truth_equivalent_point_m'][i]
        eligible=bool(total>=max(.05,.05*weight) and loads.max()/total>=.8 and np.isfinite(p).all())
        row=dict(time_s=float(data['time_s'][i]),eligible=eligible)
        if eligible:
            tag=int(loads.argmax());name=LABELS[tag];sign,lower,upper,_,radius=bands[name]
            q=data['exact_qpos'][i];quat=q[3:7]/np.linalg.norm(q[3:7]);v=np.array([np.cos(q[7]),0.,-np.sin(q[7])])
            cross=np.cross(quat[1:],v);axis=v+2*quat[0]*cross+2*np.cross(quat[1:],cross)
            vector=p-q[:3];length=float(np.linalg.norm(vector));raw=float(sign*(axis@vector));normalized=raw*radius/length
            row.update(dominant_patch=name,dominant_load_fraction=float(loads.max()/total),radius_m=length,
                radius_deficit_um=float((radius-length)*1e6),raw_margin_um=float(min(raw-lower,upper-raw)*1e6),
                radial_R_margin_um=float(min(normalized-lower,upper-normalized)*1e6))
        rows.append(row)
    eligible=[r for r in rows if r['eligible']]
    report=dict(iteration='I-48',scope='Secondary equivalent-mean representation check; not individual-point truth or identified time delay.',
        rows=len(rows),eligible_rows=len(eligible),
        raw_violating=sum(r['raw_margin_um']< -1e-6 for r in eligible),
        radial_R_violating=sum(r['radial_R_margin_um']< -1e-6 for r in eligible),
        minimum_raw_margin_um=min(r['raw_margin_um']for r in eligible),
        minimum_radial_R_margin_um=min(r['radial_R_margin_um']for r in eligible),
        selected=[min(rows,key=lambda r:abs(r['time_s']-t)) for t in (6.855,6.86)],
        rows_table=rows,limits='Normalizing a weighted equivalent point does not reconstruct the individual physical contacts or fix observer identity; no force timestamp is inferred.')
    save(HERE/'report.json',report)
    with(HERE/'rows.csv').open('w',newline='')as stream:
        writer=csv.DictWriter(stream,fieldnames=list(eligible[0]));writer.writeheader();writer.writerows(rows)
    for path in(RAW,META,XML):assert sha(path)==before[str(path.relative_to(ROOT))]
    files=[p for p in HERE.rglob('*')if p.is_file()and p.name!='artifact_manifest.json']
    save(HERE/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(HERE)),bytes=p.stat().st_size,sha256=sha(p))for p in sorted(files)]))


if __name__=='__main__':main()
