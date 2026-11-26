"""Portable exact1ms traces and exact-value5ms window subsets; no simulation."""
from pathlib import Path
import datetime,hashlib,json,shutil
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[1];OUT=BASE/'portable_data'
OUT.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
entries=[];sources=[]
for case in ('native_substeps','zero_free_shellA','hold_attitude'):
    folder=BASE/'campaign'/case
    src=folder/'substeps.npz';dst=OUT/(case+'_substeps.npz');shutil.copyfile(src,dst)
    entries.append({'path':dst.name,'origin':str(src.relative_to(ROOT)),'bytes':dst.stat().st_size,'sha256':sha(dst),'derivation':'Byte-exact copy of complete recorded1ms trace31.4–32.5; no resampling/casting'})
    arrays={};source_records=[]
    for kind,src in [('precision',folder/'campaign/runs/m100_R/precision.npz'),('sidecar',folder/'i55_sidecar.npz')]:
        with np.load(src,allow_pickle=False) as z:
            mask=(z['time_s']>=31.4-1e-9)&(z['time_s']<=32.5+1e-9)
            assert int(mask.sum())==221
            for k in z.files:arrays[kind+'__'+k]=z[k][mask].copy()
            source_records.append({'kind':kind,'path':str(src.relative_to(ROOT)),'sha256':sha(src),'rows':len(z['time_s']),'selected_rows':int(mask.sum())})
    dst=OUT/(case+'_control_window.npz');np.savez_compressed(dst,**arrays)
    with np.load(dst,allow_pickle=False) as saved:
        assert all(saved[k].dtype==v.dtype and np.array_equal(saved[k],v,equal_nan=True) if v.dtype.kind in 'fc' else saved[k].dtype==v.dtype and np.array_equal(saved[k],v) for k,v in arrays.items())
    entries.append({'path':dst.name,'bytes':dst.stat().st_size,'sha256':sha(dst),'derivation':'Exact field/window subset31.4–32.5,221rows; fieldnames prefixedprecision__/sidecar__; no resampling/casting','sources':source_records})
meta={'iteration':'I-55','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_sha256':sha(Path(__file__)),'cases':['native_substeps','zero_free_shellA','hold_attitude'],'window_s':[31.4,32.5],'files':entries,'bytes':sum(a['bytes'] for a in entries),'semantics':'Parentprecision exact_native_requested/applied are final returned values inI55. sidecar pre_output_* is computed before actuator override; hold_attitudealreadyusesheldquat. World bridgeω differsfrombodyframe precisionqvelangular. Fullprefixrawrollouts staylocal; exactparitygates/provenance copiedseparately.'}
(OUT/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps({'files':len(entries),'bytes':meta['bytes']},indent=2))
