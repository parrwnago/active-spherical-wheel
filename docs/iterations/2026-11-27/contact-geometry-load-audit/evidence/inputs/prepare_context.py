"""Project existing endpoint context only; no new model arithmetic or simulation."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,numpy as np


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def run(original_root,output):
    output.mkdir(parents=True,exist_ok=False)
    sources=[]
    for case in ('m100_B','p030_B','p090_B'):
        root=original_root/case;precision=root/'precision.npz';rollout=root/'rollout.npz'
        with np.load(precision,allow_pickle=False) as p:time=p['time_s'].copy()
        with np.load(rollout,allow_pickle=False) as p:
            names=('contact_force_N','contact_moment_Nm','fork_load_N','push_N','normal_force_N','contact_point_m')
            values={k:p[k].copy() for k in names};coarse=p['time_s'].copy()
        if len(time)!=8801 or len(coarse)!=8801 or np.max(abs(time-coarse))>4e-6:raise ValueError('Endpoint row/time alignment failed')
        folder=output/case;folder.mkdir()
        np.savez_compressed(folder/'endpoint_context.npz',time_s=time,rollout_time_s_float32=coarse,**values)
        sources.append(dict(case=case,precision_sha256=sha(precision),rollout_sha256=sha(rollout),
                            output_sha256=sha(folder/'endpoint_context.npz'),maximum_time_roundoff_s=float(np.max(abs(time-coarse))),
                            convention='Original FLOAT32 endpoint signals, aligned by verified sample row; exact precision clock retained. No temporal averaging/interpolation.'))
    (output/'manifest.json').write_text(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(Path(__file__)),cases=sources),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--original-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.original_root,a.output)
