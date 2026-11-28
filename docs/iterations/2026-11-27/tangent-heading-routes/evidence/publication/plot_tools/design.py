"""Frozen I76 route-design figure; no simulation outcomes or inference."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE=Path(__file__).resolve().parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();out=args.output.resolve()
    paths=[Path(__file__),BASE/'runtime/route.py',BASE/'runtime/heading_reference.py',BASE/'protocol/protocol.json',BASE/'runtime/fixture_stage01/stress_decision_stage04.json']
    bindings={str(p):dict(sha256=sha(p),bytes=p.stat().st_size)for p in paths}
    out.mkdir(parents=True,exist_ok=False);save(out/'registered_before_arithmetic.json',dict(files=bindings,utc=datetime.now(timezone.utc).isoformat(),no_outcomes_read=True,no_physics=True))
    shutil.copy2(Path(__file__),out/'design.py')
    spec=importlib.util.spec_from_file_location('_i76_design',BASE/'runtime/route.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    fig,axes=plt.subplots(1,3,figsize=(12.5,4.7),constrained_layout=True)
    for ax,rid in zip(axes,mod.ROUTE_IDS):
        route=mod.make_route(rid);u=np.linspace(0,1,4097)
        xy=np.array([route.shape_derivatives(float(v))[0]+route.planned_local_offset for v in u])
        ax.plot(xy[:,0],xy[:,1],color='#0072b2',lw=2)
        for v in np.linspace(.05,.95,9):
            c,d,_,_=route.shape_derivatives(float(v));c=c+route.planned_local_offset;d=d/np.linalg.norm(d)*.075
            ax.arrow(*c,*d,color='#b75c00',width=.003,head_width=.03,length_includes_head=True)
        ax.scatter(*xy[0],s=110,facecolors='none',edgecolors='black',label='Planned start',zorder=4)
        ax.scatter(*xy[-1],s=85,marker='*',color='black',label='Planned goal',zorder=5)
        domain='rough stress: moderate screen FAILED'if rid=='eight'else'moderate terrain screen passed'
        ax.set(title=f'{rid}: {route.duration_s:g} s motion\n{domain}',xlabel='Local forward from site anchor [m]',ylabel='Local transverse [m]')
        ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.margins(.15)
    axes[0].legend(fontsize=8,loc='best')
    fig.suptitle('I-76 planned routes and tangent directions\nOrange arrows are desired heading, not observed robot motion; all cases add 3 s initial + 10 s final hold',fontsize=13)
    fig.savefig(out/'route_design.png',dpi=160);fig.savefig(out/'route_design.pdf');plt.close(fig)
    if any(sha(Path(p))!=v['sha256']for p,v in bindings.items()):raise ValueError('Frozen design input changed')
    save(out/'artifact_manifest.json',dict(files={p.name:dict(sha256=sha(p),bytes=p.stat().st_size)for p in sorted(out.iterdir())if p.is_file()},no_outcomes_read=True))
if __name__=='__main__':main()
