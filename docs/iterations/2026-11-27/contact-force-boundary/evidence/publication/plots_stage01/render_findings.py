"""I69 presentation figure from sealed controlled/native reports."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

BASE=Path(__file__).resolve().parents[1]
OUT=BASE/'publication/plots_stage01'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    OUT.mkdir(exist_ok=False)
    controlled=BASE/'bench/stage01/probe_output.json';native=BASE/'bench/native_capture_stage01/report.json'
    seal=dict(registered_utc=datetime.now(timezone.utc).isoformat(),script_sha256=sha(__file__),
        inputs={str(p):sha(p) for p in (controlled,native)},scope='Rendering only; no new solves or fits.')
    (OUT/'registered_before_render.json').write_text(json.dumps(seal,indent=2)+'\n')
    a=json.loads(controlled.read_text());r=json.loads(native.read_text())
    matches={p['contact_id']:p for p in r['new_trace_matches']['matches']}
    zero=[p for p in r['applied_contacts'] if p['dim']==3 and p['impulse'][0]==0. and p['force_tangent_norm_N']>1e-8]
    assert len(zero)==5
    labels=[]
    for p in zero:
        m=matches[p['id']];tag=m['candidate_tags'][0]
        name={0:'shell A',1:'shell B',2:'belt A',3:'belt B',4:'rim A',5:'rim B',6:'fork'}[tag]
        labels.append(f"{m['control_end_time_s']:.3f} s\nstep {p['substep']}, {name}")
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,
        'axes.spines.right':False,'axes.grid':True,'grid.alpha':.15,'pdf.fonttype':42})
    fig=plt.figure(figsize=(11.7,8.3))
    fig.suptitle('I-69: the unusual forces originate in the contact solve',x=.06,ha='left',y=.96,fontsize=18,weight='bold')
    fig.text(.06,.895,'Native replay unchanged: all 59 precision fields and all 36 substep arrays reproduce byte-for-byte.',fontsize=11)
    left=fig.add_axes([.075,.48,.32,.30]);right=fig.add_axes([.49,.48,.46,.30])
    x=a['runs'][0]['x'];values=[x[0],x[1],.7*abs(x[0])]
    bars=left.bar(np.arange(3),values,color=['#277da8','#df873c','#6d9977'],width=.6)
    left.set_xticks(np.arange(3),['Final normal','Applied tangent','Limit from\nfinal normal'],fontsize=9)
    left.set_ylabel('Impulse (normalized test units)');left.set_ylim(0,.88)
    left.set_title('Installed-library controlled example',loc='left',fontsize=12,weight='bold',pad=14)
    for bar,val in zip(bars,values):left.text(bar.get_x()+bar.get_width()/2,val+.025,f'{val:.2g}',ha='center')
    right.bar(np.arange(len(zero)),[p['force_tangent_norm_N'] for p in zero],color='#df873c',width=.65)
    right.set_xticks(np.arange(len(zero)),labels,fontsize=8)
    right.set_ylabel('Reported tangential force (N)');right.set_ylim(0,44)
    right.set_title('Native applied contacts with zero normal impulse',loc='left',fontsize=12,weight='bold',pad=14)
    for i,p in enumerate(zero):right.text(i,p['force_tangent_norm_N']+1,f"{p['force_tangent_norm_N']:.2f}",ha='center',fontsize=9)
    fig.text(.075,.345,'What the solver does',fontsize=13,weight='bold')
    fig.text(.075,.225,'1. Solve the normal constraints first.\n2. Set tangential bounds from those intermediate normal impulses.\n3. Continue solving; normals can change while the bounds stay fixed.',fontsize=11,linespacing=1.7)
    fig.text(.56,.345,'What the native capture establishes',fontsize=13,weight='bold')
    fig.text(.56,.225,'20 Dantzig calls report success; no PGS fallback.\n12 / 140 applied contacts exceed final-normal component bounds.\nAll applied impulses and exported forces match the captured solve.',fontsize=10,linespacing=1.8)
    fig.text(.075,.135,'Default PGS did not converge on the controlled example. A solver switch is not yet a validated fix.',fontsize=10)
    fig.text(.075,.075,'Decision: validate a friction-consistent contact solution before fitting new estimator weights.\nThis identifies a simulation limitation; it does not demonstrate improved tracking or explain every trajectory error.',fontsize=9,color='#444')
    fig.savefig(OUT/'contact_force_cause.pdf');fig.savefig(OUT/'overview.png',dpi=150);plt.close(fig)
    (OUT/'manifest.json').write_text(json.dumps(dict(files={p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in OUT.iterdir() if p.is_file()},pages=1),indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT),zero_normal_contacts=len(zero))))


if __name__=='__main__':main()
