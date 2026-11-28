"""Plot closed I76 records; no dynamics, observer or controller replay."""
from pathlib import Path
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
BASE=ROOT/'results/tangent_heading_routes_20261127'
RUN=BASE/'campaign/runs/eight_m100_N'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    receipt=OUT/'registered_before_arrays.json'
    if receipt.exists():raise RuntimeError('Refuse to overwrite closed plot stage')
    inputs=[RUN/'precision.npz',RUN/'rollout.npz',BASE/'evaluation/completed_stage01/eight_m100_N_signals.npz',
            ROOT/'results/slope_edge_review_20261128/support_stage01/report.json']
    bindings=[dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=sha(p)) for p in inputs]
    receipt.write_text(json.dumps(dict(iteration='I-80',source_sha256=sha(Path(__file__)),inputs=bindings,
        scope='Existing native N99–103s; no physics/observer/controller execution. Geometric error copied from canonical I76 fixed4097segment evaluation.',
        selected_times_s=[101.5,101.505],no_new_physics=True),indent=2)+'\n')
    with np.load(inputs[0],allow_pickle=False) as z:d={k:z[k].copy() for k in z.files}
    with np.load(inputs[1],allow_pickle=False) as z:f={k:z[k].copy() for k in z.files}
    with np.load(inputs[2],allow_pickle=False) as z:s={k:z[k].copy() for k in z.files}
    t=d['time_s'];mask=(t>=99.-1e-8)&(t<=103.+1e-8);i=np.flatnonzero(mask);tt=t[mask]
    assert np.array_equal(t,s['time_s'])
    pressure=d['truth_patch_loads_N'][mask].sum(axis=1)
    centre=d['exact_qpos'][mask,2];point=d['truth_equivalent_point_m'][mask,2]
    vz=d['exact_qvel'][mask,2];speed=np.linalg.norm(d['exact_qvel'][mask,:2],axis=1)
    vref=np.linalg.norm(d['exact_reference_velocity_m_s'][mask],axis=1)
    clip=np.any(abs(d['exact_requested'][mask])>d['exact_input_bounds'][mask]+1e-12,axis=1)
    plt.rcParams.update({'font.size':10})
    fig,axes=plt.subplots(4,1,figsize=(12,12),sharex=True)
    fig.suptitle('I-80 | Original native N, figure-eight −1 rad | recorded motion near the slope edge\n'
                 'Positive pressure at every saved row in99–103s; this is5ms sampled evidence, not a continuous contact trace',fontsize=13)
    ax=axes[0];ax.plot(tt,1000*(centre-centre[0]),label='Wheel-centre Δz',color='#2764a4')
    ax.plot(tt,1000*(point-point[0]),label='Pressure-mean contact Δz',color='#14856f')
    ax.set_ylabel('Height change(mm)\nEach zeroed at99s');ax.axhline(0,color='.6',lw=.7)
    twin=ax.twinx();twin.plot(tt,1000*vz,label='Centre vertical speed',color='#b26f13',alpha=.85);twin.set_ylabel('Vertical speed(mm/s)',color='#b26f13')
    lines=ax.lines[:2]+twin.lines;ax.legend(lines,[x.get_label() for x in lines],loc='lower left',fontsize=9)
    ax=axes[1];ax.plot(tt,pressure,label='Sum of positive wheel pressure',color='#14856f');ax.axhline(0,color='.5',lw=.8)
    ax.set_ylabel('Pressure load(N)');ax.set_ylim(-1,46);ax.legend(loc='upper left',fontsize=9)
    ax.text(.99,.90,f'Sampled minimum={pressure.min():.2f}N; zero-pressure rows=0/{len(tt)}',transform=ax.transAxes,ha='right',va='top')
    ax=axes[2];ax.plot(tt,speed,label='Actual XY speed',color='#2764a4');ax.plot(tt,vref,label='Desired XY speed',color='#777777',ls='--')
    ax.set_ylabel('Speed(m/s)');twin=ax.twinx();twin.plot(tt,1000*s['geometric_cross_track_m'][mask],label='Distance from finite path',color='#b26f13');twin.set_ylabel('Path error(mm)',color='#b26f13')
    lines=ax.lines+twin.lines;ax.legend(lines,[x.get_label() for x in lines],loc='upper left',fontsize=9)
    ax=axes[3];ax.plot(tt,np.degrees(d['heading__world_z_rate_rad_s'][mask]),label='Physical world-z rotation rate',color='#5a4387')
    ax.fill_between(tt,0,1,where=clip,transform=ax.get_xaxis_transform(),color='#cf7d64',alpha=.22,label='Newly emitted request clips')
    ax.set_ylabel('Rotation rate(deg/s)');ax.legend(loc='lower left',fontsize=9)
    for ax in axes:
        ax.grid(alpha=.23);ax.set_xlim(99,103);ax.axvline(101.5,color='black',lw=1,alpha=.75)
        ax.axvline(101.505,color='#b9453c',lw=.8,ls=':',alpha=.8)
    axes[0].text(101.52,axes[0].get_ylim()[1]*.8,'User frame101.500s\nMaximum request101.505s',fontsize=9,va='top')
    axes[-1].set_xlabel('Recorded physical time(s)')
    fig.text(.5,.015,'Current commands are calculated after each recorded state and act next. Pressure is last completed1ms solver data saved every5ms.\n'
             'Centre height/acceleration are not whole-system COM quantities. No sub-5ms support-loss or gravity/drive work attribution is claimed.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.055,1,.94]);fig.savefig(OUT/'support_motion.png',dpi=160);fig.savefig(OUT/'support_motion.pdf');plt.close(fig)
    assert all(sha(ROOT/x['path'])==x['sha256'] for x in bindings)
    files=[dict(path=p.name,bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(OUT.iterdir()) if p.is_file()]
    (OUT/'manifest.json').write_text(json.dumps(dict(inputs_unchanged=True,no_new_physics=True,files=files),indent=2)+'\n')
    print(json.dumps(dict(rows=len(tt),no_new_physics=True,inputs_unchanged=True)))


if __name__=='__main__':main()
