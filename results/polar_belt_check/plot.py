from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent
fig,axes=plt.subplots(2,2,figsize=(10,6),constrained_layout=True)
for j,(kind,title) in enumerate([('roll','Polar orientation: +2° initial roll'),('push','Polar orientation: +0.5 N lateral push for 0.1 s')]):
    for suffix,label in [('belts','Active belt'),('no_belts','Belt commands disabled; same gains')]:
        z=np.load(root/f'{kind}_positive_{suffix}.npz'); a=z['log'];c={v:i for i,v in enumerate(z['columns'])}
        axes[0,j].plot(a[:,c['time_s']],np.rad2deg(a[:,c['relative_tilt_rad']]),label=label)
        axes[1,j].plot(a[:,c['time_s']],a[:,c['input_belt_A']],label=label)
    axes[0,j].set(title=title,ylabel='Tilt relative to upright (deg)',ylim=(0,55))
    axes[1,j].set(xlabel='Time (s)',ylabel='Belt A surface speed (m/s)')
    axes[0,j].legend(fontsize=8)
    for i in (0,1):axes[i,j].grid(alpha=.3)
fig.suptitle('Independent polar belt ablation — ideal speed servo, assumed physical parameters')
fig.savefig(root/'polar_belt_ablation.png',dpi=150)
