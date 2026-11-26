"""Plot already reduced saved-state momentum evidence; no model evaluation."""
from pathlib import Path
import hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;STAGE=HERE/'stage02_corrected'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
inputs=[STAGE/'context_rows.json',STAGE/'report.json']
(STAGE/'plot_declaration.json').write_text(json.dumps(dict(source_sha256=sha(Path(__file__)),inputs={p.name:sha(p)for p in inputs},scope=__doc__),indent=2)+'\n')
rows=json.loads(inputs[0].read_text());t=np.array([r['time_s']for r in rows]);start=int(np.argmin(abs(t-32.245)))
base=np.array([r['H_about_systemCOM_world_kg_m2_s']['fork'][2]+r['H_about_systemCOM_world_kg_m2_s']['carrier'][2]for r in rows]);shells=np.array([r['H_about_systemCOM_world_kg_m2_s']['shell_A'][2]+r['H_about_systemCOM_world_kg_m2_s']['shell_B'][2]for r in rows]);total=base+shells
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
axes[0].plot(t,[r['omega_world_rad_s']['fork'][2]for r in rows],label='Platform + fork: world vertical angular velocity');axes[0].set_ylabel('rad/s');axes[0].legend(loc='upper left')
for values,label in [(base,'Platform/fork + carrier'),(shells,'Two shells'),(total,'Whole robot')]:axes[1].plot(t,values-values[start],label=label)
axes[1].axhline(0,color='0.7',lw=.8);axes[1].set_ylabel('ΔH_z from32.245s\nkg m²/s');axes[1].legend(loc='upper left')
axes[2].plot(t,[r['applied_command'][1]for r in rows],label='Shell A motor torque');axes[2].plot(t,[r['applied_command'][2]for r in rows],label='Shell B motor torque');axes[2].plot(t,[r['actuator_shell_pair_reaction_on_carrier_z_Nm']for r in rows],label='Projected opposite reaction on carrier',lw=1.2);axes[2].set_ylabel('N m');axes[2].legend(loc='upper left');axes[2].set_xlabel('Recorded simulation time (s)')
for ax in axes:
 ax.axvspan(32.245,32.305,color='gold',alpha=.15);ax.axvline(32.250,color='0.5',lw=.8,ls='--');ax.grid(alpha=.2);ax.set_xlim(31.65,32.4)
fig.suptitle('m100_R: platform spin rises while shells gain opposite angular momentum\nOffline saved-state calculation; internal reaction is not external total yaw torque')
fig.savefig(STAGE/'momentum_exchange.png',dpi=160);fig.savefig(STAGE/'momentum_exchange.pdf');plt.close(fig)
