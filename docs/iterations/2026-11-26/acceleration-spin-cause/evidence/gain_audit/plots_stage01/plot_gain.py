"""Plot sealed I54 gain algebra, no controller or physics execution."""
from pathlib import Path
import datetime,hashlib,json,shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
BASE=HERE.parent
out=BASE/'plots_stage01';out.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
report=BASE/'stage01/report.json'
(out/'registered_before_plotting.json').write_text(json.dumps({'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'report_sha256':sha(report),'selection':'Previously registered R32.250 snapshot, all five input constraints. Fixed-state gain blend only; not a simulated policy.'},indent=2)+'\n')
shutil.copyfile(__file__,out/'plot_gain.py')
r=json.loads(report.read_text());e=next(x for x in r['cases']['m100_R']['events'] if abs(x['time_s']-32.25)<1e-8)
u0=np.array(e['frozen_current_error_previous_gain_request']);u1=np.array(e['actual_current_request']);limits=np.array([1.5,.6,.6,.3,.3]);lam=np.linspace(0,1,1001)
u=u0[None,:]+lam[:,None]*(u1-u0)
fig,axes=plt.subplots(1,2,figsize=(12,4.4),layout='constrained')
labels=['Shaft torque','Shell A torque','Shell B torque','Belt A speed','Belt B speed']
for j in [0,1,2,4]:axes[0].plot(lam,np.abs(u[:,j])/limits[j],label=labels[j],lw=2)
axes[0].axhspan(0,1,color='#d9efdf',alpha=.75,label='Within each command cap')
axes[0].axhline(1,color='black',ls='--',lw=1)
axes[0].set(xlabel='Blend fraction λ: previous gain → current gain',ylabel='Absolute request / command cap',title='At the current state, no gain blend satisfies every cap')
axes[0].legend(fontsize=8,loc='upper center');axes[0].grid(alpha=.2)
contributions=[e['LQR_step_due_support_mix'][2],e['LQR_step_due_phase'][2],e['LQR_step_due_error'][2]]
axes[1].barh(['Support-share change','Lookup-phase change','Error change'],contributions,color=['#bc4b51','#e3a857','#4078a5'])
for j,v in enumerate(contributions):axes[1].text(v-.08,j,f'{v:.3f} N m',ha='right',va='center',fontsize=10)
axes[1].set(xlabel='Contribution to shell-B LQR request change (N m)',title='The support-share switch dominates the request jump',xlim=(-9.2,.35))
axes[1].axvline(0,color='black',lw=.7);axes[1].grid(axis='x',alpha=.2);axes[1].invert_yaxis()
fig.suptitle('I-54 · recorded 32.250 s · −1.00 rad reference-guard case',fontsize=14)
fig.text(.5,-.025,'Frozen-state algebra with current non-LQR terms; not a new rollout or proof that blending preserves balance.',ha='center',fontsize=9)
for ext in ('pdf','png'):fig.savefig(out/f'gain_handoff_feasibility.{ext}',dpi=160,bbox_inches='tight')
plt.close(fig)
(out/'manifest.json').write_text(json.dumps({'files':[{'path':p.name,'bytes':p.stat().st_size,'sha256':sha(p)} for p in sorted(out.iterdir()) if p.is_file()]},indent=2)+'\n')
print('Gain figure written.')
