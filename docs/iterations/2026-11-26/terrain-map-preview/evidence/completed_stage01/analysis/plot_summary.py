"""Standalone I44 scientific figure; counts include unknowns, errors conditional."""
from pathlib import Path
import hashlib,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
FAMILY=ROOT/'results/terrain_memory_20261126'
SOURCE=FAMILY/'campaign/analyze_stage01'
report=json.loads((SOURCE/'report.json').read_text())
rows=json.loads((SOURCE/'predictions.json').read_text())
matched=json.loads((FAMILY/'analysis/matched_pairs_stage01/report.json').read_text())
out=FAMILY/'analysis/figures_stage01';out.mkdir(exist_ok=False)
(out/'plot_summary.py').write_bytes(Path(__file__).read_bytes())
colors=['#d07835','#4386c6','#17866e'];arms=['observer_plane','latest_four','memory_four']
labels=['Observer plane','Latest four views','Four-view memory']
fig=plt.figure(figsize=(12,7.8),layout='constrained')
grid=fig.add_gridspec(2,3,height_ratios=[1.,1.25])
metrics=[('coverage',None,100.,'Terrain available (%)'),('height_error_m','rmse',1000.,'Matched covered height RMSE (mm)'),('normal_error_deg','p90_abs',1.,'Matched covered normal p90 (deg)')]
for column,(key,subkey,scale,title) in enumerate(metrics):
 ax=fig.add_subplot(grid[0,column]);values=[];counts=[]
 for arm in arms:
  stats=report['summary']['uniform'][arm]['all_horizons']
  if subkey is not None and arm in matched['strata']['uniform']['surface']:stats=matched['strata']['uniform']['surface'][arm]
  value=stats[key] if subkey is None else stats[key][subkey]
  values.append(np.nan if value is None else value*scale)
  counts.append(stats['supported'] if subkey is None else stats[key]['n'])
 ax.bar(range(3),values,color=colors,width=.65)
 ax.set_xticks(range(3),['Observer\nplane','Latest\nfour','Four-view\nmemory']);ax.set_title(title,fontsize=11)
 ax.spines[['top','right']].set_visible(False)
 ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
 ax.set_ylim(0, max(1.,max(v for v in values if np.isfinite(v)))*1.3)
 if key=='coverage':ax.set_ylim(0,110)
 for i,(value,count) in enumerate(zip(values,counts)):
  if np.isfinite(value):ax.text(i,value+ax.get_ylim()[1]*.025,f'{value:.1f}\nn={count}',ha='center',fontsize=9)
  else:ax.text(i,ax.get_ylim()[1]*.08,'unavailable\nn=0',ha='center',fontsize=9)
ax=fig.add_subplot(grid[1,:])
for arm,label,color in zip(arms,labels,colors):
 sub=sorted([r for r in rows if r['arm']==arm and abs(r['horizon_s']-.2)<1e-8],key=lambda r:r['anchor_time_s'])
 t=np.array([r['anchor_time_s']for r in sub]);err=np.array([np.nan if r['surface_score']['height_error_m']is None else abs(r['surface_score']['height_error_m'])*1000 for r in sub])
 ax.plot(t,err,'.-',label=label,color=color,markersize=3,linewidth=1)
 missing=~np.isfinite(err)
 if missing.any():ax.scatter(t[missing],np.full(missing.sum(),-.6-.25*arms.index(arm)),marker='|',s=12,color=color)
ax.axvspan(6.7,6.9,color='#aa4444',alpha=.17,label='Dense bump window')
ax.set_xlabel('Forecast anchor time (s)');ax.set_ylabel('Absolute surface-height error (mm)')
ax.set_title('200 ms preview at the same CV-predicted XY; gaps/bottom ticks mark unavailable terrain',fontsize=11)
ax.grid(alpha=.18);ax.spines[['top','right']].set_visible(False);ax.legend(ncol=4,fontsize=9,loc='upper right')
fig.suptitle('I-44: four-camera terrain memory\nIdeal visible depth + exact simulated pose; native LQR motion unchanged',fontsize=14)
fig.supxlabel('Uniform metrics: 109 anchors × 3 horizons. Errors are conditional on coverage; normals are collision-face geometric proxies.\nPaired future-contact point RMS: observer 7.90 mm; memory 11.30 mm (271 pairs). No tracking-improvement claim.',fontsize=9)
for suffix in ['png','pdf','svg']:fig.savefig(out/f'terrain_memory_summary.{suffix}',dpi=180)
plt.close(fig)
manifest={'inputs':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in [SOURCE/'report.json',SOURCE/'predictions.json',FAMILY/'analysis/matched_pairs_stage01/report.json']},'outputs':{p.name:{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}for p in sorted(out.iterdir())if p.is_file()}}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'figure_dir':str(out),'files':len(manifest['outputs'])}))
