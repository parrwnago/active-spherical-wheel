"""Compact scientific I59 coefficient and same-state native-command audit plot."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def draw(source,output):
 output.mkdir(parents=True,exist_ok=False);report_path=source/'science.json';r=json.loads(report_path.read_text())
 save=lambda p,v:p.write_text(json.dumps(v,indent=2)+'\n')
 shutil.copyfile(Path(__file__),output/'plot_source.py')
 save(output/'registered_before_render.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),analysis_source_sha256=sha(Path(__file__)),science_sha256=sha(report_path),scope=__doc__))
 cases=list(r['cases']);anchors=list(r['anchors']);phase={'m100':'−1.0','p030':'+0.3','p090':'+0.9'}
 raw=np.array([[r['cases'][c]['alternatives'][a]['gain_row_only']['full']['raw_max_abs_by_input'][0]for a in anchors]for c in cases])
 clipped=np.array([[r['cases'][c]['alternatives'][a]['gain_row_only']['full']['clipped_max_abs_by_input'][0]for a in anchors]for c in cases])
 coefficients=np.array([[next(v['K0_max_abs_difference']for v in r['coefficient_pairs']if v['actual']==a and v['alternative']==b)for b in anchors]for a in anchors])
 fig=plt.figure(figsize=(11,9),layout='constrained');grid=fig.add_gridspec(2,2,height_ratios=[.7,1.7]);table_ax=fig.add_subplot(grid[0,:]);table_ax.axis('off')
 labels=[phase[a[:4]]+' rad'for a in anchors]
 table=table_ax.table(cellText=[['0'if v==0 else f'{v:.3e}'for v in row]for row in coefficients],rowLabels=labels,colLabels=labels,loc='center',cellLoc='center')
 table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.5)
 table_ax.set_title('Maximum absolute difference in stored shaft-feedback coefficients\nRows: recorded startup anchor; columns: substituted anchor (mixed state-coordinate gain units)',fontsize=11,pad=15)
 axes=[fig.add_subplot(grid[1,i])for i in range(2)];norm=LogNorm(vmin=1e-12,vmax=1e-3);cmap=plt.get_cmap('viridis').copy();cmap.set_bad('#dddddd')
 ylabels=[phase[c[:4]]+' '+('B native'if c.endswith('_B')else'P native shadow')for c in cases]
 for ax,values,title in zip(axes,[raw,clipped],['Maximum raw native shaft change','Maximum native amplitude-clipped shaft change']):
  im=ax.imshow(np.ma.masked_equal(values,0),norm=norm,cmap=cmap,aspect='auto')
  ax.set(title=title,xticks=np.arange(3),xticklabels=labels,yticks=np.arange(len(cases)),yticklabels=ylabels,xlabel='Substituted startup anchor')
  for i in range(values.shape[0]):
   for j in range(values.shape[1]):ax.text(j,i,'0'if values[i,j]==0 else f'{values[i,j]:.2e}',ha='center',va='center',color='black'if values[i,j]==0 else'white',fontsize=9)
 fig.colorbar(im,ax=axes,label='Absolute shaft-command difference (N·m); gray = exact zero',shrink=.8)
 trim=max(v['trim_max_abs_difference']for v in r['coefficient_pairs'])
 fig.suptitle('I-59 | Same recorded states, errors, references and contact context\nGain-row-only comparison across every saved 5 ms state; zero new physics',fontsize=13)
 fig.supxlabel(f'Frozen shaft resolution: 1e−3 N·m. Largest trim-only coefficient change: {trim:.2e} N·m.\nP rows audit its native shadow; no P reoptimization or trajectory/stability conclusion.',fontsize=10)
 fig.savefig(output/'anchor_effect.png',dpi=180);fig.savefig(output/'anchor_effect.pdf',dpi=180);plt.close(fig)
 save(output/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in output.iterdir()if p.is_file()and p.name!='artifact_manifest.json']))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--data-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();draw(a.data_source.resolve(),a.output.resolve())
