"""Portable NumPy-only reproduction of all I58 case metrics and gates.

Keep portable_inputs.json, portable_inputs/, source_snapshot/, report.json and
artifact_manifest.json together; --data-source can be anywhere on disk.
"""
from pathlib import Path
import argparse,gzip,hashlib,json,types
import numpy as np


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def module(path,name):
 m=types.ModuleType(name);m.__file__=str(path);exec(compile(path.read_text(),str(path),'exec'),m.__dict__);return m

def replay(stage,output):
 index=json.loads((stage/'portable_inputs.json').read_text())
 for row in json.loads((stage/'artifact_manifest.json').read_text())['files']:
  if sha(stage/row['path'])!=row['sha256']:raise ValueError('Changed portable artifact: '+row['path'])
 modules={key:module(stage/path,'_portable_i58_'+key)for key,path in index['sources'].items()}
 reducer,metrics,shared,legacy,helper=[modules[k]for k in('reducer','metrics','shared','legacy','helper')]
 program=reducer.load(stage/index['reference_program']);events=json.loads((stage/index['events']).read_text());protocol=index['protocol']
 report=dict(cases=dict(index['unavailable_cases']))
 for case,paths in index['cases'].items():
  for kind in('projection','diagnostics','metadata'):
   if sha(stage/paths[kind])!=paths[kind+'_sha256']:raise ValueError('Changed projection: '+case+'/'+kind)
  raw=reducer.load(stage/paths['projection']);meta=json.loads((stage/paths['metadata']).read_text())
  with gzip.open(stage/paths['diagnostics'],'rt',encoding='utf8')as stream:governor=json.load(stream)
  row,_,_=metrics.case_metrics(raw,meta['summary'],meta['provenance'],governor,protocol,program,events,shared,legacy,helper)
  job=meta['job'];row.update(case_id=case,phase_rad=job['phase_rad'],variant=job['variant'],historical_parity=meta['historical_parity'],
   case_source=str(Path(meta['original_precision_path']).parents[3]))
  report['cases'][case]=row
 report['comparisons']=reducer.compare_cases(report['cases'],protocol,metrics)
 original=json.loads((stage/'report.json').read_text())
 report['case_metrics_exact']={case:reducer.safe(row)==original['cases'][case]for case,row in report['cases'].items()}
 report['comparison_metrics_exact']={case:reducer.safe(row)==original['comparisons'][case]for case,row in report['comparisons'].items()}
 report['all_replayed_metrics_exact']=all(report['case_metrics_exact'].values())and all(report['comparison_metrics_exact'].values())
 report['source_stage_report_sha256']=sha(stage/'report.json')
 reducer.save(output,report)
 if not report['all_replayed_metrics_exact']:raise ValueError('Replay differs: '+json.dumps(report['case_metrics_exact'])+' '+json.dumps(report['comparison_metrics_exact']))
 print(json.dumps(dict(all_replayed_metrics_exact=True,cases=list(report['cases']))))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--data-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if a.output.exists():raise FileExistsError(a.output)
 replay(a.data_source.resolve(),a.output.resolve())
