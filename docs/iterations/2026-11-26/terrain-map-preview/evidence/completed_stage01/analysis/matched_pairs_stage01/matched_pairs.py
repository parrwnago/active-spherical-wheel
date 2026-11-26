"""Post-registered I44 matching supplement; frozen data, no retuning/new gates."""
from pathlib import Path
import hashlib,json
import numpy as np
ROOT=Path(__file__).resolve().parents[3];FAMILY=ROOT/'results/terrain_memory_20261126'
SOURCE=FAMILY/'campaign/analyze_stage01';OUT=FAMILY/'analysis/matched_pairs_stage01';OUT.mkdir(exist_ok=False)
(OUT/'matched_pairs.py').write_bytes(Path(__file__).read_bytes())
rows=json.loads((SOURCE/'predictions.json').read_text())
def stats(values):
 v=np.asarray(values,float)
 return {'n':len(v),'rmse':None if not len(v)else float(np.sqrt(np.mean(v*v))),'p90_abs':None if not len(v)else float(np.percentile(abs(v),90)),'median_abs':None if not len(v)else float(np.median(abs(v)))}
report={'status':'Post-registered descriptive matching supplement after primary results; no new pass gate or parameter change','primary_artifact_sha256':hashlib.sha256((SOURCE/'predictions.json').read_bytes()).hexdigest(),'strata':{}}
for stratum in ['uniform','dense_bump']:
 grouped={}
 for r in rows:
  if stratum in r['strata']:grouped.setdefault((r['anchor_index'],r['horizon_s']),{})[r['arm']]=r
 pairs=[g for g in grouped.values()if g['observer_plane']['surface_score']['supported']and g['memory_four']['surface_score']['supported']]
 contact=[g for g in grouped.values()if all(g[a]['nearest_candidate_error_m']is not None for a in ['observer_plane','memory_four'])]
 surface={a:{'height_error_m':stats([g[a]['surface_score']['height_error_m']for g in pairs if g[a]['surface_score']['height_error_m']is not None]),'normal_error_deg':stats([g[a]['surface_score']['normal_error_deg']for g in pairs if g[a]['surface_score']['normal_error_deg']is not None])}for a in ['observer_plane','memory_four']}
 contactstats={a:stats([g[a]['nearest_candidate_error_m']for g in contact])for a in ['observer_plane','memory_four']}
 delta=[g['memory_four']['nearest_candidate_error_m']-g['observer_plane']['nearest_candidate_error_m']for g in contact]
 report['strata'][stratum]={'surface_pairs':len(pairs),'surface':surface,'actual_future_equivalent_point_pairs':len(contact),'nearest_candidate_error_m':contactstats,'memory_nearest_candidate_better_pairs':sum(x<0 for x in delta),'memory_nearest_candidate_worse_pairs':sum(x>0 for x in delta),'mean_memory_minus_observer_distance_m':None if not delta else float(np.mean(delta)),'scope':'Both arms use same anchor/horizon/truefuturetarget. Nearest-set distance is optimistic for ambiguous sets and includes shared CV motion error; not force/load accuracy.'}
memory=sorted([r for r in rows if r['arm']=='memory_four'],key=lambda r:(r['anchor_time_s'],r['horizon_s']))
report['first_supported_memory_query']=next(({'anchor_time_s':r['anchor_time_s'],'horizon_s':r['horizon_s'],'first_capture_s':r['surface_query']['first_capture_s']}for r in memory if r['surface_score']['supported']),None)
report['dense_memory_supported_queries']=[{'anchor_time_s':r['anchor_time_s'],'horizon_s':r['horizon_s'],'first_capture_s':r['surface_query']['first_capture_s'],'contact_candidate_available':bool(r['candidate_patches'])}for r in memory if 'dense_bump'in r['strata']and r['surface_score']['supported']]
report['all_three_common_surface_queries']=sum(g['latest_four']['surface_score']['supported']and g['memory_four']['surface_score']['supported']and g['observer_plane']['surface_score']['supported']for g in grouped.values())
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
