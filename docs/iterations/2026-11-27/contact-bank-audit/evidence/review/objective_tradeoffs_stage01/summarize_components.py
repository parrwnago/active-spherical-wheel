"""Post-result descriptive I67 component tradeoffs; cached values only."""
from pathlib import Path
from datetime import datetime,timezone
from collections import Counter
import argparse,csv,hashlib,json
import numpy as np

CASES=('m100_B','p030_B','p090_B')
TERMS=('normal_prior','wrench_moment','velocity_consistency')
COMPARISONS=(('nearest_geometry','nearest_id','nearest_distance_m','nearest_geometry_load_score'),
             ('minimum_load','minscore_id','minimum_score_point_distance_m','min_load_score'))
TOL=1e-12


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False) as z:return {key:z[key].copy() for key in z.files}


def dominant_positive(delta):
    if not np.isfinite(delta).all():return 'unknown'
    maximum=float(max(delta))
    if maximum<=TOL:return 'none_above_tolerance'
    winners=[TERMS[k] for k in range(3) if maximum-float(delta[k])<=TOL]
    return winners[0] if len(winners)==1 else 'tie:'+','.join(winners)


def summarize(rows):
    known=[row for row in rows if row['largest_positive_term']!='unknown']
    def term_rows(term):return [row['component_delta'][term] for row in known]
    def sign(values):return dict(positive=sum(v>TOL for v in values),negative=sum(v < -TOL for v in values),within_tolerance=sum(abs(v)<=TOL for v in values))
    closer=[row for row in known if row['alternative_geometry_direction']=='closer']
    return dict(denominator=len(rows),known_cost_pairs=len(known),largest_positive_term_counts=dict(Counter(row['largest_positive_term'] for row in known)),
        overall_fused_tie_count=sum(row['fused_relation']=='tie' for row in known),
        selected_strictly_lower_fused_count=sum(row['fused_relation']=='selected_lower' for row in known),
        alternative_geometry_direction_counts=dict(Counter(row['alternative_geometry_direction'] for row in known)),
        geometrically_closer_alternative_largest_positive_term_counts=dict(Counter(row['largest_positive_term'] for row in closer)),
        component_sign_counts={term:sign(term_rows(term)) for term in TERMS},
        component_median_delta={term:float(np.median(term_rows(term))) if known else None for term in TERMS},
        closer_alternative_component_sign_counts={term:sign([row['component_delta'][term] for row in closer]) for term in TERMS})


def run(root,output):
    output.mkdir(parents=True,exist_ok=False)
    protocol=root/'protocol/protocol.json';science_path=root/'analysis/stage01/science.json'
    paths=[protocol,science_path]+[root/'analysis/stage01'/f'{case}_frames.npz' for case in CASES]
    binding=[dict(path=str(path),sha256=sha(path)) for path in paths]
    write(output/'registered_before_component_summary.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),inputs=binding,
        status='Append-only descriptive aggregation after the main I67 capacity result; no new bank, score, fit or decision.',
        cohorts=['fixed41focus','fixed6harmful'],comparisons=['nearest_geometry','minimum_load'],
        component_order=list(TERMS),delta='Alternative stored cost component minus selected stored cost component; positive favors selected on that term.',
        categorical_tie_absolute_tolerance=TOL,largest_positive='Largest delta strictly above tolerance; tied largest deltas reported explicitly.',
        geometry_tolerance_m=TOL,qualification='Objective arithmetic, not a physical cause, implementation bug or recommendation to change weights.'))
    science=read(science_path);events={(e['case'],int(e['index'])) for e in science['events']};records=[];max_identity=0.
    for case in CASES:
        a=load(root/'analysis/stage01'/f'{case}_frames.npz')
        selected=a['selected_cost_components'];selected_total=a['selected_fused_cost']
        for group in ['focus','harmful']:
            for i in np.flatnonzero(a['cohort_'+group]):
                for label,id_key,distance_key,load_key in COMPARISONS:
                    alternative=a[label+'_cost_components'][i];delta=alternative-selected[i]
                    category=dominant_positive(delta);total_delta=float(a[label+'_fused_cost'][i]-selected_total[i])
                    geometry_delta=float(a[distance_key][i]-a['selected_distance_m'][i])
                    if category!='unknown':
                        error=abs(float((delta[0]+delta[1])+delta[2])-total_delta);max_identity=max(max_identity,error)
                        if total_delta < -1e-10:raise ValueError('A stored alternative has a lower original fused objective than the selected minimum')
                    record=dict(case=case,cohort=group,source_index=int(a['source_index'][i]),time_s=float(a['time_s'][i]),
                        is_fixed_event=(case,int(a['source_index'][i])) in events,comparison=label,
                        selected_id=int(a['selected_id'][i]),alternative_id=int(a[id_key][i]),same_id=bool(a['selected_id'][i]==a[id_key][i]),
                        selected_distance_mm=float(a['selected_distance_m'][i]*1000.),alternative_distance_mm=float(a[distance_key][i]*1000.),
                        alternative_geometry_direction='unknown' if not np.isfinite(geometry_delta) else ('closer' if geometry_delta < -TOL else ('farther' if geometry_delta>TOL else 'numerical_tie')),
                        selected_components={term:float(selected[i,k]) for k,term in enumerate(TERMS)},
                        alternative_components={term:float(alternative[k]) for k,term in enumerate(TERMS)},
                        component_delta={term:float(delta[k]) for k,term in enumerate(TERMS)},
                        selected_fused_cost=float(selected_total[i]),alternative_fused_cost=float(a[label+'_fused_cost'][i]),fused_delta=total_delta,
                        fused_relation='unknown' if category=='unknown' else ('tie' if abs(total_delta)<=TOL else 'selected_lower'),
                        largest_positive_term=category,selected_load_score=float(a['selected_load_score'][i]),alternative_load_score=float(a[load_key][i]))
                    records.append(record)
    if len({(r['case'],r['source_index']) for r in records if r['cohort']=='focus'})!=41 or len({(r['case'],r['source_index']) for r in records if r['cohort']=='harmful'})!=6:raise ValueError('Fixed cohort denominator changed')
    if max_identity>1e-10:raise ValueError('Stored component differences do not sum to stored total difference')
    pooled={group:{label:summarize([r for r in records if r['cohort']==group and r['comparison']==label]) for label,*_ in COMPARISONS} for group in ['focus','harmful']}
    percase={case:{group:{label:summarize([r for r in records if r['case']==case and r['cohort']==group and r['comparison']==label]) for label,*_ in COMPARISONS} for group in ['focus','harmful']} for case in CASES}
    # Structured records retain every fixed frame; no favorable new event is chosen.
    write(output/'frame_tradeoffs.json',records)
    flat=[]
    for r in records:
        row={key:value for key,value in r.items() if not isinstance(value,dict)}
        for block in ['selected_components','alternative_components','component_delta']:
            row.update({block+'__'+key:value for key,value in r[block].items()})
        flat.append(row)
    with (output/'frame_tradeoffs.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(flat[0]),lineterminator='\n');writer.writeheader();writer.writerows(flat)
    report=dict(iteration='I-67',status='Post-result fixed-cohort descriptive objective attribution.',pooled=pooled,per_case=percase,
        fixed_frame_count=47,comparison_record_count=len(records),maximum_component_delta_sum_identity_error=max_identity,
        qualification='A positive term penalizes the alternative under the unchanged fused objective. This does not establish physical slip or an implementation defect; minimum load score can compensate load-model error. No weights/gates/selector changed.')
    write(output/'report.json',report)
    for item in binding:
        if sha(item['path'])!=item['sha256']:raise ValueError('Canonical summary input changed')
    print(json.dumps(pooled,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--experiment',type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument('--output',type=Path)
    a=p.parse_args();run(a.experiment.resolve(),a.output.resolve() if a.output else Path(__file__).resolve().parent/'output_stage01')
