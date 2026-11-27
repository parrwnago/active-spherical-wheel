"""Thin field-map comparison of canonical I-62 with presealed scalar audit."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import numpy as np
from independent_sweep import read_case,keep_for,select

METRICS={'errors':'error_count','harmful':'harmful_count','useful_retained':'useful_retained_count',
         'retained':'retained_proposal_count','vetoes':'veto_count','SSE_mm2':'point_SSE_mm2',
         'RMS_mm':'point_RMS_mm','p95_mm':'point_p95_mm','maximum_mm':'point_maximum_mm'}


def load(path):
    with np.load(path,allow_pickle=False)as z:return {key:z[key].copy()for key in z.files}


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def flag_name(key):
    if key=='pooled_error':return 'pooled_errors'
    if key=='retained_nonempty':return 'nonempty_retention'
    if key.endswith('_error'):return key[:-6]+'_error_rate'
    if key.endswith('_harmful'):return key[:-8]+'_harmful_nonincrease'
    return key


def compare_array(actual,expected):
    assert actual.shape==expected.shape
    if actual.dtype.kind in 'biu':
        np.testing.assert_array_equal(actual,expected);return 0.,True
    np.testing.assert_allclose(actual,expected,atol=1e-10,rtol=0.,equal_nan=True)
    good=np.isfinite(actual)&np.isfinite(expected)
    delta=float(np.max(abs(actual[good]-expected[good])))if good.any()else 0.
    return delta,actual.dtype==expected.dtype and actual.tobytes()==expected.tobytes()


def witness(metrics,mask,grid):
    chosen=select(metrics,mask,grid['a_index'],grid['b_index'])
    return chosen


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--canonical-root',type=Path,required=True);p.add_argument('--independent-root',type=Path,required=True)
    p.add_argument('--i61-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    gold=load(a.canonical_root/'all_grid_metrics.npz');own=load(a.independent_root/'grid_metrics.npz')
    science=json.loads((a.canonical_root/'science.json').read_text());reference=json.loads((a.independent_root/'report.json').read_text())
    protocol=json.loads((a.canonical_root/'protocol.json').read_text());cases=protocol['cases']
    for left,right in [('a_index','a_index'),('b_index','b_index'),('a_Nm','a_Nm'),('b','b_dimensionless')]:compare_array(own[left],gold[right])
    checked={};max_differences={};bit_exact={}
    for case in cases:
        for left,right in METRICS.items():
            key=case+'__'+right;delta,exact=compare_array(own['case__'+case+'__'+left],gold[key])
            checked[key]=True;max_differences[key]=delta;bit_exact[key]=exact
        np.testing.assert_array_equal(own['case__'+case+'__oracle_pass_count'],gold[case+'__oracle_pass_count'])
        base=science['groups'][case]['baseline']
        assert np.all(own['case__'+case+'__count']==base['eligible_count'])
        assert np.all(own['case__'+case+'__point_count']==base['point_count'])
    group_names=['train_without_'+case for case in cases]+['full']
    folds={f['heldout']:f for f in science['folds']};group_checks={};tradeoffs={}
    for group in group_names:
        prefix='all'if group=='full'else group
        for left,right in METRICS.items():
            key=prefix+'__'+right;delta,exact=compare_array(own['group__'+group+'__'+left],gold[key])
            checked[key]=True;max_differences[key]=delta;bit_exact[key]=exact
        np.testing.assert_array_equal(own['group__'+group+'__feasible'],gold[prefix+'__feasible'])
        own_flags=[key for key in own if key.startswith('group__'+group+'__gate__')]
        for key in own_flags:
            flag=key.split('__gate__',1)[1]
            np.testing.assert_array_equal(own[key],gold[prefix+'__gate_'+flag_name(flag)])
        selected=reference['groups'][group]['selected_index']
        record=science['full_data_fit']if group=='full'else folds[group.removeprefix('train_without_')]
        selection=record['selection']
        assert selection['grid_index']==selected
        assert selection['selection_failed']==(selected is None)
        count=record['feasible_count']if group=='full'else record['training_feasible_count']
        assert count==reference['groups'][group]['feasible_count']
        base=record['baseline']if group=='full'else record['training_baseline']
        metrics={key:own['group__'+group+'__'+key]for key in METRICS}
        H=base['A_harmful_count'];U=base['A_useful_count']
        harm=np.zeros(3721,bool)if not H else (H-metrics['harmful'])/H>=protocol['training_gates']['harmful_removal_fraction_min']
        useful=np.zeros(3721,bool)if not U else metrics['useful_retained']/U>=protocol['training_gates']['useful_retention_fraction_min']
        safe=metrics['maximum_mm']<=base['N']['point_maximum_mm']+protocol['training_gates']['point_comparison_roundoff_allowance_mm']
        original=record['tradeoffs']if group=='full'else record['training_tradeoffs']
        for name,mask in [('minimum_error_all_grid',np.ones(3721,bool)),('minimum_error_with_harmful_removal',harm),('minimum_error_with_point_maximum_safe',safe)]:
            assert witness(metrics,mask,own)==original[name]['selection']['grid_index'],(group,name)
        assert int(np.sum(harm&useful))==original['harmful_and_useful_jointly_qualified_count']
        minimum=lambda key,mask:None if not mask.any()else float(np.min(metrics[key][mask]))
        assert minimum('harmful',useful)==original['minimum_harmful_count_with_useful_retention']
        assert minimum('errors',harm)==original['minimum_error_count_with_harmful_removal']
        assert (None if not harm.any()else int(np.max(metrics['useful_retained'][harm])))==original['maximum_useful_retained_count_with_harmful_removal']
        assert minimum('maximum_mm',harm&useful)==original['minimum_point_maximum_with_harmful_and_useful']
        frontier=[];previous=-1
        for h in np.unique(metrics['harmful']):
            retain=int(np.max(metrics['useful_retained'][metrics['harmful']==h]))
            if retain<=previous:continue
            previous=retain;mask=(metrics['harmful']==h)&(metrics['useful_retained']==retain)
            frontier.append(dict(harmful_count=int(h),useful_retained_count=retain,useful_retention=None if not U else retain/U,
                candidate_count=int(mask.sum()),min_error_count=int(np.min(metrics['errors'][mask])),min_point_maximum_mm=minimum('maximum_mm',mask)))
        assert frontier==original['harmful_useful_frontier'],group
        ids=list(cases)if group=='full'else [case for case in cases if case!=group.removeprefix('train_without_')]
        oracle=np.sum([own['case__'+case+'__oracle_pass_count']for case in ids],axis=0)
        np.testing.assert_array_equal(oracle,gold[prefix+'__oracle_pass_count'])
        group_checks[group]=dict(feasible_count=count,selected_index=selected,all_gate_vectors_exact=len(own_flags),
            all_conditional_witnesses_and_frontiers_exact=True)
        tradeoffs[group]=dict(point_maximum_safe_count=int(safe.sum()),minimum_error_point_safe=minimum('errors',safe),
            harmful_and_useful_jointly_qualified_count=int(np.sum(harm&useful)))
    policy_checks={};own_oof=load(a.independent_root/'oof_policies.npz')
    for case in cases:
        exported=load(a.canonical_root/(case+'_selected_policies.npz'))
        np.testing.assert_array_equal(own_oof[case+'__keep'],exported['oof_keep'])
        np.testing.assert_array_equal(own_oof[case+'__family'],exported['oof_family'])
        c=read_case(a.i61_root,case);full=reference['groups']['full']['selected_index']
        expected=c['accepted']if full is None else keep_for(c,float(own['a_Nm'][full]),float(own['b'][full]))[0]
        np.testing.assert_array_equal(expected,exported['full_data_keep'])
        np.testing.assert_array_equal(np.where(expected,c['proposal'],c['native']),exported['full_data_family'])
        policy_checks[case]=dict(all_8801_OOF_and_full_data_keep_family_arrays_exact=True)
    assert reference['OOF']['success']==science['out_of_fold']['success']
    assert reference['OOF']['every_fold_qualified']==science['out_of_fold']['all_folds_have_qualified_fit']
    assert {flag_name(k):v for k,v in reference['OOF']['gates'].items()}==science['out_of_fold']['gates']
    for left,right in METRICS.items():
        np.testing.assert_allclose(reference['OOF']['metrics'][left],science['out_of_fold']['aggregate'][right],atol=1e-10,rtol=0.)
    result=dict(iteration='I-62',completed_utc=datetime.now(timezone.utc).isoformat(),new_physics_runs=0,
        groups=group_checks,selected_policy_arrays=policy_checks,conditional_tradeoffs=tradeoffs,
        numeric_metric_arrays_compared=len(checked),max_abs_floating_metric_difference=max(max_differences.values()),
        floating_arrays_byte_exact=all(bit_exact.values()),floating_max_differences=max_differences,
        all_counts_gate_flags_selections_and_policies_exact=True,out_of_fold_success=science['out_of_fold']['success'],
        canonical_source_sha256=sha(a.canonical_root/'tolerance_sweep.py'),canonical_science_sha256=sha(a.canonical_root/'science.json'),
        scope='Thin field-name mapper only. Independent scalar sweep/selection was sealed before outcomes; no canonical analyzer or selector import, no altered tie rule or numeric value substitution.')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
