"""Unit-aware I-64 portability verification with explicit limits.

Preserves strict v1. Fresh arrays are never replaced. Exact per-frame decisions
and summary reproducibility can pass while raw float byte parity remains false;
pointwise float closeness is NOT proved by this compact archive.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse,json,shutil,sys
import numpy as np

import replay_history_analysis as V1
import verification_bindings_v2 as B


def array_check(values,expected):
    rows={}
    for key in sorted(set(values)|set(expected)):
        actual=B.describe(values[key]) if key in values else None;old=expected.get(key)
        structure=bool(actual is not None and old is not None and actual['dtype']==old['dtype'] and actual['shape']==old['shape'])
        exact=structure and actual['raw_sha256']==old['raw_sha256']
        pattern=structure and actual.get('nonfinite_pattern_sha256')==old.get('nonfinite_pattern_sha256')
        floating=structure and np.issubdtype(values[key].dtype,np.floating)
        allowed_float=floating and key.startswith(('M2_','M3_')) and (key.endswith('_Nm') or key.endswith('_score') or key.endswith('_norm_share'))
        passed=structure and pattern and (exact or allowed_float)
        rows[key]=dict(passed=bool(passed),byte_exact=bool(exact),structure_exact=structure,nonfinite_pattern_exact=bool(pattern),
            byte_inexact_float_permitted_without_pointwise_claim=bool(not exact and allowed_float),expected=old,actual=actual)
    return dict(passed=all(row['passed'] for row in rows.values()),all_byte_exact=all(row['byte_exact'] for row in rows.values()),fields=rows)


def json_compare(expected,actual,protocol,path='',rows=None):
    if rows is None:rows=[]
    if isinstance(expected,dict) and isinstance(actual,dict):
        if set(expected)!=set(actual):rows.append(dict(path=path,passed=False,reason='keys differ'))
        for key in sorted(set(expected)&set(actual)):
            json_compare(expected[key],actual[key],protocol,path+'.'+key,rows)
    elif isinstance(expected,list) and isinstance(actual,list):
        if len(expected)!=len(actual):rows.append(dict(path=path,passed=False,reason='length differs'))
        for i,(a,b) in enumerate(zip(expected,actual)):json_compare(a,b,protocol,path+'[%d]'%i,rows)
    elif expected!=actual or type(expected)!=type(actual):
        row=dict(path=path,expected=expected,actual=actual)
        if path.endswith('.numpy_version'):
            row.update(passed=True,metadata_only=True)
        elif isinstance(expected,float) and isinstance(actual,(int,float)) and not isinstance(actual,bool):
            difference=abs(expected-actual);row['absolute_difference']=difference
            if path.endswith('.inherited_norm_share_median'):
                row.update(passed=True,auxiliary_norm_share_only=True,not_used_as_numeric_gate=True)
            else:
                allowance=0.;unit='exact scalar required'
                if '_Nm' in path:
                    allowance=protocol['numeric_verification']['independent_joint_predictions_Nm_atol'];unit='N m'
                elif any(name in path for name in ('.filtered_recurrence_max9','.full_filtered_reconstruction_max9')):
                    allowance=protocol['numeric_verification']['observer_recurrence_moment_and_joint_Nm_atol'];unit='mixed generalized force; registered bound'
                elif any(name in path for name in ('.point_RMS_mm','.point_p95_mm','.point_maximum_mm')):
                    allowance=protocol['training_gates']['point_comparison_roundoff_allowance_mm'];unit='mm'
                elif 'score' in path:
                    allowance=protocol['numeric_verification']['score_comparison_atol'];unit='dimensionless score'
                row.update(passed=difference<=allowance,allowance=allowance,unit=unit)
        else:row.update(passed=False,reason='exact structural/count/flag scalar differs')
        rows.append(row)
    return rows


def run(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    source=args.analysis_source.resolve();package=args.feature_package.resolve();reference=args.reference.resolve()
    protocol=V1.read(args.protocol);registration=V1.read(reference/'registered_before_evaluation.json');binding=V1.read(args.bindings)
    if V1.sha(source/'history_diagnostic.py')!=registration['source_sha256']:raise ValueError('Changed scientific source')
    if V1.sha(source/'i62_metrics.py')!=registration['metric_dependency_sha256']:raise ValueError('Changed metric dependency')
    if V1.sha(args.protocol)!=registration['protocol_sha256']:raise ValueError('Changed protocol')
    V1.write(output/'registered_before_replay.json',dict(utc=datetime.now(timezone.utc).isoformat(),numpy_version=np.__version__,
        source_sha256=V1.sha(Path(__file__)),binding_source_sha256=V1.sha(Path(B.__file__)),bindings_sha256=V1.sha(args.bindings),
        allowance_policy='Exact structures/bool/int/nonfinitepatterns/per-frame decisions and summarycounts/flags. Fresh Nm stats1e-9, scores1e-8 per protocol; auxiliary normshares explicitly nongating. No per-element float bound from hash-only archive.',
        local_pointwise_feature_audit_sha256=V1.sha(args.local_feature_audit),feature_packet_manifest_sha256=V1.sha(package/'packet_manifest.json')))
    packets=V1.module('_i64_v2_history_packet',package/'history_packets.py');features=output/'regenerated_features';features.mkdir();checks={};strict={}
    for case in packets.CASES:
        values,parity=packets.load_and_derive_case(package,case);strict[case]=parity
        checks[case]=array_check(values,binding['features'][case])
        directory=features/case;directory.mkdir();np.savez_compressed(directory/'history_features.npz',**values)
        shutil.copy2(package/case/'primitives.npz',directory/'primitives.npz')
    V1.write(output/'feature_verification.json',checks);V1.write(output/'strict_feature_parity.json',strict)
    if not all(row['passed'] for row in checks.values()):raise ValueError('Unexpected feature structure/mask/finite pattern or unallowed byte difference')
    sys.path.insert(0,str(source));primary=V1.module('_i64_v2_primary',source/'history_diagnostic.py')
    primary.run(args.protocol.resolve(),args.base_root.resolve(),features,args.old_witnesses.resolve(),output/'evaluation')
    array_reports={};decision_reports={}
    for name,expected in binding['analysis'].items():array_reports[name]=array_check(B.read_npz(output/'evaluation'/name),expected)
    for case,expected in binding['decisions'].items():
        decision_reports[case]=array_check(B.decisions(B.read_npz(output/'evaluation'/(case+'_frames.npz'))),expected)
    V1.write(output/'analysis_array_verification.json',array_reports);V1.write(output/'per_frame_decision_verification.json',decision_reports)
    expected=V1.read(reference/'science.json');actual=V1.read(output/'evaluation/science.json');diff=json_compare(expected,actual,protocol)
    scientific_json_ok=all(row['passed'] for row in diff)
    decisions_ok=all(row['all_byte_exact'] for row in decision_reports.values())
    array_structure_ok=all(row['passed'] for row in array_reports.values())
    result=dict(passed=scientific_json_ok and decisions_ok and array_structure_ok,
        validation_scope='Scientific summaries and exact decisions, not portable per-element float closeness',numpy_version=np.__version__,canonical_numpy_version=expected['numpy_version'],
        scientific_summaries_within_registered_unit_bounds=scientific_json_ok,per_frame_decisions_exact=decisions_ok,
        structures_integer_bool_finite_patterns_exact=array_structure_ok,
        feature_float_arrays_byte_exact=all(row['all_byte_exact'] for row in checks.values()),
        analysis_float_arrays_byte_exact=all(row['all_byte_exact'] for row in array_reports.values()),
        pointwise_float_bounds_verified_from_archive=False,
        separate_local_pointwise_feature_audit=dict(path=str(args.local_feature_audit),sha256=V1.sha(args.local_feature_audit),
            scope='Previously measured against full local canonical arrays; cited separately, not recomputed from compact hashes'),
        json_differences=diff,no_values_replaced=True,scientific_rules_changed=False,new_physics_steps=0)
    V1.write(output/'comparison.json',result);print(json.dumps({k:v for k,v in result.items() if k not in ('json_differences','separate_local_pointwise_feature_audit')}))
    return result['passed']


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('analysis-source','protocol','base-root','feature-package','bindings','local-feature-audit','old-witnesses','reference','output'):
        p.add_argument('--'+name,type=Path,required=True)
    sys.exit(0 if run(p.parse_args()) else 1)
