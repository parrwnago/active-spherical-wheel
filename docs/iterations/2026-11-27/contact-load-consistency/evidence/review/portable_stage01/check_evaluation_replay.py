"""Audit cross-NumPy replay differences without changing any replayed values."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def differences(a,b,path=()):
    if type(a)is not type(b):return [(path,a,b)]
    if isinstance(a,dict):
        assert set(a)==set(b),path
        return [d for key in a for d in differences(a[key],b[key],path+(key,))]
    if isinstance(a,list):
        assert len(a)==len(b),path
        return [d for i,(x,y)in enumerate(zip(a,b))for d in differences(x,y,path+(i,))]
    return []if a==b else [(path,a,b)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--canonical-root',type=Path,required=True);p.add_argument('--replay-root',type=Path,required=True)
    p.add_argument('--initial-failed-replay-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    expected=json.loads((a.canonical_root/'science.json').read_text())
    replay=json.loads((a.replay_root/'science.json').read_text())
    original_failed=json.loads((a.initial_failed_replay_root/'science.json').read_text())
    assert replay==original_failed,'Validation wrapper changed the initially computed scientific values'
    delta=differences(expected,replay);listed=[]
    event_names={'earliest_native_disagreement','earliest_accepted_wrong','maximum_margin_accepted_wrong','i60_worst_selected_point'}
    for path,old,new in delta:
        assert (len(path)==6 and path[0]=='cases'and path[1]in('m100_B','p030_B','p090_B')
                and path[2]=='fixed_events'and path[3]in event_names
                and path[4]=='distances_to_equivalent_reference_mm'and path[5]in('N','A','C'))
        assert type(old)is float and type(new)is float and math.isfinite(old)and math.isfinite(new)
        assert abs(old-new)<=1e-12
        listed.append(dict(path='/'+('/'.join(path)),canonical_value_mm=old,replayed_value_mm=new,absolute_difference_mm=abs(old-new)))
    assert replay['aggregate_primary']==expected['aggregate_primary']and replay['gates']==expected['gates']
    arrays={}
    for case in expected['cases']:
        assert replay['cases'][case]['windows']==expected['cases'][case]['windows']
        with np.load(a.canonical_root/(case+'_evaluation.npz'),allow_pickle=False)as left,np.load(a.replay_root/(case+'_evaluation.npz'),allow_pickle=False)as right:
            assert set(left.files)==set(right.files)
            for key in left.files:
                assert left[key].shape==right[key].shape and left[key].dtype==right[key].dtype and left[key].tobytes()==right[key].tobytes(),(case,key)
            arrays[case]=len(left.files)
    receipt=json.loads((a.replay_root/'replay_comparison.json').read_text())
    assert not receipt['science_exact']and not receipt['full_science_exact']
    assert receipt['primary_and_fixed_window_metrics_and_gates_exact']
    assert receipt['event_scalar_roundoff']==listed
    report=dict(iteration='I-61',completed_utc=datetime.now(timezone.utc).isoformat(),new_physics_runs=0,
        full_science_exact=False,all_primary_fixed_window_metrics_and_gates_exact=True,
        all_evaluation_arrays_byte_exact=arrays,scalar_event_differences=listed,
        maximum_event_roundoff_mm=max(x['absolute_difference_mm']for x in listed),
        science_unchanged_from_initial_failed_system_replay=True,
        replay_wrapper_sha256=sha(a.replay_root/'replay_evaluation.py'),
        scope='Validation-only narrow finite scalar event-norm tolerance. No scalar overwrite, array/boolean correction, gate change or geometric recomputation is hidden.',
        qualification='Cross-version whole scientific JSON is not claimed exact. Producer-version exact replay is a separate receipt.')
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
