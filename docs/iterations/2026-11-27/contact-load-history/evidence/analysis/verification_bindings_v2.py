"""Post-result finite-pattern and exact per-frame decision bindings.

No quantization or scientific value replacement. Full floating point arrays
remain omitted; these bindings cannot establish pointwise closeness of a new,
byte-inexact floating array.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json
import numpy as np

CASES=('m100_B','p030_B','p090_B')
MODELS=('M0','M1','M2','M3')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def describe(value):
    result=dict(dtype=value.dtype.str,shape=list(value.shape),raw_sha256=hashlib.sha256(value.tobytes()).hexdigest())
    if np.issubdtype(value.dtype,np.floating):
        pattern=np.zeros(value.shape,np.uint8);pattern[np.isnan(value)]=1;pattern[np.isposinf(value)]=2;pattern[np.isneginf(value)]=3
        result['nonfinite_pattern_sha256']=hashlib.sha256(pattern.tobytes()).hexdigest()
    return result
def read_npz(path):
    with np.load(path,allow_pickle=False) as archive:return {k:archive[k].copy() for k in archive.files}


def decisions(frames):
    output={};accepted=frames['i60_accepted'];current=frames['H_F_valid']
    for model in MODELS:
        F=frames[model+'_F_score'];N=frames[model+'_Nalt_score']
        Fvalid=frames[model+'_F_score_valid'];Nvalid=frames[model+'_Nalt_score_valid'];valid=Fvalid&Nvalid
        delta=F-N;preference=np.full(len(F),-2,np.int8)
        preference[valid&(delta < -1e-9)]=-1;preference[valid&(delta > 1e-9)]=1
        preference[valid&(abs(delta)<=1e-9)]=0
        compatible=Fvalid&(F<=1.)
        shadow=accepted&current&compatible if model in ('M0','M1') else accepted&current&(~Fvalid|compatible)
        output[model+'_preference']=preference;output[model+'_compatible']=compatible;output[model+'_shadow_keep']=shadow
    return output


def bind(reference,features,output):
    output.mkdir(parents=True,exist_ok=False)
    (output/'registered_before_binding.json').write_text(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),status='post-result verification binding, before this source numerical use; no science changes'),indent=2)+'\n')
    analysis={};decision={};feature={}
    for path in sorted(reference.glob('*.npz')):
        values=read_npz(path);analysis[path.name]={k:describe(v) for k,v in values.items()}
    for case in CASES:
        decision[case]={k:describe(v) for k,v in decisions(read_npz(reference/(case+'_frames.npz'))).items()}
        feature[case]={k:describe(v) for k,v in read_npz(features/case/'history_features.npz').items()}
    result=dict(iteration='I-64',source_sha256=sha(Path(__file__)),analysis=analysis,decisions=decision,features=feature,
        qualification='Exact bool/int/decision and nonfinite-pattern checks; byte-inexact float closeness is not derivable from hashes. No normalized/quantized reference values.')
    (output/'bindings.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(analysis_arrays=sum(len(v) for v in analysis.values()),decision_arrays=sum(len(v) for v in decision.values()),feature_arrays=sum(len(v) for v in feature.values()))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('reference','features','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();bind(a.reference.resolve(),a.features.resolve(),a.output.resolve())
