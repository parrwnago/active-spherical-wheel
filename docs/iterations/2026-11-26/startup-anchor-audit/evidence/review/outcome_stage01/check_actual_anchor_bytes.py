"""Supplemental post-outcome byteidentity check; no physics or coefficient fitting."""
from pathlib import Path
import hashlib,json,numpy as np
from datetime import datetime,timezone
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];CASES=('m100_B','m100_P','p030_B','p030_P','p090_B','p090_P')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def equal(a,b):return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
inputs=[HERE/(c+'_independent_counterfactuals.npz') for c in CASES]+[ROOT/'results/balance_priority_20261126/campaign/runs'/c/'precision.npz' for c in CASES]
record=dict(registered_utc=datetime.now(timezone.utc).isoformat(),scope='Supplementalcheckafterindependentnullresult, before thisbyteidentityarithmetic. No gating/thresholdchanges.',script_sha256=sha(Path(__file__)),input_sha256={str(p.relative_to(ROOT)):sha(p) for p in inputs})
(HERE/'actual_anchor_bytecheck_registered.json').write_text(json.dumps(record,indent=2)+'\n');result={}
for cid in CASES:
 with np.load(HERE/(cid+'_independent_counterfactuals.npz')) as z, np.load(ROOT/'results/balance_priority_20261126/campaign/runs'/cid/'precision.npz') as raw:
  anchor=cid.rsplit('_',1)[0]+'_B';result[cid]={}
  for mode in ('row_only','trim_only','combined'):
   req=z['base_native_request']+z[mode+'__'+anchor+'__requested_delta'];clipped=np.clip(req,-raw['exact_input_bounds'],raw['exact_input_bounds']);result[cid][mode]=dict(native_request_byte_exact=equal(req,raw['exact_native_requested']),native_clipped_byte_exact=equal(clipped,raw['exact_native_applied']))
assert all(all(v.values()) for c in result.values() for v in c.values())
assert all(sha(ROOT/p)==h for p,h in record['input_sha256'].items())
(HERE/'actual_anchor_bytecheck.json').write_text(json.dumps(dict(all_passed=True,results=result),indent=2)+'\n');print('All18actual-anchor request/clippedbyteidentity checks pass; correlated algebra controls, no physics.')
