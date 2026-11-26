from pathlib import Path
import hashlib,json,re,datetime,subprocess
from urllib.parse import unquote
import numpy as np
ROOT=Path.cwd();BASE=ROOT/'results/causal_replay_20261126';DOC=ROOT/'docs/iterations/2026-11-26/causal-replay';OUT=BASE/'publication_review'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text())
report={'iteration':'I-55','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Publication/evidence checks, not new physics or repository test-suite execution'}
selected=0
for mp in [DOC/'evidence/manifest.json',ROOT/'docs/iterations/2026-11-26/acceleration-spin-cause/evidence/manifest.json']:
 for a in read(mp)['files']:
  p=mp.parent/a['path'];assert p.stat().st_size==a['bytes'] and sha(p)==a['sha256'],str(p)
  assert sha(ROOT/a['origin'])==a['sha256'],a['origin'];selected+=1
report['I54_I55_evidence_records_byte_verified']=selected
m=read(DOC/'data/manifest.json');arrays=0
for a in m['files']:
 p=DOC/'data'/a['path'];assert p.stat().st_size==a['bytes'] and sha(p)==a['sha256']
 if 'origin' in a:assert sha(ROOT/a['origin'])==a['sha256']
 for s in a.get('sources',[]):
  src=ROOT/s['path'];assert sha(src)==s['sha256']
  with np.load(src,allow_pickle=False) as z,np.load(p,allow_pickle=False) as sub:
   mask=(z['time_s']>=31.4-1e-9)&(z['time_s']<=32.5+1e-9);assert int(mask.sum())==221
   for k in z.files:
    x=sub[s['kind']+'__'+k];y=z[k][mask];assert x.dtype==y.dtype
    assert np.array_equal(x,y,equal_nan=True) if x.dtype.kind in 'fc' else np.array_equal(x,y)
    arrays+=1
report['portable_data']={'artifacts':len(m['files']),'bytes':m['bytes'],'exact_subset_arrays_verified':arrays,'resampling':False}
proto=read(BASE/'protocol.json');cases=[c['id'] for c in proto['cases']];cases_verified=[]
for c in cases:
 folder=BASE/'campaign'/c;p=read(folder/'i55_provenance.json');assert p['success']
 assert p['registered_protocol_sha256']==sha(BASE/'protocol.json')
 for n,h in p['executed_source_sha256'].items():assert sha(ROOT/n)==h,n
 for n,h in p['model_sha256'].items():assert sha(Path(p['models'][n]))==h
 for a in read(folder/'i55_artifact_manifest.json')['files']:
  assert sha(folder/a['path'])==a['sha256'],str(folder/a['path'])
 cases_verified.append(c)
report['complete_case_manifests_verified']=cases_verified
required=read(ROOT/'research/frozen_workspace/manifest.json')['required_live_source_sha256']
for n,h in required.items():assert sha(ROOT/n)==h,n
report['unchanged_required_runtime_sources']=len(required)
allowed={'ITERATION_LOG.md','docs/TEAMMATE_HANDOFF.md','docs/iterations/2026-11-26/README.md'}
changed=[n for n,h in proto['input_sha256'].items() if sha(ROOT/n)!=h]
assert set(changed)<=allowed,changed
report['registered_input_changes_only_live_documentation']=changed
for n,h in read(BASE/'analysis/stage01/registered_before_reduction.json').get('input_sha256',{}).items():assert sha(ROOT/n)==h,n
checks=read(BASE/'native_parity_gate.json');assert checks['passed'];report['native_parity_gate_passed']=True
canonical=read(BASE/'analysis/final_report.json');assert canonical['technical_causal_prerequisites_pass']
assert sha(BASE/'analysis/stage01/report.json')==canonical['canonical_report_sha256']
for n,h in canonical['bound_report_sha256'].items():assert sha(BASE/n)==h,n
report['canonical_analysis_bound_reports_verified']=True
mds=[DOC/'README.md',DOC/'data/README.md',ROOT/'docs/TEAMMATE_HANDOFF.md',ROOT/'docs/iterations/2026-11-26/README.md',ROOT/'ITERATION_LOG.md',ROOT/'docs/iterations/2026-11-26/acceleration-spin-cause/README.md']
broken=[];linkcount=0
for md in mds:
 for target in re.findall(r'\[[^\]]*\]\(([^\n]+?)\)',md.read_text()):
  target=target.strip().strip('<>')
  if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target) or target.startswith('#'):continue
  target=re.sub(r':\d+$','',unquote(target.split('#')[0]))
  if not target:continue
  linkcount+=1
  if not (md.parent/target).exists():broken.append({'source':str(md.relative_to(ROOT)),'target':target})
 assert not any(line.rstrip()!=line for line in md.read_text().splitlines()),str(md)
assert not broken,broken
report['live_links']={'checked':linkcount,'missing':broken,'scope':'Live guides/index/newrecords; archived stage references can name original local raw files.'}
patterns={'github_token':rb'\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{50,})\b','private_key':rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----','aws_access_key':rb'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b','openai_token':rb'\bsk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{35,}\b'}
paths=[*DOC.rglob('*'),*(ROOT/'docs/iterations/2026-11-26/acceleration-spin-cause').rglob('*'),*mds[2:5]]
findings=[];texts=0
for p in paths:
 if not p.is_file():continue
 b=p.read_bytes()
 if b'\0' in b[:8192]:continue
 try:b.decode('utf-8')
 except UnicodeDecodeError:continue
 texts+=1
 for tag,pat in patterns.items():
  for match in re.finditer(pat,b):findings.append({'path':str(p.relative_to(ROOT)),'line':b[:match.start()].count(b'\n')+1,'pattern':tag})
assert not findings,findings
report['secret_pattern_scan']={'text_files':texts,'findings':findings}
tracked=set(subprocess.check_output(['git','diff','--name-only'],text=True).splitlines());assert tracked==allowed,tracked
report['tracked_changes_only_live_documentation']=sorted(tracked)
report['unrelated_untracked_work_preserved']=['firmware/','HAMR_Ordinary_Caster_DIY_Encoder_Codex_Handoff_ESP32C6.md']
report['passed']=True
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
