from pathlib import Path
import datetime,hashlib,json,re,subprocess
from urllib.parse import unquote
ROOT=Path.cwd();BASE=ROOT/'results/acceleration_spin_cause_20261126';DOC=ROOT/'docs/iterations/2026-11-26/acceleration-spin-cause';OUT=BASE/'review'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
reg=read(BASE/'registration.json')
allowed={'ITERATION_LOG.md','docs/TEAMMATE_HANDOFF.md','docs/iterations/2026-11-26/README.md'}
changed=[n for n,h in reg['input_sha256'].items() if sha(ROOT/n)!=h]
assert set(changed)<=allowed,changed
m=read(DOC/'evidence/manifest.json')
for a in m['files']:
 p=DOC/'evidence'/a['path'];assert p.stat().st_size==a['bytes'] and sha(p)==a['sha256'],str(p)
 assert sha(ROOT/a['origin'])==a['sha256'],a['origin']
# Prior publication remains immutable, including its exact original sources/media.
old_checked=0
for name in ['docs/iterations/2026-11-26/teammate-handoff/evidence/manifest.json','docs/handoff/media/manifest.json']:
 p=ROOT/name
 for a in read(p)['files']:
  assert sha(p.parent/a['path'])==a['sha256']
  if 'origin' in a: assert sha(ROOT/a['origin'])==a['sha256']
  old_checked+=1
sources=read(ROOT/'research/frozen_workspace/manifest.json')['required_live_source_sha256']
for n,h in sources.items():assert sha(ROOT/n)==h,n
# Cross-check headline quantities against the sealed numerical reports.
spin=read(BASE/'spin_audit/compact_stage02/summary.json')
full=read(BASE/'spin_audit/stage01/report.json')
w=full['cases']['m100_R']['windows']['after_gain_switch']
assert abs(w['heading_net_turns']-2.748353423847244)<1e-10
assert abs(w['world_z_rate_peak_abs_rad_s']-6.634509733698498)<1e-10
assert w['valid_heading_samples']==w['samples']
g=read(BASE/'gain_audit/stage01/report.json')
for case,v in g['cases'].items():assert v['maximum_gain_reconstruction_error']==0. and v['loaded_rows_verified']==8801
x=next(e for e in g['cases']['m100_R']['events'] if abs(e['time_s']-32.25)<1e-8)
assert not x['all_inputs_feasible_blend'] and x['grid_minimax_worst_input_fraction']>5.
assert abs(x['LQR_step_due_support_mix'][2]+7.429230889697348)<1e-10
mech=read(BASE/'mechanics_audit/summary.json');x=next(e for e in mech['intervals'] if e['start_s']==32.25 and e['end_s']==32.275)
assert abs(x['delta_H_fork_carrier_z_kg_m2_s']-0.02291534732482155)<1e-12
assert abs(x['delta_H_shells_z_kg_m2_s']+0.0215299039499458)<1e-12
h=read(BASE/'history_audit/compact_stage02_entering/summary.json')['cases']['m100_R']
assert abs(h['desired_up_net_change_deg']-15.621)<.001
assert h['actual_absolute_tilt_start_end_deg'][1]<h['actual_absolute_tilt_start_end_deg'][0]
mds=[DOC/'README.md',ROOT/'docs/TEAMMATE_HANDOFF.md',ROOT/'docs/iterations/2026-11-26/README.md',ROOT/'ITERATION_LOG.md',DOC/'evidence/gain_audit/README.md']
links=0;broken=[]
for md in mds:
 for target in re.findall(r'\[[^\]]*\]\(([^\n]+?)\)',md.read_text()):
  target=target.strip().strip('<>')
  if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target) or target.startswith('#'):continue
  target=re.sub(r':\d+$','',unquote(target.split('#')[0]))
  if not target:continue
  links+=1
  if not (md.parent/target).exists():broken.append({'source':str(md.relative_to(ROOT)),'target':target})
assert not broken,broken
# Current new live prose has no whitespace errors; archives retain exact executed bytes.
for md in mds:
 assert not any(s.rstrip()!=s for s in md.read_text().splitlines()),str(md)
patterns={'github_token':rb'\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{50,})\b','private_key':rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----','aws_access_key':rb'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b','openai_token':rb'\bsk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{35,}\b'}
findings=[];texts=0
for p in [*DOC.rglob('*'),ROOT/'ITERATION_LOG.md',ROOT/'docs/TEAMMATE_HANDOFF.md',ROOT/'docs/iterations/2026-11-26/README.md']:
 if not p.is_file():continue
 b=p.read_bytes()
 try:b.decode('utf-8')
 except UnicodeDecodeError:continue
 texts+=1
 for tag,pattern in patterns.items():
  for match in re.finditer(pattern,b):findings.append({'path':str(p.relative_to(ROOT)),'line':b[:match.start()].count(b'\n')+1,'pattern':tag})
assert not findings,findings
status=subprocess.check_output(['git','status','--short'],text=True)
expected={'ITERATION_LOG.md','docs/TEAMMATE_HANDOFF.md','docs/iterations/2026-11-26/README.md'}
tracked=[s for s in subprocess.check_output(['git','diff','--name-only'],text=True).splitlines()]
assert set(tracked)==expected,tracked
report={'iteration':'I-54','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'passed':True,'selected_evidence_files':len(m['files']),'selected_evidence_bytes':m['bytes'],'source_runtime_changes':[],'intentional_live_document_changes':changed,'unchanged_required_live_sources':len(sources),'prior_publication_files_reverified':old_checked,'current_relative_links_checked':links,'missing_links':broken,'headline_values_verified':True,'heading_valid_samples':w['samples'],'gain_lookup_rows_matched_per_arm':8801,'gain_lookup_max_abs_difference':0.,'secret_pattern_scan':{'text_files':texts,'findings':findings},'scope':'Saved-array identities and private static kinematics/inertia calculations, not a new trajectory campaign or full repo-suite test run. No source runtime changes. Whole unique physical cause remains unproven.','git_status':status.splitlines()}
(OUT/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
