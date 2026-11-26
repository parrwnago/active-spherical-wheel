"""I53 compact descriptive follow-up of the declared event audit."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil
import numpy as np
H=Path(__file__).resolve().parent;ROOT=H.parents[2];OLD=ROOT/'results/contact_scheduling_joint_tracking_20261126';TIMES=[31.715,31.765,31.77,31.815,32.15,32.245,32.25,32.255,32.295,32.3,32.305,32.37,32.64,33.235,33.32]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {k:safe(x)for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 return v
def save(p,v):p.write_text(json.dumps(safe(v),indent=2,allow_nan=False)+'\n')
def main():
 out=H/'compact_stage02';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'compact_events.py');mapping=OLD/'campaign/resolved_case_paths.json';folder=ROOT/json.loads(mapping.read_text())['cases']['m100_R'];inputs=[H/'stage01/report.json',folder/'precision.npz',mapping]
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs};save(out/'before_consolidation.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=hashes,fixed_followup_times_s=TIMES,scope='Descriptivefollowupafterprimaryeventselection; compactonly. Same-statefrozenKpre-syncLQR difference K_jointvel*(USED−NATIVE) isnotacounterfactualtrajectory/removingguardhistory. No model/controller/physics calls.'))
 primary=json.loads((H/'stage01/report.json').read_text());outcome={'scope':'SavedI52eventstudyonly; no newphysicalrun orcontrollerreplay.','video_mapping':primary['frame_mapping'],'windows_s':primary['registered_windows_s'],'cases':{},'R_timeline':[],'limitations':primary['limits']}
 for case,d in primary['cases'].items():
  ws={}
  for w,s in d['windows'].items():
   scalar={k:v for k,v in s.items()if k!='events'};scalar['event_times_s']={k:v['time_s']for k,v in s['events'].items()};ws[w]=scalar
  peak=d['whole_trace_max_timed_error'];outcome['cases'][case]=dict(windows=ws,whole_trace_peak={'time_s':peak['time_s'],'timed_error_m':peak['timed_error_m'],'local_path_distance_m':peak['local_path_distance_m']})
 with np.load(folder/'precision.npz')as z:p={k:z[k].copy()for k in z.files}
 selected=primary['cases']['m100_R']['selected_snapshots']
 for at in TIMES:
  row=min(selected,key=lambda x:abs(x['time_s']-at));assert abs(row['time_s']-at)<1e-7
  i=row['index'];row['same_state_guard_effect_on_pre_sync_LQR']=p['exact_feedback_gain'][i,:,15:18]@(p['exact_guarded_joint_reference'][i]-p['exact_native_joint_reference'][i]);outcome['R_timeline'].append(row)
 outcome['interpretation']={'identified_command_mechanism':'Ralreadyhaslargelean/yawbeforehandoff.31.770estimatedshell→belt routingturnslargeLQRdemandintobeltcap.32.250true+estimatedbelt→shellroutingchangesflat-bankgainrepresentationandproducesdominantattitude/velocitytorquedemand, notareference-slewlimitviolation.','time_boundary':'32.250currentgain/commandcanonlyactover32.250–32.255; itcannotcausevelocityincrementalreadyending32.250. Subsequentrecordedburstisconsistentwithsaturatedfeedback/contacttransfer, notisolatedcausalproof.','N_limit':'Nislessfastinthisfixedwindowandhaslower50msaccelerationRMS, but5mscontact/commandtransientsremain. Visuallysmootherisnotacompleteperformanceclaim.'}
 save(out/'summary.json',outcome)
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('Compacteventsummarycomplete')
if __name__=='__main__':main()
