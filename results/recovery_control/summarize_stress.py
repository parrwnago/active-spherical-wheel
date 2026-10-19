from pathlib import Path
import json, numpy as np
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--original',type=Path,required=True)
parser.add_argument('--revised',type=Path,required=True)
parser.add_argument('--output',type=Path,default=Path(__file__).with_name('stress_comparison.json'))
args=parser.parse_args()
old_root,new_root=args.original,args.revised
args.output.parent.mkdir(exist_ok=True,parents=True)
old=json.loads((old_root/'summaries.json').read_text());new=json.loads((new_root/'summaries.json').read_text())
lookup=lambda runs:{(r['scenario'],r['case']):r for r in runs}
a,b=lookup(old),lookup(new)
records=[]
for key in sorted(a):
 record={'scenario':key[0],'case':key[1]}
 for label,root,source in [('original',old_root,a[key]),('transition_aware',new_root,b[key])]:
  r=dict(source);r.pop('source_file',None)
  with np.load(root/key[0]/key[1]/'rollout.npz',allow_pickle=False) as log:
   v=log['qvel'][-1];r['final_planar_speed_m_s']=float(np.linalg.norm(v[:2]));r['final_angular_speed_rad_s']=float(np.linalg.norm(v[3:6]))
   r['settled_at_end']=bool(not r['failed'] and r['final_tilt_deg']<.5 and r['final_planar_speed_m_s']<.03 and r['final_angular_speed_rad_s']<.08)
  record[label]=r
 records.append(record)
summary={'baseline_git_commit':'bb05156b6cfe933e28c0eb8a3a2d3c7f762754d4','cases':len(records),'original_falls':sum(r['original']['failed'] for r in records),'revised_falls':sum(r['transition_aware']['failed'] for r in records),'original_settled_at_end':sum(r['original']['settled_at_end'] for r in records),'revised_settled_at_end':sum(r['transition_aware']['settled_at_end'] for r in records),'survival_improved_cases':sum(r['original']['failed'] and not r['transition_aware']['failed'] for r in records),'survival_regressed_cases':sum(not r['original']['failed'] and r['transition_aware']['failed'] for r in records),'physics_unchanged':True,'configuration':'config/model.yaml','settled_definition':'No fall; final tilt<0.5deg, planar speed<0.03m/s, total base angular speed<0.08rad/s. This does not require return to target position.','requested_histories':'3 scenarios ×3 force magnitudes [0.5,1,2]N ×3 patterns [sameY,alternatingY,cross+X+Y-X-Y] ×3(duration,spacing)=[(.1,.4),(.2,.25),(.3,.15)]s;8pulses from1s;10s runs. Pulses add without clipping.','revised_controller':'transition_aware with incoming synchronization enabled','failure_definition':'Existing fork/platform floor contact or tilt>70deg; identical in both controllers.','limits':'Empirical sampled comparison; does not certify global stability or hardware recovery. Strong4N combined same-direction trains still fail; several survivors retain position/tilt bias.','records':records}
args.output.write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='records'},indent=2))
