from pathlib import Path
import json, tempfile, numpy as np
from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.model import load_config,generate_model
cfg=load_config();cfg['simulation']['timestep_s']=.0005
workspace=tempfile.TemporaryDirectory(prefix='asw-half-step-')
lab=SimulationLab(cfg,generate_model(cfg,Path(workspace.name)/'model'))
results=[]
for scenario,case,duration,spacing,pattern in [('shell','repeated_lateral_overlap',.3,.15,[(0,.5,0)]),('polar','cross_axis_transfer',.2,.25,[(1,0,0),(0,1,0),(-1,0,0),(0,-1,0)])]:
 lab.command({'action':'reset','scenario':scenario});events={round((1+i*spacing)/lab.dt):pattern[i%len(pattern)] for i in range(8)}
 for i in range(round(12/lab.dt)):
  if i in events:lab.command({'action':'push','force_N':events[i],'duration_s':duration})
  lab.advance()
  if lab.failed:break
 s=lab.state();r={'case':case,'scenario':scenario,'timestep_s':.0005,'control_interval_s':lab.dt,'completed_duration_s':s['time_s'],'failed':s['failed'],'peak_tilt_deg':max(x['tilt_deg'] for x in lab.samples),'final_tilt_deg':s['tilt_deg'],'final_target_error_m':s['tracking_error_m'],'physical_input_bounds':lab.plant.bounds.tolist()};results.append(r);print(json.dumps(r),flush=True)
 assert not s['failed']
Path(__file__).with_suffix('.json').write_text(json.dumps({'description':'0.5 ms physics / 5 ms control repeat with redesigned local gains; not fixed-gain numerical robustness.','results':results},indent=2)+'\n')
