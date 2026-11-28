"""Preview-only camera/domain/endpoint QA for all fixed I76 pairs."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
from PIL import Image
BASE=Path(__file__).resolve().parents[2]
ROUTES={'straight':32.,'arc':32.,'eight':120.}
PHASES=('m100','p030','p090')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):
 with p.open('x')as f:json.dump(v,f,indent=2,sort_keys=True);f.write('\n')
def main():
 stage=BASE/'publication/video_preview_stage01/quality';stage.mkdir(exist_ok=False)
 paths=[Path(__file__),BASE/'campaign/outputs_sealed.json']
 for rid,motion in ROUTES.items():
  for phase in PHASES:
   folder=BASE/'publication/video_preview_stage01'/rid/phase
   paths += [folder/'metadata.json',folder/'display_schedule.json',folder/'artifact_manifest.json']
   paths += [folder/f'pose_{t:06.2f}s.jpg'for t in (0.,3.+.5*motion,13.+motion)]
 bindings={str(p.relative_to(BASE)):dict(sha256=sha(p),bytes=p.stat().st_size)for p in paths}
 save(stage/'registered_before_checks.json',dict(utc=datetime.now(timezone.utc).isoformat(),files=bindings,no_physics=True,no_metric_scoring=True))
 shutil.copy2(Path(__file__),stage/'preview_sheets.py')
 cameras=[];rows={}
 for rid,motion in ROUTES.items():
  sheet=Image.new('RGB',(2400,1440),(0,0,0))
  for row,phase in enumerate(PHASES):
   folder=BASE/'publication/video_preview_stage01'/rid/phase
   meta=json.loads((folder/'metadata.json').read_text());cameras.append(meta['shared_camera'])
   assert meta['frames']==6 and meta['planned_visualization_window_s']==[0.,13.+motion]
   assert meta['root_output_seal_verification']['passed']
   expected='rough_stress_outside_moderate_screen'if rid=='eight'else'moderate_passed'
   assert meta['terrain_category']==expected
   for col,t in enumerate((0.,3.+.5*motion,13.+motion)):
    with Image.open(folder/f'pose_{t:06.2f}s.jpg')as im:
     assert im.size==(1600,960)
     sheet.paste(im.convert('RGB').resize((800,480)),(col*800,row*480))
   rows[f'{rid}_{phase}']=dict(stage=str(folder.relative_to(BASE)),planned_visualization_window_s=meta['planned_visualization_window_s'],actual_recorded_windows_s=meta['actual_recorded_windows_s'],episode_completed_by_case=meta['episode_completed_by_case'],terrain_category=meta['terrain_category'])
  sheet.save(stage/f'{rid}_contact_sheet.jpg',quality=95)
 assert len(rows)==9 and all(c==cameras[0]for c in cameras)
 assert all(sha(BASE/p)==b['sha256']for p,b in bindings.items())
 save(stage/'report.json',dict(passed=True,all_nine_pairs=rows,shared_camera_all_eighteen=True,columns='initial / middle of planned motion / final planned time',rows='m100 / p030 / p090',no_physics=True,no_metric_scoring=True,source_inputs_unchanged=True))
 save(stage/'artifact_manifest.json',dict(files={p.name:dict(sha256=sha(p),bytes=p.stat().st_size)for p in stage.iterdir()if p.is_file()}))
 print(json.dumps(dict(passed=True,pairs=len(rows),shared_camera=True)))
if __name__=='__main__':main()
