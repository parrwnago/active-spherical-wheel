"""Exercise the real local HTTP service, MuJoCo worker and EGL JPEG renderer."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer
from io import BytesIO
from PIL import Image
from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.model import generate_model
from spherical_wheel.lab_server import LabService, _handler

root=Path(__file__).resolve().parent
lab=SimulationLab(model_path=generate_model(output_dir=root/'model'))
service=LabService(lab,width=480,height=360,fps=8)
server=ThreadingHTTPServer(('127.0.0.1',0),_handler(service))
server.daemon_threads=True
server.block_on_close=False
base=f'http://127.0.0.1:{server.server_port}'
thread=threading.Thread(target=server.serve_forever,daemon=True)

def get(path):
  with urllib.request.urlopen(base+path,timeout=10) as response:return response.read(),response.headers.get_content_type()
def state():return json.loads(get('/api/state')[0])
def command(payload):
  request=urllib.request.Request(base+'/api/command',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json','Origin':base})
  with urllib.request.urlopen(request,timeout=10) as response:return json.load(response)

service.start();thread.start()
try:
  info=json.loads(get('/api/info')[0]);assert info['engine']=='MuJoCo'
  command({'action':'reset','scenario':'polar','initial_tilt_deg':[2.,0.]})
  command({'action':'push','force_N':[0.,.5,0.],'duration_s':.1})
  time.sleep(.15)
  paused=state();assert paused['time_s']==0.
  command({'action':'resume'})
  deadline=time.monotonic()+10
  while state()['time_s']<.20:
    assert time.monotonic()<deadline
    time.sleep(.025)
  command({'action':'pause'})
  snapshot=state();assert not snapshot['failed'];assert not snapshot['server']['physics_error']
  assert snapshot['time_s']>=.2 and snapshot['active_force_N']==[0.,0.,0.]
  csv,_=get('/api/export');assert b'force_y_N' in csv and b'belt_A_speed_m_s' in csv
  deadline=time.monotonic()+10
  while service.frame_number<1:
    assert time.monotonic()<deadline
    assert not service.render_error,service.render_error
    time.sleep(.025)
  frame,mime=get('/frame.jpg');assert mime=='image/jpeg'
  image=Image.open(BytesIO(frame));assert image.size==(480,360)
  (root/'frame.jpg').write_bytes(frame)
  (root/'run.csv').write_bytes(csv)
  report={'result':'passed','base_url_during_test':base,'initial_paused_time_s':paused['time_s'],'final_time_s':snapshot['time_s'],
          'scenario':snapshot['scenario'],'tilt_deg':snapshot['tilt_deg'],'failed':snapshot['failed'],'frame_size':list(image.size),'frame_bytes':len(frame),
          'physics_error':snapshot['server']['physics_error'],'render_error':service.render_error,'sample_count':snapshot['sample_count'],
          'checks':['real MuJoCo/EGL JPEG','finite pushed polar case via HTTP','pause preserves simulation-time pulse duration','JSON state/info/command','CSV actual force and belt inputs']}
  (root/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
finally:
  service.stop();server.shutdown();server.server_close()
