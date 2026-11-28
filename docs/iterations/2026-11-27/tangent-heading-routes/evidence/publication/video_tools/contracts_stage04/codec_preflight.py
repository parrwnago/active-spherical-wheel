"""Seven synthetic frames verify the I76 sequential codec path; no robot data."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
import imageio.v2 as imageio
import imageio_ffmpeg
import numpy as np
from PIL import Image
import render


def main():
    stage=Path(__file__).parent/'codec_stage01';stage.mkdir(exist_ok=False)
    sources=[Path(__file__).resolve(),Path(render.__file__).resolve()]
    bindings={str(p.relative_to(render.ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    for path in sources:shutil.copy2(path,stage/path.name)
    (stage/'registered_before_checks.json').write_text(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),
        files=bindings,synthetic_frames=7,dimensions=[render.WIDTH,render.HEIGHT],fps=render.FPS,
        physics_steps=0,case_arrays_loaded=False),indent=2)+'\n')
    video=stage/'synthetic_codec_only.mp4'
    colors=np.array([[20+30*i,40+10*i,180-20*i] for i in range(7)],dtype=np.uint8)
    with imageio.get_writer(video,fps=render.FPS,codec='libx264',macro_block_size=2,
        quality=None,ffmpeg_log_level='error',ffmpeg_params=['-crf','21','-preset','fast','-threads','2']) as writer:
        for color in colors:writer.append_data(np.broadcast_to(color,(render.HEIGHT,render.WIDTH,3)))
    decoded=render.decode_sequential(video,stage,7)
    errors={}
    for index in (0,3,6):
        pixels=np.asarray(Image.open(stage/f'decoded_{index:04d}.jpg').convert('RGB'))
        error=float(np.max(np.abs(pixels.mean(axis=(0,1))-colors[index].astype(float))))
        if error>5.:raise AssertionError('Wrong decoded frame/color')
        errors[index]=error
    assert all(hashlib.sha256((render.ROOT/p).read_bytes()).hexdigest()==digest for p,digest in bindings.items())
    report=dict(passed=True,video=decoded,maximum_selected_frame_color_errors=errors,
        synthetic_fixture_only=True,no_robot_state=True,case_arrays_loaded=False,physics_steps=0,
        sources_unchanged=True,ffmpeg_executable=imageio_ffmpeg.get_ffmpeg_exe())
    (stage/'receipt.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=True,decoded_frames=7,selected_indices=[0,3,6],color_errors=errors)),flush=True)


if __name__=='__main__':main()
