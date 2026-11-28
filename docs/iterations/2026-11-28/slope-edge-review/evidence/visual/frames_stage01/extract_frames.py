"""Decode original recorded comparison frames; no rendering or physics."""
from pathlib import Path
import hashlib
import json
import imageio.v2 as imageio

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parent
VIDEO = ROOT / 'results/tangent_heading_routes_20261127/publication/videos_stage01/eight/m100/eight_m100_N_T.mp4'
TIMES = [99., 100., 100.5, 101., 101.5, 102., 102.5, 103.]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = BASE / 'frames_stage01'
    out.mkdir(exist_ok=False)
    receipt = {'iteration': 'I-80', 'scope': 'Decode original saved video only; no controller, observer, renderer or physics replay.',
               'input': str(VIDEO.relative_to(ROOT)), 'input_sha256': sha(VIDEO),
               'source_sha256': sha(Path(__file__)), 'requested_times_s': TIMES,
               'setup_note': 'System ffprobe is unavailable; use installed imageio/packaged FFmpeg decoder.'}
    (out / 'registered_before_video_read.json').write_text(json.dumps(receipt, indent=2) + '\n')
    (out / 'extract_frames.py').write_bytes(Path(__file__).read_bytes())
    reader = imageio.get_reader(VIDEO, 'ffmpeg')
    metadata = reader.get_meta_data()
    fps = float(metadata['fps'])
    frames = []
    for when in TIMES:
        index = round(when * fps)
        frame = reader.get_data(index)
        dest = out / f'frame_{when:06.2f}s.png'
        imageio.imwrite(dest, frame)
        frames.append({'requested_time_s': when, 'frame_index': index,
                       'decoded_video_time_s': index / fps, 'path': dest.name,
                       'sha256': sha(dest), 'shape': list(frame.shape)})
    reader.close()
    assert sha(VIDEO) == receipt['input_sha256']
    (out / 'report.json').write_text(json.dumps({'iteration': 'I-80', 'decoder_metadata': metadata,
                    'frames': frames, 'video_unchanged': True, 'no_new_physics': True}, indent=2) + '\n')
    manifest = [{'path': p.name, 'sha256': sha(p), 'bytes': p.stat().st_size}
                for p in sorted(out.iterdir()) if p.is_file()]
    (out / 'manifest.json').write_text(json.dumps({'iteration': 'I-80', 'files': manifest}, indent=2) + '\n')
    print(json.dumps({'decoded_frames': len(frames), 'fps': fps, 'video_unchanged': True}))


if __name__ == '__main__':
    main()
