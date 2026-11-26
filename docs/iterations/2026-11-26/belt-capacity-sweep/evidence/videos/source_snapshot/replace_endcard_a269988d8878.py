"""Preserved first encoding -> corrected endcard; reuse all original motion frames."""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil

import imageio.v2 as imageio

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
PRIOR=HERE/'development/encoded_stage01_endcard_wording'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    spec=importlib.util.spec_from_file_location('capacity_card',HERE/'render_capacity.py')
    renderer=importlib.util.module_from_spec(spec);spec.loader.exec_module(renderer)
    report=json.loads((HERE.parent/'analysis/stage01/report.json').read_text())
    original=json.loads((PRIOR/'video_metadata.json').read_text())
    original_video=PRIOR/'belt_capacity_comparison.mp4'
    assert sha(original_video)==original['video']['sha256']
    source=Path(__file__).resolve()
    for path in (source,HERE/'render_capacity.py'):
        dest=HERE/'source_snapshot'/f'{path.stem}_{sha(path)[:12]}.py'
        if not dest.exists():shutil.copy2(path,dest)
    card=renderer.resultcard(report,original['motion_060_090_identical'])
    imageio.imwrite(HERE/'video_resultcard.jpg',card,quality=94)
    output=HERE/'belt_capacity_comparison_corrected.mp4'
    assert not output.exists()
    prefix_hash=hashlib.sha256()
    with imageio.get_reader(original_video)as reader,imageio.get_writer(output,fps=20,codec='libx264',quality=8,macro_block_size=2,ffmpeg_log_level='error')as writer:
        for index,pixels in enumerate(reader):
            if index>=481:break
            prefix_hash.update(pixels.tobytes());writer.append_data(pixels)
        assert index==481
        for _ in range(80):writer.append_data(card)
    output.replace(HERE/'belt_capacity_comparison.mp4')
    output=HERE/'belt_capacity_comparison.mp4'
    with imageio.get_reader(output)as reader:
        count=reader.count_frames();meta=reader.get_meta_data()
        for index in (0,40,280,560):imageio.imwrite(HERE/f'decoded_frame_{index:04d}.jpg',reader.get_data(index),quality=94)
    assert count==561 and meta['fps']==20
    original['source_sha256']=sha(HERE/'render_capacity.py')
    original['input_sha256'][str((HERE/'render_capacity.py').relative_to(ROOT))]=sha(HERE/'render_capacity.py')
    for path in (source,original_video,PRIOR/'video_metadata.json'):
        original['input_sha256'][str(path.relative_to(ROOT))]=sha(path)
    original['endcard_correction']={
        'prior_video':str(original_video.relative_to(ROOT)),
        'reason':'Baseline gates already fail; neither raised arm passes them. Moving RMS and selected turn metrics worsen.',
        'actual_prefix_frames_reused':481,
        'decoded_prefix_sha256_before_reencoding':prefix_hash.hexdigest(),
        'method':'Decode existing first481frames and reencode them unchanged as source pixels; replace only final80cardframes. No motion rerender or physics.',
    }
    original['video']=dict(path=str(output.relative_to(ROOT)),sha256=sha(output),bytes=output.stat().st_size,frames=count,fps=20,duration_s=meta['duration'])
    (HERE/'video_metadata.json').write_text(json.dumps(original,indent=2)+'\n')
    print(json.dumps(original['video'],indent=2))

if __name__=='__main__':main()
