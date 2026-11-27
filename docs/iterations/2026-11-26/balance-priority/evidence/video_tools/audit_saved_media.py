"""Independent fixed-clock QA for lead/broader I58 saved-state media.

Unlike the original I56-specific audit, this also retains an ended case and
accepts a preregistered slow window that was explicitly skipped for coverage.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    stage, output = args.stage.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError('Preserve earlier media QA')
    output.mkdir(parents=True)
    metadata_path, schedule_path = stage/'metadata.json', stage/'display_schedule.json'
    metadata = json.loads(metadata_path.read_text())
    schedule = json.loads(schedule_path.read_text())
    hashes = dict(metadata_sha256=sha(metadata_path), schedule_sha256=sha(schedule_path))
    report = dict(iteration='I-58', started_utc=datetime.now(timezone.utc).isoformat(),
                  source_sha256=sha(Path(__file__)), input_hashes=hashes, videos={})
    (output/'registered_before_decode.json').write_text(json.dumps(report, indent=2)+'\n')
    if 'videos' in metadata:
        videos = metadata['videos']
    else:
        videos = {'broader_overview': metadata['video']}
    for name, item in videos.items():
        path = ROOT/item['path']
        assert sha(path) == item['sha256']
        rows = [row for row in schedule if row.get('video', name) == name]
        chapters = list(dict.fromkeys(row.get('phase_key', 'lead') for row in rows))
        clocks, sheets = [], []
        with imageio.get_reader(path) as movie:
            info = movie.get_meta_data()
            assert movie.count_frames() == item['frames']
            assert abs(info['fps']-20) < 1e-9 and list(info['size']) == [1920, 1080]
            assert abs(info['duration']-item['frames']/20) < .051
            for chapter in chapters:
                chapter_rows = [row for row in rows if row.get('phase_key', 'lead') == chapter]
                moments = [28, 30, 32, 34, 36] if name == 'slow_28_36' else [0, 18, 28, 32, 36, 44]
                images = []
                for moment in moments:
                    row = next(row for row in chapter_rows if abs(row['recorded_time_s']-moment) < 1e-9)
                    frame = row['frame']
                    assert abs(row['movie_time_s']-frame/20) < 1e-9
                    for key, sample in row['case_samples'].items():
                        coverage_key = key if chapter == 'lead' else chapter+'_'+key
                        last = metadata['coverage'][coverage_key]['last_s']
                        assert sample['saved_time_s'] <= last+1e-9
                        assert sample['ended'] == (moment > last+1e-7)
                        if not sample['ended']:
                            assert abs(sample['saved_time_s']-moment) <= .002501
                    pixels = movie.get_data(frame)
                    assert pixels.shape == (1080, 1920, 3)
                    imageio.imwrite(output/f'{name}_{chapter}_{moment:05.2f}s_frame{frame:04d}.jpg', pixels, quality=95)
                    images.append(Image.fromarray(pixels).resize((960, 540), Image.Resampling.LANCZOS))
                    clocks.append(dict(chapter=chapter, recorded_time_s=moment, frame=frame,
                                       movie_time_s=frame/20, samples=row['case_samples']))
                sheet = Image.new('RGB', (1920, 54+((len(images)+1)//2)*578), (10, 20, 31))
                draw = ImageDraw.Draw(sheet)
                draw.text((20, 10), f'I58 {name} / {chapter}: fixed decoded-frame QA, neutral labels',
                          font=ImageFont.truetype(FONT, 28), fill=(240, 230, 200))
                for index, (image, moment) in enumerate(zip(images, moments)):
                    x, y = index%2*960, 54+index//2*578
                    draw.text((x+12, y+4), f'Recorded {moment:.2f} s', font=ImageFont.truetype(FONT, 20), fill=(222, 234, 245))
                    sheet.paste(image, (x, y+36))
                sheet_path = output/f'{name}_{chapter}_contact_sheet.jpg'
                sheet.save(sheet_path, quality=95)
                sheets.append(sheet_path.name)
        report['videos'][name] = dict(sha256=item['sha256'], bytes=path.stat().st_size,
            frames=item['frames'], fps=20, duration_s=info['duration'], chapters=chapters,
            decoded_fixed_clocks=clocks, contact_sheets=sheets)
    assert hashes == dict(metadata_sha256=sha(metadata_path), schedule_sha256=sha(schedule_path))
    report['finished_utc'] = datetime.now(timezone.utc).isoformat()
    report['slow_window_skip_reason'] = metadata.get('slow_window_skip_reason')
    report['qa'] = dict(encoded_clock_checks_passed=True, saved_pose_clocks_passed=True,
        no_observation_extended=True, all_fixed_phases_and_cases_retained=True,
        scientific_acceptance_not_inferred_from_video=True)
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (output/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=p.name, sha256=sha(p), bytes=p.stat().st_size) for p in sorted(output.iterdir()) if p.is_file() and p.name != 'artifact_manifest.json']), indent=2)+'\n')
    print(json.dumps(report['qa'], indent=2))


if __name__ == '__main__':
    main()
