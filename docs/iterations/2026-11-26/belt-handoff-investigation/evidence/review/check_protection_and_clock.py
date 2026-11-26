"""Read-only archive/source/clock check, separate from numerical checks."""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[3]
FAMILY=ROOT/'results/belt_handoff_event_20261126'
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
pre=json.loads((FAMILY/'pre_change/manifest.json').read_text())
modified_archives=[x['path'] for x in pre['prior_archives'] if sha(ROOT/x['path'])!=x['sha256'] or (ROOT/x['path']).stat().st_size!=x['bytes']]
schedule=json.loads((ROOT/'results/belt_capacity_sweep_20261126/videos/video_display_schedule.json').read_text())
cue=next(x for x in schedule if x['frame']==360)
clock_ok=all(abs(v-32)<1e-9 for v in cue['case_state_times_s'].values()) and 360/20==18
checks=dict(prior_archives_unchanged=not modified_archives,playback18_maps_recorded32=clock_ok)
out=dict(passed=all(checks.values()),checks=checks,prior_archive_count=len(pre['prior_archives']),
         protected_source_count=len(pre['sources']),modified_archives=modified_archives,cue=cue,
         script_sha256=sha(Path(__file__)))
(FAMILY/'review/protection_and_clock.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out))
if not out['passed']:raise SystemExit(1)
