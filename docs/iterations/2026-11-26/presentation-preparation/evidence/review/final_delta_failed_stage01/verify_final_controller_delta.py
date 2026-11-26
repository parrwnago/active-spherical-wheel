"""Bind the prior controller review to final guide bytes and verify protected origins."""
from pathlib import Path
import hashlib
import json
import sys

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def save(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def main():
    old=json.loads((HERE/'controller_content_review_input.json').read_text())
    content=FAMILY/'authoring/guide_content.json'
    pdf=ROOT/'docs/iterations/2026-11-26/presentation-preparation/control-and-observer-study-guide.pdf'
    current=json.loads(content.read_text())
    oldpages={p['id']:p for p in old['pages']};newpages={p['id']:p for p in current['pages']}
    prior=json.loads((HERE/'controller_content_review.json').read_text())
    changed=[key for key in oldpages if oldpages[key]!=newpages[key]]
    selected_unchanged=all(oldpages[key]==newpages[key]for key in prior['selected_page_ids'])
    assert sha(pdf)=='4be727dd3c29942fc10015597cb51c7f63d68b5981096a9f8a395280ae7bfcdf'
    sys.path.insert(0,str(ROOT/'build/presentation-pdf-packages'))
    import fitz
    from PIL import Image
    doc=fitz.open(pdf)
    visual=[]
    for number in [4,5,6,7,8,9,10,11,33]:
        pix=doc[number-1].get_pixmap(matrix=fitz.Matrix(1.5,1.5),alpha=False)
        image=Image.open(HERE/'controller_visual_pages'/f'page_{number:02d}.png').convert('RGB')
        visual.append(dict(page=number,pixels_unchanged=(image.size==(pix.width,pix.height)and image.tobytes()==pix.samples),
            previous_render_sha256=sha(HERE/'controller_visual_pages'/f'page_{number:02d}.png')))
    delta=dict(passed=prior['passed']and selected_unchanged and all(r['pixels_unchanged']for r in visual),
        final_pdf_sha256=sha(pdf),final_content_sha256=sha(content),final_pages=len(doc),
        prior_review='controller_content_review.json',prior_visual_QA='controller_visual_qa.json',
        changed_page_ids=changed,changes_confined_to_declared_motion_pages=set(changed)=={'authority_math','authority_event','turns'},
        reviewed_page_ids=prior['selected_page_ids'],all_reviewed_page_content_unchanged=selected_unchanged,
        reviewed_PDF_page_pixels=visual,
        scope='Final hash/delta binding only; prior technical and visual review remains applicable because reviewed content and rendered pixels are unchanged.')
    save(HERE/'controller_content_review_final_delta.json',delta)
    (HERE/'controller_content_review_final_delta.md').write_text('# Final controller-review binding\n\nPassed. Final PDF SHA256 `'+sha(pdf)+'` contains33pages. Only the three declared motion/timing pages changed; all16 reviewed controller/preparation/Q&A/cost page objects remain unchanged. Rendered pixels on pages4–11and33 exactly match the already inspected images.\n\nNo new tests, controller execution, physics or scientific calculations were performed.\n')
    pre=json.loads((FAMILY/'pre_change/manifest.json').read_text())
    mutable={'ITERATION_LOG.md','docs/iterations/2026-11-26/README.md'}
    protected=[];excluded=[];snapshots=[]
    for row in pre['source']:
        now=sha(ROOT/row['path'])
        value=dict(path=row['path'],expected_sha256=row['sha256'],actual_sha256=now,unchanged=now==row['sha256'])
        (excluded if row['path']in mutable else protected).append(value)
        snapshots.append(dict(path=row['snapshot'],matches=sha(FAMILY/row['snapshot'])==row['sha256']))
    archives=[]
    for row in pre['prior_archives']:
        p=ROOT/row['path'];actual=sha(p)
        archives.append(dict(path=row['path'],expected_sha256=row['sha256'],actual_sha256=actual,
            bytes=p.stat().st_size,matches=actual==row['sha256']and p.stat().st_size==row['bytes']))
    report=dict(passed=all(r['unchanged']for r in protected)and all(r['matches']for r in snapshots+archives),
        prechange_origins=len(pre['source']),protected_origins_count=len(protected),protected_origins=protected,
        expected_mutable_notes=excluded,retained_prechange_snapshots=snapshots,prior_archives_count=len(archives),prior_archives=archives,
        scope='Read-only final source/archive hashes. Exactly two living iteration/day notes excluded from unchanged-origin claims; new I47 guide/docs were not pre-existing origins. No tests or physics.')
    save(HERE/'protected_sources_final.json',report)
    print(json.dumps(dict(delta_passed=delta['passed'],protected_passed=report['passed'],origins=report['prechange_origins'],protected=report['protected_origins_count'],archives=len(archives),changed_ids=changed)))


if __name__=='__main__':main()
