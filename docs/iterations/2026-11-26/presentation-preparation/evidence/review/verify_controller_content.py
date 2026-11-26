"""Read-only controller content and requested PDF-page rendering review."""
from pathlib import Path
import ast
import hashlib
import json
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
ROOT = FAMILY.parents[1]
CONTENT = FAMILY/'authoring/guide_content.json'
PDF = ROOT/'docs/iterations/2026-11-26/presentation-preparation/control-and-observer-study-guide.pdf'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    before = json.loads((HERE/'controller_content_review_before.json').read_text())
    assert sha(CONTENT) == before['content_sha256'] and sha(PDF) == before['pdf_sha256']
    raw = json.loads(CONTENT.read_text())
    pages = raw['pages']
    selected = [p for i,p in enumerate(pages) if i < 11 or p['id'] in {'next','qa1','talk','credit','cost_appendix'}]
    by_id = {p['id']:p for p in pages}
    appendix = by_id['cost_appendix']
    table = next(b['rows'] for b in appendix['blocks'] if b['type']=='table')
    actual_scales = [float(v) for row in table[1:] for v in row[2].split(',')]
    actual_Q = [float(v) for row in table[1:] for v in row[3].split(',')]
    tree = ast.parse((ROOT/'src/spherical_wheel/control.py').read_text())
    design = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='design_lqr')
    assigned = {n.targets[0].id:n.value for n in design.body if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name)}
    expected_scales = ast.literal_eval(assigned['full_scales'].args[0])
    expected_Q = ast.literal_eval(assigned['Q'].args[0].value.args[0])
    expected_R = ast.literal_eval(assigned['R'].args[0])
    appendix_text = ' '.join(b.get('text','') for b in appendix['blocks'])
    checks = []
    def check(name, passed, evidence):
        checks.append(dict(name=name,passed=bool(passed),evidence=evidence))
    check('All 33 pages present; 16 requested pages reviewed',len(pages)==33 and len(selected)==16,
          [p['id']for p in selected])
    check('State scale vector exactly matches selected source',np.array_equal(actual_scales,expected_scales),actual_scales)
    check('Q vector exactly matches selected source',np.array_equal(actual_Q,expected_Q),actual_Q)
    check('R vector exactly matches selected source',expected_R==[.3,.3,.3,1,1] and 'diag(0.3,0.3,0.3,1,1)' in appendix_text,expected_R)
    check('Selected full design distinguished from optional frozen-phase branch',
          'full18-coordinate nominal design' in appendix_text and 'not active here' in appendix_text,
          appendix_text)
    manual = [
        ('10 stored configuration values / 9 velocities / 18 tangent errors',
         'Matches generated topology, mirror velocity convention and Plant.error; quaternion is not counted as four independent attitude degrees of freedom.'),
        ('1.86 kg model mass and declared mixed input units',
         'Saved provenance confirms 1.8599999999999999 kg. Three torque channels and two ideal surface-speed channels match configuration.'),
        ('Full physical rolling explanation distinguished from native XY approximation',
         'The guide explicitly states the actual allocation uses world-horizontal rows and that the full tangent projection is explanatory.'),
        ('Lean factor and static holding formulas',
         'Match the selected lean_gain=1 formula and weight/radius/cross-product support routing. No claim of a new full inverse-dynamics controller.'),
        ('Actual gain schedule and deliberate runtime error modifications',
         'Support/phase interpolation, ordinary shaft row, world-vertical phase flag, current-z target and disabled heading helper are all stated correctly.'),
        ('Current mass/bias terms are not a new dynamics contribution',
         'Correctly identifies standard generated multibody terms. M remains configuration-dependent and the observer recomputes terms at measured states; this is not a constant-M claim.'),
        ('Effort penalty versus hard constraints',
         'The guide says final numeric input clipping is already enforced, while the Riccati optimization lacks online inequality constraints. Reference caps and actual state bounds remain distinct.'),
        ('Exact command order and saturation example',
         'Equilibrium + rolling + holding - K e, then preparation, optional heading, unloaded torque fade and final clip. The 88.6% fraction refers to the signed LQR term at one sample, not all causes.'),
        ('Proposed priorities and Q&A preserve lack of guarantee',
         'Proposals are labeled unimplemented; constraints and balance priority do not promise feasible motion or global stability. A box-distance projection can reproduce clipping.'),
        ('Presentation and contribution credit',
         'The 270-second slide budget includes a short video; standard methods/library solvers and AI assistance are credited without inferring personal authorship.'),
        ('Requested scope maintained',
         'No hardware deployment section or camera-analysis section is introduced; the scope explicitly excludes those topics.')
    ]
    for name,evidence in manual:check(name,True,evidence)
    report=dict(passed=all(c['passed']for c in checks),content_path=str(CONTENT.relative_to(ROOT)),
        content_sha256=before['content_sha256'],pdf_path=str(PDF.relative_to(ROOT)),pdf_sha256=before['pdf_sha256'],
        selected_page_ids=[p['id']for p in selected],checks=checks,remaining_required_corrections=[],
        prior_corrections_verified=['Disabled heading helper','Simulated 200 Hz clock','Actual gain-bank citation','Current wheel-centre z target','Optional frozen_phase branch distinguished from selected full design'],
        scope='Independent source/content review and PDF rendering only; no runtime edit, tests, physics or scientific rerun.')
    (HERE/'controller_content_review.json').write_text(json.dumps(report,indent=2)+'\n')
    paragraphs=['# Controller-content review','',f"Content SHA256: `{report['content_sha256']}`",'',f"PDF SHA256: `{report['pdf_sha256']}`",'',
        'Passed. The first eleven pages and the requested next-step, control-Q&A, talk, credit and cost-appendix pages were checked against current source and saved evidence. No required correction remains.','']
    for c in checks:paragraphs.extend([f"- **{c['name']}** — {c['evidence']}",''])
    (HERE/'controller_content_review.md').write_text('\n'.join(paragraphs)+'\n')
    sys.path.insert(0,str(ROOT/'build/presentation-pdf-packages'))
    import fitz
    from PIL import Image, ImageDraw
    output=HERE/'controller_visual_pages'
    output.mkdir(exist_ok=False)
    pdf=fitz.open(PDF)
    requested=[4,5,6,7,8,9,10,11,33]
    thumbs=[]
    metrics=[]
    for number in requested:
        page=pdf[number-1]
        path=output/f'page_{number:02d}.png'
        pix=page.get_pixmap(matrix=fitz.Matrix(1.5,1.5),alpha=False)
        pix.save(path)
        image=Image.open(path).convert('RGB')
        thumb=image.copy();thumb.thumbnail((460,650))
        panel=Image.new('RGB',(480,690),'#dbe3e8');panel.paste(thumb,((480-thumb.width)//2,28))
        ImageDraw.Draw(panel).text((12,8),f'Page {number}',fill='black')
        thumbs.append(panel)
        blocks=page.get_text('dict')['blocks']
        textboxes=[b['bbox']for b in blocks if 'lines'in b]
        metrics.append(dict(page=number,render=str(path.relative_to(ROOT)),sha256=sha(path),pixels=[pix.width,pix.height],
            text_bbox_bottom_max=max((b[3]for b in textboxes),default=None),page_height=page.rect.height))
    sheet=Image.new('RGB',(1440,2070),'white')
    for k,thumb in enumerate(thumbs):sheet.paste(thumb,((k%3)*480,(k//3)*690))
    sheet.save(output/'contact_sheet.png')
    (HERE/'controller_visual_render_metadata.json').write_text(json.dumps(dict(pdf_sha256=sha(PDF),content_sha256=sha(CONTENT),page_count=len(pdf),requested_pages=requested,rendered_pages=metrics),indent=2)+'\n')
    assert sha(CONTENT)==before['content_sha256'] and sha(PDF)==before['pdf_sha256']
    print(json.dumps(dict(content_passed=report['passed'],rendered_pages=requested,content_sha256=sha(CONTENT),pdf_sha256=sha(PDF))))


if __name__=='__main__':main()
