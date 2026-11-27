"""I-66 report-only publication figures; no new scoring, fitting or physics.

Run only after the canonical stage has been sealed. Refuses an existing output
directory; preserve each render and use a new stage for any layout revision.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.patches import FancyBboxPatch
from matplotlib.ticker import MaxNLocator
import numpy as np

ROUTES = (
    ('C_candidate', 'C', 'Physical candidate', '#61758a'),
    ('T_sampled_point', 'T', 'Sampled centroid', '#16828c'),
    ('W_single_family_wrench', 'W', 'Full inferred wrench', '#7655a8'),
    ('S_normal_share_proxy', 'S', 'Normal-share proxy', '#c38c28'),
)
CASES = ('m100_B', 'p030_B', 'p090_B')
CASE_LABEL = {'m100_B': '−1.0 rad', 'p030_B': '+0.3 rad', 'p090_B': '+0.9 rad'}
FAMILIES = ('shell A', 'shell B', 'belt A', 'belt B')
INK = '#142b40'; MUTED = '#5c6d7d'; BG = '#f4f7fa'; GRID = '#dce4ed'
STYLE = {'font.family': 'DejaVu Sans', 'font.size': 11, 'text.color': INK,
         'axes.labelcolor': INK, 'axes.titlecolor': INK, 'xtick.color': MUTED,
         'ytick.color': INK, 'pdf.fonttype': 42, 'ps.fonttype': 42,
         'savefig.facecolor': BG, 'axes.spines.top': False, 'axes.spines.right': False}


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() for k in z.files}


def fmt(value, digits=3):
    return 'unknown' if value is None or not np.isfinite(value) else f'{value:.{digits}f}'


def box(fig, rect, face='white', edge='#e0e7ee', radius=.012):
    x, y, w, h = rect
    patch = FancyBboxPatch((x, y), w, h,
        boxstyle=f'round,pad=0.007,rounding_size={radius}',
        transform=fig.transFigure, facecolor=face, edgecolor=edge, linewidth=.8, zorder=-5)
    fig.add_artist(patch)


def family(code):
    return FAMILIES[int(code)] if 0 <= int(code) < len(FAMILIES) else 'unknown'


def heading(fig, title, subtitle, page=None):
    fig.text(.045, .947, title, fontsize=23, weight='bold', va='top')
    fig.text(.046, .903, subtitle, fontsize=11, color=MUTED, va='top')
    if page is not None:
        fig.text(.955, .948, f'{page} / 8', ha='right', va='top', color=MUTED, fontsize=11)


def footer(fig, text, page=None):
    fig.text(.045, .032, text, fontsize=9, color=MUTED, va='bottom')
    if page is not None:
        fig.text(.955, .032, f'I-66 · {page}/8', fontsize=9, color=MUTED, ha='right', va='bottom')


def metric_card(fig, rect, label, value, detail, color=INK):
    box(fig, rect)
    x, y, w, h = rect
    fig.text(x+.014, y+h-.022, label, fontsize=10, color=MUTED, va='top')
    fig.text(x+.014, y+h-.053, value, fontsize=22, weight='bold', color=color, va='top')
    fig.text(x+.014, y+.012, detail, fontsize=9.2, color=MUTED, va='bottom')


def counter_card(fig, rect, label, row, preference):
    box(fig, rect, face='#f9fbfd')
    x, y, w, h = rect
    fig.text(x+.014, y+h-.019, label, fontsize=11, weight='bold', va='top')
    fig.text(x+.014, y+h-.046, preference, fontsize=9.5, color=MUTED, va='top')
    fig.text(x+.014, y+h-.065,
             f"{row['original_correct']} → {row['replacement_correct']} / {row['denominator']}",
             fontsize=23, weight='bold', va='top')
    fig.text(x+.014, y+.023,
             f"+{row['newly_correct']} newly correct    −{row['newly_wrong']} newly wrong",
             fontsize=9.5, color=MUTED, va='bottom')
    fig.text(x+.014, y+.007,
             f"Ambiguous {row['ambiguous']} · tied {row['tied']} · unknown {row['unknown']}",
             fontsize=8.8, color=MUTED, va='bottom')


def overview(report):
    focus = report['cohorts']['focus']; n = focus['denominator']
    C, T = focus['paired_C'], focus['paired_T']
    fig = plt.figure(figsize=(14.5, 10), facecolor=BG)
    heading(fig, 'Does sampled geometry explain the load mismatch?',
        f'I-66 · {n} useful corrections lost by I-65 · fixed native Gazebo/DART logs · no new physics')
    box(fig, (.045, .836, .91, .044), face='#fff7e8', edge='#ead8b7')
    fig.text(.060, .859,
        'SAMPLED REFERENCE ≠ EXACT 5 ms CONTACT TRUTH   ·   Live LQR / observer / feedforward unchanged',
        fontsize=10.5, weight='bold', va='center', color='#775629')
    old, new = C['vector_RMS_Nm'], T['vector_RMS_Nm']
    fraction = None if old in (None, 0) or new is None else (old-new)/old
    change = 'No finite paired comparison' if fraction is None else f"{abs(fraction)*100:.1f}% {'lower' if fraction >= 0 else 'higher'} RMS · {focus['paired_count']}/{n} paired"
    metric_card(fig, (.045, .701, .292, .112), 'PRIMARY: C → T JOINT VECTOR RMS',
                f'{fmt(old)} → {fmt(new)} N m', change, '#147b85')
    tr = focus['routes']['T_sampled_point']
    failed = tr['count']-tr['compatible_count']
    metric_card(fig, (.354, .701, .292, .112), 'T: WITHIN THE UNCHANGED COMPONENT BOUND',
                f"{tr['compatible_count']} / {n}",
                f"{failed} still incompatible · {tr['unknown']} unknown")
    point = focus['geometry']['point_distance_mm']
    metric_card(fig, (.663, .701, .292, .112), 'CANDIDATE → SAMPLED CENTROID DISTANCE',
                f"{fmt(point['median'], 2)} mm median",
                f"p95 {fmt(point['p95'], 2)} mm · raw centroid, no surface projection")

    box(fig, (.045, .339, .584, .334))
    fig.text(.062, .651, 'Four load representations, same focus cohort', fontsize=14, weight='bold')
    ax = fig.add_axes([.225, .425, .352, .188], facecolor='white')
    positions = np.arange(4)[::-1]
    values = []
    for k, code, name, color in ROUTES:
        v = focus['routes'][k]['vector_RMS_Nm']; values.append(0. if v is None else v)
    upper = max(values+[.01])*1.30
    ax.barh(positions, values, color=[r[3] for r in ROUTES], height=.54, zorder=3)
    ax.set_xlim(0, upper); ax.set_yticks(positions)
    ax.set_yticklabels([f'{code}  {name}' for _, code, name, _ in ROUTES], fontsize=10)
    ax.tick_params(axis='y', length=0, pad=10)
    ax.xaxis.set_major_locator(MaxNLocator(4)); ax.grid(axis='x', color=GRID, zorder=0)
    ax.set_xlabel('Raw three-joint residual vector RMS (N m)', fontsize=10, labelpad=7)
    ax.spines['left'].set_visible(False); ax.spines['bottom'].set_color(GRID)
    for y, v, (k, _, _, color) in zip(positions, values, ROUTES):
        row = focus['routes'][k]
        ax.text(v+upper*.022, y, fmt(row['vector_RMS_Nm']), va='center', fontsize=11, weight='bold', color=color)
    fig.text(.062, .369,
        'Compatible / full cohort:  '+ '    '.join(f"{code} {focus['routes'][k]['compatible_count']}/{n}" for k, code, _, _ in ROUTES),
        fontsize=9.3, color=MUTED)
    fig.text(.062, .350,
        'Finite rows:  '+ '    '.join(f"{code} {focus['routes'][k]['count']}/{n}" for k, code, _, _ in ROUTES)
        +'   ·   unknowns remain in denominators', fontsize=8.8, color=MUTED)

    fig.text(.660, .651, 'Truth-assisted point counterfactual', fontsize=14, weight='bold')
    counter_card(fig, (.663, .493, .292, .134), 'Useful-correction controls',
                 report['counterfactual']['useful'], 'Correct preference: physical F')
    counter_card(fig, (.663, .339, .292, .134), 'Harmful-override controls',
                 report['counterfactual']['harmful'], 'Correct preference: native-family alternative')
    fig.text(.665, .312, 'Only the known-correct point is replaced; the other branch\nand frozen I-65 calibration stay fixed. No deployable score.',
             fontsize=9.2, color=MUTED, va='top', linespacing=1.4)

    rep = focus['point_replacement']; box(fig, (.045, .205, .584, .105))
    fig.text(.062, .290, 'What remains after point replacement?', fontsize=12, weight='bold', va='top')
    columns = [(.065, 'C fail → T pass', rep['C_fail_T_pass']),
               (.245, 'C fail → T fail', rep['C_fail_T_fail']),
               (.425, 'C pass → T fail', rep['C_pass_T_fail'])]
    for x, label, count in columns:
        fig.text(x, .251, str(count), fontsize=22, weight='bold', va='top')
        fig.text(x+.040, .248, label, fontsize=9.6, color=MUTED, va='top')
    criterion = report['geometry_question']['qualified_geometric_improvement']
    failed_phases = [CASE_LABEL[c] for c in CASES if not report['geometry_question']['gates'].get(c+'_RMS_nonincrease', True)]
    reason = ' · '+', '.join(failed_phases)+' focus RMS increased' if failed_phases else ''
    fig.text(.045, .175, 'Registered geometry-reference criterion: '+('met' if criterion else 'not met')+reason,
             fontsize=11, weight='bold', color='#147b85' if criterion else '#946324')
    fig.text(.045, .143,
        'T  Raw normal-weighted endpoint centroid; not a guaranteed geometry ceiling.\n'
        'W  Full inferred force + centre moment placed on one family; a free couple is allowed.\n'
        'S  Normal-load shares with a common total-force direction; not measured multi-contact force routing.',
        fontsize=9.3, color=MUTED, va='top', linespacing=1.6)
    footer(fig, 'Raw observer load: preceding 5 ms.  Pose / sampled contacts: endpoint / last reported 1 ms.  Retrospective diagnostic; no causal or tracking claim.')
    return fig


def explanation_page(report):
    fig = plt.figure(figsize=(14.5, 10), facecolor=BG)
    heading(fig, 'How to read the fixed event windows',
            'I-66 · 14 unchanged I-65 anchors · seven paired-event pages after this guide', 1)
    box(fig, (.045, .756, .91, .109), face='#fff7e8', edge='#ead8b7')
    fig.text(.062, .838, 'These plots localize a model limitation; they do not recover true contact history.',
             fontsize=15, weight='bold', color='#775629')
    fig.text(.062, .804,
        'Every curve uses the same saved pose, observed raw interval force and raw three-joint target.\n'
        'Sampled contact means belong to the last reported 1 ms solver step, not an exact 5 ms interval average.',
        fontsize=12, color='#775629', va='top', linespacing=1.6)
    descriptions = [
        ('C', 'Physical correct-family candidate',
         'Original F if its family matches the sampled truth; otherwise the physical native-family alternative.\n'
         'This selection is evaluation-only. Missing correct candidates remain explicit.'),
        ('T', 'Replace only the point',
         'Use the raw dominant-family normal-weighted centroid, retaining the same family and observed force.\n'
         'The centroid can be interior to the sphere; it is not projected to a material surface.'),
        ('W', 'Place the full inferred wrench on one family',
         'Use the same force and the observer\'s raw centre moment directly. This is point-independent\n'
         'and permits a free couple; it is not an independently measured contact wrench.'),
        ('S', 'Split the total force by normal-load shares',
         'Use every family\'s own sampled centroid with a common force direction and instantaneous shares.\n'
         'This is an assumed routing proxy, not saved per-family vector-force truth.'),
    ]
    for j, ((_, code, _, color), (_, title, text)) in enumerate(zip(ROUTES, descriptions)):
        y = .590-j*.125
        box(fig, (.045, y, .91, .103))
        fig.text(.062, y+.074, code, color=color, fontsize=22, weight='bold', va='top')
        fig.text(.102, y+.077, title, fontsize=13, weight='bold', va='top')
        fig.text(.102, y+.048, text, fontsize=10.5, color=MUTED, va='top', linespacing=1.5)
    box(fig, (.045, .104, .91, .083), face='#edf3f8')
    fig.text(.062, .164, 'Left: raw joint-residual norm (N m).  Right: candidate ↔ sampled-centroid distance (mm).',
             fontsize=12, weight='bold', va='top')
    fig.text(.062, .135,
        'The dashed distance is the component perpendicular to force. Grey gaps mean unknown data.\n'
        'Time zero is the fixed anchor. Later plotted observations are explanatory context, never inference inputs.',
        fontsize=10.4, color=MUTED, va='top', linespacing=1.5)
    footer(fig, 'Live baseline unchanged · no fitting, new scores, physics, camera inputs or controller changes', 1)
    return fig


def reason_text(reasons):
    names = {'every_harmful': 'harmful override', 'first_useful': 'first useful',
             'known_useful_7.875': 'known useful example', 'worst_A_point': 'worst policy-A point',
             'first_lost_useful_primary': 'first covariance-lost useful'}
    return ' · '.join(names.get(x, x.replace('_', ' ')) for x in reasons)


def event_block(fig, event, arrays, upper):
    case = event['case']; t0 = event['time_s']; t = arrays['time_s']
    selected = abs(t-t0) <= .100+1e-9; x = (t[selected]-t0)*1000.
    y0 = .505 if upper else .110
    box(fig, (.045, y0, .91, .335))
    fig.text(.062, y0+.315,
        f"{CASE_LABEL[case]}   ·   t = {t0:.3f} s   ·   {reason_text(event['reasons'])}",
        fontsize=12, weight='bold', va='top')
    correct = {0: 'physical F', 1: 'physical native alternative'}.get(event['correct_branch'], 'unavailable')
    fig.text(.062, y0+.289,
        f"Sampled family: {family(event['truth_family'])}   |   correct-family candidate: {correct}",
        fontsize=9.6, color=MUTED, va='top')
    ax = fig.add_axes([.092, y0+.070, .542, .192], facecolor='white')
    for key, code, _, color in ROUTES:
        values = np.linalg.norm(arrays[key+'_residual_Nm'][selected], axis=1)
        values[~arrays[key+'_valid'][selected]] = np.nan
        ax.plot(x, values, color=color, lw=1.6 if code == 'S' else 1.45,
                ls=':' if code == 'S' else '-', label=code)
    ax.axvline(0, color=INK, ls='--', lw=.9, alpha=.7)
    ax.set_xlim(-100, 100); ax.set_ylim(bottom=0); ax.xaxis.set_major_locator(MaxNLocator(5))
    ax.yaxis.set_major_locator(MaxNLocator(4)); ax.grid(color=GRID, lw=.65)
    ax.set_ylabel('Joint residual norm (N m)', fontsize=10)
    ax.set_xlabel('Time from fixed event (ms)', fontsize=9)
    ax.tick_params(labelsize=9)
    ax2 = fig.add_axes([.721, y0+.070, .207, .192], facecolor='white')
    ax2.plot(x, arrays['point_distance_m'][selected]*1000., color='#16828c', lw=1.7, label='distance')
    ax2.plot(x, arrays['point_perpendicular_m'][selected]*1000., color='#61758a', ls='--', lw=1.2, label='perpendicular to F')
    ax2.axvline(0, color=INK, ls='--', lw=.9, alpha=.7)
    ax2.set_xlim(-100, 100); ax2.set_ylim(bottom=0); ax2.xaxis.set_major_locator(MaxNLocator(5))
    ax2.yaxis.set_major_locator(MaxNLocator(4)); ax2.grid(color=GRID, lw=.65)
    ax2.set_ylabel('Point distance (mm)', fontsize=10)
    ax2.set_xlabel('Time from event (ms)', fontsize=9); ax2.tick_params(labelsize=9)
    ax2.legend(loc='best', fontsize=7.3, frameon=False)
    anchor = int(event['index'])
    values = []
    for key, code, _, _ in ROUTES:
        value = np.linalg.norm(arrays[key+'_residual_Nm'][anchor]) if arrays[key+'_valid'][anchor] else None
        values.append(f'{code} {fmt(value)}')
    fig.text(.062, y0+.015, 'At anchor, residual norm (N m):  '+'    '.join(values), fontsize=9, color=MUTED)
    return dict(case=case, index=anchor, time_s=float(t0), reasons=event['reasons'],
                plotted_rows=int(selected.sum()), window_ms=[-100, 100], correct_branch=int(event['correct_branch']))


def event_page(events, frames, page):
    fig = plt.figure(figsize=(14.5, 10), facecolor=BG)
    heading(fig, 'Fixed event windows: geometry and load representation',
            'Raw residuals share the same observer interval target; the sampled point is an evaluation reference.', page)
    handles = [Line2D([0], [0], color=color, lw=2, label=f'{code}  {name}') for _, code, name, color in ROUTES]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(.055, .877), ncol=4,
               frameon=False, fontsize=10, handlelength=2.0, columnspacing=2.1)
    receipt = [event_block(fig, event, frames[event['case']], j == 0) for j, event in enumerate(events)]
    footer(fig, 'C = available correct-family physical branch; T / W / S use sampled-family information. All are diagnostic only, not a deployed controller.', page)
    return fig, receipt


def pdf_page_count(path, writer_count):
    executable = shutil.which('pdfinfo')
    if executable:
        result = subprocess.run([executable, str(path)], capture_output=True, text=True, check=True)
        for line in result.stdout.splitlines():
            if line.startswith('Pages:'):
                count = int(line.split(':', 1)[1].strip())
                if count != writer_count: raise ValueError('PDF reader/writer page-count mismatch')
                return count, 'pdfinfo + PdfPages.get_pagecount'
    return writer_count, 'PdfPages.get_pagecount'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); root = args.root.resolve(); output = args.output.resolve()
    if output.exists(): raise FileExistsError('Render is immutable; choose a new output stage: '+str(output))
    science = root/'analysis/stage01/science.json'; protocol = root/'protocol/protocol.json'
    array_bindings = root/'analysis/stage01/array_bindings.json'
    if not science.is_file() or not array_bindings.is_file(): raise ValueError('Canonical stage01 report and array bindings must exist')
    report = json.loads(science.read_text()); spec = json.loads(protocol.read_text())
    if report['iteration'] != 'I-66' or spec['iteration'] != 'I-66': raise ValueError('Wrong iteration')
    events = report['events']
    if len(events) != 14 or len({(x['case'], x['index']) for x in events}) != 14:
        raise ValueError('Expected the unchanged fourteen unique fixed I-65 anchors')
    frames = {case: load(root/'analysis/stage01'/(case+'_frames.npz')) for case in CASES}
    bindings = json.loads(array_bindings.read_text())
    for case, arrays in frames.items():
        expected = bindings[case+'_frames.npz']
        if set(arrays) != set(expected): raise ValueError('Canonical NPZ schema differs')
        for key, a in arrays.items():
            b = expected[key]
            if a.dtype.str != b['dtype'] or list(a.shape) != b['shape'] or hashlib.sha256(a.tobytes()).hexdigest() != b['sha256']:
                raise ValueError('Canonical array binding differs: '+case+'/'+key)
    output.mkdir(parents=True)
    source = Path(__file__).resolve(); shutil.copy2(source, output/'plot_results.py')
    inputs = [science, protocol, array_bindings]+[root/'analysis/stage01'/(c+'_frames.npz') for c in CASES]
    input_hashes = {str(p): sha(p) for p in inputs}
    started = datetime.now(timezone.utc).isoformat(); receipts = []
    (output/'registered_before_render.json').write_text(json.dumps(dict(
        created_utc=started, source_sha256=sha(source), input_hashes=input_hashes,
        status='Source snapshot and canonical inputs bound before figure rendering; scientific results already known',
        no_new_scoring_or_fitting=True), indent=2)+'\n')
    with plt.rc_context(STYLE):
        fig = overview(report)
        fig.savefig(output/'overview.png', dpi=180)
        with PdfPages(output/'geometry_load_comparison.pdf', metadata={'Title': 'I-66 geometry and load-model audit'}) as pdf:
            pdf.savefig(fig); overview_pages = pdf.get_pagecount()
        plt.close(fig)
        with PdfPages(output/'event_windows.pdf', metadata={'Title': 'I-66 fourteen fixed event windows'}) as pdf:
            fig = explanation_page(report); pdf.savefig(fig); plt.close(fig)
            for j in range(7):
                fig, page_receipt = event_page(events[2*j:2*j+2], frames, j+2)
                pdf.savefig(fig); plt.close(fig); receipts.append(dict(page=j+2, events=page_receipt))
            event_pages = pdf.get_pagecount()
    op, omethod = pdf_page_count(output/'geometry_load_comparison.pdf', overview_pages)
    ep, emethod = pdf_page_count(output/'event_windows.pdf', event_pages)
    if op != 1 or ep != 8: raise ValueError('Expected one overview and eight event pages')
    page_receipt = dict(overview_pages=op, overview_count_method=omethod,
        appendix_pages=ep, appendix_count_method=emethod, explanation_pages=1,
        anchors=14, anchors_per_data_page=2, pages=receipts)
    (output/'page_receipt.json').write_text(json.dumps(page_receipt, indent=2)+'\n')
    metadata = dict(created_utc=started, iteration='I-66', plot_source_sha256=sha(source),
        input_hashes=input_hashes, canonical_array_bindings_verified=True,
        scope='Plots of sealed canonical summaries and saved raw residual/geometry traces; no new scores, fit or physics',
        fixed_event_source='Canonical I66 copy of the fourteen fixed I65 events, original order preserved',
        retrospective=True, live_changes=False, output_immutable=True,
        geometry_qualification='Sampled normal-weighted endpoint centroids are not exact5ms contact truth or a guaranteed ceiling',
        matplotlib_version=matplotlib.__version__, numpy_version=np.__version__)
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    for p, value in input_hashes.items():
        if sha(p) != value: raise ValueError('Canonical input changed during plotting')
    manifest = dict(created_utc=datetime.now(timezone.utc).isoformat(), files=[
        dict(path=p.name, bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(output.iterdir()) if p.is_file()])
    (output/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), overview_pages=op, appendix_pages=ep, fixed_events=14)))


if __name__ == '__main__': main()
