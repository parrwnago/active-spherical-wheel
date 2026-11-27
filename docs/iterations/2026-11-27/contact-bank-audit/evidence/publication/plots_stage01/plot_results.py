"""I-67 report-only figures, using a sealed canonical audit and cached points.

No new scores, fits, candidates, thresholds, or physics are produced here.
An existing output directory is refused: preserve each layout stage.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.patches import FancyBboxPatch
from matplotlib.ticker import MaxNLocator
import numpy as np


CASES = ('m100_B', 'p030_B', 'p090_B')
CASE_LABELS = {'m100_B': '−1.0 rad', 'p030_B': '+0.3 rad', 'p090_B': '+0.9 rad'}
FAMILIES = ('shell A', 'shell B', 'belt A', 'belt B')
INK = '#172f43'; MUTED = '#5f7181'; BG = '#f3f6fa'; GRID = '#dce5ed'
BLUE = '#4b82ab'; TEAL = '#16848b'; AMBER = '#c98b27'; PURPLE = '#7c5ba8'
STYLE = {'font.family': 'DejaVu Sans', 'font.size': 11,
         'text.color': INK, 'axes.labelcolor': INK, 'axes.titlecolor': INK,
         'xtick.color': MUTED, 'ytick.color': MUTED,
         'pdf.fonttype': 42, 'ps.fonttype': 42,
         'axes.spines.top': False, 'axes.spines.right': False,
         'savefig.facecolor': BG}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {key: z[key].copy() for key in z.files}


def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def number(value, digits=2):
    if value is None or not np.isfinite(value):
        return '—'
    return f'{value:.{digits}f}'


def family(value):
    value = int(value)
    return FAMILIES[value] if 0 <= value < 4 else 'unknown family'


def box(fig, rect, color='white', edge='#dfe7ef'):
    x, y, width, height = rect
    fig.add_artist(FancyBboxPatch((x, y), width, height,
        transform=fig.transFigure, facecolor=color, edgecolor=edge,
        boxstyle='round,pad=0.006,rounding_size=0.010', linewidth=.8, zorder=-5))


def heading(fig, title, subtitle, page=None):
    fig.text(.045, .952, title, fontsize=23, weight='bold', va='top')
    fig.text(.046, .910, subtitle, fontsize=10.5, color=MUTED, va='top')
    if page is not None:
        fig.text(.955, .951, f'{page}/8', ha='right', va='top', fontsize=11, color=MUTED)


def footer(fig, text, page=None):
    fig.text(.045, .027, text, fontsize=8.5, color=MUTED, va='bottom')
    if page is not None:
        fig.text(.955, .027, f'I-67 · {page}/8', fontsize=9, color=MUTED, ha='right')


def card(fig, rect, label, value, explanation, color=INK):
    box(fig, rect)
    x, y, width, height = rect
    fig.text(x+.015, y+height-.020, label, va='top', fontsize=9.7, color=MUTED)
    fig.text(x+.015, y+height-.050, value, va='top', fontsize=23, color=color, weight='bold')
    fig.text(x+.015, y+.013, explanation, va='bottom', fontsize=9.0, color=MUTED)


def count_label(row, key):
    return f"{row[key]}/{row['denominator']}"


def overview(report):
    primary = report['primary_capacity']; focus = report['cohorts']['focus']
    fig = plt.figure(figsize=(14.5, 10), facecolor=BG)
    heading(fig, 'Does the original bank contain useful alternatives?',
        'I-67 · frozen 102-slot bank · same native Gazebo/DART recordings · no new driving episode')
    box(fig, (.045, .837, .91, .044), '#fff6e7', '#e8d6b3')
    fig.text(.060, .859,
        'TRUTH-ASSISTED CAPACITY, NOT A NEW SELECTOR   ·   Live LQR / momentum observer / feedforward unchanged',
        va='center', fontsize=10.3, color='#805c25', weight='bold')
    card(fig, (.045, .698, .292, .116), 'I-66 REFERENCE REPAIRS SUPPORTED',
        f"{primary['supported']} / {primary['reference_repaired_denominator']}",
        f"{primary['unsupported']} unsupported under the original usable-bank rules", TEAL)
    card(fig, (.354, .698, .292, .116), 'FOCUS: ANY USABLE CANDIDATE CORRECT',
        count_label(focus, 'any_existing_candidate_correct'),
        f"Selected {focus['original_correct']} · sampled reference {focus['sampled_reference_correct']}")
    card(fig, (.663, .698, .292, .116), 'POINT OF MINIMUM LOAD SCORE',
        f"{number(focus['minimum_score_point_distance_mm']['median'])} mm median",
        '3D distance to sampled centroid; lowest score need not mean closest point', PURPLE)

    box(fig, (.045, .344, .447, .325))
    fig.text(.062, .644, 'Correct relative preferences / full cohort', fontsize=13.5, weight='bold')
    ax = fig.add_axes([.132, .423, .329, .183], facecolor='white')
    group_names = ('focus', 'useful', 'harmful')
    y = np.arange(3)[::-1]
    series = (('original_correct', 'Original selected', '#6b7e91'),
              ('sampled_reference_correct', 'I-66 sampled point', TEAL),
              ('any_existing_candidate_correct', 'Any usable bank point', PURPLE))
    for offset, (key, label, color) in zip((.24, 0., -.24), series):
        values = [100 * report['cohorts'][name][key] / report['cohorts'][name]['denominator'] for name in group_names]
        ax.barh(y+offset, values, height=.21, color=color, label=label, zorder=3)
        for yy, value, name in zip(y+offset, values, group_names):
            row = report['cohorts'][name]
            ax.text(min(value+2, 102), yy, f"{row[key]}/{row['denominator']}",
                    va='center', fontsize=8.5, color=INK)
    ax.set_yticks(y); ax.set_yticklabels(('Lost useful (41)', 'All useful (294)', 'Harmful (6)'), fontsize=9)
    ax.set_xlim(0, 121); ax.set_xticks((0, 25, 50, 75, 100)); ax.set_xticklabels(('0', '25', '50', '75', '100%'))
    ax.grid(axis='x', color=GRID, zorder=0); ax.tick_params(axis='y', length=0, pad=8)
    ax.spines['left'].set_visible(False); ax.spines['bottom'].set_color(GRID)
    handles = [Line2D([0], [0], color=color, lw=5, label=label) for _, label, color in series]
    fig.legend(handles=handles, loc='lower left', bbox_to_anchor=(.061, .367), frameon=False,
               fontsize=8.3, ncol=3, handlelength=1.4, columnspacing=1.2)

    box(fig, (.514, .344, .441, .325))
    fig.text(.532, .644, 'Geometry gaps on the same 41 focus rows', fontsize=13.5, weight='bold')
    geometry = focus['geometry']
    distance_keys = ('selected_distance_m', 'nearest_distance_m', 'pre_admission_distance_m')
    values = [geometry[key]['median'] for key in distance_keys]
    p95 = [geometry[key]['p95'] for key in distance_keys]
    ax = fig.add_axes([.655, .441, .252, .156], facecolor='white')
    colors = (AMBER, TEAL, '#9aa6b2')
    largest = max([v for v in values+p95 if v is not None]+[1.])
    for yy, value, high, color in zip((2, 1, 0), values, p95, colors):
        if value is not None:
            ax.barh(yy, value, height=.47, color=color, zorder=3)
            ax.text(value+largest*.03, yy, number(value), va='center', fontsize=9.5, weight='bold')
        if high is not None:
            ax.plot(high, yy, marker='|', color=INK, markersize=12, markeredgewidth=1.5)
    ax.set_yticks((2, 1, 0)); ax.set_yticklabels(('Original selected', 'Nearest usable', 'Nearest pre-admission'), fontsize=9)
    ax.set_xlim(0, largest*1.22); ax.set_ylim(-.65, 2.65)
    ax.xaxis.set_major_locator(MaxNLocator(4)); ax.grid(axis='x', color=GRID, zorder=0)
    ax.tick_params(axis='y', length=0, pad=8); ax.set_xlabel('3D distance (mm) · bar median · tick p95', fontsize=8.5)
    ax.spines['left'].set_visible(False); ax.spines['bottom'].set_color(GRID)
    fig.text(.532, .397,
        f"Median selection gap {number(geometry['geometric_selection_gap_m']['median'])} mm"
        f"   ·   admission gap {number(geometry['geometric_admission_gap_m']['median'])} mm",
        fontsize=9.2, color=MUTED)
    fig.text(.532, .371,
        f"Centroid radius deviation: median {number(geometry['reference_radial_error_m']['median'])} mm",
        fontsize=9.2, color=MUTED)
    fig.text(.532, .352, 'Excluded projections are not usable hypotheses; centroids may be inside the sphere.',
        fontsize=8.4, color=MUTED)

    box(fig, (.045, .141, .447, .175), '#f9fbfd')
    fig.text(.061, .294, 'Two different minimizations', fontsize=12.5, weight='bold')
    fig.text(.061, .265,
        'Original F / Nalt minimizes: angle + moment + velocity fused cost.\n'
        'The audit also finds minimum frozen I-65 joint-load score.\n'
        'Changing that objective is not an implementation correction.',
        fontsize=10, linespacing=1.65, va='top')
    fig.text(.061, .159, 'A lower load score may compensate model error rather than improve geometry.',
        fontsize=9.0, color=MUTED)
    box(fig, (.514, .141, .441, .175), '#f9fbfd')
    fig.text(.532, .294, 'Capacity is not an achieved error reduction', fontsize=12.5, weight='bold')
    fig.text(.532, .265,
        'Correct family is known only for this evaluation. Wrong branch stays fixed.\n'
        'Any-point existence preserves the selected-point result by inclusion.\n'
        'A usable point can still be ambiguous or tied under the frozen comparison.',
        fontsize=9.8, linespacing=1.65, va='top')
    fig.text(.532, .159, 'No new weights, candidate generation, controller commands, or physics.',
        fontsize=9.0, color=MUTED)
    footer(fig, 'Sampled centroid: last reported 1 ms · observer interval: preceding 5 ms · not exact interval contact truth')
    return fig


def active_mask(packet):
    """Display-only reconstruction of the sealed original rank-active mask."""
    angle = packet['old__normal_angle_rad']; speed = packet['old__velocity_residual_norm_m_s']
    moment = packet['old__moment_residual_norm']; vector = packet['old__velocity_residual_m_s']
    return (packet['old__valid'] & np.isfinite(angle) & np.isfinite(speed) & np.isfinite(moment)
            & (angle >= 0) & (speed >= 0) & (moment >= 0) & np.isfinite(vector).all(axis=2)
            & packet['old__frame_valid'][:, None] & np.isfinite(packet['old__excitation_m_s'])[:, None])


def event_index(arrays, event):
    found = np.flatnonzero(arrays['source_index'] == event['index'])
    if len(found) != 1:
        raise ValueError('Missing or repeated fixed event anchor')
    return int(found[0])


def cloud(ax, packet, bank, frames, i):
    target = packet['sampled_reference_point_m'][i]
    points = bank['points_m'][i]
    family_mask = (bank['family_index'] == frames['truth_family'][i]) & bank['reconstructed'][i]
    usable = active_mask(packet)[i] & family_mask
    if int(usable.sum()) != int(frames['usable_correct_count'][i]):
        raise ValueError('Display mask differs from canonical usable count')
    excluded = family_mask & ~usable
    delta = (points-target)*1000.
    for mask, color, marker, size in ((excluded, '#abb5bf', 'x', 30), (usable, BLUE, 'o', 30)):
        finite = mask & np.isfinite(delta).all(axis=1)
        ax.scatter(delta[finite, 0], delta[finite, 1], color=color, marker=marker,
                   s=size, alpha=.75, linewidths=.9, zorder=3)
    ax.scatter([0], [0], color=INK, marker='+', s=110, linewidths=1.8, zorder=9)
    markers = (('selected_id', '*', AMBER, 175, 'S'),
               ('nearest_id', 'D', TEAL, 70, 'G'),
               ('minscore_id', 'P', PURPLE, 100, 'L'))
    for key, marker, color, size, text in markers:
        index = int(frames[key][i])
        if index >= 0 and np.isfinite(delta[index]).all():
            ax.scatter(delta[index, 0], delta[index, 1], marker=marker, s=size,
                color=color, edgecolors='white', linewidths=.8, zorder=6)
            offset = {'S': (5, 7), 'G': (5, -12), 'L': (-12, 7)}[text]
            ax.annotate(text, delta[index, :2], xytext=offset, textcoords='offset points',
                        fontsize=9, weight='bold', color=color, zorder=10)
    finite = family_mask & np.isfinite(delta).all(axis=1)
    if finite.any():
        xs = np.r_[delta[finite, 0], 0.]; ys = np.r_[delta[finite, 1], 0.]
        span = max(np.ptp(xs), np.ptp(ys), 4.)
        midx = (xs.min()+xs.max())/2; midy = (ys.min()+ys.max())/2
        half = .63*span
        ax.set_xlim(midx-half, midx+half); ax.set_ylim(midy-half, midy+half)
    ax.axvline(0, color=GRID, lw=.7); ax.axhline(0, color=GRID, lw=.7)
    ax.set_aspect('equal', adjustable='box'); ax.grid(color=GRID, alpha=.7, lw=.6)
    ax.set_xlabel('World X offset (mm)', fontsize=9); ax.set_ylabel('World Y offset (mm)', fontsize=9)
    ax.tick_params(labelsize=8); ax.xaxis.set_major_locator(MaxNLocator(4)); ax.yaxis.set_major_locator(MaxNLocator(4))
    ax.set_title('XY projection · same evaluated family', fontsize=9.5, pad=5)


def event_panel(fig, rect, event, packet, bank, frames):
    x, y, width, height = rect; i = event_index(frames, event)
    box(fig, rect)
    case = event['case']; name = family(frames['truth_family'][i])
    fig.text(x+.014, y+height-.015,
        f"{CASE_LABELS[case]}   ·   t = {event['time_s']:.3f} s   ·   {name}",
        fontsize=12.5, weight='bold', va='top')
    reasons = ' · '.join(event.get('reasons', []))
    fig.text(x+.014, y+height-.045, textwrap.shorten(reasons, width=125, placeholder='…'),
             fontsize=8.6, color=MUTED, va='top')
    ax = fig.add_axes([x+.023, y+.061, .287, height-.145], facecolor='white')
    cloud(ax, packet, bank, frames, i)
    tx = x+.343
    fig.text(tx, y+height-.081,
        f"Correct-family slots: {int(frames['usable_correct_count'][i])} usable"
        f" / {int(frames['pre_admission_correct_count'][i])} pre-admission",
        fontsize=9.5, weight='bold')
    fig.text(tx, y+height-.108, 'Original fused cost components (unitless)', fontsize=9.3, color=MUTED)
    columns = (tx, tx+.094, tx+.160, tx+.226, tx+.296, tx+.379, tx+.474)
    for xx, label in zip(columns, ('Point / ID', 'Angle', 'Moment', 'Velocity', 'Fused Σ', 'Load score', '3D mm')):
        fig.text(xx, y+height-.134, label, fontsize=8.1, color=MUTED, weight='bold')
    rows = (
        ('S', 'selected', 'selected_id', 'selected_load_score', 'selected_distance_m', AMBER),
        ('G', 'nearest_geometry', 'nearest_id', 'nearest_geometry_load_score', 'nearest_distance_m', TEAL),
        ('L', 'minimum_load', 'minscore_id', 'min_load_score', 'minimum_score_point_distance_m', PURPLE),
    )
    for j, (letter, prefix, idkey, loadkey, distancekey, color) in enumerate(rows):
        yy = y+height-.165-j*.029
        index = int(frames[idkey][i]); pieces = frames[prefix+'_cost_components'][i]
        values = [f'{letter} / {index}' if index >= 0 else f'{letter} / —']
        values += [number(v) for v in pieces]
        values += [number(frames[prefix+'_fused_cost'][i]), number(frames[loadkey][i]),
                   number(frames[distancekey][i]*1000)]
        for k, (xx, value) in enumerate(zip(columns, values)):
            fig.text(xx, yy, value, fontsize=9.2, color=color if k == 0 else INK,
                     weight='bold' if k == 0 else 'normal')
    fig.text(tx, y+height-.263,
        f"Nearest pre-admission: {number(frames['pre_admission_distance_m'][i]*1000)} mm"
        f"    ·    radial deviation {number(frames['reference_radial_error_m'][i]*1000)} mm",
        fontsize=9, color=MUTED)
    if frames['opportunity'][i]:
        verdict = lambda key: 'correct' if frames[key][i] else 'not correct'
        detail = (f"Selected: {verdict('baseline_correct')}   ·   sampled reference: {verdict('reference_correct')}\n"
                  f"Any usable: {verdict('any_bank_correct')}   ·   nearest geometry: {verdict('nearest_geometry_correct')}")
    else:
        detail = 'Geometry-only anchor outside the 300 paired opportunities.\nNo invented correct-versus-wrong branch or capacity decision.'
    fig.text(tx, y+.036, detail, fontsize=9.3, linespacing=1.5, color=INK)
    fig.text(x+.014, y+.014, '102 slots are alternative hypotheses, not contacts. Full 3D distances are printed; XY hides Z.',
             fontsize=8.3, color=MUTED)


def guide(report):
    fig = plt.figure(figsize=(14.5, 11), facecolor=BG)
    heading(fig, 'Read the bank audit without turning capacity into control',
        'I-67 event atlas · all 14 fixed I-66 anchors, original order · two anchors per following page', 1)
    box(fig, (.045, .676, .91, .190))
    fig.text(.062, .840, 'One evaluated family; four meanings of “best”', fontsize=16, weight='bold')
    lines = [
        ('S — selected', 'Original fused winner in the correct family; F first, otherwise physical Nalt.', AMBER),
        ('G — geometry', 'Nearest usable original point to the recorded centroid, using full 3D distance.', TEAL),
        ('L — load score', 'Lowest frozen I-65 covariance score among usable correct-family points.', PURPLE),
        ('+ — reference', 'Recorded normal-load-weighted centroid from the last reported 1 ms step.', INK),
    ]
    for j, (label, text, color) in enumerate(lines):
        yy = .799-j*.031
        fig.text(.063, yy, label, fontsize=11.2, color=color, weight='bold')
        fig.text(.224, yy, text, fontsize=10.8)

    box(fig, (.045, .448, .441, .196))
    fig.text(.062, .619, 'Which dots are allowed?', fontsize=14.5, weight='bold')
    handles = [Line2D([0], [0], color=BLUE, marker='o', linestyle='', markersize=7, label='Usable correct-family slot'),
               Line2D([0], [0], color='#abb5bf', marker='x', linestyle='', markersize=7, label='Excluded pre-admission projection')]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(.055, .597), frameon=False, fontsize=10.5)
    fig.text(.062, .514,
        '17 normal seeds × 6 surface/rim slots = 102 original hypotheses.\n'
        'Original finite/cone/compression admission is unchanged.\n'
        'Duplicate projected slots are not distinct physical contacts.',
        va='top', fontsize=10.5, linespacing=1.65)

    box(fig, (.514, .448, .441, .196))
    fig.text(.532, .619, 'Different objective, not a selector bug', fontsize=14.5, weight='bold')
    fig.text(.532, .582,
        'Fused = (angle / 10°)² + moment² + (velocity / .02)².\n'
        'Load score uses frozen held-out I-65 mean and covariance.\n'
        'S must win the original fused objective within its family.\n'
        'G or L can be elsewhere without violating that invariant.',
        va='top', fontsize=10.4, linespacing=1.85)
    box(fig, (.045, .226, .91, .190), '#fff7e8', '#e8d8bb')
    fig.text(.062, .389, 'The inference boundary', fontsize=15, weight='bold', color='#805c25')
    fig.text(.062, .351,
        'The evaluated correct family and reference centroid are simulator truth used only in this offline audit.\n'
        'The wrong-family physical branch, original velocity scores, calibration, and ambiguity/tie rules stay frozen.\n'
        '“Any usable point works” is an existence statement, not a rule that can choose that point online.\n'
        'A better load fit can compensate a wrong interval / single-point model rather than improve contact geometry.',
        fontsize=11, va='top', linespacing=1.8)
    fig.text(.062, .250,
        'Raw observer load covers the previous 5 ms; sampled centroid covers the last 1 ms. No exact interval contact truth.',
        fontsize=10.2, color='#805c25', weight='bold')
    p = report['primary_capacity']
    fig.text(.062, .170,
        f"Primary capacity result: {p['supported']}/{p['reference_repaired_denominator']} I-66 repairs supported;"
        f" {p['unsupported']} unsupported.", fontsize=15, weight='bold')
    fig.text(.062, .129,
        'No fit, expanded bank, changed weights, new controller commands, or physics run. All fixed event anchors are retained.',
        fontsize=10.7, color=MUTED)
    footer(fig, 'Compare full 3D distance and both objective tables; an XY overlap does not imply equal contact points.', 1)
    return fig


def pdf_pages(path):
    executable = shutil.which('pdfinfo')
    if executable is None:
        return None
    result = subprocess.run([executable, str(path)], check=True, capture_output=True, text=True)
    found = re.search(r'^Pages:\s+(\d+)', result.stdout, re.MULTILINE)
    if found is None:
        raise ValueError('pdfinfo did not report a page count')
    return int(found[1])


def run(args):
    if args.output.exists():
        raise FileExistsError('Use a new immutable layout stage: '+str(args.output))
    report = read(args.analysis/'science.json')
    if report['iteration'] != 'I-67' or read(args.protocol)['iteration'] != 'I-67':
        raise ValueError('Expected sealed I67 analysis and protocol')
    if len(report['events']) != 14:
        raise ValueError('Exactly fourteen fixed anchors are required')
    if not (args.analysis/'array_bindings.json').is_file():
        raise ValueError('Canonical stage must be complete before rendering')
    packet_manifest = read(args.packets/'manifest.json')
    manifest_events = [(case, e['index'], e['time_s']) for case in CASES for e in packet_manifest['cases'][case]['events']]
    report_events = [(e['case'], e['index'], e['time_s']) for e in report['events']]
    if report_events != manifest_events:
        raise ValueError('Fixed anchor order changed')
    inputs = [args.analysis/'science.json', args.analysis/'array_bindings.json', args.protocol,
              args.packets/'manifest.json', args.banks/'bank_manifest.json']
    frames = {}; packets = {}; banks = {}
    for case in CASES:
        paths = (args.analysis/(case+'_frames.npz'), args.packets/(case+'.npz'), args.banks/(case+'.npz'))
        inputs.extend(paths)
        frames[case], packets[case], banks[case] = [load(path) for path in paths]
        if not np.array_equal(frames[case]['source_index'], packets[case]['source_index']) or not np.array_equal(frames[case]['source_index'], banks[case]['source_index']):
            raise ValueError('Canonical/packet/bank ROI clocks differ')
    args.output.mkdir(parents=True)
    shutil.copy2(Path(__file__), args.output/'plot_results.py')
    write(args.output/'registered_before_render.json', {
        'utc': datetime.now(timezone.utc).isoformat(), 'iteration': 'I-67',
        'scope': 'Report-only charts and XY coordinate projection. No new scoring, fitting, selection, or physics.',
        'source_sha256': sha(__file__), 'matplotlib_version': matplotlib.__version__,
        'numpy_version': np.__version__,
        'inputs': [{'path': str(p), 'sha256': sha(p)} for p in inputs],
        'fixed_anchors': [{'case': e['case'], 'source_index': e['index'], 'time_s': e['time_s']} for e in report['events']]})
    with plt.rc_context(STYLE):
        fig = overview(report)
        fig.savefig(args.output/'overview.png', dpi=170)
        fig.savefig(args.output/'bank_capacity_comparison.pdf')
        plt.close(fig)
        with PdfPages(args.output/'event_windows.pdf') as pdf:
            fig = guide(report); pdf.savefig(fig); plt.close(fig)
            events = report['events']
            for pair in range(7):
                fig = plt.figure(figsize=(14.5, 11), facecolor=BG)
                heading(fig, 'Frozen candidate geometry and the two objectives',
                    'No new selection policy · S selected · G nearest usable geometry · L minimum load score · + sampled centroid', pair+2)
                for event, rect in zip(events[2*pair:2*pair+2], ((.045, .493, .91, .365), (.045, .087, .91, .365))):
                    case = event['case']
                    event_panel(fig, rect, event, packets[case], banks[case], frames[case])
                footer(fig, 'Dots: rank-usable hypotheses · ×: excluded projections · full 3D distances printed · last 1 ms centroid ≠ exact 5 ms truth', pair+2)
                pdf.savefig(fig)
                if pair in (0, 6):
                    fig.savefig(args.output/f'event_page_{pair+2:02d}.png', dpi=125)
                plt.close(fig)
    counts = {name: pdf_pages(args.output/name) for name in ('bank_capacity_comparison.pdf', 'event_windows.pdf')}
    if counts['bank_capacity_comparison.pdf'] not in (None, 1) or counts['event_windows.pdf'] not in (None, 8):
        raise ValueError('Publication page count mismatch')
    write(args.output/'publication_qa.json', {
        'utc': datetime.now(timezone.utc).isoformat(), 'iteration': 'I-67',
        'primary_capacity': report['primary_capacity'],
        'fixed_anchors': 14, 'fixed_anchor_order_exact': True,
        'expected_pages': {'bank_capacity_comparison.pdf': 1, 'event_windows.pdf': 8},
        'pdfinfo_pages': counts, 'new_scoring': False, 'new_physics': False,
        'interpretation': 'Truth-assisted conditional capacity. Nearest geometry and minimum load objective are separate diagnostics; excluded projections are not usable bank members.'})
    write(args.output/'artifact_manifest.json', {
        'iteration': 'I-67', 'complete_utc': datetime.now(timezone.utc).isoformat(),
        'files': [{'path': str(p.relative_to(args.output)), 'sha256': sha(p), 'bytes': p.stat().st_size}
                  for p in sorted(args.output.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json']})
    print(json.dumps({'output': str(args.output), 'pdf_pages': counts,
                      'fixed_anchors': 14, 'manifest_sha256': sha(args.output/'artifact_manifest.json')}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('analysis', 'packets', 'banks', 'protocol', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    arguments = parser.parse_args()
    for name in ('analysis', 'packets', 'banks', 'protocol', 'output'):
        setattr(arguments, name, getattr(arguments, name).resolve())
    run(arguments)
