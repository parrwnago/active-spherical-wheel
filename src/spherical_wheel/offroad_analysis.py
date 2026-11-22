"""Figures for off-road runs: where the wheel went and what the ground did to it.

Every plotted value is also in the run's ``summary.json`` or ``rollout.npz``;
the figures add nothing that is not in the saved data.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

SURFACE, INK, SECONDARY, MUTED = '#fcfcfb', '#0b0b0b', '#52514e', '#898781'
GRID, AXIS = '#e1e0d9', '#c3c2b7'
BLUE, ORANGE, AQUA, YELLOW = '#2a78d6', '#eb6834', '#1baf7a', '#eda100'
CRITICAL = '#d03b3b'
HEIGHT_RAMP = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#104281']
# Neutral ramp for ground height, so the blue wheel path stays legible on it.
GROUND_RAMP = ['#f4f3f0', '#dcdbd4', '#c3c2b7', '#a3a29a', '#7d7c76']
CONTROLLER_COLORS = {'terrain_aware': BLUE, 'flat_baseline': ORANGE}
CONTROLLER_LABELS = {'terrain_aware': 'Contact-aware', 'flat_baseline': 'Flat-floor baseline'}


def _pyplot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'font.family': ['Segoe UI', 'DejaVu Sans', 'sans-serif'], 'font.size': 9,
        'text.color': INK, 'axes.labelcolor': SECONDARY, 'axes.titlecolor': INK,
        'axes.titlesize': 10, 'axes.titleweight': 'semibold', 'axes.titlelocation': 'left',
        'axes.edgecolor': AXIS, 'axes.linewidth': .8, 'axes.spines.top': False, 'axes.spines.right': False,
        'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.labelcolor': SECONDARY, 'ytick.labelcolor': SECONDARY,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': .8, 'grid.linestyle': '-',
        'lines.linewidth': 2., 'lines.solid_capstyle': 'round', 'lines.solid_joinstyle': 'round',
        'legend.frameon': False, 'legend.fontsize': 8, 'axes.axisbelow': True,
    })
    return plt


def _legend(axis, **options):
    return axis.legend(**{'handlelength': 1.4, 'borderaxespad': .2, 'labelcolor': SECONDARY, **options})


def plot_run(log, summary, terrain, path, output, title=None):
    """One run on one terrain: path, tracking, ground reaction and drive effort."""
    plt = _pyplot()
    from matplotlib.colors import LinearSegmentedColormap
    time = log['time_s']
    weight = None
    figure = plt.figure(figsize=(15., 13.2), constrained_layout=True)
    grid = figure.add_gridspec(4, 3)
    # -- map ---------------------------------------------------------------
    axis = figure.add_subplot(grid[0, :2])
    xy, reference = log['qpos'][:, :2], log['reference_xy']
    points = np.vstack((xy, reference))
    # A strip around the path. The axes are not to equal scale: a straight
    # course is far longer than it is wide.
    pad = .3
    x0, x1 = points[:, 0].min()-pad, points[:, 0].max()+pad
    y0, y1 = points[:, 1].min()-pad, points[:, 1].max()+pad
    xs, ys = np.linspace(x0, x1, 160), np.linspace(y0, y1, 80)
    heights = terrain.grid(xs, ys)*1000.
    if np.ptp(heights) > .5:
        colormap = LinearSegmentedColormap.from_list('height', GROUND_RAMP)
        mesh = axis.pcolormesh(xs, ys, heights, cmap=colormap, shading='auto', rasterized=True)
        bar = figure.colorbar(mesh, ax=axis, pad=.01, fraction=.035)
        bar.set_label('Ground height (mm)', color=SECONDARY)
        bar.outline.set_visible(False)
    nominal = float(np.nanmax(log['ground_friction']))
    friction = np.array([[terrain.friction(float(x), float(y), nominal) for x in xs] for y in ys])
    if np.ptp(friction) > 1e-3:
        axis.contour(xs, ys, friction, levels=[.5*(friction.min()+friction.max())], colors=[INK], linewidths=1.)
        row, col = np.unravel_index(np.argmin(friction), friction.shape)
        inside = np.flatnonzero(friction[row] < .5*(friction.min()+friction.max()))
        axis.annotate(f'low friction, mu = {friction.min():.2f}', (xs[inside].mean(), y1), xytext=(0, -4),
                      textcoords='offset points', ha='center', va='top', fontsize=8, color=INK)
    axis.plot(reference[:, 0], reference[:, 1], color=INK, linewidth=1., label='Reference')
    axis.plot(xy[:, 0], xy[:, 1], color=BLUE, label='Wheel centre')
    axis.plot(*xy[0], 'o', color=BLUE, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
    axis.set(xlim=(x0, x1), ylim=(y0, y1))
    axis.grid(False)
    axis.set(xlabel='x (m)', ylabel='y (m)', title='Path over the ground (axes not to equal scale)')
    if hasattr(path, 'kind'):
        axis.set_aspect('equal', adjustable='box')
        axis.set_title('Path over the ground (equal scale)')
    legend = _legend(axis, loc='lower left', ncols=2, borderaxespad=.6)
    legend.set_frame_on(True)
    legend.get_frame().set(facecolor=SURFACE, edgecolor='none', alpha=.9)
    # -- friction cone -------------------------------------------------------
    axis = figure.add_subplot(grid[0, 2])
    normal, tangential = log['normal_force_N'], log['tangential_force_N']
    axis.scatter(normal, tangential, s=5, color=BLUE, alpha=.35, linewidths=0, rasterized=True)
    limit = max(normal.max()*1.05, 1.)
    for value in sorted({round(float(v), 3) for v in (np.nanmin(log['ground_friction']), np.nanmax(log['ground_friction']))}):
        axis.plot([0, limit], [0, value*limit], color=SECONDARY, linewidth=1.)
        axis.annotate(f'limit, mu = {value:g}', (limit, value*limit), xytext=(-2, 3), textcoords='offset points',
                      ha='right', va='bottom', fontsize=8, color=SECONDARY)
    axis.set(xlim=(0, limit), ylim=(0, None), xlabel='Normal force (N)', ylabel='Tangential force (N)',
             title='Ground reaction against the friction limit')
    # -- time series ---------------------------------------------------------
    def series(position, title, ylabel, lines, legend=True, limits=None):
        axis = figure.add_subplot(grid[position])
        for values, label, color, width in lines:
            axis.plot(time, values, color=color, linewidth=width, label=label)
        axis.set(title=title, ylabel=ylabel, xlabel='Time (s)', xlim=(time[0], time[-1]))
        if limits:
            axis.set_ylim(*limits)
        if legend and len(lines) > 1:
            _legend(axis, loc='best', ncols=min(len(lines), 4))
        return axis

    error = np.linalg.norm(xy-reference, axis=1)*1000.
    series((1, 0), 'Distance from the reference', 'mm', [(error, None, BLUE, 2.)])
    up = np.degrees(log['tilt_rad'])
    quat = log['reference_quat']
    reference_lean = np.degrees(np.arccos(np.clip(1.-2.*(quat[:, 1]**2+quat[:, 2]**2), -1., 1.)))
    series((1, 1), 'Body lean from vertical', 'degrees',
           [(reference_lean, 'Reference', MUTED, 1.5), (up, 'Actual', BLUE, 2.)])
    slope = np.degrees(np.arccos(np.clip(log['ground_normal'][:, 2], -1., 1.)))
    sensed = np.degrees(np.arccos(np.clip(log['estimated_normal'][:, 2], -1., 1.)))
    series((1, 2), 'Ground slope under the wheel', 'degrees',
           [(slope, 'Actual', MUTED, 1.5), (sensed, 'Sensed from force and moment', BLUE, 2.)])
    weight = float(np.median(np.linalg.norm(log['contact_force_N'], axis=1)[:20])) or 1.
    axis = series((2, 0), 'Ground reaction, as a share of weight', 'force / weight',
                  [(normal/weight, 'Normal', BLUE, 2.), (tangential/weight, 'Tangential', ORANGE, 2.)])
    axis.set_ylim(0, None)
    axis = series((2, 1), 'Share of the friction limit in use', 'tangential / (mu x normal)',
                  [(log['utilization'], None, BLUE, 1.5)], limits=(0, 1.08))
    axis.axhline(1., color=SECONDARY, linewidth=1.)
    axis.annotate('sliding', (time[-1], 1.), xytext=(-2, 2), textcoords='offset points', ha='right',
                  va='bottom', fontsize=8, color=SECONDARY)
    series((2, 2), 'Friction coefficient', 'mu',
           [(log['ground_friction'], 'Under the wheel', MUTED, 1.5),
            (log['estimated_friction'], "Controller's estimate", BLUE, 2.)], limits=(0, .8))
    series((3, 0), 'Drive torques', 'N m',
           [(log['input'][:, 0], 'Shaft', BLUE, 1.5), (log['input'][:, 1], 'Shell A', ORANGE, 1.5),
            (log['input'][:, 2], 'Shell B', AQUA, 1.5)])
    belts = series((3, 1), 'Belt surface speeds', 'm/s',
                   [(log['input'][:, 3], 'Belt A', BLUE, 1.5), (log['input'][:, 4], 'Belt B', ORANGE, 1.5)])
    if np.max(np.abs(log['input'][:, 3:])) < 1e-3:
        belts.set_ylim(-.01, .01)   # idle belts: keep a readable axis instead of a 1e-8 scale
    loads = log['support_loads_N']
    series((3, 2), 'Which surface carries the wheel', 'normal load (N)',
           [(loads[:, 0], 'Shell A', BLUE, 1.5), (loads[:, 1], 'Shell B', ORANGE, 1.5),
            (loads[:, 2], 'Belt A', AQUA, 1.5), (loads[:, 3], 'Belt B', YELLOW, 1.5)])
    outcome = 'completed' if summary['completed'] else f"stopped at {summary['completed_duration_s']:.1f} s: {summary['failure']}"
    heading = title or f"{summary['terrain']} terrain, {CONTROLLER_LABELS[summary['controller']].lower()} controller"
    figure.suptitle(f"{heading} ({outcome})", x=.01, ha='left', fontsize=13, fontweight='semibold', color=INK)
    figure.text(.01, -.012, 'Provisional simulated model: assumed masses, friction and actuator bounds; '
                'state and ground reaction are simulator ground truth.', fontsize=8, color=MUTED)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130, bbox_inches='tight')
    plt.close(figure)
    return output


_COMPARISON_METRICS = (
    ('Final distance from target', 'mm', lambda s: s['tracking']['final_error_m']*1000., True),
    ('Largest distance from the reference', 'mm', lambda s: s['tracking']['max_error_m']*1000., True),
    ('Largest attitude error', 'degrees', lambda s: s['attitude']['max_error_from_reference_deg'], True),
    ('Sliding distance at the contact', 'mm', lambda s: s['contact']['slip_distance_m']*1000., True),
    ('Friction limit in use, 95th percentile', 'share of limit', lambda s: s['contact']['utilization_p95'], False),
    ('Largest normal load', 'x weight', lambda s: s['contact']['normal_load_fraction_max'], False),
)


def plot_comparison(results, output, title='Flat-floor baseline against the contact-aware controller'):
    """Dot plot of each metric per case for both controllers.

    ``results`` maps a case name to ``{controller: summary or list of
    summaries}``. With several runs per case the dot is their median and the
    line spans the best and worst. A case in which any run did not finish is
    drawn with a cross and counted under the figure.
    """
    plt = _pyplot()
    names = list(results)
    figure, axes = plt.subplots(2, 3, figsize=(15., 2.6+.62*len(names)*2), constrained_layout=True, sharey=True)
    offsets = {'flat_baseline': -.16, 'terrain_aware': .16}
    unfinished, several = {}, False
    for axis, (label, unit, getter, logarithmic) in zip(axes.flat, _COMPARISON_METRICS):
        for row, name in enumerate(names):
            for mode, group in results[name].items():
                runs = group if isinstance(group, (list, tuple)) else [group]
                several = several or len(runs) > 1
                values = np.array([getter(run) for run in runs], float)
                values = values[np.isfinite(values)]
                if not len(values):
                    continue
                y = row+offsets.get(mode, 0.)
                missed = sum(not run['completed'] for run in runs)
                if missed:
                    unfinished[name, mode] = (missed, len(runs))
                if len(values) > 1:
                    axis.plot([values.min(), values.max()], [y, y], color=CONTROLLER_COLORS[mode], linewidth=2.,
                              alpha=.45, zorder=2)
                axis.plot(np.median(values), y, 'X' if missed else 'o', color=CONTROLLER_COLORS[mode], markersize=8,
                          markeredgecolor=SURFACE, markeredgewidth=1.5, zorder=3,
                          label=CONTROLLER_LABELS[mode] if row == 0 else None)
        if logarithmic:
            axis.set_xscale('log')
        axis.set(title=label, xlabel=unit)
        axis.set_yticks(range(len(names)), [name.replace('_', ' ') for name in names])
        axis.set_ylim(len(names)-.5, -.5)
        axis.grid(axis='y', visible=False)
    _legend(axes[0, 0], loc='lower right')
    figure.suptitle(title, x=.01, ha='left', fontsize=13, fontweight='semibold', color=INK)
    note = ('Dot: median over the starts of each case. Line: best to worst start. Both controllers start from the '
            'same states.' if several else 'Each row is one run with both controllers from the same starting state.')
    if unfinished:
        note += ' Crosses mark cases with runs that did not finish: '+', '.join(
            f"{name.replace('_', ' ')} ({CONTROLLER_LABELS[mode].lower()}, {missed} of {total})"
            for (name, mode), (missed, total) in unfinished.items())+'.'
    figure.text(.01, -.02, note, fontsize=8, color=MUTED, wrap=True)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130, bbox_inches='tight')
    plt.close(figure)
    return output


def plot_sweep(sweep, output, title='Controller tuning: suite cost against each setting'):
    """Small multiples: suite cost as one setting varies with the others held.

    ``sweep`` is a list of ``{'parameter', 'values', 'costs', 'failures',
    'chosen'}``. The chosen value is marked and labelled.
    """
    plt = _pyplot()
    columns = min(3, len(sweep))
    rows = int(np.ceil(len(sweep)/columns))
    figure, axes = plt.subplots(rows, columns, figsize=(5.*columns, 3.3*rows), constrained_layout=True, squeeze=False)
    for axis, item in zip(axes.flat, sweep):
        labels = [str(value) for value in item['values']]
        positions = np.arange(len(labels))
        costs = np.asarray(item['costs'], float)
        errors = np.asarray(item.get('standard_errors', np.zeros(len(costs))), float)
        # Whiskers are one standard error of the mean over the suite's runs.
        axis.vlines(positions, costs-errors, costs+errors, color=SECONDARY, linewidth=1., zorder=1)
        axis.plot(positions, costs, color=BLUE, marker='o', markersize=7, markeredgecolor=SURFACE,
                  markeredgewidth=1.5, zorder=2)
        chosen = item['values'].index(item['chosen'])
        axis.plot(positions[chosen], costs[chosen], 'o', color=INK, markersize=9, markeredgecolor=SURFACE,
                  markeredgewidth=1.5, zorder=3)
        axis.annotate('kept', (positions[chosen], costs[chosen]+errors[chosen]), xytext=(0, 5),
                      textcoords='offset points', ha='center', fontsize=8, color=INK)
        for position, cost, error, failures in zip(positions, costs, errors, item['failures']):
            if failures:
                axis.annotate(f'{failures} did not finish', (position, cost-error), xytext=(0, -12), textcoords='offset points',
                              ha='center', fontsize=8, color=SECONDARY)
        axis.set_xticks(positions, labels)
        axis.set(title=item['parameter'].replace('_', ' '), ylabel='Mean suite cost')
        axis.set_ylim(0, 1.3*float(np.max(costs+errors)))
        axis.margins(x=.18)
    for axis in axes.flat[len(sweep):]:
        axis.set_visible(False)
    figure.suptitle(title, x=.01, ha='left', fontsize=13, fontweight='semibold', color=INK)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130, bbox_inches='tight')
    plt.close(figure)
    return output


def plot_ablation(rows, output, title='Suite cost with one setting changed at a time'):
    """Horizontal bars of mean suite cost per controller variant, lowest first.

    Each row needs ``label``, ``cost`` and ``failures``. The row marked
    ``reference`` is drawn in ink and its cost is carried down as a line, so
    every other bar reads as better or worse than it.
    """
    plt = _pyplot()
    rows = sorted(rows, key=lambda row: row['cost'])
    figure, axis = plt.subplots(figsize=(10., 1.4+.42*len(rows)), constrained_layout=True)
    largest = max(row['cost'] for row in rows)
    for index, row in enumerate(rows):
        axis.barh(index, row['cost'], height=.42, color=INK if row.get('reference') else BLUE, linewidth=0)
        label = f"{row['cost']:.2f}"
        if row['failures']:
            label += f"   {row['failures']} did not finish"
        axis.annotate(label, (row['cost'], index), xytext=(5, 0), textcoords='offset points', va='center',
                      fontsize=8, color=SECONDARY)
    reference = next((row['cost'] for row in rows if row.get('reference')), None)
    if reference is not None:
        axis.axvline(reference, color=SECONDARY, linewidth=1., zorder=0)
    axis.set_yticks(range(len(rows)), [row['label'] for row in rows])
    axis.set_ylim(len(rows)-.5, -.5)
    axis.set_xlim(0, largest*1.18)
    axis.grid(axis='y', visible=False)
    axis.set(xlabel='Mean suite cost (lower is better)', title=title)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130, bbox_inches='tight')
    plt.close(figure)
    return output
