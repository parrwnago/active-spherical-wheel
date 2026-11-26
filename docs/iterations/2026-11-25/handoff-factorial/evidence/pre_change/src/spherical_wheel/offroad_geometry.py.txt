"""Where the ground touches the wheel, and which drives still have leverage there.

On level ground the support point stays on one great circle of the wheel, so a
single angle (the carrier's phase) says which surface is down. On a slope the
ground normal also leans sideways out of the carrier's plane: the body leans
to balance and the slope adds to it. The support point then moves off that
great circle, and can leave the polar belt cap while a pole is down. This
module maps that, and plots the slope-holding results against it.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .offroad_analysis import (AQUA, CONTROLLER_COLORS, CONTROLLER_LABELS, HEIGHT_RAMP, INK, ORANGE,
                               SECONDARY, SURFACE, YELLOW, _legend, _pyplot)


def support_leverage(config, phases_deg, lateral_deg):
    """Weakest-direction drive leverage at each support point on the wheel.

    A support point is located by ``phase`` (how far the carrier has turned
    about the shaft relative to the ground normal) and ``lateral`` (how far
    the ground normal leans sideways out of the carrier's own plane). The
    loaded surface's drives move the contact in two tangential directions:
    the shaft, and either the loaded shell or the belt. The smaller singular
    value of that map, as a share of the wheel radius, is the leverage left in
    the weakest direction. A belt's surface speed is counted as one radius of
    lever, so 1 means a full-radius lever in every direction and 0 means one
    direction cannot be driven at all.

    Returns ``(leverage, patch)`` shaped ``(len(lateral_deg), len(phases_deg))``
    with ``patch`` 0 in the equatorial gap, 1 on a shell and 2 on a belt cap.
    """
    geometry = config['geometry']
    radius, gap = geometry['sphere_radius_m'], geometry.get('equatorial_gap_half_m', 0.)
    rim = radius*np.cos(np.radians(geometry['polar_cap_angle_deg']))
    leverage = np.zeros((len(lateral_deg), len(phases_deg)))
    patch = np.zeros(leverage.shape, dtype=int)
    x_axis, y_axis = np.array([1., 0., 0.]), np.array([0., 1., 0.])
    for row, lateral in enumerate(np.radians(lateral_deg)):
        for column, phase in enumerate(np.radians(phases_deg)):
            # Ground normal in the carrier frame; the support point is opposite it.
            normal = np.array([-np.sin(phase)*np.cos(lateral), np.sin(lateral), np.cos(phase)*np.cos(lateral)])
            direction = -normal
            local_x = abs(radius*direction[0])
            on_belt = local_x >= rim
            shaft = np.cross(y_axis, direction)
            sideways = (np.eye(3)-np.outer(normal, normal))@y_axis if on_belt else np.cross(x_axis, direction)
            patch[row, column] = 2 if on_belt else (1 if local_x >= gap else 0)
            leverage[row, column] = np.linalg.svd(np.column_stack((shaft, sideways)), compute_uv=False)[-1]
    return leverage, patch


def support_path(log):
    """Phase and sideways angle of the support point over a run, in degrees, while loaded."""
    support = log['support_direction']
    loaded = log['normal_force_N'] > 1.
    phase = np.degrees(np.arctan2(-support[:, 0], support[:, 2]))[loaded]
    lateral = np.degrees(np.arcsin(np.clip(support[:, 1], -1., 1.)))[loaded]
    return phase, lateral


def plot_support_map(config, output, logs=None,
                     title='Where the ground touches the wheel, and how much drive leverage is left there'):
    """Leverage over the wheel surface, with the support path of saved runs drawn on it.

    ``logs`` maps a label to a run log (at most three). Dark is full leverage
    in both directions; pale is a direction no loaded drive can move.
    """
    plt = _pyplot()
    from matplotlib.colors import LinearSegmentedColormap
    phases, lateral = np.linspace(-180., 180., 361), np.linspace(-32., 32., 129)
    leverage, patch = support_leverage(config, phases, lateral)
    figure, axis = plt.subplots(figsize=(13.5, 5.6), constrained_layout=True)
    colormap = LinearSegmentedColormap.from_list('leverage', ['#f0efec']+HEIGHT_RAMP)
    mesh = axis.pcolormesh(phases, lateral, leverage, cmap=colormap, vmin=0., vmax=1., shading='auto', rasterized=True)
    bar = figure.colorbar(mesh, ax=axis, pad=.01, fraction=.03)
    bar.set_label('Weakest-direction drive leverage (share of wheel radius)', color=SECONDARY)
    bar.outline.set_visible(False)
    axis.contour(phases, lateral, patch, levels=[1.5], colors=[INK], linewidths=1.)
    for centre, name in ((90., 'belt A cap'), (-90., 'belt B cap')):
        axis.annotate(name, (centre, 11.), ha='center', va='center', fontsize=8, color='#ffffff')
    for (label, log), color in zip((logs or {}).items(), (ORANGE, AQUA, YELLOW)):
        phase, side = support_path(log)
        axis.scatter(phase, side, s=4, color=color, linewidths=0, label=label, rasterized=True, zorder=3)
    if logs:
        legend = _legend(axis, loc='lower left', markerscale=3.5, ncols=len(logs))
        legend.set_frame_on(True)
        legend.get_frame().set(facecolor=SURFACE, edgecolor='none', alpha=.9)
    axis.grid(False)
    axis.set(xlabel='Phase: carrier angle about the shaft, measured against the ground normal (degrees). '
                    'The equatorial gap is at 0 and 180.',
             ylabel='Sideways lean of the ground normal\nout of the carrier plane (degrees)',
             xlim=(-180, 180), ylim=(lateral[0], lateral[-1]), title=title)
    axis.set_xticks(np.arange(-180, 181, 45))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130, bbox_inches='tight')
    plt.close(figure)
    return output


def plot_envelope(cases, limits, output, title='Holding position on a uniform slope through a 0.5 N push'):
    """Final distance from the target against slope, by slope direction and support.

    ``cases`` come from ``slope_envelope`` and ``limits`` from ``slope_limits``.
    A run that did not finish (it fell, or drifted more than 1 m from the
    target) is drawn as a cross on the top edge of its panel.
    """
    plt = _pyplot()
    phases = sorted({case['phase_rad'] for case in cases})
    panels = [(direction, phase) for direction in ('along', 'across') for phase in phases]
    figure, axes = plt.subplots(2, len(phases), figsize=(6.4*len(phases), 8.4), constrained_layout=True,
                                sharex=True, sharey=True, squeeze=False)
    ceiling = 2000.
    for axis, (direction, phase) in zip(axes.flat, panels):
        pole = abs(abs(phase)-np.pi/2) < .2
        for mode in ('flat_baseline', 'terrain_aware'):
            rows = sorted((case for case in cases if case['direction'] == direction
                           and abs(case['phase_rad']-phase) < 1e-9 and case['controller'] == mode),
                          key=lambda case: case['grade_deg'])
            grades = [row['grade_deg'] for row in rows]
            values = [max(row['final_error_m']*1000., .1) if row['completed'] else ceiling for row in rows]
            axis.plot(grades, values, color=CONTROLLER_COLORS[mode], label=CONTROLLER_LABELS[mode], zorder=2)
            for grade, value, row in zip(grades, values, rows):
                axis.plot(grade, value, 'o' if row['completed'] else 'X', color=CONTROLLER_COLORS[mode],
                          markersize=8, markeredgecolor=SURFACE, markeredgewidth=1.5, zorder=3)
        if direction == 'across':
            value = limits['belt_cap_side_slope_limit_deg'] if pole else limits['shell_torque_limit_deg_at_initial_phase']
            text = 'support leaves the belt cap' if pole else 'shell torque bound'
            axis.axvline(value, color=SECONDARY, linewidth=1.)
            axis.annotate(f'{text}, {value:.1f} deg', (value, .12), xytext=(4, 0), textcoords='offset points',
                          fontsize=8, color=SECONDARY, va='bottom')
        support = 'a pole down, on the belt' if pole else f'on a shell, phase {phase:.2f} rad'
        slope = 'Slope along the shaft-driven direction' if direction == 'along' else 'Side slope'
        axis.set(title=f'{slope}; {support}', yscale='log', ylim=(.08, ceiling*1.6))
    for axis in axes[-1]:
        axis.set_xlabel('Slope (degrees)')
    for axis in axes[:, 0]:
        axis.set_ylabel('Final distance from target (mm)')
        axis.annotate('did not\nfinish', xy=(0., ceiling), xycoords=('axes fraction', 'data'), xytext=(-6, 0),
                      textcoords='offset points', ha='right', va='center', fontsize=8, color=SECONDARY,
                      annotation_clip=False)
    _legend(axes[0, 0], loc='center left')
    figure.suptitle(title, x=.01, ha='left', fontsize=13, fontweight='semibold', color=INK)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130, bbox_inches='tight')
    plt.close(figure)
    return output
