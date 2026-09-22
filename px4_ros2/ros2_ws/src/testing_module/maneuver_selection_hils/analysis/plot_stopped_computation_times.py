#!/usr/bin/env python3
"""Offline plot of measured stage spans; never imported by flight nodes."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from analyze_stopped_stage_timing import analyze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycle-budget', action='store_true',
                        help='Zoom measured values with explicit broken axes and 1/f reference lines')
    args = parser.parse_args()
    summary = json.loads(args.summary.read_text())
    start = summary['actual_evaluation_start_ns'] // 1000
    duration = summary['common_duration_s']
    end = start + round(duration * 1e6)
    text = args.log.read_text()
    report = analyze(text, start, end)  # validates spans, order and dump footer
    if report['dropped_records']:
        raise ValueError('Timing buffer overflow: refusing an unqualified full-run plot')
    rows = [list(map(int, line.split(',')[1:])) for line in text.splitlines()
            if line.startswith('[stop-stage],')]
    panels = [(1, 'Trajectory refresh'), (2, 'Maneuver combination evaluation')]
    # Preserve every measured vertex; only remove dense point markers.
    matplotlib.rcParams['path.simplify'] = False
    upper_axes = []
    if args.cycle_budget:
        fig = plt.figure(figsize=(9, 6.3))
        outer = fig.add_gridspec(2, 1, left=.09, right=.98, bottom=.13,
                                 top=.855, hspace=.55)
        axes = []
        for index in range(2):
            inner = outer[index].subgridspec(2, 1, height_ratios=[1, 4], hspace=.13)
            upper = fig.add_subplot(inner[0])
            lower = fig.add_subplot(inner[1], sharex=upper)
            upper_axes.append(upper)
            axes.append(lower)
    else:
        fig, axes = plt.subplots(2, 1, figsize=(9, 5.8), sharex=True)
    fig.patch.set_facecolor('white')
    measured_color, mean_color = '#27648c', '#555555'
    metrics = {}
    heading_artists = []
    for ax, (stage, title) in zip(axes, panels):
        selected = [r for r in rows if r[0] == stage and start <= r[1] <= end]
        if not selected:
            raise ValueError(f'Missing stage {stage}')
        time = np.array([(r[1] - start) / 1e6 for r in selected])
        elapsed = np.array([(r[5] - r[4]) / 1e6 for r in selected])
        mean = float(elapsed.mean())
        maximum = float(elapsed.max())
        metrics[title] = dict(count=len(selected), mean_ms=mean, max_ms=maximum,
                              p95_ms=float(np.percentile(elapsed, 95)))
        ax.set_facecolor('white')
        ax.plot(time, elapsed, color=measured_color, linewidth=.45, alpha=.9)
        ax.axhline(mean, color=mean_color, linestyle='--', linewidth=1.1)
        peak = int(np.argmax(elapsed))
        ax.plot(time[peak], elapsed[peak], marker='D', markersize=4,
                color=measured_color, linestyle='none', clip_on=False, zorder=4)
        heading_ax = upper_axes[stage - 1] if args.cycle_budget else ax
        heading = heading_ax.set_title(f'({chr(96 + stage)}) {title}', fontsize=11,
                               loc='left', pad=12)
        values = heading_ax.text(1, 1.065,
                f'Mean: {mean:.2f} ms    Max: {maximum:.2f} ms',
                transform=heading_ax.transAxes, ha='right', va='bottom',
                fontsize=9, color='#333333')
        heading_artists.append((heading, values))
        ax.set_ylabel('Computation time [ms]', fontsize=10, labelpad=9)
        ax.set_ylim(0, 4 if stage == 1 else 11)
        ax.set_yticks(np.arange(0, 5, 1) if stage == 1 else np.arange(0, 11, 2))
        ax.grid(color='#dddddd', linewidth=.5)
        ax.set_axisbelow(True)
        ax.tick_params(axis='both', labelsize=9, length=3, width=.6, pad=5)
        for name, spine in ax.spines.items():
            spine.set_visible(name in ('left', 'bottom'))
            spine.set_linewidth(.6)
            spine.set_color('#777777')
        if args.cycle_budget:
            # A removed y interval is explicit; all measured points remain visible.
            low, high = ((1.5, 3.8) if stage == 1 else (4, 10.2))
            if elapsed.min() < low or elapsed.max() > high:
                raise ValueError('Zoom would hide a measured value')
            ax.set_ylim(low, high)
            ax.set_yticks([1.5, 2, 2.5, 3, 3.5] if stage == 1 else [4, 6, 8, 10])
            ax.set_xlim(0, duration)
            ax.set_xticks(np.arange(0, duration + .001, 30))
            ax.tick_params(labelbottom=stage == 2)
            upper = upper_axes[stage - 1]
            budget, hz = (50, 20) if stage == 1 else (250, 4)
            upper.set_ylim(budget * .94, budget * 1.1)
            upper.set_yticks([budget])
            upper.axhline(budget, color='#c72c2c', linewidth=1.3)
            upper.text(.99, .78, f'{hz} Hz cycle budget: {budget} ms',
                       transform=upper.transAxes, ha='right', va='center', fontsize=9)
            upper.tick_params(axis='x', bottom=False, labelbottom=False)
            upper.tick_params(axis='y', labelsize=9, length=3, width=.6, pad=5)
            for name, spine in upper.spines.items():
                spine.set_visible(name == 'left')
                spine.set_linewidth(.6)
                spine.set_color('#777777')
            # Constant-size diagonal glyphs at the discontinuity, on both edges.
            for target, y in [(upper, 0), (ax, 1)]:
                target.plot([0, 1], [y, y], transform=target.transAxes,
                            marker=[(-1, -.5), (1, .5)], markersize=7,
                            linestyle='none', color='#666666', markeredgewidth=.8,
                            clip_on=False)
    axes[-1].set_xlim(0, duration)
    axes[-1].set_xticks(np.arange(0, duration + .001, 30))
    axes[-1].set_xlabel('Time from common Formation start [s]', fontsize=10, labelpad=8)
    handles = [
        Line2D([], [], color=measured_color, linewidth=1, label='Measured computation'),
        Line2D([], [], color=mean_color, linewidth=1.25, linestyle='--', label='Mean'),
        Line2D([], [], color=measured_color, marker='D', markersize=4,
               linestyle='none', label='Maximum')]
    if args.cycle_budget:
        handles.append(Line2D([], [], color='#c72c2c', linewidth=1.3,
                              label='Cycle budget (1/f)'))
    legend = fig.legend(handles=handles,
        loc='upper center', bbox_to_anchor=(.53, .985), ncol=len(handles),
        frameon=False, fontsize=9, handlelength=2.5, columnspacing=1.8)
    if args.cycle_budget:
        fig.text(.535, .027, 'Broken y-axes: measured ranges enlarged; cycle budgets derived from 20 Hz and 4 Hz.',
                 ha='center', fontsize=8, color='#444444')
    else:
        fig.subplots_adjust(top=.855, bottom=.115, left=.085, right=.98, hspace=.43)
    fig.canvas.draw()
    legend_box = legend.get_window_extent(fig.canvas.get_renderer())
    if any(legend_box.overlaps(ax.get_window_extent()) for ax in list(axes) + upper_axes):
        raise RuntimeError('Legend must remain outside all data axes')
    for heading, values in heading_artists:
        if heading.get_window_extent().overlaps(values.get_window_extent()):
            raise RuntimeError('Panel heading overlaps statistics')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=300)
    fig.savefig(args.output.with_suffix('.svg'))
    plt.close(fig)
    print(json.dumps(dict(log=str(args.log), evaluation_duration_s=duration,
                          dropped_records=report['dropped_records'], stages=metrics), indent=2))


if __name__ == '__main__':
    main()
