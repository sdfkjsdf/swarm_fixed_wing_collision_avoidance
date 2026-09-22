#!/usr/bin/env python3
"""Offline report: candidate readiness, trajectory cadence, proposal-to-commit.

Reads complete shutdown logs; never imported by the real-time nodes. Candidate
readiness is the first successful seven-trajectory output in each epoch, NOT a
dedicated measurement of the candidate-ID generation function. All durations
use one Pi's monotonic clock; x coordinates use its recorded state timestamp.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from analyze_stopped_stage_timing import analyze
from read_stopped_observations import parse


def distribution(values):
    a = np.asarray(values, dtype=float)
    if not len(a) or not np.all(np.isfinite(a)) or np.any(a < 0):
        raise ValueError('Missing or invalid duration data')
    return dict(count=len(a), mean_s=float(a.mean()), median_s=float(np.median(a)),
                p95_s=float(np.percentile(a, 95)), p99_s=float(np.percentile(a, 99)),
                max_s=float(a.max()))


def cadence(rows, start, reference):
    if len(rows) < 2:
        raise ValueError('At least two completion records are required')
    delta_ns = np.diff([r[5] for r in rows])
    if np.any(delta_ns <= 0):
        raise ValueError('Completion timestamps must strictly increase')
    elapsed = delta_ns / 1e9
    metrics = distribution(elapsed)
    metrics.update(completed_events=len(rows), reference_interval_s=reference,
                   completion_rate_hz=float(1 / elapsed.mean()),
                   above_reference_count=int(np.sum(delta_ns > round(reference * 1e9))),
                   definition='Consecutive completion timestamps; not pure CPU computation time')
    return (np.array([(r[1] - start) / 1e6 for r in rows[1:]]), elapsed, metrics)


def load(log, summary, computation=False):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    start = summary['actual_evaluation_start_ns'] // 1000
    duration = summary['common_duration_s']
    end = start + round(duration * 1e6)
    text = log.read_text()
    validation = analyze(text, start, end)
    if validation['vehicle'] != 0 or validation['dropped_records']:
        raise ValueError('Expected a complete, non-overflowed Pi vehicle-0 log')
    rows = [list(map(int, line.split(',')[1:])) for line in text.splitlines()
            if line.startswith('[stop-stage],')]
    completed = [r for r in rows if r[0] == 1 and r[7] and r[8]]
    if any(r[6] != 7 for r in completed):
        raise ValueError('This report expects the seven-candidate configuration')
    first = {}
    for r in completed:
        first.setdefault(r[2], r)
    candidates = [r for r in first.values() if start <= r[1] <= end]
    trajectories = [r for r in completed if start <= r[1] <= end]
    p1 = cadence(candidates, start, .25)
    p2 = cadence(trajectories, start, .05)
    epoch_steps = np.diff([r[2] for r in candidates])
    if np.any(epoch_steps <= 0):
        raise ValueError('Candidate epochs must increase')
    p1[2].update(missing_epochs_between_observations=int(np.sum(epoch_steps - 1)),
                 definition='First complete seven-candidate trajectory set in each epoch; '
                 'proxy for readiness, NOT candidate-ID generation function timing')
    if computation:
        spans = []
        for stage, label, reference in [
                (2, 'evaluateCurrentSet: graph construction, component search, '
                 'cross-check and proposal preparation', .25),
                (1, 'Seven-candidate trajectory refresh including packet preparation '
                 'and local reconstruction/covariance processing', .05)]:
            selected = [r for r in rows if r[0] == stage and start <= r[1] <= end]
            if any(r[6] != 7 or not r[8] or (stage == 1 and not r[7]) for r in selected):
                raise ValueError('Incomplete stage output: inspect before plotting')
            x = np.array([(r[1] - start) / 1e6 for r in selected])
            y = np.array([(r[5] - r[4]) / 1e9 for r in selected])
            stats = distribution(y)
            stats.update(definition=label, cycle_reference_s=reference,
                         proposal_unavailable_count=sum(not r[7] for r in selected)
                         if stage == 2 else None,
                         clock='Single-host monotonic elapsed span, not CPU thread time')
            spans.append((x, y, stats))
        p1, p2 = spans

    blocks = [b for b in parse(text) if b['vehicle'] == 0 and b['owner'] == 'worker']
    if len(blocks) != 1 or blocks[0]['dropped']:
        raise ValueError('Expected one complete, non-overflowed worker trace')
    block = blocks[0]
    cls = get_message(block['message_type'])
    messages = [deserialize_message(raw, cls) for _, raw in block['records']]
    if any(m.vehicle_id != 0 or m.dropped_trace_count for m in messages):
        raise ValueError('Mixed vehicle IDs or dropped budget events')
    proposals, commits, matches = {}, set(), []
    in_window = lambda m: start <= m.state_timestamp_us <= end
    for m in sorted(messages, key=lambda m: m.steady_ns):
        if m.event not in (1, 2):
            continue
        key = (m.epoch, m.candidate_id, m.input_revision)
        if m.event == 1:
            if key in proposals:
                raise ValueError('Ambiguous duplicate proposal key')
            proposals[key] = m
        else:
            if key in commits:
                raise ValueError('Duplicate commit key')
            commits.add(key)
            if in_window(m):
                if key not in proposals:
                    raise ValueError('Commit has no matching proposal')
                proposal = proposals[key]
                matches.append(((m.state_timestamp_us - start) / 1e6,
                                (m.steady_ns - proposal.steady_ns) / 1e9))
    x, y = np.array(matches).T
    metrics = distribution(y)
    window_proposals = {k for k, m in proposals.items() if in_window(m)}
    metrics.update(proposals_in_window=len(window_proposals),
                   proposals_without_commit_in_complete_log=len(window_proposals - commits),
                   matched_commits_in_window=len(matches),
                   definition='Pi proposal-ready event 1 to matching commit event 2; '
                   'includes coordination waiting, excludes prior search and later PX4 delivery',
                   numerical_agreement_deadline_s=None)
    report = dict(log=str(log.resolve()), vehicle_id=0,
                  source_window_start_us=start, source_window_end_us=end,
                  common_formation_duration_s=duration,
                  candidate_readiness=p1[2], trajectory_refresh=p2[2], agreement=metrics,
                  caveats=['Cadence includes input and scheduling delays between completions.',
                           '50/250 ms lines are 1/f cycle references, not paper WCET bounds.',
                           'No numeric agreement deadline is assigned.',
                           'No packet capture, rerun, or runtime-code modification required.'])
    report['measurement_mode'] = 'computation' if computation else 'cadence'
    if computation:
        report['group_and_combination_evaluation'] = report.pop('candidate_readiness')
        report['caveats'][0] = ('Computation = stage end minus stage start, excluding upstream input '
                                'draining and between-job waits; scheduler preemption within the span is included.')
        report['caveats'][1] = ('50/250 ms are this research project\'s computation-time targets '
                                'adopted from reference-study operating cadences, not published '
                                'WCET bounds or verified end-to-end deadlines.')
        for metric in (p1[2], p2[2]):
            metric['computation_target_s'] = metric['cycle_reference_s']
            metric['observed_max_below_target'] = metric['max_s'] < metric['computation_target_s']
    return [p1, p2, (x, y, metrics)], report


def plot(panels, report, output):
    matplotlib.rcParams.update({'path.simplify': False, 'font.size': 10,
                               'svg.fonttype': 'none'})
    blue, grey, red = '#24658c', '#616161', '#c72c2c'
    titles = ['Candidate-set readiness per new epoch',
              'Candidate trajectory refresh',
              'Agreement: proposal ready to commit']
    suffixes = ['candidate_readiness', 'trajectory_refresh', 'agreement_latency']
    notes = ['First complete seven-candidate set per epoch; not candidate-ID function timing.',
             'Consecutive trajectory completion intervals; includes input and scheduling waits.',
             'Proposal-to-commit elapsed time includes coordination waiting; not pure computation time.']
    computation = report['measurement_mode'] == 'computation'
    if computation:
        titles[:2] = ['Risk grouping and maneuver combination evaluation',
                      'Seven-candidate trajectory refresh']
        suffixes[:2] = ['group_and_combination_compute', 'trajectory_compute']
        notes[:2] = ['Our target follows reference-study cadence; not an end-to-end deadline. Broken y-axis.'] * 2
    output.parent.mkdir(parents=True, exist_ok=True)
    for i, (x, y, metrics) in enumerate(panels):
        fig, ax = plt.subplots(figsize=(9, 3.9))
        fig.subplots_adjust(left=.10, right=.98, bottom=.22, top=.71)
        upper = None
        if computation and i < 2:
            ax.remove()
            grid = fig.add_gridspec(2, 1, left=.10, right=.98, bottom=.22, top=.71,
                                   height_ratios=[1, 4], hspace=.17)
            upper = fig.add_subplot(grid[0])
            ax = fig.add_subplot(grid[1], sharex=upper)
        scale = 1000 if computation and i < 2 else 1
        values = y * scale
        ax.plot(x, values, color=blue, linewidth=.6, alpha=.9)
        ax.axhline(metrics['mean_s'] * scale, color=grey, linestyle='--', linewidth=1)
        peak = int(np.argmax(y))
        ax.plot(x[peak], values[peak], marker='D', markersize=4, color=blue, zorder=4)
        reference = (.25, .05)[i] if i < 2 else None
        if reference is not None:
            (upper if upper is not None else ax).axhline(reference * scale, color=red, linewidth=1.25)
        title = fig.suptitle(f'Raspberry Pi 5 | {titles[i]}',
                            x=.54, y=.975, fontsize=11)
        annotation = f"Mean {metrics['mean_s']:.4f} s   |   Max {metrics['max_s']:.4f} s"
        if computation and i < 2:
            annotation = (f"Mean {metrics['mean_s']*1000:.2f} ms   |   Max {metrics['max_s']*1000:.2f} ms"
                          f"   |   Computation target {reference*1000:.0f} ms ({(4,20)[i]} Hz)")
        elif i < 2:
            annotation += (f"   |   Average {metrics['completion_rate_hz']:.2f} Hz"
                           f"   |   Reference {reference:.3f} s ({(4,20)[i]} Hz)")
        else:
            annotation += f"   |   {metrics['count']} matched commits; no prescribed deadline"
        statistics = fig.text(.54, .765, annotation,
                              fontsize=8.5, ha='center', va='center', color='#333333')
        ax.set_ylabel(('Computation time [ms]' if computation else 'Completion interval [s]')
                      if i < 2 else 'Agreement latency [s]')
        ax.set_ylim(0, max(float(values.max()), 0 if upper is not None else reference or 0) * 1.14)
        if upper is not None:
            limit = reference * scale
            upper.set_ylim(limit * .92, limit * 1.1)
            upper.set_yticks([limit])
            upper.tick_params(axis='x', bottom=False, labelbottom=False)
            upper.tick_params(axis='y', labelsize=9, length=3)
            for name, spine in upper.spines.items():
                spine.set_visible(name == 'left')
                spine.set_color('#777777')
                spine.set_linewidth(.6)
            if float(values.max()) >= limit * .92:
                raise ValueError('Broken axis would hide data')
            for target, at in [(upper, 0), (ax, 1)]:
                target.plot([0, 1], [at, at], transform=target.transAxes,
                            marker=[(-1, -.5), (1, .5)], markersize=7,
                            linestyle='none', color=grey, clip_on=False, markeredgewidth=.8)
        ax.grid(color='#dddddd', linewidth=.5)
        ax.set_axisbelow(True)
        ax.tick_params(labelsize=9, length=3)
        for name, spine in ax.spines.items():
            spine.set_visible(name in ('left', 'bottom'))
            spine.set_linewidth(.6)
            spine.set_color('#777777')
        ax.set_xlim(0, report['common_formation_duration_s'])
        ax.set_xticks(np.arange(0, report['common_formation_duration_s'] + .001, 30))
        ax.set_xlabel('Time from common Formation start [s]', labelpad=8)
        handles = [
            Line2D([], [], color=blue, linewidth=1,
                   label=('Measured computation' if computation else 'Measured interval')
                   if i < 2 else 'Measured agreement latency'),
            Line2D([], [], color=grey, linestyle='--', label='Mean')]
        if reference is not None:
            handles.append(Line2D([], [], color=red, linewidth=1.25,
                                  label='Computation-time target' if computation else 'Cycle reference (1/f)'))
        handles.append(Line2D([], [], color=blue, marker='D', linestyle='none',
                              markersize=4, label='Maximum'))
        legend = fig.legend(handles=handles, loc='upper center',
                            bbox_to_anchor=(.54, .895), ncol=len(handles),
                            frameon=False, fontsize=9, columnspacing=1.6)
        note = fig.text(.54, .035, notes[i], ha='center', va='center',
                        fontsize=8.3, color='#444444')
        fig.canvas.draw()
        renderer = fig.canvas.get_renderer()
        texts = [title, statistics, note, ax.xaxis.label]
        for artist in [ax, *texts, *([upper] if upper is not None else [])]:
            if legend.get_window_extent(renderer).overlaps(artist.get_window_extent(renderer)):
                raise RuntimeError('Legend overlaps data axes or text')
        for artist in texts:
            bounds = artist.get_window_extent(renderer)
            if not fig.bbox.contains(bounds.x0, bounds.y0) or not fig.bbox.contains(bounds.x1, bounds.y1):
                raise RuntimeError('Text extends outside the report image')
        target = output.with_name(f'{output.stem}_{suffixes[i]}.png')
        fig.savefig(target, dpi=300, facecolor='white')
        fig.savefig(target.with_suffix('.svg'), facecolor='white')
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--computation', action='store_true',
                        help='Stage start-to-end durations instead of completion intervals')
    args = parser.parse_args()
    panels, report = load(args.log, json.loads(args.summary.read_text()), args.computation)
    plot(panels, report, args.output)
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
