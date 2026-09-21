#!/usr/bin/env python3
"""Offline-only decomposition of stopped-buffer timing, not a deadline proof."""
import argparse
import json
from pathlib import Path
from analyze_stopped_stage_timing import analyze as analyze_stages, stats


def parse(text):
    begin = end = None
    pipelines, beliefs, stages = [], [], []
    for line in text.splitlines():
        if not line.startswith(('[stop-pipeline', '[stop-belief]', '[stop-stage],')):
            continue
        key, *fields = line.split(',')
        r = list(map(int, fields))
        if key == '[stop-pipeline-begin]':
            if begin is not None or len(r) != 6 or r[0] != 1:
                raise ValueError('Expected exactly one version-1 pipeline dump')
            begin = r
        elif key == '[stop-pipeline-end]':
            if begin is None or end is not None:
                raise ValueError('Unexpected pipeline footer')
            end = r
        elif key == '[stop-stage]':
            stages.append(r)
        else:
            if begin is None or end is not None or len(r) != 9:
                raise ValueError('Malformed/outside-dump pipeline record')
            (pipelines if key == '[stop-pipeline]' else beliefs).append(r)
    if (begin is None or end != [begin[1], begin[2], begin[4]]
            or len(pipelines) != begin[2] or len(beliefs) != begin[4]):
        raise ValueError('Incomplete pipeline dump')
    for i, r in enumerate(pipelines):
        if not (0 < r[1] <= r[2] <= r[3]) or (i and r[1] < pipelines[i-1][3]):
            raise ValueError('Invalid/overlapping worker pass')
        if r[4] + r[5] > r[2] - r[1]:
            raise ValueError('Subspans exceed drain span')
    for r in beliefs:
        if not (0 < r[5] <= r[6] <= r[7]) or (r[2] and r[2] > r[5]):
            raise ValueError('Invalid local monotonic belief timing')
    return begin, pipelines, beliefs, stages


def analyze(text, low=0, high=None):
    base = analyze_stages(text, low, high)  # also validates complete stage dump
    header, all_passes, all_beliefs, stages = parse(text)
    inside = lambda source: source >= low and (high is None or source <= high)
    passes = [r for r in all_passes if inside(r[0])]
    beliefs = [r for r in all_beliefs if inside(r[0])]
    callbacks = [r for r in beliefs if r[2]]
    accepted = [r for r in beliefs if r[8]]
    def durations(rows, a, b, scale=1e6):
        return stats([(r[b]-r[a])/scale for r in rows])
    def intervals(rows, column, scale=1e6):
        return stats([(b[column]-a[column])/scale for a,b in zip(rows, rows[1:])])
    report = dict(vehicle=header[1], stage_timing=base,
        pipeline_records=len(all_passes), belief_records=len(all_beliefs),
        pipeline_dropped=header[3], belief_dropped=header[5],
        accepted_beliefs=len(accepted), rejected_beliefs=len(beliefs)-len(accepted),
        callback_interval=intervals(callbacks, 2),
        source_interval=intervals(callbacks, 0, 1000),
        accepted_source_interval=intervals(accepted, 0, 1000),
        callback_to_enqueue=durations(callbacks, 2, 5),
        enqueue_to_dispatch=durations(beliefs, 5, 6),
        belief_accept_processing=durations(beliefs, 6, 7),
        input_drain=durations(passes, 1, 2),
        worker_pass=durations(passes, 1, 3),
        remote_processing_per_pass=stats([r[4]/1e6 for r in passes if r[7]]),
        apparent_source_to_callback_age=stats([(r[3]-r[0]*1000)/1e6 for r in callbacks]),
        middleware_received_available=sum(r[4]>0 for r in callbacks))
    remote = [list(map(int, line.split(',')[1:])) for line in text.splitlines()
              if line.startswith('[stop-remote-worker],')]
    report['remote_processing_scope'] = 'serial reconstruction and cache update'
    if remote:
        if (len(remote) != 1 or len(remote[0]) != 6 or remote[0][0] != header[1]
                or any(v < 0 for v in remote[0]) or remote[0][2] > remote[0][1]
                or remote[0][5] > remote[0][4]):
            raise ValueError('Malformed remote reconstruction summary')
        _, processed, rejected, completed, total_ns, max_ns = remote[0]
        report['remote_processing_scope'] = 'complete-set installation on state owner only'
        report['remote_reconstruction_thread'] = dict(
            scope='entire run, not filtered by the requested source-time window',
            processed_packets=processed, rejected_packets=rejected, completed_sets=completed,
            handler_mean_ms=total_ns/processed/1e6 if processed else None,
            handler_max_ms=max_ns/1e6 if processed else None,
            includes_rejected_handlers=True, includes_queue_wait=False)
    report['source_age_clock_contract'] = dict(
        definition='Pi callback system time minus PX4-converted publication timestamp',
        unit='ms', pure_transport_latency=False,
        px4_to_pc_clock_mapping_error_accounted_for=False,
        pc_to_pi_clock_offset_accounted_for=False,
        includes_ekf_sample_to_publication_interval=False)
    middleware = [r for r in callbacks if 0 < r[4] <= r[3]]
    report['middleware_to_callback_wall'] = durations(middleware, 4, 3)
    report['middleware_clock_invalid_count'] = sum(r[4] > r[3] for r in callbacks)
    report['largest_completion_gaps'] = {}
    for stage, name in [(1,'trajectory_refresh'),(2,'combination_selection')]:
        completed = sorted((r for r in stages if r[0]==stage and inside(r[1])
                            and r[8] and (stage != 1 or r[7])), key=lambda r: r[5])
        pairs = sorted(zip(completed, completed[1:]),
                       key=lambda ab: ab[1][5]-ab[0][5], reverse=True)[:10]
        result = []
        for a,b in pairs:
            left, right = a[5], b[5]
            overlap = lambda x,y: max(0, min(right,y)-max(left,x))
            busy = sum(overlap(r[1],r[3]) for r in all_passes)
            selection_thread = base.get('dump_version', 1) == 2 and stage == 2
            if selection_thread:
                busy = sum(overlap(r[4],r[5]) for r in stages if r[0] == 2)
            drain = sum(overlap(r[1],r[2]) for r in all_passes)
            matches = [r for r in all_beliefs if r[8] and r[0]==b[1] and r[7]<=b[4]]
            entry = dict(gap_ms=(right-left)/1e6, from_source_us=a[1], to_source_us=b[1],
                source_advance_ms=(b[1]-a[1])/1000,
                worker_busy_overlap_ms=busy/1e6, input_drain_overlap_ms=drain/1e6,
                other_time_ms=(right-left-busy)/1e6,
                next_stage_computation_ms=(b[5]-b[4])/1e6)
            if base.get('dump_version', 1) == 2:
                entry['execution_worker'] = 'selection' if selection_thread else 'state_owner'
            if matches:
                r=matches[-1]
                entry['next_state_enqueue_to_dispatch_ms']=(r[6]-r[5])/1e6
                entry['next_state_accept_to_stage_begin_ms']=(b[4]-r[7])/1e6
                if r[2]: entry['next_state_callback_to_stage_end_ms']=(b[5]-r[2])/1e6
                previous = [v for v in all_beliefs if v[8] and v[0]==a[1] and v[7]<=a[4]]
                if previous and previous[-1][2] and r[2]:
                    v=previous[-1]
                    entry['previous_state_callback_to_stage_end_ms']=(a[5]-v[2])/1e6
                    entry['selected_state_callback_spacing_ms']=(r[2]-v[2])/1e6
            result.append(entry)
        report['largest_completion_gaps'][name]=result
    report['limits'] = [
        'No controller tuning. Local steady clocks measure callbacks/queue/worker spans.',
        'Apparent source-to-callback age includes PX4 clock-mapping error and PC/Pi clock offset, as well as source pipeline, network and executor waiting; not calibrated one-way latency.',
        'RMW received timestamp may be unavailable or implementation-specific; unavailable is not zero delay.',
        'Other time includes idle polls, sleep/scheduling and recording overhead; not exclusively network waiting.',
        'Remote-processing per pass aggregates calls, not per-packet latency.',
        'Clock-read/storage overhead is nonzero; no hard real-time proof or PX4 actuation timestamps.']
    if base.get('dump_version', 1) == 2:
        report['limits'].append(
            'Version 2 selection gaps refer to the separate kernel worker. Input-drain overlap belongs to the state owner and is not evidence that it blocked the kernel.')
    if remote:
        report['limits'].append(
            'Remote reconstruction runs concurrently: do not add its elapsed time to owner input-drain spans. Its summary covers the entire run, including rejected packets, and excludes queue wait.')
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--start-source-us', type=int, default=0)
    p.add_argument('--end-source-us', type=int)
    a=p.parse_args()
    r=analyze(a.log.read_text(), a.start_source_us, a.end_source_us)
    a.output.write_text(json.dumps(r, indent=2)+'\n')
    print(a.output)
