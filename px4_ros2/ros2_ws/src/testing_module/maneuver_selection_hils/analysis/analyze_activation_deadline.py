#!/usr/bin/env python3
"""Offline only: distinguish stage compute, command delivery and arrival proxies.

Reads existing shutdown buffers and rosbag. Never imported by flight nodes.
All deadline durations use one Pi's steady clock; cross-host differences remain
explicitly uncalibrated, and no negative values are silently clamped away.
"""
import argparse
from bisect import bisect_left, bisect_right
from collections import defaultdict
import json
from pathlib import Path
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from analyze_stopped_pipeline_timing import parse as parse_pipeline
from analyze_stopped_stage_timing import analyze as validate_stages, stats
from read_stopped_observations import parse as parse_observations


def describe(values):
    values = list(values)
    return dict(stats(values), mean_ms=float(np.mean(values)) if values else None,
                minimum_ms=float(min(values)) if values else None,
                above_50_ms=sum(v > 50 for v in values),
                negative_count=sum(v < 0 for v in values))


def read_topics(bag, wanted):
    result = defaultdict(list)
    for path in sorted(bag.glob('*.db3')):
        with sqlite3.connect(f'file:{path}?mode=ro', uri=True) as conn:
            for tid, name, kind in conn.execute('SELECT id,name,type FROM topics'):
                if name not in wanted:
                    continue
                cls = get_message(kind)
                result[name].extend((t, deserialize_message(b, cls)) for t, b in
                    conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,)))
    for records in result.values():
        records.sort(key=lambda r: r[0])
    return result


def analyze(run_dir, log):
    summary = json.loads((run_dir/'summary.json').read_text())
    start = int(summary['actual_evaluation_start_ns'])
    end = start + 200_000_000_000
    assert summary['common_duration_s'] >= 200
    text = log.read_text()
    stage_report = validate_stages(text, start//1000, end//1000)
    assert not stage_report['dropped_records']
    header, passes, beliefs, stages = parse_pipeline(text)
    assert not header[3] and not header[5]
    streams = parse_observations(text)
    traces = []
    for stream in streams:
        if stream['message_type'] != 'collision_avoidance/msg/ManeuverBudgetTrace':
            continue
        assert not stream['dropped'], 'Incomplete stopped trace'
        cls = get_message(stream['message_type'])
        traces.extend(deserialize_message(payload, cls) for _, payload in stream['records'])
    traces.sort(key=lambda m: m.steady_ns)
    assert traces and all(not m.dropped_trace_count for m in traces)
    vehicle = int(stage_report['vehicle'])
    prefix = f'/common/px4_{vehicle}/'
    topics = read_topics(Path(summary['bag']), {
        prefix+'maneuver_selection_decision', prefix+'trans_estimator_trajectory_belief',
        prefix+'trajectory_intent'})
    inside = lambda r: start <= r[1]*1000 <= end
    selected = [r for r in stages if inside(r)]
    metrics, series = {}, {}
    for stage, name in [(1, 'trajectory_compute'), (3, 'ad_monitor_compute')]:
        rows = [r for r in selected if r[0] == stage]
        series[name] = [[(r[1]*1000-start)/1e9, (r[5]-r[4])/1e6] for r in rows]
        metrics[name] = describe(y for _, y in series[name])
    # Aggregate serial ownship work in the SAME refresh/monitor pass, never
    # add independently observed maxima or concurrent selection-thread spans.
    pass_ends = [r[3] for r in passes]
    by_pass = defaultdict(dict)
    for r in stages:
        if r[0] in (1, 3):
            idx = bisect_left(pass_ends, r[5])
            assert idx < len(passes) and passes[idx][1] <= r[4]
            assert r[0] not in by_pass[idx], 'Duplicate stage in a worker pass'
            by_pass[idx][r[0]] = r
    series['refresh_through_ad'] = [[(d[3][1]*1000-start)/1e9, (d[3][5]-d[1][4])/1e6]
        for d in by_pass.values() if set(d) == {1, 3} and inside(d[3])]
    metrics['refresh_through_ad'] = describe(y for _, y in series['refresh_through_ad'])

    pubs = [m for m in traces if m.event == 4]
    pub_stamps = [m.steady_ns for m in pubs]
    monitor = defaultdict(list)
    pair_traces = [m for m in traces if m.event == 3]
    pair_stamps = [m.steady_ns for m in pair_traces]
    for r in stages:
        if r[0] == 3:
            monitor[r[1]].append(r)
    starts, unmatched = [], []
    seen = set()
    for _, d in topics[prefix+'maneuver_selection_decision']:
        if not d.activation_just_started or not start <= d.activation_timestamp_us*1000 <= end:
            continue
        key = (int(d.activation_timestamp_us), int(d.local_selection_epoch))
        if key in seen:
            continue
        seen.add(key)
        rows = []
        for row in monitor[key[0]]:
            observed = pair_traces[bisect_left(pair_stamps, row[4]):bisect_right(pair_stamps, row[5])]
            if observed and all(not m.active and int(m.epoch) == key[1] for m in observed):
                rows.append(row)
        if len(rows) != 1:
            unmatched.append(dict(key=key, reason='nonunique pre-activation AD pass', count=len(rows)))
            continue
        row = rows[0]
        next_i = bisect_left(pub_stamps, row[5])
        identity = (int(d.local_selection_epoch), int(d.ownship_candidate_id),
                    int(d.selected_candidate_input_revisions[vehicle]))
        candidates = [(k, m) for k, m in enumerate(pubs[next_i:next_i+40], start=next_i)
            if m.active and (int(m.epoch), int(m.candidate_id), int(m.input_revision)) == identity]
        if not candidates:
            unmatched.append(dict(key=key, reason='no matching ROS command'))
            continue
        k, p = candidates[0]
        # A start must correspond to an observed inactive->active publication,
        # not an arbitrary later repeated command with the same value.
        if k and pubs[k-1].active:
            unmatched.append(dict(key=key, reason='not an inactive-to-active publication transition'))
            continue
        publish_span = int(p.publish_end_wall_ns)-int(p.wall_ns)
        assert publish_span >= 0
        entry = dict(time_s=(key[0]*1000-start)/1e9, epoch=key[1], candidate_id=identity[1],
            activation_ad_m=float(d.ad_m), local_trigger=bool(d.local_activation_request_timestamp_us),
            stage_start_ns=row[4], stage_end_ns=row[5], publish_start_ns=int(p.steady_ns),
            ad_compute_ms=(row[5]-row[4])/1e6,
            ad_start_to_publish_start_ms=(p.steady_ns-row[4])/1e6,
            ad_end_to_publish_start_ms=(p.steady_ns-row[5])/1e6,
            publish_bracket_ms=publish_span/1e6,
            ad_start_to_publish_return_ms=(p.steady_ns-row[4]+publish_span)/1e6)
        refresh = by_pass[bisect_left(pass_ends, row[5])].get(1)
        if refresh:
            entry['refresh_start_to_publish_return_ms'] = (p.steady_ns-refresh[4]+publish_span)/1e6
        starts.append(entry)
    metrics['ad_start_to_publish_return'] = describe(r['ad_start_to_publish_return_ms'] for r in starts)
    metrics['refresh_start_to_publish_return'] = describe(r['refresh_start_to_publish_return_ms']
        for r in starts if 'refresh_start_to_publish_return_ms' in r)
    # Ownship state CALLBACK -> this AD pass, local monotonic clock. This is
    # neither remote candidate arrival nor a send-to-PX4 end-to-end measurement.
    accepted = sorted([r for r in beliefs if r[8] and r[2]], key=lambda r:r[7])
    accepted_ends = [r[7] for r in accepted]
    series['latest_state_callback_to_ad_end'] = []
    for r in selected:
        if r[0] != 3:
            continue
        idx = bisect_right(accepted_ends, r[4])-1
        if idx >= 0:
            b = accepted[idx]
            series['latest_state_callback_to_ad_end'].append([(r[1]*1000-start)/1e9, (r[5]-b[2])/1e6])
    metrics['latest_state_callback_to_ad_end'] = describe(y for _,y in series['latest_state_callback_to_ad_end'])
    for entry in starts:
        idx = bisect_right(accepted_ends, entry['stage_start_ns'])-1
        if idx >= 0:
            entry['latest_state_callback_to_publish_return_ms'] = (
                entry['publish_start_ns']-accepted[idx][2])/1e6+entry['publish_bracket_ms']
    metrics['latest_state_callback_to_publish_return'] = describe(r['latest_state_callback_to_publish_return_ms']
        for r in starts if 'latest_state_callback_to_publish_return_ms' in r)

    # Complete seven-candidate arrival at the PC recorder. Receiver observation
    # cadence, NOT the sender's CPU time or delivery to every participant.
    batches = defaultdict(dict)
    for stamp, m in topics[prefix+'trajectory_intent']:
        if start <= m.source_timestamp_us*1000 <= end:
            batches[(int(m.selection_epoch), int(m.source_timestamp_us))].setdefault(int(m.candidate_id), stamp)
    complete = sorted(max(d.values()) for d in batches.values() if set(d) == set(range(7)))
    series['pc_full_batch_arrival_gap'] = [[(b-start)/1e9, (b-a)/1e6] for a,b in zip(complete, complete[1:])]
    metrics['pc_full_batch_arrival_gap'] = describe(y for _,y in series['pc_full_batch_arrival_gap'])
    batch_counts = dict(observed=len(batches), complete=len(complete), incomplete=len(batches)-len(complete),
        average_complete_arrival_hz=(len(complete)-1)*1e9/(complete[-1]-complete[0]) if len(complete)>1 else None)

    # Match identical state messages. PX4 timestamp conversion cancels, but
    # independent PC/Pi clock offsets and the PC recorder's own delay DO NOT.
    pc_receipt = {}
    for stamp,m in topics[prefix+'trans_estimator_trajectory_belief']:
        pc_receipt.setdefault((int(m.timestamp), int(m.timestamp_sample)), stamp)
    communication = []
    for b in beliefs:
        if not b[2] or not start <= b[0]*1000 <= end:
            continue
        pc = pc_receipt.get((b[0], b[1]))
        if pc is None:
            continue
        communication.append(dict(time_s=(pc-start)/1e9,
            pi_callback_minus_pc_bag_ms=(b[3]-pc)/1e6,
            pi_dds_minus_pc_bag_ms=(b[4]-pc)/1e6 if b[4]>0 else None,
            pi_dds_to_callback_ms=(b[3]-b[4])/1e6 if 0<b[4]<=b[3] else None,
            apparent_source_age_ms=(b[3]-b[0]*1000)/1e6))
    communication_summary = {k: describe(r[k] for r in communication if r[k] is not None)
        for k in ['pi_callback_minus_pc_bag_ms','pi_dds_minus_pc_bag_ms',
                  'pi_dds_to_callback_ms','apparent_source_age_ms']}
    return dict(run=run_dir.name, window_s=200, vehicle=vehicle,
        source_commit='050c549', stage_report=stage_report, metrics=metrics, series=series,
        activation_events=starts, unmatched_activations=unmatched, activation_decisions=len(seen),
        batch_counts=batch_counts, communication_summary=communication_summary,
        communication_samples=communication, limitations=[
            'AD monitor includes activation and release logic; not the 4 Hz combination search.',
            'ROS publish return is not PX4 receipt, actuator response, or network delivery to all aircraft.',
            'Publication bracket uses local wall duration added to local steady duration; no cross-host subtraction.',
            'Complete batch arrivals are measured only at the PC recorder.',
            'PC/Pi arrival differences are uncalibrated signed proxies, not one-way communication latency.',
            'The full remote trajectory send->receive->AD->PX4 chain lacks matching instrumentation.',
            'No runtime changes, new packet capture or flight replay used for this analysis.'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir', type=Path, required=True)
    p.add_argument('--log', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args=p.parse_args()
    result=analyze(args.run_dir, args.log)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ['run','metrics','activation_events',
        'unmatched_activations','batch_counts','communication_summary']}, indent=2))


if __name__=='__main__':
    main()
