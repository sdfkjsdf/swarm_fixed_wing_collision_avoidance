#!/usr/bin/env python3
"""Offline PX4 clock-filter audit. Reads a rosbag SQLite file; JSON to stdout.

The recorded observed offset is not ground truth. Synthetic clocks provide
known-truth regression examples separately. This tool never corrects a flight
timestamp or imports into the real-time control path.
"""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import struct


def stats(values):
    values = sorted(values)
    if not values:
        return {'count': 0}

    def quantile(p):
        index = (len(values) - 1) * p
        lower = int(index)
        return values[lower] + (values[min(lower + 1, len(values) - 1)]
                                - values[lower]) * (index - lower)

    return dict(count=len(values), min=min(values), median=quantile(.5),
                p95=quantile(.95), p99=quantile(.99), max=max(values))


def load_status(bag, vehicle):
    with sqlite3.connect(bag.resolve().as_uri() + '?mode=ro', uri=True) as db:
        topic = f'/px4_{vehicle}/fmu/out/timesync_status'
        selected = db.execute('SELECT id,type FROM topics WHERE name=?', (topic,)).fetchone()
        if not selected or selected[1] != 'px4_msgs/msg/TimesyncStatus':
            raise ValueError(f'Missing or unsupported topic: {topic}')
        rows = []
        for wall_ns, data in db.execute(
                'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                (selected[0],)):
            if len(data) != 48 or data[:4] != b'\x00\x01\x00\x00':
                raise ValueError('Expected recorded little-endian TimesyncStatus layout')
            if data[12] != 2:  # SOURCE_PROTOCOL_DDS
                raise ValueError('Expected DDS timesync, not another protocol instance')
            remote, observed, estimated, rtt = struct.unpack_from('<QqqI', data, 20)
            rows.append(dict(wall_ns=wall_ns, remote_us=remote, observed_us=observed,
                             estimated_us=estimated, rtt_us=rtt,
                             local_midpoint_us=remote + observed))
    return rows


def load_belief_receipts(bag, vehicle):
    """PC bag receipt times, keyed by both unchanged PX4 message timestamps."""
    with sqlite3.connect(bag.resolve().as_uri() + '?mode=ro', uri=True) as db:
        topic = f'/common/px4_{vehicle}/trans_estimator_trajectory_belief'
        selected = db.execute('SELECT id,type FROM topics WHERE name=?', (topic,)).fetchone()
        if not selected or selected[1] != 'px4_msgs/msg/EstimatorTrajectoryBelief':
            raise ValueError(f'Missing or unsupported topic: {topic}')
        receipts = {}
        for wall_ns, data in db.execute(
                'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                (selected[0],)):
            if len(data) < 20 or data[:4] != b'\x00\x01\x00\x00':
                raise ValueError('Expected little-endian belief timestamp pair')
            key = struct.unpack_from('<QQ', data, 4)
            receipts.setdefault(key, []).append(wall_ns)
    return receipts


def analyze_delivery(beliefs, receipts, start_ns, end_ns):
    """Exact message join, NOT a one-way network latency estimate.

    Pi callback - source = (PC bag receipt - source) + (Pi callback - PC bag).
    PX4 mapping error cancels only in the second term. PC/Pi clock offset and
    the two observers' middleware/recording waits are still uncalibrated.
    """
    joined = []
    missing = ambiguous = eligible = 0
    for row in beliefs:
        if not row[2] or not start_ns <= row[0] * 1000 <= end_ns:
            continue
        eligible += 1
        matches = receipts.get(tuple(row[:2]), [])
        if len(matches) != 1:
            missing += not matches
            ambiguous += len(matches) > 1
            continue
        joined.append((row, matches[0]))
    joined.sort(key=lambda item: item[0][2])  # Pi local steady callback order
    report = dict(
        eligible_callbacks=eligible, matched_messages=len(joined),
        missing_pc_receipt=missing, ambiguous_pc_receipt=ambiguous,
        apparent_pi_source_age_ms=stats([(r[3]-r[0]*1000)/1e6 for r,t in joined]),
        pc_bag_source_age_ms=stats([(t-r[0]*1000)/1e6 for r,t in joined]),
        pi_callback_minus_pc_bag_ms=stats([(r[3]-t)/1e6 for r,t in joined]),
        sample_to_publication_ms=stats([(r[0]-r[1])/1000 for r,t in joined]),
        identity_max_error_ns=max((abs((r[3]-r[0]*1000)
            - ((t-r[0]*1000)+(r[3]-t))) for r,t in joined), default=0),
        clock_contract=dict(
            pure_transport_latency=False,
            pc_pi_offset_corrected=False,
            pc_bag_is_transmission_time=False,
            join_key='publication timestamp AND EKF sample timestamp',
            negatives_preserved=True))
    report['largest_interobserver_gap'] = None
    if joined:
        index = max(range(len(joined)), key=lambda i: joined[i][0][3]-joined[i][1])
        row, pc_ns = joined[index]
        event = dict(source_us=row[0], pc_bag_wall_ns=pc_ns,
                     pi_callback_wall_ns=row[3],
                     apparent_pi_source_age_ms=(row[3]-row[0]*1000)/1e6,
                     pc_bag_source_age_ms=(pc_ns-row[0]*1000)/1e6,
                     pi_callback_minus_pc_bag_ms=(row[3]-pc_ns)/1e6)
        if index:
            previous, previous_pc_ns = joined[index-1]
            event.update(
                pc_receipt_interval_ms=(pc_ns-previous_pc_ns)/1e6,
                pi_callback_steady_interval_ms=(row[2]-previous[2])/1e6,
                pi_wall_minus_steady_increment_ms=(
                    (row[3]-previous[3])-(row[2]-previous[2]))/1e6)
        report['largest_interobserver_gap'] = event
    report['limits'] = [
        'All statistics use exact joined messages; missing/ambiguous matches are excluded explicitly.',
        'Do not subtract medians or maxima of separate distributions.',
        'PC bag is a parallel subscriber, not a transmission timestamp.',
        'Interobserver difference cancels the shared PX4 source timestamp, not PC/Pi clock offset.',
        'A large callback gap is real cadence variation, but does not identify Wi-Fi versus DDS versus scheduling.',
        'Timesync observed offset and RTT are not independent clock truth or a PC-to-Pi link measurement.']
    return report


def gain(sequence):
    if sequence >= 500:
        return .003
    p = 1 - math.exp(.5 * (1 - 1 / (1 - sequence / 500)))
    return .05 * (1 - p) + .003 * p


def synthetic(elapsed_time_aware, epoch_offset_us=0):
    """Known truth: -500 us/s, 500 samples at 10 ms then 200 at 1 s.

    Coefficients/initialization are shared. Only the drift's time unit changes.
    This is a model comparison, supplemented by production C++ regression tests.
    """
    now_us = 0
    estimate = skew = 0.
    errors = []
    for sequence in range(700):
        interval_us = 10_000 if sequence < 500 else 1_000_000
        now_us += interval_us
        observation = epoch_offset_us - now_us // 2000
        if sequence == 0:
            estimate = float(observation)
        else:
            a = gain(sequence)
            scale = interval_us * 1e-6 if elapsed_time_aware else 1.
            previous = estimate
            estimate = a * observation + (1 - a) * (estimate + skew * scale)
            skew = a * (estimate - previous) / scale + (1 - a) * skew
        if sequence >= 500:
            errors.append(abs(estimate - observation) / 1000)
    return stats(errors)


def analyze(rows, start_ns, end_ns):
    window = [r for r in rows if start_ns <= r['wall_ns'] <= end_ns and r['estimated_us']]
    if len(window) < 3:
        raise ValueError('Need at least three nonzero filter outputs in the operating window')
    if any(r['rtt_us'] >= 10_000 for r in window):
        raise ValueError('Window contains rejected-RTT samples; cannot assume consecutive updates')
    intervals = [b['local_midpoint_us'] - a['local_midpoint_us']
                 for a, b in zip(window, window[1:])]
    if any(value <= 0 for value in intervals):
        raise ValueError('Non-increasing local clock in the operating window')

    # Conditional legacy replay: infer initial per-update skew from two outputs.
    # Retain epoch magnitude to reproduce the original double arithmetic. Do not
    # treat replay's rounded output agreement as independent clock ground truth.
    estimate = float(window[0]['estimated_us'])
    a = .003
    skew = (window[1]['estimated_us'] - a * window[1]['observed_us']) / (1 - a) - estimate
    errors = []
    for row in window[1:]:
        previous = estimate
        estimate = a * row['observed_us'] + (1 - a) * (estimate + skew)
        skew = a * (estimate - previous) + (1 - a) * skew
        errors.append(abs(estimate - row['estimated_us']))

    elapsed = (window[-1]['local_midpoint_us'] - window[0]['local_midpoint_us']) * 1e-6
    return dict(
        recorded_status_count=len(rows), operating_status_count=len(window),
        observed_offset_change_us=window[-1]['observed_us'] - window[0]['observed_us'],
        observed_average_change_us_per_s=(window[-1]['observed_us'] - window[0]['observed_us']) / elapsed,
        operating_sample_intervals_ms=stats([dt / 1000 for dt in intervals]),
        recorded_estimated_minus_observed_ms=stats(
            [(r['estimated_us'] - r['observed_us']) / 1000 for r in window]),
        legacy_replay_error_us=stats(errors),
        synthetic_known_clock_error_ms={
            'zero_origin': dict(legacy=synthetic(False), elapsed_time=synthetic(True)),
            'epoch_origin': dict(legacy=synthetic(False, -1_600_000_000_000_000),
                                 elapsed_time=synthetic(True, -1_600_000_000_000_000))},
        limits=[
            'Recorded observed offset is not independent clock ground truth.',
            'Legacy replay assumes consecutive post-convergence updates; output error tests that assumption.',
            'Startup status is decimated. Full-rate startup filter inputs are not available in this bag.',
            'Synthetic before/after results are not a corrected flight run or measured one-way latency.',
            'Elapsed-time units repair cadence dependence, not arbitrary clock drift/noise or network jitter.',
            'No PX4/Pi clock or recorded timestamp is changed by this analysis.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--vehicle', type=int, default=0)
    parser.add_argument('--callback-log', type=Path,
                        help='Optional existing stopped timing log for exact PC/Pi message joining')
    parser.add_argument('--output', type=Path, help='Optional offline JSON report')
    args = parser.parse_args()
    summary = json.loads(args.summary.read_text())
    start = summary['actual_evaluation_start_ns']
    end = start + round(summary['common_duration_s'] * 1e9)
    result = analyze(load_status(args.bag, args.vehicle), start, end)
    if args.callback_log:
        from analyze_stopped_pipeline_timing import parse
        header, _, beliefs, _ = parse(args.callback_log.read_text())
        if header[1] != args.vehicle:
            raise ValueError('Callback log vehicle does not match selected bag topic')
        result['delivery_decomposition'] = analyze_delivery(
            beliefs, load_belief_receipts(args.bag, args.vehicle), start, end)
    output = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(output)
        print(args.output)
    else:
        print(output, end='')
