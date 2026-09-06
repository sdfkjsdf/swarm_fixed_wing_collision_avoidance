#!/usr/bin/env python3
"""Offline only. Accept one complete stopped-worker dump in guidance_N.log."""
import argparse
import json
from collections import Counter
from pathlib import Path


def stats(values):
    if not values:
        return {"count": 0}
    a = sorted(values)
    def percentile(p):
        x = (len(a) - 1) * p
        lo = int(x)
        return a[lo] + (a[min(lo + 1, len(a) - 1)] - a[lo]) * (x - lo)
    return dict(count=len(a), median_ms=percentile(.5), p95_ms=percentile(.95),
                p99_ms=percentile(.99), max_ms=a[-1])


def analyze(text, start_source_us=0, end_source_us=None):
    begin = end = None
    rows = []
    for line in text.splitlines():
        if not line.startswith("[stop-stage"):
            continue
        fields = line.split(",")
        values = list(map(int, fields[1:]))
        if fields[0] == "[stop-stage-begin]":
            if begin is not None or len(values) != 4 or values[0] != 1:
                raise ValueError("Expected exactly one version-1 dump")
            begin = values
        elif fields[0] == "[stop-stage]":
            if begin is None or end is not None or len(values) != 9:
                raise ValueError("Record outside dump or malformed record")
            rows.append(values)
        elif fields[0] == "[stop-stage-end]":
            if end is not None:
                raise ValueError("Duplicate footer")
            end = values
    if begin is None or end != [begin[1], begin[2]] or len(rows) != begin[2]:
        raise ValueError("Missing/truncated dump; do not report a successful measurement")
    for i, r in enumerate(rows):
        if r[0] not in (1, 2, 3) or r[4] <= 0 or r[5] < r[4]:
            raise ValueError("Invalid timing span")
        if i and r[4] < rows[i-1][5]:
            raise ValueError("Non-monotonic/overlapping stage records")
    report = dict(vehicle=begin[1], recorded_count=len(rows), dropped_records=begin[3], stages={})
    for stage, name in enumerate(("trajectory_refresh", "combination_selection", "activation_monitor"), 1):
        sample = [r for r in rows if r[0] == stage and r[1] >= start_source_us
                  and (end_source_us is None or r[1] <= end_source_us)]
        completed = [r for r in sample if r[8] and (stage != 1 or r[7])]
        intervals = [(b[5] - a[5]) / 1e6 for a, b in zip(completed, completed[1:])]
        report["stages"][name] = dict(
            computation=stats([(r[5]-r[4])/1e6 for r in sample]),
            candidate_counts=dict(Counter(r[6] for r in sample)),
            unavailable_count=sum(not r[7] for r in sample) if stage != 3 else None,
            output_not_queued_count=sum(not r[8] for r in sample),
            completion_intervals=stats(intervals),
            completion_rate_hz=1000*len(intervals)/sum(intervals) if intervals and sum(intervals)>0 else None)
    report["limits"] = ["Single-host steady clocks; not PX4 application times.",
        "Completion means stage finished and its output batch enqueued, not ROS publication.",
        "Stage spans exclude upstream input draining and most instrumentation bookkeeping.",
        "Selection unavailable means proposal_valid=false, not necessarily failed evaluation.",
        "A full buffer retains the initial window only; check dropped_records.",
        "No hard real-time or zero-overhead guarantee."]
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--log", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--start-source-us", type=int, default=0)
    p.add_argument("--end-source-us", type=int)
    a = p.parse_args()
    result = analyze(a.log.read_text(), a.start_source_us, a.end_source_us)
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
