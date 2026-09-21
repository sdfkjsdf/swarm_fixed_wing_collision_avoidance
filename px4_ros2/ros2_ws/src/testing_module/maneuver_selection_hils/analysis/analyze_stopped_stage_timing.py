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
            if begin is not None or len(values) != 4 or values[0] not in (1, 2):
                raise ValueError("Expected exactly one version-1 or version-2 dump")
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
    version = begin[0]
    allowed = (1, 2, 3) if version == 1 else (1, 2, 3, 4, 5)
    for i, r in enumerate(rows):
        if r[0] not in allowed or r[4] <= 0 or r[5] < r[4]:
            raise ValueError("Invalid timing span")
        if version == 1 and i and r[4] < rows[i-1][5]:
            raise ValueError("Non-monotonic/overlapping stage records")
    if version == 2:
        # Different threads may overlap and results may be recorded later.
        # Overlapping spans within the same owner are still an error.
        for stages in ((1, 3, 4, 5), (2,)):
            serial = sorted((r for r in rows if r[0] in stages), key=lambda r: r[4])
            if any(b[4] < a[5] for a, b in zip(serial, serial[1:])):
                raise ValueError("Overlapping records within one worker")
    report = dict(vehicle=begin[1], dump_version=version, recorded_count=len(rows),
                  dropped_records=begin[3], stages={})
    names = ("trajectory_refresh", "combination_selection", "activation_monitor")
    if version == 2:
        # Do not let old plotters silently label kernel-only time as the old
        # complete selection operation. Version 2 callers use selection_total.
        names = ("trajectory_refresh", "combination_kernel", "activation_monitor",
                 "selection_snapshot", "selection_apply")
    for stage, name in enumerate(names, 1):
        sample = sorted((r for r in rows if r[0] == stage and r[1] >= start_source_us
                         and (end_source_us is None or r[1] <= end_source_us)), key=lambda r: r[5])
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
    if version == 2:
        counters = [line.split(',')[1:] for line in text.splitlines()
                    if line.startswith('[stop-selection-worker],')]
        if len(counters) > 1:
            raise ValueError("Duplicate selection-worker counters")
        report["selection_worker_counts"] = None
        if counters:
            values = list(map(int, counters[0]))
            if len(values) != 5 or values[0] != begin[1] or any(v < 0 for v in values):
                raise ValueError("Invalid selection-worker counters")
            report["selection_worker_counts"] = dict(zip(
                ("submitted", "applied", "skipped_busy", "expired"), values[1:]))
        jobs = {}
        for r in rows:
            if r[0] not in (2, 4, 5) or r[1] < start_source_us or (end_source_us is not None and r[1] > end_source_us):
                continue
            job = jobs.setdefault((r[1], r[2]), {})
            if r[0] in job:
                raise ValueError("Duplicate stage for selection job")
            job[r[0]] = r
        complete = [j for j in jobs.values() if set(j) == {2, 4, 5}]
        for j in complete:
            if j[2][4] < j[4][5] or j[5][4] < j[2][5]:
                raise ValueError("Selection handoff order is invalid")
        report["selection_total"] = dict(
            computation=stats([sum(j[s][5]-j[s][4] for s in (4, 2, 5))/1e6 for j in complete]),
            dispatch_to_apply=stats([(j[5][5]-j[4][4])/1e6 for j in complete]),
            request_wait=stats([(j[2][4]-j[4][5])/1e6 for j in complete]),
            result_wait=stats([(j[5][4]-j[2][5])/1e6 for j in complete]),
            incomplete_job_count=len(jobs)-len(complete),
            expired_job_count=sum(not j[5][7] for j in complete))
        report["limits"] += [
            "Version 2 stage 2 is only graph/search; compare selection_total.computation with version 1 selection time.",
            "selection_total sums snapshot, kernel and application spans; queue waits are reported separately.",
            "selection_worker_counts cover the entire dump, not the selected source-time window.",
            "Parallel stage 2 timestamps precede owner handoff; completion rate is kernel completion, not proposal publication."]
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
