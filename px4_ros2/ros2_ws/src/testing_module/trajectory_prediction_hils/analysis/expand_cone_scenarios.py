#!/usr/bin/env python3
"""Validate a cone scenario profile and emit a deterministic TSV run manifest."""

import argparse
import sys
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--case-dir", required=True)
    parser.add_argument("--list-profiles", action="store_true")
    args = parser.parse_args()

    matrix_path = Path(args.matrix)
    case_dir = Path(args.case_dir)
    with matrix_path.open(encoding="utf-8") as stream:
        matrix = yaml.safe_load(stream)

    profiles = matrix.get("profiles", {})
    if args.list_profiles:
        for name in profiles:
            print(name)
        return 0
    if args.profile not in profiles:
        available = ", ".join(profiles)
        print(
            f"ERROR: unknown profile '{args.profile}'. available: {available}",
            file=sys.stderr,
        )
        return 2

    defaults = matrix.get("defaults", {})
    profile = profiles[args.profile]
    default_repetitions = int(profile.get(
        "repetitions", defaults.get("repetitions", 1)))
    default_timeout = int(profile.get("timeout_s", defaults.get("timeout_s", 180)))
    default_pause = int(profile.get(
        "pause_after_case_s", defaults.get("pause_after_case_s", 2)))
    if default_repetitions < 1 or default_timeout < 1 or default_pause < 0:
        print("ERROR: invalid repetitions/timeout/pause in scenario matrix", file=sys.stderr)
        return 2

    rows = []
    seen_run_ids = set()
    for entry in profile.get("cases", []):
        if isinstance(entry, str):
            case_id = entry
            repetitions = default_repetitions
            timeout_s = default_timeout
            pause_s = default_pause
        elif isinstance(entry, dict):
            case_id = str(entry["id"])
            repetitions = int(entry.get("repetitions", default_repetitions))
            timeout_s = int(entry.get("timeout_s", default_timeout))
            pause_s = int(entry.get("pause_after_case_s", default_pause))
        else:
            print(f"ERROR: invalid case entry: {entry!r}", file=sys.stderr)
            return 2

        case_file = (case_dir / f"{case_id}.yaml").resolve()
        if not case_file.is_file():
            print(f"ERROR: missing case yaml: {case_file}", file=sys.stderr)
            return 2
        if repetitions < 1 or timeout_s < 1 or pause_s < 0:
            print(f"ERROR: invalid settings for {case_id}", file=sys.stderr)
            return 2

        for repetition in range(1, repetitions + 1):
            run_id = f"{case_id}_r{repetition:02d}"
            if run_id in seen_run_ids:
                print(f"ERROR: duplicate run id: {run_id}", file=sys.stderr)
                return 2
            seen_run_ids.add(run_id)
            rows.append((run_id, case_id, case_file, repetition, timeout_s, pause_s))

    if not rows:
        print(f"ERROR: profile '{args.profile}' contains no cases", file=sys.stderr)
        return 2

    print("run_id\tcase_id\tcase_file\trepetition\ttimeout_s\tpause_s")
    for row in rows:
        print("\t".join(str(value) for value in row))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
