#!/usr/bin/env python3
"""Validate the FixedWing ZOH scenario matrix and emit a run manifest."""

import argparse
import math
import re
import sys
from pathlib import Path

import yaml


CASE_ID_PATTERN = re.compile(r"^[A-Za-z0-9_]+$")


def finite_float(value, label):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--list-profiles", action="store_true")
    args = parser.parse_args()

    matrix_path = Path(args.matrix)
    try:
        with matrix_path.open(encoding="utf-8") as stream:
            matrix = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as error:
        print(f"ERROR: cannot read matrix: {error}", file=sys.stderr)
        return 2

    profiles = matrix.get("profiles", {})
    if args.list_profiles:
        for name, profile in profiles.items():
            print(f"{name}\t{profile.get('dataset', '')}")
        return 0
    if args.profile not in profiles:
        available = ", ".join(profiles)
        print(
            f"ERROR: unknown profile '{args.profile}'. available: {available}",
            file=sys.stderr,
        )
        return 2

    defaults = matrix.get("defaults", {})
    cases = matrix.get("cases", {})
    constraints = matrix.get("constraints", {})
    profile = profiles[args.profile]
    try:
        default_repetitions = int(profile.get(
            "repetitions", defaults.get("repetitions", 1)))
        default_timeout = int(profile.get(
            "timeout_s", defaults.get("timeout_s", 180)))
        default_pause = int(profile.get(
            "pause_after_case_s", defaults.get("pause_after_case_s", 2)))
        seed_base = int(profile["seed_base"])
    except (KeyError, TypeError, ValueError) as error:
        print(f"ERROR: invalid profile defaults: {error}", file=sys.stderr)
        return 2
    dataset = str(profile.get("dataset", args.profile))
    if "\t" in dataset or "\n" in dataset:
        print("ERROR: dataset must not contain tabs/newlines", file=sys.stderr)
        return 2
    if (default_repetitions < 1 or default_timeout < 1 or default_pause < 0
            or seed_base < 0):
        print("ERROR: invalid repetitions/timeout/pause/seed_base", file=sys.stderr)
        return 2
    try:
        avoidance_height_rate = finite_float(
            constraints.get("avoidance_height_rate", 0.0),
            "constraints.avoidance_height_rate")
    except (TypeError, ValueError) as error:
        print(f"ERROR: invalid matrix constraint: {error}", file=sys.stderr)
        return 2
    if not math.isclose(avoidance_height_rate, 0.0, abs_tol=1.0e-9):
        print(
            "ERROR: operational avoidance_height_rate must remain 0.0 m/s",
            file=sys.stderr,
        )
        return 2

    rows = []
    seen_case_ids = set()
    seen_run_ids = set()
    for case_index, entry in enumerate(profile.get("cases", [])):
        if isinstance(entry, str):
            case_id = entry
            repetitions = default_repetitions
            timeout_s = default_timeout
            pause_s = default_pause
        elif isinstance(entry, dict):
            try:
                case_id = str(entry["id"])
                repetitions = int(entry.get("repetitions", default_repetitions))
                timeout_s = int(entry.get("timeout_s", default_timeout))
                pause_s = int(entry.get("pause_after_case_s", default_pause))
            except (KeyError, TypeError, ValueError) as error:
                print(f"ERROR: invalid case entry: {error}", file=sys.stderr)
                return 2
        else:
            print(f"ERROR: invalid case entry: {entry!r}", file=sys.stderr)
            return 2

        if not CASE_ID_PATTERN.fullmatch(case_id):
            print(f"ERROR: invalid case id: {case_id!r}", file=sys.stderr)
            return 2
        if case_id in seen_case_ids:
            print(f"ERROR: duplicate case in profile: {case_id}", file=sys.stderr)
            return 2
        seen_case_ids.add(case_id)
        if case_id not in cases or not isinstance(cases[case_id], dict):
            print(f"ERROR: missing case definition: {case_id}", file=sys.stderr)
            return 2
        if repetitions < 1 or timeout_s < 1 or pause_s < 0:
            print(f"ERROR: invalid run settings for {case_id}", file=sys.stderr)
            return 2

        try:
            definition = cases[case_id]
            if "height_rate" in definition:
                raise ValueError(
                    f"{case_id}.height_rate is not part of the lateral-only schema")
            v_cmd = finite_float(definition["v_cmd"], f"{case_id}.v_cmd")
            lateral_acceleration = finite_float(
                definition["lateral_acceleration"],
                f"{case_id}.lateral_acceleration")
        except (KeyError, TypeError, ValueError) as error:
            print(f"ERROR: invalid case definition: {error}", file=sys.stderr)
            return 2
        if not 10.0 <= v_cmd <= 25.0:
            print(f"ERROR: {case_id}.v_cmd outside [10, 25] m/s", file=sys.stderr)
            return 2
        if abs(lateral_acceleration) > 11.68:
            print(
                f"ERROR: {case_id}.lateral_acceleration exceeds 50 deg bank limit",
                file=sys.stderr,
            )
            return 2

        for repetition in range(1, repetitions + 1):
            run_id = f"{case_id}_r{repetition:02d}"
            if run_id in seen_run_ids:
                print(f"ERROR: duplicate run id: {run_id}", file=sys.stderr)
                return 2
            seen_run_ids.add(run_id)
            gazebo_seed = seed_base + case_index * 100 + repetition
            if gazebo_seed > 2**32 - 1:
                print(f"ERROR: Gazebo seed out of range: {gazebo_seed}", file=sys.stderr)
                return 2
            rows.append((
                run_id, case_id, dataset, repetition, gazebo_seed,
                v_cmd, lateral_acceleration, timeout_s, pause_s))

    if not rows:
        print(f"ERROR: profile '{args.profile}' contains no cases", file=sys.stderr)
        return 2

    print(
        "run_id\tcase_id\tdataset\trepetition\tgazebo_seed\t"
        "trim_speed_mps\tlateral_acceleration\ttimeout_s\tpause_s")
    for row in rows:
        print("\t".join(str(value) for value in row))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
