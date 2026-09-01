#!/usr/bin/env python3
"""Extract reproducible formation range/closure and velocity-noise evidence.

Only common-frame odometry is read, so the tool remains usable when an older
bag's ManeuverSelectionDecision schema no longer matches the current build.
It reports evidence; it does not silently choose or enable runtime thresholds.
"""

import argparse
import csv
import json
import math
from itertools import combinations
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


AIRCRAFT_COUNT = 5


def read_odometry(bag: Path):
    topics = {
        f"/common/px4_{vehicle}/trans_vehicle_odometry"
        for vehicle in range(AIRCRAFT_COUNT)
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = sorted(topics - topic_types.keys())
    if missing:
        raise RuntimeError("missing odometry topics: " + ", ".join(missing))
    classes = {topic: get_message(topic_types[topic]) for topic in topics}
    records = {topic: [] for topic in topics}
    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic not in records:
            continue
        message = deserialize_message(serialized, classes[topic])
        records[topic].append((
            int(bag_time_ns),
            np.asarray(message.position, dtype=np.float64),
            np.asarray(message.velocity, dtype=np.float64),
        ))
    return records


def interpolate(records, sample_hz: float, window_s: float):
    ordered_topics = sorted(records)
    raw_times = [
        np.asarray([item[0] for item in records[topic]], dtype=np.int64)
        for topic in ordered_topics
    ]
    common_end_ns = min(times[-1] for times in raw_times)
    common_start_ns = max(times[0] for times in raw_times)
    if window_s > 0.0:
        common_start_ns = max(
            common_start_ns,
            common_end_ns - int(round(window_s * 1.0e9)))
    step_ns = max(1, int(round(1.0e9 / sample_hz)))
    grid_ns = np.arange(
        common_start_ns, common_end_ns + 1, step_ns, dtype=np.int64)
    positions = np.empty(
        (AIRCRAFT_COUNT, len(grid_ns), 3), dtype=np.float64)
    velocities = np.empty_like(positions)
    grid_float = grid_ns.astype(np.float64)
    for vehicle, topic in enumerate(ordered_topics):
        times = raw_times[vehicle]
        raw_positions = np.asarray(
            [item[1] for item in records[topic]], dtype=np.float64)
        raw_velocities = np.asarray(
            [item[2] for item in records[topic]], dtype=np.float64)
        for axis in range(3):
            positions[vehicle, :, axis] = np.interp(
                grid_float, times.astype(np.float64), raw_positions[:, axis])
            velocities[vehicle, :, axis] = np.interp(
                grid_float, times.astype(np.float64), raw_velocities[:, axis])
    return grid_ns, positions, velocities


def pair_observations(grid_ns, positions, velocities, label: str):
    rows = []
    relative_velocities = []
    for first, second in combinations(range(AIRCRAFT_COUNT), 2):
        relative_position = positions[second] - positions[first]
        relative_velocity = velocities[second] - velocities[first]
        ranges = np.linalg.norm(relative_position, axis=1)
        closures = -np.sum(
            relative_position * relative_velocity, axis=1) / ranges
        relative_speeds = np.linalg.norm(relative_velocity, axis=1)
        relative_velocities.append(relative_velocity)
        for index, timestamp_ns in enumerate(grid_ns):
            rows.append({
                "label": label,
                "timestamp_ns": int(timestamp_ns),
                "aircraft_a": first,
                "aircraft_b": second,
                "range_m": float(ranges[index]),
                "closure_mps": float(closures[index]),
                "relative_speed_mps": float(relative_speeds[index]),
            })
    return rows, np.asarray(relative_velocities)


def distribution(rows, maximum_range_m: float):
    selected = [row for row in rows if row["range_m"] <= maximum_range_m]
    if not selected:
        return {"sample_count": 0}
    ranges = np.asarray([row["range_m"] for row in selected])
    closures = np.asarray([row["closure_mps"] for row in selected])
    return {
        "sample_count": len(selected),
        "maximum_range_filter_m": maximum_range_m,
        "range_quantiles_m": {
            str(q): float(np.percentile(ranges, q))
            for q in (1, 5, 50, 95, 99)
        },
        "closure_quantiles_mps": {
            str(q): float(np.percentile(closures, q))
            for q in (1, 5, 50, 95, 99)
        },
        "closure_minimum_mps": float(np.min(closures)),
        "closure_maximum_mps": float(np.max(closures)),
    }


def relative_velocity_noise(relative_velocities):
    first_difference = np.diff(relative_velocities, axis=1) / math.sqrt(2.0)
    component_sigma = np.std(first_difference.reshape(-1, 3), axis=0)
    sigma_3d = float(np.linalg.norm(component_sigma))
    return {
        "estimator": "first_difference_sigma",
        "component_sigma_mps": component_sigma.tolist(),
        "sigma_3d_mps": sigma_3d,
        "three_sigma_3d_mps": 3.0 * sigma_3d,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--normal-bag", type=Path, required=True)
    parser.add_argument("--collision-bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--normal-window-s", type=float, default=0.0)
    parser.add_argument("--collision-window-s", type=float, default=0.0)
    parser.add_argument(
        "--noise-window-s", type=float, default=0.0,
        help="optional stable tail of the normal bag used only for noise")
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--maximum-range-m", type=float, default=60.0)
    args = parser.parse_args()
    if args.sample_hz <= 0.0 or args.maximum_range_m <= 0.0:
        parser.error("sample-hz and maximum-range-m must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    all_rows = []
    summary = {
        "normal_bag": str(args.normal_bag),
        "collision_bag": str(args.collision_bag),
        "sample_hz": args.sample_hz,
        "note": "Evidence only; profile thresholds require review.",
    }
    normal_relative_velocities = None
    for label, bag, window_s in (
            ("normal_rejoin", args.normal_bag, args.normal_window_s),
            ("collision_approach", args.collision_bag,
             args.collision_window_s)):
        grid, positions, velocities = interpolate(
            read_odometry(bag), args.sample_hz, window_s)
        rows, relative_velocities = pair_observations(
            grid, positions, velocities, label)
        all_rows.extend(rows)
        summary[label] = distribution(rows, args.maximum_range_m)
        if label == "normal_rejoin":
            normal_relative_velocities = relative_velocities

    if args.noise_window_s > 0.0:
        grid, positions, velocities = interpolate(
            read_odometry(args.normal_bag), args.sample_hz,
            args.noise_window_s)
        _, normal_relative_velocities = pair_observations(
            grid, positions, velocities, "normal_noise_window")
    summary["normal_relative_velocity_noise"] = {
        "window_s": args.noise_window_s or args.normal_window_s,
        **relative_velocity_noise(normal_relative_velocities),
    }
    with (args.output_dir / "pair_range_closure.csv").open(
            "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=all_rows[0].keys())
        writer.writeheader()
        writer.writerows(all_rows)
    (args.output_dir / "calibration_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
