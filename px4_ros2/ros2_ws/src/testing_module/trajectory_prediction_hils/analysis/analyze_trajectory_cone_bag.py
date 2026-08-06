#!/usr/bin/env python3
"""Integrity and ground-truth analysis for trajectory-cone ROS 2 bags."""

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CHI_SQUARE_95_DF3 = 7.8147279


def read_selected_messages(bag: Path, selected_topics: set):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    messages: Dict[str, List[Tuple[int, object]]] = {
        topic: [] for topic in selected_topics if topic in topic_types
    }
    message_classes = {
        topic: get_message(topic_types[topic]) for topic in messages
    }
    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic in messages:
            messages[topic].append(
                (bag_time_ns, deserialize_message(serialized, message_classes[topic])))
    return topic_types, messages


def interpolate_ground_truth(times_us: np.ndarray, positions: np.ndarray, target_us: float):
    if target_us < times_us[0] or target_us > times_us[-1]:
        return None
    right = int(np.searchsorted(times_us, target_us, side="left"))
    if right == 0:
        return positions[0]
    if right >= len(times_us):
        return positions[-1]
    left = right - 1
    span = times_us[right] - times_us[left]
    if span <= 0:
        return positions[right]
    fraction = (target_us - times_us[left]) / span
    return positions[left] + fraction * (positions[right] - positions[left])


def analyze(args) -> int:
    namespace = args.namespace.rstrip("/")
    topics = {
        "belief": f"{namespace}/fmu/out/estimator_trajectory_belief",
        "local_position": f"{namespace}/fmu/out/vehicle_local_position_v1",
        "odometry": f"{namespace}/fmu/out/vehicle_odometry",
        "attitude": f"{namespace}/fmu/out/vehicle_attitude",
        "ground_truth_position":
            f"{namespace}/fmu/out/vehicle_local_position_groundtruth_v1",
        "ground_truth_attitude":
            f"{namespace}/fmu/out/vehicle_attitude_groundtruth",
        "vehicle_status": f"{namespace}/fmu/out/vehicle_status_v3",
        "cone": f"{namespace}/collision_estimation/trajectory_cone",
        "key_samples": f"{namespace}/collision_estimation/key_samples",
    }
    required_keys = (
        "belief", "local_position", "ground_truth_position",
        "ground_truth_attitude", "cone")
    topic_types, messages = read_selected_messages(Path(args.bag), set(topics.values()))
    counts = {key: len(messages.get(topic, [])) for key, topic in topics.items()}
    missing = [key for key in required_keys if counts[key] == 0]

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "bag": str(Path(args.bag).resolve()),
        "namespace": namespace,
        "topic_counts": counts,
        "missing_required_topics": missing,
        "discovered_topic_count": len(topic_types),
        "chi_square_95_df3": CHI_SQUARE_95_DF3,
    }
    if missing:
        (output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))
        return 2

    ground_truth_messages = messages[topics["ground_truth_position"]]
    gt_times_us = np.asarray(
        [item.timestamp for _, item in ground_truth_messages], dtype=np.float64)
    gt_positions = np.asarray(
        [[item.x, item.y, item.z] for _, item in ground_truth_messages], dtype=np.float64)
    order = np.argsort(gt_times_us)
    gt_times_us = gt_times_us[order]
    gt_positions = gt_positions[order]

    rows = []
    covariance_failures = 0
    invalid_cones = 0
    complete_cones = 0
    cone_source_timestamps = []
    npz_mean = []
    npz_covariance = []
    npz_ground_truth = []

    for _, cone in messages[topics["cone"]]:
        cone_source_timestamps.append(cone.source_timestamp)
        if not cone.valid or cone.point_count != 46:
            invalid_cones += 1
            continue
        mean = np.asarray(cone.mean_position_ned, dtype=np.float64).reshape(46, 3)
        covariance = np.asarray(
            cone.position_covariance_ned, dtype=np.float64).reshape(46, 3, 3)
        offsets = np.asarray(cone.time_offset_s, dtype=np.float64)
        gt_for_cone = []
        cone_rows = []
        complete = True

        for index in range(46):
            current_covariance = covariance[index]
            symmetry_error = float(np.max(np.abs(
                current_covariance - current_covariance.T)))
            eigenvalues = np.linalg.eigvalsh(
                0.5 * (current_covariance + current_covariance.T))
            finite = bool(np.isfinite(mean[index]).all()
                          and np.isfinite(current_covariance).all())
            psd = finite and symmetry_error <= args.symmetry_tolerance \
                and float(eigenvalues[0]) >= -args.psd_tolerance
            if not psd:
                covariance_failures += 1

            target_us = float(cone.source_timestamp) + offsets[index] * 1.0e6
            ground_truth = interpolate_ground_truth(
                gt_times_us, gt_positions, target_us)
            if ground_truth is None:
                complete = False
                break
            error = ground_truth - mean[index]
            mahalanobis_sq = float(
                error @ np.linalg.pinv(current_covariance, hermitian=True) @ error)
            cone_rows.append({
                "source_timestamp_us": int(cone.source_timestamp),
                "horizon_s": float(offsets[index]),
                "mean_n": float(mean[index, 0]),
                "mean_e": float(mean[index, 1]),
                "mean_d": float(mean[index, 2]),
                "truth_n": float(ground_truth[0]),
                "truth_e": float(ground_truth[1]),
                "truth_d": float(ground_truth[2]),
                "error_norm_m": float(np.linalg.norm(error)),
                "mahalanobis_sq": mahalanobis_sq,
                "inside_95": int(mahalanobis_sq <= CHI_SQUARE_95_DF3),
                "symmetry_error": symmetry_error,
                "min_eigenvalue": float(eigenvalues[0]),
                "max_eigenvalue": float(eigenvalues[-1]),
            })
            gt_for_cone.append(ground_truth)

        if complete:
            complete_cones += 1
            rows.extend(cone_rows)
            npz_mean.append(mean)
            npz_covariance.append(covariance)
            npz_ground_truth.append(np.asarray(gt_for_cone))

    if rows:
        coverage = float(np.mean([row["inside_95"] for row in rows]))
        endpoint_rows = [row for row in rows if math.isclose(row["horizon_s"], 4.5, abs_tol=1e-3)]
        endpoint_coverage = float(np.mean([row["inside_95"] for row in endpoint_rows]))
        mean_error = float(np.mean([row["error_norm_m"] for row in rows]))
        max_error = float(np.max([row["error_norm_m"] for row in rows]))
    else:
        coverage = endpoint_coverage = mean_error = max_error = None

    source_times = np.unique(np.asarray(cone_source_timestamps, dtype=np.float64))
    if len(source_times) > 1:
        duration_s = (source_times[-1] - source_times[0]) * 1.0e-6
        cone_rate_hz = float((len(source_times) - 1) / duration_s) if duration_s > 0 else None
    else:
        cone_rate_hz = None

    summary.update({
        "invalid_cones": invalid_cones,
        "complete_ground_truth_cones": complete_cones,
        "analyzed_points": len(rows),
        "covariance_failures": covariance_failures,
        "cone_rate_hz": cone_rate_hz,
        "empirical_coverage_all_horizons": coverage,
        "empirical_coverage_at_4_5_s": endpoint_coverage,
        "mean_position_error_m": mean_error,
        "max_position_error_m": max_error,
        "formal_coverage_pass": None,
        "formal_coverage_note":
            "Q calibration and repeated scenarios are deferred; coverage is reported, not gated.",
    })

    with (output_dir / "cone_samples.csv").open("w", newline="", encoding="utf-8") as stream:
        if rows:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    np.savez_compressed(
        output_dir / "cone_arrays.npz",
        predicted_mean=np.asarray(npz_mean),
        predicted_position_covariance=np.asarray(npz_covariance),
        ground_truth=np.asarray(npz_ground_truth),
    )
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))

    smoke_failed = invalid_cones > 0 or covariance_failures > 0 or complete_cones == 0
    return 1 if smoke_failed else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("--output", required=True, help="analysis output directory")
    parser.add_argument("--namespace", default="/px4_0")
    parser.add_argument("--symmetry-tolerance", type=float, default=1.0e-5)
    parser.add_argument("--psd-tolerance", type=float, default=1.0e-7)
    return analyze(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
