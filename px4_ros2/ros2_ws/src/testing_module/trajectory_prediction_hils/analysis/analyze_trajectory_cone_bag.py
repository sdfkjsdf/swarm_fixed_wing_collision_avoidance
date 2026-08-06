#!/usr/bin/env python3
"""Integrity and causal ground-truth analysis for trajectory-cone ROS 2 bags."""

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CHI_SQUARE_95_DF3 = 7.8147279
TRAJECTORY_POINT_COUNT = 46
PREDICTION_INTERVAL_COUNT = TRAJECTORY_POINT_COUNT - 1
# V_cmd [m/s], h_cmd [m], h_dot_cmd [m/s], a_lat_cmd [m/s^2].
INPUT_CHANGE_TOLERANCE = np.asarray([0.05, 0.10, 0.05, 0.05])


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


def select_ground_truth_times(messages, allow_publication_fallback: bool):
    publication = np.asarray([item.timestamp for _, item in messages], dtype=np.float64)
    sample = np.asarray([item.timestamp_sample for _, item in messages], dtype=np.float64)
    publication_span = float(np.ptp(publication)) if len(publication) else 0.0
    sample_span = float(np.ptp(sample)) if len(sample) else 0.0
    sample_unique = int(len(np.unique(sample)))
    minimum_unique = max(10, int(0.5 * len(sample)))
    sample_valid = bool(
        len(sample) > 1
        and np.isfinite(sample).all()
        and np.all(sample > 0.0)
        and sample_unique >= minimum_unique
        and sample_span >= max(1.0e6, 0.5 * publication_span)
    )
    diagnostics = {
        "ground_truth_sample_time_valid": sample_valid,
        "ground_truth_sample_unique_count": sample_unique,
        "ground_truth_sample_span_s": sample_span * 1.0e-6,
        "ground_truth_publication_span_s": publication_span * 1.0e-6,
        "ground_truth_time_source": None,
        "ground_truth_publication_minus_sample_median_s": None,
    }
    if sample_valid:
        diagnostics["ground_truth_time_source"] = "timestamp_sample"
        diagnostics["ground_truth_publication_minus_sample_median_s"] = float(
            np.median(publication - sample) * 1.0e-6)
        return sample, diagnostics
    if allow_publication_fallback:
        diagnostics["ground_truth_time_source"] = "timestamp_publication_fallback"
        return publication, diagnostics
    return None, diagnostics


def candidate_changed(lhs: np.ndarray, rhs: np.ndarray) -> bool:
    if not np.isfinite(lhs).all() or not np.isfinite(rhs).all():
        return True
    return bool(np.any(np.abs(lhs - rhs) > INPUT_CHANGE_TOLERANCE))


def analyze(args) -> int:
    namespace = args.namespace.rstrip("/")
    common_namespace = f"/common/{namespace.lstrip('/')}"
    if args.coordinate_frame == "common":
        belief_topic = f"{common_namespace}/trans_estimator_trajectory_belief"
        local_position_topic = f"{common_namespace}/trans_vehicle_local_position"
        odometry_topic = f"{common_namespace}/trans_vehicle_odometry"
        ground_truth_topic = (
            f"{common_namespace}/trans_vehicle_local_position_groundtruth")
        cone_topic = f"{common_namespace}/trans_trajectory_cone"
    else:
        belief_topic = f"{namespace}/fmu/out/estimator_trajectory_belief"
        local_position_topic = f"{namespace}/fmu/out/vehicle_local_position_v1"
        odometry_topic = f"{namespace}/fmu/out/vehicle_odometry"
        ground_truth_topic = (
            f"{namespace}/fmu/out/vehicle_local_position_groundtruth_v1")
        cone_topic = f"{namespace}/collision_estimation/trajectory_cone"
    topics = {
        "belief": belief_topic,
        "raw_belief": f"{namespace}/fmu/out/estimator_trajectory_belief",
        "local_position": local_position_topic,
        "odometry": odometry_topic,
        "attitude": f"{namespace}/fmu/out/vehicle_attitude",
        "ground_truth_position": ground_truth_topic,
        "ground_truth_attitude":
            f"{namespace}/fmu/out/vehicle_attitude_groundtruth",
        "vehicle_status": f"{namespace}/fmu/out/vehicle_status_v3",
        "cone": cone_topic,
        "raw_cone": f"{namespace}/collision_estimation/trajectory_cone",
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
        "coordinate_frame": args.coordinate_frame,
        "common_namespace": common_namespace if args.coordinate_frame == "common" else None,
        "topic_counts": counts,
        "missing_required_topics": missing,
        "discovered_topic_count": len(topic_types),
        "chi_square_95_df3": CHI_SQUARE_95_DF3,
        "evaluation_mode": "current-input ZOH with causal horizon censoring",
        "input_change_tolerance": {
            "V_cmd_mps": float(INPUT_CHANGE_TOLERANCE[0]),
            "h_cmd_m": float(INPUT_CHANGE_TOLERANCE[1]),
            "h_dot_cmd_mps": float(INPUT_CHANGE_TOLERANCE[2]),
            "a_lat_cmd_mps2": float(INPUT_CHANGE_TOLERANCE[3]),
        },
    }
    if missing:
        (output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))
        return 2

    common_contract_failures = 0
    if args.coordinate_frame == "common":
        local_messages = messages[topics["local_position"]]
        truth_messages = messages[topics["ground_truth_position"]]
        local_references = np.asarray([
            [item.ref_lat, item.ref_lon, item.ref_alt]
            for _, item in local_messages
        ], dtype=np.float64)
        truth_references = np.asarray([
            [item.ref_lat, item.ref_lon, item.ref_alt]
            for _, item in truth_messages
        ], dtype=np.float64)
        reference = np.median(local_references, axis=0)
        reference_spread = max(
            float(np.max(np.abs(local_references - reference))),
            float(np.max(np.abs(truth_references - reference))),
        )
        reference_flags_valid = all(
            item.xy_global and item.z_global
            for _, item in local_messages + truth_messages)
        reference_valid = bool(
            np.isfinite(local_references).all()
            and np.isfinite(truth_references).all()
            and reference_flags_valid
            and reference_spread <= args.common_reference_tolerance
        )
        if not reference_valid:
            common_contract_failures += 1
        summary.update({
            "common_reference_valid": reference_valid,
            "common_reference_lat": float(reference[0]),
            "common_reference_lon": float(reference[1]),
            "common_reference_alt_m": float(reference[2]),
            "common_reference_max_abs_spread": reference_spread,
        })

        raw_cones = {
            item.source_timestamp: item
            for _, item in messages.get(topics["raw_cone"], [])
        }
        transformed_cones = {
            item.source_timestamp: item
            for _, item in messages[topics["cone"]]
        }
        cone_keys = set(raw_cones).intersection(transformed_cones)
        cone_covariance_change = max((
            float(np.max(np.abs(
                np.asarray(raw_cones[key].position_covariance_ned, dtype=np.float64)
                - np.asarray(
                    transformed_cones[key].position_covariance_ned,
                    dtype=np.float64))))
            for key in cone_keys
        ), default=math.inf)

        raw_beliefs = {
            item.timestamp: item
            for _, item in messages.get(topics["raw_belief"], [])
        }
        transformed_beliefs = {
            item.timestamp: item
            for _, item in messages[topics["belief"]]
        }
        belief_keys = set(raw_beliefs).intersection(transformed_beliefs)
        belief_covariance_change = max((
            float(np.max(np.abs(
                np.asarray(
                    raw_beliefs[key].covariance_upper_triangle,
                    dtype=np.float64)
                - np.asarray(
                    transformed_beliefs[key].covariance_upper_triangle,
                    dtype=np.float64))))
            for key in belief_keys
        ), default=math.inf)
        covariance_translation_valid = bool(
            cone_covariance_change <= args.covariance_translation_tolerance
            and belief_covariance_change <= args.covariance_translation_tolerance)
        if not covariance_translation_valid:
            common_contract_failures += 1
        summary.update({
            "common_matched_cone_count": len(cone_keys),
            "common_matched_belief_count": len(belief_keys),
            "common_cone_covariance_max_abs_change": cone_covariance_change,
            "common_belief_covariance_max_abs_change": belief_covariance_change,
            "common_translation_covariance_valid": covariance_translation_valid,
            "common_contract_failures": common_contract_failures,
        })

    ground_truth_messages = messages[topics["ground_truth_position"]]
    gt_times_us, time_diagnostics = select_ground_truth_times(
        ground_truth_messages, args.allow_publication_time_fallback)
    summary.update(time_diagnostics)
    if gt_times_us is None:
        summary["analysis_error"] = (
            "vehicle_local_position_groundtruth.timestamp_sample is invalid; "
            "rebuild PX4 with HIL sample-time propagation and record a new bag"
        )
        (output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))
        return 2

    gt_positions = np.asarray(
        [[item.x, item.y, item.z] for _, item in ground_truth_messages], dtype=np.float64)
    order = np.argsort(gt_times_us, kind="stable")
    gt_times_us = gt_times_us[order]
    gt_positions = gt_positions[order]
    # Keep the newest value for duplicate sample timestamps.
    _, reverse_indices = np.unique(gt_times_us[::-1], return_index=True)
    keep = np.sort(len(gt_times_us) - 1 - reverse_indices)
    gt_times_us = gt_times_us[keep]
    gt_positions = gt_positions[keep]

    cone_records = []
    invalid_cones = 0
    non_zoh_cones = 0
    for _, cone in messages[topics["cone"]]:
        if not cone.valid or cone.point_count != TRAJECTORY_POINT_COUNT:
            invalid_cones += 1
            continue
        prediction_inputs = np.asarray(
            cone.prediction_inputs, dtype=np.float64).reshape(
                PREDICTION_INTERVAL_COUNT, 4)
        candidate = prediction_inputs[0].copy()
        zoh = bool(
            np.isfinite(prediction_inputs).all()
            and np.all(np.abs(prediction_inputs - candidate) <= INPUT_CHANGE_TOLERANCE)
        )
        if not zoh:
            non_zoh_cones += 1
        cone_records.append({
            "message": cone,
            "source_timestamp": int(cone.source_timestamp),
            "candidate": candidate,
            "zoh": zoh,
        })
    cone_records.sort(key=lambda record: record["source_timestamp"])

    # The first subsequently published cone with another current command marks
    # the end of the interval in which the original ZOH candidate was applied.
    for index, record in enumerate(cone_records):
        valid_until_us: Optional[int] = None
        for later in cone_records[index + 1:]:
            if candidate_changed(record["candidate"], later["candidate"]):
                valid_until_us = later["source_timestamp"]
                break
        record["valid_until_us"] = valid_until_us

    rows = []
    covariance_failures = 0
    complete_cones = 0
    partial_cones = 0
    causal_partial_cones = 0
    ground_truth_partial_cones = 0
    censored_points = 0
    npz_mean = []
    npz_covariance = []
    npz_ground_truth = []
    npz_point_count = []
    npz_source_timestamp = []
    npz_candidate_inputs = []

    for record in cone_records:
        cone = record["message"]
        mean = np.asarray(cone.mean_position_ned, dtype=np.float64).reshape(
            TRAJECTORY_POINT_COUNT, 3)
        covariance = np.asarray(
            cone.position_covariance_ned, dtype=np.float64).reshape(
                TRAJECTORY_POINT_COUNT, 3, 3)
        offsets = np.asarray(cone.time_offset_s, dtype=np.float64)
        ground_truth_array = np.full((TRAJECTORY_POINT_COUNT, 3), np.nan)
        cone_rows = []
        causal_limit_s = 4.5
        if record["valid_until_us"] is not None:
            causal_limit_s = max(
                0.0,
                (record["valid_until_us"] - record["source_timestamp"]) * 1.0e-6,
            )
            causal_targets_us = (
                float(cone.source_timestamp) + offsets * 1.0e6)
            censored_points += int(np.count_nonzero(
                causal_targets_us >= record["valid_until_us"]))

        partial_reason = None

        for index in range(TRAJECTORY_POINT_COUNT):
            target_us = float(cone.source_timestamp) + offsets[index] * 1.0e6
            if (record["valid_until_us"] is not None
                    and target_us >= record["valid_until_us"]):
                partial_reason = "input_change"
                break

            current_covariance = covariance[index]
            symmetry_error = float(np.max(np.abs(
                current_covariance - current_covariance.T)))
            symmetric_covariance = 0.5 * (
                current_covariance + current_covariance.T)
            eigenvalues = np.linalg.eigvalsh(symmetric_covariance)
            finite = bool(np.isfinite(mean[index]).all()
                          and np.isfinite(current_covariance).all())
            psd = finite and symmetry_error <= args.symmetry_tolerance \
                and float(eigenvalues[0]) >= -args.psd_tolerance
            if not psd:
                covariance_failures += 1

            ground_truth = interpolate_ground_truth(
                gt_times_us, gt_positions, target_us)
            if ground_truth is None:
                partial_reason = "ground_truth_boundary"
                break
            error = ground_truth - mean[index]
            mahalanobis_sq = float(
                error @ np.linalg.pinv(current_covariance, hermitian=True) @ error)
            candidate = record["candidate"]
            cone_rows.append({
                "source_timestamp_us": int(cone.source_timestamp),
                "horizon_s": float(offsets[index]),
                "causal_horizon_limit_s": causal_limit_s,
                "candidate_V_cmd": float(candidate[0]),
                "candidate_h_cmd": float(candidate[1]),
                "candidate_h_dot_cmd": float(candidate[2]),
                "candidate_a_lat_cmd": float(candidate[3]),
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
            ground_truth_array[index] = ground_truth

        point_count = len(cone_rows)
        if point_count == 0:
            continue
        rows.extend(cone_rows)
        if point_count == TRAJECTORY_POINT_COUNT:
            complete_cones += 1
        else:
            partial_cones += 1
            if partial_reason == "input_change":
                causal_partial_cones += 1
            elif partial_reason == "ground_truth_boundary":
                ground_truth_partial_cones += 1
        npz_mean.append(mean)
        npz_covariance.append(covariance)
        npz_ground_truth.append(ground_truth_array)
        npz_point_count.append(point_count)
        npz_source_timestamp.append(record["source_timestamp"])
        npz_candidate_inputs.append(record["candidate"])

    if rows:
        coverage = float(np.mean([row["inside_95"] for row in rows]))
        endpoint_rows = [
            row for row in rows
            if math.isclose(row["horizon_s"], 4.5, abs_tol=1e-3)
        ]
        endpoint_coverage = (
            float(np.mean([row["inside_95"] for row in endpoint_rows]))
            if endpoint_rows else None
        )
        mean_error = float(np.mean([row["error_norm_m"] for row in rows]))
        max_error = float(np.max([row["error_norm_m"] for row in rows]))
    else:
        coverage = endpoint_coverage = mean_error = max_error = None

    source_times = np.unique(np.asarray(
        [record["source_timestamp"] for record in cone_records], dtype=np.float64))
    if len(source_times) > 1:
        duration_s = (source_times[-1] - source_times[0]) * 1.0e-6
        cone_rate_hz = float((len(source_times) - 1) / duration_s) if duration_s > 0 else None
    else:
        cone_rate_hz = None

    summary.update({
        "invalid_cones": invalid_cones,
        "non_zoh_cones": non_zoh_cones,
        "complete_ground_truth_cones": complete_cones,
        "causal_full_horizon_cones": complete_cones,
        "partial_horizon_cones": partial_cones,
        "causal_partial_horizon_cones": causal_partial_cones,
        "ground_truth_partial_horizon_cones": ground_truth_partial_cones,
        "causally_censored_points": censored_points,
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
        causal_point_count=np.asarray(npz_point_count, dtype=np.int32),
        source_timestamp_us=np.asarray(npz_source_timestamp, dtype=np.int64),
        candidate_inputs=np.asarray(npz_candidate_inputs),
    )
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))

    smoke_failed = (
        common_contract_failures > 0
        or invalid_cones > 0
        or non_zoh_cones > 0
        or covariance_failures > 0
        or complete_cones == 0
    )
    return 1 if smoke_failed else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("--output", required=True, help="analysis output directory")
    parser.add_argument("--namespace", default="/px4_0")
    parser.add_argument(
        "--coordinate-frame",
        choices=("common", "local"),
        default="common",
        help="formal analysis uses common; local is retained for legacy diagnostics",
    )
    parser.add_argument("--symmetry-tolerance", type=float, default=1.0e-5)
    parser.add_argument("--psd-tolerance", type=float, default=1.0e-7)
    parser.add_argument("--common-reference-tolerance", type=float, default=1.0e-5)
    parser.add_argument(
        "--covariance-translation-tolerance", type=float, default=1.0e-12)
    parser.add_argument(
        "--allow-publication-time-fallback",
        action="store_true",
        help="legacy diagnostic only; formal runs require timestamp_sample",
    )
    return analyze(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
