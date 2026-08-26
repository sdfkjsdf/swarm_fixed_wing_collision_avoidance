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


def quaternion_to_roll(quaternion) -> float:
    q_w, q_x, q_y, q_z = (float(value) for value in quaternion)
    sin_roll = 2.0 * (q_w * q_x + q_y * q_z)
    cos_roll = 1.0 - 2.0 * (q_x * q_x + q_y * q_y)
    return math.atan2(sin_roll, cos_roll)


def vector_error_statistics(vectors: List[np.ndarray]):
    """Return JSON-safe vector and norm statistics for NED errors."""
    if not vectors:
        return {
            "count": 0,
            "mean_vector_ned_m": None,
            "median_vector_ned_m": None,
            "mean_norm_m": None,
            "median_norm_m": None,
            "percentile_95_norm_m": None,
            "max_norm_m": None,
        }
    values = np.asarray(vectors, dtype=np.float64)
    norms = np.linalg.norm(values, axis=1)
    return {
        "count": int(len(values)),
        "mean_vector_ned_m": np.mean(values, axis=0).tolist(),
        "median_vector_ned_m": np.median(values, axis=0).tolist(),
        "mean_norm_m": float(np.mean(norms)),
        "median_norm_m": float(np.median(norms)),
        "percentile_95_norm_m": float(np.percentile(norms, 95)),
        "max_norm_m": float(np.max(norms)),
    }


def kinematic_position_velocity_error(times_us, positions, velocities):
    """Compare finite-difference position rate with the reported NED velocity."""
    times_us = np.asarray(times_us, dtype=np.float64)
    positions = np.asarray(positions, dtype=np.float64)
    velocities = np.asarray(velocities, dtype=np.float64)
    if len(times_us) < 2:
        return vector_error_statistics([])
    order = np.argsort(times_us, kind="stable")
    times_us = times_us[order]
    positions = positions[order]
    velocities = velocities[order]
    dt_s = np.diff(times_us) * 1.0e-6
    valid = np.isfinite(dt_s) & (dt_s > 1.0e-4) & (dt_s < 0.2)
    position_rate = np.diff(positions, axis=0)[valid] / dt_s[valid, None]
    velocity_midpoint = 0.5 * (
        velocities[:-1][valid] + velocities[1:][valid])
    return vector_error_statistics(list(position_rate - velocity_midpoint))


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
        "gps_position": f"{namespace}/fmu/out/vehicle_gps_position",
        "airspeed": f"{namespace}/fmu/out/airspeed_validated_v1",
        "attitude": f"{namespace}/fmu/out/vehicle_attitude",
        "ground_truth_position": ground_truth_topic,
        "ground_truth_attitude":
            f"{namespace}/fmu/out/vehicle_attitude_groundtruth",
        "vehicle_status": f"{namespace}/fmu/out/vehicle_status_v3",
        "cone": cone_topic,
        "raw_cone": f"{namespace}/collision_estimation/trajectory_cone",
        "key_samples": f"{namespace}/collision_estimation/key_samples",
        "prediction_debug":
            f"{namespace}/testing/trajectory_prediction_debug",
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
        "analysis_version": "propagation_alignment_v1",
        "time_contract": (
            "cone point 0 and ground truth use cone.source_timestamp; "
            "later points use source_timestamp + message time_offset_s"),
        "input_change_tolerance": {
            "V_cmd_mps": float(INPUT_CHANGE_TOLERANCE[0]),
            "h_cmd_m": float(INPUT_CHANGE_TOLERANCE[1]),
            "h_dot_cmd_mps": float(INPUT_CHANGE_TOLERANCE[2]),
            "a_lat_cmd_mps2": float(INPUT_CHANGE_TOLERANCE[3]),
        },
    }

    debug_messages = [
        item for _, item in messages.get(topics["prediction_debug"], [])
    ]
    if debug_messages:
        cone_source_timestamps = {
            int(item.source_timestamp)
            for _, item in messages.get(topics["cone"], [])
        }
        debug_non_zoh = 0
        debug_nonzero_height_rate = 0
        for item in debug_messages:
            debug_inputs = np.asarray(
                item.prediction_inputs, dtype=np.float64).reshape(
                    PREDICTION_INTERVAL_COUNT, 4)
            if not (
                np.isfinite(debug_inputs).all()
                and np.all(
                    np.abs(debug_inputs - debug_inputs[0])
                    <= INPUT_CHANGE_TOLERANCE)
            ):
                debug_non_zoh += 1
            if np.any(np.abs(debug_inputs[:, 2]) > 1.0e-9):
                debug_nonzero_height_rate += 1
        valid_count = sum(bool(item.valid) for item in debug_messages)
        point_count_ok = sum(
            int(item.point_count) == TRAJECTORY_POINT_COUNT
            for item in debug_messages)
        epoch_self_consistent = sum(
            int(item.cone_epoch_timestamp) == int(item.source_timestamp)
            for item in debug_messages)
        epoch_matched_to_cone = sum(
            int(item.cone_epoch_timestamp) in cone_source_timestamps
            for item in debug_messages)
        applied_before_source = sum(
            0 < int(item.applied_input_timestamp) <= int(item.source_timestamp)
            for item in debug_messages)
        debug_count = len(debug_messages)
        summary["propagation_test_contract"] = {
            "present": True,
            "case_ids": sorted({item.case_id for item in debug_messages}),
            "message_count": debug_count,
            "valid_message_count": valid_count,
            "point_count_46_count": point_count_ok,
            "zoh_message_count": debug_count - debug_non_zoh,
            "lateral_only_message_count": (
                debug_count - debug_nonzero_height_rate),
            "epoch_self_consistent_count": epoch_self_consistent,
            "epoch_matched_to_cone_count": epoch_matched_to_cone,
            "input_applied_before_source_count": applied_before_source,
            "pass": bool(
                valid_count == debug_count
                and point_count_ok == debug_count
                and debug_non_zoh == 0
                and debug_nonzero_height_rate == 0
                and epoch_self_consistent == debug_count
                and epoch_matched_to_cone == debug_count
                and applied_before_source == debug_count),
        }
    else:
        summary["propagation_test_contract"] = {
            "present": False,
            "pass": None,
        }

    summary["trim_gate"] = {
        "required": bool(args.require_propagation_contract),
        "pass": None,
    }
    if debug_messages:
        applied_timestamps = sorted({
            int(item.applied_input_timestamp) for item in debug_messages
            if int(item.applied_input_timestamp) > 0
        })
        first_inputs = np.asarray(
            debug_messages[0].prediction_inputs,
            dtype=np.float64).reshape(PREDICTION_INTERVAL_COUNT, 4)[0]
        if applied_timestamps:
            applied_us = applied_timestamps[0]
            window_start_us = applied_us - int(args.trim_hold_s * 1.0e6)
            airspeed_samples = [
                (int(item.timestamp), float(item.calibrated_airspeed_m_s))
                for _, item in messages.get(topics["airspeed"], [])
                if window_start_us <= int(item.timestamp) <= applied_us
                and math.isfinite(float(item.calibrated_airspeed_m_s))
            ]
            vertical_speed_samples = [
                (int(item.timestamp), float(item.vz))
                for _, item in messages.get(topics["local_position"], [])
                if window_start_us <= int(item.timestamp) <= applied_us
                and math.isfinite(float(item.vz))
            ]
            roll_samples = [
                (int(item.timestamp), quaternion_to_roll(item.q))
                for _, item in messages.get(topics["attitude"], [])
                if window_start_us <= int(item.timestamp) <= applied_us
            ]
            max_age_us = int(args.trim_max_sample_age_s * 1.0e6)

            def window_valid(samples):
                return bool(
                    samples
                    and samples[0][0] <= window_start_us + max_age_us
                    and samples[-1][0] >= applied_us - max_age_us)

            target_airspeed = float(first_inputs[0])
            max_airspeed_error = (
                max(abs(value - target_airspeed)
                    for _, value in airspeed_samples)
                if airspeed_samples else None)
            max_vertical_speed = (
                max(abs(value) for _, value in vertical_speed_samples)
                if vertical_speed_samples else None)
            max_roll_rad = (
                max(abs(value) for _, value in roll_samples)
                if roll_samples else None)
            window_coverage_valid = bool(
                window_valid(airspeed_samples)
                and window_valid(vertical_speed_samples)
                and window_valid(roll_samples))
            trim_pass = bool(
                len(applied_timestamps) == 1
                and window_coverage_valid
                and max_airspeed_error is not None
                and max_airspeed_error <= args.trim_airspeed_tolerance_mps
                and max_vertical_speed is not None
                and max_vertical_speed
                    <= args.trim_vertical_speed_tolerance_mps
                and max_roll_rad is not None
                and max_roll_rad
                    <= math.radians(args.trim_roll_tolerance_deg))
            summary["trim_gate"] = {
                "required": bool(args.require_propagation_contract),
                "pass": trim_pass,
                "applied_input_timestamp_us": applied_us,
                "unique_applied_input_timestamp_count": len(applied_timestamps),
                "hold_s": float(args.trim_hold_s),
                "window_coverage_valid": window_coverage_valid,
                "target_calibrated_airspeed_mps": target_airspeed,
                "max_airspeed_error_mps": max_airspeed_error,
                "airspeed_tolerance_mps": float(
                    args.trim_airspeed_tolerance_mps),
                "max_abs_vertical_speed_mps": max_vertical_speed,
                "vertical_speed_tolerance_mps": float(
                    args.trim_vertical_speed_tolerance_mps),
                "max_abs_roll_deg": (
                    math.degrees(max_roll_rad)
                    if max_roll_rad is not None else None),
                "roll_tolerance_deg": float(args.trim_roll_tolerance_deg),
                "sample_counts": {
                    "airspeed": len(airspeed_samples),
                    "vertical_speed": len(vertical_speed_samples),
                    "roll": len(roll_samples),
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

    gt_velocities_all = np.asarray(
        [[item.vx, item.vy, item.vz] for _, item in ground_truth_messages],
        dtype=np.float64)[order][keep]
    ground_truth_kinematic_error = kinematic_position_velocity_error(
        gt_times_us, gt_positions, gt_velocities_all)

    belief_messages_all = [item for _, item in messages[topics["belief"]]]
    belief_kinematic_error = kinematic_position_velocity_error(
        [item.timestamp_sample for item in belief_messages_all],
        [item.position for item in belief_messages_all],
        [item.velocity for item in belief_messages_all],
    )

    current_local_position_errors = []
    local_position_messages = [
        item for _, item in messages[topics["local_position"]]
    ]
    local_times_us = np.asarray([
        item.timestamp_sample for item in local_position_messages
    ], dtype=np.float64)
    local_positions = np.asarray([
        [item.x, item.y, item.z] for item in local_position_messages
    ], dtype=np.float64)
    local_order = np.argsort(local_times_us, kind="stable")
    local_times_us = local_times_us[local_order]
    local_positions = local_positions[local_order]
    for local_position in local_position_messages:
        truth = interpolate_ground_truth(
            gt_times_us, gt_positions, float(local_position.timestamp_sample))
        if truth is not None:
            current_local_position_errors.append(
                truth - np.asarray([
                    local_position.x, local_position.y, local_position.z
                ], dtype=np.float64))

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

    beliefs_by_timestamp = {
        int(item.timestamp): item for _, item in messages[topics["belief"]]
    }

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
    npz_time_offsets = []
    initial_alignment_rows = []
    cone_evaluation_rows = []

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
        initial_ground_truth = None
        initial_predicted_position = mean[0].copy()
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

        # Exact horizon-zero decomposition:
        # truth(now) - cone(0)
        #   = [truth(fusion) - belief(fusion)]
        #   + [(truth(now) - truth(fusion)) - (cone(0) - belief(fusion))].
        # This separates the EKF mean error from delay-compensation error without
        # attributing either component to process noise Q.
        belief = beliefs_by_timestamp.get(record["source_timestamp"])
        if belief is not None:
            publication_time_us = float(record["source_timestamp"])
            timestamp_delay_s = (
                publication_time_us - float(belief.timestamp_sample)) * 1.0e-6
            total_source_delay_s = float(cone.source_delay_s)
            additional_output_delay_s = max(
                0.0, total_source_delay_s - timestamp_delay_s)
            fusion_time_us = (
                publication_time_us - total_source_delay_s * 1.0e6)
            truth_at_fusion = interpolate_ground_truth(
                gt_times_us, gt_positions, fusion_time_us)
            truth_at_publication = interpolate_ground_truth(
                gt_times_us, gt_positions, publication_time_us)
            if truth_at_fusion is not None and truth_at_publication is not None:
                belief_position = np.asarray(belief.position, dtype=np.float64)
                predicted_delay_motion = mean[0] - belief_position
                truth_delay_motion = truth_at_publication - truth_at_fusion
                fusion_error = truth_at_fusion - belief_position
                delay_compensation_error = (
                    truth_delay_motion - predicted_delay_motion)
                horizon_zero_error = truth_at_publication - mean[0]
                reconstructed_error = fusion_error + delay_compensation_error
                closure_error = horizon_zero_error - reconstructed_error
                current_ekf_position = interpolate_ground_truth(
                    local_times_us, local_positions, publication_time_us)
                cone_to_current_ekf = (
                    current_ekf_position - mean[0]
                    if current_ekf_position is not None else np.full(3, np.nan))
                initial_alignment_rows.append({
                    "source_timestamp_us": int(record["source_timestamp"]),
                    "source_timestamp_sample_us": int(belief.timestamp_sample),
                    "effective_fusion_timestamp_us": int(round(fusion_time_us)),
                    "timestamp_delay_s": timestamp_delay_s,
                    "additional_output_delay_s": additional_output_delay_s,
                    "source_delay_s": total_source_delay_s,
                    "fusion_error_n": float(fusion_error[0]),
                    "fusion_error_e": float(fusion_error[1]),
                    "fusion_error_d": float(fusion_error[2]),
                    "fusion_error_norm_m": float(np.linalg.norm(fusion_error)),
                    "truth_delay_motion_n": float(truth_delay_motion[0]),
                    "truth_delay_motion_e": float(truth_delay_motion[1]),
                    "truth_delay_motion_d": float(truth_delay_motion[2]),
                    "truth_delay_motion_norm_m": float(np.linalg.norm(truth_delay_motion)),
                    "predicted_delay_motion_n": float(predicted_delay_motion[0]),
                    "predicted_delay_motion_e": float(predicted_delay_motion[1]),
                    "predicted_delay_motion_d": float(predicted_delay_motion[2]),
                    "predicted_delay_motion_norm_m": float(
                        np.linalg.norm(predicted_delay_motion)),
                    "delay_compensation_error_n": float(delay_compensation_error[0]),
                    "delay_compensation_error_e": float(delay_compensation_error[1]),
                    "delay_compensation_error_d": float(delay_compensation_error[2]),
                    "delay_compensation_error_norm_m": float(
                        np.linalg.norm(delay_compensation_error)),
                    "horizon_zero_error_n": float(horizon_zero_error[0]),
                    "horizon_zero_error_e": float(horizon_zero_error[1]),
                    "horizon_zero_error_d": float(horizon_zero_error[2]),
                    "horizon_zero_error_norm_m": float(
                        np.linalg.norm(horizon_zero_error)),
                    "cone_to_current_ekf_n": float(cone_to_current_ekf[0]),
                    "cone_to_current_ekf_e": float(cone_to_current_ekf[1]),
                    "cone_to_current_ekf_d": float(cone_to_current_ekf[2]),
                    "cone_to_current_ekf_norm_m": float(
                        np.linalg.norm(cone_to_current_ekf)),
                    "decomposition_closure_norm_m": float(np.linalg.norm(closure_error)),
                })

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
            if initial_ground_truth is None:
                initial_ground_truth = ground_truth.copy()
            error = ground_truth - mean[index]
            propagation_error = (
                (ground_truth - initial_ground_truth)
                - (mean[index] - initial_predicted_position))
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
                "error_n": float(error[0]),
                "error_e": float(error[1]),
                "error_d": float(error[2]),
                "error_norm_m": float(np.linalg.norm(error)),
                "propagation_error_n": float(propagation_error[0]),
                "propagation_error_e": float(propagation_error[1]),
                "propagation_error_d": float(propagation_error[2]),
                "propagation_error_norm_m": float(
                    np.linalg.norm(propagation_error)),
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
        inside_flags = [bool(row["inside_95"]) for row in cone_rows]
        first_exit_index = next(
            (index for index, inside in enumerate(inside_flags) if not inside), None)
        first_exit_horizon_s = (
            float(cone_rows[first_exit_index]["horizon_s"])
            if first_exit_index is not None else None)
        full_horizon = point_count == TRAJECTORY_POINT_COUNT
        cone_evaluation_rows.append({
            "source_timestamp_us": int(record["source_timestamp"]),
            "analyzed_point_count": point_count,
            "evaluated_horizon_s": float(cone_rows[-1]["horizon_s"]),
            "partial_reason": partial_reason or "none",
            "all_analyzed_points_inside_95": int(all(inside_flags)),
            "inside_fraction": float(np.mean(inside_flags)),
            "first_exit_horizon_s": first_exit_horizon_s,
            "max_mahalanobis_sq": float(max(
                row["mahalanobis_sq"] for row in cone_rows)),
            "full_4_5s_horizon": int(full_horizon),
            "full_4_5s_trajectory_inside_95": (
                int(all(inside_flags)) if full_horizon else None),
            "horizon_zero_alignment_error_m": float(
                cone_rows[0]["error_norm_m"]),
            "terminal_absolute_error_m": float(
                cone_rows[-1]["error_norm_m"]),
            "terminal_propagation_error_m": float(
                cone_rows[-1]["propagation_error_norm_m"]),
        })
        npz_mean.append(mean)
        npz_covariance.append(covariance)
        npz_ground_truth.append(ground_truth_array)
        npz_point_count.append(point_count)
        npz_source_timestamp.append(record["source_timestamp"])
        npz_candidate_inputs.append(record["candidate"])
        npz_time_offsets.append(offsets)

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

    horizon_statistics = []
    for horizon in sorted({round(row["horizon_s"], 6) for row in rows}):
        horizon_rows = [
            row for row in rows if math.isclose(
                row["horizon_s"], horizon, abs_tol=1.0e-6)
        ]
        errors = np.asarray(
            [row["error_norm_m"] for row in horizon_rows], dtype=np.float64)
        mahalanobis = np.asarray(
            [row["mahalanobis_sq"] for row in horizon_rows], dtype=np.float64)
        propagation_errors = np.asarray([
            row["propagation_error_norm_m"] for row in horizon_rows
        ], dtype=np.float64)
        horizon_statistics.append({
            "horizon_s": horizon,
            "sample_count": len(horizon_rows),
            "empirical_coverage": float(np.mean([
                row["inside_95"] for row in horizon_rows])),
            "mean_error_m": float(np.mean(errors)),
            "median_error_m": float(np.median(errors)),
            "percentile_95_error_m": float(np.percentile(errors, 95)),
            "mean_propagation_error_m": float(np.mean(propagation_errors)),
            "median_propagation_error_m": float(np.median(propagation_errors)),
            "percentile_95_propagation_error_m": float(
                np.percentile(propagation_errors, 95)),
            "mean_mahalanobis_sq": float(np.mean(mahalanobis)),
            "median_mahalanobis_sq": float(np.median(mahalanobis)),
            "percentile_95_mahalanobis_sq": float(np.percentile(mahalanobis, 95)),
        })

    causal_containment = (
        float(np.mean([
            row["all_analyzed_points_inside_95"]
            for row in cone_evaluation_rows
        ])) if cone_evaluation_rows else None)
    full_evaluations = [
        row for row in cone_evaluation_rows if row["full_4_5s_horizon"]
    ]
    full_containment = (
        float(np.mean([
            row["full_4_5s_trajectory_inside_95"] for row in full_evaluations
        ])) if full_evaluations else None)
    first_exit_horizons = [
        row["first_exit_horizon_s"] for row in cone_evaluation_rows
        if row["first_exit_horizon_s"] is not None
    ]
    full_endpoint_propagation_errors = [np.asarray([
        row["propagation_error_n"], row["propagation_error_e"],
        row["propagation_error_d"]
    ]) for row in rows if math.isclose(
        row["horizon_s"], 4.5, abs_tol=1.0e-3)]

    fusion_errors = [np.asarray([
        row["fusion_error_n"], row["fusion_error_e"], row["fusion_error_d"]
    ]) for row in initial_alignment_rows]
    truth_delay_motions = [np.asarray([
        row["truth_delay_motion_n"], row["truth_delay_motion_e"],
        row["truth_delay_motion_d"]
    ]) for row in initial_alignment_rows]
    predicted_delay_motions = [np.asarray([
        row["predicted_delay_motion_n"], row["predicted_delay_motion_e"],
        row["predicted_delay_motion_d"]
    ]) for row in initial_alignment_rows]
    delay_compensation_errors = [np.asarray([
        row["delay_compensation_error_n"], row["delay_compensation_error_e"],
        row["delay_compensation_error_d"]
    ]) for row in initial_alignment_rows]
    horizon_zero_errors = [np.asarray([
        row["horizon_zero_error_n"], row["horizon_zero_error_e"],
        row["horizon_zero_error_d"]
    ]) for row in initial_alignment_rows]
    cone_to_current_ekf_errors = [np.asarray([
        row["cone_to_current_ekf_n"], row["cone_to_current_ekf_e"],
        row["cone_to_current_ekf_d"]
    ]) for row in initial_alignment_rows
        if np.isfinite(row["cone_to_current_ekf_norm_m"])]
    initial_alignment = {
        "matched_cone_belief_count": len(initial_alignment_rows),
        "source_delay_median_s": (
            float(np.median([
                row["source_delay_s"] for row in initial_alignment_rows
            ])) if initial_alignment_rows else None),
        "timestamp_delay_median_s": (
            float(np.median([
                row["timestamp_delay_s"] for row in initial_alignment_rows
            ])) if initial_alignment_rows else None),
        "additional_output_delay_median_s": (
            float(np.median([
                row["additional_output_delay_s"]
                for row in initial_alignment_rows
            ])) if initial_alignment_rows else None),
        "fusion_horizon_ekf_error": vector_error_statistics(fusion_errors),
        "ground_truth_delay_motion": vector_error_statistics(truth_delay_motions),
        "predicted_delay_motion": vector_error_statistics(predicted_delay_motions),
        "delay_compensation_error": vector_error_statistics(
            delay_compensation_errors),
        "horizon_zero_error": vector_error_statistics(horizon_zero_errors),
        "cone_to_current_ekf_error": vector_error_statistics(
            cone_to_current_ekf_errors),
        "decomposition_closure_max_norm_m": (
            float(max(
                row["decomposition_closure_norm_m"]
                for row in initial_alignment_rows
            )) if initial_alignment_rows else None),
    }
    alignment_norms = np.asarray([
        row["error_norm_m"] for row in rows
        if math.isclose(row["horizon_s"], 0.0, abs_tol=1.0e-6)
    ], dtype=np.float64)
    alignment_gate = {
        "required": bool(args.require_alignment),
        "median_limit_m": float(args.alignment_median_limit_m),
        "percentile_95_limit_m": float(args.alignment_p95_limit_m),
        "sample_count": int(len(alignment_norms)),
        "median_m": (
            float(np.median(alignment_norms)) if len(alignment_norms) else None),
        "percentile_95_m": (
            float(np.percentile(alignment_norms, 95))
            if len(alignment_norms) else None),
    }
    alignment_gate["pass"] = bool(
        len(alignment_norms)
        and alignment_gate["median_m"] <= args.alignment_median_limit_m
        and alignment_gate["percentile_95_m"] <= args.alignment_p95_limit_m)

    # A timestamp shift sweep is diagnostic only. A large optimum shift aligned
    # with the flight direction indicates a state-time labeling issue; it must
    # not be silently applied as a correction because spatial bias can look
    # similar during straight flight.
    alignment_shifts_s = np.linspace(-0.25, 0.25, 501)
    alignment_shift_scores = []
    belief_messages = [item for _, item in messages[topics["belief"]]]
    for shift_s in alignment_shifts_s:
        norms = []
        for belief in belief_messages:
            truth = interpolate_ground_truth(
                gt_times_us, gt_positions,
                float(belief.timestamp_sample) + shift_s * 1.0e6)
            if truth is not None:
                error = truth - np.asarray(belief.position, dtype=np.float64)
                norms.append(float(np.linalg.norm(error)))
        alignment_shift_scores.append(
            float(np.median(norms)) if norms else math.inf)
    if alignment_shift_scores and np.isfinite(alignment_shift_scores).any():
        best_shift_index = int(np.argmin(alignment_shift_scores))
        initial_alignment.update({
            "diagnostic_best_fusion_truth_time_shift_s": float(
                alignment_shifts_s[best_shift_index]),
            "diagnostic_best_shift_median_error_m": float(
                alignment_shift_scores[best_shift_index]),
            "diagnostic_zero_shift_median_error_m": float(
                alignment_shift_scores[len(alignment_shifts_s) // 2]),
            "diagnostic_shift_search_range_s": [-0.25, 0.25],
        })

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
        "causal_segment_trajectory_containment_rate": causal_containment,
        "full_4_5s_trajectory_containment_rate": full_containment,
        "cones_with_95_first_exit": len(first_exit_horizons),
        "first_exit_horizon_median_s": (
            float(np.median(first_exit_horizons)) if first_exit_horizons else None),
        "first_exit_horizon_mean_s": (
            float(np.mean(first_exit_horizons)) if first_exit_horizons else None),
        "mean_position_error_m": mean_error,
        "max_position_error_m": max_error,
        "propagation_error_at_4_5_s": vector_error_statistics(
            full_endpoint_propagation_errors),
        "initial_alignment": initial_alignment,
        "alignment_gate": alignment_gate,
        "current_local_position_error": vector_error_statistics(
            current_local_position_errors),
        "belief_position_velocity_kinematic_error_mps": belief_kinematic_error,
        "ground_truth_position_velocity_kinematic_error_mps": (
            ground_truth_kinematic_error),
        "formal_coverage_pass": None,
        "formal_coverage_note":
            "Preliminary Q scaling is reported; independent holdout and simultaneous-path acceptance are not gated.",
    })

    with (output_dir / "cone_samples.csv").open("w", newline="", encoding="utf-8") as stream:
        if rows:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    with (output_dir / "initial_alignment.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        if initial_alignment_rows:
            writer = csv.DictWriter(
                stream, fieldnames=list(initial_alignment_rows[0].keys()))
            writer.writeheader()
            writer.writerows(initial_alignment_rows)
    with (output_dir / "cone_evaluation.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        if cone_evaluation_rows:
            writer = csv.DictWriter(
                stream, fieldnames=list(cone_evaluation_rows[0].keys()))
            writer.writeheader()
            writer.writerows(cone_evaluation_rows)
    with (output_dir / "coverage_by_horizon.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        if horizon_statistics:
            writer = csv.DictWriter(
                stream, fieldnames=list(horizon_statistics[0].keys()))
            writer.writeheader()
            writer.writerows(horizon_statistics)
    np.savez_compressed(
        output_dir / "cone_arrays.npz",
        predicted_mean=np.asarray(npz_mean),
        predicted_position_covariance=np.asarray(npz_covariance),
        ground_truth=np.asarray(npz_ground_truth),
        causal_point_count=np.asarray(npz_point_count, dtype=np.int32),
        source_timestamp_us=np.asarray(npz_source_timestamp, dtype=np.int64),
        candidate_inputs=np.asarray(npz_candidate_inputs),
        time_offsets_s=np.asarray(npz_time_offsets),
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
        or (args.require_alignment and not alignment_gate["pass"])
        or (
            args.require_propagation_contract
            and not summary["propagation_test_contract"]["pass"])
        or (
            args.require_propagation_contract
            and not summary["trim_gate"]["pass"])
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
    parser.add_argument(
        "--require-alignment", action="store_true",
        help="fail analysis when horizon-zero alignment exceeds configured limits")
    parser.add_argument(
        "--require-propagation-contract", action="store_true",
        help=(
            "require the FixedWing propagation debug topic and a valid "
            "input/epoch/ZOH contract"))
    parser.add_argument("--trim-hold-s", type=float, default=2.0)
    parser.add_argument(
        "--trim-airspeed-tolerance-mps", type=float, default=1.0)
    parser.add_argument(
        "--trim-vertical-speed-tolerance-mps", type=float, default=0.5)
    parser.add_argument("--trim-roll-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--trim-max-sample-age-s", type=float, default=0.5)
    parser.add_argument("--alignment-median-limit-m", type=float, default=0.5)
    parser.add_argument("--alignment-p95-limit-m", type=float, default=1.0)
    return analyze(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
