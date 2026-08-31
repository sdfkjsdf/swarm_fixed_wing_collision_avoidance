#!/usr/bin/env python3
"""Analyze a five-aircraft point-convergence bag and render an MP4 offline."""

import argparse
import csv
import json
import math
import re
from collections import Counter
from itertools import combinations
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import FancyArrowPatch
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


AIRCRAFT_COUNT = 5
CANDIDATE_ROLL_DEGREES = (-50, -30, -15, 0, 15, 30, 50)
V4_CANDIDATE_ROLES = ("near", "left", "right")
COLORS = plt.get_cmap("tab10").colors[:AIRCRAFT_COUNT]


def quaternion_yaw_ned(quaternion):
    """Return body heading in NED from PX4's [w, x, y, z] quaternion."""
    values = np.asarray(quaternion, dtype=np.float64)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        return math.nan
    norm = float(np.linalg.norm(values))
    if norm <= np.finfo(np.float64).eps:
        return math.nan
    w, x, y, z = values / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z))


def command_execution_requested(message):
    """Read the actual PX4 command gate, with old-bag compatibility."""
    return bool(getattr(
        message, "command_execution_requested", message.activation_requested))


def observed_rate_hz(timestamps):
    """Return the event rate over the first-to-last observed timestamp."""
    ordered = sorted(set(int(timestamp) for timestamp in timestamps))
    if len(ordered) < 2 or ordered[-1] <= ordered[0]:
        return None
    return (len(ordered) - 1) * 1.0e6 / (ordered[-1] - ordered[0])


def read_bag(bag: Path):
    selected_topics = set()
    for vehicle in range(AIRCRAFT_COUNT):
        selected_topics.add(
            f"/common/px4_{vehicle}/trans_vehicle_odometry")
        selected_topics.add(
            f"/common/px4_{vehicle}/maneuver_selection_decision")
        selected_topics.add(
            f"/common/px4_{vehicle}/trajectory_intent")

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
    missing_odometry = [
        topic for topic in selected_topics
        if topic.endswith("trans_vehicle_odometry") and topic not in topic_types
    ]
    if missing_odometry:
        raise RuntimeError(
            "missing required odometry topics: " + ", ".join(missing_odometry))

    messages = {topic: [] for topic in selected_topics if topic in topic_types}
    classes = {topic: get_message(topic_types[topic]) for topic in messages}
    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic in messages:
            messages[topic].append(
                (bag_time_ns, deserialize_message(serialized, classes[topic])))
    return messages


def interpolate_tracks(messages, sample_hz: float, evaluation_start_ns: int):
    raw_tracks = []
    for vehicle in range(AIRCRAFT_COUNT):
        topic = f"/common/px4_{vehicle}/trans_vehicle_odometry"
        records = messages[topic]
        if len(records) < 2:
            raise RuntimeError(f"too few odometry messages on {topic}")
        times_ns = np.asarray([record[0] for record in records], dtype=np.int64)
        positions = np.asarray(
            [record[1].position for record in records], dtype=np.float64)
        velocities = np.asarray(
            [record[1].velocity for record in records], dtype=np.float64)
        headings = np.asarray(
            [quaternion_yaw_ned(record[1].q) for record in records],
            dtype=np.float64)
        order = np.argsort(times_ns, kind="stable")
        times_ns = times_ns[order]
        positions = positions[order]
        velocities = velocities[order]
        headings = headings[order]
        unique = np.concatenate(([True], np.diff(times_ns) > 0))
        raw_tracks.append(
            (times_ns[unique], positions[unique], velocities[unique],
             headings[unique]))

    common_stream_start_ns = max(track[0][0] for track in raw_tracks)
    start_ns = max(common_stream_start_ns, evaluation_start_ns)
    end_ns = min(track[0][-1] for track in raw_tracks)
    if end_ns <= start_ns:
        raise RuntimeError("aircraft odometry streams have no common time span")
    step_ns = max(1, int(round(1.0e9 / sample_hz)))
    grid_ns = np.arange(start_ns, end_ns + 1, step_ns, dtype=np.int64)
    tracks = np.empty((AIRCRAFT_COUNT, len(grid_ns), 3), dtype=np.float64)
    velocities = np.empty_like(tracks)
    body_headings = np.empty(
        (AIRCRAFT_COUNT, len(grid_ns)), dtype=np.float64)
    for vehicle, (times_ns, positions, raw_velocities, headings) in enumerate(
            raw_tracks):
        for axis in range(3):
            tracks[vehicle, :, axis] = np.interp(
                grid_ns.astype(np.float64),
                times_ns.astype(np.float64),
                positions[:, axis])
            velocities[vehicle, :, axis] = np.interp(
                grid_ns.astype(np.float64),
                times_ns.astype(np.float64),
                raw_velocities[:, axis])
        finite_heading = np.isfinite(headings)
        if np.count_nonzero(finite_heading) < 2:
            raise RuntimeError(
                f"too few valid attitude quaternions for aircraft {vehicle}")
        body_headings[vehicle] = np.interp(
            grid_ns.astype(np.float64),
            times_ns[finite_heading].astype(np.float64),
            np.unwrap(headings[finite_heading]))
    elapsed_s = (grid_ns - grid_ns[0]).astype(np.float64) * 1.0e-9
    return grid_ns, elapsed_s, tracks, velocities, body_headings


def fleet_standard_deviations(tracks, velocities):
    """Return population 3-D spread about the five-aircraft centroid."""
    position_sigma = np.linalg.norm(np.std(tracks, axis=0), axis=1)
    velocity_sigma = np.linalg.norm(np.std(velocities, axis=0), axis=1)
    return position_sigma, velocity_sigma


def separation_history(tracks):
    pair_list = list(combinations(range(AIRCRAFT_COUNT), 2))
    pair_distance = np.empty((len(pair_list), tracks.shape[1]), dtype=np.float64)
    pair_horizontal_distance = np.empty_like(pair_distance)
    for pair_index, (first, second) in enumerate(pair_list):
        difference = tracks[first] - tracks[second]
        pair_distance[pair_index] = np.linalg.norm(difference, axis=1)
        pair_horizontal_distance[pair_index] = np.linalg.norm(
            difference[:, :2], axis=1)
    nearest_pair_index = np.argmin(pair_distance, axis=0)
    time_index = np.arange(tracks.shape[1])
    minimum_distance = pair_distance[nearest_pair_index, time_index]
    minimum_horizontal = pair_horizontal_distance[
        nearest_pair_index, time_index]
    return (
        pair_list, pair_distance, minimum_distance, minimum_horizontal,
        nearest_pair_index)


def decision_records(messages, start_ns):
    decisions = []
    for vehicle in range(AIRCRAFT_COUNT):
        topic = f"/common/px4_{vehicle}/maneuver_selection_decision"
        records = messages.get(topic, [])
        decisions.append([
            ((bag_time_ns - start_ns) * 1.0e-9, message)
            for bag_time_ns, message in records if bag_time_ns >= start_ns
        ])
    return decisions


def intent_records(messages):
    return [
        messages.get(f"/common/px4_{vehicle}/trajectory_intent", [])
        for vehicle in range(AIRCRAFT_COUNT)
    ]


def decision_summary(decisions):
    result = []
    for vehicle, records in enumerate(decisions):
        qualified = sum(
            bool(message.coordination_qualified) for _, message in records)
        evaluated_243 = sum(
            int(message.evaluated_combination_count) == 243
            for _, message in records)
        evaluated_16807 = sum(
            int(message.evaluated_combination_count) == 16807
            for _, message in records)
        valid_proposals = sum(
            bool(message.proposal_valid) for _, message in records)
        confirmed_proposals = sum(
            bool(message.proposal_consensus_confirmed)
            for _, message in records)
        superiority_evaluations = sum(
            bool(getattr(message, "switch_superiority_evaluated", False))
            for _, message in records)
        clearly_superior = sum(
            bool(getattr(message, "switch_clearly_superior", False))
            for _, message in records)
        accepted_switches = sum(
            bool(message.new_best_accepted
                 and getattr(message, "switch_clearly_superior", False))
            for _, message in records)
        selected_v4 = sum(
            bool(message.selected_v4_cutover) for _, message in records)
        proposed_v4 = sum(
            bool(message.proposal_valid and message.proposed_v4_cutover)
            for _, message in records)
        full_evaluations = {
            (int(message.proposal_epoch),
             int(message.proposal_timestamp_us)):
                int(message.evaluated_combination_count)
            for _, message in records
            if int(message.proposal_timestamp_us) > 0
            and int(message.evaluated_combination_count) in (243, 16807)
        }
        evaluation_timestamps = [
            timestamp for _, timestamp in full_evaluations]
        bag_timestamps_us = [
            int(round(time_s * 1.0e6)) for time_s, _ in records]
        result.append({
            "vehicle_id": vehicle,
            "decision_count": len(records),
            "qualified_count": qualified,
            "evaluated_243_count": evaluated_243,
            "evaluated_16807_count": evaluated_16807,
            "valid_proposal_count": valid_proposals,
            "confirmed_proposal_count": confirmed_proposals,
            "switch_superiority_evaluation_count": superiority_evaluations,
            "clearly_superior_proposal_count": clearly_superior,
            "accepted_active_switch_count": accepted_switches,
            "selected_v4_cutover_count": selected_v4,
            "proposed_v4_cutover_count": proposed_v4,
            "decision_publish_rate_hz": observed_rate_hz(bag_timestamps_us),
            "unique_full_evaluation_count": len(full_evaluations),
            "full_evaluation_rate_hz": observed_rate_hz(
                evaluation_timestamps),
            "full_evaluation_combination_histogram": dict(sorted(Counter(
                full_evaluations.values()).items())),
        })
    return result


def v4_shadow_summary(decisions):
    result = []
    for vehicle, records in enumerate(decisions):
        enabled = [message for _, message in records if message.v4_enabled]
        evaluated = [
            message for message in enabled if message.v4_shadow_evaluated]
        effective_rates = [
            float(message.v4_effective_max_heading_rate_radps)
            for message in evaluated
            if math.isfinite(message.v4_effective_max_heading_rate_radps)]
        result.append({
            "vehicle_id": vehicle,
            "enabled_record_count": len(enabled),
            "evaluated_record_count": len(evaluated),
            "shadow_only_record_count": sum(
                bool(message.v4_shadow_only) for message in enabled),
            "cutover_record_count": sum(
                not message.v4_shadow_only for message in enabled),
            "shadow_status_counts": dict(sorted(Counter(
                int(message.v4_shadow_status)
                for message in enabled).items())),
            "core_status_counts": dict(sorted(Counter(
                int(message.v4_core_status)
                for message in evaluated).items())),
            "airspeed_snapshot_status_counts": dict(sorted(Counter(
                int(message.v4_airspeed_snapshot_status)
                for message in enabled).items())),
            "airspeed_source_counts": dict(sorted(Counter(
                int(message.v4_airspeed_source)
                for message in enabled).items())),
            "nominal_snapshot_status_counts": dict(sorted(Counter(
                int(message.v4_nominal_snapshot_status)
                for message in enabled).items())),
            "candidate_count_histogram": dict(sorted(Counter(
                int(message.v4_candidate_count)
                for message in evaluated).items())),
            "left_infeasible_count": sum(
                not message.v4_left_feasible for message in evaluated),
            "right_infeasible_count": sum(
                not message.v4_right_feasible for message in evaluated),
            "maximum_airspeed_age_us": max(
                (int(message.v4_airspeed_age_us) for message in enabled),
                default=None),
            "maximum_nominal_age_us": max(
                (int(message.v4_nominal_age_us) for message in enabled),
                default=None),
            "effective_max_heading_rate_radps_range": (
                [min(effective_rates), max(effective_rates)]
                if effective_rates else None),
        })
    return result


def v4_horizon_gate_summary(decisions):
    result = []
    for vehicle, records in enumerate(decisions):
        evaluated = [
            message for _, message in records
            if message.v4_horizon_gate_evaluated]
        valid = [
            message for message in evaluated
            if message.v4_horizon_gate_valid
            and math.isfinite(message.v4_horizon_h_worst_m)]
        h_worst = [
            float(message.v4_horizon_h_worst_m) for message in valid]
        result.append({
            "vehicle_id": vehicle,
            "evaluated_record_count": len(evaluated),
            "valid_record_count": len(valid),
            "local_active_record_count": sum(
                bool(message.v4_horizon_local_gate_active)
                for _, message in records),
            "active_record_count": sum(
                bool(message.v4_horizon_gate_active)
                for _, message in records),
            "command_execution_record_count": sum(
                bool(message.command_execution_requested)
                for _, message in records),
            "h_worst_m_range": (
                [min(h_worst), max(h_worst)] if h_worst else None),
            "trigger_m_values": sorted(set(
                float(message.v4_horizon_trigger_m)
                for message in valid
                if math.isfinite(message.v4_horizon_trigger_m))),
        })
    return result


def activation_state_summary(decisions):
    result = []
    for vehicle, records in enumerate(decisions):
        starts = []
        ends = []
        active_candidate_switch_count = 0
        active_revision_switch_count = 0
        repeated_start_flag_count = 0
        repeated_end_flag_count = 0
        handoff_cpa_clear_count = 0
        handoff_post_safe_count = 0
        handoff_ready_count = 0
        handoff_consensus_count = 0
        handoff_post_ad_values = []
        previous_active = False
        previous_active_candidate = None
        previous_active_revision = None
        for time_s, message in records:
            active = command_execution_requested(message)
            handoff_cpa_clear_count += int(bool(
                getattr(message, "handoff_cpa_clear", False)))
            handoff_post_safe_count += int(bool(
                getattr(message, "handoff_post_ad_safe", False)))
            handoff_ready_count += int(bool(
                getattr(message, "handoff_ready", False)))
            handoff_consensus_count += int(bool(
                getattr(message, "handoff_consensus_confirmed", False)))
            handoff_post_ad = float(getattr(
                message, "handoff_post_minimum_ad_m", math.nan))
            if math.isfinite(handoff_post_ad):
                handoff_post_ad_values.append(handoff_post_ad)
            if active and not previous_active:
                starts.append({
                    "time_s": float(time_s),
                    "ad_m": float(message.ad_m),
                    "candidate_id": int(message.ownship_candidate_id),
                    "candidate_input_revision": int(
                        message.selected_candidate_input_revisions[vehicle]),
                })
            if not active and previous_active:
                ends.append({
                    "time_s": float(time_s),
                    "reason": int(message.deactivation_reason),
                })
            if message.activation_just_started and previous_active:
                repeated_start_flag_count += 1
            if message.activation_just_ended and not previous_active:
                repeated_end_flag_count += 1
            if active:
                candidate_id = int(message.ownship_candidate_id)
                revision = int(
                    message.selected_candidate_input_revisions[vehicle])
                if (previous_active_candidate is not None
                        and candidate_id != previous_active_candidate):
                    active_candidate_switch_count += 1
                if (previous_active_revision is not None
                        and revision != previous_active_revision):
                    active_revision_switch_count += 1
                previous_active_candidate = candidate_id
                previous_active_revision = revision
            else:
                previous_active_candidate = None
                previous_active_revision = None
            previous_active = active
        result.append({
            "vehicle_id": vehicle,
            "activation_start_count": len(starts),
            "activation_end_count": len(ends),
            "active_candidate_switch_count": active_candidate_switch_count,
            "active_revision_switch_count": active_revision_switch_count,
            "repeated_start_flag_count": repeated_start_flag_count,
            "repeated_end_flag_count": repeated_end_flag_count,
            "handoff_cpa_clear_record_count": handoff_cpa_clear_count,
            "handoff_post_safe_record_count": handoff_post_safe_count,
            "handoff_ready_record_count": handoff_ready_count,
            "handoff_consensus_record_count": handoff_consensus_count,
            "handoff_post_ad_m_range": (
                [min(handoff_post_ad_values), max(handoff_post_ad_values)]
                if handoff_post_ad_values else None),
            "starts": starts,
            "ends": ends,
        })
    return result


def trajectory_intent_summary(intents, decisions, start_ns):
    result = []
    for vehicle in range(AIRCRAFT_COUNT):
        all_records = intents[vehicle]
        evaluation_records = [
            message for bag_time_ns, message in all_records
            if bag_time_ns >= start_ns]
        exact_signatures = {
            (int(message.source_timestamp_us), int(message.candidate_id),
             int(message.candidate_input_revision),
             int(message.candidate_set_kind))
            for _, message in all_records
        }
        command_signatures = {
            (int(message.candidate_id),
             int(message.candidate_input_revision),
             int(message.candidate_set_kind))
            for _, message in all_records
        }
        selected_reference_count = 0
        missing_exact_reference_count = 0
        missing_command_reference_count = 0
        for _, decision in decisions[vehicle]:
            if not decision.coordination_qualified:
                continue
            selected_reference_count += 1
            kind = 1 if decision.selected_v4_cutover else 0
            exact = (
                int(decision.selected_candidate_source_timestamps_us[vehicle]),
                int(decision.selected_candidate_ids[vehicle]),
                int(decision.selected_candidate_input_revisions[vehicle]),
                kind)
            command = (exact[1], exact[2], exact[3])
            if exact not in exact_signatures:
                missing_exact_reference_count += 1
            if command not in command_signatures:
                missing_command_reference_count += 1
        invalid_v4_metadata_count = sum(
            int(message.candidate_set_size) < 1
            or int(message.candidate_set_size) > 3
            or int(message.candidate_id) >= len(V4_CANDIDATE_ROLES)
            for message in evaluation_records
            if int(message.candidate_set_kind) == 1)
        source_timestamps = [
            int(message.source_timestamp_us)
            for message in evaluation_records]
        epoch_first_source_timestamp = {}
        for message in evaluation_records:
            epoch = int(message.selection_epoch)
            timestamp = int(message.source_timestamp_us)
            if epoch not in epoch_first_source_timestamp:
                epoch_first_source_timestamp[epoch] = timestamp
            else:
                epoch_first_source_timestamp[epoch] = min(
                    epoch_first_source_timestamp[epoch], timestamp)
        result.append({
            "vehicle_id": vehicle,
            "intent_record_count": len(evaluation_records),
            "candidate_set_kind_counts": dict(sorted(Counter(
                int(message.candidate_set_kind)
                for message in evaluation_records).items())),
            "v4_candidate_set_size_histogram": dict(sorted(Counter(
                int(message.candidate_set_size)
                for message in evaluation_records
                if int(message.candidate_set_kind) == 1).items())),
            "invalid_v4_metadata_count": invalid_v4_metadata_count,
            "selected_reference_count": selected_reference_count,
            "missing_exact_selected_intent_count": (
                missing_exact_reference_count),
            "missing_selected_command_count": (
                missing_command_reference_count),
            "unique_trajectory_refresh_count": len(set(source_timestamps)),
            "trajectory_refresh_rate_hz": observed_rate_hz(
                source_timestamps),
            "unique_candidate_generation_epoch_count": len(
                epoch_first_source_timestamp),
            "candidate_generation_rate_hz": observed_rate_hz(
                epoch_first_source_timestamp.values()),
        })
    return result


def formation_override_summary(log_dir, messages, intents):
    pattern = re.compile(
        r"^\[WARN\] \[(\d+\.\d+)\].*candidate=(\d+) .*"
        r"a_lat=([-+0-9.]+) v_ground=([-+0-9.]+)")
    result = []
    for vehicle in range(AIRCRAFT_COUNT):
        decision_topic = (
            f"/common/px4_{vehicle}/maneuver_selection_decision")
        raw_decisions = messages.get(decision_topic, [])
        decision_times = np.asarray(
            [record[0] for record in raw_decisions], dtype=np.int64)
        exact_intents = {
            (int(message.source_timestamp_us), int(message.candidate_id),
             int(message.candidate_input_revision),
             int(message.candidate_set_kind)): message
            for _, message in intents[vehicle]
        }
        override_count = 0
        exact_match_count = 0
        missing_reference_count = 0
        inactive_decision_count = 0
        log_path = log_dir / f"guidance_{vehicle}.log"
        lines = (log_path.read_text(errors="replace").splitlines()
                 if log_path.is_file() else [])
        for line in lines:
            matched = pattern.search(line)
            if matched is None:
                continue
            override_count += 1
            timestamp_ns = int(round(float(matched.group(1)) * 1.0e9))
            index = int(np.searchsorted(
                decision_times, timestamp_ns, side="right")) - 1
            if index < 0:
                missing_reference_count += 1
                continue
            decision = raw_decisions[index][1]
            if not command_execution_requested(decision):
                inactive_decision_count += 1
                continue
            kind = 1 if decision.selected_v4_cutover else 0
            key = (
                int(decision.selected_candidate_source_timestamps_us[vehicle]),
                int(decision.selected_candidate_ids[vehicle]),
                int(decision.selected_candidate_input_revisions[vehicle]),
                kind)
            intent = exact_intents.get(key)
            if intent is None:
                missing_reference_count += 1
                continue
            logged_candidate_id = int(matched.group(2))
            logged_lateral_acceleration = float(matched.group(3))
            logged_ground_speed = float(matched.group(4))
            if (logged_candidate_id == key[1]
                    and abs(logged_ground_speed
                            - float(intent.candidate_input[0])) <= 0.011
                    and abs(logged_lateral_acceleration
                            - float(intent.candidate_input[3])) <= 0.011):
                exact_match_count += 1
        result.append({
            "vehicle_id": vehicle,
            "override_log_count": override_count,
            "selected_intent_match_count": exact_match_count,
            "missing_selected_intent_count": missing_reference_count,
            "latest_decision_inactive_count": inactive_decision_count,
            "rounded_value_tolerance": 0.011,
        })
    return result


def runtime_policy_summary(log_dir):
    pattern = re.compile(
        r"execution_policy=(\S+).*active_switch=(\d+) "
        r"switch_cost_margin=([-+0-9.eE]+) "
        r"switch_ad_margin=([-+0-9.eE]+)")
    by_vehicle = []
    signatures = set()
    for vehicle in range(AIRCRAFT_COUNT):
        log_path = log_dir / f"guidance_{vehicle}.log"
        matched = None
        if log_path.is_file():
            for line in log_path.read_text(errors="replace").splitlines():
                matched = pattern.search(line)
                if matched is not None:
                    break
        if matched is None:
            by_vehicle.append({
                "vehicle_id": vehicle,
                "available": False,
            })
            continue
        signature = (
            matched.group(1), bool(int(matched.group(2))),
            float(matched.group(3)), float(matched.group(4)))
        signatures.add(signature)
        by_vehicle.append({
            "vehicle_id": vehicle,
            "available": True,
            "execution_policy": signature[0],
            "active_switching_enabled": signature[1],
            "active_switch_cost_margin": signature[2],
            "active_switch_minimum_ad_margin_m": signature[3],
        })
    return {
        "all_vehicle_logs_available": all(
            item["available"] for item in by_vehicle),
        "all_vehicle_policy_values_match": len(signatures) == 1
            and all(item["available"] for item in by_vehicle),
        "by_vehicle": by_vehicle,
    }


def decision_consensus_summary(decisions, elapsed_s):
    all_qualified_count = 0
    consensus_count = 0
    first_mismatch_time_s = None
    for time_s in elapsed_s:
        latest = [latest_decision(records, time_s) for records in decisions]
        if any(decision is None for decision in latest):
            continue
        if not all(decision.coordination_qualified for decision in latest):
            continue
        all_qualified_count += 1
        identities = [selected_identity(decision) for decision in latest]
        if all(identity == identities[0] for identity in identities[1:]):
            consensus_count += 1
        elif first_mismatch_time_s is None:
            first_mismatch_time_s = float(time_s)
    per_vehicle_by_epoch = []
    for records in decisions:
        by_epoch = {}
        for _, decision in records:
            if decision.coordination_qualified:
                by_epoch[int(decision.local_selection_epoch)] = decision
        per_vehicle_by_epoch.append(by_epoch)
    common_epochs = set(per_vehicle_by_epoch[0])
    for by_epoch in per_vehicle_by_epoch[1:]:
        common_epochs.intersection_update(by_epoch)
    same_epoch_tuple_count = 0
    first_epoch_mismatch = None
    for epoch in sorted(common_epochs):
        identities = [
            selected_identity(by_epoch[epoch])
            for by_epoch in per_vehicle_by_epoch
        ]
        if all(identity == identities[0] for identity in identities[1:]):
            same_epoch_tuple_count += 1
        elif first_epoch_mismatch is None:
            first_epoch_mismatch = epoch

    return {
        "common_qualified_epoch_count": len(common_epochs),
        "same_tuple_common_epoch_count": same_epoch_tuple_count,
        "same_tuple_common_epoch_ratio": (
            float(same_epoch_tuple_count / len(common_epochs))
            if common_epochs else None),
        "first_common_epoch_mismatch": first_epoch_mismatch,
        "wall_time_all_nodes_qualified_sample_count": all_qualified_count,
        "wall_time_same_latest_tuple_sample_count": consensus_count,
        "wall_time_same_latest_tuple_ratio": (
            float(consensus_count / all_qualified_count)
            if all_qualified_count else None),
        "wall_time_first_mismatch_s": first_mismatch_time_s,
    }


def coordination_invariant_summary(decisions):
    per_vehicle_by_selected_epoch = []
    per_vehicle_by_proposal_epoch = []
    active_selected_slot_mismatch_count = 0
    pending_active_proposal_count = 0
    unauthorized_active_proposal_count = 0
    unconfirmed_current_epoch_qualified_count = 0
    selected_epoch_vector_mismatch_count = 0

    for vehicle, records in enumerate(decisions):
        selected_by_epoch = {}
        proposal_by_epoch = {}
        for _, decision in records:
            selected_epoch = int(decision.local_selection_epoch)
            proposal_epoch = int(decision.proposal_epoch)
            if decision.coordination_qualified:
                selected_by_epoch[selected_epoch] = decision
                if any(
                        int(epoch) != selected_epoch
                        for epoch in decision.selection_epochs_by_aircraft[
                            :AIRCRAFT_COUNT]):
                    selected_epoch_vector_mismatch_count += 1
            if decision.proposal_valid:
                proposal_by_epoch[proposal_epoch] = decision
            if (decision.coordination_qualified
                    and decision.proposal_valid
                    and selected_epoch == proposal_epoch
                    and not decision.proposal_consensus_confirmed):
                unconfirmed_current_epoch_qualified_count += 1
            if (command_execution_requested(decision)
                    and int(decision.selected_candidate_ids[vehicle])
                    != int(decision.ownship_candidate_id)):
                active_selected_slot_mismatch_count += 1
            if (command_execution_requested(decision)
                    and decision.proposal_valid
                    and int(decision.proposed_candidate_ids[vehicle])
                    != int(decision.ownship_candidate_id)):
                pending_active_proposal_count += 1
                if not (decision.switch_superiority_evaluated
                        and decision.switch_clearly_superior):
                    unauthorized_active_proposal_count += 1
        per_vehicle_by_selected_epoch.append(selected_by_epoch)
        per_vehicle_by_proposal_epoch.append(proposal_by_epoch)

    common_selected_epochs = set(per_vehicle_by_selected_epoch[0])
    common_proposal_epochs = set(per_vehicle_by_proposal_epoch[0])
    for by_epoch in per_vehicle_by_selected_epoch[1:]:
        common_selected_epochs.intersection_update(by_epoch)
    for by_epoch in per_vehicle_by_proposal_epoch[1:]:
        common_proposal_epochs.intersection_update(by_epoch)

    peer_ownship_assumption_mismatch_count = 0
    first_peer_ownship_mismatch_epoch = None
    for epoch in sorted(common_selected_epochs):
        decisions_at_epoch = [
            by_epoch[epoch] for by_epoch in per_vehicle_by_selected_epoch]
        for observer in range(AIRCRAFT_COUNT):
            observer_tuple = decisions_at_epoch[
                observer].selected_candidate_ids
            for peer in range(AIRCRAFT_COUNT):
                if int(observer_tuple[peer]) != int(
                        decisions_at_epoch[peer].ownship_candidate_id):
                    peer_ownship_assumption_mismatch_count += 1
                    if first_peer_ownship_mismatch_epoch is None:
                        first_peer_ownship_mismatch_epoch = epoch

    same_proposal_tuple_count = 0
    same_proposal_command_count = 0
    first_proposal_mismatch_epoch = None
    first_proposal_command_mismatch_epoch = None
    for epoch in sorted(common_proposal_epochs):
        identities = [
            proposal_identity(by_epoch[epoch])
            for by_epoch in per_vehicle_by_proposal_epoch]
        command_identities = [
            proposal_command_identity(by_epoch[epoch])
            for by_epoch in per_vehicle_by_proposal_epoch]
        if all(identity == identities[0] for identity in identities[1:]):
            same_proposal_tuple_count += 1
        elif first_proposal_mismatch_epoch is None:
            first_proposal_mismatch_epoch = epoch
        if all(
                identity == command_identities[0]
                for identity in command_identities[1:]):
            same_proposal_command_count += 1
        elif first_proposal_command_mismatch_epoch is None:
            first_proposal_command_mismatch_epoch = epoch

    return {
        "common_selected_epoch_count": len(common_selected_epochs),
        "peer_ownship_assumption_mismatch_count": (
            peer_ownship_assumption_mismatch_count),
        "first_peer_ownship_mismatch_epoch": (
            first_peer_ownship_mismatch_epoch),
        "common_proposal_epoch_count": len(common_proposal_epochs),
        "same_proposal_tuple_count": same_proposal_tuple_count,
        "same_proposal_tuple_ratio": (
            float(same_proposal_tuple_count / len(common_proposal_epochs))
            if common_proposal_epochs else None),
        "first_proposal_mismatch_epoch": first_proposal_mismatch_epoch,
        "same_proposal_command_count": same_proposal_command_count,
        "same_proposal_command_ratio": (
            float(same_proposal_command_count / len(common_proposal_epochs))
            if common_proposal_epochs else None),
        "first_proposal_command_mismatch_epoch": (
            first_proposal_command_mismatch_epoch),
        "unconfirmed_current_epoch_qualified_count": (
            unconfirmed_current_epoch_qualified_count),
        "selected_epoch_vector_mismatch_count": (
            selected_epoch_vector_mismatch_count),
        "active_selected_slot_mismatch_count": (
            active_selected_slot_mismatch_count),
        "pending_active_proposal_count": pending_active_proposal_count,
        "unauthorized_active_proposal_count": (
            unauthorized_active_proposal_count),
    }


def latest_decision(records, elapsed_s):
    if not records:
        return None
    times = [record[0] for record in records]
    index = int(np.searchsorted(times, elapsed_s, side="right")) - 1
    return records[index][1] if index >= 0 else None


def selected_identity(decision):
    return (
        bool(decision.selected_v4_cutover),
        tuple(decision.selected_candidate_ids[:AIRCRAFT_COUNT]),
        tuple(decision.selected_candidate_input_revisions[:AIRCRAFT_COUNT]),
        tuple(
            decision.selected_candidate_source_timestamps_us[
                :AIRCRAFT_COUNT]))


def proposal_identity(decision):
    return (
        bool(decision.proposed_v4_cutover),
        tuple(decision.proposed_candidate_ids[:AIRCRAFT_COUNT]),
        tuple(decision.proposed_candidate_input_revisions[:AIRCRAFT_COUNT]),
        tuple(
            decision.proposed_candidate_source_timestamps_us[
                :AIRCRAFT_COUNT]))


def proposal_command_identity(decision):
    return (
        bool(decision.proposed_v4_cutover),
        tuple(decision.proposed_candidate_ids[:AIRCRAFT_COUNT]),
        tuple(decision.proposed_candidate_input_revisions[:AIRCRAFT_COUNT]))


def tuple_label(decision):
    if decision is None or not decision.coordination_qualified:
        return "selection: waiting"
    labels = []
    for candidate_id in decision.selected_candidate_ids[:AIRCRAFT_COUNT]:
        if (decision.selected_v4_cutover
                and candidate_id < len(V4_CANDIDATE_ROLES)):
            labels.append(V4_CANDIDATE_ROLES[candidate_id])
        elif candidate_id < len(CANDIDATE_ROLL_DEGREES):
            labels.append(f"{CANDIDATE_ROLL_DEGREES[candidate_id]:+d}°")
        else:
            labels.append("?")
    kind = "V4 role" if decision.selected_v4_cutover else "roll"
    return f"selected {kind} tuple: [" + ", ".join(labels) + "]"


def distributed_tuple_label(decisions, elapsed_s):
    latest = [latest_decision(records, elapsed_s) for records in decisions]
    if any(decision is None for decision in latest):
        return "selection: waiting"
    qualified = [
        decision for decision in latest if decision.coordination_qualified]
    if not qualified:
        return "selection: unqualified"
    identities = [selected_identity(decision) for decision in qualified]
    label = tuple_label(qualified[0])
    if len(qualified) != AIRCRAFT_COUNT:
        return f"{label} ({len(qualified)}/5 nodes qualified)"
    if not all(identity == identities[0] for identity in identities[1:]):
        return f"node 0 {label.removeprefix('selected ')} (tuple mismatch)"
    return label + " (5/5 consensus)"


def save_summary_plot(
        path, elapsed_s, tracks, body_headings, minimum_distance,
        position_sigma, velocity_sigma, dsd_m, target_norths, target_easts,
        scenario_label, show_targets):
    figure = plt.figure(figsize=(15, 9), constrained_layout=True)
    grid = figure.add_gridspec(2, 2)
    map_axis = figure.add_subplot(grid[:, 0])
    separation_axis = figure.add_subplot(grid[0, 1])
    dispersion_axis = figure.add_subplot(grid[1, 1])
    heading_stride = max(
        1, int(round(3.0 / max(float(np.median(np.diff(elapsed_s))), 1e-6))))
    heading_indices = np.arange(0, len(elapsed_s), heading_stride)
    horizontal_span = max(
        float(np.ptp(tracks[:, :, 0])),
        float(np.ptp(tracks[:, :, 1])), 1.0)
    arrow_length = max(8.0, 0.025 * horizontal_span)
    for vehicle in range(AIRCRAFT_COUNT):
        map_axis.plot(
            tracks[vehicle, :, 1], tracks[vehicle, :, 0],
            color=COLORS[vehicle], label=f"aircraft {vehicle}")
        map_axis.scatter(
            tracks[vehicle, 0, 1], tracks[vehicle, 0, 0],
            marker="o", color=COLORS[vehicle], s=30)
        map_axis.scatter(
            tracks[vehicle, -1, 1], tracks[vehicle, -1, 0],
            marker="x", color=COLORS[vehicle], s=50)
        map_axis.quiver(
            tracks[vehicle, heading_indices, 1],
            tracks[vehicle, heading_indices, 0],
            arrow_length * np.sin(body_headings[vehicle, heading_indices]),
            arrow_length * np.cos(body_headings[vehicle, heading_indices]),
            angles="xy", scale_units="xy", scale=1.0,
            color=COLORS[vehicle], alpha=0.72, width=0.0035,
            headwidth=4.0, headlength=5.0)
    if show_targets:
        map_axis.scatter(
            target_easts, target_norths, marker="*", c=COLORS, s=160,
            edgecolors="black", linewidths=0.5,
            label="assigned destinations")
    map_axis.set_title(
        f"Actual common-NED ground tracks — {scenario_label.replace('_', ' ')}\n"
        "arrows show body heading every 3 s")
    map_axis.set_xlabel("East [m]")
    map_axis.set_ylabel("North [m]")
    map_axis.axis("equal")
    map_axis.grid(True, alpha=0.3)
    map_axis.legend(loc="best", fontsize=8)

    separation_axis.plot(elapsed_s, minimum_distance, color="black")
    separation_axis.axhline(
        dsd_m, color="red", linestyle="--", label=f"DSD = {dsd_m:.1f} m")
    separation_axis.set_title("Actual minimum 3D pair separation")
    separation_axis.set_xlabel("Scenario elapsed time [s]")
    separation_axis.set_ylabel("Separation [m]")
    separation_axis.grid(True, alpha=0.3)
    separation_axis.legend(loc="best")

    position_line, = dispersion_axis.plot(
        elapsed_s, position_sigma, color="tab:blue",
        label=r"position $\sigma_{3D}$")
    dispersion_axis.set_title(
        "Fleet position and velocity standard deviation\n"
        "population spread about the five-aircraft centroid")
    dispersion_axis.set_xlabel("Scenario elapsed time [s]")
    dispersion_axis.set_ylabel(
        r"3D position standard deviation $\sigma_p$ [m]",
        color="tab:blue")
    dispersion_axis.tick_params(axis="y", labelcolor="tab:blue")
    dispersion_axis.grid(True, alpha=0.3)
    velocity_axis = dispersion_axis.twinx()
    velocity_line, = velocity_axis.plot(
        elapsed_s, velocity_sigma, color="tab:orange",
        label=r"velocity $\sigma_{3D}$")
    velocity_axis.set_ylabel(
        r"3D velocity standard deviation $\sigma_v$ [m/s]",
        color="tab:orange")
    velocity_axis.tick_params(axis="y", labelcolor="tab:orange")
    dispersion_axis.legend(
        [position_line, velocity_line],
        [position_line.get_label(), velocity_line.get_label()],
        loc="best", fontsize=8)
    dispersion_axis.text(
        0.02, 0.04,
        f"final $\\sigma_p$ = {position_sigma[-1]:.2f} m\n"
        f"final $\\sigma_v$ = {velocity_sigma[-1]:.2f} m/s",
        transform=dispersion_axis.transAxes, va="bottom", ha="left",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.82},
        fontsize=9)
    figure.savefig(path, dpi=160)
    plt.close(figure)


def save_video(
        path, elapsed_s, tracks, body_headings, minimum_distance,
        nearest_pair_index,
        pair_list, decisions, dsd_m, target_norths, target_easts,
        scenario_label, show_targets, fps):
    figure, (map_axis, separation_axis) = plt.subplots(
        1, 2, figsize=(14, 6), constrained_layout=True)
    all_east = tracks[:, :, 1].ravel()
    all_north = tracks[:, :, 0].ravel()
    if show_targets:
        all_east = np.concatenate((all_east, target_easts))
        all_north = np.concatenate((all_north, target_norths))
    span = max(float(np.ptp(all_east)), float(np.ptp(all_north)), 1.0)
    padding = 0.08 * span
    map_axis.set_xlim(float(np.min(all_east)) - padding,
                      float(np.max(all_east)) + padding)
    map_axis.set_ylim(float(np.min(all_north)) - padding,
                      float(np.max(all_north)) + padding)
    map_axis.set_aspect("equal", adjustable="box")
    map_axis.set_xlabel("East [m]")
    map_axis.set_ylabel("North [m]")
    map_axis.grid(True, alpha=0.3)
    if show_targets:
        map_axis.scatter(
            target_easts, target_norths, marker="*", c=COLORS, s=160,
            edgecolors="black", linewidths=0.5,
            label="assigned destinations")

    trails = []
    points = []
    heading_arrows = []
    heading_arrow_length = max(8.0, 0.025 * span)
    for vehicle in range(AIRCRAFT_COUNT):
        trail, = map_axis.plot(
            [], [], color=COLORS[vehicle], linewidth=1.8,
            label=f"aircraft {vehicle}")
        point, = map_axis.plot(
            [], [], marker="o", color=COLORS[vehicle], markersize=7)
        trails.append(trail)
        points.append(point)
        heading_arrow = FancyArrowPatch(
            (0.0, 0.0), (0.0, 0.0), arrowstyle="-|>",
            mutation_scale=14, color=COLORS[vehicle], linewidth=2.0,
            zorder=4)
        map_axis.add_patch(heading_arrow)
        heading_arrows.append(heading_arrow)
    closest_line, = map_axis.plot(
        [], [], color="red", linestyle="--", linewidth=1.5,
        label="closest pair")
    map_axis.legend(loc="upper right", fontsize=8)

    separation_axis.plot(
        elapsed_s, minimum_distance, color="0.70", linewidth=1.0)
    separation_axis.axhline(
        dsd_m, color="red", linestyle="--", label=f"DSD = {dsd_m:.1f} m")
    separation_cursor, = separation_axis.plot([], [], color="black", linewidth=2.0)
    separation_point, = separation_axis.plot([], [], "ko", markersize=5)
    separation_axis.set_xlim(float(elapsed_s[0]), float(elapsed_s[-1]))
    upper = max(float(np.max(minimum_distance)) * 1.05, dsd_m * 1.5)
    separation_axis.set_ylim(0.0, upper)
    separation_axis.set_xlabel("Scenario elapsed time [s]")
    separation_axis.set_ylabel("Minimum 3D separation [m]")
    separation_axis.grid(True, alpha=0.3)
    separation_axis.legend(loc="upper right", fontsize=8)

    title = figure.suptitle("")

    def update(frame):
        for vehicle in range(AIRCRAFT_COUNT):
            trails[vehicle].set_data(
                tracks[vehicle, :frame + 1, 1],
                tracks[vehicle, :frame + 1, 0])
            points[vehicle].set_data(
                [tracks[vehicle, frame, 1]],
                [tracks[vehicle, frame, 0]])
            east = tracks[vehicle, frame, 1]
            north = tracks[vehicle, frame, 0]
            heading = body_headings[vehicle, frame]
            heading_arrows[vehicle].set_positions(
                (east, north),
                (east + heading_arrow_length * math.sin(heading),
                 north + heading_arrow_length * math.cos(heading)))
        first, second = pair_list[int(nearest_pair_index[frame])]
        closest_line.set_data(
            [tracks[first, frame, 1], tracks[second, frame, 1]],
            [tracks[first, frame, 0], tracks[second, frame, 0]])
        separation_cursor.set_data(
            elapsed_s[:frame + 1], minimum_distance[:frame + 1])
        separation_point.set_data(
            [elapsed_s[frame]], [minimum_distance[frame]])
        title.set_text(
            f"{scenario_label.replace('_', ' ')}, t={elapsed_s[frame]:.1f}s | "
            f"closest={first}-{second}: {minimum_distance[frame]:.1f}m\n"
            f"{distributed_tuple_label(decisions, elapsed_s[frame])}")
        return trails + points + heading_arrows + [
            closest_line, separation_cursor, separation_point, title]

    animation = FuncAnimation(
        figure, update, frames=len(elapsed_s), interval=1000.0 / fps,
        blit=False)
    animation.save(
        path,
        writer=FFMpegWriter(
            fps=fps, codec="libx264", bitrate=2400,
            extra_args=["-pix_fmt", "yuv420p"]),
        dpi=120)
    plt.close(figure)


def analyze(args):
    messages = read_bag(args.bag)
    grid_ns, elapsed_s, tracks, velocities, body_headings = interpolate_tracks(
        messages, args.sample_hz, args.evaluation_start_ns)
    position_sigma, velocity_sigma = fleet_standard_deviations(
        tracks, velocities)
    (pair_list, pair_distance, minimum_distance, minimum_horizontal,
     nearest_pair_index) = separation_history(tracks)
    decisions = decision_records(messages, int(grid_ns[0]))
    intents = intent_records(messages)

    global_flat_index = int(np.argmin(pair_distance))
    pair_index, time_index = np.unravel_index(
        global_flat_index, pair_distance.shape)
    closest_pair = pair_list[pair_index]
    below_dsd = minimum_distance < args.desired_separation_distance
    sample_dt_s = 1.0 / args.sample_hz
    target_norths = np.asarray(
        args.target_norths
        if args.target_norths is not None
        else [args.target_north] * AIRCRAFT_COUNT,
        dtype=float)
    target_easts = np.asarray(
        args.target_easts
        if args.target_easts is not None
        else [args.target_east] * AIRCRAFT_COUNT,
        dtype=float)

    for directory in (args.summary_dir, args.plot_dir, args.video_dir):
        directory.mkdir(parents=True, exist_ok=True)

    summary = {
        "bag": str(args.bag.resolve()),
        "aircraft_count": AIRCRAFT_COUNT,
        "sample_hz": args.sample_hz,
        "scenario_label": args.scenario_label,
        "evaluation_basis": f"all_vehicles_{args.scenario_label}_active",
        "assigned_targets_ned_m": [
            {
                "vehicle_id": vehicle,
                "north_m": float(target_norths[vehicle]),
                "east_m": float(target_easts[vehicle]),
            }
            for vehicle in range(AIRCRAFT_COUNT)
        ] if args.show_targets else [],
        "requested_evaluation_start_ns": args.evaluation_start_ns,
        "actual_evaluation_start_ns": int(grid_ns[0]),
        "common_duration_s": float(elapsed_s[-1]),
        "desired_separation_distance_m": args.desired_separation_distance,
        "actual_minimum_3d_separation_m": float(
            pair_distance[pair_index, time_index]),
        "actual_minimum_horizontal_separation_m": float(
            minimum_horizontal[time_index]),
        "closest_pair": list(closest_pair),
        "closest_time_s": float(elapsed_s[time_index]),
        "dsd_violation_sample_count": int(np.count_nonzero(below_dsd)),
        "estimated_dsd_violation_duration_s": float(
            np.count_nonzero(below_dsd) * sample_dt_s),
        "fleet_standard_deviation_definition": (
            "population 3-D spread about the five-aircraft centroid"),
        "final_position_standard_deviation_m": float(position_sigma[-1]),
        "final_velocity_standard_deviation_mps": float(velocity_sigma[-1]),
        "decision_diagnostics": decision_summary(decisions),
        "v4_shadow_diagnostics": v4_shadow_summary(decisions),
        "v4_horizon_gate_diagnostics": v4_horizon_gate_summary(decisions),
        "activation_state_diagnostics": activation_state_summary(decisions),
        "trajectory_intent_diagnostics": trajectory_intent_summary(
            intents, decisions, int(grid_ns[0])),
        "formation_override_diagnostics": formation_override_summary(
            args.log_dir, messages, intents),
        "runtime_policy": runtime_policy_summary(args.log_dir),
        "distributed_decision_consensus": decision_consensus_summary(
            decisions, elapsed_s),
        "coordination_invariants": coordination_invariant_summary(decisions),
    }
    with (args.summary_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)

    with (args.summary_dir / "separation_history.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "elapsed_s", "minimum_3d_separation_m",
            "minimum_horizontal_separation_m", "closest_aircraft_a",
            "closest_aircraft_b", "below_dsd",
            "position_standard_deviation_m",
            "velocity_standard_deviation_mps"])
        for index, time_s in enumerate(elapsed_s):
            first, second = pair_list[int(nearest_pair_index[index])]
            writer.writerow([
                f"{time_s:.3f}", f"{minimum_distance[index]:.6f}",
                f"{minimum_horizontal[index]:.6f}", first, second,
                int(below_dsd[index]), f"{position_sigma[index]:.6f}",
                f"{velocity_sigma[index]:.6f}"])

    save_summary_plot(
        args.plot_dir / "actual_maneuver_overview.png",
        elapsed_s, tracks, body_headings, minimum_distance,
        position_sigma, velocity_sigma, args.desired_separation_distance,
        target_norths, target_easts, args.scenario_label,
        args.show_targets)
    save_video(
        args.video_dir / "actual_maneuver.mp4",
        elapsed_s, tracks, body_headings, minimum_distance,
        nearest_pair_index,
        pair_list, decisions, args.desired_separation_distance,
        target_norths, target_easts, args.scenario_label,
        args.show_targets, args.fps)
    print(json.dumps(summary, indent=2))
    return 0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--summary-dir", type=Path, required=True)
    parser.add_argument("--plot-dir", type=Path, required=True)
    parser.add_argument("--video-dir", type=Path, required=True)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--evaluation-start-ns", type=int, required=True)
    parser.add_argument("--desired-separation-distance", type=float, default=10.0)
    parser.add_argument("--scenario-label", default="point_convergence")
    parser.add_argument("--show-targets", action="store_true")
    parser.add_argument("--target-north", type=float, default=300.0)
    parser.add_argument("--target-east", type=float, default=300.0)
    parser.add_argument(
        "--target-norths", type=float, nargs=AIRCRAFT_COUNT, default=None)
    parser.add_argument(
        "--target-easts", type=float, nargs=AIRCRAFT_COUNT, default=None)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--fps", type=int, default=15)
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(analyze(parse_args()))
