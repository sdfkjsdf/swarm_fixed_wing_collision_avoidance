#pragma once
#include <algorithm>
#include <ostream>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <collision_avoidance/msg/interaction_graph_diagnostics.hpp>
#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>
#include <collision_avoidance/communication/ManeuverBudgetTraceMessage.hpp>

namespace collision_avoidance::communication {
// Shutdown-only conversion. No call from control callbacks or the worker loop.
inline msg::InteractionGraphDiagnostics graphDiagnosticsMessage(
    const selection::InteractionGraphDiagnostics & diagnostics,
    const selection::InteractionGraphParams & params)
{
            const auto & graph = diagnostics.graph;
            collision_avoidance::msg::InteractionGraphDiagnostics message;
            message.evaluation_timestamp_us = graph.evaluation_timestamp_us;
            message.selection_epoch = graph.selection_epoch;
            message.vehicle_id = diagnostics.vehicle_id;
            message.aircraft_count = static_cast<std::uint8_t>(
                graph.aircraft_count);
            message.graph_status = static_cast<std::uint8_t>(graph.status);
            message.enabled = diagnostics.enabled;
            message.component_proposal_used =
                diagnostics.component_proposal_used;
            message.ad_screen_m = static_cast<float>(
                params.ad_screen_m);
            message.trajectory_library_version =
                graph.trajectory_library_version;
            message.ad_masd_config_version = graph.ad_masd_config_version;
            message.graph_config_version =
                params.config_version;
            message.candidate_library_hash = graph.candidate_library_hash;
            std::copy(
                graph.participant_vehicle_ids.begin(),
                graph.participant_vehicle_ids.end(),
                message.participant_vehicle_ids.begin());
            std::copy(
                graph.source_timestamps_us.begin(),
                graph.source_timestamps_us.end(),
                message.source_timestamps_us.begin());
            std::transform(
                graph.pair_minimum_ad_m.begin(),
                graph.pair_minimum_ad_m.end(),
                message.pair_minimum_ad_m.begin(),
                [](double value) { return static_cast<float>(value); });
            std::copy(
                graph.pair_minimum_first_candidate_id.begin(),
                graph.pair_minimum_first_candidate_id.end(),
                message.pair_minimum_first_candidate_id.begin());
            std::copy(
                graph.pair_minimum_second_candidate_id.begin(),
                graph.pair_minimum_second_candidate_id.end(),
                message.pair_minimum_second_candidate_id.begin());
            std::copy(
                graph.pair_edge_required.begin(),
                graph.pair_edge_required.end(),
                message.pair_edge_required.begin());
            message.adjacency_bitmask = graph.adjacency_bitmask;
            std::copy(
                graph.component_ids.begin(),
                graph.component_ids.end(),
                message.component_ids.begin());
            std::copy(
                graph.component_sizes.begin(),
                graph.component_sizes.end(),
                message.component_sizes.begin());
            message.component_count = graph.component_count;
            message.edge_count = graph.edge_count;
            message.naive_evaluation_count = graph.naive_evaluation_count;
            message.component_evaluation_count =
                graph.component_evaluation_count;
            message.trajectory_generation_count =
                graph.trajectory_generation_count;
            message.pairwise_ad_evaluation_count =
                graph.pairwise_ad_evaluation_count;
            message.certification_hash = graph.certification_hash;
            message.graph_hash = graph.graph_hash;
            message.component_hash = graph.component_hash;
            message.evaluation_status = static_cast<std::uint8_t>(
                diagnostics.status);
            message.component_search_evaluated =
                diagnostics.component_search_evaluated;
            message.candidate_ready_mask = diagnostics.candidate_ready_mask;
            std::copy(
                diagnostics.candidate_counts.begin(),
                diagnostics.candidate_counts.end(),
                message.candidate_counts.begin());
            std::copy(
                diagnostics.candidate_source_timestamps_us.begin(),
                diagnostics.candidate_source_timestamps_us.end(),
                message.candidate_source_timestamps_us.begin());
            message.dropped_ownship_belief_count =
                diagnostics.dropped_ownship_belief_count;
            message.dropped_remote_intent_count =
                diagnostics.dropped_remote_intent_count;
            message.dropped_remote_decision_count =
                diagnostics.dropped_remote_decision_count;
            std::copy(
                diagnostics.assembled_candidate_ids.begin(),
                diagnostics.assembled_candidate_ids.end(),
                message.assembled_candidate_ids.begin());
            message.assembled_candidate_valid_mask =
                diagnostics.assembled_candidate_valid_mask;
            message.assembled_candidate_hash =
                diagnostics.assembled_candidate_hash;
            message.component_solution_hash =
                diagnostics.component_solution_hash;
            message.global_crosscheck_evaluated =
                diagnostics.global_crosscheck_evaluated;
            message.global_crosscheck_pass =
                diagnostics.global_crosscheck_pass;
            message.global_crosscheck_minimum_ad_m = static_cast<float>(
                diagnostics.global_crosscheck_minimum_ad_m);
            message.certification_compute_time_ns =
                graph.certification_compute_time_ns;
            message.graph_compute_time_ns = graph.graph_compute_time_ns;
            message.component_search_time_ns =
                diagnostics.component_search_time_ns;
            message.global_crosscheck_time_ns =
                diagnostics.global_crosscheck_time_ns;
            message.total_evaluation_time_ns =
                diagnostics.total_evaluation_time_ns;
    return message;
}

template<class Message>
inline void writeStoppedMessage(std::ostream & out, std::uint64_t timestamp,
                                const Message & message) {
    rclcpp::Serialization<Message> codec;
    rclcpp::SerializedMessage bytes;
    codec.serialize_message(&message, &bytes);
    const auto & raw = bytes.get_rcl_serialized_message();
    constexpr char hex[] = "0123456789abcdef";
    out << "[stop-observation]," << timestamp << ',';
    for (std::size_t i=0; i<raw.buffer_length; ++i)
        out << hex[raw.buffer[i] >> 4] << hex[raw.buffer[i] & 15];
    out << '\n';
}
inline void writeStoppedBudget(std::ostream & out, int vehicle, const char * owner,
                              const selection::StoppedBudgetRecords * records) {
    if (!records) return;
    out << "[stop-observation-begin],1," << vehicle << ',' << owner
        << ",/common/px4_" << vehicle << "/maneuver_budget_trace,collision_avoidance/msg/ManeuverBudgetTrace,"
        << records->size << ',' << records->dropped << '\n';
    for (std::size_t i=0; i<records->size; ++i)
        writeStoppedMessage(out, records->records[i].wall_ns, budgetTraceMessage(records->records[i]));
    out << "[stop-observation-end]," << vehicle << ',' << owner << ',' << records->size << '\n';
}
}
