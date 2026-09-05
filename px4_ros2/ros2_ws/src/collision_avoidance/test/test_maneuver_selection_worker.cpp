#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>

namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

// Past-state compensation must not use any of the hypothetical future inputs.
// These fixtures use the existing public worker queue and production propagator.
namespace {
cs::ManeuverSelectionPublishedSetpointSnapshot publishedInput(
    std::uint64_t stamp, double alat, double speed = 20.0, bool valid = true)
{
    return {stamp, {speed, std::numeric_limits<double>::quiet_NaN(), 0.0, alat}, valid};
}

void expectPacketInitialState(
    const cs::ManeuverSelectionWorkerOutput & output,
    const ce::PredictState & state, const ce::PredictStateCovariance & covariance)
{
    ASSERT_GT(output.intent_packet_count, 0U);
    const std::array<double, 7> mean{state.p_n,state.p_e,state.h,state.V,
                                   state.psi,state.h_dot,state.phi};
    for (std::size_t i = 0; i < output.intent_packet_count; ++i) {
        for (std::size_t k = 0; k < mean.size(); ++k)
            EXPECT_FLOAT_EQ(output.intent_packets[i].initial_state[k], float(mean[k]));
        for (std::size_t k = 0; k < covariance.size(); ++k)
            EXPECT_NEAR(output.intent_packets[i].initial_covariance[k], covariance[k],
                1.0e-10 + 4.0 * std::numeric_limits<float>::epsilon()
                    * std::abs(covariance[k]));
    }
}
} // namespace

TEST(ManeuverExecutionPolicy, SeparatesAmacActivationFromContinuousV4)
{
    cs::ManeuverSelectionDecision decision;
    decision.coordination_qualified = true;
    decision.ownship_candidate_valid = true;
    decision.selected_v4_cutover = true;
    decision.activation_requested = false;

    EXPECT_FALSE(cs::maneuverCommandExecutionRequested(
        cs::ManeuverExecutionPolicy::AmacAdThreshold, decision));
    EXPECT_TRUE(cs::maneuverCommandExecutionRequested(
        cs::ManeuverExecutionPolicy::ContinuousV4, decision));
    EXPECT_FALSE(cs::maneuverCommandExecutionRequested(
        cs::ManeuverExecutionPolicy::HorizonGatedV4, decision));

    decision.v4_horizon_gate_active = true;
    EXPECT_TRUE(cs::maneuverCommandExecutionRequested(
        cs::ManeuverExecutionPolicy::HorizonGatedV4, decision));

    decision.coordination_qualified = false;
    EXPECT_FALSE(cs::maneuverCommandExecutionRequested(
        cs::ManeuverExecutionPolicy::ContinuousV4, decision));

    decision.coordination_qualified = true;
    decision.ownship_candidate_valid = false;
    EXPECT_FALSE(cs::maneuverCommandExecutionRequested(
        cs::ManeuverExecutionPolicy::ContinuousV4, decision));
}

TEST(FormationActivationGate, RequiresTargetAndCurrentSpacingAboveHardBudget)
{
    EXPECT_TRUE(cs::formationSpacingCompatible(30.0, 25.0, 20.0));
    EXPECT_FALSE(cs::formationSpacingCompatible(20.0, 25.0, 20.0));
    EXPECT_FALSE(cs::formationSpacingCompatible(30.0, 20.0, 20.0));
    EXPECT_FALSE(cs::formationSpacingCompatible(19.0, 25.0, 20.0));
    EXPECT_FALSE(cs::formationSpacingCompatible(30.0, 19.0, 20.0));
    EXPECT_FALSE(cs::formationSpacingCompatible(
        std::numeric_limits<double>::quiet_NaN(), 25.0, 20.0));
    EXPECT_FALSE(cs::formationSpacingCompatible(
        30.0, std::numeric_limits<double>::infinity(), 20.0));
}

TEST(ManeuverExecutionPolicy,
    ContinuousV4TreatsOnlyCandidateRoleChangesAsSwitches)
{
    std::array<std::uint8_t, cs::kMaximumSelectionAircraft> selected{};
    std::array<std::uint8_t, cs::kMaximumSelectionAircraft> proposed{};
    selected[0] = 1;
    proposed[0] = 1;

    EXPECT_FALSE(cs::continuousV4RoleChanged(
        cs::ManeuverExecutionPolicy::ContinuousV4,
        true,
        true,
        selected,
        proposed,
        2));

    proposed[1] = 2;
    EXPECT_TRUE(cs::continuousV4RoleChanged(
        cs::ManeuverExecutionPolicy::ContinuousV4,
        true,
        true,
        selected,
        proposed,
        2));
    EXPECT_FALSE(cs::continuousV4RoleChanged(
        cs::ManeuverExecutionPolicy::AmacAdThreshold,
        true,
        true,
        selected,
        proposed,
        2));
}

TEST(ManeuverExecutionPolicy, HorizonGateUsesWorstClearanceThreshold)
{
    EXPECT_FALSE(cs::updateV4HorizonGateState(
        false, true, 10.001, 10.0));
    EXPECT_TRUE(cs::updateV4HorizonGateState(
        false, true, 10.0, 10.0));
    EXPECT_TRUE(cs::updateV4HorizonGateState(
        false, true, -2.0, 10.0));
    EXPECT_TRUE(cs::updateV4HorizonGateState(
        true,
        false,
        std::numeric_limits<double>::quiet_NaN(),
        10.0));
    EXPECT_FALSE(cs::updateV4HorizonGateState(
        false,
        false,
        std::numeric_limits<double>::quiet_NaN(),
        10.0));

    constexpr std::uint64_t activation_us = 1'000'000ULL;
    constexpr std::uint64_t horizon_us = 4'500'000ULL;
    EXPECT_FALSE(cs::v4HorizonHoldElapsed(
        activation_us, activation_us + horizon_us - 1, horizon_us));
    EXPECT_TRUE(cs::v4HorizonHoldElapsed(
        activation_us, activation_us + horizon_us, horizon_us));
    EXPECT_FALSE(cs::v4HorizonHoldElapsed(
        activation_us, activation_us - 1, horizon_us));

    EXPECT_TRUE(cs::v4HorizonFailClosedRequested(
        cs::ManeuverExecutionPolicy::HorizonGatedV4,
        cs::SafeControlSetStatus::SearchSetInfeasible));
    EXPECT_FALSE(cs::v4HorizonFailClosedRequested(
        cs::ManeuverExecutionPolicy::HorizonGatedV4,
        cs::SafeControlSetStatus::Valid));
    EXPECT_FALSE(cs::v4HorizonFailClosedRequested(
        cs::ManeuverExecutionPolicy::ContinuousV4,
        cs::SafeControlSetStatus::SearchSetInfeasible));
}

namespace
{

cs::ManeuverSelectionBeliefSnapshot beliefSnapshot(
    std::uint64_t timestamp_us,
    double north,
    double east,
    double velocity_north,
    double velocity_east)
{
    cs::ManeuverSelectionBeliefSnapshot snapshot;
    snapshot.timestamp_us = timestamp_us;
    snapshot.timestamp_sample_us = timestamp_us;
    snapshot.valid = true;
    snapshot.belief.attitude_q = {1.0, 0.0, 0.0, 0.0};
    snapshot.belief.position_ned = {north, east, -100.0};
    snapshot.belief.velocity_ned = {
        velocity_north, velocity_east, 0.0};
    for (std::size_t index = 0; index < ce::kEstimatorBeliefDimension; ++index) {
        snapshot.belief.covariance[
            index * ce::kEstimatorBeliefDimension + index] = 0.01;
    }
    return snapshot;
}

cs::ManeuverSelectionWorkerParams params(
    int vehicle_id = 0,
    int total_agent_count = 2)
{
    cs::ManeuverSelectionWorkerParams value;
    value.vehicle_id = vehicle_id;
    value.total_agent_count = total_agent_count;
    value.predictor_params.V_min = 10.0;
    value.predictor_params.V_max = 25.0;
    value.evaluator_params.desired_separation_distance_m = 10.0;
    value.evaluator_params.ownship_half_wingspan_m = 1.072;
    value.evaluator_params.threat_half_wingspan_m = 1.072;
    return value;
}

cs::ManeuverSelectionAirspeedSnapshot airspeedSnapshot(
    std::uint64_t timestamp_us,
    double true_airspeed_mps,
    bool valid = true)
{
    cs::ManeuverSelectionAirspeedSnapshot snapshot;
    snapshot.timestamp_us = timestamp_us;
    snapshot.true_airspeed_mps = true_airspeed_mps;
    snapshot.px4_airspeed_source = 1;
    snapshot.valid = valid;
    return snapshot;
}

cs::ManeuverSelectionNominalSetpointSnapshot nominalSnapshot(
    std::uint64_t timestamp_us,
    double ground_speed_command_mps = 20.0,
    double lateral_acceleration_mps2 = 0.0)
{
    cs::ManeuverSelectionNominalSetpointSnapshot snapshot;
    snapshot.timestamp_us = timestamp_us;
    snapshot.ground_speed_command_mps = ground_speed_command_mps;
    snapshot.altitude_command_m = 100.0;
    snapshot.lateral_acceleration_px4_mps2 =
        lateral_acceleration_mps2;
    snapshot.valid = true;
    return snapshot;
}

void pushV4Inputs(
    cs::ManeuverSelectionWorker & worker,
    std::uint64_t timestamp_us,
    double true_airspeed_mps = 20.0,
    double lateral_acceleration_mps2 = 0.0)
{
    ASSERT_TRUE(worker.pushAirspeed(
        airspeedSnapshot(timestamp_us, true_airspeed_mps)));
    ASSERT_TRUE(worker.pushNominalSetpoint(
        nominalSnapshot(
            timestamp_us, 20.0, lateral_acceleration_mps2)));
}

cs::ManeuverSelectionWorkerOutput pushBeliefAndProcess(
    cs::ManeuverSelectionWorker & worker,
    const cs::ManeuverSelectionBeliefSnapshot & belief)
{
    EXPECT_TRUE(worker.pushOwnshipBelief(belief));
    EXPECT_TRUE(worker.processPendingForTest());
    const auto output = worker.tryPopOutput();
    EXPECT_TRUE(output.has_value());
    return output.value_or(cs::ManeuverSelectionWorkerOutput{});
}

void exchangePackets(
    cs::ManeuverSelectionWorker & first,
    cs::ManeuverSelectionWorker & second,
    const cs::ManeuverSelectionWorkerOutput & first_output,
    const cs::ManeuverSelectionWorkerOutput & second_output)
{
    for (std::size_t index = 0;
         index < first_output.intent_packet_count; ++index) {
        ASSERT_TRUE(second.pushRemoteIntent(0, first_output.intent_packets[index]));
    }
    for (std::size_t index = 0;
         index < second_output.intent_packet_count; ++index) {
        ASSERT_TRUE(first.pushRemoteIntent(1, second_output.intent_packets[index]));
    }
    if (second_output.intent_packet_count > 0) {
        EXPECT_TRUE(first.processPendingForTest());
    }
    if (first_output.intent_packet_count > 0) {
        EXPECT_TRUE(second.processPendingForTest());
    }
}

cs::ManeuverSelectionPeerDecision peerDecision(
    const cs::ManeuverSelectionDecision & decision)
{
    cs::ManeuverSelectionPeerDecision peer;
    peer.vehicle_id = decision.vehicle_id;
    peer.selection_timestamp_us = decision.selection_timestamp_us;
    peer.local_selection_epoch = decision.local_selection_epoch;
    peer.selected_candidate_ids = decision.selected_candidate_ids;
    peer.selected_candidate_valid_mask =
        decision.selected_candidate_valid_mask;
    peer.selected_candidate_input_revisions =
        decision.selected_candidate_input_revisions;
    peer.selected_candidate_source_timestamps_us =
        decision.selected_candidate_source_timestamps_us;
    peer.selected_v4_cutover = decision.selected_v4_cutover;
    peer.ownship_candidate_id = decision.ownship_candidate_id;
    peer.ownship_candidate_valid = decision.ownship_candidate_valid;
    peer.proposal_timestamp_us = decision.proposal_timestamp_us;
    peer.proposal_epoch = decision.proposal_epoch;
    peer.proposed_candidate_ids = decision.proposed_candidate_ids;
    peer.proposed_candidate_valid_mask =
        decision.proposed_candidate_valid_mask;
    peer.proposed_candidate_input_revisions =
        decision.proposed_candidate_input_revisions;
    peer.proposed_candidate_source_timestamps_us =
        decision.proposed_candidate_source_timestamps_us;
    peer.proposed_v4_cutover = decision.proposed_v4_cutover;
    peer.proposed_component_graph = decision.proposed_component_graph;
    peer.proposed_candidate_library_hash =
        decision.proposed_candidate_library_hash;
    peer.proposed_graph_hash = decision.proposed_graph_hash;
    peer.proposed_component_hash = decision.proposed_component_hash;
    peer.proposed_component_solution_hash =
        decision.proposed_component_solution_hash;
    peer.proposal_valid = decision.proposal_valid;
    peer.proposal_consensus_confirmed =
        decision.proposal_consensus_confirmed;
    peer.coordination_qualified = decision.coordination_qualified;
    peer.activation_requested = decision.activation_requested;
    peer.activation_just_started = decision.activation_just_started;
    peer.command_execution_requested =
        decision.command_execution_requested;
    peer.nominal_setpoint_available =
        decision.nominal_setpoint_available;
    peer.nominal_setpoint_timestamp_us =
        decision.nominal_setpoint_timestamp_us;
    peer.nominal_ground_speed_command_mps =
        decision.nominal_ground_speed_command_mps;
    peer.nominal_altitude_command_m =
        decision.nominal_altitude_command_m;
    peer.nominal_lateral_acceleration_mps2 =
        decision.nominal_lateral_acceleration_mps2;
    peer.post_release_evaluated = decision.post_release_evaluated;
    peer.post_release_safe = decision.post_release_safe;
    peer.post_release_evaluation_timestamp_us =
        decision.post_release_evaluation_timestamp_us;
    peer.v4_control_architecture = decision.v4_control_architecture;
    peer.v4_horizon_local_gate_active =
        decision.v4_horizon_local_gate_active;
    peer.v4_cutover_candidate_ready =
        cs::v4CutoverCandidateReady(decision);
    return peer;
}

cs::ManeuverSelectionPeerDecision coordinatedPeerForIntent(
    int vehicle_id,
    const ce::TrajectoryIntentPacket & packet)
{
    cs::ManeuverSelectionPeerDecision peer;
    const std::size_t vehicle_index = static_cast<std::size_t>(vehicle_id);
    peer.vehicle_id = vehicle_id;
    peer.coordination_qualified = true;
    peer.ownship_candidate_id = packet.candidate_id;
    peer.ownship_candidate_valid = true;
    peer.selected_candidate_valid_mask =
        std::uint32_t{1} << vehicle_index;
    peer.selected_candidate_ids[vehicle_index] = packet.candidate_id;
    peer.selected_candidate_input_revisions[vehicle_index] =
        packet.candidate_input_revision;
    peer.selected_candidate_source_timestamps_us[vehicle_index] =
        packet.source_timestamp_us;
    peer.selected_v4_cutover = packet.candidate_set_kind
        == ce::CandidateSetKind::V4SafeControl;
    return peer;
}

cs::ManeuverSelectionPeerDecision nominalPeerDecision(
    int vehicle_id,
    std::uint64_t timestamp_us)
{
    cs::ManeuverSelectionPeerDecision peer;
    peer.vehicle_id = vehicle_id;
    peer.nominal_setpoint_available = true;
    peer.nominal_setpoint_timestamp_us = timestamp_us;
    peer.nominal_ground_speed_command_mps = 20.0;
    peer.nominal_altitude_command_m = 100.0;
    peer.nominal_lateral_acceleration_mps2 = 0.0;
    return peer;
}

std::array<cs::ManeuverSelectionWorkerOutput, 2> confirmTwoAircraftProposal(
    cs::ManeuverSelectionWorker & first,
    cs::ManeuverSelectionWorker & second,
    const cs::ManeuverSelectionWorkerOutput & first_proposal,
    const cs::ManeuverSelectionWorkerOutput & second_proposal)
{
    EXPECT_TRUE(first_proposal.has_decision);
    EXPECT_TRUE(second_proposal.has_decision);
    EXPECT_TRUE(first_proposal.decision.proposal_valid);
    EXPECT_TRUE(second_proposal.decision.proposal_valid);
    EXPECT_TRUE(first.pushRemoteDecision(
        1, peerDecision(second_proposal.decision)));
    EXPECT_TRUE(second.pushRemoteDecision(
        0, peerDecision(first_proposal.decision)));
    EXPECT_TRUE(first.processPendingForTest());
    EXPECT_TRUE(second.processPendingForTest());
    const auto first_commit = first.tryPopOutput();
    const auto second_commit = second.tryPopOutput();
    EXPECT_TRUE(first_commit.has_value());
    EXPECT_TRUE(second_commit.has_value());
    return {
        first_commit.value_or(cs::ManeuverSelectionWorkerOutput{}),
        second_commit.value_or(cs::ManeuverSelectionWorkerOutput{})};
}

template<std::size_t AircraftCount>
std::array<cs::ManeuverSelectionWorkerOutput, AircraftCount>
confirmAllAircraftProposals(
    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, AircraftCount>
        & workers,
    const std::array<cs::ManeuverSelectionWorkerOutput, AircraftCount>
        & proposals)
{
    for (std::size_t sender = 0; sender < AircraftCount; ++sender) {
        EXPECT_TRUE(proposals[sender].has_decision);
        EXPECT_TRUE(proposals[sender].decision.proposal_valid);
        for (std::size_t receiver = 0; receiver < AircraftCount; ++receiver) {
            if (sender == receiver) {
                continue;
            }
            EXPECT_TRUE(workers[receiver]->pushRemoteDecision(
                static_cast<int>(sender),
                peerDecision(proposals[sender].decision)));
        }
    }

    std::array<cs::ManeuverSelectionWorkerOutput, AircraftCount> commits{};
    for (std::size_t aircraft = 0; aircraft < AircraftCount; ++aircraft) {
        EXPECT_TRUE(workers[aircraft]->processPendingForTest());
        const auto output = workers[aircraft]->tryPopOutput();
        EXPECT_TRUE(output.has_value());
        commits[aircraft] = output.value_or(
            cs::ManeuverSelectionWorkerOutput{});
    }
    return commits;
}

}  // namespace

TEST(ManeuverSelectionWorker, V4ShadowReportsMissingPeerWithoutChangingIntents)
{
    auto worker_params = params();
    worker_params.v4_safe_control_enabled = true;
    cs::ManeuverSelectionWorker worker(worker_params);
    constexpr std::uint64_t timestamp_us = 500'000ULL;
    pushV4Inputs(worker, timestamp_us, 21.0, 1.5);

    const auto output = pushBeliefAndProcess(
        worker,
        beliefSnapshot(timestamp_us, 0.0, 0.0, 20.0, 0.0));

    ASSERT_TRUE(output.has_decision);
    ASSERT_EQ(output.intent_packet_count, 3U);
    EXPECT_TRUE(output.decision.v4_enabled);
    EXPECT_TRUE(output.decision.v4_shadow_only);
    EXPECT_FALSE(output.decision.v4_shadow_evaluated);
    EXPECT_EQ(
        output.decision.v4_shadow_status,
        cs::V4ShadowEvaluationStatus::MissingPeerDecision);
    EXPECT_EQ(
        output.decision.v4_airspeed_snapshot_status,
        cs::V4SnapshotStatus::Valid);
    EXPECT_EQ(
        output.decision.v4_airspeed_source,
        cs::V4AirspeedSource::ActualTas);
    EXPECT_TRUE(output.decision.v4_nominal_available);
    EXPECT_FALSE(output.decision.coordination_qualified);
    EXPECT_FALSE(output.decision.activation_requested);
}

TEST(ManeuverSelectionWorker, V4ShadowEvaluatesFreshSelectedPeerIntent)
{
    constexpr std::uint64_t timestamp_us = 750'000ULL;
    cs::ManeuverSelectionWorker remote(params(1));
    const auto remote_output = pushBeliefAndProcess(
        remote,
        beliefSnapshot(timestamp_us, 100.0, 0.0, -20.0, 0.0));
    ASSERT_EQ(remote_output.intent_packet_count, 3U);

    auto local_params = params();
    local_params.v4_safe_control_enabled = true;
    cs::ManeuverSelectionWorker local(local_params);
    for (const auto & packet : remote_output.intent_packets) {
        ASSERT_TRUE(local.pushRemoteIntent(1, packet));
    }
    const auto peer = coordinatedPeerForIntent(
        1, remote_output.intent_packets[0]);
    ASSERT_TRUE(local.pushRemoteDecision(1, peer));
    pushV4Inputs(local, timestamp_us, 20.0, -2.0);

    const auto output = pushBeliefAndProcess(
        local,
        beliefSnapshot(timestamp_us, -100.0, 0.0, 20.0, 0.0));

    ASSERT_TRUE(output.has_decision);
    EXPECT_TRUE(output.decision.v4_shadow_evaluated);
    EXPECT_EQ(
        output.decision.v4_shadow_status,
        cs::V4ShadowEvaluationStatus::CoreEvaluated);
    EXPECT_EQ(
        output.decision.v4_safe_control.status,
        cs::SafeControlSetStatus::Valid);
    EXPECT_EQ(output.decision.v4_safe_control.evaluated_threat_count, 1U);
    EXPECT_EQ(
        output.decision.v4_airspeed_source,
        cs::V4AirspeedSource::ActualTas);
    EXPECT_TRUE(output.decision.v4_nominal_available);
    ASSERT_EQ(
        output.decision.v4_candidates.status,
        cs::SafeControlCandidateAdapterStatus::Valid);
    EXPECT_LE(output.decision.v4_candidates.candidate_count, 3U);
    ASSERT_GT(output.decision.v4_candidates.candidate_count, 0U);
    EXPECT_EQ(
        output.decision.v4_candidates.candidates[0].role,
        cs::SafeCandidateRole::NearNominal);

    // Step 3 is diagnostic-only: legacy intent IDs and command state remain.
    ASSERT_EQ(output.intent_packet_count, 3U);
    EXPECT_FALSE(output.decision.coordination_qualified);
    EXPECT_FALSE(output.decision.activation_requested);

    const double first_maximum_rate =
        output.decision.v4_safe_control.effective_max_heading_rate_radps;
    constexpr std::uint64_t next_timestamp_us = timestamp_us + 50'000ULL;
    pushV4Inputs(local, next_timestamp_us, 15.0, -2.0);
    const auto next_output = pushBeliefAndProcess(
        local,
        beliefSnapshot(next_timestamp_us, -99.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(next_output.has_decision);
    EXPECT_TRUE(next_output.decision.v4_shadow_evaluated);
    EXPECT_GT(
        next_output.decision.v4_safe_control
            .effective_max_heading_rate_radps,
        first_maximum_rate);
}

TEST(ManeuverSelectionWorker, ModeBShadowUsesAlignedIntentAndRealInterpolator)
{
    constexpr std::uint64_t timestamp_us = 825'000ULL;
    cs::ManeuverSelectionWorker remote(params(1));
    const auto remote_output = pushBeliefAndProcess(
        remote,
        beliefSnapshot(timestamp_us, 0.0, 200.0, 20.0, 0.0));
    ASSERT_EQ(remote_output.intent_packet_count, 3U);

    auto local_params = params();
    local_params.v4_safe_control_enabled = true;
    local_params.v4_control_architecture =
        cs::V4ControlArchitecture::ClosedFormBackupModeB;
    local_params.mode_b_interpolator_params.certifier.reference_margin_m =
        local_params.evaluator_params.desired_separation_distance_m;
    local_params.mode_b_intent_adapter_params.predictor =
        local_params.predictor_params;
    cs::ManeuverSelectionWorker local(local_params);
    for (const auto & packet : remote_output.intent_packets) {
        ASSERT_TRUE(local.pushRemoteIntent(1, packet));
    }
    ASSERT_TRUE(local.pushRemoteDecision(
        1, coordinatedPeerForIntent(1, remote_output.intent_packets[0])));
    pushV4Inputs(local, timestamp_us, 20.0, 1.0);

    const auto output = pushBeliefAndProcess(
        local,
        beliefSnapshot(timestamp_us, 0.0, 0.0, 20.0, 0.0));

    ASSERT_TRUE(output.has_decision);
    EXPECT_EQ(
        output.decision.v4_control_architecture,
        cs::V4ControlArchitecture::ClosedFormBackupModeB);
    EXPECT_TRUE(output.decision.v4_shadow_evaluated);
    EXPECT_EQ(
        output.decision.v4_shadow_status,
        cs::V4ShadowEvaluationStatus::CoreEvaluated);
    EXPECT_EQ(
        output.decision.mode_b_threat_status,
        cs::BackupThreatIntentStatusV4::Valid);
    EXPECT_EQ(
        output.decision.mode_b_interpolation_status,
        cs::BackupInterpolationStatusV4::Valid);
    EXPECT_TRUE(
        output.decision.mode_b_left_certified
        || output.decision.mode_b_right_certified);
    ASSERT_EQ(
        output.decision.v4_candidates.status,
        cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_GT(output.decision.v4_candidates.candidate_count, 0U);
    for (std::size_t index = 0;
         index < output.decision.v4_candidates.candidate_count; ++index) {
        const auto & candidate = output.decision.v4_candidates.candidates[index];
        EXPECT_DOUBLE_EQ(candidate.predictor_input.h_dot_cmd, 0.0);
        EXPECT_NEAR(
            candidate.predictor_input.a_lat_cmd,
            -20.0 * candidate.heading_rate_v4_radps,
            1.0e-12);
    }
    // Shadow mode does not replace the existing published candidate family.
    EXPECT_EQ(
        output.intent_packets[0].candidate_set_kind,
        ce::CandidateSetKind::LegacyRoll);
}

TEST(ManeuverSelectionWorker,
    ModeBDistributedBootstrapCommitsAndRequestsContinuousExecution)
{
    const auto mode_b_params = [](const int vehicle_id) {
        auto value = params(vehicle_id);
        value.v4_safe_control_enabled = true;
        value.v4_shadow_only = false;
        value.v4_control_architecture =
            cs::V4ControlArchitecture::ClosedFormBackupModeB;
        value.execution_policy = cs::ManeuverExecutionPolicy::ContinuousV4;
        value.active_switching_enabled = true;
        value.active_switch_cost_margin = 1.0e-9;
        value.active_switch_minimum_ad_margin_m = 1.0e-9;
        value.mode_b_interpolator_params.certifier.reference_margin_m =
            value.evaluator_params.desired_separation_distance_m;
        value.mode_b_intent_adapter_params.predictor = value.predictor_params;
        return value;
    };

    // Match runtime ownership: two large worker objects must not share the
    // test thread's 8 MiB stack with nested trajectory-evaluation scratch data.
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(mode_b_params(0));
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(mode_b_params(1));
    auto & first = *first_storage;
    auto & second = *second_storage;
    constexpr std::uint64_t start = 1'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        pushV4Inputs(first, start + offset);
        pushV4Inputs(second, start + offset);
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset,
                20.0 * static_cast<double>(offset) * 1.0e-6,
                0.0,
                20.0,
                0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset,
                20.0 * static_cast<double>(offset) * 1.0e-6,
                200.0,
                20.0,
                0.0));
        exchangePackets(first, second, first_output, second_output);
    }

    constexpr std::uint64_t bootstrap_offset = 250'000ULL;
    pushV4Inputs(first, start + bootstrap_offset);
    pushV4Inputs(second, start + bootstrap_offset);
    first_output = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + bootstrap_offset, 5.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second,
        beliefSnapshot(start + bootstrap_offset, 5.0, 200.0, 20.0, 0.0));
    const auto bootstrap = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    ASSERT_TRUE(bootstrap[0].decision.coordination_qualified);
    ASSERT_TRUE(bootstrap[1].decision.coordination_qualified);
    ASSERT_FALSE(bootstrap[0].decision.selected_v4_cutover);
    ASSERT_FALSE(bootstrap[1].decision.selected_v4_cutover);
    ASSERT_TRUE(first.pushRemoteDecision(
        1, peerDecision(bootstrap[1].decision)));
    ASSERT_TRUE(second.pushRemoteDecision(
        0, peerDecision(bootstrap[0].decision)));

    constexpr std::uint64_t readiness_offset = 300'000ULL;
    pushV4Inputs(first, start + readiness_offset);
    pushV4Inputs(second, start + readiness_offset);
    first_output = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + readiness_offset, 6.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second,
        beliefSnapshot(start + readiness_offset, 6.0, 200.0, 20.0, 0.0));
    ASSERT_TRUE(cs::v4CutoverCandidateReady(first_output.decision));
    ASSERT_TRUE(cs::v4CutoverCandidateReady(second_output.decision));
    exchangePackets(first, second, first_output, second_output);
    ASSERT_TRUE(first.pushRemoteDecision(
        1, peerDecision(second_output.decision)));
    ASSERT_TRUE(second.pushRemoteDecision(
        0, peerDecision(first_output.decision)));
    ASSERT_TRUE(first.processPendingForTest());
    ASSERT_TRUE(second.processPendingForTest());

    for (const std::uint64_t offset : {
             350'000ULL, 400'000ULL, 450'000ULL}) {
        pushV4Inputs(first, start + offset);
        pushV4Inputs(second, start + offset);
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset,
                20.0 * static_cast<double>(offset) * 1.0e-6,
                0.0,
                20.0,
                0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset,
                20.0 * static_cast<double>(offset) * 1.0e-6,
                200.0,
                20.0,
                0.0));
        ASSERT_GT(first_output.intent_packet_count, 0U);
        ASSERT_GT(second_output.intent_packet_count, 0U);
        EXPECT_EQ(
            first_output.intent_packets[0].candidate_set_kind,
            ce::CandidateSetKind::V4SafeControl);
        EXPECT_EQ(
            second_output.intent_packets[0].candidate_set_kind,
            ce::CandidateSetKind::V4SafeControl);
        exchangePackets(first, second, first_output, second_output);
    }

    constexpr std::uint64_t selection_offset = 500'000ULL;
    pushV4Inputs(first, start + selection_offset);
    pushV4Inputs(second, start + selection_offset);
    second_output = pushBeliefAndProcess(
        second,
        beliefSnapshot(start + selection_offset, 10.0, 200.0, 20.0, 0.0));
    exchangePackets(first, second, first_output, second_output);
    first_output = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + selection_offset, 10.0, 0.0, 20.0, 0.0));
    exchangePackets(first, second, first_output, second_output);
    ASSERT_TRUE(first_output.decision.proposal_valid);
    ASSERT_TRUE(second_output.decision.proposal_valid);
    ASSERT_TRUE(first_output.decision.proposed_v4_cutover);
    ASSERT_TRUE(second_output.decision.proposed_v4_cutover);

    const auto committed = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    for (const auto & output : committed) {
        ASSERT_TRUE(output.decision.selected_v4_cutover);
        EXPECT_FALSE(output.decision.activation_requested);
        EXPECT_TRUE(output.decision.command_execution_requested);
    }

}

TEST(ManeuverSelectionWorker, V4ShadowUsesTrimAndOmitsStaleNominal)
{
    constexpr std::uint64_t source_timestamp_us = 1'000'000ULL;
    constexpr std::uint64_t evaluation_timestamp_us = 1'150'000ULL;
    cs::ManeuverSelectionWorker remote(params(1));
    const auto remote_output = pushBeliefAndProcess(
        remote,
        beliefSnapshot(source_timestamp_us, 100.0, 0.0, -20.0, 0.0));

    auto local_params = params();
    local_params.v4_safe_control_enabled = true;
    local_params.v4_trim_airspeed_mps = 16.0;
    local_params.v4_maximum_airspeed_age_us = 100'000;
    local_params.v4_maximum_nominal_age_us = 100'000;
    cs::ManeuverSelectionWorker local(local_params);
    for (const auto & packet : remote_output.intent_packets) {
        ASSERT_TRUE(local.pushRemoteIntent(1, packet));
    }
    const auto peer = coordinatedPeerForIntent(
        1, remote_output.intent_packets[0]);
    ASSERT_TRUE(local.pushRemoteDecision(1, peer));
    pushV4Inputs(local, source_timestamp_us, 22.0, -2.0);

    const auto output = pushBeliefAndProcess(
        local,
        beliefSnapshot(
            evaluation_timestamp_us, -97.0, 0.0, 20.0, 0.0));

    ASSERT_TRUE(output.has_decision);
    EXPECT_TRUE(output.decision.v4_shadow_evaluated);
    EXPECT_EQ(
        output.decision.v4_airspeed_snapshot_status,
        cs::V4SnapshotStatus::Stale);
    EXPECT_EQ(
        output.decision.v4_airspeed_source,
        cs::V4AirspeedSource::TrimFallback);
    EXPECT_EQ(output.decision.v4_airspeed_age_us, 150'000U);
    EXPECT_EQ(
        output.decision.v4_nominal_snapshot_status,
        cs::V4SnapshotStatus::Stale);
    EXPECT_FALSE(output.decision.v4_nominal_available);
    for (std::size_t index = 0;
         index < output.decision.v4_candidates.candidate_count; ++index) {
        EXPECT_NE(
            output.decision.v4_candidates.candidates[index].role,
            cs::SafeCandidateRole::NearNominal);
    }
}

TEST(ManeuverSelectionWorker, V4ShadowRejectsFuturePeerIntentExplicitly)
{
    constexpr std::uint64_t evaluation_timestamp_us = 1'500'000ULL;
    constexpr std::uint64_t future_timestamp_us = 1'600'000ULL;
    cs::ManeuverSelectionWorker remote(params(1));
    const auto remote_output = pushBeliefAndProcess(
        remote,
        beliefSnapshot(future_timestamp_us, 100.0, 0.0, -20.0, 0.0));

    auto local_params = params();
    local_params.v4_safe_control_enabled = true;
    cs::ManeuverSelectionWorker local(local_params);
    for (const auto & packet : remote_output.intent_packets) {
        ASSERT_TRUE(local.pushRemoteIntent(1, packet));
    }
    const auto peer = coordinatedPeerForIntent(
        1, remote_output.intent_packets[0]);
    ASSERT_TRUE(local.pushRemoteDecision(1, peer));
    pushV4Inputs(local, evaluation_timestamp_us);

    const auto output = pushBeliefAndProcess(
        local,
        beliefSnapshot(
            evaluation_timestamp_us, -100.0, 0.0, 20.0, 0.0));

    ASSERT_TRUE(output.has_decision);
    EXPECT_FALSE(output.decision.v4_shadow_evaluated);
    EXPECT_EQ(
        output.decision.v4_shadow_status,
        cs::V4ShadowEvaluationStatus::FuturePeerIntent);
}

TEST(ManeuverSelectionWorker, V4ShadowClassifiesFutureTasAndInvalidNominal)
{
    constexpr std::uint64_t evaluation_timestamp_us = 1'800'000ULL;
    cs::ManeuverSelectionWorker remote(params(1));
    const auto remote_output = pushBeliefAndProcess(
        remote,
        beliefSnapshot(
            evaluation_timestamp_us, 100.0, 0.0, -20.0, 0.0));

    auto local_params = params();
    local_params.v4_safe_control_enabled = true;
    local_params.v4_trim_airspeed_mps = 16.0;
    cs::ManeuverSelectionWorker local(local_params);
    for (const auto & packet : remote_output.intent_packets) {
        ASSERT_TRUE(local.pushRemoteIntent(1, packet));
    }
    const auto peer = coordinatedPeerForIntent(
        1, remote_output.intent_packets[0]);
    ASSERT_TRUE(local.pushRemoteDecision(1, peer));
    ASSERT_TRUE(local.pushAirspeed(
        airspeedSnapshot(evaluation_timestamp_us + 10'000ULL, 24.0)));
    auto invalid_nominal = nominalSnapshot(evaluation_timestamp_us);
    invalid_nominal.ground_speed_command_mps =
        std::numeric_limits<double>::quiet_NaN();
    ASSERT_TRUE(local.pushNominalSetpoint(invalid_nominal));

    const auto output = pushBeliefAndProcess(
        local,
        beliefSnapshot(
            evaluation_timestamp_us, -100.0, 0.0, 20.0, 0.0));

    ASSERT_TRUE(output.has_decision);
    EXPECT_TRUE(output.decision.v4_shadow_evaluated);
    EXPECT_EQ(
        output.decision.v4_airspeed_snapshot_status,
        cs::V4SnapshotStatus::Future);
    EXPECT_EQ(
        output.decision.v4_airspeed_source,
        cs::V4AirspeedSource::TrimFallback);
    EXPECT_EQ(
        output.decision.v4_nominal_snapshot_status,
        cs::V4SnapshotStatus::Invalid);
    EXPECT_FALSE(output.decision.v4_nominal_available);
    for (std::size_t index = 0;
         index < output.decision.v4_candidates.candidate_count; ++index) {
        EXPECT_NE(
            output.decision.v4_candidates.candidates[index].role,
            cs::SafeCandidateRole::NearNominal);
    }
}

TEST(ManeuverSelectionWorker, HorizonGatedV4RequiresConeBarrierFilter)
{
    auto invalid_params = params();
    invalid_params.v4_safe_control_enabled = true;
    invalid_params.v4_shadow_only = false;
    invalid_params.execution_policy =
        cs::ManeuverExecutionPolicy::HorizonGatedV4;
    invalid_params.evaluator_params.robust_cone_filter_enabled = false;
    cs::ManeuverSelectionWorker worker(invalid_params);
    EXPECT_FALSE(worker.processPendingForTest());
    EXPECT_FALSE(worker.start());
    EXPECT_STREQ(
        cs::v4ShadowEvaluationStatusName(
            cs::V4ShadowEvaluationStatus::StalePeerIntent),
        "stale_peer_intent");
    EXPECT_STREQ(
        cs::v4AirspeedSourceName(cs::V4AirspeedSource::TrimFallback),
        "trim_fallback");
}

TEST(ManeuverSelectionWorker, ActiveSwitchingRequiresPositiveMargins)
{
    auto invalid_params = params();
    invalid_params.active_switching_enabled = true;
    invalid_params.active_switch_cost_margin = 0.0;
    invalid_params.active_switch_minimum_ad_margin_m = 0.0;
    cs::ManeuverSelectionWorker worker(invalid_params);
    EXPECT_FALSE(worker.processPendingForTest());
    EXPECT_FALSE(worker.start());
}

TEST(ManeuverSelectionWorker,
    V4CutoverUsesDynamicCandidatesOnlyAfterLegacyBootstrapConsensus)
{
    auto first_params = params();
    first_params.v4_safe_control_enabled = true;
    first_params.v4_shadow_only = false;
    first_params.evaluator_params.desired_separation_distance_m = 200.0;
    auto second_params = params(1);
    second_params.v4_safe_control_enabled = true;
    second_params.v4_shadow_only = false;
    second_params.evaluator_params.desired_separation_distance_m = 200.0;
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(
        first_params);
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(
        second_params);
    auto & first = *first_storage;
    auto & second = *second_storage;
    constexpr std::uint64_t start = 20'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        pushV4Inputs(first, start + offset);
        pushV4Inputs(second, start + offset);
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset,
                -100.0 + 20.0 * elapsed_s,
                0.0,
                20.0,
                0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset,
                100.0 - 20.0 * elapsed_s,
                0.0,
                -20.0,
                0.0));
        exchangePackets(first, second, first_output, second_output);
    }

    pushV4Inputs(first, start + 250'000ULL);
    pushV4Inputs(second, start + 250'000ULL);
    first_output = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + 250'000ULL, -95.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second,
        beliefSnapshot(start + 250'000ULL, 95.0, 0.0, -20.0, 0.0));
    auto bootstrap_commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    ASSERT_TRUE(bootstrap_commits[0].decision.coordination_qualified);
    ASSERT_TRUE(bootstrap_commits[1].decision.coordination_qualified);
    EXPECT_FALSE(bootstrap_commits[0].decision.selected_v4_cutover);
    EXPECT_FALSE(bootstrap_commits[1].decision.selected_v4_cutover);
    EXPECT_FALSE(bootstrap_commits[0].decision.activation_requested);
    EXPECT_FALSE(bootstrap_commits[1].decision.activation_requested);
    ASSERT_TRUE(first.pushRemoteDecision(
        1, peerDecision(bootstrap_commits[1].decision)));
    ASSERT_TRUE(second.pushRemoteDecision(
        0, peerDecision(bootstrap_commits[0].decision)));

    // Phase 1: both aircraft can independently generate a V4 candidate set,
    // but they keep broadcasting legacy intents until that readiness has
    // crossed the peer decision channel in both directions.
    constexpr std::uint64_t readiness_offset = 300'000ULL;
    pushV4Inputs(first, start + readiness_offset);
    pushV4Inputs(second, start + readiness_offset);
    first_output = pushBeliefAndProcess(
        first,
        beliefSnapshot(
            start + readiness_offset, -94.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second,
        beliefSnapshot(
            start + readiness_offset, 94.0, 0.0, -20.0, 0.0));
    ASSERT_TRUE(cs::v4CutoverCandidateReady(first_output.decision));
    ASSERT_TRUE(cs::v4CutoverCandidateReady(second_output.decision));
    ASSERT_GT(first_output.intent_packet_count, 0U);
    ASSERT_GT(second_output.intent_packet_count, 0U);
    EXPECT_EQ(
        first_output.intent_packets[0].candidate_set_kind,
        ce::CandidateSetKind::LegacyRoll);
    EXPECT_EQ(
        second_output.intent_packets[0].candidate_set_kind,
        ce::CandidateSetKind::LegacyRoll);
    exchangePackets(first, second, first_output, second_output);
    ASSERT_TRUE(first.pushRemoteDecision(
        1, peerDecision(second_output.decision)));
    ASSERT_TRUE(second.pushRemoteDecision(
        0, peerDecision(first_output.decision)));
    // A later 20 Hz diagnostic can be temporarily unready while the peer has
    // already demonstrated V4 bootstrap capability. Consuming both updates in
    // one worker cycle must not erase the readiness event.
    auto second_transient_not_ready = peerDecision(second_output.decision);
    second_transient_not_ready.v4_cutover_candidate_ready = false;
    auto first_transient_not_ready = peerDecision(first_output.decision);
    first_transient_not_ready.v4_cutover_candidate_ready = false;
    ASSERT_TRUE(first.pushRemoteDecision(1, second_transient_not_ready));
    ASSERT_TRUE(second.pushRemoteDecision(0, first_transient_not_ready));
    EXPECT_TRUE(first.processPendingForTest());
    EXPECT_TRUE(second.processPendingForTest());

    // Phase 2: after the all-participant readiness barrier, both aircraft
    // publish V4 intents. Actual command execution still waits for the normal
    // distributed proposal/confirmation below.
    std::array<std::uint64_t, 3> first_epoch_revisions{};
    bool captured_first_epoch_revisions = false;
    for (const auto & [offset, nominal_lateral_acceleration] :
         std::array<std::pair<std::uint64_t, double>, 3>{
             std::pair{350'000ULL, 0.0},
             std::pair{400'000ULL, 2.0},
             std::pair{450'000ULL, -2.0}}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        pushV4Inputs(
            first, start + offset, 20.0, nominal_lateral_acceleration);
        pushV4Inputs(
            second, start + offset, 20.0, nominal_lateral_acceleration);
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset,
                -100.0 + 20.0 * elapsed_s,
                0.0,
                20.0,
                0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset,
                100.0 - 20.0 * elapsed_s,
                0.0,
                -20.0,
                0.0));
        ASSERT_GT(first_output.intent_packet_count, 0U);
        ASSERT_GT(second_output.intent_packet_count, 0U);
        EXPECT_LE(first_output.intent_packet_count, 3U);
        EXPECT_LE(second_output.intent_packet_count, 3U);
        EXPECT_EQ(
            first_output.intent_packets[0].candidate_set_kind,
            ce::CandidateSetKind::V4SafeControl);
        EXPECT_EQ(
            second_output.intent_packets[0].candidate_set_kind,
            ce::CandidateSetKind::V4SafeControl);
        std::array<std::uint64_t, 3> current_revisions{};
        for (std::size_t index = 0;
             index < first_output.intent_packet_count; ++index) {
            current_revisions[index] =
                first_output.intent_packets[index].candidate_input_revision;
        }
        if (!captured_first_epoch_revisions) {
            first_epoch_revisions = current_revisions;
            captured_first_epoch_revisions = true;
        } else {
            EXPECT_EQ(current_revisions, first_epoch_revisions);
        }
        exchangePackets(first, second, first_output, second_output);
    }

    pushV4Inputs(first, start + 500'000ULL);
    pushV4Inputs(second, start + 500'000ULL);
    // Let the peer cross the epoch boundary first.  Its newly published set
    // advances the receiver's current cache, so evaluation of the preceding
    // epoch must use the true previous-set cache while the selected legacy
    // bootstrap intent remains independently retained for V4 evaluation.
    second_output = pushBeliefAndProcess(
        second,
        beliefSnapshot(start + 500'000ULL, 90.0, 0.0, -20.0, 0.0));
    for (std::size_t index = 0;
         index < second_output.intent_packet_count; ++index) {
        ASSERT_TRUE(first.pushRemoteIntent(
            1, second_output.intent_packets[index]));
    }
    first_output = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + 500'000ULL, -90.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(first_output.decision.proposal_valid);
    ASSERT_TRUE(second_output.decision.proposal_valid);
    EXPECT_TRUE(first_output.decision.proposed_v4_cutover);
    EXPECT_TRUE(second_output.decision.proposed_v4_cutover);

    auto first_transmitted_proposal = first_output;
    auto second_transmitted_proposal = second_output;
    // Each receiver may hold a different 20 Hz trajectory snapshot for the
    // other aircraft while still selecting the exact same command revision.
    // Snapshot time is therefore owner-canonicalized, not a command-consensus
    // key.
    first_transmitted_proposal.decision
        .proposed_candidate_source_timestamps_us[1] += 1;
    second_transmitted_proposal.decision
        .proposed_candidate_source_timestamps_us[0] += 2;
    const auto v4_commits = confirmTwoAircraftProposal(
        first,
        second,
        first_transmitted_proposal,
        second_transmitted_proposal);
    ASSERT_TRUE(v4_commits[0].decision.coordination_qualified);
    ASSERT_TRUE(v4_commits[1].decision.coordination_qualified);
    EXPECT_TRUE(v4_commits[0].decision.selected_v4_cutover);
    EXPECT_TRUE(v4_commits[1].decision.selected_v4_cutover);
    EXPECT_TRUE(v4_commits[0].decision.activation_requested);
    EXPECT_TRUE(v4_commits[1].decision.activation_requested);
    EXPECT_EQ(
        v4_commits[0].decision.selected_candidate_ids,
        v4_commits[1].decision.selected_candidate_ids);
    EXPECT_EQ(
        v4_commits[0].decision.selected_candidate_input_revisions,
        v4_commits[1].decision.selected_candidate_input_revisions);
    EXPECT_EQ(
        v4_commits[0].decision.selected_candidate_source_timestamps_us,
        v4_commits[1].decision.selected_candidate_source_timestamps_us);
    EXPECT_EQ(
        v4_commits[0].decision.selected_candidate_source_timestamps_us[0],
        first_output.decision.proposed_candidate_source_timestamps_us[0]);
    EXPECT_EQ(
        v4_commits[0].decision.selected_candidate_source_timestamps_us[1],
        second_output.decision.proposed_candidate_source_timestamps_us[1]);

    const std::uint8_t latched_role =
        v4_commits[0].decision.ownship_candidate_id;
    const std::uint64_t latched_revision =
        v4_commits[0].decision.selected_candidate_input_revisions[0];
    pushV4Inputs(first, start + 550'000ULL, 20.0, 4.0);
    const auto refreshed = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + 550'000ULL, -89.0, 0.0, 20.0, 0.0));
    ASSERT_GT(refreshed.intent_packet_count, 0U);
    EXPECT_TRUE(refreshed.decision.activation_requested);
    EXPECT_FALSE(refreshed.decision.activation_just_started);
    const auto retained = std::find_if(
        refreshed.intent_packets.begin(),
        refreshed.intent_packets.begin()
            + static_cast<std::ptrdiff_t>(refreshed.intent_packet_count),
        [latched_role](const ce::TrajectoryIntentPacket & packet) {
            return packet.candidate_id == latched_role;
        });
    ASSERT_NE(
        retained,
        refreshed.intent_packets.begin()
            + static_cast<std::ptrdiff_t>(refreshed.intent_packet_count));
    EXPECT_EQ(retained->candidate_input_revision, latched_revision);
    EXPECT_EQ(
        retained->candidate_set_kind,
        ce::CandidateSetKind::V4SafeControl);

    first.setActivationEnabled(false);
    pushV4Inputs(first, start + 600'000ULL, 20.0, -4.0);
    const auto deactivated = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + 600'000ULL, -88.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(deactivated.has_decision);

    pushV4Inputs(first, start + 650'000ULL, 20.0, -4.0);
    const auto selected_but_inactive = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + 650'000ULL, -87.0, 0.0, 20.0, 0.0));
    EXPECT_FALSE(selected_but_inactive.decision.activation_requested);
    ASSERT_GT(selected_but_inactive.intent_packet_count, 0U);
    const auto retained_while_inactive = std::find_if(
        selected_but_inactive.intent_packets.begin(),
        selected_but_inactive.intent_packets.begin()
            + static_cast<std::ptrdiff_t>(
                selected_but_inactive.intent_packet_count),
        [latched_role](const ce::TrajectoryIntentPacket & packet) {
            return packet.candidate_id == latched_role;
        });
    ASSERT_NE(
        retained_while_inactive,
        selected_but_inactive.intent_packets.begin()
            + static_cast<std::ptrdiff_t>(
                selected_but_inactive.intent_packet_count));
    EXPECT_EQ(
        retained_while_inactive->candidate_input_revision,
        latched_revision);
}

TEST(ManeuverSelectionWorker,
    V4InfeasibilityDoesNotFallBackToLegacyOrBestUnsafeCandidates)
{
    auto local_params = params(0, 3);
    local_params.v4_safe_control_enabled = true;
    local_params.v4_shadow_only = false;
    auto local = std::make_unique<cs::ManeuverSelectionWorker>(local_params);
    auto west_remote = std::make_unique<cs::ManeuverSelectionWorker>(
        params(1, 3));
    auto east_remote = std::make_unique<cs::ManeuverSelectionWorker>(
        params(2, 3));
    constexpr std::uint64_t start = 30'000'000ULL;

    const auto west_far = pushBeliefAndProcess(
        *west_remote,
        beliefSnapshot(start, -200.0, -100.0, -20.0, 0.0));
    const auto east_far = pushBeliefAndProcess(
        *east_remote,
        beliefSnapshot(start, -200.0, 100.0, -20.0, 0.0));
    ASSERT_GT(west_far.intent_packet_count, 0U);
    ASSERT_GT(east_far.intent_packet_count, 0U);
    for (const auto & packet : west_far.intent_packets) {
        ASSERT_TRUE(local->pushRemoteIntent(1, packet));
    }
    for (const auto & packet : east_far.intent_packets) {
        ASSERT_TRUE(local->pushRemoteIntent(2, packet));
    }
    auto west_ready = coordinatedPeerForIntent(
        1, west_far.intent_packets[0]);
    auto east_ready = coordinatedPeerForIntent(
        2, east_far.intent_packets[0]);
    west_ready.v4_cutover_candidate_ready = true;
    east_ready.v4_cutover_candidate_ready = true;
    ASSERT_TRUE(local->pushRemoteDecision(1, west_ready));
    ASSERT_TRUE(local->pushRemoteDecision(2, east_ready));
    pushV4Inputs(*local, start);
    const auto initially_valid = pushBeliefAndProcess(
        *local,
        beliefSnapshot(start, 0.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(initially_valid.has_decision);
    ASSERT_EQ(
        initially_valid.decision.v4_safe_control.status,
        cs::SafeControlSetStatus::Valid);
    ASSERT_GT(initially_valid.intent_packet_count, 0U);
    EXPECT_EQ(
        initially_valid.intent_packets[0].candidate_set_kind,
        ce::CandidateSetKind::V4SafeControl);

    constexpr std::uint64_t close_time = start + 50'000ULL;
    const auto west_close = pushBeliefAndProcess(
        *west_remote,
        beliefSnapshot(close_time, 20.0, -20.0, -20.0, 0.0));
    const auto east_close = pushBeliefAndProcess(
        *east_remote,
        beliefSnapshot(close_time, 20.0, 20.0, -20.0, 0.0));
    ASSERT_GT(west_close.intent_packet_count, 0U);
    ASSERT_GT(east_close.intent_packet_count, 0U);
    for (const auto & packet : west_close.intent_packets) {
        ASSERT_TRUE(local->pushRemoteIntent(1, packet));
    }
    for (const auto & packet : east_close.intent_packets) {
        ASSERT_TRUE(local->pushRemoteIntent(2, packet));
    }
    west_ready = coordinatedPeerForIntent(1, west_close.intent_packets[0]);
    east_ready = coordinatedPeerForIntent(2, east_close.intent_packets[0]);
    west_ready.v4_cutover_candidate_ready = true;
    east_ready.v4_cutover_candidate_ready = true;
    ASSERT_TRUE(local->pushRemoteDecision(1, west_ready));
    ASSERT_TRUE(local->pushRemoteDecision(2, east_ready));
    pushV4Inputs(*local, close_time);
    const auto infeasible = pushBeliefAndProcess(
        *local,
        beliefSnapshot(close_time, 0.0, 0.0, 20.0, 0.0));

    ASSERT_TRUE(infeasible.has_decision);
    EXPECT_TRUE(infeasible.decision.v4_shadow_evaluated);
    EXPECT_EQ(
        infeasible.decision.v4_safe_control.status,
        cs::SafeControlSetStatus::SearchSetInfeasible);
    EXPECT_EQ(
        infeasible.decision.v4_candidates.status,
        cs::SafeControlCandidateAdapterStatus::SearchSetInfeasible);
    EXPECT_EQ(infeasible.decision.v4_candidates.candidate_count, 0U);
    EXPECT_EQ(infeasible.intent_packet_count, 0U);
    EXPECT_FALSE(infeasible.decision.proposal_valid);
    EXPECT_FALSE(infeasible.decision.activation_requested);
}

TEST(ManeuverSelectionWorker, ImplementsTwentyAndFourHertzCadenceWithoutSleeps)
{
    cs::ManeuverSelectionWorker worker(params());
    constexpr std::uint64_t start = 1'000'000ULL;

    auto output = pushBeliefAndProcess(
        worker, beliefSnapshot(start, 0.0, 0.0, 20.0, 0.0));
    ASSERT_EQ(output.intent_packet_count, 3U);
    EXPECT_EQ(output.selection_epoch, 4U);
    EXPECT_FALSE(output.has_decision);
    for (std::size_t index = 0; index < output.intent_packet_count; ++index) {
        EXPECT_EQ(output.intent_packets[index].selection_epoch, 4U);
        EXPECT_EQ(output.intent_packets[index].source_timestamp_us, start);
    }

    EXPECT_TRUE(worker.pushOwnshipBelief(
        beliefSnapshot(start + 49'999, 1.0, 0.0, 20.0, 0.0)));
    EXPECT_TRUE(worker.processPendingForTest());
    EXPECT_FALSE(worker.tryPopOutput().has_value());

    output = pushBeliefAndProcess(
        worker,
        beliefSnapshot(start + 50'000, 1.0, 0.0, 20.0, 0.0));
    EXPECT_EQ(output.intent_packet_count, 3U);
    EXPECT_EQ(output.selection_epoch, 4U);

    for (std::uint64_t offset : {100'000ULL, 150'000ULL, 200'000ULL}) {
        output = pushBeliefAndProcess(
            worker,
            beliefSnapshot(start + offset, 20.0e-6 * offset, 0.0, 20.0, 0.0));
        EXPECT_EQ(output.selection_epoch, 4U);
        EXPECT_FALSE(output.has_decision);
    }

    output = pushBeliefAndProcess(
        worker,
        beliefSnapshot(start + 251'000, 5.0, 0.0, 20.0, 0.0));
    EXPECT_TRUE(output.has_decision);
    EXPECT_EQ(output.decision.selection_timestamp_us, 0U);
    EXPECT_EQ(output.decision.proposal_timestamp_us, start + 250'000);
    EXPECT_EQ(output.decision.proposal_epoch, 4U);
    EXPECT_FALSE(output.decision.proposal_valid);
    EXPECT_FALSE(output.decision.coordination_qualified);
    EXPECT_TRUE(output.decision.previous_best_retained);
    EXPECT_EQ(output.selection_epoch, 5U);
    ASSERT_EQ(output.intent_packet_count, 3U);
    for (std::size_t index = 0; index < output.intent_packet_count; ++index) {
        EXPECT_EQ(output.intent_packets[index].selection_epoch, 5U);
    }
}

TEST(ManeuverSelectionWorker, DoesNotMixAdjacentIncompleteRemoteEpochs)
{
    const auto worker_params = params();
    cs::ManeuverSelectionWorker ownship(worker_params);
    cs::ManeuverSelectionWorker remote(params(1));
    constexpr std::uint64_t start = 2'000'000ULL;

    const auto own_output = pushBeliefAndProcess(
        ownship, beliefSnapshot(start, -45.0, 0.0, 20.0, 0.0));
    auto remote_output = pushBeliefAndProcess(
        remote, beliefSnapshot(start, 45.0, 0.0, -20.0, 0.0));

    ASSERT_TRUE(ownship.pushRemoteIntent(1, remote_output.intent_packets[0]));
    ASSERT_TRUE(ownship.pushRemoteIntent(1, remote_output.intent_packets[1]));
    auto future_epoch_packet = remote_output.intent_packets[2];
    future_epoch_packet.selection_epoch += 1;
    ASSERT_TRUE(ownship.pushRemoteIntent(1, future_epoch_packet));
    EXPECT_TRUE(ownship.processPendingForTest());

    for (std::uint64_t offset : {50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        static_cast<void>(pushBeliefAndProcess(
            ownship,
            beliefSnapshot(
                start + offset, -45.0 + 20.0e-6 * offset,
                0.0, 20.0, 0.0)));
    }
    const auto decision_output = pushBeliefAndProcess(
        ownship,
        beliefSnapshot(start + 250'000, -40.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(decision_output.has_decision);
    EXPECT_FALSE(decision_output.decision.coordination_qualified);
    EXPECT_TRUE(decision_output.decision.previous_best_retained);
    EXPECT_EQ(decision_output.decision.remote_selection_epoch, 0U);
    static_cast<void>(own_output);
}

TEST(ManeuverSelectionWorker, RetainsLastCompleteSetUntilNewRefreshIsComplete)
{
    const auto worker_params = params();
    cs::ManeuverSelectionWorker ownship(worker_params);
    cs::ManeuverSelectionWorker remote(params(1));
    constexpr std::uint64_t start = 2'500'000ULL;

    static_cast<void>(pushBeliefAndProcess(
        ownship, beliefSnapshot(start, -45.0, 0.0, 20.0, 0.0)));
    const auto remote_output = pushBeliefAndProcess(
        remote, beliefSnapshot(start, 45.0, 0.0, -20.0, 0.0));
    for (const auto & packet : remote_output.intent_packets) {
        ASSERT_TRUE(ownship.pushRemoteIntent(1, packet));
    }
    EXPECT_TRUE(ownship.processPendingForTest());

    auto partial_next_refresh = remote_output.intent_packets[0];
    partial_next_refresh.source_timestamp_us = start + 50'000;
    ASSERT_TRUE(ownship.pushRemoteIntent(1, partial_next_refresh));
    EXPECT_TRUE(ownship.processPendingForTest());

    for (std::uint64_t offset : {50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        static_cast<void>(pushBeliefAndProcess(
            ownship,
            beliefSnapshot(
                start + offset, -45.0 + 20.0e-6 * offset,
                0.0, 20.0, 0.0)));
    }
    const auto decision_output = pushBeliefAndProcess(
        ownship,
        beliefSnapshot(start + 250'000, -40.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(decision_output.has_decision);
    EXPECT_FALSE(decision_output.decision.coordination_qualified);
    EXPECT_TRUE(decision_output.decision.proposal_valid);
    EXPECT_EQ(decision_output.decision.proposal_epoch, 10U);
    EXPECT_EQ(decision_output.decision.remote_selection_epoch, 0U);
}

TEST(ManeuverSelectionWorker, BudgetTracingDoesNotChangeControlResults)
{
    const auto replay = [](bool enabled) {
        auto p0 = params(0);
        auto p1 = params(1);
        p0.masd_diagnostics_enabled = enabled;
        p1.masd_diagnostics_enabled = enabled;
        auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(p0);
        auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(p1);
        auto & first = *first_storage;
        auto & second = *second_storage;
        std::vector<cs::ManeuverSelectionWorkerOutput> history;
        cs::ManeuverSelectionWorkerOutput a, b;
        for (std::uint64_t offset = 0; offset <= 250'000; offset += 50'000) {
            const double elapsed = offset * 1.0e-6;
            a = pushBeliefAndProcess(first, beliefSnapshot(
                3'000'000 + offset, -45.0 + 20.0 * elapsed, 0, 20, 0));
            b = pushBeliefAndProcess(second, beliefSnapshot(
                3'000'000 + offset, 45.0 - 20.0 * elapsed, 0, -20, 0));
            history.push_back(a);
            history.push_back(b);
            if (offset < 250'000) exchangePackets(first, second, a, b);
        }
        const auto committed = confirmTwoAircraftProposal(first, second, a, b);
        history.insert(history.end(), committed.begin(), committed.end());
        std::vector<cs::ManeuverBudgetTrace> traces;
        while (auto t = first.tryPopBudgetTrace()) traces.push_back(*t);
        while (auto t = second.tryPopBudgetTrace()) traces.push_back(*t);
        return std::make_pair(history, traces);
    };
    const auto ordinary_result = replay(false);
    const auto traced_result = replay(true);
    const auto & ordinary = ordinary_result.first;
    const auto & traced = traced_result.first;
    EXPECT_TRUE(ordinary_result.second.empty());
    ASSERT_EQ(ordinary.size(), traced.size());
    std::array<bool, 4> observed{};
    for (std::size_t k = 0; k < ordinary.size(); ++k) {
        const auto & a = ordinary[k];
        const auto & b = traced[k];
        EXPECT_EQ(a.intent_packet_count, b.intent_packet_count);
        for (std::size_t n = 0; n < a.intent_packet_count; ++n) {
            EXPECT_EQ(a.intent_packets[n].initial_state, b.intent_packets[n].initial_state);
            EXPECT_EQ(a.intent_packets[n].initial_covariance, b.intent_packets[n].initial_covariance);
            EXPECT_EQ(a.intent_packets[n].candidate_input_revision, b.intent_packets[n].candidate_input_revision);
        }
        EXPECT_EQ(a.decision.selected_candidate_ids, b.decision.selected_candidate_ids);
        EXPECT_EQ(a.decision.proposed_candidate_ids, b.decision.proposed_candidate_ids);
        EXPECT_EQ(a.decision.coordination_qualified, b.decision.coordination_qualified);
        EXPECT_EQ(a.decision.activation_requested, b.decision.activation_requested);
        EXPECT_EQ(a.decision.command_execution_requested, b.decision.command_execution_requested);
        EXPECT_EQ(a.decision.deactivation_reason, b.decision.deactivation_reason);
        if (std::isfinite(a.decision.ad_m)) EXPECT_DOUBLE_EQ(a.decision.ad_m, b.decision.ad_m);
    }
    for (const auto & trace : traced_result.second) {
        ASSERT_LT(trace.event, observed.size());
        observed[trace.event] = true;
        EXPECT_GT(trace.wall_ns, 0U);
        EXPECT_GT(trace.steady_ns, 0U);
        EXPECT_EQ(trace.dropped_trace_count, 0U);
        EXPECT_LE(trace.state_sample_timestamp_us, trace.state_timestamp_us);
        if (trace.event == 3) EXPECT_DOUBLE_EQ(trace.pmr_m - trace.masd_m, trace.ad_m);
    }
    EXPECT_TRUE(observed[1]);
    EXPECT_TRUE(observed[2]);
    EXPECT_TRUE(observed[3]);
}

TEST(ManeuverSelectionWorker, IndependentlySelectsAndRequestsActivation)
{
    const auto worker_params = params();
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(worker_params);
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(params(1));
    auto & second = *second_storage;
    constexpr std::uint64_t start = 3'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset, -45.0 + 20.0 * elapsed_s,
                0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset, 45.0 - 20.0 * elapsed_s,
                0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }

    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 250'000, -40.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 250'000, 40.0, 0.0, -20.0, 0.0));

    const auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    first_output = commits[0];
    second_output = commits[1];

    ASSERT_TRUE(first_output.has_decision);
    ASSERT_TRUE(second_output.has_decision);
    EXPECT_TRUE(first_output.decision.coordination_qualified);
    EXPECT_TRUE(second_output.decision.coordination_qualified);
    EXPECT_TRUE(first_output.decision.new_best_accepted);
    EXPECT_TRUE(second_output.decision.new_best_accepted);
    EXPECT_TRUE(first_output.decision.activation_requested);
    EXPECT_TRUE(second_output.decision.activation_requested);
    EXPECT_LT(first_output.decision.ad_m, 0.0);
    EXPECT_LT(second_output.decision.ad_m, 0.0);
    EXPECT_EQ(first_output.decision.local_selection_epoch, 12U);
    EXPECT_EQ(second_output.decision.local_selection_epoch, 12U);
}

TEST(ManeuverSelectionWorker, FormationGateSuppressesOnlyNewAmacActivation)
{
    const auto formation_params = [](const int vehicle_id) {
        auto value = params(vehicle_id);
        value.formation_discrimination_enabled = true;
        value.formation_target_separation_m = 1000.0;
        value.formation_aggregation_policy = collision_avoidance::formation::
            FormationAggregationPolicy::PerThreatExemptionOnly;
        auto & config = value.formation_boundary_config;
        config.profile_name = "test_calibrated_profile";
        config.representative_wingspan_m = 2.0;
        config.range0_wingspan_scale = 1.0;
        config.uncertainty_margin_m = 0.0;
        config.range1_offset_m = 1.0;
        config.closure_upper_entry_table = {{0.0, 100.0}, {200.0, 100.0}};
        config.closure_upper_exit_table = {{0.0, 100.0}, {200.0, 100.0}};
        config.closure_lower_entry_mps = -100.0;
        config.closure_lower_exit_mps = -100.0;
        config.fdz_entry_limit_m = 100.0;
        config.fdz_exit_limit_m = 110.0;
        config.max_range_entry_m = 200.0;
        config.max_range_exit_m = 200.0;
        config.maximum_state_age_s = 0.1;
        config.maximum_future_skew_s = 0.0;
        config.maximum_timestamp_skew_s = 0.0;
        return value;
    };

    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(formation_params(0));
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(formation_params(1));
    auto & second = *second_storage;
    constexpr std::uint64_t start = 4'000'000ULL;
    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset, -45.0 + 20.0 * elapsed_s,
                0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset, 45.0 - 20.0 * elapsed_s,
                0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }

    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 250'000, -40.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 250'000, 40.0, 0.0, -20.0, 0.0));
    const auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);

    for (const auto & output : commits) {
        ASSERT_TRUE(output.has_decision);
        EXPECT_LT(output.decision.ad_m, 0.0);
        EXPECT_TRUE(output.decision.formation_evaluated);
        EXPECT_TRUE(output.decision.formation_inhibit);
        EXPECT_FALSE(output.decision.formation_allow_new_activation);
        EXPECT_NE(output.decision.formation_inhibited_threat_mask, 0U);
        EXPECT_FALSE(output.decision.activation_requested);
    }
}

TEST(ManeuverSelectionWorker,
    RejectsFormationTargetInsideBaseHardSafetyBudget)
{
    auto value = params();
    value.formation_discrimination_enabled = true;
    value.formation_target_separation_m = 12.0;
    auto & config = value.formation_boundary_config;
    config.profile_name = "incompatible_test_profile";
    config.representative_wingspan_m = 2.0;
    config.range0_wingspan_scale = 1.0;
    config.uncertainty_margin_m = 0.0;
    config.range1_offset_m = 1.0;
    config.closure_upper_entry_table = {{0.0, 1.0}, {20.0, 1.0}};
    config.closure_upper_exit_table = {{0.0, 2.0}, {20.0, 2.0}};
    config.closure_lower_entry_mps = -1.0;
    config.closure_lower_exit_mps = -2.0;
    config.fdz_entry_limit_m = 0.0;
    config.fdz_exit_limit_m = 0.0;
    config.max_range_entry_m = 20.0;
    config.max_range_exit_m = 20.0;
    config.maximum_state_age_s = 0.1;
    config.maximum_future_skew_s = 0.0;
    config.maximum_timestamp_skew_s = 0.0;

    cs::ManeuverSelectionWorker worker(value);
    EXPECT_FALSE(worker.start());
}

TEST(ManeuverSelectionWorker, DoesNotActivateWhileCurrentPlanIsSafe)
{
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(params());
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(params(1));
    auto & second = *second_storage;
    constexpr std::uint64_t start = 5'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset, -200.0 + 20.0 * elapsed_s,
                0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset, 200.0 - 20.0 * elapsed_s,
                0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }

    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 250'000, -195.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 250'000, 195.0, 0.0, -20.0, 0.0));

    const auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    first_output = commits[0];
    second_output = commits[1];

    ASSERT_TRUE(first_output.has_decision);
    ASSERT_TRUE(second_output.has_decision);
    EXPECT_TRUE(first_output.decision.coordination_qualified);
    EXPECT_TRUE(second_output.decision.coordination_qualified);
    EXPECT_GT(first_output.decision.ad_m, 0.0);
    EXPECT_GT(second_output.decision.ad_m, 0.0);
    EXPECT_FALSE(first_output.decision.activation_requested);
    EXPECT_FALSE(second_output.decision.activation_requested);
}

TEST(ManeuverSelectionWorker, MonitorsActivationBetweenSelectionEvents)
{
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(params());
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(params(1));
    auto & second = *second_storage;
    constexpr std::uint64_t start = 6'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(start + offset, -200.0, 0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(start + offset, 200.0, 0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }
    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 250'000, -200.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 250'000, 200.0, 0.0, -20.0, 0.0));
    exchangePackets(first, second, first_output, second_output);
    const auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    first_output = commits[0];
    second_output = commits[1];
    ASSERT_TRUE(first_output.has_decision);
    ASSERT_FALSE(first_output.decision.activation_requested);
    const std::uint64_t selected_epoch =
        first_output.decision.local_selection_epoch;

    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 300'000, -5.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 300'000, 5.0, 0.0, -20.0, 0.0));
    exchangePackets(first, second, first_output, second_output);

    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 350'000, -4.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 350'000, 4.0, 0.0, -20.0, 0.0));
    ASSERT_TRUE(first_output.has_decision);
    ASSERT_TRUE(second_output.has_decision);
    EXPECT_LT(first_output.decision.ad_m, 0.0);
    EXPECT_LT(second_output.decision.ad_m, 0.0);
    EXPECT_TRUE(first_output.decision.activation_requested);
    EXPECT_TRUE(second_output.decision.activation_requested);
    EXPECT_TRUE(first_output.decision.activation_just_started);
    EXPECT_TRUE(second_output.decision.activation_just_started);
    EXPECT_EQ(first_output.decision.local_selection_epoch, selected_epoch);
    EXPECT_EQ(second_output.decision.local_selection_epoch, selected_epoch);
}

TEST(ManeuverSelectionWorker,
    DeactivatesWithoutV4AfterCpaAndCoordinatedFormationRolloutAreSafe)
{
    auto first_params = params();
    auto second_params = params(1);
    ASSERT_FALSE(first_params.v4_safe_control_enabled);
    ASSERT_FALSE(second_params.v4_safe_control_enabled);
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(first_params);
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(second_params);
    auto & second = *second_storage;
    constexpr std::uint64_t start = 6'500'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        ASSERT_TRUE(first.pushNominalSetpoint(
            nominalSnapshot(start + offset)));
        ASSERT_TRUE(second.pushNominalSetpoint(
            nominalSnapshot(start + offset)));
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset, -45.0 + 20.0 * elapsed_s,
                0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset, 45.0 - 20.0 * elapsed_s,
                0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }

    ASSERT_TRUE(first.pushNominalSetpoint(
        nominalSnapshot(start + 250'000ULL)));
    ASSERT_TRUE(second.pushNominalSetpoint(
        nominalSnapshot(start + 250'000ULL)));
    first_output = pushBeliefAndProcess(
        first,
        beliefSnapshot(start + 250'000ULL, -40.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second,
        beliefSnapshot(start + 250'000ULL, 40.0, 0.0, -20.0, 0.0));
    auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    first_output = commits[0];
    second_output = commits[1];
    ASSERT_TRUE(first_output.decision.activation_requested);
    ASSERT_TRUE(second_output.decision.activation_requested);

    bool first_ended = false;
    bool second_ended = false;
    for (std::size_t step = 0; step < 5; ++step) {
        const std::uint64_t timestamp_us =
            start + 300'000ULL + step * 50'000ULL;
        ASSERT_TRUE(first.pushNominalSetpoint(
            nominalSnapshot(timestamp_us)));
        ASSERT_TRUE(second.pushNominalSetpoint(
            nominalSnapshot(timestamp_us)));
        auto second_peer = peerDecision(second_output.decision);
        auto first_peer = peerDecision(first_output.decision);
        // Nominal post-release safety is timestamped independently of the
        // avoidance candidate-selection epoch. A component topology change may
        // advance the two selected epochs at different instants, so that
        // unrelated epoch must not deadlock an otherwise coordinated release.
        second_peer.local_selection_epoch += 100U;
        first_peer.local_selection_epoch += 200U;
        ASSERT_TRUE(first.pushRemoteDecision(1, second_peer));
        ASSERT_TRUE(second.pushRemoteDecision(0, first_peer));
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                timestamp_us,
                -100.0 - static_cast<double>(step),
                0.0, -20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                timestamp_us,
                100.0 + static_cast<double>(step),
                0.0, 20.0, 0.0));
        ASSERT_TRUE(first_output.has_decision);
        ASSERT_TRUE(second_output.has_decision);
        exchangePackets(first, second, first_output, second_output);
        first_ended = first_ended
            || first_output.decision.activation_just_ended;
        second_ended = second_ended
            || second_output.decision.activation_just_ended;
        if (!first_output.decision.activation_requested
            && !second_output.decision.activation_requested) {
            break;
        }
    }

    EXPECT_TRUE(first_ended);
    EXPECT_TRUE(second_ended);
    EXPECT_TRUE(first_output.decision.post_release_evaluated);
    EXPECT_TRUE(second_output.decision.post_release_evaluated);
    EXPECT_TRUE(first_output.decision.post_release_safe);
    EXPECT_TRUE(second_output.decision.post_release_safe);
    EXPECT_FALSE(first_output.decision.activation_requested);
    EXPECT_FALSE(second_output.decision.activation_requested);
    EXPECT_EQ(
        first_output.decision.deactivation_reason,
        cs::ManeuverDeactivationReason::FutureCpaClear);
    EXPECT_EQ(
        second_output.decision.deactivation_reason,
        cs::ManeuverDeactivationReason::FutureCpaClear);
}

TEST(ManeuverSelectionWorker, WarmsSelectionButDoesNotActivateBeforeGateOpens)
{
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(params());
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(params(1));
    auto & second = *second_storage;
    first.setActivationEnabled(false);
    second.setActivationEnabled(false);
    constexpr std::uint64_t start = 7'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(start + offset, -20.0, 0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(start + offset, 20.0, 0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }
    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 250'000, -20.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 250'000, 20.0, 0.0, -20.0, 0.0));
    const auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    first_output = commits[0];
    second_output = commits[1];
    ASSERT_TRUE(first_output.has_decision);
    ASSERT_TRUE(second_output.has_decision);
    EXPECT_TRUE(first_output.decision.coordination_qualified);
    EXPECT_TRUE(second_output.decision.coordination_qualified);
    EXPECT_LT(first_output.decision.ad_m, 0.0);
    EXPECT_LT(second_output.decision.ad_m, 0.0);
    EXPECT_FALSE(first_output.decision.activation_requested);
    EXPECT_FALSE(second_output.decision.activation_requested);

    first.setActivationEnabled(true);
    second.setActivationEnabled(true);
    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 300'000, -19.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 300'000, 19.0, 0.0, -20.0, 0.0));
    ASSERT_TRUE(first_output.has_decision);
    ASSERT_TRUE(second_output.has_decision);
    EXPECT_TRUE(first_output.decision.activation_requested);
    EXPECT_TRUE(second_output.decision.activation_requested);
    EXPECT_TRUE(first_output.decision.activation_just_started);
    EXPECT_TRUE(second_output.decision.activation_just_started);
}

TEST(ManeuverSelectionWorker,
    AmacAcceptsSameManeuverTupleDespitePeerRevisionMismatch)
{
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(params());
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(params(1));
    auto & second = *second_storage;
    first.setActivationEnabled(false);
    second.setActivationEnabled(false);
    constexpr std::uint64_t start = 8'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset, -45.0 + 20.0 * elapsed_s,
                0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset, 45.0 - 20.0 * elapsed_s,
                0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }

    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 250'000, -40.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 250'000, 40.0, 0.0, -20.0, 0.0));
    auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    ASSERT_TRUE(commits[0].decision.coordination_qualified);
    const auto committed_tuple = commits[0].decision.selected_candidate_ids;
    const std::uint64_t committed_epoch =
        commits[0].decision.local_selection_epoch;
    const std::uint64_t committed_timestamp =
        commits[0].decision.selection_timestamp_us;
    exchangePackets(first, second, commits[0], commits[1]);

    for (const std::uint64_t offset : {
             300'000ULL, 350'000ULL, 400'000ULL, 450'000ULL}) {
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(start + offset, -40.0, 0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(start + offset, 40.0, 0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }
    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 500'000, -40.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 500'000, 40.0, 0.0, -20.0, 0.0));
    ASSERT_TRUE(first_output.decision.proposal_valid);
    ASSERT_GT(first_output.decision.proposal_epoch, committed_epoch);

    auto mismatched_peer = peerDecision(second_output.decision);
    ASSERT_EQ(
        mismatched_peer.proposed_candidate_ids,
        first_output.decision.proposed_candidate_ids);
    mismatched_peer.proposed_candidate_input_revisions[0] ^= 1ULL;
    ASSERT_TRUE(first.pushRemoteDecision(1, mismatched_peer));
    ASSERT_TRUE(first.processPendingForTest());
    const auto accepted = first.tryPopOutput();
    ASSERT_TRUE(accepted.has_value());
    ASSERT_TRUE(accepted->has_decision);
    EXPECT_TRUE(accepted->decision.proposal_consensus_confirmed);
    EXPECT_TRUE(accepted->decision.coordination_qualified);
    EXPECT_EQ(
        accepted->decision.local_selection_epoch,
        first_output.decision.proposal_epoch);
    EXPECT_GT(accepted->decision.local_selection_epoch, committed_epoch);
    EXPECT_GT(accepted->decision.selection_timestamp_us, committed_timestamp);
    EXPECT_EQ(
        accepted->decision.selected_candidate_ids,
        first_output.decision.proposed_candidate_ids);
    EXPECT_NE(accepted->decision.selected_candidate_ids, committed_tuple);
}

TEST(ManeuverSelectionWorker,
    NonSuperiorActiveSwitchRetainsPreviousBest)
{
    auto first_params = params();
    first_params.active_switching_enabled = true;
    first_params.active_switch_cost_margin = 1.0e9;
    first_params.active_switch_minimum_ad_margin_m = 1.0e9;
    first_params.evaluator_params.desired_separation_distance_m = 1'000.0;
    auto second_params = params(1);
    second_params.evaluator_params.desired_separation_distance_m = 1'000.0;
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(first_params);
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(second_params);
    auto & first = *first_storage;
    auto & second = *second_storage;
    constexpr std::uint64_t start = 12'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset, -45.0 + 20.0 * elapsed_s,
                0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset, 45.0 - 20.0 * elapsed_s,
                0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }
    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 250'000, -40.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 250'000, 40.0, 0.0, -20.0, 0.0));
    auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    ASSERT_TRUE(commits[0].decision.activation_requested);
    const std::uint8_t latched_ownship_id =
        commits[0].decision.ownship_candidate_id;
    const std::uint64_t first_committed_epoch =
        commits[0].decision.local_selection_epoch;
    exchangePackets(first, second, commits[0], commits[1]);

    auto peer_inactive = peerDecision(commits[1].decision);
    peer_inactive.activation_requested = false;
    ASSERT_TRUE(first.pushRemoteDecision(1, peer_inactive));
    ASSERT_TRUE(first.processPendingForTest());

    for (const std::uint64_t offset : {
             300'000ULL, 350'000ULL, 400'000ULL, 450'000ULL}) {
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(start + offset, -40.0, 0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(start + offset, 40.0, 0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }
    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 500'000, -40.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(first_output.has_decision);
    EXPECT_FALSE(first_output.decision.proposal_valid);
    EXPECT_TRUE(first_output.decision.switch_superiority_evaluated);
    EXPECT_FALSE(first_output.decision.switch_clearly_superior);
    EXPECT_EQ(first_output.decision.ownship_candidate_id, latched_ownship_id);
    EXPECT_EQ(
        first_output.decision.local_selection_epoch, first_committed_epoch);
    EXPECT_TRUE(first_output.decision.activation_requested);
    EXPECT_TRUE(first_output.decision.command_execution_requested);
}

TEST(ManeuverSelectionWorker,
    ClearlySuperiorAmacBestCommitsAfterManeuverTupleAgreement)
{
    auto first_params = params();
    first_params.active_switching_enabled = true;
    first_params.active_switch_cost_margin = 1.0e-9;
    first_params.active_switch_minimum_ad_margin_m = 1.0e-9;
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(first_params);
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(params(1));
    auto & second = *second_storage;
    constexpr std::uint64_t start = 14'000'000ULL;

    cs::ManeuverSelectionWorkerOutput first_output;
    cs::ManeuverSelectionWorkerOutput second_output;
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(
                start + offset, -45.0 + 20.0 * elapsed_s,
                0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(
                start + offset, 45.0 - 20.0 * elapsed_s,
                0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }
    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 250'000, -40.0, 0.0, 20.0, 0.0));
    second_output = pushBeliefAndProcess(
        second, beliefSnapshot(start + 250'000, 40.0, 0.0, -20.0, 0.0));
    auto commits = confirmTwoAircraftProposal(
        first, second, first_output, second_output);
    ASSERT_TRUE(commits[0].decision.activation_requested);
    const std::uint8_t previous_ownship_id =
        commits[0].decision.ownship_candidate_id;
    const std::uint64_t activation_timestamp_us =
        commits[0].decision.activation_timestamp_us;
    exchangePackets(first, second, commits[0], commits[1]);

    auto peer_inactive = peerDecision(commits[1].decision);
    peer_inactive.activation_requested = false;
    ASSERT_TRUE(first.pushRemoteDecision(1, peer_inactive));
    ASSERT_TRUE(first.processPendingForTest());

    for (const std::uint64_t offset : {
             300'000ULL, 350'000ULL, 400'000ULL, 450'000ULL}) {
        first_output = pushBeliefAndProcess(
            first,
            beliefSnapshot(start + offset, -40.0, 0.0, 20.0, 0.0));
        second_output = pushBeliefAndProcess(
            second,
            beliefSnapshot(start + offset, 40.0, 0.0, -20.0, 0.0));
        exchangePackets(first, second, first_output, second_output);
    }
    first_output = pushBeliefAndProcess(
        first, beliefSnapshot(start + 500'000, -40.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(first_output.has_decision);
    ASSERT_TRUE(first_output.decision.proposal_valid);
    ASSERT_TRUE(first_output.decision.switch_superiority_evaluated);
    ASSERT_TRUE(first_output.decision.switch_clearly_superior);
    const std::uint8_t proposed_ownship_id =
        first_output.decision.proposed_candidate_ids[0];
    ASSERT_NE(proposed_ownship_id, previous_ownship_id);
    EXPECT_EQ(
        first_output.decision.ownship_candidate_id, previous_ownship_id);

    auto matching_peer = peerDecision(first_output.decision);
    matching_peer.vehicle_id = 1;
    matching_peer.ownship_candidate_id =
        matching_peer.selected_candidate_ids[1];
    matching_peer.coordination_qualified = false;
    matching_peer.activation_requested = false;
    matching_peer.proposal_consensus_confirmed = false;
    ASSERT_TRUE(first.pushRemoteDecision(1, matching_peer));
    ASSERT_TRUE(first.processPendingForTest());
    const auto committed = first.tryPopOutput();
    ASSERT_TRUE(committed.has_value());
    ASSERT_TRUE(committed->has_decision);
    EXPECT_TRUE(committed->decision.new_best_accepted);
    EXPECT_FALSE(committed->decision.previous_best_retained);
    EXPECT_TRUE(committed->decision.activation_requested);
    EXPECT_FALSE(committed->decision.activation_just_started);
    EXPECT_FALSE(committed->decision.activation_just_ended);
    EXPECT_TRUE(committed->decision.command_execution_requested);
    EXPECT_EQ(
        committed->decision.activation_timestamp_us,
        activation_timestamp_us);
    EXPECT_EQ(committed->decision.ownship_candidate_id, proposed_ownship_id);
    EXPECT_EQ(
        committed->decision.selected_candidate_ids[0], proposed_ownship_id);
}

TEST(ManeuverSelectionWorker, StartsStopsAndKeepsInstancesIndependent)
{
    auto first_storage = std::make_unique<cs::ManeuverSelectionWorker>(params());
    auto & first = *first_storage;
    auto second_storage = std::make_unique<cs::ManeuverSelectionWorker>(params());
    auto & second = *second_storage;
    ASSERT_TRUE(first.start());
    ASSERT_TRUE(second.start());
    EXPECT_TRUE(first.running());
    EXPECT_TRUE(second.running());

    ASSERT_TRUE(first.pushOwnshipBelief(
        beliefSnapshot(4'000'000ULL, 0.0, 0.0, 20.0, 0.0)));
    ASSERT_TRUE(second.pushOwnshipBelief(
        beliefSnapshot(8'000'000ULL, 100.0, 0.0, -20.0, 0.0)));

    std::optional<cs::ManeuverSelectionWorkerOutput> first_output;
    std::optional<cs::ManeuverSelectionWorkerOutput> second_output;
    for (int attempt = 0; attempt < 100; ++attempt) {
        if (!first_output.has_value()) {
            first_output = first.tryPopOutput();
        }
        if (!second_output.has_value()) {
            second_output = second.tryPopOutput();
        }
        if (first_output.has_value() && second_output.has_value()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    first.stop();
    second.stop();
    EXPECT_FALSE(first.running());
    EXPECT_FALSE(second.running());
    ASSERT_TRUE(first_output.has_value());
    ASSERT_TRUE(second_output.has_value());
    EXPECT_EQ(first_output->generated_timestamp_us, 4'000'000ULL);
    EXPECT_EQ(second_output->generated_timestamp_us, 8'000'000ULL);
    EXPECT_EQ(first.droppedInputCount(), 0U);
    EXPECT_EQ(second.droppedOutputCount(), 0U);
}

TEST(ManeuverSelectionWorker, FiveAircraftWorkersSelectSameJointTuple)
{
    constexpr std::size_t aircraft_count = 5;
    constexpr std::uint64_t start = 9'000'000ULL;
    constexpr double radius_m = 45.0;
    constexpr double speed_mps = 20.0;
    constexpr double two_pi = 2.0 * M_PI;

    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, aircraft_count>
        workers;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            params(static_cast<int>(aircraft), aircraft_count));
    }

    std::array<cs::ManeuverSelectionWorkerOutput, aircraft_count> outputs{};
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        for (std::size_t aircraft = 0;
             aircraft < aircraft_count; ++aircraft) {
            const double angle = two_pi * static_cast<double>(aircraft)
                / static_cast<double>(aircraft_count);
            const double unit_north = std::cos(angle);
            const double unit_east = std::sin(angle);
            outputs[aircraft] = pushBeliefAndProcess(
                *workers[aircraft],
                beliefSnapshot(
                    start + offset,
                    (radius_m - speed_mps * elapsed_s) * unit_north,
                    (radius_m - speed_mps * elapsed_s) * unit_east,
                    -speed_mps * unit_north,
                    -speed_mps * unit_east));
        }

        for (std::size_t sender = 0; sender < aircraft_count; ++sender) {
            for (std::size_t receiver = 0;
                 receiver < aircraft_count; ++receiver) {
                if (sender == receiver) {
                    continue;
                }
                for (std::size_t packet_index = 0;
                     packet_index < outputs[sender].intent_packet_count;
                     ++packet_index) {
                    ASSERT_TRUE(workers[receiver]->pushRemoteIntent(
                        static_cast<int>(sender),
                        outputs[sender].intent_packets[packet_index]));
                }
            }
        }
        for (auto & worker : workers) {
            EXPECT_TRUE(worker->processPendingForTest());
        }
    }

    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        const double angle = two_pi * static_cast<double>(aircraft)
            / static_cast<double>(aircraft_count);
        const double unit_north = std::cos(angle);
        const double unit_east = std::sin(angle);
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start + 250'000,
                (radius_m - 5.0) * unit_north,
                (radius_m - 5.0) * unit_east,
                -speed_mps * unit_north,
                -speed_mps * unit_east));
    }

    outputs = confirmAllAircraftProposals(workers, outputs);
    const auto expected_tuple = outputs[0].decision.selected_candidate_ids;
    const auto expected_input_revisions =
        outputs[0].decision.selected_candidate_input_revisions;
    const auto expected_source_timestamps =
        outputs[0].decision.selected_candidate_source_timestamps_us;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        const auto & decision = outputs[aircraft].decision;
        ASSERT_TRUE(outputs[aircraft].has_decision);
        EXPECT_TRUE(decision.coordination_qualified);
        EXPECT_EQ(decision.aircraft_count, aircraft_count);
        EXPECT_EQ(decision.evaluated_combination_count, 243U);
        EXPECT_EQ(decision.selected_candidate_ids, expected_tuple);
        EXPECT_EQ(
            decision.selected_candidate_input_revisions,
            expected_input_revisions);
        EXPECT_EQ(
            decision.selected_candidate_source_timestamps_us,
            expected_source_timestamps);
        EXPECT_EQ(
            decision.ownship_candidate_id,
            expected_tuple[aircraft]);
    }
}

TEST(ManeuverSelectionWorker, ExhaustiveTestModeEvaluatesAllFiveAircraftRollTuples)
{
    constexpr std::size_t aircraft_count = 5;
    constexpr std::uint64_t start = 11'000'000ULL;
    constexpr double radius_m = 45.0;
    constexpr double speed_mps = 20.0;

    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, aircraft_count>
        workers;
    std::array<cs::ManeuverSelectionWorkerOutput, aircraft_count> outputs{};
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        auto worker_params = params(static_cast<int>(aircraft), aircraft_count);
        worker_params.exhaustive_test_mode = true;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        const double angle = 2.0 * M_PI * static_cast<double>(aircraft)
            / static_cast<double>(aircraft_count);
        const double unit_north = std::cos(angle);
        const double unit_east = std::sin(angle);
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start,
                radius_m * unit_north,
                radius_m * unit_east,
                -speed_mps * unit_north,
                -speed_mps * unit_east));
        EXPECT_EQ(outputs[aircraft].intent_packet_count, 7U);
    }

    for (std::size_t sender = 0; sender < aircraft_count; ++sender) {
        for (std::size_t receiver = 0; receiver < aircraft_count; ++receiver) {
            if (sender == receiver) {
                continue;
            }
            for (std::size_t packet_index = 0;
                 packet_index < outputs[sender].intent_packet_count;
                 ++packet_index) {
                ASSERT_TRUE(workers[receiver]->pushRemoteIntent(
                    static_cast<int>(sender),
                    outputs[sender].intent_packets[packet_index]));
            }
        }
    }
    for (auto & worker : workers) {
        EXPECT_TRUE(worker->processPendingForTest());
    }

    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        const double angle = 2.0 * M_PI * static_cast<double>(aircraft)
            / static_cast<double>(aircraft_count);
        const double unit_north = std::cos(angle);
        const double unit_east = std::sin(angle);
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start + 250'000,
                (radius_m - 5.0) * unit_north,
                (radius_m - 5.0) * unit_east,
                -speed_mps * unit_north,
                -speed_mps * unit_east));
    }

    outputs = confirmAllAircraftProposals(workers, outputs);
    const auto expected_tuple = outputs[0].decision.selected_candidate_ids;
    const auto expected_input_revisions =
        outputs[0].decision.selected_candidate_input_revisions;
    const auto expected_source_timestamps =
        outputs[0].decision.selected_candidate_source_timestamps_us;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        const auto & decision = outputs[aircraft].decision;
        ASSERT_TRUE(outputs[aircraft].has_decision);
        EXPECT_TRUE(decision.coordination_qualified);
        EXPECT_EQ(decision.evaluated_combination_count, 16'807U);
        EXPECT_EQ(decision.selected_candidate_ids, expected_tuple);
        EXPECT_EQ(
            decision.selected_candidate_input_revisions,
            expected_input_revisions);
        EXPECT_EQ(
            decision.selected_candidate_source_timestamps_us,
            expected_source_timestamps);
    }
}

TEST(ManeuverSelectionWorker,
    InteractionGraphReportsWhichCandidateLibraryIsIncomplete)
{
    constexpr std::uint64_t start = 13'750'000ULL;
    auto worker_params = params(0, 2);
    worker_params.exhaustive_test_mode = true;
    worker_params.interaction_graph_params.enabled = true;
    cs::ManeuverSelectionWorker worker(worker_params);
    ASSERT_TRUE(worker.pushNominalSetpoint(nominalSnapshot(start)));

    const auto first_output = pushBeliefAndProcess(
        worker, beliefSnapshot(start, 0.0, 0.0, 20.0, 0.0));
    ASSERT_EQ(first_output.intent_packet_count, 7U);
    static_cast<void>(pushBeliefAndProcess(
        worker,
        beliefSnapshot(start + 250'000ULL, 5.0, 0.0, 20.0, 0.0)));

    const auto diagnostics_message =
        worker.tryPopInteractionGraphDiagnostics();
    ASSERT_TRUE(diagnostics_message.has_value());
    ASSERT_TRUE(diagnostics_message.value());
    const auto & diagnostics = *diagnostics_message.value();
    EXPECT_EQ(
        diagnostics.status,
        cs::InteractionGraphEvaluationStatus::CandidateSetsIncomplete);
    EXPECT_EQ(diagnostics.candidate_ready_mask, 0b01U);
    EXPECT_EQ(diagnostics.candidate_counts[0], 7U);
    EXPECT_EQ(diagnostics.candidate_counts[1], 0U);
    EXPECT_GT(diagnostics.candidate_source_timestamps_us[0], 0U);
    EXPECT_EQ(diagnostics.dropped_remote_intent_count, 0U);
}

TEST(ManeuverSelectionWorker,
    InteractionGraphFreezesFutureEpochLibraryAcceptedBeforeLocalEpochAdvance)
{
    constexpr std::uint64_t start = 16'000'000ULL;
    auto ownship_params = params(0, 2);
    ownship_params.exhaustive_test_mode = true;
    ownship_params.interaction_graph_params.enabled = true;
    auto remote_params = params(1, 2);
    remote_params.exhaustive_test_mode = true;
    remote_params.interaction_graph_params.enabled = true;
    cs::ManeuverSelectionWorker ownship(ownship_params);
    cs::ManeuverSelectionWorker remote(remote_params);
    ASSERT_TRUE(ownship.pushNominalSetpoint(nominalSnapshot(start)));
    ASSERT_TRUE(remote.pushNominalSetpoint(nominalSnapshot(start)));

    static_cast<void>(pushBeliefAndProcess(
        ownship, beliefSnapshot(start, 0.0, 0.0, 20.0, 0.0)));
    static_cast<void>(pushBeliefAndProcess(
        remote, beliefSnapshot(start, 80.0, 0.0, -20.0, 0.0)));
    for (std::uint64_t offset : {
            50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        static_cast<void>(pushBeliefAndProcess(
            ownship,
            beliefSnapshot(
                start + offset, 20.0e-6 * offset,
                0.0, 20.0, 0.0)));
    }

    const auto future_epoch_output = pushBeliefAndProcess(
        remote,
        beliefSnapshot(start + 250'000ULL, 75.0, 0.0, -20.0, 0.0));
    ASSERT_EQ(future_epoch_output.selection_epoch, 65U);
    ASSERT_EQ(future_epoch_output.intent_packet_count, 7U);
    for (std::size_t index = 0;
         index < future_epoch_output.intent_packet_count; ++index) {
        ASSERT_TRUE(ownship.pushRemoteIntent(
            1, future_epoch_output.intent_packets[index]));
    }
    ASSERT_TRUE(ownship.processPendingForTest());

    static_cast<void>(pushBeliefAndProcess(
        ownship,
        beliefSnapshot(start + 250'000ULL, 5.0, 0.0, 20.0, 0.0)));
    const auto old_epoch_diagnostics =
        ownship.tryPopInteractionGraphDiagnostics();
    ASSERT_TRUE(old_epoch_diagnostics.has_value());
    for (std::uint64_t offset : {
            300'000ULL, 350'000ULL, 400'000ULL, 450'000ULL}) {
        static_cast<void>(pushBeliefAndProcess(
            ownship,
            beliefSnapshot(
                start + offset, 20.0e-6 * offset,
                0.0, 20.0, 0.0)));
    }
    static_cast<void>(pushBeliefAndProcess(
        ownship,
        beliefSnapshot(start + 500'000ULL, 10.0, 0.0, 20.0, 0.0)));

    const auto diagnostics_message =
        ownship.tryPopInteractionGraphDiagnostics();
    ASSERT_TRUE(diagnostics_message.has_value());
    ASSERT_TRUE(diagnostics_message.value());
    const auto & diagnostics = *diagnostics_message.value();
    EXPECT_EQ(diagnostics.graph.selection_epoch, 65U);
    EXPECT_TRUE(diagnostics.graph.valid());
    EXPECT_EQ(diagnostics.candidate_ready_mask, 0b11U);
    EXPECT_EQ(diagnostics.candidate_counts[0], 7U);
    EXPECT_EQ(diagnostics.candidate_counts[1], 7U);
}

TEST(ManeuverSelectionWorker,
    InteractionGraphGlobalCrosscheckRejectsMissedUnsafePair)
{
    constexpr std::uint64_t start = 14'000'000ULL;
    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, 2> workers;
    std::array<cs::ManeuverSelectionWorkerOutput, 2> outputs{};
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        auto worker_params = params(static_cast<int>(aircraft), 2);
        worker_params.exhaustive_test_mode = true;
        worker_params.interaction_graph_params.enabled = true;
        // Deliberately wrong screening threshold: the aircraft are closer
        // than the hard budget but are split into isolated components.
        worker_params.interaction_graph_params.ad_screen_m = -1.0e6;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start,
                5.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
    }

    exchangePackets(*workers[0], *workers[1], outputs[0], outputs[1]);
    ASSERT_TRUE(workers[0]->pushRemoteDecision(
        1, nominalPeerDecision(1, start)));
    ASSERT_TRUE(workers[1]->pushRemoteDecision(
        0, nominalPeerDecision(0, start)));
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start + 250'000ULL,
                5.0 + 5.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
        const auto diagnostics_message =
            workers[aircraft]->tryPopInteractionGraphDiagnostics();
        ASSERT_TRUE(diagnostics_message.has_value());
        ASSERT_TRUE(diagnostics_message.value());
        const auto & diagnostics = *diagnostics_message.value();
        ASSERT_TRUE(diagnostics.graph.valid());
        EXPECT_EQ(diagnostics.candidate_ready_mask, 0b11U);
        EXPECT_EQ(diagnostics.candidate_counts[0], 7U);
        EXPECT_EQ(diagnostics.candidate_counts[1], 7U);
        EXPECT_TRUE(diagnostics.component_search_evaluated);
        EXPECT_TRUE(diagnostics.global_crosscheck_evaluated);
        EXPECT_FALSE(diagnostics.global_crosscheck_pass);
        EXPECT_EQ(diagnostics.status,
            cs::InteractionGraphEvaluationStatus::GlobalCrosscheckFailed);
    }
}

TEST(ManeuverSelectionWorker,
    InteractionGraphRetainsBestUnsafeConnectedComponent)
{
    constexpr std::uint64_t start = 14'500'000ULL;
    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, 2> workers;
    std::array<cs::ManeuverSelectionWorkerOutput, 2> outputs{};
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        auto worker_params = params(static_cast<int>(aircraft), 2);
        worker_params.exhaustive_test_mode = true;
        worker_params.interaction_graph_params.enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = 0.0;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start,
                5.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
    }

    exchangePackets(*workers[0], *workers[1], outputs[0], outputs[1]);
    ASSERT_TRUE(workers[0]->pushRemoteDecision(
        1, nominalPeerDecision(1, start)));
    ASSERT_TRUE(workers[1]->pushRemoteDecision(
        0, nominalPeerDecision(0, start)));
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start + 250'000ULL,
                5.0 + 5.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
        ASSERT_TRUE(outputs[aircraft].has_decision);
        EXPECT_TRUE(outputs[aircraft].decision.proposal_valid);
        EXPECT_EQ(outputs[aircraft].decision.evaluated_combination_count, 49U);
        EXPECT_FALSE(outputs[aircraft].decision.selected_combination_safe);

        const auto diagnostics_message =
            workers[aircraft]->tryPopInteractionGraphDiagnostics();
        ASSERT_TRUE(diagnostics_message.has_value());
        ASSERT_TRUE(diagnostics_message.value());
        const auto & diagnostics = *diagnostics_message.value();
        ASSERT_TRUE(diagnostics.graph.valid());
        EXPECT_EQ(diagnostics.graph.component_count, 1U);
        EXPECT_EQ(diagnostics.graph.component_evaluation_count, 49U);
        EXPECT_TRUE(diagnostics.global_crosscheck_evaluated);
        EXPECT_TRUE(diagnostics.global_crosscheck_pass);
        EXPECT_FALSE(
            diagnostics.global_crosscheck_evaluation.all_pairs_feasible);
    }
}

TEST(ManeuverSelectionWorker,
    InteractionGraphReplacesLegacyExhaustiveEvaluation)
{
    constexpr std::uint64_t start = 15'000'000ULL;
    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, 2> workers;
    std::array<cs::ManeuverSelectionWorkerOutput, 2> outputs{};
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        auto worker_params = params(static_cast<int>(aircraft), 2);
        worker_params.exhaustive_test_mode = true;
        worker_params.interaction_graph_params.enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = -1.0e6;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start,
                100.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
    }

    exchangePackets(*workers[0], *workers[1], outputs[0], outputs[1]);
    ASSERT_TRUE(workers[0]->pushRemoteDecision(
        1, nominalPeerDecision(1, start)));
    ASSERT_TRUE(workers[1]->pushRemoteDecision(
        0, nominalPeerDecision(0, start)));
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start + 250'000ULL,
                5.0 + 100.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
        ASSERT_TRUE(outputs[aircraft].has_decision);
        EXPECT_TRUE(outputs[aircraft].decision.proposal_valid);
        // Both nodes are isolated, so no component combination is evaluated;
        // the legacy 7^2=49 search must not run when the graph is active.
        EXPECT_EQ(outputs[aircraft].decision.evaluated_combination_count, 0U);
        const auto diagnostics_message =
            workers[aircraft]->tryPopInteractionGraphDiagnostics();
        ASSERT_TRUE(diagnostics_message.has_value());
        ASSERT_TRUE(diagnostics_message.value());
        const auto & diagnostics = *diagnostics_message.value();
        EXPECT_TRUE(diagnostics.enabled);
        EXPECT_TRUE(diagnostics.component_proposal_used);
        EXPECT_TRUE(diagnostics.global_crosscheck_pass);
        EXPECT_TRUE(outputs[aircraft].decision.proposed_component_graph);
        EXPECT_EQ(
            outputs[aircraft].decision.proposed_candidate_ids,
            diagnostics.assembled_candidate_ids);
        EXPECT_EQ(diagnostics.assembled_candidate_valid_mask, 0b11U);
        EXPECT_EQ(
            outputs[aircraft].decision.proposed_candidate_valid_mask, 0b11U);
        EXPECT_NE(
            outputs[aircraft].decision.proposed_candidate_library_hash, 0U);
        EXPECT_NE(outputs[aircraft].decision.proposed_graph_hash, 0U);
        EXPECT_NE(outputs[aircraft].decision.proposed_component_hash, 0U);
        EXPECT_NE(
            outputs[aircraft].decision.proposed_component_solution_hash,
            0U);
    }
    EXPECT_EQ(
        outputs[0].decision.proposed_candidate_library_hash,
        outputs[1].decision.proposed_candidate_library_hash);
    EXPECT_EQ(
        outputs[0].decision.proposed_graph_hash,
        outputs[1].decision.proposed_graph_hash);
    EXPECT_EQ(
        outputs[0].decision.proposed_component_solution_hash,
        outputs[1].decision.proposed_component_solution_hash);

    auto peer_for_first = peerDecision(outputs[1].decision);
    auto peer_for_second = peerDecision(outputs[0].decision);
    ++peer_for_first.proposed_candidate_library_hash;
    ++peer_for_first.proposed_graph_hash;
    ++peer_for_first.proposed_component_hash;
    ++peer_for_first.proposed_component_solution_hash;
    ++peer_for_second.proposed_candidate_library_hash;
    ++peer_for_second.proposed_graph_hash;
    ++peer_for_second.proposed_component_hash;
    ++peer_for_second.proposed_component_solution_hash;
    ASSERT_TRUE(workers[0]->pushRemoteDecision(1, peer_for_first));
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, peer_for_second));
    ASSERT_TRUE(workers[0]->processPendingForTest());
    ASSERT_TRUE(workers[1]->processPendingForTest());
    const auto first_commit = workers[0]->tryPopOutput();
    const auto second_commit = workers[1]->tryPopOutput();
    ASSERT_TRUE(first_commit.has_value());
    ASSERT_TRUE(second_commit.has_value());
    const std::array<cs::ManeuverSelectionWorkerOutput, 2> commits{
        first_commit.value(), second_commit.value()};
    // Raw floating-point graph hashes may differ across CPU architectures.
    // Matching discrete candidate tuples remain the execution consensus key.
    EXPECT_TRUE(commits[0].decision.coordination_qualified);
    EXPECT_TRUE(commits[1].decision.coordination_qualified);
    EXPECT_TRUE(commits[0].decision.ownship_candidate_valid);
    EXPECT_TRUE(commits[1].decision.ownship_candidate_valid);
    EXPECT_FALSE(commits[0].decision.command_execution_requested);
    EXPECT_FALSE(commits[1].decision.command_execution_requested);
}

TEST(ManeuverSelectionWorker,
    MatchingComponentPeerActivationEdgeActivatesLocallySafeParticipant)
{
    constexpr std::uint64_t start = 15'500'000ULL;
    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, 2> workers;
    std::array<cs::ManeuverSelectionWorkerOutput, 2> outputs{};
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        auto worker_params = params(static_cast<int>(aircraft), 2);
        worker_params.exhaustive_test_mode = true;
        worker_params.interaction_graph_params.enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = 0.0;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        workers[aircraft]->setActivationEnabled(false);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start,
                5.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
    }

    exchangePackets(*workers[0], *workers[1], outputs[0], outputs[1]);
    ASSERT_TRUE(workers[0]->pushRemoteDecision(
        1, nominalPeerDecision(1, start)));
    ASSERT_TRUE(workers[1]->pushRemoteDecision(
        0, nominalPeerDecision(0, start)));
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start + 250'000ULL,
                5.0 + 5.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
    }
    auto commits = confirmTwoAircraftProposal(
        *workers[0], *workers[1], outputs[0], outputs[1]);
    ASSERT_FALSE(commits[1].decision.activation_requested);

    workers[1]->setActivationEnabled(true);
    auto matching_trigger = peerDecision(commits[0].decision);
    matching_trigger.activation_just_started = true;
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, matching_trigger));
    ASSERT_TRUE(workers[1]->processPendingForTest());
    const auto coordinated = pushBeliefAndProcess(
        *workers[1],
        beliefSnapshot(start + 300'000ULL, 500.0, 0.0, 20.0, 0.0));
    EXPECT_TRUE(coordinated.decision.activation_requested);
    EXPECT_TRUE(coordinated.decision.activation_just_started);
    EXPECT_EQ(
        coordinated.decision.ownship_candidate_id,
        commits[1].decision.selected_candidate_ids[1]);
}

TEST(ManeuverSelectionWorker,
    DifferentComponentPeerActivationEdgeDoesNotActivateParticipant)
{
    constexpr std::uint64_t start = 16'500'000ULL;
    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, 2> workers;
    std::array<cs::ManeuverSelectionWorkerOutput, 2> outputs{};
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        auto worker_params = params(static_cast<int>(aircraft), 2);
        worker_params.exhaustive_test_mode = true;
        worker_params.interaction_graph_params.enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = -1.0e6;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        workers[aircraft]->setActivationEnabled(false);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start,
                1'000.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
    }

    exchangePackets(*workers[0], *workers[1], outputs[0], outputs[1]);
    ASSERT_TRUE(workers[0]->pushRemoteDecision(
        1, nominalPeerDecision(1, start)));
    ASSERT_TRUE(workers[1]->pushRemoteDecision(
        0, nominalPeerDecision(0, start)));
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        outputs[aircraft] = pushBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start + 250'000ULL,
                5.0 + 1'000.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
    }
    auto commits = confirmTwoAircraftProposal(
        *workers[0], *workers[1], outputs[0], outputs[1]);
    ASSERT_FALSE(commits[1].decision.activation_requested);

    workers[1]->setActivationEnabled(true);
    auto other_component_trigger = peerDecision(commits[0].decision);
    other_component_trigger.activation_just_started = true;
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, other_component_trigger));
    ASSERT_TRUE(workers[1]->processPendingForTest());
    const auto unaffected = pushBeliefAndProcess(
        *workers[1],
        beliefSnapshot(start + 300'000ULL, 1'005.0, 0.0, 20.0, 0.0));
    EXPECT_FALSE(unaffected.decision.activation_requested);
    EXPECT_FALSE(unaffected.decision.activation_just_started);
}

TEST(FusionInputHistory, UsesPublishedInputForAllCandidateStartingStates)
{
    auto p = params(); p.exhaustive_test_mode = true;
    cs::ManeuverSelectionWorker worker(p);
    auto b = beliefSnapshot(1'152'000, 0, 0, 20, 0);
    b.timestamp_sample_us = 1'000'000;
    const auto actual = publishedInput(1'000'000, 3.0, 22.0);
    ASSERT_TRUE(worker.pushPublishedSetpoint(actual));
    // Nominal metadata is not the executed command either, e.g. during override.
    ASSERT_TRUE(worker.pushNominalSetpoint(nominalSnapshot(1'000'000, 20.0, -8.0)));
    const auto result = pushBeliefAndProcess(worker, b);
    ce::TrajectoryUncertainty uncertainty(p.uncertainty_params);
    ce::TrajectoryPredict predictor(p.predictor_params);
    ce::PredictState state; ce::PredictStateCovariance covariance;
    ASSERT_TRUE(uncertainty.initializeFromEstimatorBelief(b.belief, state, covariance));
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, actual.input, .152, state, covariance));
    EXPECT_GT(state.phi, 0.0);
    EXPECT_GT(state.V, 20.0);
    expectPacketInitialState(result, state, covariance);
    ASSERT_EQ(result.intent_packet_count, 7U);
    EXPECT_LT(result.intent_packets.front().candidate_input[3], 0.0F);
    EXPECT_GT(result.intent_packets.back().candidate_input[3], 0.0F);
}

TEST(FusionInputHistory, SplitsMeanAndCovarianceAtActualCommandSwitch)
{
    auto p = params(); cs::ManeuverSelectionWorker worker(p);
    auto b = beliefSnapshot(1'152'000, 0, 0, 20, 0);
    b.timestamp_sample_us = 1'000'000;
    const auto first = publishedInput(990'000, 5.0);
    const auto second = publishedInput(1'070'000, -8.0);
    ASSERT_TRUE(worker.pushPublishedSetpoint(first));
    ASSERT_TRUE(worker.pushPublishedSetpoint(second));
    // An input starting exactly at the endpoint must not be applied to the past.
    ASSERT_TRUE(worker.pushPublishedSetpoint(publishedInput(b.timestamp_us, 11.0)));
    const auto result = pushBeliefAndProcess(worker, b);
    ce::TrajectoryUncertainty uncertainty(p.uncertainty_params);
    ce::TrajectoryPredict predictor(p.predictor_params);
    ce::PredictState state; ce::PredictStateCovariance covariance;
    ASSERT_TRUE(uncertainty.initializeFromEstimatorBelief(b.belief, state, covariance));
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, first.input, .070, state, covariance));
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, second.input, .082, state, covariance));
    expectPacketInitialState(result, state, covariance);
}

TEST(FusionInputHistory, DoesNotBackfillMissingHistoryWithAFutureCommand)
{
    auto p = params(); cs::ManeuverSelectionWorker worker(p);
    auto b = beliefSnapshot(1'152'000, 0, 0, 20, 0);
    b.timestamp_sample_us = 1'000'000;
    ASSERT_TRUE(worker.pushPublishedSetpoint(publishedInput(1'050'000, 5.0)));
    ASSERT_TRUE(worker.pushOwnshipBelief(b));
    ASSERT_TRUE(worker.processPendingForTest());
    EXPECT_FALSE(worker.tryPopOutput().has_value());
    // Resume normally once the entire delayed interval has a known command.
    b.timestamp_sample_us = 1'050'000; b.timestamp_us = 1'202'000;
    const auto result = pushBeliefAndProcess(worker, b);
    EXPECT_GT(result.intent_packet_count, 0U);
}

TEST(FusionInputHistory, RejectsModeGapEvenIfAValidCommandArrivesLater)
{
    auto p = params(); cs::ManeuverSelectionWorker worker(p);
    auto b = beliefSnapshot(1'152'000, 0, 0, 20, 0);
    b.timestamp_sample_us = 1'000'000;
    ASSERT_TRUE(worker.pushPublishedSetpoint(publishedInput(990'000, 5.0)));
    ASSERT_TRUE(worker.pushPublishedSetpoint(publishedInput(1'040'000, 0, 20, false)));
    ASSERT_TRUE(worker.pushPublishedSetpoint(publishedInput(1'100'000, -5.0)));
    ASSERT_TRUE(worker.pushOwnshipBelief(b));
    ASSERT_TRUE(worker.processPendingForTest());
    EXPECT_FALSE(worker.tryPopOutput().has_value());
}

TEST(FusionInputHistory, IgnoresStaleInputAndCoalescesUnchangedPublications)
{
    auto p = params(); cs::ManeuverSelectionWorker worker(p);
    const auto actual = publishedInput(900'000, 3.0);
    ASSERT_TRUE(worker.pushPublishedSetpoint(actual));
    for (std::uint64_t t = 910'000; t < 1'050'000; t += 10'000)
        ASSERT_TRUE(worker.pushPublishedSetpoint(publishedInput(t, 3.0)));
    ASSERT_TRUE(worker.pushPublishedSetpoint(publishedInput(950'000, -10.0)));
    auto b = beliefSnapshot(1'152'000, 0, 0, 20, 0);
    b.timestamp_sample_us = 1'000'000;
    const auto result = pushBeliefAndProcess(worker, b);
    ce::TrajectoryUncertainty uncertainty(p.uncertainty_params);
    ce::TrajectoryPredict predictor(p.predictor_params);
    ce::PredictState state; ce::PredictStateCovariance covariance;
    ASSERT_TRUE(uncertainty.initializeFromEstimatorBelief(b.belief, state, covariance));
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, actual.input, .152, state, covariance));
    expectPacketInitialState(result, state, covariance);
}

TEST(FusionInputHistory, UndelayedBeliefNeedsNoCommandHistory)
{
    cs::ManeuverSelectionWorker worker(params());
    const auto result = pushBeliefAndProcess(
        worker, beliefSnapshot(1'000'000, 0, 0, 20, 0));
    ASSERT_GT(result.intent_packet_count, 0U);
    EXPECT_FLOAT_EQ(result.intent_packets[0].initial_state[6], 0.0F);
}

TEST(FusionInputHistory, OverwrittenHistoryCannotBeUsedForAnOlderBelief)
{
    cs::ManeuverSelectionWorker worker(params());
    for (std::uint64_t i = 0; i < 300; ++i) {
        ASSERT_TRUE(worker.pushPublishedSetpoint(
            publishedInput(1'000'000 + i * 1'000, i % 2 ? 3.0 : -3.0)));
        ASSERT_TRUE(worker.processPendingForTest());
    }
    auto b = beliefSnapshot(1'152'000, 0, 0, 20, 0);
    b.timestamp_sample_us = 1'000'000;
    ASSERT_TRUE(worker.pushOwnshipBelief(b));
    ASSERT_TRUE(worker.processPendingForTest());
    EXPECT_FALSE(worker.tryPopOutput().has_value());
    b.timestamp_sample_us = 1'200'000; b.timestamp_us = 1'352'000;
    const auto result = pushBeliefAndProcess(worker, b);
    EXPECT_GT(result.intent_packet_count, 0U);
}

TEST(FusionInputHistory, DroppedPublicationInvalidatesTheHistory)
{
    cs::ManeuverSelectionWorker worker(params());
    bool dropped = false;
    for (std::uint64_t i = 0; i < 10'000; ++i) {
        if (!worker.pushPublishedSetpoint(publishedInput(1'000'000 + i, 3.0))) {
            dropped = true; break;
        }
    }
    ASSERT_TRUE(dropped);
    ASSERT_TRUE(worker.processPendingForTest());
    auto b = beliefSnapshot(1'152'000, 0, 0, 20, 0);
    b.timestamp_sample_us = 1'000'000;
    ASSERT_TRUE(worker.pushOwnshipBelief(b));
    ASSERT_TRUE(worker.processPendingForTest());
    EXPECT_FALSE(worker.tryPopOutput().has_value());
}
