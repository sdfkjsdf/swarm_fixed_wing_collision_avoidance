#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>

namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

TEST(SpscQueue, EntryBatchDoesNotChaseProducerAndRetainsFifoAcrossWrap)
{
    collision_avoidance::common::SpscQueue<int, 4> queue;
    EXPECT_EQ(queue.sizeForConsumer(), 0U);
    for (int round = 0; round < 10; ++round) {
        ASSERT_TRUE(queue.try_push(0));
        ASSERT_TRUE(queue.try_push(1));
        const auto count = queue.sizeForConsumer();
        ASSERT_EQ(count, 2U);
        for (std::size_t i = 0; i < count; ++i) {
            ASSERT_EQ(queue.try_pop(), static_cast<int>(i));
            ASSERT_TRUE(queue.try_push(static_cast<int>(i) + 2));
        }
        // Arrivals during the pass belong to the next pass, not the current one.
        EXPECT_EQ(queue.sizeForConsumer(), 2U);
        EXPECT_EQ(queue.try_pop(), 2);
        EXPECT_EQ(queue.try_pop(), 3);
        EXPECT_FALSE(queue.try_pop());
    }
    for (int i = 0; i < 4; ++i) ASSERT_TRUE(queue.try_push(i));
    EXPECT_EQ(queue.sizeForConsumer(), 4U);
    EXPECT_FALSE(queue.try_push(4));
}

TEST(OrderedSpscInbox, IsolatesStreamsAndRetainsGlobalFifoAcrossWrap)
{
    collision_avoidance::common::OrderedSpscInbox<int, 3, 4> inbox;
    std::array<int, 2> batch{};
    for (int round = 0; round < 10; ++round) {
        for (int i = 0; i < 4; ++i) ASSERT_TRUE(inbox.try_push(1, i));
        ASSERT_FALSE(inbox.try_push(1, 999));
        // A full peer queue must not consume local or another peer's reserve.
        ASSERT_TRUE(inbox.try_push(0, 4));
        ASSERT_TRUE(inbox.try_push(2, 5));
        ASSERT_FALSE(inbox.try_push(3, 999));
        for (int i = 0; i < 6; i += 2) {
            ASSERT_EQ(inbox.drainTo(batch), 2U);
            EXPECT_EQ(batch[0], i);
            EXPECT_EQ(batch[1], i + 1);
        }
        EXPECT_EQ(inbox.drainTo(batch), 0U);
        // Bulk removal frees slots before a caller runs expensive handlers.
        ASSERT_TRUE(inbox.try_push(2, 6));
        ASSERT_TRUE(inbox.try_push(0, 7));
        ASSERT_EQ(inbox.drainTo(batch), 2U);
        ASSERT_TRUE(inbox.try_push(0, 8));
        EXPECT_EQ(batch[0], 6);
        EXPECT_EQ(batch[1], 7);
        ASSERT_EQ(inbox.drainTo(batch), 1U);
        EXPECT_EQ(batch[0], 8);
    }
}

TEST(OrderedSpscInbox, ConcurrentProducerPreservesOrderWithoutUnboundedDrain)
{
    collision_avoidance::common::OrderedSpscInbox<int, 5, 64> inbox;
    constexpr int count = 100'000;
    std::thread producer([&] {
        for (int i = 0; i < count; ++i) {
            while (!inbox.try_push(static_cast<std::size_t>(i % 5), i))
                std::this_thread::yield();
        }
    });
    std::array<int, 64> batch{};
    int received = 0;
    while (received < count) {
        const auto size = inbox.drainTo(batch);
        EXPECT_LE(size, batch.size());
        for (std::size_t i = 0; i < size; ++i) EXPECT_EQ(batch[i], received++);
        if (!size) std::this_thread::yield();
    }
    producer.join();
    EXPECT_EQ(inbox.drainTo(batch), 0U);
}

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
        EXPECT_FLOAT_EQ(output.intent_packets[i].initial_roll_setpoint_rad,
                        float(state.phi_setpoint));
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

cs::ManeuverSelectionWorkerOutput pushGraphBeliefAndProcess(
    cs::ManeuverSelectionWorker & worker,
    const cs::ManeuverSelectionBeliefSnapshot & belief)
{
    // These straight-flight coordination fixtures now supply the actual held
    // command explicitly. Production receives this from Formation's publisher.
    EXPECT_TRUE(worker.pushPublishedSetpoint(publishedInput(belief.timestamp_us, 0)));
    return pushBeliefAndProcess(worker, belief);
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
    peer.activation_timestamp_us = decision.activation_timestamp_us;
    peer.local_activation_request_timestamp_us =
        decision.local_activation_request_timestamp_us;
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

TEST(ManeuverSelectionWorker, TimerRefreshesSevenCandidatesWithoutAnotherBelief)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    cs::ManeuverSelectionWorker worker(p);
    constexpr std::uint64_t start = 1'000'000;
    const auto actual = publishedInput(start, 3.0);
    ASSERT_TRUE(worker.pushPublishedSetpoint(actual));
    pushBeliefAndProcess(worker, beliefSnapshot(start, 0, 0, 20, 0));
    const auto early = beliefSnapshot(start + 48'000, .96, 0, 20, 0);
    ASSERT_TRUE(worker.pushOwnshipBelief(early));
    ASSERT_TRUE(worker.processPendingForTest());
    EXPECT_FALSE(worker.tryPopOutput());
    EXPECT_FALSE(worker.processPendingForTest(1'999));
    EXPECT_FALSE(worker.tryPopOutput());

    ASSERT_TRUE(worker.processPendingForTest(2'000));
    const auto output = worker.tryPopOutput();
    ASSERT_TRUE(output);
    ASSERT_EQ(output->intent_packet_count, 7U);
    for (const auto & packet : output->intent_packets)
        EXPECT_EQ(packet.source_timestamp_us, start + 50'000);

    ce::TrajectoryUncertainty uncertainty(p.uncertainty_params);
    ce::TrajectoryPredict predictor(p.predictor_params);
    ce::PredictState state;
    ce::PredictStateCovariance covariance;
    ASSERT_TRUE(uncertainty.initializeFromEstimatorBelief(early.belief, state, covariance));
    state.phi_setpoint = predictor.rollSetpointAfter(0.0, actual.input, .048);
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, actual.input, .002, state, covariance));
    expectPacketInitialState(*output, state, covariance);
    EXPECT_FALSE(worker.processPendingForTest(2'000));
    EXPECT_FALSE(worker.tryPopOutput());
}

TEST(ManeuverSelectionWorker, TimerDoesNotDiscardMeasurementsBehindPredictedTime)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    cs::ManeuverSelectionWorker worker(p);
    constexpr std::uint64_t start = 2'000'000;
    const auto actual = publishedInput(start, 0.0);
    ASSERT_TRUE(worker.pushPublishedSetpoint(actual));
    pushBeliefAndProcess(worker, beliefSnapshot(start, 0, 0, 20, 0));
    ASSERT_TRUE(worker.processPendingForTest(50'000));
    ASSERT_TRUE(worker.tryPopOutput());

    // A newer measured state can legitimately precede the last predicted frame.
    const auto late = beliefSnapshot(start + 49'000, 100, 0, 20, 0);
    ASSERT_TRUE(worker.pushOwnshipBelief(late));
    ASSERT_TRUE(worker.processPendingForTest());
    EXPECT_FALSE(worker.tryPopOutput());
    ASSERT_TRUE(worker.processPendingForTest(51'000));
    const auto output = worker.tryPopOutput();
    ASSERT_TRUE(output);
    ce::TrajectoryUncertainty uncertainty(p.uncertainty_params);
    ce::TrajectoryPredict predictor(p.predictor_params);
    ce::PredictState state;
    ce::PredictStateCovariance covariance;
    ASSERT_TRUE(uncertainty.initializeFromEstimatorBelief(late.belief, state, covariance));
    state.phi_setpoint = 0.0;
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, actual.input, .051, state, covariance));
    expectPacketInitialState(*output, state, covariance);
    EXPECT_FALSE(worker.processPendingForTest(0)); // no time reversal / duplicate
    EXPECT_FALSE(worker.tryPopOutput());
}

TEST(ManeuverSelectionWorker, TimerCannotInventHistoryOrRefreshAStaleBelief)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    cs::ManeuverSelectionWorker missing(p), stale(p);
    constexpr std::uint64_t start = 3'000'000;
    pushBeliefAndProcess(missing, beliefSnapshot(start, 0, 0, 20, 0));
    EXPECT_FALSE(missing.processPendingForTest(50'000));
    EXPECT_FALSE(missing.tryPopOutput());
    ASSERT_TRUE(stale.pushPublishedSetpoint(publishedInput(start, 0)));
    pushBeliefAndProcess(stale, beliefSnapshot(start, 0, 0, 20, 0));
    EXPECT_FALSE(stale.processPendingForTest(p.maximum_belief_delay_us + 1));
    EXPECT_FALSE(stale.tryPopOutput());
    const auto fresh = pushBeliefAndProcess(stale,
        beliefSnapshot(start + 1'100'000, 22, 0, 20, 0));
    EXPECT_EQ(fresh.intent_packet_count, 7U);
}

TEST(ManeuverSelectionWorker, TimerSelectionKeepsTheCommonEpochAndFrozenLibrary)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    p.interaction_graph_params.enabled = true;
    cs::ManeuverSelectionWorker local(p);
    p.vehicle_id = 1;
    cs::ManeuverSelectionWorker peer(p);
    constexpr std::uint64_t start = 4'000'000;
    ASSERT_TRUE(local.pushPublishedSetpoint(publishedInput(start, 0)));
    const auto own = pushGraphBeliefAndProcess(local, beliefSnapshot(start, 0, 0, 20, 0));
    const auto other = pushGraphBeliefAndProcess(peer, beliefSnapshot(start, 1000, 0, 20, 0));
    exchangePackets(local, peer, own, other);
    for (std::uint64_t elapsed : {50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        ASSERT_TRUE(local.processPendingForTest(elapsed));
        ASSERT_TRUE(local.tryPopOutput());
    }
    ASSERT_TRUE(local.processPendingForTest(250'000));
    const auto output = local.tryPopOutput();
    ASSERT_TRUE(output);
    ASSERT_TRUE(output->has_decision);
    EXPECT_TRUE(output->decision.proposal_valid);
    EXPECT_EQ(output->decision.proposal_epoch, start / 250'000);
    EXPECT_EQ(output->decision.proposal_timestamp_us, start + 250'000);
    EXPECT_EQ(output->selection_epoch, start / 250'000 + 1);
    EXPECT_FALSE(output->decision.coordination_qualified);
    EXPECT_FALSE(output->decision.command_execution_requested);
}

TEST(ManeuverSelectionWorker, ReusedFrozenStorageDoesNotLeakAcrossIncompleteEpoch)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    p.interaction_graph_params.enabled = true;
    p.masd_diagnostics_enabled = true;
    cs::ManeuverSelectionWorker local(p);
    p.vehicle_id = 1;
    cs::ManeuverSelectionWorker peer(p);
    constexpr std::uint64_t start = 4'000'000;
    const auto own = pushGraphBeliefAndProcess(local, beliefSnapshot(start, 0, 0, 20, 0));
    const auto other = pushGraphBeliefAndProcess(peer, beliefSnapshot(start, 1000, 0, 20, 0));
    exchangePackets(local, peer, own, other);

    const auto check = [&](std::uint64_t elapsed, bool complete) {
        const auto output = pushGraphBeliefAndProcess(local,
            beliefSnapshot(start + elapsed, 20.0e-6 * elapsed, 0, 20, 0));
        const auto * records = local.stoppedGraphDiagnostics();
        ASSERT_NE(records, nullptr);
        ASSERT_GT(records->size, 0U);
        const auto & graph = records->records[records->size - 1].value;
        EXPECT_EQ(graph.graph.selection_epoch, (start + elapsed) / 250'000 - 1);
        EXPECT_EQ(graph.graph.valid(), complete);
        EXPECT_EQ(graph.candidate_ready_mask, complete ? 0b11U : 0b01U);
        if (!complete) {
            EXPECT_EQ(graph.status, cs::InteractionGraphEvaluationStatus::CandidateSetsIncomplete);
            EXPECT_FALSE(output.decision.proposal_valid);
        }
    };
    check(250'000, true);
    // Storage still contains the peer's old library, but epoch readiness is reset.
    check(500'000, false);
    const auto fresh = pushGraphBeliefAndProcess(peer,
        beliefSnapshot(start + 500'000, 1010, 0, 20, 0));
    for (std::size_t i = 0; i < fresh.intent_packet_count; ++i)
        ASSERT_TRUE(local.pushRemoteIntent(1, fresh.intent_packets[i]));
    ASSERT_TRUE(local.processPendingForTest());
    check(750'000, true);
}

TEST(ManeuverSelectionWorker, PeerAgreementDoesNotRequireANewTimerPrediction)
{
    for (const std::uint64_t elapsed : {2'000ULL, 60'000ULL}) {
        auto p = params();
        p.exhaustive_test_mode = true;
        p.interaction_graph_params.enabled = true;
        cs::ManeuverSelectionWorker local(p);
        p.vehicle_id = 1;
        cs::ManeuverSelectionWorker peer(p);
        constexpr std::uint64_t start = 4'000'000;
        const auto own = pushGraphBeliefAndProcess(local, beliefSnapshot(start, 0, 0, 20, 0));
        const auto other = pushGraphBeliefAndProcess(peer, beliefSnapshot(start, 1000, 0, 20, 0));
        exchangePackets(local, peer, own, other);
        const auto proposal = pushGraphBeliefAndProcess(local,
            beliefSnapshot(start + 250'000, 5, 0, 20, 0));
        const auto peer_proposal = pushGraphBeliefAndProcess(peer,
            beliefSnapshot(start + 250'000, 1005, 0, 20, 0));
        ASSERT_TRUE(proposal.decision.proposal_valid);
        ASSERT_TRUE(peer_proposal.decision.proposal_valid);

        // Invalidate subsequent execution history AFTER the frozen library was
        // built: this forbids a new timer prediction, not proposal agreement.
        ASSERT_TRUE(local.pushPublishedSetpoint(
            publishedInput(start + 250'001, 0, 20, false)));
        ASSERT_TRUE(local.pushRemoteDecision(1, peerDecision(peer_proposal.decision)));
        ASSERT_TRUE(local.processPendingForTest(elapsed));
        const auto committed = local.tryPopOutput();
        ASSERT_TRUE(committed);
        EXPECT_TRUE(committed->decision.coordination_qualified);
        EXPECT_EQ(committed->generated_timestamp_us, start + 250'000);
        EXPECT_EQ(committed->intent_packet_count, 0U);
        EXPECT_FALSE(local.processPendingForTest(60'000));
        EXPECT_FALSE(local.tryPopOutput());
    }
}

TEST(ManeuverSelectionWorker, DeferredRemoteReconstructionDoesNotBlockTrajectoryTicks)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    p.interaction_graph_params.enabled = true;
    cs::ManeuverSelectionWorker local(p);
    p.vehicle_id = 1;
    cs::ManeuverSelectionWorker peer(p);
    constexpr std::uint64_t start = 4'000'000;
    const auto own = pushGraphBeliefAndProcess(local, beliefSnapshot(start, 0, 0, 20, 0));
    const auto other = pushGraphBeliefAndProcess(peer, beliefSnapshot(start, 1000, 0, 20, 0));
    ASSERT_EQ(own.intent_packet_count, 7U);
    for (std::size_t i = 0; i < other.intent_packet_count; ++i)
        ASSERT_TRUE(local.pushRemoteIntent(1, other.intent_packets[i]));
    ASSERT_TRUE(local.pushPublishedSetpoint(publishedInput(start, 0.0)));
    for (std::uint64_t tick = 50'000; tick <= 200'000; tick += 50'000) {
        ASSERT_TRUE(local.processPendingForTest(tick, true, false));
        const auto output = local.tryPopOutput();
        ASSERT_TRUE(output);
        EXPECT_EQ(output->intent_packet_count, 7U);
        EXPECT_EQ(output->generated_timestamp_us, start + tick);
    }
    // Finish the deferred batch, then evaluate the original frozen epoch.
    ASSERT_TRUE(local.processPendingForTest(200'000));
    ASSERT_TRUE(local.processPendingForTest(250'000));
    const auto proposal = local.tryPopOutput();
    ASSERT_TRUE(proposal);
    EXPECT_TRUE(proposal->decision.proposal_valid);
}

TEST(ManeuverSelectionWorker, PeerAgreementBypassesPendingRemoteReconstruction)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    p.interaction_graph_params.enabled = true;
    cs::ManeuverSelectionWorker local(p);
    p.vehicle_id = 1;
    cs::ManeuverSelectionWorker peer(p);
    constexpr std::uint64_t start = 4'000'000;
    const auto own = pushGraphBeliefAndProcess(local, beliefSnapshot(start, 0, 0, 20, 0));
    const auto other = pushGraphBeliefAndProcess(peer, beliefSnapshot(start, 1000, 0, 20, 0));
    exchangePackets(local, peer, own, other);
    const auto proposal = pushGraphBeliefAndProcess(local, beliefSnapshot(start + 250'000, 5, 0, 20, 0));
    const auto peer_proposal = pushGraphBeliefAndProcess(peer, beliefSnapshot(start + 250'000, 1005, 0, 20, 0));
    ASSERT_TRUE(proposal.decision.proposal_valid);
    ASSERT_TRUE(peer_proposal.decision.proposal_valid);
    for (std::size_t i = 0; i < peer_proposal.intent_packet_count; ++i)
        ASSERT_TRUE(local.pushRemoteIntent(1, peer_proposal.intent_packets[i]));
    ASSERT_TRUE(local.pushRemoteDecision(1, peerDecision(peer_proposal.decision)));
    ASSERT_TRUE(local.processPendingForTest(2'000, true, false));
    const auto committed = local.tryPopOutput();
    ASSERT_TRUE(committed);
    EXPECT_TRUE(committed->decision.coordination_qualified);
    EXPECT_EQ(committed->decision.selected_candidate_ids, proposal.decision.proposed_candidate_ids);
    EXPECT_EQ(committed->intent_packet_count, 0U);
}

TEST(ManeuverSelectionWorker, RunningTimerRefreshesWithoutMoreInputOrDiagnostics)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    ASSERT_FALSE(p.stopped_stage_timing_enabled);
    auto worker = std::make_unique<cs::ManeuverSelectionWorker>(p);
    constexpr std::uint64_t start = 5'000'000;
    ASSERT_TRUE(worker->pushPublishedSetpoint(publishedInput(start, 0)));
    ASSERT_TRUE(worker->pushOwnshipBelief(beliefSnapshot(start, 0, 0, 20, 0)));
    ASSERT_TRUE(worker->start());
    std::array<std::uint64_t, 2> stamps{};
    std::size_t count = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (count < stamps.size() && std::chrono::steady_clock::now() < deadline) {
        if (const auto output = worker->tryPopOutput()) {
            EXPECT_EQ(output->intent_packet_count, 7U);
            stamps[count++] = output->intent_packets[0].source_timestamp_us;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    worker->stop();
    ASSERT_EQ(count, stamps.size());
    EXPECT_EQ(stamps[0], start);
    EXPECT_GE(stamps[1], start + p.trajectory_refresh_period_us);
    EXPECT_EQ(worker->droppedOutputCount(), 0U);
}

TEST(ManeuverSelectionWorker, DeferredSearchKeepsRefreshingAndPreservesFrozenProposal)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    p.interaction_graph_params.enabled = true;
    auto reference = std::make_unique<cs::ManeuverSelectionWorker>(p);
    auto deferred = std::make_unique<cs::ManeuverSelectionWorker>(p);
    p.vehicle_id = 1;
    auto peer = std::make_unique<cs::ManeuverSelectionWorker>(p);
    constexpr std::uint64_t start = 6'000'000;
    for (std::uint64_t offset : {0ULL, 50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        const auto own = beliefSnapshot(start + offset, 20e-6 * offset, 0, 20, 0);
        pushGraphBeliefAndProcess(*reference, own);
        pushGraphBeliefAndProcess(*deferred, own);
        const auto other = pushGraphBeliefAndProcess(*peer,
            beliefSnapshot(start + offset, 1000 + 20e-6 * offset, 0, 20, 0));
        for (std::size_t i = 0; i < other.intent_packet_count; ++i) {
            ASSERT_TRUE(reference->pushRemoteIntent(1, other.intent_packets[i]));
            ASSERT_TRUE(deferred->pushRemoteIntent(1, other.intent_packets[i]));
        }
    }
    const auto own = beliefSnapshot(start + 250'000, 5, 0, 20, 0);
    const auto expected = pushGraphBeliefAndProcess(*reference, own);
    ASSERT_TRUE(expected.decision.proposal_valid);
    ASSERT_TRUE(deferred->pushOwnshipBelief(own));
    ASSERT_TRUE(deferred->processPendingForTest(0, false)); // search queued, not run
    auto output = deferred->tryPopOutput();
    ASSERT_TRUE(output);
    EXPECT_EQ(output->intent_packet_count, 7U);
    EXPECT_FALSE(output->has_decision);

    // Deliberately change the live state and epoch while the job is pending.
    ASSERT_TRUE(deferred->pushOwnshipBelief(
        beliefSnapshot(start + 300'000, 700, 100, 0, 20)));
    ASSERT_TRUE(deferred->processPendingForTest(0, false));
    output = deferred->tryPopOutput();
    ASSERT_TRUE(output);
    EXPECT_EQ(output->intent_packet_count, 7U);
    EXPECT_EQ(output->intent_packets[0].source_timestamp_us, start + 300'000);
    EXPECT_FALSE(output->has_decision);

    ASSERT_TRUE(deferred->processPendingForTest()); // release the same kernel
    output = deferred->tryPopOutput();
    ASSERT_TRUE(output);
    ASSERT_TRUE(output->has_decision);
    EXPECT_EQ(output->decision.proposal_epoch, expected.decision.proposal_epoch);
    EXPECT_EQ(output->decision.proposal_timestamp_us, start + 250'000);
    EXPECT_EQ(output->decision.proposed_candidate_ids, expected.decision.proposed_candidate_ids);
    EXPECT_EQ(output->decision.proposed_candidate_source_timestamps_us,
              expected.decision.proposed_candidate_source_timestamps_us);
    EXPECT_EQ(output->decision.proposed_candidate_library_hash,
              expected.decision.proposed_candidate_library_hash);
    EXPECT_EQ(output->decision.proposed_graph_hash, expected.decision.proposed_graph_hash);
    EXPECT_EQ(deferred->skippedSelectionCount(), 0U);
    EXPECT_EQ(deferred->expiredSelectionCount(), 0U);
}

TEST(ManeuverSelectionWorker, BusySearchDoesNotBlockRefreshOrApplyAnExpiredResult)
{
    auto p = params();
    p.exhaustive_test_mode = true;
    cs::ManeuverSelectionWorker worker(p);
    constexpr std::uint64_t start = 7'000'000;
    pushBeliefAndProcess(worker, beliefSnapshot(start, 0, 0, 20, 0));
    for (std::uint64_t elapsed : {250'000ULL, 300'000ULL, 500'000ULL}) {
        ASSERT_TRUE(worker.pushOwnshipBelief(
            beliefSnapshot(start + elapsed, 20e-6 * elapsed, 0, 20, 0)));
        ASSERT_TRUE(worker.processPendingForTest(0, false));
        const auto output = worker.tryPopOutput();
        ASSERT_TRUE(output);
        EXPECT_EQ(output->intent_packet_count, 7U);
        EXPECT_FALSE(output->has_decision);
    }
    EXPECT_EQ(worker.skippedSelectionCount(), 1U);
    worker.processPendingForTest();
    EXPECT_EQ(worker.expiredSelectionCount(), 1U);
    EXPECT_FALSE(worker.tryPopOutput());
    const auto recovered = pushBeliefAndProcess(worker,
        beliefSnapshot(start + 750'000, 15, 0, 20, 0));
    EXPECT_TRUE(recovered.has_decision);
    EXPECT_EQ(recovered.decision.proposal_epoch, (start + 500'000) / 250'000);
}

TEST(ManeuverEvaluationWorker, BoundedMailboxHoldsSnapshotUntilReleaseAndRestarts)
{
    cs::ManeuverEvaluationWorker evaluator({}, {});
    auto * slot = evaluator.beginRequest();
    ASSERT_NE(slot, nullptr);
    slot->epoch = 42;
    slot->complete = false;
    evaluator.submit();
    EXPECT_EQ(evaluator.beginRequest(), nullptr);
    ASSERT_TRUE(evaluator.processOneForTest());
    ASSERT_NE(evaluator.readyResult(), nullptr);
    EXPECT_EQ(evaluator.readyResult()->request.epoch, 42U);
    EXPECT_EQ(evaluator.beginRequest(), nullptr);
    evaluator.release();
    EXPECT_EQ(evaluator.beginRequest(), slot);
    for (std::uint64_t epoch : {43ULL, 44ULL}) {
        evaluator.start();
        auto * request = evaluator.beginRequest();
        ASSERT_NE(request, nullptr);
        request->epoch = epoch;
        evaluator.submit();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!evaluator.readyResult() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const auto * result = evaluator.readyResult();
        ASSERT_NE(result, nullptr);
        EXPECT_EQ(result->request.epoch, epoch);
        EXPECT_GE(result->result.end_ns, result->result.start_ns);
        evaluator.release();
        evaluator.stop();
    }
}

TEST(ManeuverEvaluationWorker, FrozenGraphStartsCandidatesAfterHeldExecution)
{
    cs::InteractionGraphParams graph;
    graph.enabled = true;
    cs::ManeuverEvaluationWorker worker({}, graph);
    auto * request = worker.beginRequest();
    ASSERT_NE(request, nullptr);
    request->timestamp_us = 1'200'000;
    request->epoch = 4;
    request->aircraft_count = 2;
    request->complete = true;
    request->counts[0] = request->counts[1] = 7;
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    const auto table = ce::makeLevelTurnCandidateTable(20, 100);
    ce::TrajectoryIntentSender sender(predictor, table);
    ce::TrajectoryIntentReceiver receiver(predictor);
    ce::TrajectoryUncertainty uncertainty;
    auto reference = std::make_unique<cs::MultiAircraftExhaustiveCandidateIntentSets>();
    for (int v = 0; v < 2; ++v) {
        ce::PredictState state{30.0 * v, 10.0 * v, 100, 20, 3.0 * v, 0, .3};
        state.phi_setpoint = -.2;
        ce::PredictStateCovariance covariance{};
        for (int k = 0; k < 7; ++k) covariance[k * 7 + k] = .01;
        const auto held = table.inputs[0];
        ce::TrajectoryIntentPacket packet;
        for (int id = 0; id < 7; ++id) {
            ASSERT_TRUE(sender.buildForSelectedCandidate(1'000'000, id,
                state, covariance, packet, 4));
            packet.candidate_set_size = 7;
            packet.source_execution_input_available = true;
            packet.source_execution_input = {20, 100, 0, float(held.a_lat_cmd)};
            ASSERT_TRUE(receiver.receive(packet, request->candidates[v][id]));
        }
        auto x = request->candidates[v][0].cone[0].mean;
        auto p = request->candidates[v][0].cone[0].state_covariance;
        ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(predictor,
            request->candidates[v][0].source_execution_input, .2, x, p));
        // Independent construction: actual held input first, then each candidate.
        for (int id = 0; id < 7; ++id) {
            ASSERT_TRUE(sender.buildForSelectedCandidate(request->timestamp_us,
                id, x, p, packet, 4));
            packet.candidate_set_size = 7;
            ASSERT_TRUE(receiver.receive(packet, (*reference)[v][id]));
        }
    }
    worker.submit();
    ASSERT_TRUE(worker.processOneForTest());
    const auto * result = worker.readyResult();
    ASSERT_NE(result, nullptr);
    ASSERT_TRUE(result->result.certifications.valid);
    cs::ManeuverCombinationEvaluator pair_evaluator;
    for (int a = 0; a < 7; ++a) for (int b = 0; b < 7; ++b) {
        cs::CombinationEvaluation expected;
        ASSERT_TRUE(pair_evaluator.evaluatePair(request->timestamp_us,
            (*reference)[0][a], (*reference)[1][b], expected));
        ASSERT_NE(result->result.certifications.findPair(0, 1), nullptr);
        EXPECT_NEAR(result->result.certifications.findPair(0, 1)->find(a, b)->ad_m,
            expected.ad_m, 1e-9);
    }
    // Coordination must retain original wire identities, not the derived time.
    EXPECT_EQ(result->request.candidates[0][6].source_timestamp_us, 1'000'000U);
    EXPECT_EQ(result->request.candidates[0][6].candidate_input_revision,
        request->candidates[0][6].candidate_input_revision);
    EXPECT_EQ(result->result.certifications.source_timestamps_us[0], 1'200'000U);
    EXPECT_EQ(result->result.graph.graph.source_timestamps_us[0], 1'000'000U);
    worker.release();
    // No silent use of a hypothetical input if execution metadata is missing.
    request = worker.beginRequest();
    request->candidates[0][0].source_execution_input_available = false;
    worker.submit();
    ASSERT_TRUE(worker.processOneForTest());
    EXPECT_FALSE(worker.readyResult()->result.evaluated);
    EXPECT_EQ(worker.readyResult()->result.graph.status,
        cs::InteractionGraphEvaluationStatus::GraphInvalid);
    worker.release();
    request = worker.beginRequest();
    request->candidates[0][0].source_execution_input_available = true;
    request->timestamp_us = 5'000'001;
    worker.submit();
    ASSERT_TRUE(worker.processOneForTest());
    EXPECT_FALSE(worker.readyResult()->result.evaluated); // stale stays stale
    worker.release();
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
    const auto replay = [](bool enabled, bool stopped_timing = false) {
        auto p0 = params(0);
        auto p1 = params(1);
        p0.masd_diagnostics_enabled = enabled;
        p1.masd_diagnostics_enabled = enabled;
        p0.stopped_stage_timing_enabled = stopped_timing;
        p1.stopped_stage_timing_enabled = stopped_timing;
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
        for (const auto * worker : {&first, &second}) {
            if (const auto * records = worker->stoppedBudgetTraces())
                traces.insert(traces.end(), records->records.begin(),
                              records->records.begin() + records->size);
        }
        return std::make_pair(history, traces);
    };
    const auto ordinary_result = replay(false);
    const auto traced_result = replay(true);
    const auto buffered_result = replay(false, true);
    ASSERT_EQ(ordinary_result.first.size(), buffered_result.first.size());
    EXPECT_TRUE(buffered_result.second.empty()); // no legacy trace transport
    for (std::size_t k = 0; k < ordinary_result.first.size(); ++k) {
        const auto & a = ordinary_result.first[k];
        const auto & b = buffered_result.first[k];
        ASSERT_EQ(a.intent_packet_count, b.intent_packet_count);
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

TEST(ManeuverSelectionWorker, StoppedTimingIsBoundedAndKeepsFirstRecords)
{
    auto buffer = std::make_unique<cs::StoppedStageTiming>();
    for (std::size_t i = 0; i < cs::StoppedStageTiming::capacity + 3; ++i) {
        cs::StageTimingRecord record;
        record.source_us = i;
        buffer->append(record);
    }
    EXPECT_EQ(buffer->size, cs::StoppedStageTiming::capacity);
    EXPECT_EQ(buffer->dropped, 3U);
    EXPECT_EQ(buffer->records.front().source_us, 0U);
    EXPECT_EQ(buffer->records.back().source_us, cs::StoppedStageTiming::capacity - 1);
    for (std::size_t i = 0; i < cs::StoppedStageTiming::pipeline_capacity + 2; ++i) {
        cs::PipelineTimingRecord record;
        record.source_us = i;
        buffer->appendPipeline(record);
    }
    for (std::size_t i = 0; i < cs::StoppedStageTiming::belief_capacity + 2; ++i) {
        cs::BeliefTimingRecord record;
        record.source_us = i;
        buffer->appendBelief(record);
    }
    EXPECT_EQ(buffer->pipeline_dropped, 2U);
    EXPECT_EQ(buffer->belief_dropped, 2U);
    EXPECT_EQ(buffer->pipelines.front().source_us, 0U);
    EXPECT_EQ(buffer->beliefs.front().source_us, 0U);
    EXPECT_EQ(buffer->pipeline_size, cs::StoppedStageTiming::pipeline_capacity);
    EXPECT_EQ(buffer->belief_size, cs::StoppedStageTiming::belief_capacity);
}

TEST(ManeuverSelectionWorker, StoppedTimingDoesNotNeedBudgetDiagnostics)
{
    auto p = params(0);
    p.stopped_stage_timing_enabled = true;
    p.masd_diagnostics_enabled = false;
    p.exhaustive_test_mode = true;
    auto worker = std::make_unique<cs::ManeuverSelectionWorker>(p);
    for (const auto offset : {0ULL, 64'000ULL, 120'000ULL}) {
        const auto output = pushBeliefAndProcess(*worker,
            beliefSnapshot(3'000'000 + offset, -45, 0, 20, 0));
        EXPECT_EQ(output.intent_packet_count, 7U);
    }
    EXPECT_EQ(worker->stoppedBudgetTraces(), nullptr);
    std::ostringstream out;
    worker->stopAndWriteStageTiming(out);
    EXPECT_FALSE(worker->running());
    EXPECT_NE(out.str().find("[stop-stage-begin],2,0,3,0"), std::string::npos);
    EXPECT_NE(out.str().find("[stop-stage],1,3064000,"), std::string::npos);
    EXPECT_NE(out.str().find(",7,1,1\n"), std::string::npos);
    EXPECT_NE(out.str().find("[stop-stage-end],0,3"), std::string::npos);
    EXPECT_NE(out.str().find("[stop-pipeline-begin],1,0,3,0,3,0"), std::string::npos);
    EXPECT_NE(out.str().find("[stop-belief],3064000,"), std::string::npos);
    EXPECT_NE(out.str().find("[stop-pipeline-end],0,3,3"), std::string::npos);
}

TEST(ManeuverSelectionWorker, StoppedTimingJoinsThreadAndDisabledWritesNothing)
{
    auto p = params(0);
    auto off = std::make_unique<cs::ManeuverSelectionWorker>(p);
    std::ostringstream empty;
    off->stopAndWriteStageTiming(empty);
    EXPECT_TRUE(empty.str().empty());
    p.stopped_stage_timing_enabled = true;
    auto on = std::make_unique<cs::ManeuverSelectionWorker>(p);
    ASSERT_TRUE(on->start());
    ASSERT_TRUE(on->pushOwnshipBelief(beliefSnapshot(3'000'000, -45, 0, 20, 0)));
    std::ostringstream log;
    on->stopAndWriteStageTiming(log); // final drain is joined before serialization
    EXPECT_FALSE(on->running());
    EXPECT_NE(log.str().find("[stop-stage],1,"), std::string::npos);
    EXPECT_NE(log.str().find("[stop-stage-end],0,1"), std::string::npos);
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
    ASSERT_TRUE(first.pushPublishedSetpoint(publishedInput(start, 0.0)));
    ASSERT_TRUE(second.pushPublishedSetpoint(publishedInput(start, 0.0)));
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

TEST(ManeuverSelectionWorker, NominalIntentReuseIsLimitedToOneActivationUpdate)
{
    cs::ManeuverSelectionWorker local(params());
    cs::ManeuverSelectionWorker peer(params(1));
    constexpr std::uint64_t start = 6'500'000;
    ASSERT_TRUE(local.pushPublishedSetpoint(publishedInput(start, 0.0)));
    ASSERT_TRUE(peer.pushPublishedSetpoint(publishedInput(start, 0.0)));
    const auto own = pushBeliefAndProcess(local, beliefSnapshot(start, 0, 0, 20, 0));
    const auto other = pushBeliefAndProcess(peer, beliefSnapshot(start, 0, 100, 20, 0));
    exchangePackets(local, peer, own, other);
    auto output = pushBeliefAndProcess(local,
        beliefSnapshot(start + 250'000, 5, 0, 20, 0));
    const auto remote_proposal = pushBeliefAndProcess(peer,
        beliefSnapshot(start + 250'000, 5, 100, 20, 0));
    const auto commits = confirmTwoAircraftProposal(local, peer, output, remote_proposal);
    output = commits[0];
    ASSERT_TRUE(output.has_decision);
    ASSERT_TRUE(output.decision.coordination_qualified);
    EXPECT_FALSE(output.decision.post_release_evaluated);

    // A failed build in one update must not prevent recovery on new inputs.
    ASSERT_TRUE(local.pushNominalSetpoint(nominalSnapshot(start + 300'000)));
    auto heartbeat = peerDecision(commits[1].decision);
    heartbeat.nominal_setpoint_available = true;
    heartbeat.nominal_setpoint_timestamp_us = start + 300'000;
    heartbeat.nominal_ground_speed_command_mps = 20.0;
    heartbeat.nominal_altitude_command_m = 100.0;
    heartbeat.nominal_lateral_acceleration_mps2 = 0.0;
    ASSERT_TRUE(local.pushRemoteDecision(1, heartbeat));
    output = pushBeliefAndProcess(local,
        beliefSnapshot(start + 300'000, 6, 0, 20, 0));
    ASSERT_TRUE(output.decision.post_release_evaluated);
    EXPECT_EQ(output.decision.post_release_evaluation_timestamp_us, start + 300'000);
    const double straight_ad = output.decision.post_release_minimum_ad_m;

    // A successful build must not hide a changed Formation input next update.
    ASSERT_TRUE(local.pushNominalSetpoint(nominalSnapshot(start + 350'000, 20, 8)));
    output = pushBeliefAndProcess(local,
        beliefSnapshot(start + 350'000, 7, 0, 20, 0));
    ASSERT_TRUE(output.decision.post_release_evaluated);
    EXPECT_EQ(output.decision.post_release_evaluation_timestamp_us, start + 350'000);
    EXPECT_GT(std::abs(output.decision.post_release_minimum_ad_m - straight_ad), 1.0);
}

TEST(ManeuverSelectionWorker, UnavailableNominalDoesNotSuppressSevenCandidates)
{
    for (bool future_nominal : {false, true}) {
        SCOPED_TRACE(future_nominal);
        auto p0 = params(0);
        auto p1 = params(1);
        for (auto * p : {&p0, &p1}) {
            p->exhaustive_test_mode = true;
            p->evaluator_params.desired_separation_distance_m = 100.0;
            p->v4_maximum_nominal_age_us = 100'000;
        }
        auto first = std::make_unique<cs::ManeuverSelectionWorker>(p0);
        auto second = std::make_unique<cs::ManeuverSelectionWorker>(p1);
        constexpr std::uint64_t start = 6'500'000;
        ASSERT_TRUE(first->pushPublishedSetpoint(publishedInput(start, 0.0)));
        ASSERT_TRUE(second->pushPublishedSetpoint(publishedInput(start, 0.0)));
        cs::ManeuverSelectionWorkerOutput a, b;
        for (std::uint64_t offset = 0; offset <= 250'000; offset += 50'000) {
            ASSERT_TRUE(first->pushNominalSetpoint(nominalSnapshot(start + offset)));
            ASSERT_TRUE(second->pushNominalSetpoint(nominalSnapshot(start + offset)));
            a = pushBeliefAndProcess(*first, beliefSnapshot(start + offset, -40, 0, 20, 0));
            b = pushBeliefAndProcess(*second, beliefSnapshot(start + offset, 40, 0, -20, 0));
            exchangePackets(*first, *second, a, b);
        }
        const auto commits = confirmTwoAircraftProposal(*first, *second, a, b);
        ASSERT_TRUE(commits[0].decision.activation_requested);
        for (const auto offset : {300'000ULL, 350'000ULL}) {
            ASSERT_TRUE(first->pushNominalSetpoint(nominalSnapshot(start + offset)));
            ASSERT_TRUE(second->pushNominalSetpoint(nominalSnapshot(start + offset)));
            a = pushBeliefAndProcess(*first, beliefSnapshot(start + offset, -1000, 0, -20, 0));
            b = pushBeliefAndProcess(*second, beliefSnapshot(start + offset, 1000, 0, 20, 0));
            exchangePackets(*first, *second, a, b);
        }
        ASSERT_TRUE(a.decision.activation_requested);
        ASSERT_TRUE(a.decision.safe_rejoin_active);
        ce::TrajectoryPredict predictor(p0.predictor_params);
        ce::TrajectoryIntentReceiver receiver(predictor, p0.uncertainty_params);
        const auto check = [&](std::uint64_t offset) {
            SCOPED_TRACE(offset);
            auto output = pushBeliefAndProcess(*first,
                beliefSnapshot(start + offset, -1000, 0, -20, 0));
            EXPECT_EQ(output.intent_packet_count, 7U);
            EXPECT_TRUE(output.decision.activation_requested);
            for (std::size_t i = 0; i < output.intent_packet_count; ++i) {
                const auto & packet = output.intent_packets[i];
                EXPECT_EQ(packet.candidate_id, i);
                ce::ReceivedTrajectoryIntent received;
                EXPECT_TRUE(receiver.receive(packet, received));
            }
        };
        // Exercise the recorded +8 ms future-nominal failure while release
        // remains blocked. Nominal metadata no longer belongs to the library.
        ASSERT_TRUE(first->pushNominalSetpoint(nominalSnapshot(
            start + 400'000 + (future_nominal ? 8'000 : 0))));
        check(400'000);
        check(450'000); // Nominal becomes usable without relaxing time checks.
        check(550'000); // Stale nominal must not suppress avoidance either.
        auto invalid = nominalSnapshot(start + 600'000);
        invalid.valid = false;
        ASSERT_TRUE(first->pushNominalSetpoint(invalid));
        check(600'000);
    }
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
    worker_params.masd_diagnostics_enabled = true;
    cs::ManeuverSelectionWorker worker(worker_params);
    ASSERT_TRUE(worker.pushNominalSetpoint(nominalSnapshot(start)));

    const auto first_output = pushGraphBeliefAndProcess(
        worker, beliefSnapshot(start, 0.0, 0.0, 20.0, 0.0));
    ASSERT_EQ(first_output.intent_packet_count, 7U);
    static_cast<void>(pushGraphBeliefAndProcess(
        worker,
        beliefSnapshot(start + 250'000ULL, 5.0, 0.0, 20.0, 0.0)));

    const auto diagnostics_message =
        worker.stoppedGraphDiagnostics();
    ASSERT_NE(diagnostics_message, nullptr);
    ASSERT_GT(diagnostics_message->size, 0U);
    const auto & diagnostics = diagnostics_message->records[diagnostics_message->size - 1].value;
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
    ownship_params.masd_diagnostics_enabled = true;
    auto remote_params = params(1, 2);
    remote_params.exhaustive_test_mode = true;
    remote_params.interaction_graph_params.enabled = true;
    remote_params.masd_diagnostics_enabled = true;
    cs::ManeuverSelectionWorker ownship(ownship_params);
    cs::ManeuverSelectionWorker remote(remote_params);
    ASSERT_TRUE(ownship.pushNominalSetpoint(nominalSnapshot(start)));
    ASSERT_TRUE(remote.pushNominalSetpoint(nominalSnapshot(start)));

    static_cast<void>(pushGraphBeliefAndProcess(
        ownship, beliefSnapshot(start, 0.0, 0.0, 20.0, 0.0)));
    static_cast<void>(pushGraphBeliefAndProcess(
        remote, beliefSnapshot(start, 80.0, 0.0, -20.0, 0.0)));
    for (std::uint64_t offset : {
            50'000ULL, 100'000ULL, 150'000ULL, 200'000ULL}) {
        static_cast<void>(pushGraphBeliefAndProcess(
            ownship,
            beliefSnapshot(
                start + offset, 20.0e-6 * offset,
                0.0, 20.0, 0.0)));
    }

    const auto future_epoch_output = pushGraphBeliefAndProcess(
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

    static_cast<void>(pushGraphBeliefAndProcess(
        ownship,
        beliefSnapshot(start + 250'000ULL, 5.0, 0.0, 20.0, 0.0)));
    const auto old_epoch_diagnostics =
        ownship.stoppedGraphDiagnostics();
    ASSERT_NE(old_epoch_diagnostics, nullptr);
    ASSERT_GT(old_epoch_diagnostics->size, 0U);
    for (std::uint64_t offset : {
            300'000ULL, 350'000ULL, 400'000ULL, 450'000ULL}) {
        static_cast<void>(pushGraphBeliefAndProcess(
            ownship,
            beliefSnapshot(
                start + offset, 20.0e-6 * offset,
                0.0, 20.0, 0.0)));
    }
    static_cast<void>(pushGraphBeliefAndProcess(
        ownship,
        beliefSnapshot(start + 500'000ULL, 10.0, 0.0, 20.0, 0.0)));

    const auto diagnostics_message =
        ownship.stoppedGraphDiagnostics();
    ASSERT_NE(diagnostics_message, nullptr);
    ASSERT_GT(diagnostics_message->size, 0U);
    const auto & diagnostics = diagnostics_message->records[diagnostics_message->size - 1].value;
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
    worker_params.masd_diagnostics_enabled = true;
        // Deliberately wrong screening threshold: the aircraft are closer
        // than the hard budget but are split into isolated components.
        worker_params.interaction_graph_params.ad_screen_m = -1.0e6;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
        outputs[aircraft] = pushGraphBeliefAndProcess(
            *workers[aircraft],
            beliefSnapshot(
                start + 250'000ULL,
                5.0 + 5.0 * static_cast<double>(aircraft),
                0.0,
                20.0,
                0.0));
        const auto diagnostics_message =
            workers[aircraft]->stoppedGraphDiagnostics();
        ASSERT_NE(diagnostics_message, nullptr);
        ASSERT_GT(diagnostics_message->size, 0U);
        const auto & diagnostics = diagnostics_message->records[diagnostics_message->size - 1].value;
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
    worker_params.masd_diagnostics_enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = 0.0;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
            workers[aircraft]->stoppedGraphDiagnostics();
        ASSERT_NE(diagnostics_message, nullptr);
        ASSERT_GT(diagnostics_message->size, 0U);
        const auto & diagnostics = diagnostics_message->records[diagnostics_message->size - 1].value;
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
    worker_params.masd_diagnostics_enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = -1.0e6;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
            workers[aircraft]->stoppedGraphDiagnostics();
        ASSERT_NE(diagnostics_message, nullptr);
        ASSERT_GT(diagnostics_message->size, 0U);
        const auto & diagnostics = diagnostics_message->records[diagnostics_message->size - 1].value;
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
    worker_params.masd_diagnostics_enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = 0.0;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        workers[aircraft]->setActivationEnabled(false);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
    matching_trigger.activation_requested = true;
    matching_trigger.activation_just_started = true;
    matching_trigger.activation_timestamp_us = start + 250'000;
    matching_trigger.local_activation_request_timestamp_us = start + 250'000;
    // Even a paused remote reconstruction queue must not hold an activation
    // event behind its trajectory packets. The last complete set stays usable.
    for (std::size_t i = 0; i < outputs[0].intent_packet_count; ++i)
        ASSERT_TRUE(workers[1]->pushRemoteIntent(0, outputs[0].intent_packets[i]));
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, matching_trigger));
    ASSERT_TRUE(workers[1]->processPendingForTest(0, true, false));
    ASSERT_TRUE(workers[1]->pushOwnshipBelief(
        beliefSnapshot(start + 300'000ULL, 500.0, 0.0, 20.0, 0.0)));
    ASSERT_TRUE(workers[1]->processPendingForTest(0, true, false));
    const auto coordinated_output = workers[1]->tryPopOutput();
    ASSERT_TRUE(coordinated_output);
    const auto & coordinated = *coordinated_output;
    EXPECT_TRUE(coordinated.decision.activation_requested);
    EXPECT_TRUE(coordinated.decision.activation_just_started);
    EXPECT_EQ(coordinated.decision.local_activation_request_timestamp_us, 0U);
    EXPECT_EQ(
        coordinated.decision.ownship_candidate_id,
        commits[1].decision.selected_candidate_ids[1]);

    // Joining alone must not originate a request. A subsequent local unsafe
    // pair must originate one, without changing the execution episode ID.
    const auto local_risk = pushGraphBeliefAndProcess(*workers[1],
        beliefSnapshot(start + 350'000, 5.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(local_risk.decision.activation_requested);
    ASSERT_LT(local_risk.decision.ad_m, 0.0);
    EXPECT_FALSE(local_risk.decision.activation_just_started);
    EXPECT_EQ(local_risk.decision.activation_timestamp_us,
              coordinated.decision.activation_timestamp_us);
    EXPECT_EQ(local_risk.decision.local_activation_request_timestamp_us,
              start + 350'000);
    const auto still_unsafe = pushGraphBeliefAndProcess(*workers[1],
        beliefSnapshot(start + 400'000, 5.0, 0.0, 20.0, 0.0));
    ASSERT_TRUE(still_unsafe.decision.activation_requested);
    ASSERT_LT(still_unsafe.decision.ad_m, 0.0);
    EXPECT_EQ(still_unsafe.decision.local_activation_request_timestamp_us,
              local_risk.decision.local_activation_request_timestamp_us);
}

static void verifyDeferredComponentActivation(
    bool peer_ended, bool lost_edge = false, bool late_active = false,
    bool stale = false, bool mismatched_tuple = false, bool peer_only = false,
    bool reordered_participation = false)
{
    constexpr std::uint64_t start = 15'500'000ULL;
    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, 2> workers;
    std::array<cs::ManeuverSelectionWorkerOutput, 2> outputs{};
    for (std::size_t aircraft = 0; aircraft < workers.size(); ++aircraft) {
        auto worker_params = params(static_cast<int>(aircraft), 2);
        worker_params.exhaustive_test_mode = true;
        worker_params.evaluator_params.stale_timeout_s = 0.275;
        worker_params.interaction_graph_params.enabled = true;
    worker_params.masd_diagnostics_enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = 0.0;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        workers[aircraft]->setActivationEnabled(false);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
    matching_trigger.activation_requested = true;
    matching_trigger.activation_just_started = !lost_edge;
    matching_trigger.activation_timestamp_us = start + 250'000;
    matching_trigger.local_activation_request_timestamp_us =
        peer_only ? 0 : start + 250'000;
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, matching_trigger));
    ASSERT_TRUE(workers[1]->processPendingForTest());
    const auto unavailable = pushGraphBeliefAndProcess(
        *workers[1], beliefSnapshot(start + 300'000ULL, 500.0, 0.0, 20.0, 0.0));
    ASSERT_FALSE(unavailable.decision.activation_requested);
    auto heartbeat = matching_trigger;
    heartbeat.activation_just_started = false;
    heartbeat.activation_requested = !peer_ended;
    if (reordered_participation) heartbeat.local_activation_request_timestamp_us = 0;
    ++heartbeat.local_selection_epoch;  // Must not relabel the saved event.
    if (stale) heartbeat.proposal_timestamp_us = start - 1'000'000;
    if (mismatched_tuple) {
        heartbeat.selected_candidate_ids[1] =
            (heartbeat.selected_candidate_ids[1] + 1) % ce::kManeuverCandidateCount;
    }
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
    if (late_active) {
        // Reordered/duplicated active status must not resurrect the ended episode.
        ASSERT_TRUE(workers[1]->pushRemoteDecision(0, matching_trigger));
        ASSERT_TRUE(workers[1]->pushRemoteDecision(0, matching_trigger));
    }
    const auto fresh_peer = pushGraphBeliefAndProcess(
        *workers[0], beliefSnapshot(start + 350'000ULL, 7.0, 0.0, 20.0, 0.0));
    for (std::size_t i = 0; i < fresh_peer.intent_packet_count; ++i) {
        ASSERT_TRUE(workers[1]->pushRemoteIntent(0, fresh_peer.intent_packets[i]));
    }
    const auto coordinated = pushGraphBeliefAndProcess(
        *workers[1],
        beliefSnapshot(start + 350'000ULL, 500.0, 0.0, 20.0, 0.0));
    const bool should_activate = !peer_ended && !stale && !mismatched_tuple && !peer_only;
    EXPECT_EQ(coordinated.decision.activation_requested, should_activate);
    EXPECT_EQ(coordinated.decision.activation_just_started, should_activate);
    EXPECT_EQ(
        coordinated.decision.ownship_candidate_id,
        commits[1].decision.selected_candidate_ids[1]);

    if (peer_only) {
        EXPECT_GT(coordinated.decision.ad_m, 0.0);
        EXPECT_EQ(coordinated.decision.local_activation_request_timestamp_us, 0U);
    }
    if (peer_ended || peer_only) {
        return;
    }

    auto active = coordinated;
    if (stale || mismatched_tuple) {
        // A temporarily ineligible status must not consume the episode. A
        // fresh matching heartbeat, without a new start edge, can still join.
        heartbeat = matching_trigger;
        heartbeat.activation_just_started = false;
        heartbeat.proposal_timestamp_us = start + 400'000;
        ++heartbeat.local_selection_epoch;
        ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
        active = pushGraphBeliefAndProcess(
            *workers[1], beliefSnapshot(start + 400'000, 500.0, 0.0, 20.0, 0.0));
        ASSERT_TRUE(active.decision.activation_requested);
        EXPECT_TRUE(active.decision.activation_just_started);
    }

    // Repeated active status is idempotent: it cannot restart the local
    // episode or replace the already latched command.
    ASSERT_TRUE(active.decision.activation_requested);
    heartbeat.proposal_timestamp_us = start + 450'000;
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
    const auto repeated = pushGraphBeliefAndProcess(
        *workers[1], beliefSnapshot(start + 450'000, 500.0, 0.0, 20.0, 0.0));
    EXPECT_TRUE(repeated.decision.activation_requested);
    EXPECT_FALSE(repeated.decision.activation_just_started);
    EXPECT_EQ(repeated.decision.activation_timestamp_us,
              active.decision.activation_timestamp_us);
    EXPECT_EQ(repeated.decision.ownship_candidate_id,
              active.decision.ownship_candidate_id);
    EXPECT_DOUBLE_EQ(repeated.decision.ownship_input.a_lat_cmd,
                     active.decision.ownship_input.a_lat_cmd);
    EXPECT_DOUBLE_EQ(repeated.decision.ownship_input.V_cmd,
                     active.decision.ownship_input.V_cmd);
    EXPECT_DOUBLE_EQ(repeated.decision.ownship_input.h_dot_cmd,
                     active.decision.ownship_input.h_dot_cmd);
    // NaN h_cmd selects vertical-speed control, not an invalid command.
    if (std::isnan(active.decision.ownship_input.h_cmd)) {
        EXPECT_TRUE(std::isnan(repeated.decision.ownship_input.h_cmd));
    } else {
        EXPECT_DOUBLE_EQ(repeated.decision.ownship_input.h_cmd,
                         active.decision.ownship_input.h_cmd);
    }
}

TEST(ManeuverSelectionWorker,
    ComponentActivationEdgeSurvivesInvalidSampleAndHeartbeat)
{
    verifyDeferredComponentActivation(false);
}

TEST(ManeuverSelectionWorker,
    EndedPeerActivationIsNotReplayedAfterInputRecovery)
{
    verifyDeferredComponentActivation(true);
}

TEST(ManeuverSelectionWorker, ComponentActivationRecoveredFromHeartbeatWithoutEdge)
{
    verifyDeferredComponentActivation(false, true);
}

TEST(ManeuverSelectionWorker, PeerParticipationIsNotAnOriginatingRequest)
{
    verifyDeferredComponentActivation(false, false, false, false, false, true);
}

TEST(ManeuverSelectionWorker, ReorderedParticipationDoesNotEraseOriginatingRequest)
{
    verifyDeferredComponentActivation(false, false, false, false, false, false, true);
}

TEST(ManeuverSelectionWorker, EndedComponentEpisodeRejectsLateDuplicateStart)
{
    verifyDeferredComponentActivation(true, false, true);
}

TEST(ManeuverSelectionWorker, ComponentActivationRequiresFreshStatusAndAgreedTuple)
{
    verifyDeferredComponentActivation(false, false, false, true);
    verifyDeferredComponentActivation(false, false, false, false, true);
}

TEST(ManeuverSelectionWorker, ComponentEpisodeSurvivesLocalCommitInEitherArrivalOrder)
{
    for (bool start_after_commit : {false, true}) {
        SCOPED_TRACE(start_after_commit);
        constexpr std::uint64_t start = 19'000'000;
        std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, 2> workers;
        std::array<cs::ManeuverSelectionWorkerOutput, 2> outputs;
        for (int i = 0; i < 2; ++i) {
            auto p = params(i);
            p.exhaustive_test_mode = true;
            p.interaction_graph_params.enabled = true;
            p.interaction_graph_params.ad_screen_m = 1.0e6;
            workers[i] = std::make_unique<cs::ManeuverSelectionWorker>(p);
            workers[i]->setActivationEnabled(false);
            ASSERT_TRUE(workers[i]->pushPublishedSetpoint(publishedInput(start, 0.0)));
            ASSERT_TRUE(workers[i]->pushNominalSetpoint(nominalSnapshot(start)));
        }
        for (std::uint64_t offset = 0; offset <= 500'000; offset += 50'000) {
            for (int i = 0; i < 2; ++i)
                outputs[i] = pushGraphBeliefAndProcess(*workers[i], beliefSnapshot(
                    start + offset, 500.0 * i, 0.0, 20.0, 0.0));
            if (offset == 250'000) {
                auto commits = confirmTwoAircraftProposal(
                    *workers[0], *workers[1], outputs[0], outputs[1]);
                ASSERT_TRUE(commits[1].decision.coordination_qualified);
                for (int i = 0; i < 2; ++i)
                    ASSERT_TRUE(workers[i]->pushRemoteDecision(1-i, peerDecision(commits[1-i].decision)));
            }
            if (offset < 500'000)
                exchangePackets(*workers[0], *workers[1], outputs[0], outputs[1]);
        }
        ASSERT_TRUE(outputs[1].decision.proposal_valid);
        ASSERT_EQ(outputs[0].decision.proposed_candidate_ids, outputs[1].decision.proposed_candidate_ids);
        auto heartbeat = peerDecision(outputs[0].decision);
        const auto new_epoch = outputs[1].decision.proposal_epoch;
        ASSERT_LT(heartbeat.local_selection_epoch, new_epoch);
        ASSERT_EQ(heartbeat.selected_candidate_ids, outputs[1].decision.proposed_candidate_ids);
        workers[1]->setActivationEnabled(true);
        if (start_after_commit) {
            ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
            ASSERT_TRUE(workers[1]->processPendingForTest());
            const auto committed = workers[1]->tryPopOutput();
            ASSERT_TRUE(committed.has_value());
            ASSERT_EQ(committed->decision.local_selection_epoch, new_epoch);
            ASSERT_FALSE(committed->decision.activation_requested);
        }
        heartbeat.activation_requested = true;
        heartbeat.activation_just_started = false; // periodic status suffices
        heartbeat.activation_timestamp_us = start + 450'000;
        heartbeat.local_activation_request_timestamp_us = start + 450'000;
        ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
        ASSERT_TRUE(workers[1]->processPendingForTest());
        if (!start_after_commit) {
            const auto result = workers[1]->tryPopOutput();
            ASSERT_TRUE(result.has_value());
            EXPECT_EQ(result->decision.local_selection_epoch, new_epoch);
            EXPECT_TRUE(result->decision.activation_requested);
        } else {
            const auto result = pushGraphBeliefAndProcess(*workers[1],
                beliefSnapshot(start + 550'000, 500.0, 0.0, 20.0, 0.0));
            EXPECT_EQ(result.decision.local_selection_epoch, new_epoch);
            EXPECT_TRUE(result.decision.activation_requested);
        }
        heartbeat.post_release_evaluated = true;
        heartbeat.post_release_safe = true;
        heartbeat.post_release_evaluation_timestamp_us = start + 600'000;
        heartbeat.proposal_timestamp_us = start + 600'000;
        heartbeat.nominal_setpoint_timestamp_us = start + 600'000;
        heartbeat.nominal_setpoint_available = true;
        heartbeat.nominal_ground_speed_command_mps = 20.0;
        heartbeat.nominal_altitude_command_m = 100.0;
        heartbeat.nominal_lateral_acceleration_mps2 = 0.0;
        ASSERT_TRUE(workers[1]->pushNominalSetpoint(nominalSnapshot(start + 600'000)));
        ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
        const auto released = pushGraphBeliefAndProcess(*workers[1],
            beliefSnapshot(start + 600'000, 500.0, 0.0, 22.0, 0.0));
        ASSERT_FALSE(released.decision.activation_requested)
            << "CPA=" << released.decision.cpa_clear
            << " post=" << released.decision.post_release_evaluated
            << "," << released.decision.post_release_safe
            << " peer=" << released.decision.post_release_peer_confirmed
            << " AD=" << released.decision.ad_m;
        ASSERT_TRUE(released.decision.activation_just_ended);
        ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
        const auto duplicate = pushGraphBeliefAndProcess(*workers[1],
            beliefSnapshot(start + 650'000, 500.0, 0.0, 22.0, 0.0));
        EXPECT_FALSE(duplicate.decision.activation_requested);
        // A later execution start caused by joining another peer is not a
        // fresh risk request. This used to reopen the released local episode.
        heartbeat.activation_timestamp_us = start + 650'000;
        heartbeat.local_activation_request_timestamp_us = 0;
        heartbeat.proposal_timestamp_us = start + 650'000;
        ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
        const auto participation = pushGraphBeliefAndProcess(*workers[1],
            beliefSnapshot(start + 700'000, 500.0, 0.0, 22.0, 0.0));
        EXPECT_FALSE(participation.decision.activation_requested);
        EXPECT_GT(participation.decision.ad_m, 0.0);
        // A genuinely new episode is still eligible after release; this is
        // not a fixed cooldown or a permanent inhibit.
        // The participant can itself detect risk without restarting execution.
        heartbeat.local_activation_request_timestamp_us = start + 700'000;
        heartbeat.proposal_timestamp_us = start + 700'000;
        ASSERT_TRUE(workers[1]->pushRemoteDecision(0, heartbeat));
        const auto next_episode = pushGraphBeliefAndProcess(*workers[1],
            beliefSnapshot(start + 750'000, 500.0, 0.0, 22.0, 0.0));
        EXPECT_TRUE(next_episode.decision.activation_requested);
        EXPECT_TRUE(next_episode.decision.activation_just_started);
    }
}

static void verifyComponentProposalActivationRace(bool superior)
{
    constexpr std::uint64_t start = 18'000'000;
    std::array<std::unique_ptr<cs::ManeuverSelectionWorker>, 2> workers;
    std::array<cs::ManeuverSelectionWorkerOutput, 2> outputs;
    for (int i = 0; i < 2; ++i) {
        auto p = params(i);
        p.active_switching_enabled = true;
        p.active_switch_cost_margin = 0.01;
        p.active_switch_minimum_ad_margin_m = 1.0;
        if (!superior) {
            p.evaluator_params.desired_separation_distance_m = 1'000.0;
            p.active_switch_cost_margin = 1.0e9;
            p.active_switch_minimum_ad_margin_m = 1.0e9;
        }
        p.exhaustive_test_mode = true;
        p.interaction_graph_params.enabled = true;
        workers[i] = std::make_unique<cs::ManeuverSelectionWorker>(p);
        workers[i]->setActivationEnabled(false);
        ASSERT_TRUE(workers[i]->pushNominalSetpoint(nominalSnapshot(start)));
    }
    for (std::uint64_t offset = 0; offset <= 500'000; offset += 50'000) {
        for (int i = 0; i < 2; ++i) {
            // First epoch is head-on; the next common snapshot is offset.
            outputs[i] = pushGraphBeliefAndProcess(*workers[i], beliefSnapshot(
                start + offset, i ? 40.0 : -40.0,
                offset >= 300'000 ? (i ? -20.0 : 20.0) : 0.0,
                i ? -20.0 : 20.0, 0.0));
        }
        if (offset == 250'000) {
            auto commits = confirmTwoAircraftProposal(
                *workers[0], *workers[1], outputs[0], outputs[1]);
            ASSERT_TRUE(commits[0].decision.coordination_qualified);
            ASSERT_FALSE(commits[0].decision.activation_requested);
            for (int i = 0; i < 2; ++i)
                ASSERT_TRUE(workers[i]->pushRemoteDecision(1-i, peerDecision(commits[1-i].decision)));
        }
        if (offset < 500'000)
            exchangePackets(*workers[0], *workers[1], outputs[0], outputs[1]);
    }
    if (!superior) {
        EXPECT_TRUE(outputs[0].decision.switch_superiority_evaluated);
        EXPECT_FALSE(outputs[0].decision.switch_clearly_superior);
        EXPECT_FALSE(outputs[0].decision.proposal_valid);
        EXPECT_FALSE(outputs[0].decision.activation_requested);
        return;
    }
    ASSERT_TRUE(outputs[0].decision.proposal_valid)
        << "superiority=" << outputs[0].decision.switch_superiority_evaluated
        << " costs=" << outputs[0].decision.switch_current_cost << ","
        << outputs[0].decision.switch_proposed_cost
        << " AD=" << outputs[0].decision.switch_current_minimum_ad_m << ","
        << outputs[0].decision.switch_proposed_minimum_ad_m;
    ASSERT_EQ(outputs[0].decision.proposed_candidate_ids, outputs[1].decision.proposed_candidate_ids);
    ASSERT_NE(outputs[0].decision.proposed_candidate_ids[0], outputs[0].decision.selected_candidate_ids[0]);
    auto matching = peerDecision(outputs[1].decision);
    const auto proposed = outputs[0].decision.proposed_candidate_ids;
    const auto epoch = outputs[0].decision.proposal_epoch;
    workers[0]->setActivationEnabled(true);
    // Activate the old command before delivering the matching proposal.
    const auto active = pushGraphBeliefAndProcess(*workers[0],
        beliefSnapshot(start + 550'000, 38.0, -20.0, 20.0, 0.0));
    ASSERT_TRUE(active.decision.activation_requested);
    ASSERT_TRUE(workers[0]->pushRemoteDecision(1, matching));
    ASSERT_TRUE(workers[0]->processPendingForTest());
    const auto committed = workers[0]->tryPopOutput();
    ASSERT_TRUE(committed.has_value());
    EXPECT_EQ(committed->decision.local_selection_epoch, epoch);
    EXPECT_EQ(committed->decision.selected_candidate_ids, proposed);
    EXPECT_TRUE(committed->decision.proposal_consensus_confirmed);
    EXPECT_EQ(committed->decision.ownship_candidate_id, proposed[0]);
}

TEST(ManeuverSelectionWorker, ComponentProposalSurvivesActivationBeforeAgreement)
{
    verifyComponentProposalActivationRace(true);
}

TEST(ManeuverSelectionWorker, ComponentIncumbentPersistenceIsCheckedBeforeInactiveProposal)
{
    verifyComponentProposalActivationRace(false);
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
    worker_params.masd_diagnostics_enabled = true;
        worker_params.interaction_graph_params.ad_screen_m = -1.0e6;
        workers[aircraft] = std::make_unique<cs::ManeuverSelectionWorker>(
            worker_params);
        workers[aircraft]->setActivationEnabled(false);
        ASSERT_TRUE(workers[aircraft]->pushNominalSetpoint(
            nominalSnapshot(start)));
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
        outputs[aircraft] = pushGraphBeliefAndProcess(
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
    other_component_trigger.activation_requested = true;
    other_component_trigger.activation_just_started = true;
    other_component_trigger.activation_timestamp_us = start + 250'000;
    other_component_trigger.local_activation_request_timestamp_us = start + 250'000;
    ASSERT_TRUE(workers[1]->pushRemoteDecision(0, other_component_trigger));
    ASSERT_TRUE(workers[1]->processPendingForTest());
    const auto unaffected = pushGraphBeliefAndProcess(
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
    state.phi_setpoint = 0.0;
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, actual.input, .152, state, covariance));
    EXPECT_GT(state.phi, 0.0);
    EXPECT_GT(state.V, 20.0);
    expectPacketInitialState(result, state, covariance);
    ASSERT_EQ(result.intent_packet_count, 7U);
    EXPECT_LT(result.intent_packets.front().candidate_input[3], 0.0F);
    EXPECT_GT(result.intent_packets.back().candidate_input[3], 0.0F);
    for (std::size_t i = 0; i < result.intent_packet_count; ++i) {
        const auto & packet = result.intent_packets[i];
        EXPECT_TRUE(packet.source_execution_input_available);
        EXPECT_FLOAT_EQ(packet.source_execution_input[0], 22.0F);
        EXPECT_FLOAT_EQ(packet.source_execution_input[3], 3.0F);
        EXPECT_NE(packet.source_execution_input[3], packet.candidate_input[3]);
    }
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
    state.phi_setpoint = predictor.rollSetpointAfter(0.0, first.input, .010);
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, first.input, .070, state, covariance));
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
        predictor, second.input, .082, state, covariance));
    expectPacketInitialState(result, state, covariance);
    ASSERT_GT(result.intent_packet_count, 0U);
    for (std::size_t i = 0; i < result.intent_packet_count; ++i) {
        const auto & packet = result.intent_packets[i];
        EXPECT_TRUE(packet.source_execution_input_available);
        // At the source time the new command is already published, even though
        // past-state compensation above must use the preceding two commands.
        EXPECT_FLOAT_EQ(packet.source_execution_input[0], 20.0F);
        EXPECT_FLOAT_EQ(packet.source_execution_input[3], 11.0F);
    }
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
    state.phi_setpoint = predictor.rollSetpointAfter(0.0, actual.input, .100);
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

TEST(FusionInputHistory, UndelayedBeliefCarriesContinuingSetpointRamp)
{
    auto p=params(); p.exhaustive_test_mode=true;
    cs::ManeuverSelectionWorker worker(p);
    const auto first=publishedInput(1'000'000,11.0);
    ASSERT_TRUE(worker.pushPublishedSetpoint(first));
    const auto output=pushBeliefAndProcess(worker,beliefSnapshot(1'100'000,0,0,20,0));
    ASSERT_EQ(output.intent_packet_count,7U);
    const ce::TrajectoryPredict predictor(p.predictor_params);
    const double expected=predictor.rollSetpointAfter(0.0,first.input,.1);
    for (std::size_t i=0;i<output.intent_packet_count;++i) {
        EXPECT_NEAR(output.intent_packets[i].initial_roll_setpoint_rad,expected,1e-7);
        EXPECT_FLOAT_EQ(output.intent_packets[i].initial_state[6],0.0F);
    }
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

TEST(FusionInputHistory, PeerBurstCannotDropLocalHistoryOrBelief)
{
    // Exercise every ownship ID: the local partition must not alias a peer.
    for (int ownship = 0; ownship < 5; ++ownship) {
        auto p = params(ownship, 5);
        p.exhaustive_test_mode = true;
        auto worker = std::make_unique<cs::ManeuverSelectionWorker>(p);
        ce::TrajectoryIntentPacket invalid_packet{};
        for (int peer = 0; peer < 5; ++peer) {
            if (peer == ownship) continue;
            for (std::size_t i = 0; i < cs::kSelectionWorkerInputCapacity; ++i)
                ASSERT_TRUE(worker->pushRemoteIntent(peer, invalid_packet));
            EXPECT_FALSE(worker->pushRemoteIntent(peer, invalid_packet));
        }
        auto belief = beliefSnapshot(1'152'000, 0, 0, 20, 0);
        belief.timestamp_sample_us = 1'000'000;
        const auto actual = publishedInput(1'000'000, 3.0);
        ASSERT_TRUE(worker->pushPublishedSetpoint(actual));
        ASSERT_TRUE(worker->pushOwnshipBelief(belief));
        // Paused remote reconstruction must not delay local input processing.
        ASSERT_TRUE(worker->processPendingForTest(0, true, false));
        const auto output = worker->tryPopOutput();
        ASSERT_TRUE(output);
        ASSERT_EQ(output->intent_packet_count, 7U);
        ce::TrajectoryUncertainty uncertainty(p.uncertainty_params);
        ce::TrajectoryPredict predictor(p.predictor_params);
        ce::PredictState state;
        ce::PredictStateCovariance covariance;
        ASSERT_TRUE(uncertainty.initializeFromEstimatorBelief(belief.belief, state, covariance));
        state.phi_setpoint = 0.0;
        ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(
            predictor, actual.input, .152, state, covariance));
        expectPacketInitialState(*output, state, covariance);
        EXPECT_EQ(worker->droppedInputCount(), 4U);
    }
}

TEST(StoppedObservations, FullBufferDropsWithoutOverwriteOrDynamicRecords)
{
    collision_avoidance::common::StoppedRecordBuffer<std::uint64_t, 2> records;
    records.append(10); records.append(20); records.append(30);
    EXPECT_EQ(records.size, 2U);
    EXPECT_EQ(records.dropped, 1U);
    EXPECT_EQ(records.records[0], 10U);
    EXPECT_EQ(records.records[1], 20U);
}

TEST(StoppedObservations, GraphControlAndCommitAreIndependentOfRecording)
{
    const auto replay = [](bool record) {
        auto p0 = params(0), p1 = params(1);
        for (auto * p : {&p0, &p1}) {
            p->exhaustive_test_mode = true;
            p->interaction_graph_params.enabled = true;
            p->masd_diagnostics_enabled = record;
        }
        cs::ManeuverSelectionWorker first(p0), second(p1);
        constexpr std::uint64_t start = 30'000'000;
        EXPECT_TRUE(first.pushNominalSetpoint(nominalSnapshot(start)));
        EXPECT_TRUE(second.pushNominalSetpoint(nominalSnapshot(start)));
        auto a = pushGraphBeliefAndProcess(first, beliefSnapshot(start, -45, 0, 20, 0));
        auto b = pushGraphBeliefAndProcess(second, beliefSnapshot(start, 45, 0, -20, 0));
        exchangePackets(first, second, a, b);
        a = pushGraphBeliefAndProcess(first, beliefSnapshot(start+250'000, -40, 0, 20, 0));
        b = pushGraphBeliefAndProcess(second, beliefSnapshot(start+250'000, 40, 0, -20, 0));
        EXPECT_TRUE(a.decision.proposed_component_graph);
        EXPECT_TRUE(a.decision.proposal_valid);
        EXPECT_EQ(first.stoppedGraphDiagnostics() != nullptr, record);
        EXPECT_EQ(first.stoppedBudgetTraces() != nullptr, record);
        return confirmTwoAircraftProposal(first, second, a, b);
    };
    const auto off = replay(false), on = replay(true);
    for (std::size_t i=0; i<2; ++i) {
        EXPECT_EQ(off[i].decision.selected_candidate_ids, on[i].decision.selected_candidate_ids);
        EXPECT_EQ(off[i].decision.coordination_qualified, on[i].decision.coordination_qualified);
        EXPECT_EQ(off[i].decision.command_execution_requested, on[i].decision.command_execution_requested);
        EXPECT_EQ(off[i].decision.activation_requested, on[i].decision.activation_requested);
        EXPECT_DOUBLE_EQ(off[i].decision.ad_m, on[i].decision.ad_m);
    }
}
