#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>

namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

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
    peer.ownship_candidate_id = decision.ownship_candidate_id;
    peer.proposal_timestamp_us = decision.proposal_timestamp_us;
    peer.proposal_epoch = decision.proposal_epoch;
    peer.proposed_candidate_ids = decision.proposed_candidate_ids;
    peer.proposal_valid = decision.proposal_valid;
    peer.proposal_consensus_confirmed =
        decision.proposal_consensus_confirmed;
    peer.coordination_qualified = decision.coordination_qualified;
    peer.activation_requested = decision.activation_requested;
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

TEST(ManeuverSelectionWorker, IndependentlySelectsAndRequestsActivation)
{
    const auto worker_params = params();
    cs::ManeuverSelectionWorker first(worker_params);
    cs::ManeuverSelectionWorker second(params(1));
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

TEST(ManeuverSelectionWorker, DoesNotActivateWhileCurrentPlanIsSafe)
{
    cs::ManeuverSelectionWorker first(params());
    cs::ManeuverSelectionWorker second(params(1));
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
    cs::ManeuverSelectionWorker first(params());
    cs::ManeuverSelectionWorker second(params(1));
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

TEST(ManeuverSelectionWorker, WarmsSelectionButDoesNotActivateBeforeGateOpens)
{
    cs::ManeuverSelectionWorker first(params());
    cs::ManeuverSelectionWorker second(params(1));
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
    RejectsMismatchedProposalWithoutRelabelingTheCommittedEpoch)
{
    cs::ManeuverSelectionWorker first(params());
    cs::ManeuverSelectionWorker second(params(1));
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
    mismatched_peer.proposed_candidate_ids[0] = static_cast<std::uint8_t>(
        (first_output.decision.proposed_candidate_ids[0] + 1U)
        % ce::kManeuverCandidateCount);
    ASSERT_TRUE(first.pushRemoteDecision(1, mismatched_peer));
    ASSERT_TRUE(first.processPendingForTest());
    const auto rejected = first.tryPopOutput();
    ASSERT_TRUE(rejected.has_value());
    ASSERT_TRUE(rejected->has_decision);
    EXPECT_FALSE(rejected->decision.proposal_consensus_confirmed);
    EXPECT_TRUE(rejected->decision.coordination_qualified);
    EXPECT_TRUE(rejected->decision.previous_best_retained);
    EXPECT_FALSE(rejected->decision.new_best_accepted);
    EXPECT_EQ(rejected->decision.local_selection_epoch, committed_epoch);
    EXPECT_EQ(rejected->decision.selection_timestamp_us, committed_timestamp);
    EXPECT_EQ(rejected->decision.selected_candidate_ids, committed_tuple);
}

TEST(ManeuverSelectionWorker,
    ActiveAircraftLatchesOnlyItsOwnCandidateAcrossAConfirmedNewEpoch)
{
    cs::ManeuverSelectionWorker first(params());
    cs::ManeuverSelectionWorker second(params(1));
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
    ASSERT_TRUE(first_output.decision.proposal_valid);
    EXPECT_EQ(
        first_output.decision.proposed_candidate_ids[0], latched_ownship_id);
    EXPECT_EQ(first_output.decision.ownship_candidate_id, latched_ownship_id);
    EXPECT_EQ(
        first_output.decision.local_selection_epoch, first_committed_epoch);
    EXPECT_GT(
        first_output.decision.proposal_epoch,
        first_output.decision.local_selection_epoch);
    EXPECT_FALSE(first_output.decision.proposal_consensus_confirmed);

    auto matching_peer = peerDecision(first_output.decision);
    matching_peer.vehicle_id = 1;
    matching_peer.ownship_candidate_id =
        matching_peer.proposed_candidate_ids[1];
    matching_peer.activation_requested = false;
    ASSERT_TRUE(first.pushRemoteDecision(1, matching_peer));
    ASSERT_TRUE(first.processPendingForTest());
    const auto next_commit = first.tryPopOutput();
    ASSERT_TRUE(next_commit.has_value());
    ASSERT_TRUE(next_commit->has_decision);
    EXPECT_TRUE(next_commit->decision.proposal_consensus_confirmed);
    EXPECT_TRUE(next_commit->decision.coordination_qualified);
    EXPECT_EQ(
        next_commit->decision.local_selection_epoch,
        first_output.decision.proposal_epoch);
    EXPECT_EQ(next_commit->decision.ownship_candidate_id, latched_ownship_id);
    EXPECT_EQ(
        next_commit->decision.selected_candidate_ids[0], latched_ownship_id);
    EXPECT_EQ(
        next_commit->decision.selected_candidate_ids[1],
        first_output.decision.proposed_candidate_ids[1]);
}

TEST(ManeuverSelectionWorker, StartsStopsAndKeepsInstancesIndependent)
{
    cs::ManeuverSelectionWorker first(params());
    cs::ManeuverSelectionWorker second(params());
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
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        const auto & decision = outputs[aircraft].decision;
        ASSERT_TRUE(outputs[aircraft].has_decision);
        EXPECT_TRUE(decision.coordination_qualified);
        EXPECT_EQ(decision.aircraft_count, aircraft_count);
        EXPECT_EQ(decision.evaluated_combination_count, 243U);
        EXPECT_EQ(decision.selected_candidate_ids, expected_tuple);
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
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        const auto & decision = outputs[aircraft].decision;
        ASSERT_TRUE(outputs[aircraft].has_decision);
        EXPECT_TRUE(decision.coordination_qualified);
        EXPECT_EQ(decision.evaluated_combination_count, 16'807U);
        EXPECT_EQ(decision.selected_candidate_ids, expected_tuple);
    }
}
