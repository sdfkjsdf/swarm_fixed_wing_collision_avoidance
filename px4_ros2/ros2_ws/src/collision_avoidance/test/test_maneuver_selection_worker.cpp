#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

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

cs::ManeuverSelectionWorkerParams params()
{
    cs::ManeuverSelectionWorkerParams value;
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
        ASSERT_TRUE(second.pushRemoteIntent(first_output.intent_packets[index]));
    }
    for (std::size_t index = 0;
         index < second_output.intent_packet_count; ++index) {
        ASSERT_TRUE(first.pushRemoteIntent(second_output.intent_packets[index]));
    }
    EXPECT_TRUE(first.processPendingForTest());
    EXPECT_TRUE(second.processPendingForTest());
}

}  // namespace

TEST(ManeuverSelectionWorker, ImplementsTwentyAndFourHertzCadenceWithoutSleeps)
{
    cs::ManeuverSelectionWorker worker(params());
    constexpr std::uint64_t start = 1'000'000ULL;

    auto output = pushBeliefAndProcess(
        worker, beliefSnapshot(start, 0.0, 0.0, 20.0, 0.0));
    ASSERT_EQ(output.intent_packet_count, 3U);
    EXPECT_EQ(output.selection_epoch, 1U);
    EXPECT_FALSE(output.has_decision);
    for (const auto & packet : output.intent_packets) {
        EXPECT_EQ(packet.selection_epoch, 1U);
        EXPECT_EQ(packet.source_timestamp_us, start);
    }

    EXPECT_TRUE(worker.pushOwnshipBelief(
        beliefSnapshot(start + 49'999, 1.0, 0.0, 20.0, 0.0)));
    EXPECT_TRUE(worker.processPendingForTest());
    EXPECT_FALSE(worker.tryPopOutput().has_value());

    output = pushBeliefAndProcess(
        worker,
        beliefSnapshot(start + 50'000, 1.0, 0.0, 20.0, 0.0));
    EXPECT_EQ(output.intent_packet_count, 3U);
    EXPECT_EQ(output.selection_epoch, 1U);

    for (std::uint64_t offset : {100'000ULL, 150'000ULL, 200'000ULL}) {
        output = pushBeliefAndProcess(
            worker,
            beliefSnapshot(start + offset, 20.0e-6 * offset, 0.0, 20.0, 0.0));
        EXPECT_EQ(output.selection_epoch, 1U);
        EXPECT_FALSE(output.has_decision);
    }

    output = pushBeliefAndProcess(
        worker,
        beliefSnapshot(start + 250'000, 5.0, 0.0, 20.0, 0.0));
    EXPECT_TRUE(output.has_decision);
    EXPECT_FALSE(output.decision.coordination_qualified);
    EXPECT_TRUE(output.decision.previous_best_retained);
    EXPECT_EQ(output.selection_epoch, 2U);
    ASSERT_EQ(output.intent_packet_count, 3U);
    for (const auto & packet : output.intent_packets) {
        EXPECT_EQ(packet.selection_epoch, 2U);
    }
}

TEST(ManeuverSelectionWorker, DoesNotMixAdjacentIncompleteRemoteEpochs)
{
    const auto worker_params = params();
    cs::ManeuverSelectionWorker ownship(worker_params);
    cs::ManeuverSelectionWorker remote(worker_params);
    constexpr std::uint64_t start = 2'000'000ULL;

    const auto own_output = pushBeliefAndProcess(
        ownship, beliefSnapshot(start, -45.0, 0.0, 20.0, 0.0));
    auto remote_output = pushBeliefAndProcess(
        remote, beliefSnapshot(start, 45.0, 0.0, -20.0, 0.0));

    ASSERT_TRUE(ownship.pushRemoteIntent(remote_output.intent_packets[0]));
    ASSERT_TRUE(ownship.pushRemoteIntent(remote_output.intent_packets[1]));
    auto future_epoch_packet = remote_output.intent_packets[2];
    future_epoch_packet.selection_epoch = 2;
    ASSERT_TRUE(ownship.pushRemoteIntent(future_epoch_packet));
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
    cs::ManeuverSelectionWorker remote(worker_params);
    constexpr std::uint64_t start = 2'500'000ULL;

    static_cast<void>(pushBeliefAndProcess(
        ownship, beliefSnapshot(start, -45.0, 0.0, 20.0, 0.0)));
    const auto remote_output = pushBeliefAndProcess(
        remote, beliefSnapshot(start, 45.0, 0.0, -20.0, 0.0));
    for (const auto & packet : remote_output.intent_packets) {
        ASSERT_TRUE(ownship.pushRemoteIntent(packet));
    }
    EXPECT_TRUE(ownship.processPendingForTest());

    auto partial_next_refresh = remote_output.intent_packets[0];
    partial_next_refresh.source_timestamp_us = start + 50'000;
    ASSERT_TRUE(ownship.pushRemoteIntent(partial_next_refresh));
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
    EXPECT_TRUE(decision_output.decision.coordination_qualified);
    EXPECT_EQ(decision_output.decision.remote_selection_epoch, 1U);
}

TEST(ManeuverSelectionWorker, IndependentlySelectsAndRequestsActivation)
{
    const auto worker_params = params();
    cs::ManeuverSelectionWorker first(worker_params);
    cs::ManeuverSelectionWorker second(worker_params);
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
    EXPECT_EQ(first_output.decision.local_selection_epoch, 1U);
    EXPECT_EQ(second_output.decision.local_selection_epoch, 1U);
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
