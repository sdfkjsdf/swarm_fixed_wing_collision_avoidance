#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <sstream>
#include <thread>

#include <collision_avoidance/selection/RemoteTrajectoryWorker.hpp>

namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

namespace
{
using Packets = std::array<ce::TrajectoryIntentPacket, 7>;
Packets packets(std::uint64_t stamp = 1'000'000, std::uint64_t epoch = 4)
{
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    ce::TrajectoryIntentSender sender(predictor, ce::makeLevelTurnCandidateTable(20, 100));
    ce::PredictStateCovariance covariance{};
    for (std::size_t i = 0; i < 7; ++i) covariance[i * 7 + i] = .04;
    ce::PredictState state{10, -5, 100, 20, .25, 0, .1};
    state.phi_setpoint = .3;
    Packets result;
    for (std::size_t i = 0; i < result.size(); ++i) {
        EXPECT_TRUE(sender.buildForSelectedCandidate(stamp, i, state, covariance, result[i], epoch));
        result[i].candidate_set_size = 7;
    }
    return result;
}

void pushSet(cs::RemoteTrajectoryWorker & worker, const Packets & set, int peer = 1)
{
    for (const auto & packet : set) ASSERT_TRUE(worker.push(peer, packet));
    ASSERT_TRUE(worker.processAvailableForTest());
}
} // namespace

TEST(RemoteTrajectoryWorker, CompleteSortedSetMatchesExistingReconstructorExactly)
{
    cs::RemoteTrajectoryWorker worker(0, 2, 7, {}, {});
    const auto input = packets();
    for (int id = 6; id >= 1; --id) ASSERT_TRUE(worker.push(1, input[id]));
    ASSERT_TRUE(worker.processAvailableForTest());
    EXPECT_EQ(worker.readyResult(), nullptr);
    ASSERT_TRUE(worker.push(1, input[0]));
    ASSERT_TRUE(worker.processAvailableForTest());
    const auto * result = worker.readyResult();
    ASSERT_NE(result, nullptr);
    EXPECT_EQ(result->vehicle_id, 1);
    ASSERT_EQ(result->count, 7U);
    ce::TrajectoryIntentReceiver receiver(ce::TrajectoryPredict(ce::PredictParams{}));
    for (std::size_t id = 0; id < 7; ++id) {
        ce::ReceivedTrajectoryIntent reference;
        ASSERT_TRUE(receiver.receive(input[id], reference));
        const auto & actual = result->candidates[id];
        EXPECT_EQ(actual.candidate_id, id);
        EXPECT_EQ(actual.candidate_input_revision, reference.candidate_input_revision);
        for (std::size_t k = 0; k < ce::kTrajectoryPointCount; ++k) {
            EXPECT_DOUBLE_EQ(actual.reconstructed_mean[k].p_n, reference.reconstructed_mean[k].p_n);
            EXPECT_DOUBLE_EQ(actual.reconstructed_mean[k].p_e, reference.reconstructed_mean[k].p_e);
            EXPECT_DOUBLE_EQ(actual.reconstructed_mean[k].h, reference.reconstructed_mean[k].h);
            EXPECT_DOUBLE_EQ(actual.reconstructed_mean[k].V, reference.reconstructed_mean[k].V);
            EXPECT_DOUBLE_EQ(actual.reconstructed_mean[k].psi, reference.reconstructed_mean[k].psi);
            EXPECT_DOUBLE_EQ(actual.reconstructed_mean[k].h_dot, reference.reconstructed_mean[k].h_dot);
            EXPECT_DOUBLE_EQ(actual.reconstructed_mean[k].phi, reference.reconstructed_mean[k].phi);
            EXPECT_DOUBLE_EQ(actual.reconstructed_mean[k].phi_setpoint, reference.reconstructed_mean[k].phi_setpoint);
            EXPECT_EQ(actual.cone[k].state_covariance, reference.cone[k].state_covariance);
            EXPECT_EQ(actual.cone[k].position_covariance_ned, reference.cone[k].position_covariance_ned);
        }
    }
    worker.releaseResult();
    EXPECT_EQ(worker.readyResult(), nullptr);
}

TEST(RemoteTrajectoryWorker, NeverMixesEpochsOrTimestampsAndRetainsCompleteResult)
{
    cs::RemoteTrajectoryWorker worker(0, 2, 7, {}, {});
    const auto old = packets(), newer = packets(1'050'000, 4), next_epoch = packets(1'100'000, 5);
    pushSet(worker, old);
    for (int id = 0; id < 3; ++id) ASSERT_TRUE(worker.push(1, newer[id]));
    for (int id = 3; id < 7; ++id) ASSERT_TRUE(worker.push(1, next_epoch[id]));
    ASSERT_TRUE(worker.processAvailableForTest());
    ASSERT_EQ(worker.readyCount(), 1U);
    EXPECT_EQ(worker.readyResult()->source_timestamp_us, 1'000'000U);
    worker.releaseResult();
    for (int id = 0; id < 3; ++id) ASSERT_TRUE(worker.push(1, newer[id])); // obsolete
    ASSERT_TRUE(worker.processAvailableForTest());
    EXPECT_EQ(worker.readyCount(), 0U);
    for (int id = 0; id < 3; ++id) ASSERT_TRUE(worker.push(1, next_epoch[id]));
    ASSERT_TRUE(worker.processAvailableForTest());
    ASSERT_EQ(worker.readyCount(), 1U);
    EXPECT_EQ(worker.readyResult()->selection_epoch, 5U);
    for (const auto & candidate : worker.readyResult()->candidates) {
        EXPECT_EQ(candidate.source_timestamp_us, 1'100'000U);
        EXPECT_EQ(candidate.selection_epoch, 5U);
    }
}

TEST(RemoteTrajectoryWorker, InvalidNewPacketDoesNotEraseStagingAndDuplicatesDoNotCount)
{
    cs::RemoteTrajectoryWorker worker(0, 2, 7, {}, {});
    const auto input = packets();
    for (int i = 0; i < 6; ++i) ASSERT_TRUE(worker.push(1, input[i]));
    ASSERT_TRUE(worker.push(1, input[0]));
    auto invalid = input[6];
    invalid.source_timestamp_us += 50'000;
    invalid.initial_state[0] = std::numeric_limits<float>::quiet_NaN();
    ASSERT_TRUE(worker.push(1, invalid));
    ASSERT_TRUE(worker.processAvailableForTest());
    EXPECT_EQ(worker.readyCount(), 0U);
    ASSERT_TRUE(worker.push(1, input[6]));
    ASSERT_TRUE(worker.processAvailableForTest());
    ASSERT_EQ(worker.readyCount(), 1U);
    worker.releaseResult();
    pushSet(worker, input); // duplicates after completion must not republish
    EXPECT_EQ(worker.readyCount(), 0U);
}

TEST(RemoteTrajectoryWorker, FullResultQueueRetainsPendingSetWithoutOverwriting)
{
    cs::RemoteTrajectoryWorker worker(0, 2, 7, {}, {});
    auto input = packets();
    for (std::size_t set = 0; set < cs::kRemoteTrajectoryResultCapacity + 2; ++set) {
        for (auto & packet : input) packet.source_timestamp_us = 1'000'000 + set * 50'000;
        for (const auto & packet : input) ASSERT_TRUE(worker.push(1, packet));
        worker.processAvailableForTest();
    }
    EXPECT_EQ(worker.readyCount(), cs::kRemoteTrajectoryResultCapacity);
    EXPECT_FALSE(worker.processAvailableForTest()); // pending, no spin or drop
    for (std::size_t set = 0; set < cs::kRemoteTrajectoryResultCapacity + 2; ++set) {
        ASSERT_NE(worker.readyResult(), nullptr);
        EXPECT_EQ(worker.readyResult()->source_timestamp_us, 1'000'000 + set * 50'000);
        worker.releaseResult();
        worker.processAvailableForTest();
    }
    EXPECT_EQ(worker.readyCount(), 0U);
}

TEST(RemoteTrajectoryWorker, LegacyThreeAndV4VariableCountRemainSupported)
{
    for (const auto kind : {ce::CandidateSetKind::LegacyRoll, ce::CandidateSetKind::V4SafeControl}) {
        cs::RemoteTrajectoryWorker worker(0, 2, 3, {}, {});
        auto input = packets();
        const auto count = kind == ce::CandidateSetKind::LegacyRoll ? 3 : 2;
        for (int id = 0; id < count; ++id) {
            input[id].candidate_set_size = count;
            input[id].candidate_set_kind = kind;
            ASSERT_TRUE(worker.push(1, input[id]));
        }
        ASSERT_TRUE(worker.processAvailableForTest());
        ASSERT_NE(worker.readyResult(), nullptr);
        EXPECT_EQ(worker.readyResult()->count, static_cast<std::size_t>(count));
        EXPECT_EQ(worker.readyResult()->candidate_set_kind, kind);
    }
}

TEST(RemoteTrajectoryWorker, PeerInputReservesAndInvalidPeerChecks)
{
    cs::RemoteTrajectoryWorker worker(0, 5, 7, {}, {});
    ce::TrajectoryIntentPacket invalid;
    EXPECT_FALSE(worker.push(0, invalid));
    EXPECT_FALSE(worker.push(5, invalid));
    EXPECT_FALSE(worker.push(-1, invalid));
    for (std::size_t n = 0; n < cs::kRemoteTrajectoryInputCapacity; ++n)
        ASSERT_TRUE(worker.push(1, invalid));
    EXPECT_FALSE(worker.push(1, invalid));
    for (int peer = 2; peer < 5; ++peer) EXPECT_TRUE(worker.push(peer, invalid));
}

TEST(RemoteTrajectoryWorker, ThreadedFifoBackpressureAndRestart)
{
    cs::RemoteTrajectoryWorker worker(0, 2, 7, {}, {}, true);
    const auto input = packets();
    for (int restart = 0; restart < 2; ++restart) {
        worker.start();
        EXPECT_FALSE(worker.processAvailableForTest());
        std::atomic<bool> abort{false};
        std::thread producer([&] {
            for (int set = 0; set < 40 && !abort.load(); ++set) {
                for (auto packet : input) {
                    packet.source_timestamp_us += (restart * 40 + set) * 50'000;
                    while (!worker.push(1, packet) && !abort.load()) std::this_thread::yield();
                    if (abort.load()) return;
                }
            }
        });
        int received = 0;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (received < 40 && std::chrono::steady_clock::now() < deadline) {
            if (const auto * result = worker.readyResult()) {
                EXPECT_EQ(result->source_timestamp_us, 1'000'000U + (restart * 40 + received) * 50'000U);
                EXPECT_EQ(result->count, 7U);
                worker.releaseResult();
                ++received;
            } else std::this_thread::yield();
        }
        abort.store(true);
        producer.join();
        worker.stop();
        EXPECT_EQ(received, 40);
    }
    std::ostringstream stopped;
    worker.writeStoppedStatistics(stopped);
    EXPECT_NE(stopped.str().find("[stop-remote-worker],0,560,0,80,"), std::string::npos);
}
