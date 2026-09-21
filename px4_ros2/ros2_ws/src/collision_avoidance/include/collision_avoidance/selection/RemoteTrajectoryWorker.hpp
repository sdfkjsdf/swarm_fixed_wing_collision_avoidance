#pragma once

#include <atomic>
#include <memory>
#include <ostream>
#include <thread>

#include <collision_avoidance/common/OrderedSpscInbox.hpp>
#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kRemoteTrajectoryInputCapacity = 64; // per peer
inline constexpr std::size_t kRemoteTrajectoryResultCapacity = 16;

// A result is published only after all distinct candidates of one source
// timestamp/epoch have passed reconstruction. Immutable until consumer release.
struct RemoteTrajectoryCandidateSet
{
    int vehicle_id{-1};
    std::uint64_t selection_epoch{0}, source_timestamp_us{0};
    estimation::CandidateSetKind candidate_set_kind{estimation::CandidateSetKind::LegacyRoll};
    std::size_t expected_count{0}, count{0};
    ExhaustiveCandidateIntentSet candidates{};
};

// ROS executor -> reconstruction thread -> selection state owner. Neither
// channel has multiple producers/consumers. No control or live owner references.
class RemoteTrajectoryWorker
{
public:
    RemoteTrajectoryWorker(int vehicle_id, int aircraft_count, std::size_t legacy_count,
        const estimation::PredictParams & predictor,
        const estimation::UncertaintyParams & uncertainty, bool measure = false);
    ~RemoteTrajectoryWorker();
    void start();
    void stop();
    bool push(int peer, const estimation::TrajectoryIntentPacket & packet) noexcept;
    const RemoteTrajectoryCandidateSet * readyResult() const noexcept;
    std::size_t readyCount() const noexcept;
    void releaseResult() noexcept;
    // Deterministic tests use the exact production kernel, without a thread.
    bool processAvailableForTest();
    void writeStoppedStatistics(std::ostream & out) const;

private:
    struct Request { int vehicle_id{-1}; estimation::TrajectoryIntentPacket packet{}; };
    struct Staging {
        RemoteTrajectoryCandidateSet set{};
        std::array<bool, kExhaustiveCandidatesPerAircraft> occupied{};
    };
    struct Storage {
        common::OrderedSpscInbox<Request, kMaximumSelectionAircraft,
            kRemoteTrajectoryInputCapacity> requests;
        common::SpscQueue<RemoteTrajectoryCandidateSet, kRemoteTrajectoryResultCapacity> results;
        std::array<Staging, kMaximumSelectionAircraft> staging{};
        std::array<Request, 1> request{};
        RemoteTrajectoryCandidateSet pending{};
    };
    bool processOne();
    bool reconstruct(const Request & request);
    void loop();

    int m_vehicle_id, m_aircraft_count;
    std::size_t m_legacy_count;
    bool m_measure, m_has_pending{false}; // reconstruction thread only
    estimation::TrajectoryIntentReceiver m_receiver;
    std::unique_ptr<Storage> m_storage;
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    // Reconstruction-thread counters; read only after join. No runtime logging.
    std::uint64_t m_processed{0}, m_rejected{0}, m_completed{0};
    std::uint64_t m_total_ns{0}, m_max_ns{0};
};

} // namespace collision_avoidance::selection
