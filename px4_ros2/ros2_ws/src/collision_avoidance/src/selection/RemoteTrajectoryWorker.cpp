#include <collision_avoidance/selection/RemoteTrajectoryWorker.hpp>
#include <collision_avoidance/selection/SafeControlCandidateAdapter.hpp>

#include <algorithm>
#include <cassert>
#include <chrono>

namespace collision_avoidance::selection
{
RemoteTrajectoryWorker::RemoteTrajectoryWorker(int vehicle_id, int aircraft_count,
    std::size_t legacy_count, const estimation::PredictParams & predictor,
    const estimation::UncertaintyParams & uncertainty, bool measure)
: m_vehicle_id(vehicle_id), m_aircraft_count(aircraft_count), m_legacy_count(legacy_count),
  m_measure(measure), m_receiver(estimation::TrajectoryPredict(predictor), uncertainty),
  m_storage(std::make_unique<Storage>())
{
}

RemoteTrajectoryWorker::~RemoteTrajectoryWorker() { stop(); }

void RemoteTrajectoryWorker::start()
{
    if (m_thread.joinable()) return;
    m_running.store(true, std::memory_order_release);
    try { m_thread = std::thread(&RemoteTrajectoryWorker::loop, this); }
    catch (...) { m_running.store(false, std::memory_order_release); throw; }
}

void RemoteTrajectoryWorker::stop()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) m_thread.join();
}

bool RemoteTrajectoryWorker::push(int peer,
    const estimation::TrajectoryIntentPacket & packet) noexcept
{
    if (peer < 0 || peer >= m_aircraft_count || peer == m_vehicle_id
        || static_cast<std::size_t>(peer) >= kMaximumSelectionAircraft) return false;
    return m_storage->requests.try_push(static_cast<std::size_t>(peer), Request{peer, packet});
}

const RemoteTrajectoryCandidateSet * RemoteTrajectoryWorker::readyResult() const noexcept
{
    return m_storage->results.peekForConsumer();
}

std::size_t RemoteTrajectoryWorker::readyCount() const noexcept
{
    return m_storage->results.sizeForConsumer();
}

void RemoteTrajectoryWorker::releaseResult() noexcept
{
    const auto result = m_storage->results.try_pop();
    assert(result.has_value());
    (void)result;
}

bool RemoteTrajectoryWorker::processAvailableForTest()
{
    if (m_thread.joinable()) return false;
    bool processed = false;
    for (std::size_t n = 0; n < kRemoteTrajectoryInputCapacity && processOne(); ++n)
        processed = true;
    return processed;
}

bool RemoteTrajectoryWorker::processOne()
{
    // A full result channel never discards or overwrites a completed batch.
    // Backpressure stops this thread, not the trajectory/activation owner.
    if (m_has_pending) {
        if (!m_storage->results.try_push(m_storage->pending)) return false;
        m_has_pending = false;
        ++m_completed;
        return true;
    }
    if (m_storage->requests.drainTo(m_storage->request) == 0) return false;
    const auto begin = m_measure ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
    const bool accepted = reconstruct(m_storage->request[0]);
    ++m_processed;
    if (!accepted) ++m_rejected;
    if (m_measure) {
        const auto ns = static_cast<std::uint64_t>(std::chrono::duration_cast<
            std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count());
        m_total_ns += ns;
        m_max_ns = std::max(m_max_ns, ns);
    }
    return true;
}

bool RemoteTrajectoryWorker::reconstruct(const Request & request)
{
    const auto & packet = request.packet;
    auto & staging = m_storage->staging[static_cast<std::size_t>(request.vehicle_id)];
    auto & set = staging.set;
    const auto count = packet.candidate_set_size;
    if (count == 0 || count > kExhaustiveCandidatesPerAircraft
        || !((packet.candidate_set_kind == estimation::CandidateSetKind::LegacyRoll
                && count == m_legacy_count)
            || (packet.candidate_set_kind == estimation::CandidateSetKind::V4SafeControl
                && count <= kMaximumSafeControlCandidates
                && packet.candidate_id < kMaximumSafeControlCandidates))) return false;

    const bool same_key = set.selection_epoch == packet.selection_epoch
        && set.source_timestamp_us == packet.source_timestamp_us
        && set.candidate_set_kind == packet.candidate_set_kind && set.expected_count == count;
    if (!same_key && set.count > 0
        && (packet.selection_epoch < set.selection_epoch
            || (packet.selection_epoch == set.selection_epoch
                && packet.source_timestamp_us < set.source_timestamp_us))) return false;

    estimation::ReceivedTrajectoryIntent received;
    if (!m_receiver.receive(packet, received)) return false;
    // Invalid/newer packets must not erase an older, still-completable batch.
    if (!same_key) {
        staging = Staging{};
        set.vehicle_id = request.vehicle_id;
        set.selection_epoch = packet.selection_epoch;
        set.source_timestamp_us = packet.source_timestamp_us;
        set.candidate_set_kind = packet.candidate_set_kind;
        set.expected_count = count;
    }
    for (std::size_t i = 0; i < set.candidates.size(); ++i) {
        if (staging.occupied[i] && set.candidates[i].candidate_id == packet.candidate_id) {
            set.candidates[i] = received;
            return true; // duplicate IDs never complete or republish a set
        }
    }
    for (std::size_t i = 0; i < set.candidates.size(); ++i) {
        if (staging.occupied[i]) continue;
        staging.occupied[i] = true;
        set.candidates[i] = received;
        if (++set.count == set.expected_count) {
            m_storage->pending = set;
            auto & candidates = m_storage->pending.candidates;
            std::sort(candidates.begin(), candidates.begin() + set.count,
                [](const auto & a, const auto & b) { return a.candidate_id < b.candidate_id; });
            m_has_pending = true;
        }
        return true;
    }
    return false;
}

void RemoteTrajectoryWorker::loop()
{
    while (m_running.load(std::memory_order_acquire)) {
        if (!processOne()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void RemoteTrajectoryWorker::writeStoppedStatistics(std::ostream & out) const
{
    assert(!m_thread.joinable());
    if (!m_measure) return;
    // vehicle, processed packets, rejected packets, delivered complete sets,
    // sum/max elapsed reconstruction-handler nanoseconds; entire run only.
    out << "[stop-remote-worker]," << m_vehicle_id << ',' << m_processed << ','
        << m_rejected << ',' << m_completed << ',' << m_total_ns << ',' << m_max_ns << '\n';
}
} // namespace collision_avoidance::selection
