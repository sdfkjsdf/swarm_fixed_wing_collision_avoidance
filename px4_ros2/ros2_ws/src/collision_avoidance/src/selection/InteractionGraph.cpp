#include <collision_avoidance/selection/InteractionGraph.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <type_traits>

namespace collision_avoidance::selection
{
namespace
{

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

template<typename T>
void hashValue(std::uint64_t & hash, const T & value) noexcept
{
    static_assert(std::is_trivially_copyable_v<T>);
    const auto * bytes = reinterpret_cast<const unsigned char *>(&value);
    for (std::size_t index = 0; index < sizeof(T); ++index) {
        hash ^= static_cast<std::uint64_t>(bytes[index]);
        hash *= kFnvPrime;
    }
}

constexpr std::uint32_t adjacencyBit(
    std::size_t row, std::size_t column) noexcept
{
    return std::uint32_t{1}
        << (row * kMaximumInteractionGraphAircraft + column);
}

std::uint32_t integerPower(std::uint32_t base, std::size_t exponent) noexcept
{
    std::uint32_t result = 1;
    for (std::size_t index = 0; index < exponent; ++index) {
        result *= base;
    }
    return result;
}

}  // namespace

bool InteractionGraphResult::adjacent(
    std::size_t first, std::size_t second) const noexcept
{
    if (first >= aircraft_count || second >= aircraft_count
        || first == second) {
        return false;
    }
    return (adjacency_bitmask & adjacencyBit(first, second)) != 0U;
}

InteractionGraphBuilder::InteractionGraphBuilder(
    const InteractionGraphParams & params)
: m_params(params)
{
}

InteractionGraphResult InteractionGraphBuilder::build(
    const PairwiseAdCertificationSet & certifications) const noexcept
{
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    InteractionGraphResult result;
    result.participant_vehicle_ids.fill(-1);
    result.component_ids.fill(kNoInteractionComponent);
    result.pair_minimum_ad_m.fill(
        std::numeric_limits<double>::quiet_NaN());
    result.evaluation_timestamp_us = certifications.evaluation_timestamp_us;
    result.selection_epoch = certifications.selection_epoch;
    result.trajectory_library_version =
        certifications.trajectory_library_version;
    result.ad_masd_config_version = certifications.ad_masd_config_version;
    result.candidate_library_hash = certifications.candidate_library_hash;
    result.certification_compute_time_ns = certifications.compute_time_ns;
    result.aircraft_count = certifications.aircraft_count;

    if (!m_params.enabled) {
        result.status = InteractionGraphStatus::Disabled;
        return result;
    }
    if (!std::isfinite(m_params.ad_screen_m)
        || m_params.config_version == 0) {
        result.status = InteractionGraphStatus::InvalidConfiguration;
        return result;
    }
    const std::size_t expected_pair_count = certifications.aircraft_count
        * (certifications.aircraft_count - 1) / 2;
    if (!certifications.valid
        || certifications.aircraft_count < 2
        || certifications.aircraft_count
            > kMaximumInteractionGraphAircraft
        || certifications.pair_count != expected_pair_count
        || certifications.evaluated_pair_candidate_count
            != expected_pair_count * kPairwiseAdMatrixSize
        || certifications.candidate_library_hash == 0) {
        result.status = InteractionGraphStatus::InvalidCertification;
        return result;
    }

    result.participant_vehicle_ids = certifications.participant_vehicle_ids;
    result.source_timestamps_us = certifications.source_timestamps_us;
    result.trajectory_generation_count = static_cast<std::uint32_t>(
        certifications.aircraft_count * kExhaustiveCandidatesPerAircraft);
    result.pairwise_ad_evaluation_count = static_cast<std::uint32_t>(
        certifications.evaluated_pair_candidate_count);

    std::uint64_t certification_hash = kFnvOffset;
    hashValue(certification_hash, certifications.selection_epoch);
    hashValue(certification_hash, certifications.evaluation_timestamp_us);
    hashValue(certification_hash, certifications.trajectory_library_version);
    hashValue(certification_hash, certifications.ad_masd_config_version);
    hashValue(certification_hash, certifications.candidate_library_hash);
    hashValue(certification_hash, certifications.aircraft_count);

    for (std::size_t pair_index = 0;
         pair_index < certifications.pair_count; ++pair_index) {
        const auto & pair = certifications.pair_certifications[pair_index];
        if (!pair.valid || pair.evaluated_count != kPairwiseAdMatrixSize
            || !std::isfinite(pair.minimum_ad_m)
            || pair.first_aircraft >= pair.second_aircraft
            || pair.second_aircraft >= certifications.aircraft_count) {
            result.status = InteractionGraphStatus::InvalidCertification;
            return result;
        }
        result.pair_minimum_ad_m[pair_index] = pair.minimum_ad_m;
        result.pair_minimum_first_candidate_id[pair_index] =
            pair.minimum_first_candidate_id;
        result.pair_minimum_second_candidate_id[pair_index] =
            pair.minimum_second_candidate_id;
        const bool edge_required = pair.minimum_ad_m < m_params.ad_screen_m;
        result.pair_edge_required[pair_index] = edge_required ? 1U : 0U;
        if (edge_required) {
            result.adjacency_bitmask |= adjacencyBit(
                pair.first_aircraft, pair.second_aircraft);
            result.adjacency_bitmask |= adjacencyBit(
                pair.second_aircraft, pair.first_aircraft);
            ++result.edge_count;
        }
        hashValue(certification_hash, pair.first_aircraft);
        hashValue(certification_hash, pair.second_aircraft);
        hashValue(certification_hash, pair.minimum_ad_m);
        hashValue(certification_hash, pair.minimum_first_candidate_id);
        hashValue(certification_hash, pair.minimum_second_candidate_id);
        for (const auto & evaluation : pair.evaluations) {
            hashValue(certification_hash, evaluation.ownship_candidate_id);
            hashValue(certification_hash, evaluation.threat_candidate_id);
            hashValue(certification_hash, evaluation.validity);
            hashValue(certification_hash, evaluation.ad_m);
        }
    }
    result.certification_hash = certification_hash;

    std::uint8_t next_component = 0;
    for (std::size_t root = 0; root < result.aircraft_count; ++root) {
        if (result.component_ids[root] != kNoInteractionComponent) {
            continue;
        }
        std::queue<std::size_t> pending;
        pending.push(root);
        result.component_ids[root] = next_component;
        std::uint8_t component_size = 0;
        while (!pending.empty()) {
            const std::size_t current = pending.front();
            pending.pop();
            ++component_size;
            for (std::size_t neighbour = 0;
                 neighbour < result.aircraft_count; ++neighbour) {
                if (result.component_ids[neighbour]
                        == kNoInteractionComponent
                    && result.adjacent(current, neighbour)) {
                    result.component_ids[neighbour] = next_component;
                    pending.push(neighbour);
                }
            }
        }
        result.component_sizes[next_component] = component_size;
        ++next_component;
    }
    result.component_count = next_component;
    result.naive_evaluation_count = integerPower(
        static_cast<std::uint32_t>(kExhaustiveCandidatesPerAircraft),
        result.aircraft_count);
    for (std::size_t component = 0;
         component < result.component_count; ++component) {
        const std::size_t size = result.component_sizes[component];
        if (size >= 2) {
            result.component_evaluation_count += integerPower(
                static_cast<std::uint32_t>(
                    kExhaustiveCandidatesPerAircraft), size);
        }
    }

    std::uint64_t graph_hash = kFnvOffset;
    hashValue(graph_hash, result.certification_hash);
    hashValue(graph_hash, m_params.config_version);
    hashValue(graph_hash, m_params.ad_screen_m);
    hashValue(graph_hash, result.adjacency_bitmask);
    result.graph_hash = graph_hash;

    std::uint64_t component_hash = kFnvOffset;
    hashValue(component_hash, result.graph_hash);
    hashValue(component_hash, result.component_count);
    for (std::size_t aircraft = 0;
         aircraft < result.aircraft_count; ++aircraft) {
        hashValue(component_hash, result.component_ids[aircraft]);
    }
    result.component_hash = component_hash;
    result.graph_compute_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - start).count());
    result.status = InteractionGraphStatus::Valid;
    return result;
}

}  // namespace collision_avoidance::selection
