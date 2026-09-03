#include <collision_avoidance/selection/PairwiseAdCertification.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <type_traits>

namespace collision_avoidance::selection
{
namespace
{

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr double kComparisonTolerance = 1.0e-12;

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

bool finiteCandidate(const estimation::ReceivedTrajectoryIntent & intent)
    noexcept
{
    if (intent.candidate_id >= kExhaustiveCandidatesPerAircraft
        || intent.candidate_set_size != kExhaustiveCandidatesPerAircraft
        || intent.candidate_set_kind
            != estimation::CandidateSetKind::LegacyRoll
        || !std::isfinite(intent.candidate_input.V_cmd)
        || !std::isfinite(intent.candidate_input.h_dot_cmd)
        || !std::isfinite(intent.candidate_input.a_lat_cmd)
        || (intent.safe_rejoin_requested
            && !std::isfinite(
                intent.nominal_lateral_acceleration_mps2))) {
        return false;
    }
    for (const auto & point : intent.cone) {
        if (!std::isfinite(point.mean.p_n)
            || !std::isfinite(point.mean.p_e)
            || !std::isfinite(point.mean.h)
            || !std::isfinite(point.mean.V)
            || !std::isfinite(point.mean.psi)
            || !std::isfinite(point.mean.h_dot)
            || !std::isfinite(point.mean.phi)
            || !std::all_of(
                point.position_covariance_ned.begin(),
                point.position_covariance_ned.end(),
                [](double value) { return std::isfinite(value); })) {
            return false;
        }
    }
    return true;
}

}  // namespace

const CombinationEvaluation * PairwiseAdCertification::find(
    std::size_t first_candidate_slot,
    std::size_t second_candidate_slot) const noexcept
{
    if (!valid
        || first_candidate_slot >= kExhaustiveCandidatesPerAircraft
        || second_candidate_slot >= kExhaustiveCandidatesPerAircraft) {
        return nullptr;
    }
    return &evaluations[
        first_candidate_slot * kExhaustiveCandidatesPerAircraft
        + second_candidate_slot];
}

const PairwiseAdCertification * PairwiseAdCertificationSet::findPair(
    std::size_t first_aircraft,
    std::size_t second_aircraft) const noexcept
{
    if (!valid || first_aircraft >= second_aircraft
        || second_aircraft >= aircraft_count) {
        return nullptr;
    }
    for (std::size_t pair = 0; pair < pair_count; ++pair) {
        const auto & candidate = pair_certifications[pair];
        if (candidate.first_aircraft == first_aircraft
            && candidate.second_aircraft == second_aircraft) {
            return &candidate;
        }
    }
    return nullptr;
}

std::uint64_t trajectoryCandidateLibraryHash(
    std::uint64_t selection_epoch,
    std::uint64_t trajectory_library_version,
    std::uint64_t ad_masd_config_version,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    std::size_t aircraft_count) noexcept
{
    std::uint64_t hash = kFnvOffset;
    hashValue(hash, selection_epoch);
    hashValue(hash, trajectory_library_version);
    hashValue(hash, ad_masd_config_version);
    hashValue(hash, aircraft_count);
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        hashValue(hash, aircraft);
        for (const auto & intent : candidate_sets[aircraft]) {
            hashValue(hash, intent.source_timestamp_us);
            hashValue(hash, intent.selection_epoch);
            hashValue(hash, intent.candidate_id);
            hashValue(hash, intent.candidate_set_size);
            hashValue(hash, intent.candidate_set_kind);
            hashValue(hash, intent.candidate_input_revision);
            hashValue(hash, intent.candidate_input.V_cmd);
            hashValue(hash, intent.candidate_input.h_cmd);
            hashValue(hash, intent.candidate_input.h_dot_cmd);
            hashValue(hash, intent.candidate_input.a_lat_cmd);
            hashValue(hash, intent.nominal_lateral_acceleration_mps2);
            hashValue(hash, intent.safe_rejoin_requested);
            for (const auto & point : intent.cone) {
                hashValue(hash, point.mean.p_n);
                hashValue(hash, point.mean.p_e);
                hashValue(hash, point.mean.h);
                hashValue(hash, point.mean.V);
                hashValue(hash, point.mean.psi);
                hashValue(hash, point.mean.h_dot);
                hashValue(hash, point.mean.phi);
                for (double covariance : point.position_covariance_ned) {
                    hashValue(hash, covariance);
                }
            }
        }
    }
    return hash;
}

PairwiseAdCertificationEvaluator::PairwiseAdCertificationEvaluator(
    const ManeuverCombinationEvaluatorParams & params)
: m_pair_evaluator(params)
{
}

bool PairwiseAdCertificationEvaluator::evaluate(
    std::uint64_t evaluation_timestamp_us,
    std::uint64_t selection_epoch,
    std::uint64_t trajectory_library_version,
    std::uint64_t ad_masd_config_version,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    std::size_t aircraft_count,
    PairwiseAdCertificationSet & result) const
{
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    PairwiseAdCertificationSet candidate;
    candidate.evaluation_timestamp_us = evaluation_timestamp_us;
    candidate.selection_epoch = selection_epoch;
    candidate.trajectory_library_version = trajectory_library_version;
    candidate.ad_masd_config_version = ad_masd_config_version;
    candidate.aircraft_count = aircraft_count;
    candidate.participant_vehicle_ids.fill(-1);
    if (aircraft_count < 2 || aircraft_count > kMaximumSelectionAircraft
        || trajectory_library_version == 0 || ad_masd_config_version == 0) {
        result = candidate;
        return false;
    }

    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        candidate.participant_vehicle_ids[aircraft] =
            static_cast<int>(aircraft);
        const std::uint64_t source_timestamp_us =
            candidate_sets[aircraft][0].source_timestamp_us;
        const double nominal_lateral_acceleration_mps2 =
            candidate_sets[aircraft][0]
                .nominal_lateral_acceleration_mps2;
        const bool safe_rejoin_requested =
            candidate_sets[aircraft][0].safe_rejoin_requested;
        candidate.source_timestamps_us[aircraft] = source_timestamp_us;
        for (std::size_t slot = 0;
             slot < kExhaustiveCandidatesPerAircraft; ++slot) {
            const auto & intent = candidate_sets[aircraft][slot];
            if (!finiteCandidate(intent)
                || intent.selection_epoch != selection_epoch
                || intent.source_timestamp_us != source_timestamp_us
                || intent.candidate_id != slot
                || intent.safe_rejoin_requested != safe_rejoin_requested
                || !(intent.nominal_lateral_acceleration_mps2
                        == nominal_lateral_acceleration_mps2
                    || (std::isnan(
                            intent.nominal_lateral_acceleration_mps2)
                        && std::isnan(
                            nominal_lateral_acceleration_mps2)))) {
                result = candidate;
                return false;
            }
        }
    }
    candidate.candidate_library_hash = trajectoryCandidateLibraryHash(
        selection_epoch,
        trajectory_library_version,
        ad_masd_config_version,
        candidate_sets,
        aircraft_count);

    for (std::size_t first = 0; first < aircraft_count; ++first) {
        for (std::size_t second = first + 1;
             second < aircraft_count; ++second) {
            PairwiseAdCertification & certification =
                candidate.pair_certifications[candidate.pair_count++];
            certification.first_aircraft = first;
            certification.second_aircraft = second;
            double minimum_ad_m = std::numeric_limits<double>::infinity();
            for (std::size_t first_slot = 0;
                 first_slot < kExhaustiveCandidatesPerAircraft; ++first_slot) {
                for (std::size_t second_slot = 0;
                     second_slot < kExhaustiveCandidatesPerAircraft;
                     ++second_slot) {
                    const std::size_t matrix_index =
                        first_slot * kExhaustiveCandidatesPerAircraft
                        + second_slot;
                    CombinationEvaluation & evaluation =
                        certification.evaluations[matrix_index];
                    evaluation.combination_index = matrix_index;
                    if (!m_pair_evaluator.evaluatePair(
                            evaluation_timestamp_us,
                            candidate_sets[first][first_slot],
                            candidate_sets[second][second_slot],
                            evaluation)
                        || !std::isfinite(evaluation.ad_m)) {
                        result = candidate;
                        return false;
                    }
                    ++certification.evaluated_count;
                    ++candidate.evaluated_pair_candidate_count;
                    if (evaluation.ad_m < minimum_ad_m) {
                        minimum_ad_m = evaluation.ad_m;
                        certification.minimum_first_candidate_slot =
                            static_cast<std::uint8_t>(first_slot);
                        certification.minimum_second_candidate_slot =
                            static_cast<std::uint8_t>(second_slot);
                        certification.minimum_first_candidate_id =
                            candidate_sets[first][first_slot].candidate_id;
                        certification.minimum_second_candidate_id =
                            candidate_sets[second][second_slot].candidate_id;
                    }
                }
            }
            certification.minimum_ad_m = minimum_ad_m;
            certification.valid = certification.evaluated_count
                == kPairwiseAdMatrixSize;
        }
    }
    candidate.compute_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - start).count());
    candidate.valid = candidate.pair_count
            == aircraft_count * (aircraft_count - 1) / 2
        && candidate.evaluated_pair_candidate_count
            == kPairwiseAdMatrixSize * candidate.pair_count;
    result = candidate;
    return result.valid;
}

bool CertifiedComponentManeuverEvaluator::evaluate(
    const PairwiseAdCertificationSet & certifications,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & member_aircraft,
    std::size_t member_count,
    CertifiedComponentEvaluation & result,
    const ManeuverRejoinObjective * rejoin_objective) const
{
    CertifiedComponentEvaluation candidate;
    candidate.member_aircraft = member_aircraft;
    candidate.member_count = member_count;
    if (!certifications.valid || member_count < 2
        || member_count > certifications.aircraft_count) {
        result = candidate;
        return false;
    }
    std::size_t combination_count = 1;
    for (std::size_t member = 0; member < member_count; ++member) {
        if (member_aircraft[member] >= certifications.aircraft_count
            || (member > 0
                && member_aircraft[member - 1] >= member_aircraft[member])) {
            result = candidate;
            return false;
        }
        combination_count *= kExhaustiveCandidatesPerAircraft;
    }
    candidate.combination_count = combination_count;

    bool has_safe = false;
    double best_safe_cost = std::numeric_limits<double>::infinity();
    double best_safe_rejoin_cost = std::numeric_limits<double>::infinity();
    double best_unsafe_minimum_ad = -std::numeric_limits<double>::infinity();
    for (std::size_t combination_index = 0;
         combination_index < combination_count; ++combination_index) {
        JointCombinationEvaluation combination;
        combination.combination_index = combination_index;
        combination.aircraft_count = member_count;
        combination.valid = true;
        combination.all_pairs_feasible = true;
        combination.minimum_ad_m = std::numeric_limits<double>::infinity();
        combination.minimum_pmr_m = std::numeric_limits<double>::infinity();
        combination.minimum_masd_m =
            std::numeric_limits<double>::quiet_NaN();
        combination.reciprocal_cost_sum = 0.0;

        std::size_t encoded = combination_index;
        for (std::size_t member = 0; member < member_count; ++member) {
            const std::size_t reverse = member_count - 1 - member;
            combination.candidate_slots[reverse] =
                static_cast<std::uint8_t>(
                    encoded % kExhaustiveCandidatesPerAircraft);
            encoded /= kExhaustiveCandidatesPerAircraft;
        }

        for (std::size_t first_member = 0;
             first_member < member_count; ++first_member) {
            for (std::size_t second_member = first_member + 1;
                 second_member < member_count; ++second_member) {
                const std::size_t first = member_aircraft[first_member];
                const std::size_t second = member_aircraft[second_member];
                const auto * pair = certifications.findPair(first, second);
                const auto * evaluation = pair == nullptr ? nullptr
                    : pair->find(
                        combination.candidate_slots[first_member],
                        combination.candidate_slots[second_member]);
                if (evaluation == nullptr
                    || evaluation->validity != CombinationValidity::Valid) {
                    combination.valid = false;
                    combination.all_pairs_feasible = false;
                    break;
                }
                ++combination.evaluated_pair_count;
                if (evaluation->ad_m < combination.minimum_ad_m) {
                    combination.minimum_ad_m = evaluation->ad_m;
                    combination.minimum_pmr_m = evaluation->pmr_m;
                    combination.minimum_masd_m = evaluation->masd_m;
                }
                if (!evaluation->feasible
                    || !evaluation->reciprocal_cost_defined) {
                    combination.all_pairs_feasible = false;
                } else {
                    combination.reciprocal_cost_sum +=
                        evaluation->reciprocal_cost;
                }
            }
            if (!combination.valid) {
                break;
            }
        }
        if (!combination.valid) {
            continue;
        }
        ++candidate.valid_combination_count;
        if (!std::isfinite(candidate.maximum_minimum_ad_m)
            || combination.minimum_ad_m > candidate.maximum_minimum_ad_m) {
            candidate.maximum_minimum_ad_m = combination.minimum_ad_m;
        }
        if (combination.all_pairs_feasible) {
            ++candidate.safe_combination_count;
        }

        const bool use_rejoin = rejoin_objective != nullptr
            && rejoin_objective->enabled;
        if (use_rejoin) {
            combination.nominal_rejoin_cost = 0.0;
            for (std::size_t member = 0; member < member_count; ++member) {
                const std::size_t aircraft = member_aircraft[member];
                const double nominal = rejoin_objective
                    ->nominal_lateral_acceleration_mps2[aircraft];
                const double command = candidate_sets[aircraft][
                    combination.candidate_slots[member]]
                        .candidate_input.a_lat_cmd;
                if (!std::isfinite(nominal) || !std::isfinite(command)) {
                    combination.nominal_rejoin_cost =
                        std::numeric_limits<double>::infinity();
                    break;
                }
                const double error = command - nominal;
                combination.nominal_rejoin_cost += error * error;
            }
        }

        bool select = false;
        if (combination.all_pairs_feasible) {
            const bool lower_rejoin = use_rejoin
                && combination.nominal_rejoin_cost
                    < best_safe_rejoin_cost - kComparisonTolerance;
            const bool tied_rejoin = use_rejoin
                && std::abs(
                    combination.nominal_rejoin_cost
                    - best_safe_rejoin_cost) <= kComparisonTolerance;
            if (!has_safe || lower_rejoin
                || ((!use_rejoin || tied_rejoin)
                    && combination.reciprocal_cost_sum
                        < best_safe_cost - kComparisonTolerance)) {
                select = true;
                has_safe = true;
                best_safe_cost = combination.reciprocal_cost_sum;
                best_safe_rejoin_cost = combination.nominal_rejoin_cost;
            }
        } else if (!has_safe
            && (!candidate.has_best
                || combination.minimum_ad_m
                    > best_unsafe_minimum_ad + kComparisonTolerance)) {
            select = true;
            best_unsafe_minimum_ad = combination.minimum_ad_m;
        }
        if (select) {
            candidate.has_best = true;
            candidate.best_combination = combination;
            candidate.best_combination.selected_best = true;
        }
    }
    result = candidate;
    return result.has_best;
}

bool CertifiedComponentManeuverEvaluator::evaluateTuple(
    const PairwiseAdCertificationSet & certifications,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::uint8_t, kMaximumSelectionAircraft>
        & candidate_ids,
    JointCombinationEvaluation & result,
    const ManeuverRejoinObjective * rejoin_objective) const
{
    JointCombinationEvaluation candidate;
    candidate.aircraft_count = certifications.aircraft_count;
    candidate.valid = certifications.valid;
    candidate.all_pairs_feasible = certifications.valid;
    candidate.minimum_ad_m = std::numeric_limits<double>::infinity();
    candidate.minimum_pmr_m = std::numeric_limits<double>::infinity();
    candidate.minimum_masd_m = std::numeric_limits<double>::quiet_NaN();
    candidate.reciprocal_cost_sum = 0.0;
    if (!certifications.valid) {
        result = candidate;
        return false;
    }

    for (std::size_t aircraft = 0;
         aircraft < certifications.aircraft_count; ++aircraft) {
        const auto found = std::find_if(
            candidate_sets[aircraft].begin(),
            candidate_sets[aircraft].end(),
            [candidate_id = candidate_ids[aircraft]](const auto & intent) {
                return intent.candidate_id == candidate_id;
            });
        if (found == candidate_sets[aircraft].end()) {
            candidate.valid = false;
            candidate.all_pairs_feasible = false;
            result = candidate;
            return false;
        }
        candidate.candidate_slots[aircraft] = static_cast<std::uint8_t>(
            std::distance(candidate_sets[aircraft].begin(), found));
    }

    for (std::size_t first = 0;
         first < certifications.aircraft_count; ++first) {
        for (std::size_t second = first + 1;
             second < certifications.aircraft_count; ++second) {
            const auto * pair = certifications.findPair(first, second);
            const auto * evaluation = pair == nullptr ? nullptr
                : pair->find(
                    candidate.candidate_slots[first],
                    candidate.candidate_slots[second]);
            if (evaluation == nullptr
                || evaluation->validity != CombinationValidity::Valid) {
                candidate.valid = false;
                candidate.all_pairs_feasible = false;
                result = candidate;
                return false;
            }
            ++candidate.evaluated_pair_count;
            if (evaluation->ad_m < candidate.minimum_ad_m) {
                candidate.minimum_ad_m = evaluation->ad_m;
                candidate.minimum_pmr_m = evaluation->pmr_m;
                candidate.minimum_masd_m = evaluation->masd_m;
            }
            if (!evaluation->feasible
                || !evaluation->reciprocal_cost_defined) {
                candidate.all_pairs_feasible = false;
            } else {
                candidate.reciprocal_cost_sum += evaluation->reciprocal_cost;
            }
        }
    }

    if (rejoin_objective != nullptr && rejoin_objective->enabled) {
        candidate.nominal_rejoin_cost = 0.0;
        for (std::size_t aircraft = 0;
             aircraft < certifications.aircraft_count; ++aircraft) {
            const double nominal = rejoin_objective
                ->nominal_lateral_acceleration_mps2[aircraft];
            const double command = candidate_sets[aircraft][
                candidate.candidate_slots[aircraft]]
                    .candidate_input.a_lat_cmd;
            if (!std::isfinite(nominal) || !std::isfinite(command)) {
                candidate.nominal_rejoin_cost =
                    std::numeric_limits<double>::infinity();
                break;
            }
            const double error = command - nominal;
            candidate.nominal_rejoin_cost += error * error;
        }
    }
    result = candidate;
    return result.valid;
}

}  // namespace collision_avoidance::selection
