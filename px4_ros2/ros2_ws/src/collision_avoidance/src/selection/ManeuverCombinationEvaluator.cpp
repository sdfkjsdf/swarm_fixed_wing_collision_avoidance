#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace collision_avoidance::selection
{
namespace
{

constexpr double kStepSeconds = estimation::kTrajectoryIntentStepSeconds;
constexpr double kHorizonSeconds =
    static_cast<double>(estimation::kTrajectoryPointCount - 1) * kStepSeconds;
constexpr double kWindowSeconds = 1.5;
constexpr double kTimeTolerance = 1.0e-9;
constexpr double kBarrierTolerance = 1.0e-9;

struct AlignedConeSample
{
    estimation::PredictState mean{};
    std::array<double, 3> position_ned{};
    estimation::PositionCovariance position_covariance_ned{};
};

struct PairCandidateCache
{
    std::size_t first_aircraft{0};
    std::size_t second_aircraft{0};
    std::array<
        CombinationEvaluation,
        kExhaustiveCandidatesPerAircraft
            * kExhaustiveCandidatesPerAircraft> evaluations{};
    std::array<
        bool,
        kExhaustiveCandidatesPerAircraft
            * kExhaustiveCandidatesPerAircraft> ad_evaluated{};
    std::array<
        BarrierDirectionEvaluation,
        kExhaustiveCandidatesPerAircraft
            * kExhaustiveCandidatesPerAircraft> first_left{};
    std::array<
        BarrierDirectionEvaluation,
        kExhaustiveCandidatesPerAircraft
            * kExhaustiveCandidatesPerAircraft> first_right{};
    std::array<
        BarrierDirectionEvaluation,
        kExhaustiveCandidatesPerAircraft
            * kExhaustiveCandidatesPerAircraft> second_left{};
    std::array<
        BarrierDirectionEvaluation,
        kExhaustiveCandidatesPerAircraft
            * kExhaustiveCandidatesPerAircraft> second_right{};
};

bool finiteParams(const ManeuverCombinationEvaluatorParams & params) noexcept
{
    return std::isfinite(params.desired_separation_distance_m)
        && params.desired_separation_distance_m >= 0.0
        && std::isfinite(params.ownship_half_wingspan_m)
        && params.ownship_half_wingspan_m >= 0.0
        && std::isfinite(params.threat_half_wingspan_m)
        && params.threat_half_wingspan_m >= 0.0
        && std::isfinite(params.confidence_chi_squared)
        && params.confidence_chi_squared > 0.0
        && std::isfinite(params.stale_timeout_s)
        && params.stale_timeout_s > 0.0
        && (!params.positive_margin_filter_enabled
            || (std::isfinite(params.positive_margin_gamma)
                && params.positive_margin_gamma > 0.0
                && params.positive_margin_gamma <= 1.0
                && std::isfinite(params.positive_margin_reference_m)
                && params.positive_margin_reference_m > 0.0
                && std::isfinite(
                    params.maximum_lateral_acceleration_mps2)
                && params.maximum_lateral_acceleration_mps2 > 0.0));
}

bool finiteConePoint(const estimation::TrajectoryConePoint & point) noexcept
{
    const auto & mean = point.mean;
    if (!std::isfinite(mean.p_n) || !std::isfinite(mean.p_e)
        || !std::isfinite(mean.h) || !std::isfinite(mean.V)
        || !std::isfinite(mean.psi) || !std::isfinite(mean.h_dot)
        || !std::isfinite(mean.phi)) {
        return false;
    }
    return std::all_of(
        point.position_covariance_ned.begin(),
        point.position_covariance_ned.end(),
        [](double value) { return std::isfinite(value); });
}

CombinationValidity timestampValidity(
    const estimation::ReceivedTrajectoryIntent & intent,
    std::uint64_t evaluation_timestamp_us,
    double stale_timeout_s,
    double & age_s) noexcept
{
    if (intent.source_timestamp_us > evaluation_timestamp_us) {
        return CombinationValidity::FutureTimestamp;
    }
    age_s = static_cast<double>(
        evaluation_timestamp_us - intent.source_timestamp_us) * 1.0e-6;
    if (age_s > stale_timeout_s + kTimeTolerance) {
        return CombinationValidity::StaleTimestamp;
    }
    if (age_s > kHorizonSeconds + kTimeTolerance) {
        return CombinationValidity::NoCommonHorizon;
    }
    return CombinationValidity::Valid;
}

bool interpolateCone(
    const estimation::ReceivedTrajectoryIntent & intent,
    double local_time_s,
    AlignedConeSample & sample) noexcept
{
    if (!std::isfinite(local_time_s) || local_time_s < -kTimeTolerance
        || local_time_s > kHorizonSeconds + kTimeTolerance) {
        return false;
    }

    const double clamped_time = std::clamp(local_time_s, 0.0, kHorizonSeconds);
    const double fractional_index = clamped_time / kStepSeconds;
    std::size_t lower = static_cast<std::size_t>(std::floor(fractional_index));
    lower = std::min(lower, estimation::kTrajectoryPointCount - 1);
    const std::size_t upper = std::min(
        lower + 1, estimation::kTrajectoryPointCount - 1);
    const double alpha = upper == lower
        ? 0.0
        : std::clamp(fractional_index - static_cast<double>(lower), 0.0, 1.0);

    const auto & lower_point = intent.cone[lower];
    const auto & upper_point = intent.cone[upper];
    if (!finiteConePoint(lower_point) || !finiteConePoint(upper_point)) {
        return false;
    }

    const std::array<double, 3> lower_position{
        lower_point.mean.p_n,
        lower_point.mean.p_e,
        -lower_point.mean.h};
    const std::array<double, 3> upper_position{
        upper_point.mean.p_n,
        upper_point.mean.p_e,
        -upper_point.mean.h};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        sample.position_ned[axis] =
            (1.0 - alpha) * lower_position[axis]
            + alpha * upper_position[axis];
    }
    sample.mean.p_n = sample.position_ned[0];
    sample.mean.p_e = sample.position_ned[1];
    sample.mean.h = -sample.position_ned[2];
    sample.mean.V = (1.0 - alpha) * lower_point.mean.V
        + alpha * upper_point.mean.V;
    sample.mean.psi = lower_point.mean.psi + alpha * std::remainder(
        upper_point.mean.psi - lower_point.mean.psi, 2.0 * std::acos(-1.0));
    sample.mean.h_dot = (1.0 - alpha) * lower_point.mean.h_dot
        + alpha * upper_point.mean.h_dot;
    sample.mean.phi = (1.0 - alpha) * lower_point.mean.phi
        + alpha * upper_point.mean.phi;
    for (std::size_t element = 0;
         element < sample.position_covariance_ned.size(); ++element) {
        sample.position_covariance_ned[element] =
            (1.0 - alpha) * lower_point.position_covariance_ned[element]
            + alpha * upper_point.position_covariance_ned[element];
    }
    return true;
}

CombinationValidity alignedHorizon(
    std::uint64_t evaluation_timestamp_us,
    const estimation::ReceivedTrajectoryIntent & ownship_intent,
    const estimation::ReceivedTrajectoryIntent & threat_intent,
    double stale_timeout_s,
    double & ownship_age_s,
    double & threat_age_s,
    std::size_t & sample_count) noexcept
{
    const CombinationValidity ownship_validity = timestampValidity(
        ownship_intent,
        evaluation_timestamp_us,
        stale_timeout_s,
        ownship_age_s);
    if (ownship_validity != CombinationValidity::Valid) {
        return ownship_validity;
    }
    const CombinationValidity threat_validity = timestampValidity(
        threat_intent,
        evaluation_timestamp_us,
        stale_timeout_s,
        threat_age_s);
    if (threat_validity != CombinationValidity::Valid) {
        return threat_validity;
    }

    const double common_horizon_s = std::min(
        kHorizonSeconds - ownship_age_s,
        kHorizonSeconds - threat_age_s);
    if (common_horizon_s < -kTimeTolerance) {
        return CombinationValidity::NoCommonHorizon;
    }
    sample_count = std::min(
        estimation::kTrajectoryPointCount,
        static_cast<std::size_t>(
            std::floor((common_horizon_s + kTimeTolerance) / kStepSeconds)) + 1);
    return sample_count == 0
        ? CombinationValidity::NoCommonHorizon
        : CombinationValidity::Valid;
}

enum class CandidateDirectionRequirement : std::uint8_t
{
    Left,
    Right,
    EitherSingleDirection,
};

CandidateDirectionRequirement directionRequirement(
    std::uint8_t candidate_id) noexcept
{
    using estimation::ManeuverCandidateId;
    switch (static_cast<ManeuverCandidateId>(candidate_id)) {
    case ManeuverCandidateId::RollMinus50:
    case ManeuverCandidateId::RollMinus30:
    case ManeuverCandidateId::RollMinus15:
        return CandidateDirectionRequirement::Left;
    case ManeuverCandidateId::RollPlus15:
    case ManeuverCandidateId::RollPlus30:
    case ManeuverCandidateId::RollPlus50:
        return CandidateDirectionRequirement::Right;
    case ManeuverCandidateId::RollZero:
        return CandidateDirectionRequirement::EitherSingleDirection;
    }
    return CandidateDirectionRequirement::EitherSingleDirection;
}

bool candidateBarrierAdmissible(
    std::uint8_t candidate_id,
    bool left_admissible,
    bool right_admissible) noexcept
{
    if (candidate_id >= estimation::kManeuverCandidateCount) {
        return false;
    }
    switch (directionRequirement(candidate_id)) {
    case CandidateDirectionRequirement::Left:
        return left_admissible;
    case CandidateDirectionRequirement::Right:
        return right_admissible;
    case CandidateDirectionRequirement::EitherSingleDirection:
        return left_admissible || right_admissible;
    }
    return false;
}

double selectedBarrierResidual(
    std::uint8_t candidate_id,
    bool left_admissible,
    double left_residual,
    bool right_admissible,
    double right_residual) noexcept
{
    if (candidate_id >= estimation::kManeuverCandidateCount) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    switch (directionRequirement(candidate_id)) {
    case CandidateDirectionRequirement::Left:
        return left_residual;
    case CandidateDirectionRequirement::Right:
        return right_residual;
    case CandidateDirectionRequirement::EitherSingleDirection:
        static_cast<void>(left_admissible);
        static_cast<void>(right_admissible);
        return std::max(left_residual, right_residual);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

struct BarrierPairEvaluations
{
    BarrierDirectionEvaluation first_left{};
    BarrierDirectionEvaluation first_right{};
    BarrierDirectionEvaluation second_left{};
    BarrierDirectionEvaluation second_right{};
};

template<typename PairEvaluationProvider>
bool evaluateSelectedCombinationBarrier(
    const std::array<
        const estimation::ReceivedTrajectoryIntent *,
        kMaximumSelectionAircraft> & selected_intents,
    std::size_t aircraft_count,
    PairEvaluationProvider && provide_pair_evaluations,
    JointCombinationEvaluation & combination)
{
    combination.barrier_evaluated = true;
    std::array<bool, kMaximumSelectionAircraft> left_admissible{};
    std::array<bool, kMaximumSelectionAircraft> right_admissible{};
    std::array<double, kMaximumSelectionAircraft> left_residual{};
    std::array<double, kMaximumSelectionAircraft> right_residual{};
    left_admissible.fill(true);
    right_admissible.fill(true);
    left_residual.fill(std::numeric_limits<double>::infinity());
    right_residual.fill(std::numeric_limits<double>::infinity());

    for (std::size_t first = 0; first < aircraft_count; ++first) {
        if (selected_intents[first] == nullptr) {
            combination.valid = false;
            break;
        }
        for (std::size_t second = first + 1;
             second < aircraft_count; ++second) {
            if (selected_intents[second] == nullptr) {
                combination.valid = false;
                break;
            }
            BarrierPairEvaluations pair{};
            if (!provide_pair_evaluations(first, second, pair)
                || pair.first_left.validity != CombinationValidity::Valid
                || pair.first_right.validity != CombinationValidity::Valid
                || pair.second_left.validity != CombinationValidity::Valid
                || pair.second_right.validity != CombinationValidity::Valid) {
                combination.valid = false;
                break;
            }
            left_admissible[first] = left_admissible[first]
                && pair.first_left.admissible;
            right_admissible[first] = right_admissible[first]
                && pair.first_right.admissible;
            left_admissible[second] = left_admissible[second]
                && pair.second_left.admissible;
            right_admissible[second] = right_admissible[second]
                && pair.second_right.admissible;
            left_residual[first] = std::min(
                left_residual[first], pair.first_left.minimum_residual_m);
            right_residual[first] = std::min(
                right_residual[first], pair.first_right.minimum_residual_m);
            left_residual[second] = std::min(
                left_residual[second], pair.second_left.minimum_residual_m);
            right_residual[second] = std::min(
                right_residual[second], pair.second_right.minimum_residual_m);
        }
        if (!combination.valid) {
            break;
        }
    }

    combination.barrier_admissible = combination.valid;
    combination.minimum_barrier_residual_m =
        std::numeric_limits<double>::infinity();
    if (combination.valid) {
        for (std::size_t aircraft = 0;
             aircraft < aircraft_count; ++aircraft) {
            const std::uint8_t candidate_id =
                selected_intents[aircraft]->candidate_id;
            combination.barrier_admissible =
                combination.barrier_admissible
                && candidateBarrierAdmissible(
                    candidate_id,
                    left_admissible[aircraft],
                    right_admissible[aircraft]);
            combination.minimum_barrier_residual_m = std::min(
                combination.minimum_barrier_residual_m,
                selectedBarrierResidual(
                    candidate_id,
                    left_admissible[aircraft],
                    left_residual[aircraft],
                    right_admissible[aircraft],
                    right_residual[aircraft]));
        }
    }
    if (!combination.valid || !combination.barrier_admissible) {
        combination.barrier_admissible = false;
        combination.all_pairs_feasible = false;
        combination.reciprocal_cost_sum =
            std::numeric_limits<double>::quiet_NaN();
        return false;
    }
    return true;
}

std::size_t pairCacheIndex(
    std::size_t first,
    std::size_t second,
    std::size_t aircraft_count) noexcept
{
    return first * (2 * aircraft_count - first - 1) / 2
        + second - first - 1;
}

template<typename CandidateSets>
std::size_t initializePairCandidateCaches(
    std::uint64_t evaluation_timestamp_us,
    const CandidateSets & candidate_sets,
    std::size_t aircraft_count,
    std::size_t candidate_count,
    bool positive_margin_filter_enabled,
    const PositiveMarginBarrierEvaluator & barrier_evaluator,
    const ManeuverCombinationEvaluator & pair_evaluator,
    std::array<PairCandidateCache, kMaximumSelectionPairCount> & pair_caches,
    std::size_t & evaluated_ad_pair_count)
{
    std::size_t pair_cache_count = 0;
    for (std::size_t first = 0; first < aircraft_count; ++first) {
        for (std::size_t second = first + 1;
             second < aircraft_count; ++second) {
            PairCandidateCache & cache = pair_caches[pair_cache_count++];
            cache.first_aircraft = first;
            cache.second_aircraft = second;
            for (std::size_t first_candidate = 0;
                 first_candidate < candidate_count; ++first_candidate) {
                for (std::size_t second_candidate = 0;
                     second_candidate < candidate_count; ++second_candidate) {
                    const std::size_t cache_index =
                        first_candidate * candidate_count + second_candidate;
                    if (positive_margin_filter_enabled) {
                        static_cast<void>(barrier_evaluator.evaluateDirection(
                            evaluation_timestamp_us,
                            candidate_sets[first][first_candidate],
                            candidate_sets[second][second_candidate],
                            BarrierDirection::Left,
                            cache.first_left[cache_index]));
                        static_cast<void>(barrier_evaluator.evaluateDirection(
                            evaluation_timestamp_us,
                            candidate_sets[first][first_candidate],
                            candidate_sets[second][second_candidate],
                            BarrierDirection::Right,
                            cache.first_right[cache_index]));
                        static_cast<void>(barrier_evaluator.evaluateDirection(
                            evaluation_timestamp_us,
                            candidate_sets[second][second_candidate],
                            candidate_sets[first][first_candidate],
                            BarrierDirection::Left,
                            cache.second_left[cache_index]));
                        static_cast<void>(barrier_evaluator.evaluateDirection(
                            evaluation_timestamp_us,
                            candidate_sets[second][second_candidate],
                            candidate_sets[first][first_candidate],
                            BarrierDirection::Right,
                            cache.second_right[cache_index]));
                    } else {
                        static_cast<void>(pair_evaluator.evaluatePair(
                            evaluation_timestamp_us,
                            candidate_sets[first][first_candidate],
                            candidate_sets[second][second_candidate],
                            cache.evaluations[cache_index]));
                        cache.ad_evaluated[cache_index] = true;
                        ++evaluated_ad_pair_count;
                    }
                }
            }
        }
    }
    return pair_cache_count;
}

std::size_t pmrWindow(double time_offset_s) noexcept
{
    if (time_offset_s < kWindowSeconds - kTimeTolerance) {
        return 0;
    }
    if (time_offset_s < 2.0 * kWindowSeconds - kTimeTolerance) {
        return 1;
    }
    return 2;
}

double distance(
    const AlignedConeSample & ownship,
    const AlignedConeSample & threat) noexcept
{
    const double north = threat.position_ned[0] - ownship.position_ned[0];
    const double east = threat.position_ned[1] - ownship.position_ned[1];
    const double down = threat.position_ned[2] - ownship.position_ned[2];
    return std::sqrt(north * north + east * east + down * down);
}

bool uncertaintyMargin(
    const AlignedConeSample & ownship,
    const AlignedConeSample & threat,
    double pmr_m,
    double confidence_chi_squared,
    double & margin_m) noexcept
{
    estimation::PositionCovariance relative_covariance{};
    for (std::size_t index = 0; index < relative_covariance.size(); ++index) {
        relative_covariance[index] =
            ownship.position_covariance_ned[index]
            + threat.position_covariance_ned[index];
    }

    double variance = 0.0;
    if (pmr_m > 1.0e-9) {
        const std::array<double, 3> direction{
            (threat.position_ned[0] - ownship.position_ned[0]) / pmr_m,
            (threat.position_ned[1] - ownship.position_ned[1]) / pmr_m,
            (threat.position_ned[2] - ownship.position_ned[2]) / pmr_m};
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                variance += direction[row]
                    * relative_covariance[row * 3 + column]
                    * direction[column];
            }
        }
    } else {
        // LOS is undefined at nominal coincidence. Trace is a conservative
        // PSD upper bound for the directional variance and affects reporting,
        // not the already-negative collision classification.
        variance = relative_covariance[0]
            + relative_covariance[4]
            + relative_covariance[8];
    }

    if (!std::isfinite(variance) || variance < -1.0e-8) {
        return false;
    }
    margin_m = std::sqrt(
        confidence_chi_squared * std::max(0.0, variance));
    return std::isfinite(margin_m);
}

CombinationValidity evaluatePairImpl(
    std::uint64_t evaluation_timestamp_us,
    const estimation::ReceivedTrajectoryIntent & ownship_intent,
    const estimation::ReceivedTrajectoryIntent & threat_intent,
    const ManeuverCombinationEvaluatorParams & params,
    CombinationEvaluation & result) noexcept
{
    double ownship_age_s = 0.0;
    double threat_age_s = 0.0;
    std::size_t sample_count = 0;
    const CombinationValidity horizon_validity = alignedHorizon(
        evaluation_timestamp_us,
        ownship_intent,
        threat_intent,
        params.stale_timeout_s,
        ownship_age_s,
        threat_age_s,
        sample_count);
    if (horizon_validity != CombinationValidity::Valid) {
        return horizon_validity;
    }

    for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
        const double time_offset_s =
            static_cast<double>(sample_index) * kStepSeconds;
        AlignedConeSample ownship_sample;
        AlignedConeSample threat_sample;
        if (!interpolateCone(
                ownship_intent,
                ownship_age_s + time_offset_s,
                ownship_sample)
            || !interpolateCone(
                threat_intent,
                threat_age_s + time_offset_s,
                threat_sample)) {
            return CombinationValidity::InvalidTrajectory;
        }

        const double range_m = distance(ownship_sample, threat_sample);
        if (!std::isfinite(range_m)) {
            return CombinationValidity::InvalidTrajectory;
        }
        const std::size_t window = pmrWindow(time_offset_s);
        PmrWindowResult & window_result = result.pmr_windows[window];
        if (!window_result.available || range_m < window_result.pmr_m) {
            window_result.available = true;
            window_result.pmr_m = range_m;
            window_result.time_offset_s = time_offset_s;
            window_result.sample_index = sample_index;
        }
    }

    result.evaluated_sample_count = sample_count;
    bool found_pmr = false;
    for (std::size_t window = 0; window < result.pmr_windows.size(); ++window) {
        const PmrWindowResult & candidate = result.pmr_windows[window];
        if (!candidate.available) {
            continue;
        }
        if (!found_pmr || candidate.pmr_m < result.pmr_m) {
            found_pmr = true;
            result.selected_pmr_window = window;
            result.pmr_m = candidate.pmr_m;
            result.pmr_time_offset_s = candidate.time_offset_s;
        }
    }
    if (!found_pmr) {
        return CombinationValidity::NoCommonHorizon;
    }

    AlignedConeSample ownship_at_pmr;
    AlignedConeSample threat_at_pmr;
    if (!interpolateCone(
            ownship_intent,
            ownship_age_s + result.pmr_time_offset_s,
            ownship_at_pmr)
        || !interpolateCone(
            threat_intent,
            threat_age_s + result.pmr_time_offset_s,
            threat_at_pmr)) {
        return CombinationValidity::InvalidTrajectory;
    }

    if (!uncertaintyMargin(
            ownship_at_pmr,
            threat_at_pmr,
            result.pmr_m,
            params.confidence_chi_squared,
            result.uncertainty_margin_95_m)) {
        return CombinationValidity::InvalidTrajectory;
    }

    result.pmr_timestamp_us = evaluation_timestamp_us
        + static_cast<std::uint64_t>(std::llround(
            result.pmr_time_offset_s * 1.0e6));
    result.aircraft_size_margin_m =
        params.ownship_half_wingspan_m + params.threat_half_wingspan_m;
    result.desired_separation_distance_m =
        params.desired_separation_distance_m;
    result.masd_m = result.aircraft_size_margin_m
        + result.desired_separation_distance_m
        + result.uncertainty_margin_95_m;
    result.ad_m = result.pmr_m - result.masd_m;
    result.feasible = result.ad_m > 0.0;
    result.reciprocal_cost_defined = result.feasible;
    if (result.reciprocal_cost_defined) {
        result.reciprocal_cost = 1.0 / result.ad_m;
    }
    return CombinationValidity::Valid;
}

}  // namespace

PositiveMarginBarrierEvaluator::PositiveMarginBarrierEvaluator(
    const ManeuverCombinationEvaluatorParams & params)
: m_params(params)
{
}

bool PositiveMarginBarrierEvaluator::evaluateDirection(
    std::uint64_t evaluation_timestamp_us,
    const estimation::ReceivedTrajectoryIntent & ownship_intent,
    const estimation::ReceivedTrajectoryIntent & threat_intent,
    BarrierDirection direction,
    BarrierDirectionEvaluation & evaluation) const
{
    BarrierDirectionEvaluation result{};
    result.direction = direction;
    if (!std::isfinite(m_params.positive_margin_gamma)
        || m_params.positive_margin_gamma <= 0.0
        || m_params.positive_margin_gamma > 1.0
        || !std::isfinite(m_params.positive_margin_reference_m)
        || m_params.positive_margin_reference_m <= 0.0
        || !std::isfinite(m_params.maximum_lateral_acceleration_mps2)
        || m_params.maximum_lateral_acceleration_mps2 <= 0.0
        || !std::isfinite(m_params.ownship_half_wingspan_m)
        || m_params.ownship_half_wingspan_m < 0.0
        || !std::isfinite(m_params.threat_half_wingspan_m)
        || m_params.threat_half_wingspan_m < 0.0
        || !std::isfinite(m_params.stale_timeout_s)
        || m_params.stale_timeout_s <= 0.0) {
        evaluation = result;
        return false;
    }

    double ownship_age_s = 0.0;
    double threat_age_s = 0.0;
    std::size_t sample_count = 0;
    result.validity = alignedHorizon(
        evaluation_timestamp_us,
        ownship_intent,
        threat_intent,
        m_params.stale_timeout_s,
        ownship_age_s,
        threat_age_s,
        sample_count);
    if (result.validity != CombinationValidity::Valid || sample_count < 2) {
        if (result.validity == CombinationValidity::Valid) {
            result.validity = CombinationValidity::NoCommonHorizon;
        }
        evaluation = result;
        return false;
    }

    const double direction_sign = direction == BarrierDirection::Right
        ? 1.0
        : -1.0;
    const double size_clearance_m = m_params.ownship_half_wingspan_m
        + m_params.threat_half_wingspan_m;
    const auto clearance = [&](const AlignedConeSample & ownship,
                               const AlignedConeSample & threat,
                               double & value) {
        const double horizontal_speed_squared = std::max(
            0.0,
            ownship.mean.V * ownship.mean.V
                - ownship.mean.h_dot * ownship.mean.h_dot);
        const double radius_m = horizontal_speed_squared
            / m_params.maximum_lateral_acceleration_mps2;
        const double right_north = -std::sin(ownship.mean.psi);
        const double right_east = std::cos(ownship.mean.psi);
        const double center_north = ownship.mean.p_n
            + direction_sign * radius_m * right_north;
        const double center_east = ownship.mean.p_e
            + direction_sign * radius_m * right_east;
        value = std::hypot(
            center_north - threat.mean.p_n,
            center_east - threat.mean.p_e)
            - (size_clearance_m + radius_m);
        return std::isfinite(value);
    };

    bool all_residuals_nonnegative = true;
    bool all_clearances_nonnegative = true;
    result.minimum_clearance_m = std::numeric_limits<double>::infinity();
    result.minimum_residual_m = std::numeric_limits<double>::infinity();
    for (std::size_t interval = 0; interval + 1 < sample_count; ++interval) {
        const double time_offset_s = static_cast<double>(interval) * kStepSeconds;
        AlignedConeSample ownship_current;
        AlignedConeSample ownship_next;
        AlignedConeSample threat_current;
        AlignedConeSample threat_next;
        if (!interpolateCone(
                ownship_intent,
                ownship_age_s + time_offset_s,
                ownship_current)
            || !interpolateCone(
                ownship_intent,
                ownship_age_s + time_offset_s + kStepSeconds,
                ownship_next)
            || !interpolateCone(
                threat_intent,
                threat_age_s + time_offset_s,
                threat_current)
            || !interpolateCone(
                threat_intent,
                threat_age_s + time_offset_s + kStepSeconds,
                threat_next)) {
            result.validity = CombinationValidity::InvalidTrajectory;
            evaluation = result;
            return false;
        }

        double current_clearance_m = 0.0;
        double next_clearance_m = 0.0;
        if (!clearance(ownship_current, threat_current, current_clearance_m)
            || !clearance(ownship_next, threat_next, next_clearance_m)) {
            result.validity = CombinationValidity::InvalidTrajectory;
            evaluation = result;
            return false;
        }

        result.minimum_clearance_m = std::min(
            result.minimum_clearance_m,
            std::min(current_clearance_m, next_clearance_m));
        if (interval == 0) {
            result.initial_clearance_nonnegative =
                current_clearance_m >= -kBarrierTolerance;
        }
        const double required_next_m =
            (1.0 - m_params.positive_margin_gamma) * current_clearance_m
            + m_params.positive_margin_gamma
                * m_params.positive_margin_reference_m;
        const double residual_m = next_clearance_m - required_next_m;
        result.minimum_residual_m = std::min(
            result.minimum_residual_m, residual_m);
        ++result.evaluated_interval_count;

        const bool clearance_ok = current_clearance_m >= -kBarrierTolerance
            && next_clearance_m >= -kBarrierTolerance;
        const bool residual_ok = residual_m >= -kBarrierTolerance;
        if ((!clearance_ok || !residual_ok)
            && all_clearances_nonnegative && all_residuals_nonnegative) {
            result.first_violation_interval = interval;
        }
        all_clearances_nonnegative = all_clearances_nonnegative && clearance_ok;
        all_residuals_nonnegative = all_residuals_nonnegative && residual_ok;
    }

    result.admissible = result.initial_clearance_nonnegative
        && all_clearances_nonnegative && all_residuals_nonnegative;
    evaluation = result;
    return true;
}

ManeuverCombinationEvaluator::ManeuverCombinationEvaluator(
    const ManeuverCombinationEvaluatorParams & params)
: m_params(params)
{
}

bool ManeuverCombinationEvaluator::evaluatePair(
    std::uint64_t evaluation_timestamp_us,
    const estimation::ReceivedTrajectoryIntent & ownship_intent,
    const estimation::ReceivedTrajectoryIntent & threat_intent,
    CombinationEvaluation & evaluation) const
{
    CombinationEvaluation candidate{};
    candidate.ownship_candidate_id = ownship_intent.candidate_id;
    candidate.threat_candidate_id = threat_intent.candidate_id;
    candidate.validity = evaluatePairImpl(
        evaluation_timestamp_us,
        ownship_intent,
        threat_intent,
        m_params,
        candidate);
    evaluation = candidate;
    return evaluation.validity == CombinationValidity::Valid;
}

bool ManeuverCombinationEvaluator::evaluate(
    std::uint64_t evaluation_timestamp_us,
    const CandidateIntentSet & ownship_candidates,
    const CandidateIntentSet & threat_candidates,
    StaticCombinationEvaluation & evaluation) const
{
    StaticCombinationEvaluation candidate_evaluation{};
    candidate_evaluation.evaluation_timestamp_us = evaluation_timestamp_us;
    if (!finiteParams(m_params)) {
        evaluation = candidate_evaluation;
        return false;
    }

    double best_ad = -std::numeric_limits<double>::infinity();
    const PositiveMarginBarrierEvaluator barrier_evaluator(m_params);
    for (std::size_t ownship_index = 0;
         ownship_index < kCandidatesPerAircraft; ++ownship_index) {
        for (std::size_t threat_index = 0;
             threat_index < kCandidatesPerAircraft; ++threat_index) {
            const std::size_t combination_index =
                ownship_index * kCandidatesPerAircraft + threat_index;
            CombinationEvaluation & combination =
                candidate_evaluation.combinations[combination_index];
            combination.combination_index = combination_index;
            combination.ownship_candidate_id =
                ownship_candidates[ownship_index].candidate_id;
            combination.threat_candidate_id =
                threat_candidates[threat_index].candidate_id;
            if (m_params.positive_margin_filter_enabled) {
                BarrierDirectionEvaluation ownship_left;
                BarrierDirectionEvaluation ownship_right;
                BarrierDirectionEvaluation threat_left;
                BarrierDirectionEvaluation threat_right;
                combination.barrier_evaluated = true;
                const bool barrier_valid =
                    barrier_evaluator.evaluateDirection(
                        evaluation_timestamp_us,
                        ownship_candidates[ownship_index],
                        threat_candidates[threat_index],
                        BarrierDirection::Left,
                        ownship_left)
                    && barrier_evaluator.evaluateDirection(
                        evaluation_timestamp_us,
                        ownship_candidates[ownship_index],
                        threat_candidates[threat_index],
                        BarrierDirection::Right,
                        ownship_right)
                    && barrier_evaluator.evaluateDirection(
                        evaluation_timestamp_us,
                        threat_candidates[threat_index],
                        ownship_candidates[ownship_index],
                        BarrierDirection::Left,
                        threat_left)
                    && barrier_evaluator.evaluateDirection(
                        evaluation_timestamp_us,
                        threat_candidates[threat_index],
                        ownship_candidates[ownship_index],
                        BarrierDirection::Right,
                        threat_right);
                if (!barrier_valid) {
                    combination.validity = ownship_left.validity
                        != CombinationValidity::Valid
                        ? ownship_left.validity
                        : ownship_right.validity != CombinationValidity::Valid
                            ? ownship_right.validity
                            : threat_left.validity != CombinationValidity::Valid
                                ? threat_left.validity
                                : threat_right.validity;
                    continue;
                }
                const bool ownship_admissible = candidateBarrierAdmissible(
                    combination.ownship_candidate_id,
                    ownship_left.admissible,
                    ownship_right.admissible);
                const bool threat_admissible = candidateBarrierAdmissible(
                    combination.threat_candidate_id,
                    threat_left.admissible,
                    threat_right.admissible);
                combination.minimum_barrier_residual_m = std::min(
                    selectedBarrierResidual(
                        combination.ownship_candidate_id,
                        ownship_left.admissible,
                        ownship_left.minimum_residual_m,
                        ownship_right.admissible,
                        ownship_right.minimum_residual_m),
                    selectedBarrierResidual(
                        combination.threat_candidate_id,
                        threat_left.admissible,
                        threat_left.minimum_residual_m,
                        threat_right.admissible,
                        threat_right.minimum_residual_m));
                combination.barrier_admissible =
                    ownship_admissible && threat_admissible;
                if (!combination.barrier_admissible) {
                    combination.validity = CombinationValidity::BarrierRejected;
                    continue;
                }
            }
            const bool barrier_evaluated = combination.barrier_evaluated;
            const bool barrier_admissible = combination.barrier_admissible;
            const double minimum_barrier_residual_m =
                combination.minimum_barrier_residual_m;
            static_cast<void>(evaluatePair(
                evaluation_timestamp_us,
                ownship_candidates[ownship_index],
                threat_candidates[threat_index],
                combination));
            combination.combination_index = combination_index;
            combination.barrier_evaluated = barrier_evaluated;
            combination.barrier_admissible = barrier_admissible;
            combination.minimum_barrier_residual_m =
                minimum_barrier_residual_m;

            if (combination.validity == CombinationValidity::Valid
                && (!candidate_evaluation.has_best
                    || combination.ad_m > best_ad)) {
                candidate_evaluation.has_best = true;
                candidate_evaluation.best_combination_index = combination_index;
                best_ad = combination.ad_m;
            }
        }
    }

    if (candidate_evaluation.has_best) {
        candidate_evaluation.combinations[
            candidate_evaluation.best_combination_index].selected_best = true;
    }
    evaluation = candidate_evaluation;
    return evaluation.has_best;
}

JointManeuverCombinationEvaluator::JointManeuverCombinationEvaluator(
    const ManeuverCombinationEvaluatorParams & params)
: m_pair_evaluator(params),
  m_barrier_evaluator(params),
  m_positive_margin_filter_enabled(params.positive_margin_filter_enabled)
{
}

bool JointManeuverCombinationEvaluator::evaluate(
    std::uint64_t evaluation_timestamp_us,
    const MultiAircraftCandidateIntentSets & candidate_sets,
    std::size_t aircraft_count,
    JointManeuverEvaluation & evaluation) const
{
    JointManeuverEvaluation candidate_evaluation{};
    candidate_evaluation.evaluation_timestamp_us = evaluation_timestamp_us;
    candidate_evaluation.aircraft_count = aircraft_count;
    if (aircraft_count < 2 || aircraft_count > kMaximumSelectionAircraft) {
        evaluation = candidate_evaluation;
        return false;
    }

    std::size_t combination_count = 1;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        combination_count *= kCandidatesPerAircraft;
    }
    candidate_evaluation.combination_count = combination_count;

    std::array<PairCandidateCache, kMaximumSelectionPairCount> pair_caches{};
    std::size_t ignored_evaluated_ad_pair_count = 0;
    const std::size_t pair_cache_count = initializePairCandidateCaches(
        evaluation_timestamp_us,
        candidate_sets,
        aircraft_count,
        kCandidatesPerAircraft,
        m_positive_margin_filter_enabled,
        m_barrier_evaluator,
        m_pair_evaluator,
        pair_caches,
        ignored_evaluated_ad_pair_count);

    bool has_safe_combination = false;
    double best_safe_cost = std::numeric_limits<double>::infinity();
    double best_unsafe_minimum_ad = -std::numeric_limits<double>::infinity();
    constexpr double comparison_tolerance = 1.0e-12;

    for (std::size_t combination_index = 0;
         combination_index < combination_count; ++combination_index) {
        JointCombinationEvaluation & combination =
            candidate_evaluation.combinations[combination_index];
        combination.combination_index = combination_index;
        combination.aircraft_count = aircraft_count;

        std::size_t encoded = combination_index;
        for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
            const std::size_t reverse_aircraft = aircraft_count - 1 - aircraft;
            combination.candidate_slots[reverse_aircraft] =
                static_cast<std::uint8_t>(encoded % kCandidatesPerAircraft);
            encoded /= kCandidatesPerAircraft;
        }

        combination.valid = true;
        combination.all_pairs_feasible = true;
        combination.minimum_ad_m = std::numeric_limits<double>::infinity();
        combination.minimum_pmr_m = std::numeric_limits<double>::infinity();
        combination.minimum_masd_m = std::numeric_limits<double>::quiet_NaN();
        combination.reciprocal_cost_sum = 0.0;

        if (m_positive_margin_filter_enabled) {
            std::array<
                const estimation::ReceivedTrajectoryIntent *,
                kMaximumSelectionAircraft> selected_intents{};
            for (std::size_t aircraft = 0;
                 aircraft < aircraft_count; ++aircraft) {
                selected_intents[aircraft] = &candidate_sets[aircraft][
                    combination.candidate_slots[aircraft]];
            }
            const auto provide_pair = [aircraft_count,
                                       &pair_caches,
                                       &combination](
                                          std::size_t first,
                                          std::size_t second,
                                          BarrierPairEvaluations & pair) {
                const PairCandidateCache & cache = pair_caches[
                    pairCacheIndex(first, second, aircraft_count)];
                const std::size_t cache_index =
                    static_cast<std::size_t>(
                        combination.candidate_slots[first])
                        * kCandidatesPerAircraft
                    + combination.candidate_slots[second];
                pair.first_left = cache.first_left[cache_index];
                pair.first_right = cache.first_right[cache_index];
                pair.second_left = cache.second_left[cache_index];
                pair.second_right = cache.second_right[cache_index];
                return true;
            };
            if (!evaluateSelectedCombinationBarrier(
                    selected_intents,
                    aircraft_count,
                    provide_pair,
                    combination)) {
                continue;
            }
        }

        for (std::size_t pair_index = 0;
             pair_index < pair_cache_count; ++pair_index) {
            PairCandidateCache & cache = pair_caches[pair_index];
            const std::size_t first_candidate =
                combination.candidate_slots[cache.first_aircraft];
            const std::size_t second_candidate =
                combination.candidate_slots[cache.second_aircraft];
            const std::size_t cache_index =
                first_candidate * kCandidatesPerAircraft + second_candidate;
            if (!cache.ad_evaluated[cache_index]) {
                static_cast<void>(m_pair_evaluator.evaluatePair(
                    evaluation_timestamp_us,
                    candidate_sets[cache.first_aircraft][first_candidate],
                    candidate_sets[cache.second_aircraft][second_candidate],
                    cache.evaluations[cache_index]));
                cache.ad_evaluated[cache_index] = true;
            }
            const CombinationEvaluation & pair = cache.evaluations[cache_index];
            if (pair.validity != CombinationValidity::Valid) {
                combination.valid = false;
                combination.all_pairs_feasible = false;
                break;
            }
            ++combination.evaluated_pair_count;
            if (pair.ad_m < combination.minimum_ad_m) {
                combination.minimum_ad_m = pair.ad_m;
                combination.minimum_pmr_m = pair.pmr_m;
                combination.minimum_masd_m = pair.masd_m;
            }
            if (!pair.feasible || !pair.reciprocal_cost_defined) {
                combination.all_pairs_feasible = false;
            } else {
                combination.reciprocal_cost_sum += pair.reciprocal_cost;
            }
        }

        if (!combination.valid) {
            combination.reciprocal_cost_sum =
                std::numeric_limits<double>::quiet_NaN();
            continue;
        }

        bool select = false;
        if (combination.all_pairs_feasible) {
            if (!has_safe_combination
                || combination.reciprocal_cost_sum
                    < best_safe_cost - comparison_tolerance) {
                select = true;
                has_safe_combination = true;
                best_safe_cost = combination.reciprocal_cost_sum;
            }
        } else if (!has_safe_combination
            && (!candidate_evaluation.has_best
                || combination.minimum_ad_m
                    > best_unsafe_minimum_ad + comparison_tolerance)) {
            select = true;
            best_unsafe_minimum_ad = combination.minimum_ad_m;
        }

        if (select) {
            if (candidate_evaluation.has_best) {
                candidate_evaluation.combinations[
                    candidate_evaluation.best_combination_index].selected_best = false;
            }
            candidate_evaluation.has_best = true;
            candidate_evaluation.best_combination_index = combination_index;
            combination.selected_best = true;
        }
    }

    evaluation = candidate_evaluation;
    return evaluation.has_best;
}

ExhaustiveManeuverCombinationEvaluator::
ExhaustiveManeuverCombinationEvaluator(
    const ManeuverCombinationEvaluatorParams & params)
: m_pair_evaluator(params),
  m_barrier_evaluator(params),
  m_positive_margin_filter_enabled(params.positive_margin_filter_enabled)
{
}

bool ExhaustiveManeuverCombinationEvaluator::evaluate(
    std::uint64_t evaluation_timestamp_us,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    std::size_t aircraft_count,
    ExhaustiveManeuverEvaluation & evaluation) const
{
    ExhaustiveManeuverEvaluation candidate_evaluation{};
    candidate_evaluation.evaluation_timestamp_us = evaluation_timestamp_us;
    candidate_evaluation.aircraft_count = aircraft_count;
    if (aircraft_count < 2 || aircraft_count > kMaximumSelectionAircraft) {
        evaluation = candidate_evaluation;
        return false;
    }

    std::array<PairCandidateCache, kMaximumSelectionPairCount> pair_caches{};
    const std::size_t pair_cache_count = initializePairCandidateCaches(
        evaluation_timestamp_us,
        candidate_sets,
        aircraft_count,
        kExhaustiveCandidatesPerAircraft,
        m_positive_margin_filter_enabled,
        m_barrier_evaluator,
        m_pair_evaluator,
        pair_caches,
        candidate_evaluation.evaluated_unique_pair_count);

    std::size_t combination_count = 1;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        combination_count *= kExhaustiveCandidatesPerAircraft;
    }
    candidate_evaluation.combination_count = combination_count;

    bool has_safe_combination = false;
    double best_safe_cost = std::numeric_limits<double>::infinity();
    double best_unsafe_minimum_ad = -std::numeric_limits<double>::infinity();
    constexpr double comparison_tolerance = 1.0e-12;

    for (std::size_t combination_index = 0;
         combination_index < combination_count; ++combination_index) {
        JointCombinationEvaluation combination{};
        combination.combination_index = combination_index;
        combination.aircraft_count = aircraft_count;
        combination.valid = true;
        combination.all_pairs_feasible = true;
        combination.minimum_ad_m = std::numeric_limits<double>::infinity();
        combination.minimum_pmr_m = std::numeric_limits<double>::infinity();
        combination.minimum_masd_m =
            std::numeric_limits<double>::quiet_NaN();
        combination.reciprocal_cost_sum = 0.0;

        std::size_t encoded = combination_index;
        for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
            const std::size_t reverse_aircraft = aircraft_count - 1 - aircraft;
            combination.candidate_slots[reverse_aircraft] =
                static_cast<std::uint8_t>(
                    encoded % kExhaustiveCandidatesPerAircraft);
            encoded /= kExhaustiveCandidatesPerAircraft;
        }

        if (m_positive_margin_filter_enabled) {
            std::array<
                const estimation::ReceivedTrajectoryIntent *,
                kMaximumSelectionAircraft> selected_intents{};
            for (std::size_t aircraft = 0;
                 aircraft < aircraft_count; ++aircraft) {
                selected_intents[aircraft] = &candidate_sets[aircraft][
                    combination.candidate_slots[aircraft]];
            }
            const auto provide_pair = [aircraft_count,
                                       &pair_caches,
                                       &combination](
                                          std::size_t first,
                                          std::size_t second,
                                          BarrierPairEvaluations & pair) {
                const std::size_t pair_index = pairCacheIndex(
                    first, second, aircraft_count);
                const PairCandidateCache & cache = pair_caches[pair_index];
                const std::size_t cache_index =
                    static_cast<std::size_t>(
                        combination.candidate_slots[first])
                        * kExhaustiveCandidatesPerAircraft
                    + combination.candidate_slots[second];
                pair.first_left = cache.first_left[cache_index];
                pair.first_right = cache.first_right[cache_index];
                pair.second_left = cache.second_left[cache_index];
                pair.second_right = cache.second_right[cache_index];
                return true;
            };
            if (!evaluateSelectedCombinationBarrier(
                    selected_intents,
                    aircraft_count,
                    provide_pair,
                    combination)) {
                continue;
            }
        }

        for (std::size_t pair_index = 0;
             pair_index < pair_cache_count; ++pair_index) {
            PairCandidateCache & cache = pair_caches[pair_index];
            const std::size_t first_candidate =
                combination.candidate_slots[cache.first_aircraft];
            const std::size_t second_candidate =
                combination.candidate_slots[cache.second_aircraft];
            const std::size_t cache_index =
                first_candidate * kExhaustiveCandidatesPerAircraft
                + second_candidate;
            if (!cache.ad_evaluated[cache_index]) {
                static_cast<void>(m_pair_evaluator.evaluatePair(
                    evaluation_timestamp_us,
                    candidate_sets[cache.first_aircraft][first_candidate],
                    candidate_sets[cache.second_aircraft][second_candidate],
                    cache.evaluations[cache_index]));
                cache.ad_evaluated[cache_index] = true;
                ++candidate_evaluation.evaluated_unique_pair_count;
            }
            const CombinationEvaluation & pair =
                cache.evaluations[cache_index];
            if (pair.validity != CombinationValidity::Valid) {
                combination.valid = false;
                combination.all_pairs_feasible = false;
                break;
            }
            ++combination.evaluated_pair_count;
            if (pair.ad_m < combination.minimum_ad_m) {
                combination.minimum_ad_m = pair.ad_m;
                combination.minimum_pmr_m = pair.pmr_m;
                combination.minimum_masd_m = pair.masd_m;
            }
            if (!pair.feasible || !pair.reciprocal_cost_defined) {
                combination.all_pairs_feasible = false;
            } else {
                combination.reciprocal_cost_sum += pair.reciprocal_cost;
            }
        }

        if (!combination.valid) {
            continue;
        }

        bool select = false;
        if (combination.all_pairs_feasible) {
            if (!has_safe_combination
                || combination.reciprocal_cost_sum
                    < best_safe_cost - comparison_tolerance) {
                select = true;
                has_safe_combination = true;
                best_safe_cost = combination.reciprocal_cost_sum;
            }
        } else if (!has_safe_combination
            && (!candidate_evaluation.has_best
                || combination.minimum_ad_m
                    > best_unsafe_minimum_ad + comparison_tolerance)) {
            select = true;
            best_unsafe_minimum_ad = combination.minimum_ad_m;
        }

        if (select) {
            candidate_evaluation.has_best = true;
            candidate_evaluation.best_combination_index = combination_index;
            candidate_evaluation.best_combination = combination;
        }
    }

    if (candidate_evaluation.has_best) {
        candidate_evaluation.best_combination.selected_best = true;
    }
    evaluation = candidate_evaluation;
    return evaluation.has_best;
}

const char * ManeuverCombinationEvaluator::validityName(
    CombinationValidity validity) noexcept
{
    switch (validity) {
    case CombinationValidity::Valid:
        return "VALID";
    case CombinationValidity::FutureTimestamp:
        return "FUTURE_TIME";
    case CombinationValidity::StaleTimestamp:
        return "STALE_TIME";
    case CombinationValidity::NoCommonHorizon:
        return "NO_OVERLAP";
    case CombinationValidity::InvalidTrajectory:
        return "INVALID_TRAJ";
    case CombinationValidity::BarrierRejected:
        return "BARRIER_REJECT";
    }
    return "UNKNOWN";
}

std::string ManeuverCombinationEvaluator::formatTable(
    const StaticCombinationEvaluation & evaluation)
{
    std::ostringstream stream;
    stream << "evaluation_timestamp_us=" << evaluation.evaluation_timestamp_us
           << '\n';
    stream << "idx  A  B  status        samples  PMR1     PMR2     PMR3     "
              "win  t_PMR  PMR      size    DSD     U95      MASD     AD       "
              "feasible  cost      best\n";
    stream << std::fixed << std::setprecision(3);

    for (const CombinationEvaluation & combination : evaluation.combinations) {
        stream << std::setw(3) << combination.combination_index << "  "
               << std::setw(2) << static_cast<unsigned>(
                      combination.ownship_candidate_id) << " "
               << std::setw(2) << static_cast<unsigned>(
                      combination.threat_candidate_id) << "  "
               << std::setw(12) << validityName(combination.validity) << "  ";
        if (combination.validity != CombinationValidity::Valid) {
            stream << "--       --       --       --       --   --     --       "
                      "--      --      --       --       --       --        --        "
                   << (combination.selected_best ? "YES" : "") << '\n';
            continue;
        }

        stream << std::setw(7) << combination.evaluated_sample_count << "  ";
        for (const PmrWindowResult & window : combination.pmr_windows) {
            if (window.available) {
                stream << std::setw(7) << window.pmr_m << "  ";
            } else {
                stream << std::setw(7) << "N/A" << "  ";
            }
        }
        stream << std::setw(3) << (combination.selected_pmr_window + 1) << "  "
               << std::setw(5) << combination.pmr_time_offset_s << "  "
               << std::setw(7) << combination.pmr_m << "  "
               << std::setw(6) << combination.aircraft_size_margin_m << "  "
               << std::setw(6) << combination.desired_separation_distance_m << "  "
               << std::setw(7) << combination.uncertainty_margin_95_m << "  "
               << std::setw(7) << combination.masd_m << "  "
               << std::setw(7) << combination.ad_m << "  "
               << std::setw(8) << (combination.feasible ? "YES" : "NO") << "  ";
        if (combination.reciprocal_cost_defined) {
            stream << std::setw(8) << combination.reciprocal_cost;
        } else {
            stream << std::setw(8) << "N/A";
        }
        stream << "  " << (combination.selected_best ? "YES" : "") << '\n';
    }
    return stream.str();
}

}  // namespace collision_avoidance::selection
