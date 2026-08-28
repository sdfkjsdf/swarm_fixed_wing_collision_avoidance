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

struct AlignedConeSample
{
    std::array<double, 3> position_ned{};
    estimation::PositionCovariance position_covariance_ned{};
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
        && params.stale_timeout_s > 0.0;
}

bool finiteConePoint(const estimation::TrajectoryConePoint & point) noexcept
{
    const auto & mean = point.mean;
    if (!std::isfinite(mean.p_n) || !std::isfinite(mean.p_e)
        || !std::isfinite(mean.h)) {
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
    for (std::size_t element = 0;
         element < sample.position_covariance_ned.size(); ++element) {
        sample.position_covariance_ned[element] =
            (1.0 - alpha) * lower_point.position_covariance_ned[element]
            + alpha * upper_point.position_covariance_ned[element];
    }
    return true;
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

CombinationValidity evaluatePair(
    std::uint64_t evaluation_timestamp_us,
    const estimation::ReceivedTrajectoryIntent & ownship_intent,
    const estimation::ReceivedTrajectoryIntent & threat_intent,
    const ManeuverCombinationEvaluatorParams & params,
    CombinationEvaluation & result) noexcept
{
    double ownship_age_s = 0.0;
    double threat_age_s = 0.0;
    const CombinationValidity ownship_validity = timestampValidity(
        ownship_intent,
        evaluation_timestamp_us,
        params.stale_timeout_s,
        ownship_age_s);
    if (ownship_validity != CombinationValidity::Valid) {
        return ownship_validity;
    }
    const CombinationValidity threat_validity = timestampValidity(
        threat_intent,
        evaluation_timestamp_us,
        params.stale_timeout_s,
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

    const std::size_t sample_count = std::min(
        estimation::kTrajectoryPointCount,
        static_cast<std::size_t>(
            std::floor((common_horizon_s + kTimeTolerance) / kStepSeconds)) + 1);
    if (sample_count == 0) {
        return CombinationValidity::NoCommonHorizon;
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

ManeuverCombinationEvaluator::ManeuverCombinationEvaluator(
    const ManeuverCombinationEvaluatorParams & params)
: m_params(params)
{
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
            combination.validity = evaluatePair(
                evaluation_timestamp_us,
                ownship_candidates[ownship_index],
                threat_candidates[threat_index],
                m_params,
                combination);

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
