#pragma once

#include <collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp>
#include <collision_avoidance/selection/SafeControlSetV4.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kMaximumSafeControlCandidates = 3;

enum class SafeCandidateRole : std::uint8_t
{
    NearNominal = 0,
    RobustLeft = 1,
    RobustRight = 2,
};

enum class SafeControlCandidateAdapterStatus : std::uint8_t
{
    Valid,
    SearchSetInfeasible,
    InvalidConfiguration,
    InvalidSafeSet,
    InvalidAirspeed,
    InvalidPredictorCommand,
    InvalidNominalRate,
};

struct SafeControlCandidateAdapterParams
{
    // Robustness guard applied inside the aggressive interval endpoint.
    double robustness_guard_radps{0.008726646259971648};  // 0.5 deg/s
    // Candidate rates closer than this value are treated as duplicates.
    double duplicate_tolerance_radps{1.0e-7};
    // Positive speed/division guard for sign conversion.
    double speed_tolerance_mps{1.0e-3};
};

struct SafeControlCandidateAdapterInput
{
    SafeControlSetV4Result safe_set{};
    double true_airspeed_mps{0.0};
    bool nominal_rate_available{false};
    // V4 convention: positive is Left and negative is Right.
    double nominal_heading_rate_v4_radps{0.0};
    // Existing ground-kinematic predictor commands retained by each candidate.
    double ground_speed_command_mps{0.0};
    double altitude_command_m{std::numeric_limits<double>::quiet_NaN()};
};

struct SafeControlCandidate
{
    SafeCandidateRole role{SafeCandidateRole::NearNominal};
    double heading_rate_v4_radps{0.0};
    estimation::PredictInput predictor_input{};
};

struct SafeControlCandidateAdapterResult
{
    SafeControlCandidateAdapterStatus status{
        SafeControlCandidateAdapterStatus::InvalidConfiguration};
    std::array<
        SafeControlCandidate,
        kMaximumSafeControlCandidates> candidates{};
    std::size_t candidate_count{0};
};

class SafeControlCandidateAdapter
{
public:
    explicit SafeControlCandidateAdapter(
        const SafeControlCandidateAdapterParams & params = {});

    const SafeControlCandidateAdapterParams & params() const noexcept;

    SafeControlCandidateAdapterResult generate(
        const SafeControlCandidateAdapterInput & input) const noexcept;

    static bool validParams(
        const SafeControlCandidateAdapterParams & params) noexcept;

    static double v4HeadingRateToPx4LateralAcceleration(
        double true_airspeed_mps,
        double heading_rate_v4_radps,
        double speed_tolerance_mps = 1.0e-3) noexcept;

    static double px4LateralAccelerationToV4HeadingRate(
        double true_airspeed_mps,
        double lateral_acceleration_px4_mps2,
        double speed_tolerance_mps = 1.0e-3) noexcept;

private:
    SafeControlCandidateAdapterParams m_params;
};

const char * safeControlCandidateAdapterStatusName(
    SafeControlCandidateAdapterStatus status) noexcept;

}  // namespace collision_avoidance::selection
