#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kMaximumSafeControlThreats = 4;
inline constexpr std::size_t kSafeControlDirectionCount = 2;
inline constexpr std::size_t kMaximumSafeControlDiagnostics =
    kMaximumSafeControlThreats * kSafeControlDirectionCount;

enum class SafeControlDirection : std::uint8_t
{
    Left,
    Right,
};

enum class SafeControlSetStatus : std::uint8_t
{
    Valid,
    InvalidConfiguration,
    InvalidOwnshipState,
    InvalidAirspeed,
    InvalidThreatCount,
    InvalidThreatState,
    FutureThreatTimestamp,
    StaleThreatTimestamp,
    DegenerateGeometry,
    SearchSetInfeasible,
};

enum class LongitudinalDriftSource : std::uint8_t
{
    ValidatedExternal,
    LocalOneStepFreeze,
};

struct SafeControlNumericalTolerances
{
    // Tolerance on the affine barrier residual [m/s].
    double constraint_mps{1.0e-5};
    // Tolerance used when deciding whether a rate interval is empty [rad/s].
    double interval_radps{1.0e-7};
    // Positive airspeed/division guard [m/s].
    double speed_mps{1.0e-3};
    // Minimum resolvable q norm and scalar rate coefficient [m].
    double direction_m{1.0e-4};
};

struct SafeControlSetV4Params
{
    double margin_reference_m{10.0};
    double margin_time_constant_s{5.0};
    double control_period_s{0.05};
    double gravity_mps2{9.80665};
    double maximum_roll_rad{0.8726646259971648};  // 50 deg
    double maximum_yaw_rate_radps{0.8726646259971648};  // 50 deg/s
    SafeControlNumericalTolerances tolerances{};
};

struct SafeControlOwnshipState
{
    std::uint64_t timestamp_us{0};
    double north_m{0.0};
    double east_m{0.0};
    // NED course convention: atan2(E, N), positive toward Right. Under the
    // baseline no-wind assumption this is also the air-relative heading.
    double heading_ned_rad{0.0};
    double true_airspeed_mps{0.0};
    double longitudinal_acceleration_mps2{0.0};
    LongitudinalDriftSource longitudinal_source{
        LongitudinalDriftSource::LocalOneStepFreeze};
};

struct SafeControlThreatState
{
    int vehicle_id{-1};
    // The adapter must align this state to the ownship evaluation timestamp.
    std::uint64_t timestamp_us{0};
    double north_m{0.0};
    double east_m{0.0};
    double velocity_north_mps{0.0};
    double velocity_east_mps{0.0};
    // Baseline C_j: ownship physical radius + this threat physical radius [m].
    double physical_clearance_m{0.0};
};

struct SafeControlSetV4Input
{
    SafeControlOwnshipState ownship{};
    std::array<SafeControlThreatState, kMaximumSafeControlThreats> threats{};
    std::size_t threat_count{0};
};

struct HeadingRateInterval
{
    double lower_radps{0.0};
    double upper_radps{0.0};
    bool feasible{false};
};

struct SafeControlThreatDiagnostic
{
    int vehicle_id{-1};
    SafeControlDirection direction{SafeControlDirection::Left};
    double clearance_m{0.0};
    double drift_mps{0.0};
    double rate_coefficient_m{0.0};
    double required_control_term_mps{0.0};
    double imposed_bound_radps{0.0};
    // Positive amount by which the current physical/intersected interval
    // cannot meet the affine constraint [m/s].
    double constraint_shortfall_mps{0.0};
    bool constraint_degenerate{false};
    bool constraint_feasible{false};
};

struct SafeControlSetV4Result
{
    SafeControlSetStatus status{SafeControlSetStatus::InvalidConfiguration};
    LongitudinalDriftSource longitudinal_source{
        LongitudinalDriftSource::LocalOneStepFreeze};
    HeadingRateInterval left_safe{};
    HeadingRateInterval right_safe{};
    double effective_max_heading_rate_radps{0.0};
    double kappa_per_s{0.0};
    double gamma_diagnostic{0.0};
    std::size_t evaluated_threat_count{0};
    std::array<
        SafeControlThreatDiagnostic,
        kMaximumSafeControlDiagnostics> diagnostics{};
    std::size_t diagnostic_count{0};
    int first_infeasible_vehicle_id{-1};
    SafeControlDirection first_infeasible_direction{
        SafeControlDirection::Left};
};

class SafeControlSetV4
{
public:
    explicit SafeControlSetV4(
        const SafeControlSetV4Params & params = {});

    const SafeControlSetV4Params & params() const noexcept;

    SafeControlSetV4Result evaluate(
        const SafeControlSetV4Input & input) const noexcept;

    static bool validParams(
        const SafeControlSetV4Params & params) noexcept;

private:
    SafeControlSetV4Params m_params;
};

const char * safeControlSetStatusName(
    SafeControlSetStatus status) noexcept;

}  // namespace collision_avoidance::selection
