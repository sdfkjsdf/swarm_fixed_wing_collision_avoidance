#pragma once

#include <collision_avoidance/selection/SafeControlSetV4.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kMaximumBackupTrajectoryPointsV4 = 128;

enum class BackupDirectionV4 : std::uint8_t
{
    Left,
    Right,
};

enum class BackupPropagationStatusV4 : std::uint8_t
{
    Valid,
    InvalidConfiguration,
    InvalidInitialState,
    InvalidAirspeed,
    TrajectoryCapacityExceeded,
    NumericalFailure,
};

struct BackupControlModelV4Params
{
    double gravity_mps2{9.80665};
    double maximum_roll_rad{0.8726646259971648};  // 50 deg
    double maximum_yaw_rate_radps{0.8726646259971648};  // 50 deg/s
    double speed_tolerance_mps{1.0e-3};
    double heading_rate_tolerance_radps{1.0e-9};
    double time_tolerance_s{1.0e-9};
};

struct BackupTrajectoryPointV4
{
    double time_offset_s{0.0};
    double north_m{0.0};
    double east_m{0.0};
    double heading_ned_rad{0.0};
    double true_airspeed_mps{0.0};
    // V4 convention: positive is LEFT and negative is RIGHT.
    double backup_heading_rate_v4_radps{0.0};
};

struct BackupTrajectoryV4
{
    BackupPropagationStatusV4 status{
        BackupPropagationStatusV4::InvalidConfiguration};
    BackupDirectionV4 direction{BackupDirectionV4::Left};
    std::array<
        BackupTrajectoryPointV4,
        kMaximumBackupTrajectoryPointsV4> points{};
    std::size_t point_count{0};
};

class BackupControlModelV4
{
public:
    explicit BackupControlModelV4(
        const BackupControlModelV4Params & params = {});

    const BackupControlModelV4Params & params() const noexcept;

    double effectiveMaxHeadingRate(
        double true_airspeed_mps) const noexcept;

    double backupHeadingRate(
        BackupDirectionV4 direction,
        double true_airspeed_mps) const noexcept;

    BackupTrajectoryV4 propagate(
        const SafeControlOwnshipState & initial_state,
        BackupDirectionV4 direction,
        double horizon_s,
        double integration_step_s) const noexcept;

    static bool validParams(
        const BackupControlModelV4Params & params) noexcept;

private:
    BackupControlModelV4Params m_params;
};

const char * backupPropagationStatusName(
    BackupPropagationStatusV4 status) noexcept;

}  // namespace collision_avoidance::selection
