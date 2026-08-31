#pragma once

#include <collision_avoidance/selection/BackupControlModelV4.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace collision_avoidance::selection
{

enum class BackupCertificationStatusV4 : std::uint8_t
{
    Valid,
    InvalidConfiguration,
    InvalidOwnshipState,
    InvalidThreatCount,
    InvalidThreatState,
    FutureThreatTimestamp,
    StaleThreatTimestamp,
    InvalidThreatTrajectory,
    PropagationFailed,
    DegenerateTerminalGeometry,
};

enum class BackupBranchFailureReasonV4 : std::uint8_t
{
    None,
    PathMarginViolation,
    TerminalTurnMarginViolation,
    PathAndTerminalMarginViolation,
};

enum class BackupBranchClassificationV4 : std::uint8_t
{
    BothCertified,
    LeftOnly,
    RightOnly,
    NeitherCertified,
};

struct BackupThreatPointV4
{
    double time_offset_s{0.0};
    double north_m{0.0};
    double east_m{0.0};
};

struct BackupThreatTrajectoryV4
{
    int vehicle_id{-1};
    std::uint64_t source_timestamp_us{0};
    double physical_clearance_m{0.0};
    std::array<
        BackupThreatPointV4,
        kMaximumBackupTrajectoryPointsV4> points{};
    std::size_t point_count{0};
};

struct BackupSafetyCertifierV4Params
{
    BackupControlModelV4Params model{};
    double horizon_s{4.5};
    double integration_step_s{0.1};
    double reference_margin_m{10.0};
    double certification_tolerance_m{1.0e-6};
    double threat_time_tolerance_s{1.0e-7};
};

struct BackupSafetyCertifierV4Input
{
    SafeControlOwnshipState ownship{};
    std::array<
        BackupThreatTrajectoryV4,
        kMaximumSafeControlThreats> threats{};
    std::size_t threat_count{0};
};

struct BackupBranchResultV4
{
    BackupDirectionV4 direction{BackupDirectionV4::Left};
    bool certified{false};
    BackupTrajectoryV4 trajectory{};
    double backup_heading_rate_initial_v4_radps{0.0};
    double minimum_path_margin_m{
        std::numeric_limits<double>::infinity()};
    double terminal_turn_margin_m{
        std::numeric_limits<double>::infinity()};
    int limiting_path_threat_id{-1};
    double limiting_path_time_offset_s{0.0};
    int limiting_terminal_threat_id{-1};
    BackupBranchFailureReasonV4 failure_reason{
        BackupBranchFailureReasonV4::None};
};

struct BackupSafetyCertifierV4Result
{
    BackupCertificationStatusV4 status{
        BackupCertificationStatusV4::InvalidConfiguration};
    BackupBranchClassificationV4 classification{
        BackupBranchClassificationV4::NeitherCertified};
    BackupBranchResultV4 left{};
    BackupBranchResultV4 right{};
    double effective_max_heading_rate_initial_radps{0.0};
    std::size_t evaluated_threat_count{0};
    int invalid_threat_vehicle_id{-1};
    BackupPropagationStatusV4 propagation_status{
        BackupPropagationStatusV4::Valid};
};

class BackupSafetyCertifierV4
{
public:
    explicit BackupSafetyCertifierV4(
        const BackupSafetyCertifierV4Params & params = {});

    const BackupSafetyCertifierV4Params & params() const noexcept;

    BackupSafetyCertifierV4Result evaluate(
        const BackupSafetyCertifierV4Input & input) const noexcept;

    static bool validParams(
        const BackupSafetyCertifierV4Params & params) noexcept;

private:
    BackupSafetyCertifierV4Params m_params;
    BackupControlModelV4 m_model;
};

const char * backupCertificationStatusName(
    BackupCertificationStatusV4 status) noexcept;

const char * backupBranchFailureReasonName(
    BackupBranchFailureReasonV4 reason) noexcept;

}  // namespace collision_avoidance::selection
