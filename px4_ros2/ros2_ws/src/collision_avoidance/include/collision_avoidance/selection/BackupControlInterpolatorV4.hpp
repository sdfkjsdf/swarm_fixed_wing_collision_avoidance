#pragma once

#include <collision_avoidance/selection/BackupSafetyCertifierV4.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kMaximumBackupInterpolationConstraintsV4 =
    kMaximumSafeControlThreats
    * (kMaximumBackupTrajectoryPointsV4 + 1);

enum class BackupInterpolationStatusV4 : std::uint8_t
{
    Valid,
    InvalidConfiguration,
    InvalidNominalRate,
    CertificationFailed,
    NoCertifiedBranch,
    InterpolationInfeasible,
    NumericalFailure,
};

enum class BackupInterpolationBranchStatusV4 : std::uint8_t
{
    NotCertified,
    Feasible,
    Infeasible,
    NumericalFailure,
};

enum class BackupInterpolationConstraintKindV4 : std::uint8_t
{
    PathSeparation,
    TerminalTurnCertificate,
};

struct BackupInterpolationTolerancesV4
{
    double coefficient_mps{1.0e-9};
    double residual_mps{1.0e-9};
    double mu{1.0e-9};
    double distance_m{1.0e-9};
};

struct BackupControlInterpolatorV4Params
{
    BackupSafetyCertifierV4Params certifier{};
    double path_alpha_gain_per_s{0.2};
    double terminal_alpha_gain_per_s{0.5};
    BackupInterpolationTolerancesV4 tolerances{};
};

struct ScalarMuIntervalV4
{
    double lower{0.0};
    double upper{1.0};
    bool feasible{true};
};

struct BackupInterpolationConstraintDiagnosticV4
{
    BackupInterpolationConstraintKindV4 kind{
        BackupInterpolationConstraintKindV4::PathSeparation};
    int vehicle_id{-1};
    double time_offset_s{0.0};
    double margin_m{0.0};
    double hdot_nominal_mps{0.0};
    double hdot_backup_mps{0.0};
    double a_mps{0.0};
    double b_mps{0.0};
    double imposed_mu_bound{0.0};
};

struct BackupInterpolationBranchResultV4
{
    BackupDirectionV4 direction{BackupDirectionV4::Left};
    BackupInterpolationBranchStatusV4 status{
        BackupInterpolationBranchStatusV4::NotCertified};
    ScalarMuIntervalV4 mu_interval{};
    double mu_star{std::numeric_limits<double>::quiet_NaN()};
    double nominal_heading_rate_v4_radps{0.0};
    double backup_heading_rate_v4_radps{0.0};
    double safe_heading_rate_v4_radps{
        std::numeric_limits<double>::quiet_NaN()};
    double minimum_residual_at_mu_star_mps{
        std::numeric_limits<double>::infinity()};
    std::size_t constraint_count{0};
    bool lower_bound_diagnostic_valid{false};
    BackupInterpolationConstraintDiagnosticV4 lower_bound_diagnostic{};
    bool upper_bound_diagnostic_valid{false};
    BackupInterpolationConstraintDiagnosticV4 upper_bound_diagnostic{};
    bool infeasible_diagnostic_valid{false};
    BackupInterpolationConstraintDiagnosticV4 infeasible_diagnostic{};
};

struct BackupControlInterpolatorV4Result
{
    BackupInterpolationStatusV4 status{
        BackupInterpolationStatusV4::InvalidConfiguration};
    BackupSafetyCertifierV4Result certification{};
    BackupInterpolationBranchResultV4 left{};
    BackupInterpolationBranchResultV4 right{};
    std::size_t certified_branch_count{0};
    std::size_t feasible_branch_count{0};
};

class BackupControlInterpolatorV4
{
public:
    explicit BackupControlInterpolatorV4(
        const BackupControlInterpolatorV4Params & params = {});

    const BackupControlInterpolatorV4Params & params() const noexcept;

    BackupControlInterpolatorV4Result evaluate(
        const BackupSafetyCertifierV4Input & input,
        double nominal_heading_rate_v4_radps) const noexcept;

    static bool validParams(
        const BackupControlInterpolatorV4Params & params) noexcept;

    static bool applyScalarConstraint(
        double a_mps,
        double b_mps,
        const BackupInterpolationTolerancesV4 & tolerances,
        ScalarMuIntervalV4 & interval) noexcept;

private:
    BackupControlInterpolatorV4Params m_params;
    BackupSafetyCertifierV4 m_certifier;
    BackupControlModelV4 m_model;
};

const char * backupInterpolationStatusName(
    BackupInterpolationStatusV4 status) noexcept;

const char * backupInterpolationBranchStatusName(
    BackupInterpolationBranchStatusV4 status) noexcept;

}  // namespace collision_avoidance::selection
