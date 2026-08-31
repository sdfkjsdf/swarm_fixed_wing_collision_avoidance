#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <collision_avoidance/formation/FormationDiscrimination.hpp>

namespace collision_avoidance::formation
{

class FormationDecisionCsvFormatter
{
public:
    [[nodiscard]] static std::string header();
    [[nodiscard]] static std::string row(
        const FormationResult & result,
        const FormationBoundaryConfig & config);
};

struct FormationReplaySample
{
    FormationUpdateInput input{};
    bool collision_activation_requested{false};
    bool has_expected_inhibit{false};
    bool expected_inhibit{false};
};

struct FormationReplayRecord
{
    FormationReplaySample sample{};
    FormationResult result{};
};

struct FormationMetricParams
{
    double boundary_range_tolerance_m{0.0};
    double boundary_closure_tolerance_mps{0.0};
};

struct FormationReplayMetrics
{
    double first_entry_time_s{-1.0};
    double first_exit_time_s{-1.0};
    double boundary_dwell_time_s{0.0};
    std::size_t state_toggle_count{0};
    std::size_t nuisance_activation_opportunities{0};
    std::size_t missed_inhibit_cases{0};
};

struct FormationProfileSweepResult
{
    std::string profile_name{};
    FormationProfileKind profile_kind{FormationProfileKind::UavCalibrated};
    bool config_valid{false};
    std::vector<FormationReplayRecord> records{};
    FormationReplayMetrics metrics{};
};

class FormationCalibrationHarness
{
public:
    [[nodiscard]] static std::vector<FormationReplayRecord> replay(
        const std::vector<FormationReplaySample> & samples,
        const FormationBoundaryConfig & config);
    [[nodiscard]] static FormationReplayMetrics metrics(
        const std::vector<FormationReplayRecord> & records,
        const FormationMetricParams & params) noexcept;
    [[nodiscard]] static std::vector<FormationProfileSweepResult> sweep(
        const std::vector<FormationReplaySample> & samples,
        const std::vector<FormationBoundaryConfig> & profiles,
        const FormationMetricParams & params);
};

}  // namespace collision_avoidance::formation
