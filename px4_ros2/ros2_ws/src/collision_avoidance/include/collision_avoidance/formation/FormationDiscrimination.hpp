#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace collision_avoidance::formation
{

enum class FormationState : std::uint8_t
{
    OutsideFormation = 0,
    FormationDeactivationZone,
    Standby,
};

enum class FormationDecisionReason : std::uint8_t
{
    None = 0,
    InsideNominalFdz,
    InsideNominalStandby,
    RetainedInFdzByExitBoundary,
    RetainedInStandbyByExitBoundary,
    OutsideNominalFormationBoundary,
    OutsideRelaxedFormationBoundary,
    InvalidConfiguration,
    InvalidKinematicState,
    InvalidTimestamp,
    BoundaryLookupFailed,
};

enum class FormationProfileKind : std::uint8_t
{
    SourceReference = 0,
    UavCalibrated,
};

enum class FormationLookupPolicy : std::uint8_t
{
    RejectOutsideTable = 0,
    ClampToEndpoint,
};

struct FormationBoundaryNode
{
    double range_m{0.0};
    double closure_upper_mps{0.0};
};

struct FormationBoundaryConfig
{
    std::string profile_name{};
    FormationProfileKind profile_kind{FormationProfileKind::UavCalibrated};

    // Source-relation metadata. Profiles choose the calibrated coefficients;
    // the classifier never embeds aircraft-specific dimensions.
    double representative_wingspan_m{0.0};
    double range0_wingspan_scale{0.0};
    double uncertainty_margin_m{0.0};
    double range1_offset_m{0.0};

    std::vector<FormationBoundaryNode> closure_upper_entry_table{};
    std::vector<FormationBoundaryNode> closure_upper_exit_table{};
    double closure_lower_entry_mps{0.0};
    double closure_lower_exit_mps{0.0};
    double fdz_entry_limit_m{0.0};
    double fdz_exit_limit_m{0.0};
    double max_range_entry_m{0.0};
    double max_range_exit_m{0.0};

    // Track validity belongs to the profile rather than to algorithm magic
    // numbers, because telemetry rate and synchronization quality are system
    // dependent.
    double maximum_state_age_s{0.0};
    double maximum_future_skew_s{0.0};
    double maximum_timestamp_skew_s{0.0};
    double numerical_range_epsilon_m{1.0e-9};
    FormationLookupPolicy lookup_policy{
        FormationLookupPolicy::RejectOutsideTable};

    [[nodiscard]] double range0_m() const noexcept;
    [[nodiscard]] double minimum_distance_m() const noexcept;
    [[nodiscard]] double range1_m() const noexcept;
};

struct FormationKinematicState
{
    std::array<double, 3> position_ned_m{};
    std::array<double, 3> velocity_ned_mps{};
    double timestamp_s{0.0};
    bool valid{false};
};

struct FormationUpdateInput
{
    std::uint32_t threat_id{0};
    double evaluation_timestamp_s{0.0};
    FormationKinematicState ownship{};
    FormationKinematicState threat{};
};

struct FormationBoundaryLookup
{
    bool valid{false};
    bool clamped{false};
    double closure_upper_mps{0.0};
};

struct FormationResult
{
    std::uint32_t threat_id{0};
    FormationState previous_state{FormationState::OutsideFormation};
    FormationState state{FormationState::OutsideFormation};
    FormationDecisionReason reason{FormationDecisionReason::None};
    bool formation_inhibit{false};
    bool entered_formation{false};
    bool retained_by_hysteresis{false};
    bool exited_formation{false};
    bool timestamp_valid{false};
    bool geometry_valid{false};
    bool boundary_lookup_valid{false};

    double evaluation_timestamp_s{0.0};
    double state_age_s{0.0};
    double timestamp_skew_s{0.0};
    double range_m{0.0};
    double range_rate_mps{0.0};
    double closure_rate_mps{0.0};
    double relative_bearing_rad{0.0};
    double relative_altitude_m{0.0};
    double ownship_speed_mps{0.0};
    double threat_speed_mps{0.0};

    double entry_closure_limit_mps{0.0};
    double exit_closure_limit_mps{0.0};
    double closure_lower_entry_mps{0.0};
    double closure_lower_exit_mps{0.0};
    double fdz_entry_limit_m{0.0};
    double fdz_exit_limit_m{0.0};
    double max_range_entry_m{0.0};
    double max_range_exit_m{0.0};
};

class FormationDiscriminator
{
public:
    explicit FormationDiscriminator(FormationBoundaryConfig config);

    [[nodiscard]] FormationResult update(
        const FormationUpdateInput & input) noexcept;
    [[nodiscard]] FormationState state(std::uint32_t threat_id) const noexcept;
    void resetThreat(std::uint32_t threat_id) noexcept;
    void reset() noexcept;

    [[nodiscard]] const FormationBoundaryConfig & config() const noexcept;
    [[nodiscard]] static bool validConfig(
        const FormationBoundaryConfig & config) noexcept;
    [[nodiscard]] static FormationBoundaryLookup closureUpperLimit(
        const std::vector<FormationBoundaryNode> & table,
        double range_m,
        FormationLookupPolicy policy) noexcept;
    [[nodiscard]] static double closureFromRangeRate(
        double range_rate_mps) noexcept;

private:
    [[nodiscard]] FormationResult classify(
        const FormationUpdateInput & input,
        FormationState previous_state) const noexcept;

    FormationBoundaryConfig m_config;
    bool m_config_valid{false};
    std::unordered_map<std::uint32_t, FormationState> m_states;
};

enum class FormationAggregationPolicy : std::uint8_t
{
    // No global gate is asserted. The collision layer may exempt only the
    // individually classified threats using per_threat_results.
    PerThreatExemptionOnly = 0,
    // Global inhibit requires every relevant threat to be valid formation.
    AllRelevantThreatsFormation,
    // Explicit reconstruction option; use only when system safety analysis
    // accepts one formation threat inhibiting all new activations.
    AnyRelevantThreatFormation,
};

struct FormationAggregationResult
{
    FormationAggregationPolicy policy{
        FormationAggregationPolicy::PerThreatExemptionOnly};
    bool formation_inhibit{false};
    bool allow_new_activation{true};
    bool complete_collision_context{false};
    std::size_t relevant_threat_count{0};
    std::size_t matched_result_count{0};
    std::size_t inhibited_threat_count{0};
    std::vector<std::uint32_t> inhibited_threat_ids{};
};

class FormationInhibitAggregator
{
public:
    [[nodiscard]] static FormationAggregationResult aggregate(
        const std::vector<FormationResult> & per_threat_results,
        const std::vector<std::uint32_t> & collision_relevant_threat_ids,
        FormationAggregationPolicy policy) noexcept;
};

[[nodiscard]] const char * toString(FormationState state) noexcept;
[[nodiscard]] const char * toString(FormationDecisionReason reason) noexcept;
[[nodiscard]] const char * toString(FormationProfileKind kind) noexcept;
[[nodiscard]] const char * toString(FormationAggregationPolicy policy) noexcept;

}  // namespace collision_avoidance::formation
