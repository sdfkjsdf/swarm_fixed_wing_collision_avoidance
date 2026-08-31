#include <collision_avoidance/formation/FormationDiscrimination.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace collision_avoidance::formation
{
namespace
{

bool finiteVector(const std::array<double, 3> & value) noexcept
{
    return std::all_of(value.begin(), value.end(), [](const double item) {
        return std::isfinite(item);
    });
}

double norm(const std::array<double, 3> & value) noexcept
{
    return std::sqrt(
        value[0] * value[0] + value[1] * value[1] +
        value[2] * value[2]);
}

bool finiteNonnegative(const double value) noexcept
{
    return std::isfinite(value) && value >= 0.0;
}

bool validTable(const std::vector<FormationBoundaryNode> & table) noexcept
{
    if (table.empty()) {
        return false;
    }
    for (std::size_t index = 0; index < table.size(); ++index) {
        const auto & node = table[index];
        if (!finiteNonnegative(node.range_m) ||
            !std::isfinite(node.closure_upper_mps)) {
            return false;
        }
        if (index > 0 && !(node.range_m > table[index - 1].range_m)) {
            return false;
        }
    }
    return true;
}

}  // namespace

double FormationBoundaryConfig::range0_m() const noexcept
{
    return representative_wingspan_m * range0_wingspan_scale;
}

double FormationBoundaryConfig::minimum_distance_m() const noexcept
{
    return range0_m() + uncertainty_margin_m;
}

double FormationBoundaryConfig::range1_m() const noexcept
{
    return range0_m() + range1_offset_m;
}

FormationDiscriminator::FormationDiscriminator(FormationBoundaryConfig config)
: m_config(std::move(config)), m_config_valid(validConfig(m_config))
{
}

FormationResult FormationDiscriminator::update(
    const FormationUpdateInput & input) noexcept
{
    const auto iterator = m_states.find(input.threat_id);
    const auto previous = iterator == m_states.end()
        ? FormationState::OutsideFormation
        : iterator->second;
    auto result = classify(input, previous);
    m_states[input.threat_id] = result.state;
    return result;
}

FormationState FormationDiscriminator::state(
    const std::uint32_t threat_id) const noexcept
{
    const auto iterator = m_states.find(threat_id);
    return iterator == m_states.end()
        ? FormationState::OutsideFormation
        : iterator->second;
}

void FormationDiscriminator::resetThreat(const std::uint32_t threat_id) noexcept
{
    m_states.erase(threat_id);
}

void FormationDiscriminator::reset() noexcept
{
    m_states.clear();
}

const FormationBoundaryConfig & FormationDiscriminator::config() const noexcept
{
    return m_config;
}

bool FormationDiscriminator::validConfig(
    const FormationBoundaryConfig & config) noexcept
{
    if (config.profile_name.empty() ||
        !finiteNonnegative(config.representative_wingspan_m) ||
        !finiteNonnegative(config.range0_wingspan_scale) ||
        !finiteNonnegative(config.uncertainty_margin_m) ||
        !finiteNonnegative(config.range1_offset_m) ||
        !validTable(config.closure_upper_entry_table) ||
        !validTable(config.closure_upper_exit_table) ||
        !std::isfinite(config.closure_lower_entry_mps) ||
        !std::isfinite(config.closure_lower_exit_mps) ||
        !finiteNonnegative(config.fdz_entry_limit_m) ||
        !finiteNonnegative(config.fdz_exit_limit_m) ||
        !finiteNonnegative(config.max_range_entry_m) ||
        !finiteNonnegative(config.max_range_exit_m) ||
        !finiteNonnegative(config.maximum_state_age_s) ||
        !finiteNonnegative(config.maximum_future_skew_s) ||
        !finiteNonnegative(config.maximum_timestamp_skew_s) ||
        !std::isfinite(config.numerical_range_epsilon_m) ||
        config.numerical_range_epsilon_m <= 0.0) {
        return false;
    }
    if (config.fdz_exit_limit_m < config.fdz_entry_limit_m ||
        config.max_range_exit_m < config.max_range_entry_m) {
        return false;
    }
    return true;
}

FormationBoundaryLookup FormationDiscriminator::closureUpperLimit(
    const std::vector<FormationBoundaryNode> & table,
    const double range_m,
    const FormationLookupPolicy policy) noexcept
{
    FormationBoundaryLookup result{};
    if (!validTable(table) || !std::isfinite(range_m)) {
        return result;
    }

    if (range_m < table.front().range_m) {
        if (policy == FormationLookupPolicy::RejectOutsideTable) {
            return result;
        }
        result.valid = true;
        result.clamped = true;
        result.closure_upper_mps = table.front().closure_upper_mps;
        return result;
    }
    if (range_m > table.back().range_m) {
        if (policy == FormationLookupPolicy::RejectOutsideTable) {
            return result;
        }
        result.valid = true;
        result.clamped = true;
        result.closure_upper_mps = table.back().closure_upper_mps;
        return result;
    }

    const auto upper = std::lower_bound(
        table.begin(), table.end(), range_m,
        [](const FormationBoundaryNode & node, const double range) {
            return node.range_m < range;
        });
    if (upper == table.begin()) {
        result.valid = true;
        result.closure_upper_mps = upper->closure_upper_mps;
        return result;
    }
    if (upper == table.end()) {
        result.valid = true;
        result.closure_upper_mps = table.back().closure_upper_mps;
        return result;
    }

    const auto lower = std::prev(upper);
    const double fraction =
        (range_m - lower->range_m) / (upper->range_m - lower->range_m);
    result.valid = true;
    result.closure_upper_mps = lower->closure_upper_mps +
        fraction * (upper->closure_upper_mps - lower->closure_upper_mps);
    return result;
}

double FormationDiscriminator::closureFromRangeRate(
    const double range_rate_mps) noexcept
{
    return -range_rate_mps;
}

FormationResult FormationDiscriminator::classify(
    const FormationUpdateInput & input,
    const FormationState previous_state) const noexcept
{
    FormationResult result{};
    result.threat_id = input.threat_id;
    result.previous_state = previous_state;
    result.evaluation_timestamp_s = input.evaluation_timestamp_s;
    result.fdz_entry_limit_m = m_config.fdz_entry_limit_m;
    result.fdz_exit_limit_m = m_config.fdz_exit_limit_m;
    result.max_range_entry_m = m_config.max_range_entry_m;
    result.max_range_exit_m = m_config.max_range_exit_m;
    result.closure_lower_entry_mps = m_config.closure_lower_entry_mps;
    result.closure_lower_exit_mps = m_config.closure_lower_exit_mps;

    const auto fail_open = [&](const FormationDecisionReason reason) {
        result.state = FormationState::OutsideFormation;
        result.reason = reason;
        result.formation_inhibit = false;
        result.exited_formation =
            previous_state != FormationState::OutsideFormation;
        return result;
    };

    if (!m_config_valid) {
        return fail_open(FormationDecisionReason::InvalidConfiguration);
    }
    if (!input.ownship.valid || !input.threat.valid ||
        !std::isfinite(input.evaluation_timestamp_s) ||
        !std::isfinite(input.ownship.timestamp_s) ||
        !std::isfinite(input.threat.timestamp_s) ||
        !finiteVector(input.ownship.position_ned_m) ||
        !finiteVector(input.ownship.velocity_ned_mps) ||
        !finiteVector(input.threat.position_ned_m) ||
        !finiteVector(input.threat.velocity_ned_mps)) {
        return fail_open(FormationDecisionReason::InvalidKinematicState);
    }

    const double ownship_age_s =
        input.evaluation_timestamp_s - input.ownship.timestamp_s;
    const double threat_age_s =
        input.evaluation_timestamp_s - input.threat.timestamp_s;
    result.state_age_s = std::max(ownship_age_s, threat_age_s);
    result.timestamp_skew_s = std::abs(
        input.ownship.timestamp_s - input.threat.timestamp_s);
    result.timestamp_valid =
        ownship_age_s >= -m_config.maximum_future_skew_s &&
        threat_age_s >= -m_config.maximum_future_skew_s &&
        ownship_age_s <= m_config.maximum_state_age_s &&
        threat_age_s <= m_config.maximum_state_age_s &&
        result.timestamp_skew_s <= m_config.maximum_timestamp_skew_s;
    if (!result.timestamp_valid) {
        return fail_open(FormationDecisionReason::InvalidTimestamp);
    }

    std::array<double, 3> relative_position{};
    std::array<double, 3> relative_velocity{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        relative_position[axis] = input.threat.position_ned_m[axis] -
            input.ownship.position_ned_m[axis];
        relative_velocity[axis] = input.threat.velocity_ned_mps[axis] -
            input.ownship.velocity_ned_mps[axis];
    }
    result.range_m = norm(relative_position);
    result.ownship_speed_mps = norm(input.ownship.velocity_ned_mps);
    result.threat_speed_mps = norm(input.threat.velocity_ned_mps);
    result.relative_bearing_rad = std::atan2(
        relative_position[1], relative_position[0]);
    result.relative_altitude_m = -relative_position[2];
    if (!std::isfinite(result.range_m) ||
        result.range_m <= m_config.numerical_range_epsilon_m) {
        // A co-located threat has no defined radial direction. Treating it as
        // formation could mask collision logic, so invalid geometry fails open.
        return fail_open(FormationDecisionReason::InvalidKinematicState);
    }
    const double radial_dot =
        relative_position[0] * relative_velocity[0] +
        relative_position[1] * relative_velocity[1] +
        relative_position[2] * relative_velocity[2];
    result.range_rate_mps = radial_dot / result.range_m;
    result.closure_rate_mps = closureFromRangeRate(result.range_rate_mps);
    result.geometry_valid = std::isfinite(result.range_rate_mps);
    if (!result.geometry_valid) {
        return fail_open(FormationDecisionReason::InvalidKinematicState);
    }

    const auto entry_lookup = closureUpperLimit(
        m_config.closure_upper_entry_table,
        result.range_m,
        m_config.lookup_policy);
    const auto exit_lookup = closureUpperLimit(
        m_config.closure_upper_exit_table,
        result.range_m,
        m_config.lookup_policy);
    result.boundary_lookup_valid = entry_lookup.valid && exit_lookup.valid;
    result.entry_closure_limit_mps = entry_lookup.valid
        ? entry_lookup.closure_upper_mps
        : std::numeric_limits<double>::quiet_NaN();
    result.exit_closure_limit_mps = exit_lookup.valid
        ? exit_lookup.closure_upper_mps
        : std::numeric_limits<double>::quiet_NaN();

    const bool nominal_fdz = result.range_m <= m_config.fdz_entry_limit_m;
    const bool relaxed_fdz = result.range_m <= m_config.fdz_exit_limit_m;
    const bool nominal_standby = entry_lookup.valid &&
        result.range_m <= m_config.max_range_entry_m &&
        result.closure_rate_mps >= m_config.closure_lower_entry_mps &&
        result.closure_rate_mps <= entry_lookup.closure_upper_mps;
    const bool relaxed_standby = exit_lookup.valid &&
        result.range_m <= m_config.max_range_exit_m &&
        result.closure_rate_mps >= m_config.closure_lower_exit_mps &&
        result.closure_rate_mps <= exit_lookup.closure_upper_mps;

    switch (previous_state) {
    case FormationState::OutsideFormation:
        if (nominal_fdz) {
            result.state = FormationState::FormationDeactivationZone;
            result.reason = FormationDecisionReason::InsideNominalFdz;
        } else if (nominal_standby) {
            result.state = FormationState::Standby;
            result.reason = FormationDecisionReason::InsideNominalStandby;
        } else {
            result.reason = entry_lookup.valid
                ? FormationDecisionReason::OutsideNominalFormationBoundary
                : FormationDecisionReason::BoundaryLookupFailed;
        }
        break;
    case FormationState::FormationDeactivationZone:
        if (relaxed_fdz) {
            result.state = FormationState::FormationDeactivationZone;
            result.retained_by_hysteresis = !nominal_fdz;
            result.reason = result.retained_by_hysteresis
                ? FormationDecisionReason::RetainedInFdzByExitBoundary
                : FormationDecisionReason::InsideNominalFdz;
        } else if (relaxed_standby) {
            result.state = FormationState::Standby;
            result.retained_by_hysteresis = !nominal_standby;
            result.reason = result.retained_by_hysteresis
                ? FormationDecisionReason::RetainedInStandbyByExitBoundary
                : FormationDecisionReason::InsideNominalStandby;
        } else {
            result.reason = exit_lookup.valid
                ? FormationDecisionReason::OutsideRelaxedFormationBoundary
                : FormationDecisionReason::BoundaryLookupFailed;
        }
        break;
    case FormationState::Standby:
        if (relaxed_fdz) {
            result.state = FormationState::FormationDeactivationZone;
            result.retained_by_hysteresis = !nominal_fdz;
            result.reason = result.retained_by_hysteresis
                ? FormationDecisionReason::RetainedInFdzByExitBoundary
                : FormationDecisionReason::InsideNominalFdz;
        } else if (relaxed_standby) {
            result.state = FormationState::Standby;
            result.retained_by_hysteresis = !nominal_standby;
            result.reason = result.retained_by_hysteresis
                ? FormationDecisionReason::RetainedInStandbyByExitBoundary
                : FormationDecisionReason::InsideNominalStandby;
        } else {
            result.reason = exit_lookup.valid
                ? FormationDecisionReason::OutsideRelaxedFormationBoundary
                : FormationDecisionReason::BoundaryLookupFailed;
        }
        break;
    }

    result.formation_inhibit =
        result.state != FormationState::OutsideFormation;
    result.entered_formation =
        previous_state == FormationState::OutsideFormation &&
        result.formation_inhibit;
    result.exited_formation =
        previous_state != FormationState::OutsideFormation &&
        !result.formation_inhibit;
    return result;
}

FormationAggregationResult FormationInhibitAggregator::aggregate(
    const std::vector<FormationResult> & per_threat_results,
    const std::vector<std::uint32_t> & collision_relevant_threat_ids,
    const FormationAggregationPolicy policy) noexcept
{
    FormationAggregationResult aggregation{};
    aggregation.policy = policy;
    aggregation.relevant_threat_count = collision_relevant_threat_ids.size();

    std::unordered_map<std::uint32_t, const FormationResult *> by_id;
    std::unordered_set<std::uint32_t> duplicate_ids;
    for (const auto & result : per_threat_results) {
        if (!by_id.emplace(result.threat_id, &result).second) {
            duplicate_ids.insert(result.threat_id);
        }
    }

    std::unordered_set<std::uint32_t> unique_relevant;
    bool all_relevant_formation = !collision_relevant_threat_ids.empty();
    for (const auto threat_id : collision_relevant_threat_ids) {
        if (!unique_relevant.insert(threat_id).second) {
            all_relevant_formation = false;
            continue;
        }
        const auto result_iterator = by_id.find(threat_id);
        if (result_iterator == by_id.end() ||
            duplicate_ids.find(threat_id) != duplicate_ids.end()) {
            all_relevant_formation = false;
            continue;
        }
        ++aggregation.matched_result_count;
        const auto & result = *result_iterator->second;
        if (result.formation_inhibit && result.timestamp_valid &&
            result.geometry_valid) {
            ++aggregation.inhibited_threat_count;
            aggregation.inhibited_threat_ids.push_back(threat_id);
        } else {
            all_relevant_formation = false;
        }
    }
    aggregation.complete_collision_context =
        aggregation.matched_result_count == aggregation.relevant_threat_count &&
        unique_relevant.size() == aggregation.relevant_threat_count;

    switch (policy) {
    case FormationAggregationPolicy::PerThreatExemptionOnly:
        aggregation.formation_inhibit = false;
        break;
    case FormationAggregationPolicy::AllRelevantThreatsFormation:
        aggregation.formation_inhibit =
            aggregation.complete_collision_context && all_relevant_formation;
        break;
    case FormationAggregationPolicy::AnyRelevantThreatFormation:
        aggregation.formation_inhibit =
            aggregation.inhibited_threat_count > 0;
        break;
    }
    aggregation.allow_new_activation = !aggregation.formation_inhibit;
    return aggregation;
}

const char * toString(const FormationState state) noexcept
{
    switch (state) {
    case FormationState::OutsideFormation: return "OUTSIDE_FORMATION";
    case FormationState::FormationDeactivationZone: return "FDZ";
    case FormationState::Standby: return "STANDBY";
    }
    return "UNKNOWN";
}

const char * toString(const FormationDecisionReason reason) noexcept
{
    switch (reason) {
    case FormationDecisionReason::None: return "none";
    case FormationDecisionReason::InsideNominalFdz:
        return "inside nominal FDZ";
    case FormationDecisionReason::InsideNominalStandby:
        return "inside nominal standby envelope";
    case FormationDecisionReason::RetainedInFdzByExitBoundary:
        return "retained in FDZ by exit boundary";
    case FormationDecisionReason::RetainedInStandbyByExitBoundary:
        return "retained in standby by exit boundary";
    case FormationDecisionReason::OutsideNominalFormationBoundary:
        return "outside nominal formation boundary";
    case FormationDecisionReason::OutsideRelaxedFormationBoundary:
        return "outside relaxed formation boundary";
    case FormationDecisionReason::InvalidConfiguration:
        return "invalid formation configuration";
    case FormationDecisionReason::InvalidKinematicState:
        return "invalid threat kinematic state";
    case FormationDecisionReason::InvalidTimestamp:
        return "invalid or stale threat timestamp";
    case FormationDecisionReason::BoundaryLookupFailed:
        return "formation boundary lookup failed";
    }
    return "unknown";
}

const char * toString(const FormationProfileKind kind) noexcept
{
    switch (kind) {
    case FormationProfileKind::SourceReference: return "source_reference";
    case FormationProfileKind::UavCalibrated: return "uav_calibrated";
    }
    return "unknown";
}

const char * toString(const FormationAggregationPolicy policy) noexcept
{
    switch (policy) {
    case FormationAggregationPolicy::PerThreatExemptionOnly:
        return "per_threat_exemption_only";
    case FormationAggregationPolicy::AllRelevantThreatsFormation:
        return "all_relevant_threats_formation";
    case FormationAggregationPolicy::AnyRelevantThreatFormation:
        return "any_relevant_threat_formation";
    }
    return "unknown";
}

}  // namespace collision_avoidance::formation
