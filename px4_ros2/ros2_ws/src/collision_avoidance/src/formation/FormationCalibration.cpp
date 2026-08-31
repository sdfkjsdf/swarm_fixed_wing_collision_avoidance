#include <collision_avoidance/formation/FormationCalibration.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace collision_avoidance::formation
{
namespace
{

std::string quoteCsv(const std::string & value)
{
    std::string escaped{"\""};
    for (const char character : value) {
        if (character == '\"') {
            escaped.push_back('\"');
        }
        escaped.push_back(character);
    }
    escaped.push_back('\"');
    return escaped;
}

bool nearValue(
    const double value,
    const double boundary,
    const double tolerance) noexcept
{
    return std::isfinite(value) && std::isfinite(boundary) &&
        std::abs(value - boundary) <= tolerance;
}

bool nearBoundary(
    const FormationResult & result,
    const FormationMetricParams & params) noexcept
{
    return
        nearValue(result.range_m, result.fdz_entry_limit_m,
            params.boundary_range_tolerance_m) ||
        nearValue(result.range_m, result.fdz_exit_limit_m,
            params.boundary_range_tolerance_m) ||
        nearValue(result.range_m, result.max_range_entry_m,
            params.boundary_range_tolerance_m) ||
        nearValue(result.range_m, result.max_range_exit_m,
            params.boundary_range_tolerance_m) ||
        nearValue(result.closure_rate_mps, result.entry_closure_limit_mps,
            params.boundary_closure_tolerance_mps) ||
        nearValue(result.closure_rate_mps, result.exit_closure_limit_mps,
            params.boundary_closure_tolerance_mps) ||
        nearValue(result.closure_rate_mps, result.closure_lower_entry_mps,
            params.boundary_closure_tolerance_mps) ||
        nearValue(result.closure_rate_mps, result.closure_lower_exit_mps,
            params.boundary_closure_tolerance_mps);
}

}  // namespace

std::string FormationDecisionCsvFormatter::header()
{
    return
        "time_s,threat_id,range_m,range_rate_mps,closure_rate_mps,"
        "previous_state,new_state,entry_closure_limit_mps,"
        "exit_closure_limit_mps,closure_lower_entry_mps,"
        "closure_lower_exit_mps,fdz_entry_limit_m,fdz_exit_limit_m,"
        "max_range_entry_m,max_range_exit_m,formation_inhibit,"
        "entered_formation,retained_by_hysteresis,exited_formation,"
        "timestamp_valid,state_age_s,timestamp_skew_s,geometry_valid,"
        "boundary_lookup_valid,relative_bearing_rad,relative_altitude_m,"
        "ownship_speed_mps,threat_speed_mps,uncertainty_margin_m,"
        "profile_name,profile_kind,unit_system,reason";
}

std::string FormationDecisionCsvFormatter::row(
    const FormationResult & result,
    const FormationBoundaryConfig & config)
{
    std::ostringstream stream;
    stream << std::setprecision(17)
           << result.evaluation_timestamp_s << ','
           << result.threat_id << ','
           << result.range_m << ','
           << result.range_rate_mps << ','
           << result.closure_rate_mps << ','
           << quoteCsv(toString(result.previous_state)) << ','
           << quoteCsv(toString(result.state)) << ','
           << result.entry_closure_limit_mps << ','
           << result.exit_closure_limit_mps << ','
           << result.closure_lower_entry_mps << ','
           << result.closure_lower_exit_mps << ','
           << result.fdz_entry_limit_m << ','
           << result.fdz_exit_limit_m << ','
           << result.max_range_entry_m << ','
           << result.max_range_exit_m << ','
           << result.formation_inhibit << ','
           << result.entered_formation << ','
           << result.retained_by_hysteresis << ','
           << result.exited_formation << ','
           << result.timestamp_valid << ','
           << result.state_age_s << ','
           << result.timestamp_skew_s << ','
           << result.geometry_valid << ','
           << result.boundary_lookup_valid << ','
           << result.relative_bearing_rad << ','
           << result.relative_altitude_m << ','
           << result.ownship_speed_mps << ','
           << result.threat_speed_mps << ','
           << config.uncertainty_margin_m << ','
           << quoteCsv(config.profile_name) << ','
           << quoteCsv(toString(config.profile_kind)) << ','
           << quoteCsv("SI") << ','
           << quoteCsv(toString(result.reason));
    return stream.str();
}

std::vector<FormationReplayRecord> FormationCalibrationHarness::replay(
    const std::vector<FormationReplaySample> & samples,
    const FormationBoundaryConfig & config)
{
    FormationDiscriminator discriminator(config);
    std::vector<FormationReplayRecord> records;
    records.reserve(samples.size());
    for (const auto & sample : samples) {
        records.push_back({sample, discriminator.update(sample.input)});
    }
    return records;
}

FormationReplayMetrics FormationCalibrationHarness::metrics(
    const std::vector<FormationReplayRecord> & records,
    const FormationMetricParams & params) noexcept
{
    FormationReplayMetrics output{};
    struct PreviousBoundarySample
    {
        double timestamp_s{0.0};
        bool near_boundary{false};
    };
    std::unordered_map<std::uint32_t, PreviousBoundarySample> previous_by_threat;

    for (const auto & record : records) {
        const auto & result = record.result;
        if (result.entered_formation && output.first_entry_time_s < 0.0) {
            output.first_entry_time_s = result.evaluation_timestamp_s;
        }
        if (result.exited_formation && output.first_exit_time_s < 0.0) {
            output.first_exit_time_s = result.evaluation_timestamp_s;
        }
        if (result.state != result.previous_state) {
            ++output.state_toggle_count;
        }
        if (record.sample.collision_activation_requested &&
            record.sample.has_expected_inhibit &&
            record.sample.expected_inhibit) {
            ++output.nuisance_activation_opportunities;
            if (!result.formation_inhibit) {
                ++output.missed_inhibit_cases;
            }
        }

        const bool current_near_boundary = nearBoundary(result, params);
        const auto previous = previous_by_threat.find(result.threat_id);
        if (previous != previous_by_threat.end()) {
            const double elapsed_s =
                result.evaluation_timestamp_s - previous->second.timestamp_s;
            if (previous->second.near_boundary && std::isfinite(elapsed_s) &&
                elapsed_s > 0.0) {
                output.boundary_dwell_time_s += elapsed_s;
            }
        }
        previous_by_threat[result.threat_id] = {
            result.evaluation_timestamp_s, current_near_boundary};
    }
    return output;
}

std::vector<FormationProfileSweepResult> FormationCalibrationHarness::sweep(
    const std::vector<FormationReplaySample> & samples,
    const std::vector<FormationBoundaryConfig> & profiles,
    const FormationMetricParams & params)
{
    std::vector<FormationProfileSweepResult> output;
    output.reserve(profiles.size());
    for (const auto & profile : profiles) {
        FormationProfileSweepResult result{};
        result.profile_name = profile.profile_name;
        result.profile_kind = profile.profile_kind;
        result.config_valid = FormationDiscriminator::validConfig(profile);
        result.records = replay(samples, profile);
        result.metrics = metrics(result.records, params);
        output.push_back(std::move(result));
    }
    return output;
}

}  // namespace collision_avoidance::formation
