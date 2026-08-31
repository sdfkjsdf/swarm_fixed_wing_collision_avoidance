#pragma once

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <collision_avoidance/selection/BackupSafetyCertifierV4.hpp>

#include <cstdint>

namespace collision_avoidance::selection
{

enum class BackupThreatIntentStatusV4 : std::uint8_t
{
    Valid,
    InvalidConfiguration,
    InvalidIntent,
    FutureIntent,
    StaleIntent,
    PropagationFailed,
};

struct BackupThreatIntentAdapterV4Params
{
    estimation::PredictParams predictor{};
    double horizon_s{4.5};
    double integration_step_s{0.1};
    std::uint64_t maximum_intent_age_us{1'000'000};
    double time_tolerance_s{1.0e-7};
};

struct BackupThreatIntentAdapterV4Result
{
    BackupThreatIntentStatusV4 status{
        BackupThreatIntentStatusV4::InvalidConfiguration};
    BackupThreatTrajectoryV4 trajectory{};
    std::uint64_t source_age_us{0};
};

// Phase-4 bridge from the existing reconstructed intent transport to the
// common-time, full-horizon input required by the Mode-B certifier.  It uses
// the transmitted candidate input to re-propagate from the state interpolated
// at evaluation_timestamp_us; it does not invent a second transport format.
class BackupThreatIntentAdapterV4
{
public:
    explicit BackupThreatIntentAdapterV4(
        const BackupThreatIntentAdapterV4Params & params = {});

    const BackupThreatIntentAdapterV4Params & params() const noexcept;

    BackupThreatIntentAdapterV4Result alignAndPropagate(
        std::uint64_t evaluation_timestamp_us,
        int vehicle_id,
        double physical_clearance_m,
        const estimation::ReceivedTrajectoryIntent & intent) const noexcept;

    static bool validParams(
        const BackupThreatIntentAdapterV4Params & params) noexcept;

private:
    BackupThreatIntentAdapterV4Params m_params;
    estimation::TrajectoryPredict m_predictor;
};

const char * backupThreatIntentStatusName(
    BackupThreatIntentStatusV4 status) noexcept;

}  // namespace collision_avoidance::selection
