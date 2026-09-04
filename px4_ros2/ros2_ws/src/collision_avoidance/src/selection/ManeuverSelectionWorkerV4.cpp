#include "ManeuverSelectionWorkerInternal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace collision_avoidance::selection
{
using namespace worker_detail;

bool ManeuverSelectionWorker::buildV4IntentSet(
    std::uint64_t now_us,
    const SafeControlCandidateAdapterResult & candidates,
    ManeuverSelectionWorkerOutput & output)
{
    std::array<SafeControlCandidate, kMaximumSafeControlCandidates>
        selected_candidates{};
    std::size_t candidate_count = 0;
    const bool generated_safe_set =
        candidates.status == SafeControlCandidateAdapterStatus::Valid
        && candidates.candidate_count > 0
        && candidates.candidate_count <= selected_candidates.size();
    if (generated_safe_set) {
        candidate_count = candidates.candidate_count;
        std::copy_n(
            candidates.candidates.begin(),
            candidate_count,
            selected_candidates.begin());
    }

    const ManeuverActivationStatus activation =
        m_activation_controller.status();
    const bool retain_selected = activation.active
        || (m_has_selected_combination && m_selected_v4_cutover);
    if (retain_selected) {
        const std::size_t ownship_index = static_cast<std::size_t>(
            m_params.vehicle_id);
        const std::uint8_t retained_candidate_id = activation.active
            ? activation.latched_candidate_id
            : m_selected_candidate_ids[ownship_index];
        const std::uint64_t retained_input_revision = activation.active
            ? activation.latched_candidate_input_revision
            : m_selected_candidate_input_revisions[ownship_index];
        const estimation::PredictInput retained_input = activation.active
            ? activation.latched_input
            : m_latest_selection_decision.ownship_input;
        if (retained_candidate_id >= kMaximumSafeControlCandidates
            || retained_input_revision == 0) {
            return false;
        }
        const auto found = std::find_if(
            selected_candidates.begin(),
            selected_candidates.begin()
                + static_cast<std::ptrdiff_t>(candidate_count),
            [retained_candidate_id](const SafeControlCandidate & candidate) {
                return static_cast<std::uint8_t>(candidate.role)
                    == retained_candidate_id;
            });
        if (found != selected_candidates.begin()
                + static_cast<std::ptrdiff_t>(candidate_count)) {
            found->predictor_input = retained_input;
        } else if (candidate_count < selected_candidates.size()) {
            auto & retained = selected_candidates[candidate_count++];
            retained.role = static_cast<SafeCandidateRole>(
                retained_candidate_id);
            retained.predictor_input = retained_input;
        } else {
            return false;
        }
    }

    if (candidate_count == 0) {
        m_ownship_candidates_complete = false;
        m_ownship_candidate_count = 0;
        return false;
    }

    ExhaustiveCandidateIntentSet received_candidates{};
    std::array<
        estimation::TrajectoryIntentPacket,
        kExhaustiveCandidatesPerAircraft> packets{};
    for (std::size_t index = 0; index < candidate_count; ++index) {
        const std::uint8_t candidate_id = static_cast<std::uint8_t>(
            selected_candidates[index].role);
        if (!m_sender.buildForCandidateInput(
                now_us,
                candidate_id,
                selected_candidates[index].predictor_input,
                m_latest_state,
                m_latest_covariance,
                packets[index],
                m_selection_epoch)) {
                m_ownship_candidates_complete = false;
                m_ownship_candidate_count = 0;
                return false;
        }
        if (retain_selected && candidate_id == (activation.active
                ? activation.latched_candidate_id
                : m_selected_candidate_ids[static_cast<std::size_t>(
                    m_params.vehicle_id)])
            && packets[index].candidate_input_revision
                != (activation.active
                    ? activation.latched_candidate_input_revision
                    : m_selected_candidate_input_revisions[
                        static_cast<std::size_t>(m_params.vehicle_id)])) {
            m_ownship_candidates_complete = false;
            m_ownship_candidate_count = 0;
            return false;
        }
        packets[index].candidate_set_size = static_cast<std::uint8_t>(
            candidate_count);
        packets[index].candidate_set_kind =
            estimation::CandidateSetKind::V4SafeControl;
        if (!m_receiver.receive(packets[index], received_candidates[index])) {
            m_ownship_candidates_complete = false;
            m_ownship_candidate_count = 0;
            return false;
        }
    }

    const auto byCandidateId = [](const auto & lhs, const auto & rhs) {
        return lhs.candidate_id < rhs.candidate_id;
    };
    std::sort(
        received_candidates.begin(),
        received_candidates.begin()
            + static_cast<std::ptrdiff_t>(candidate_count),
        byCandidateId);
    std::sort(
        packets.begin(),
        packets.begin() + static_cast<std::ptrdiff_t>(candidate_count),
        byCandidateId);

    m_ownship_candidates = received_candidates;
    m_ownship_candidates_complete = true;
    m_ownship_candidate_count = candidate_count;
    m_ownship_candidate_set_kind =
        estimation::CandidateSetKind::V4SafeControl;
    std::copy_n(
        packets.begin(), candidate_count, output.intent_packets.begin());
    output.intent_packet_count = candidate_count;
    output.generated_timestamp_us = now_us;
    output.selection_epoch = m_selection_epoch;
    return true;
}

void ManeuverSelectionWorker::evaluateV4(
    std::uint64_t now_us,
    ManeuverSelectionWorkerOutput & output)
{
    ManeuverSelectionDecision decision = output.has_decision
        ? output.decision
        : m_latest_selection_decision;
    decision.vehicle_id = m_params.vehicle_id;
    decision.aircraft_count = static_cast<std::size_t>(
        m_params.total_agent_count);
    decision.v4_enabled = true;
    decision.v4_shadow_only = m_params.v4_shadow_only;
    decision.v4_control_architecture = m_params.v4_control_architecture;
    decision.v4_shadow_evaluated = false;
    decision.v4_shadow_status = V4ShadowEvaluationStatus::Disabled;
    decision.v4_airspeed_source = V4AirspeedSource::Unavailable;
    decision.v4_px4_airspeed_source = m_has_latest_airspeed
        ? m_latest_airspeed.px4_airspeed_source
        : -1;
    decision.v4_airspeed_timestamp_us = m_has_latest_airspeed
        ? m_latest_airspeed.timestamp_us
        : 0;
    decision.v4_nominal_timestamp_us = m_has_latest_nominal
        ? m_latest_nominal.timestamp_us
        : 0;
    decision.v4_nominal_available = false;
    decision.v4_safe_control = SafeControlSetV4Result{};
    decision.mode_b_threat_status = BackupThreatIntentStatusV4::Valid;
    decision.mode_b_invalid_threat_vehicle_id = -1;
    decision.mode_b_interpolation_status =
        BackupInterpolationStatusV4::InvalidConfiguration;
    decision.mode_b_branch_classification =
        BackupBranchClassificationV4::NeitherCertified;
    decision.mode_b_left_certified = false;
    decision.mode_b_right_certified = false;
    decision.mode_b_left_minimum_path_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_right_minimum_path_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_left_terminal_turn_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_right_terminal_turn_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_left_interpolation_status =
        BackupInterpolationBranchStatusV4::NotCertified;
    decision.mode_b_right_interpolation_status =
        BackupInterpolationBranchStatusV4::NotCertified;
    decision.mode_b_left_mu_star = std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_right_mu_star = std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_left_safe_rate_radps =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_right_safe_rate_radps =
        std::numeric_limits<double>::quiet_NaN();
    decision.v4_candidates = SafeControlCandidateAdapterResult{};
    // These are one-output transition events.  A diagnostic-only 20 Hz
    // publication must not repeat the previous activation edge.
    decision.activation_just_started = false;
    decision.activation_just_ended = false;
    decision.deactivation_reason = ManeuverDeactivationReason::None;

    const auto publishDiagnostics = [&]() {
        m_latest_selection_decision = decision;
        output.decision = decision;
        output.has_decision = true;
    };

    const bool airspeed_value_valid = m_has_latest_airspeed
        && m_latest_airspeed.valid
        && finitePositive(m_latest_airspeed.true_airspeed_mps);
    decision.v4_airspeed_snapshot_status = classifySnapshot(
        m_has_latest_airspeed,
        airspeed_value_valid,
        decision.v4_airspeed_timestamp_us,
        now_us,
        m_params.v4_maximum_airspeed_age_us,
        decision.v4_airspeed_age_us);

    double true_airspeed_mps = m_params.v4_trim_airspeed_mps;
    if (decision.v4_airspeed_snapshot_status == V4SnapshotStatus::Valid) {
        true_airspeed_mps = m_latest_airspeed.true_airspeed_mps;
        decision.v4_airspeed_source = V4AirspeedSource::ActualTas;
    } else {
        decision.v4_airspeed_source = V4AirspeedSource::TrimFallback;
    }

    const bool nominal_value_valid = m_has_latest_nominal
        && m_latest_nominal.valid
        && finitePositive(m_latest_nominal.ground_speed_command_mps)
        && (std::isfinite(m_latest_nominal.altitude_command_m)
            || std::isnan(m_latest_nominal.altitude_command_m))
        && std::isfinite(
            m_latest_nominal.lateral_acceleration_px4_mps2);
    decision.v4_nominal_snapshot_status = classifySnapshot(
        m_has_latest_nominal,
        nominal_value_valid,
        decision.v4_nominal_timestamp_us,
        now_us,
        m_params.v4_maximum_nominal_age_us,
        decision.v4_nominal_age_us);
    decision.v4_nominal_available =
        decision.v4_nominal_snapshot_status == V4SnapshotStatus::Valid;

    if (m_params.v4_control_architecture
            == V4ControlArchitecture::ClosedFormBackupModeB) {
        if (!decision.v4_nominal_available) {
            // Mode B interpolates from the actual nominal command. Missing or
            // stale nominal data cannot be replaced by an invented zero-rate
            // command; an already coordinated command is retained upstream.
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::CandidateGenerationFailed;
            publishDiagnostics();
            return;
        }

        const double nominal_heading_rate_v4_radps =
            SafeControlCandidateAdapter::
                px4LateralAccelerationToV4HeadingRate(
                    true_airspeed_mps,
                    m_latest_nominal.lateral_acceleration_px4_mps2,
                    m_params.v4_candidate_adapter_params
                        .speed_tolerance_mps);
        if (!std::isfinite(nominal_heading_rate_v4_radps)) {
            decision.v4_nominal_available = false;
            decision.v4_nominal_snapshot_status = V4SnapshotStatus::Invalid;
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::CandidateGenerationFailed;
            publishDiagnostics();
            return;
        }

        BackupSafetyCertifierV4Input backup_input;
        backup_input.ownship.timestamp_us = now_us;
        backup_input.ownship.north_m = m_latest_state.p_n;
        backup_input.ownship.east_m = m_latest_state.p_e;
        backup_input.ownship.heading_ned_rad = m_latest_state.psi;
        backup_input.ownship.true_airspeed_mps = true_airspeed_mps;
        backup_input.ownship.longitudinal_acceleration_mps2 = 0.0;
        backup_input.ownship.longitudinal_source =
            LongitudinalDriftSource::LocalOneStepFreeze;

        for (int remote_id = 0;
             remote_id < m_params.total_agent_count; ++remote_id) {
            if (remote_id == m_params.vehicle_id) {
                continue;
            }
            const std::size_t remote_index = static_cast<std::size_t>(
                remote_id);
            const RemoteDecisionCache & peer_decision =
                m_remote_decision_caches[remote_index];
            if (!peer_decision.valid
                || !peer_decision.decision.coordination_qualified) {
                decision.v4_shadow_status =
                    V4ShadowEvaluationStatus::MissingPeerDecision;
                publishDiagnostics();
                return;
            }

            const std::uint8_t candidate_id =
                peer_decision.decision.ownship_candidate_id;
            const std::uint64_t candidate_input_revision =
                peer_decision.decision.selected_candidate_input_revisions[
                    remote_index];
            const estimation::ReceivedTrajectoryIntent * selected_intent =
                nullptr;
            for (const RemoteCandidateCache * cache : {
                     &m_remote_caches[remote_index],
                     &m_remote_previous_caches[remote_index]}) {
                if (cache->count == 0
                    || cache->count != cache->expected_count) {
                    continue;
                }
                const auto found = std::find_if(
                    cache->candidates.begin(),
                    cache->candidates.begin()
                        + static_cast<std::ptrdiff_t>(cache->count),
                    [candidate_id, candidate_input_revision](
                        const auto & candidate) {
                        return candidate.candidate_id == candidate_id
                            && candidate.candidate_input_revision
                                == candidate_input_revision;
                    });
                if (found != cache->candidates.begin()
                        + static_cast<std::ptrdiff_t>(cache->count)) {
                    selected_intent = &(*found);
                    break;
                }
            }
            const RemoteSelectedIntentCache & retained =
                m_remote_selected_caches[remote_index];
            if (selected_intent == nullptr && retained.valid
                && retained.intent.candidate_id == candidate_id
                && retained.intent.candidate_input_revision
                    == candidate_input_revision) {
                selected_intent = &retained.intent;
            }
            if (selected_intent == nullptr) {
                decision.v4_shadow_status =
                    V4ShadowEvaluationStatus::MissingPeerIntent;
                publishDiagnostics();
                return;
            }

            const auto aligned = m_mode_b_intent_adapter.alignAndPropagate(
                now_us,
                remote_id,
                m_params.evaluator_params.ownship_half_wingspan_m
                    + m_params.evaluator_params.threat_half_wingspan_m,
                *selected_intent);
            decision.mode_b_threat_status = aligned.status;
            if (aligned.status != BackupThreatIntentStatusV4::Valid) {
                decision.mode_b_invalid_threat_vehicle_id = remote_id;
                decision.v4_shadow_status = aligned.status
                        == BackupThreatIntentStatusV4::FutureIntent
                    ? V4ShadowEvaluationStatus::FuturePeerIntent
                    : aligned.status == BackupThreatIntentStatusV4::StaleIntent
                        ? V4ShadowEvaluationStatus::StalePeerIntent
                        : V4ShadowEvaluationStatus::InvalidPeerIntent;
                publishDiagnostics();
                return;
            }
            if (backup_input.threat_count >= backup_input.threats.size()) {
                decision.mode_b_invalid_threat_vehicle_id = remote_id;
                decision.mode_b_threat_status =
                    BackupThreatIntentStatusV4::InvalidIntent;
                decision.v4_shadow_status =
                    V4ShadowEvaluationStatus::InvalidPeerIntent;
                publishDiagnostics();
                return;
            }
            backup_input.threats[backup_input.threat_count++] =
                aligned.trajectory;
        }

        const auto interpolation = m_mode_b_interpolator.evaluate(
            backup_input, nominal_heading_rate_v4_radps);
        decision.v4_shadow_evaluated = true;
        decision.v4_shadow_status = V4ShadowEvaluationStatus::CoreEvaluated;
        decision.mode_b_interpolation_status = interpolation.status;
        decision.mode_b_branch_classification =
            interpolation.certification.classification;
        decision.mode_b_left_certified =
            interpolation.certification.left.certified;
        decision.mode_b_right_certified =
            interpolation.certification.right.certified;
        decision.mode_b_left_minimum_path_margin_m =
            interpolation.certification.left.minimum_path_margin_m;
        decision.mode_b_right_minimum_path_margin_m =
            interpolation.certification.right.minimum_path_margin_m;
        decision.mode_b_left_terminal_turn_margin_m =
            interpolation.certification.left.terminal_turn_margin_m;
        decision.mode_b_right_terminal_turn_margin_m =
            interpolation.certification.right.terminal_turn_margin_m;
        decision.mode_b_left_interpolation_status =
            interpolation.left.status;
        decision.mode_b_right_interpolation_status =
            interpolation.right.status;
        decision.mode_b_left_mu_star = interpolation.left.mu_star;
        decision.mode_b_right_mu_star = interpolation.right.mu_star;
        decision.mode_b_left_safe_rate_radps =
            interpolation.left.safe_heading_rate_v4_radps;
        decision.mode_b_right_safe_rate_radps =
            interpolation.right.safe_heading_rate_v4_radps;

        BackupControlCandidateAdapterInputV4 candidate_input;
        candidate_input.interpolation = interpolation;
        candidate_input.true_airspeed_mps = true_airspeed_mps;
        candidate_input.ground_speed_command_mps =
            m_latest_nominal.ground_speed_command_mps;
        candidate_input.altitude_command_m =
            m_latest_nominal.altitude_command_m;
        decision.v4_candidates =
            m_v4_candidate_adapter.generateFromBackupInterpolation(
                candidate_input);
        if (decision.v4_candidates.status
                != SafeControlCandidateAdapterStatus::Valid
            && decision.v4_candidates.status
                != SafeControlCandidateAdapterStatus::SearchSetInfeasible) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::CandidateGenerationFailed;
        }
        publishDiagnostics();
        return;
    }

    SafeControlSetV4Input safe_input;
    safe_input.ownship.timestamp_us = now_us;
    safe_input.ownship.north_m = m_latest_state.p_n;
    safe_input.ownship.east_m = m_latest_state.p_e;
    safe_input.ownship.heading_ned_rad = m_latest_state.psi;
    safe_input.ownship.true_airspeed_mps = true_airspeed_mps;
    safe_input.ownship.longitudinal_acceleration_mps2 = 0.0;
    safe_input.ownship.longitudinal_source =
        LongitudinalDriftSource::LocalOneStepFreeze;

    for (int remote_id = 0;
         remote_id < m_params.total_agent_count; ++remote_id) {
        if (remote_id == m_params.vehicle_id) {
            continue;
        }
        const std::size_t remote_index = static_cast<std::size_t>(remote_id);
        const RemoteDecisionCache & peer_decision =
            m_remote_decision_caches[remote_index];
        if (!peer_decision.valid
            || !peer_decision.decision.coordination_qualified) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::MissingPeerDecision;
            publishDiagnostics();
            return;
        }

        const std::uint8_t candidate_id =
            peer_decision.decision.ownship_candidate_id;
        const std::uint64_t candidate_input_revision =
            peer_decision.decision.selected_candidate_input_revisions[
                remote_index];
        const RemoteCandidateCache * selected_cache = nullptr;
        const estimation::ReceivedTrajectoryIntent * selected_intent = nullptr;
        for (const RemoteCandidateCache * cache : {
                 &m_remote_caches[remote_index],
                 &m_remote_previous_caches[remote_index]}) {
            if (cache->count == 0
                || cache->count != cache->expected_count) {
                continue;
            }
            const auto found = std::find_if(
                cache->candidates.begin(),
                cache->candidates.begin()
                    + static_cast<std::ptrdiff_t>(cache->count),
                [candidate_id, candidate_input_revision](
                    const auto & candidate) {
                    return candidate.candidate_id == candidate_id
                        && candidate.candidate_input_revision
                            == candidate_input_revision;
                });
            if (found != cache->candidates.begin()
                    + static_cast<std::ptrdiff_t>(cache->count)) {
                selected_cache = cache;
                selected_intent = &(*found);
                break;
            }
        }
        const RemoteSelectedIntentCache & retained =
            m_remote_selected_caches[remote_index];
        if (selected_intent == nullptr && retained.valid
            && retained.intent.candidate_id == candidate_id
            && retained.intent.candidate_input_revision
                == candidate_input_revision) {
            selected_intent = &retained.intent;
        }
        if (selected_intent == nullptr) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::MissingPeerIntent;
            publishDiagnostics();
            return;
        }
        if (selected_cache != nullptr
            && (selected_intent->source_timestamp_us
                    != selected_cache->source_timestamp_us
                || selected_intent->selection_epoch
                    != selected_cache->selection_epoch)) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::InvalidPeerIntent;
            publishDiagnostics();
            return;
        }

        IntentKinematics kinematics;
        const IntentKinematicsStatus interpolation_status =
            interpolateIntentKinematics(
                *selected_intent, now_us, kinematics);
        if (interpolation_status != IntentKinematicsStatus::Valid) {
            decision.v4_shadow_status = interpolation_status
                    == IntentKinematicsStatus::Future
                ? V4ShadowEvaluationStatus::FuturePeerIntent
                : interpolation_status == IntentKinematicsStatus::Stale
                    ? V4ShadowEvaluationStatus::StalePeerIntent
                    : V4ShadowEvaluationStatus::InvalidPeerIntent;
            publishDiagnostics();
            return;
        }

        auto & threat = safe_input.threats[safe_input.threat_count++];
        threat.vehicle_id = remote_id;
        // The intent was interpolated to the common ownship time above.
        threat.timestamp_us = now_us;
        threat.north_m = kinematics.position_ned[0];
        threat.east_m = kinematics.position_ned[1];
        threat.velocity_north_mps = kinematics.velocity_ned[0];
        threat.velocity_east_mps = kinematics.velocity_ned[1];
        threat.physical_clearance_m =
            m_params.evaluator_params.ownship_half_wingspan_m
            + m_params.evaluator_params.threat_half_wingspan_m;
    }

    decision.v4_safe_control = m_v4_safe_control.evaluate(safe_input);
    decision.v4_shadow_evaluated = true;
    decision.v4_shadow_status = V4ShadowEvaluationStatus::CoreEvaluated;

    if (v4HorizonFailClosedRequested(
            m_params.execution_policy,
            decision.v4_safe_control.status)) {
        // Failing to find a safe sampled-data interval is not evidence that
        // the raw nominal command is safe. Keep the last coordinated ownship
        // V4 command instead of failing open to point convergence.
        m_v4_horizon_local_gate_active = true;
        rollupV4HorizonGate(now_us);
        decision.v4_horizon_gate_evaluated = true;
        decision.v4_horizon_gate_valid = false;
        decision.v4_horizon_local_gate_active = true;
        decision.v4_horizon_gate_active = m_v4_horizon_gate_active;
        decision.v4_horizon_trigger_m = m_params.v4_horizon_trigger_m;
    }

    if (decision.v4_safe_control.status == SafeControlSetStatus::Valid
        || decision.v4_safe_control.status
            == SafeControlSetStatus::SearchSetInfeasible) {
        SafeControlCandidateAdapterInput candidate_input;
        candidate_input.safe_set = decision.v4_safe_control;
        candidate_input.true_airspeed_mps = true_airspeed_mps;
        candidate_input.nominal_rate_available =
            decision.v4_nominal_available;
        if (decision.v4_nominal_available) {
            candidate_input.nominal_heading_rate_v4_radps =
                SafeControlCandidateAdapter::
                    px4LateralAccelerationToV4HeadingRate(
                        true_airspeed_mps,
                        m_latest_nominal.lateral_acceleration_px4_mps2,
                        m_params.v4_candidate_adapter_params
                            .speed_tolerance_mps);
            if (!std::isfinite(
                    candidate_input.nominal_heading_rate_v4_radps)) {
                candidate_input.nominal_rate_available = false;
                decision.v4_nominal_available = false;
                decision.v4_nominal_snapshot_status =
                    V4SnapshotStatus::Invalid;
            }
        }
        candidate_input.ground_speed_command_mps =
            decision.v4_nominal_available
            ? m_latest_nominal.ground_speed_command_mps
            : m_params.ground_speed_command_mps;
        candidate_input.altitude_command_m = decision.v4_nominal_available
            ? m_latest_nominal.altitude_command_m
            : std::numeric_limits<double>::quiet_NaN();
        decision.v4_candidates =
            m_v4_candidate_adapter.generate(candidate_input);
        if (decision.v4_candidates.status
                != SafeControlCandidateAdapterStatus::Valid
            && decision.v4_candidates.status
                != SafeControlCandidateAdapterStatus::SearchSetInfeasible) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::CandidateGenerationFailed;
        }
    }

    publishDiagnostics();
}


bool ManeuverSelectionWorker::evaluateV4HorizonGate(
    std::uint64_t now_us,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & candidate_counts,
    ManeuverSelectionDecision & decision)
{
    decision.v4_horizon_gate_evaluated = true;
    decision.v4_horizon_gate_valid = false;
    decision.v4_horizon_local_gate_active =
        m_v4_horizon_local_gate_active;
    decision.v4_horizon_gate_active = m_v4_horizon_gate_active;
    decision.v4_horizon_h_worst_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.v4_horizon_trigger_m = m_params.v4_horizon_trigger_m;
    decision.v4_horizon_worst_time_offset_s =
        std::numeric_limits<double>::quiet_NaN();
    decision.v4_horizon_worst_first_vehicle_id = -1;
    decision.v4_horizon_worst_second_vehicle_id = -1;

    std::array<
        const estimation::ReceivedTrajectoryIntent *,
        kMaximumSelectionAircraft> near_nominal{};
    const std::uint8_t near_nominal_id = static_cast<std::uint8_t>(
        SafeCandidateRole::NearNominal);
    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        if (candidate_counts[aircraft_index] == 0
            || candidate_counts[aircraft_index]
                > kExhaustiveCandidatesPerAircraft) {
            return false;
        }
        const auto begin = candidate_sets[aircraft_index].begin();
        const auto end = begin + static_cast<std::ptrdiff_t>(
            candidate_counts[aircraft_index]);
        const auto found = std::find_if(
            begin, end, [near_nominal_id](const auto & candidate) {
                return candidate.candidate_id == near_nominal_id;
            });
        if (found == end) {
            return false;
        }
        near_nominal[aircraft_index] = &*found;
    }

    double h_worst_m = std::numeric_limits<double>::infinity();
    for (int first = 0; first < m_params.total_agent_count; ++first) {
        for (int second = first + 1;
             second < m_params.total_agent_count; ++second) {
            TrajectoryBarrierEvaluation pair;
            if (!m_barrier_evaluator.evaluateTrajectoryPair(
                    now_us,
                    *near_nominal[static_cast<std::size_t>(first)],
                    *near_nominal[static_cast<std::size_t>(second)],
                    pair)) {
                return false;
            }
            if (pair.minimum_clearance_m < h_worst_m) {
                h_worst_m = pair.minimum_clearance_m;
                decision.v4_horizon_worst_time_offset_s =
                    pair.minimum_clearance_time_offset_s;
                decision.v4_horizon_worst_first_vehicle_id = first;
                decision.v4_horizon_worst_second_vehicle_id = second;
            }
        }
    }
    if (!std::isfinite(h_worst_m)) {
        return false;
    }

    m_v4_horizon_local_gate_active = updateV4HorizonGateState(
        m_v4_horizon_local_gate_active,
        true,
        h_worst_m,
        m_params.v4_horizon_trigger_m);
    rollupV4HorizonGate(now_us);
    decision.v4_horizon_gate_valid = true;
    decision.v4_horizon_local_gate_active =
        m_v4_horizon_local_gate_active;
    decision.v4_horizon_gate_active = m_v4_horizon_gate_active;
    decision.v4_horizon_h_worst_m = h_worst_m;
    return true;
}

void ManeuverSelectionWorker::rollupV4HorizonGate(
    std::uint64_t now_us) noexcept
{
    bool aggregate_request = m_v4_horizon_local_gate_active;
    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & cache = m_remote_decision_caches[
            static_cast<std::size_t>(aircraft)];
        aggregate_request = aggregate_request
            || (cache.valid
                && cache.decision.v4_horizon_local_gate_active);
    }

    if (!m_v4_horizon_gate_active && aggregate_request) {
        if (latchV4HorizonOwnshipCandidate()) {
            m_v4_horizon_gate_active = true;
            m_v4_horizon_activation_timestamp_us = now_us;
        }
    } else if (m_v4_horizon_gate_active && !aggregate_request
        && v4HorizonHoldElapsed(
            m_v4_horizon_activation_timestamp_us,
            now_us,
            kTrajectoryHorizonMicroseconds)) {
        m_v4_horizon_gate_active = false;
        m_v4_horizon_activation_timestamp_us = 0;
        m_v4_horizon_latch_valid = false;
        m_v4_horizon_latched_input_revision = 0;
    }
    m_latest_selection_decision.v4_horizon_local_gate_active =
        m_v4_horizon_local_gate_active;
    m_latest_selection_decision.v4_horizon_gate_active =
        m_v4_horizon_gate_active;
}

bool ManeuverSelectionWorker::latchV4HorizonOwnshipCandidate() noexcept
{
    if (!m_has_selected_combination || !m_selected_v4_cutover) {
        return false;
    }
    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    const std::uint8_t candidate_id =
        m_selected_candidate_ids[ownship_index];
    const std::uint64_t input_revision =
        m_selected_candidate_input_revisions[ownship_index];
    if (candidate_id >= kMaximumSafeControlCandidates
        || input_revision == 0) {
        return false;
    }
    m_v4_horizon_latched_candidate_id = candidate_id;
    m_v4_horizon_latched_input_revision = input_revision;
    m_v4_horizon_latched_input = m_latest_selection_decision.ownship_input;
    m_v4_horizon_latch_valid = true;
    return true;
}

bool ManeuverSelectionWorker::constrainV4ActiveAircraftCandidates(
    MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & candidate_counts) const
{
    if (m_params.execution_policy == ManeuverExecutionPolicy::AmacAdThreshold) {
        return true;
    }
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        const std::size_t candidate_count = candidate_counts[aircraft_index];
        if (candidate_count == 0
            || candidate_count > kExhaustiveCandidatesPerAircraft) {
            return false;
        }
        bool active = false;
        std::uint8_t active_candidate_id = kRollZeroId;
        std::uint64_t active_candidate_input_revision = 0;
        if (aircraft == m_params.vehicle_id) {
            const ManeuverActivationStatus status =
                m_activation_controller.status();
            if (m_params.execution_policy
                    == ManeuverExecutionPolicy::HorizonGatedV4
                && m_v4_horizon_gate_active
                && m_v4_horizon_latch_valid) {
                active = true;
                active_candidate_id = m_v4_horizon_latched_candidate_id;
                active_candidate_input_revision =
                    m_v4_horizon_latched_input_revision;
            } else {
                active = status.active;
                active_candidate_id = status.latched_candidate_id;
                active_candidate_input_revision =
                    status.latched_candidate_input_revision;
            }
        } else {
            const RemoteDecisionCache & cache =
                m_remote_decision_caches[aircraft_index];
            active = cache.valid
                && cache.decision.coordination_qualified
                && cache.decision.selected_v4_cutover
                    == m_selected_v4_cutover
                && (cache.decision.activation_requested
                    || (m_params.execution_policy
                            == ManeuverExecutionPolicy::HorizonGatedV4
                        && cache.decision.command_execution_requested));
            active_candidate_id = cache.decision.ownship_candidate_id;
            active_candidate_input_revision =
                cache.decision.selected_candidate_input_revisions[
                    aircraft_index];
        }
        if (!active) {
            continue;
        }

        auto & candidates = candidate_sets[aircraft_index];
        const auto found = std::find_if(
            candidates.begin(),
            candidates.begin() + static_cast<std::ptrdiff_t>(candidate_count),
            [active_candidate_id, active_candidate_input_revision](
                const auto & candidate) {
                return candidate.candidate_id == active_candidate_id
                    && candidate.candidate_input_revision
                        == active_candidate_input_revision;
            });
        if (found == candidates.begin()
                + static_cast<std::ptrdiff_t>(candidate_count)) {
            return false;
        }
        const estimation::ReceivedTrajectoryIntent active_candidate = *found;
        std::fill_n(candidates.begin(), candidate_count, active_candidate);
    }
    return true;
}


}  // namespace collision_avoidance::selection

