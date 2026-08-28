#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

namespace
{

ce::PredictStateCovariance initialCovariance()
{
    ce::PredictStateCovariance covariance{};
    const std::array<double, ce::kPredictStateDimension> diagonal{
        0.25, 0.25, 0.25, 0.04, 0.001, 0.04, 0.001};
    for (std::size_t index = 0; index < diagonal.size(); ++index) {
        covariance[index * ce::kPredictStateDimension + index] = diagonal[index];
    }
    return covariance;
}

bool buildCandidateSet(
    const ce::TrajectoryIntentSender & sender,
    ce::TrajectoryIntentReceiver & receiver,
    std::uint64_t timestamp_us,
    const ce::PredictState & initial_state,
    const ce::PredictStateCovariance & covariance,
    cs::CandidateIntentSet & candidates)
{
    constexpr std::array<std::uint8_t, cs::kCandidatesPerAircraft> candidate_ids{
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollMinus15),
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollZero),
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollPlus15)};

    for (std::size_t index = 0; index < candidate_ids.size(); ++index) {
        ce::TrajectoryIntentPacket packet;
        if (!sender.buildForSelectedCandidate(
                timestamp_us,
                candidate_ids[index],
                initial_state,
                covariance,
                packet)
            || !receiver.receive(packet, candidates[index])) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main()
{
    ce::PredictParams predictor_params;
    predictor_params.V_min = 10.0;
    predictor_params.V_max = 25.0;
    ce::TrajectoryPredict predictor(predictor_params);
    const auto maneuver_candidates = ce::makeLevelTurnCandidateTable(20.0, 100.0);
    ce::TrajectoryIntentSender sender(predictor, maneuver_candidates);
    ce::TrajectoryIntentReceiver receiver(predictor, maneuver_candidates);

    constexpr std::uint64_t snapshot_timestamp_us = 10'000'000ULL;
    const ce::PredictState aircraft_a{
        -45.0, 0.0, 100.0, 20.0, 0.0, 0.0, 0.0};
    const ce::PredictState aircraft_b{
        45.0, 0.0, 100.0, 20.0, M_PI, 0.0, 0.0};
    const ce::PredictStateCovariance covariance = initialCovariance();

    cs::CandidateIntentSet candidates_a{};
    cs::CandidateIntentSet candidates_b{};
    if (!buildCandidateSet(
            sender,
            receiver,
            snapshot_timestamp_us,
            aircraft_a,
            covariance,
            candidates_a)
        || !buildCandidateSet(
            sender,
            receiver,
            snapshot_timestamp_us,
            aircraft_b,
            covariance,
            candidates_b)) {
        std::cerr << "failed to build static candidate trajectories\n";
        return 1;
    }

    cs::ManeuverCombinationEvaluatorParams evaluator_params;
    evaluator_params.desired_separation_distance_m = 10.0;
    evaluator_params.ownship_half_wingspan_m = 1.072;
    evaluator_params.threat_half_wingspan_m = 1.072;
    cs::ManeuverCombinationEvaluator evaluator(evaluator_params);

    cs::StaticCombinationEvaluation evaluation;
    if (!evaluator.evaluate(
            snapshot_timestamp_us,
            candidates_a,
            candidates_b,
            evaluation)) {
        std::cerr << cs::ManeuverCombinationEvaluator::formatTable(evaluation);
        return 2;
    }

    std::cout << cs::ManeuverCombinationEvaluator::formatTable(evaluation);
    return 0;
}
