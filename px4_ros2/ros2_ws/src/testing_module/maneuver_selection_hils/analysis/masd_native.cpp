// Offline analysis bridge: use the production receiver, not a Python predictor.
// No node, controller, or simulation state is changed by this library.
#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace ce = collision_avoidance::estimation;

extern "C" int masd_reconstruct(
    const float * values, unsigned long long revision, int candidate_id,
    int candidate_count, double tau_phi, double * output)
{
    ce::PredictParams params;
    params.tau_phi = tau_phi; // Required recorded-run parameter, never today's default.
    params.phi_rate_max = 70.0 * std::acos(-1.0) / 180.0;
    params.a_lat_max = 9.80665 * std::tan(50.0 * std::acos(-1.0) / 180.0);
    ce::TrajectoryPredict predictor(params);
    ce::TrajectoryIntentReceiver receiver(predictor);
    ce::TrajectoryIntentPacket packet;
    packet.candidate_id = candidate_id;
    packet.candidate_set_size = candidate_count;
    packet.candidate_input_revision = revision;
    std::copy_n(values, 7, packet.initial_state.begin());
    std::copy_n(values + 7, 49, packet.initial_covariance.begin());
    std::copy_n(values + 56, 4, packet.candidate_input.begin());
    static_assert(sizeof(packet.compressed_mean) == 18 * sizeof(float));
    std::memcpy(&packet.compressed_mean, values + 60, sizeof(packet.compressed_mean));
    ce::ReceivedTrajectoryIntent received;
    if (!receiver.receive(packet, received)) return 0;
    ce::PredictState initial{values[0], values[1], values[2], values[3],
                             values[4], values[5], values[6]};
    ce::PredictionMeanTrajectory direct{};
    predictor.predict(initial, received.candidate_input, 0.1, direct);
    for (std::size_t k = 0; k < ce::kTrajectoryPointCount; ++k) {
        const auto & p = received.cone[k];
        double * row = output + k * 19;
        row[0] = p.mean.p_n; row[1] = p.mean.p_e; row[2] = -p.mean.h;
        std::copy(p.position_covariance_ned.begin(), p.position_covariance_ned.end(), row + 3);
        row[12] = p.mean.phi; row[13] = p.mean.V;
        row[14] = p.mean.psi; row[15] = p.mean.h_dot;
        row[16] = direct[k].p_n; row[17] = direct[k].p_e; row[18] = -direct[k].h;
    }
    return 1;
}
