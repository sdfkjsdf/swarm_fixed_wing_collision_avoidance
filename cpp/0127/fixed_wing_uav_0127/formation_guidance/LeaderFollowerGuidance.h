#ifndef LEADERFOLLOWERGUIDANCE_H
#define LEADERFOLLOWERGUIDANCE_H  // <- 이것도 추가 필요!

#include <matrix/math.hpp>
#include <formation_guidance/LeaderFollowerParameter.h>
#include <fixed_wing_aircraft_model/FixedwingSpec.h>
#include <ts_message/TsMessage.h>

class LeaderFollowerGuidance
{   
public:
    LeaderFollowerGuidance(const FixedwingSpec_t& spec, const LeaderFollowerParameter_t& leaderFollower_gain);
    ~LeaderFollowerGuidance() = default;

    matrix::Vector<double,3> caclulate_yawrate_ground_acc_ground_speed(
        bool const am_i_leader,
        const matrix::Vector<double,2>& desired_position,
        const matrix::Vector<double,11>& state,
        const TsMessage_t other_information1,
        const TsMessage_t other_information2,
        const TsMessage_t other_information3,
        const TsMessage_t other_information4); 

private:
    FixedwingSpec_t m_spec;
    LeaderFollowerParameter_t m_leaderfollower_gain;
    matrix::Vector<double,3> m_yawrate_ground_acc_ground_speed{};
};

#endif