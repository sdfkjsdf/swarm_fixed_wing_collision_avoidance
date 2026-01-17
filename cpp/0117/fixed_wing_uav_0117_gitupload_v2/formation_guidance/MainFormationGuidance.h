#ifndef MAINFORMATIONGUIDANCE_H
#define MAINFORMATIONGUIDANCE_H

#include <fixed_wing_aircraft_model/FixedwingSpec.h>
#include <ts_message/TsMessage.h>
#include <ts_message/StateInformation.h>
#include <formation_guidance/FlockingParameter.h>
#include <formation_guidance/LeaderFollowerParameter.h>
#include <formation_guidance/FlockingGuidance.h>
#include <formation_guidance/LeaderFollowerGuidance.h>
#include<cmath>
#include<matrix/math.hpp>



class MainFormationGuidance
{
    public:
        MainFormationGuidance(const FixedwingSpec_t& spec, const Flockingparameter_t& flocking_gain, const LeaderFollowerParameter_t& leaderfollower_gain);
        ~MainFormationGuidance()=default;

        matrix::Vector<double,3> calculate_roll_yawrate_groundacc_formation(bool const leader_follower_switch, 
                                                                            bool const am_i_leader,
                                                                            const matrix::Vector<double,2>& desired_position,
                                                                            const matrix::Vector<double,11>& state,
                                                                            const TsMessage_t other_information1,
                                                                            const TsMessage_t other_information2,
                                                                            const TsMessage_t other_information3,
                                                                            const TsMessage_t other_information4 );

                                                                            /* bool const am_i_leader ,desired_position, 이것은 지금 Leader-follower에 쓰임 */
                                                                            /* 나머지는 공통적으로 쓰이는 것들임*/

    private:
    /* 각종 파라미터 내장*/
    FixedwingSpec_t m_spec;
    Flockingparameter_t m_flocking_gain;
    LeaderFollowerParameter_t m_leaderfollower_gain;

    /*모듈화된 제어기 내장*/
    LeaderFollowerGuidance m_LeaderFollowerGuidance;
    FlockingGuidance m_FlockingGuidance;

    /*출력 관련 내용*/
    matrix::Vector<double,3> m_output_MainFormationGuidance{};



};







#endif