#ifndef MAINFORMATIONGUIDANCE_H
#define MAINFORMATIONGUIDANCE_H

#include <fixed_wing_aircraft_model/FixedwingSpec.h>
#include <ts_message/TsMessage.h>
#include <ts_message/StateInformation.h>
#include <formation_guidance/FlockingParameter.h>
#include <formation_guidance/LeaderFollowerParameter.h>
#include <formation_guidance/FlockingGuidance.h>
#include <formation_guidance/LeaderFollowerGuidance.h>
#include <collision_avoidance/APF/APFParameter.h>
#include <collision_avoidance/APF/APFCollisionAvoidance.h>
#include<cmath>
#include<matrix/math.hpp>



class MainFormationGuidance
{
    public:
        MainFormationGuidance(const FixedwingSpec_t& spec, const Flockingparameter_t& flocking_gain, const LeaderFollowerParameter_t& leaderfollower_gain, const APFParameter_t& APF_gain);
        ~MainFormationGuidance()=default;
       
        /* calculate_roll_yawrate_groundacc_formation_rar 해당 기능은 saturation 이 되기 전의 값 */
        matrix::Vector<double,3> calculate_roll_yawrate_groundacc_formation(bool const  APF_switch,
                                                                            bool const leader_follower_switch, 
                                                                            bool const am_i_leader,
                                                                            const matrix::Vector<double,2>& desired_position,
                                                                            const matrix::Vector<double,11>& state,
                                                                            const TsMessage_t other_information1,
                                                                            const TsMessage_t other_information2,
                                                                            const TsMessage_t other_information3,
                                                                            const TsMessage_t other_information4 );
        /* calculate_yawrate_groundacc_formation_rar(0)=yawrate_setpoint_rar
        calculate_yawrate_groundacc_formation_rar(1)=ground_speed_acc
        calculate_yawrate_groundacc_formation_rar(2)=ground_speed     */

        /*collision avoidance 가 0 이면은 지금 안함*/
        /* 1이면음  APF으로 키기 */


        /* saturation_formation_guidance 해당 기능을 통해서 실질적으로 다 saturaion하여 추종할 수 있는 가이던스를 제공*/
       inline matrix::Vector<double,3> saturation_formation_guidance(matrix::Vector<double,3>  m_output_MainFormationGuidance_rar)
            
            {
                const double g =m_spec.g;
                const double max_roll=m_spec.max_roll;
                const double yawrate_setpoint_rar=m_output_MainFormationGuidance_rar(0);
                const double ground_acclertion_setpoint=m_output_MainFormationGuidance_rar(1);
                const double ground_speed=m_output_MainFormationGuidance_rar(2);
                const double roll_sp_rar = std::atan(ground_speed * yawrate_setpoint_rar/g);
                const double roll_sp = max_roll * std::tanh(roll_sp_rar / max_roll);
                const double yawrate_setpoint_after_smoothh  = (g * tan(roll_sp)) / ground_speed;

                m_output_MainFormationGuidance(0)=roll_sp;
                m_output_MainFormationGuidance(1)=yawrate_setpoint_after_smoothh;
                m_output_MainFormationGuidance(2)=ground_acclertion_setpoint;

                return m_output_MainFormationGuidance;

            


            };

                                                                            /* bool const am_i_leader ,desired_position, 이것은 지금 Leader-follower에 쓰임 */
                                                                            /* 나머지는 공통적으로 쓰이는 것들임*/

    private:
    /* 각종 파라미터 내장*/
    FixedwingSpec_t m_spec;
    Flockingparameter_t m_flocking_gain;
    LeaderFollowerParameter_t m_leaderfollower_gain;
    APFParameter_t m_apf_gain;

    /*모듈화된 제어기 내장*/
    LeaderFollowerGuidance m_LeaderFollowerGuidance;
    FlockingGuidance m_FlockingGuidance;
    APFCollisionAvoidance m_APFCollisionAvoidance;

    /*출력 관련 내용*/
    matrix::Vector<double,3> m_output_MainFormationGuidance_rar{};
    matrix::Vector<double,3> m_output_MainFormationGuidance{};




};







#endif