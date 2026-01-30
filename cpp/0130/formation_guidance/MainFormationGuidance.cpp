#include<formation_guidance/MainFormationGuidance.h>


MainFormationGuidance::MainFormationGuidance(const FixedwingSpec_t& spec, const Flockingparameter_t& flocking_gain, const LeaderFollowerParameter_t& leaderfollower_gain,const APFParameter_t& APF_gain)
                    :m_spec(spec),
                     m_flocking_gain(flocking_gain),
                     m_apf_gain(APF_gain),
                     m_leaderfollower_gain(leaderfollower_gain),
                     m_LeaderFollowerGuidance(spec, leaderfollower_gain),
                     m_FlockingGuidance(spec, flocking_gain),
                     m_APFCollisionAvoidance(APF_gain)
{}


 matrix::Vector<double,3> MainFormationGuidance::calculate_roll_yawrate_groundacc_formation( 
                                                                      bool const APF_switch,
                                                                      bool const leader_follower_switch, 
                                                                      bool const am_i_leader,
                                                                      const matrix::Vector<double,2>& desired_position,
                                                                      const matrix::Vector<double,11>& state,
                                                                      const TsMessage_t other_information1,
                                                                      const TsMessage_t other_information2,
                                                                      const TsMessage_t other_information3,
                                                                      const TsMessage_t other_information4 )
        
    {
        if(leader_follower_switch)
        {

            m_output_MainFormationGuidance_rar=m_LeaderFollowerGuidance.caclulate_yawrate_ground_acc_ground_speed(am_i_leader,
                                                                                                       desired_position,
                                                                                                       state,
                                                                                                       other_information1,
                                                                                                       other_information2,
                                                                                                       other_information3,
                                                                                                       other_information4 );

        }

        else
        {
            m_output_MainFormationGuidance_rar=m_FlockingGuidance.caclulate_yawrate_ground_acc_ground_speed(state,
                                                                                                other_information1,
                                                                                                other_information2,
                                                                                                other_information3,
                                                                                                other_information4);
        }




        if (APF_switch)
        {
            matrix::Vector<double,3> repulsive_effect =m_APFCollisionAvoidance.calculate_repulsive_effect(state,
                                                                                                other_information1,
                                                                                                other_information2,
                                                                                                other_information3,
                                                                                                other_information4);

            m_output_MainFormationGuidance_rar+=repulsive_effect;


        }
        
        else
        {

            /*스위치가 안 켜지면 별도로 제공 안함*/



        }



       
        
        m_output_MainFormationGuidance=saturation_formation_guidance(m_output_MainFormationGuidance_rar);
        return m_output_MainFormationGuidance;



    }
