#include<agent/Agent.h>



 Agent::Agent(int agent_id,
          const FixedwingSpec_t& g_FixedwingSpec,
          const Flockingparameter_t& g_Flockingparameter,
          const LeaderFollowerParameter_t& g_LeaderFollowerParameter,
          const GainParameter_t& g_GainParameter,
          const NdiParameter_t& g_NdiParameter,
          const APFParameter_t& g_APFParameter,
          const double dt)
          
          :m_agent_id(agent_id),
           m_dt(dt),
           m_dynamics(g_FixedwingSpec),
           m_guidance(g_FixedwingSpec, g_Flockingparameter, g_LeaderFollowerParameter,g_APFParameter),
           m_controller(g_FixedwingSpec, g_GainParameter, g_NdiParameter),
           m_DynamicsLinearization(g_FixedwingSpec,m_dt){}



void Agent::compute_control()
{
    // 1. Guidance 계산
    matrix::Vector<double, 3> guidance_output = m_guidance.calculate_roll_yawrate_groundacc_formation(
        m_APF_mode,
        m_leader_follower_switch,
        m_am_i_leader,
        m_desired_position,
        agent_state,
        m_received_other_information1,
        m_received_other_information2,
        m_received_other_information3,
        m_received_other_information4
    );

    // 2. Controller 계산
     m_agent_control_input = m_controller.calculate_control_input(  // ✅ 실제 함수 이름
        agent_state,
        m_setpoint,
        guidance_output
    );
}



