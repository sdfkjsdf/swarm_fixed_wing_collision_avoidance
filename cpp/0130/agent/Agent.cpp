#include<agent/Agent.h>
#include<chrono>
#include <fstream>
#include<sstream>
#include <iostream> 

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
        m_agent_state,
        m_received_other_information1,
        m_received_other_information2,
        m_received_other_information3,
        m_received_other_information4
    );

    // 2. Controller 계산
     m_agent_control_input = m_controller.calculate_control_input(  
        m_agent_state,
        m_setpoint,
        guidance_output
    );

    /*3. 실속 경고 체크 */
    const double current_speed = m_agent_state(6);
    const double stall_speed = m_controller.get_stall_speed();
    const double stall_margin = current_speed - stall_speed;
    
    if (stall_margin < -0.5) {
        std::cout << "[WARNING] Agent " << m_agent_id 
                  << " STALL DETECTED! V - V_stall = " << stall_margin 
                  << " m/s (V=" << current_speed 
                  << ", V_stall=" << stall_speed << ")" << std::endl;
    }




}

void Agent:: zoh_propagation_next_step()
{


    auto start = std::chrono::high_resolution_clock::now();
    
    m_agent_zoh_next_state = m_DynamicsLinearization.propagtion_zoh(m_agent_state, m_agent_control_input);
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> elapsed = end - start;  // 마이크로초 단위
    
   // std::cout << "Agent " << m_agent_id << " ZOH time: " << elapsed.count() << " us" << std::endl;



}

