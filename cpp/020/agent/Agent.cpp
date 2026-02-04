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
           m_DynamicsLinearization(g_FixedwingSpec,m_dt),
           m_SafetyBarrierCertificate(){}


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
    constexpr double STALL_WARNING_MARGIN = -0.5;
    
    if (stall_margin < STALL_WARNING_MARGIN) {
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

void Agent::caluclate_safety_certificate(const matrix::Vector<double, 11>& state,const TsMessage_t& other_information1)
{
    matrix::Matrix<double,11,11> temp_Ad{};
    matrix::Matrix<double,11,4> temp_BdB{};
    matrix::Vector<double,11> temp_BdC{};
  
    matrix::Matrix<double,1,11> temp_gradient_h_x_i{};
    double temp_h_x=0.0;
    
    /*zoh 기반 이산화 과정 수행*/

    m_DynamicsLinearization.calculate_zoh_matrix(state);
    m_SafetyBarrierCertificate.calculate_h_x_i_gradient_h_x_i(state,other_information1);

    temp_Ad=m_DynamicsLinearization.get_Ad();
    temp_BdB=m_DynamicsLinearization.get_BdB();
    temp_BdC=m_DynamicsLinearization.get_BdC();

    /* h(x) 선형화 및 연산 수행*/

    temp_h_x=m_SafetyBarrierCertificate.get_h_x();
    temp_gradient_h_x_i=m_SafetyBarrierCertificate.get_gradient_h_x_i();

    m_lgh_i=temp_gradient_h_x_i*temp_BdB;

    // 출력 확인
    //  std::cout << "[Agent " << m_agent_id << "] h(x) = " << temp_h_x << std::endl;

    // std::cout << "[Agent " << m_agent_id << "] BdB rows 7-10:" << std::endl;
    //     for (int i = 7; i < 11; i++) {
    //     std::cout << "  row " << i << ": [" 
    //           << temp_BdB(i,0) << ", "
    //           << temp_BdB(i,1) << ", "
    //           << temp_BdB(i,2) << ", "
    //           << temp_BdB(i,3) << "]" << std::endl;
    //                                     }

    // std::cout << "[Agent " << m_agent_id << "] m_lgh_i = ["
    //           << m_lgh_i(0,0) << ", "
    //           << m_lgh_i(0,1) << ", "
    //           << m_lgh_i(0,2) << ", "
    //           << m_lgh_i(0,3) << "]" << std::endl;

    // std::cout << "grad_h[7..10] = " 
    //       << temp_gradient_h_x_i(0,7) << " "
    //       << temp_gradient_h_x_i(0,8) << " "
    //       << temp_gradient_h_x_i(0,9) << " "
    //       << temp_gradient_h_x_i(0,10) << std::endl;



}




