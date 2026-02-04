#include <collision_avoidance/control_barrier_function/SafetyBarrierCertificate.h>
#include <ts_message/TsMessage.h>
#include<cmath>
#include <iostream>


void SafetyBarrierCertificate:: calculate_grad_pij_x()
{

    m_grad_pij_x(0,0) = 1.0;
    m_grad_pij_x(1,1) = 1.0;
    m_grad_pij_x(2,2) = 1.0;


}




void SafetyBarrierCertificate:: calculate_grad_vij_x(const double speed,
                                                                            const double c_p,
                                                                            const double s_p,
                                                                            const double c_y,
                                                                            const double s_y)
                {   
                            m_grad_vij_x(0,4)=-speed*s_p*c_y;
                            m_grad_vij_x(1,4)=-speed*s_p*s_y;
                            m_grad_vij_x(2,4)=-speed*c_p;

                            m_grad_vij_x(0,5)=-speed*c_p*s_y;
                            m_grad_vij_x(1,5)= speed*c_p*c_y;

                            m_grad_vij_x(0,6)= c_p*c_y;
                            m_grad_vij_x(1,6)= c_p*s_y;
                            m_grad_vij_x(2,6)= -s_p;



                }





void SafetyBarrierCertificate::calculate_h_x_i_gradient_h_x_i(
    const matrix::Vector<double,11>& state_i,
    const TsMessage_t& other_information)
{
    // === 기존 코드 유지 ===
    matrix::Vector<double,3> position_i{};
    matrix::Vector<double,3> position_j{};
    matrix::Vector<double,3> velocity_i{};
    matrix::Vector<double,3> velocity_j{};

    const double roll_i = state_i(3);      // 롤
    const double pitch_i = state_i(4);
    const double yaw_i = state_i(5);
    const double speed_i = state_i(6);
    const double q_i = state_i(8);
    const double r_i = state_i(9);         // body yaw rate

    const double c_r_i = std::cos(roll_i);
    const double s_r_i = std::sin(roll_i);
    const double c_p_i = std::cos(pitch_i);
    const double s_p_i = std::sin(pitch_i);
    const double c_y_i = std::cos(yaw_i);
    const double s_y_i = std::sin(yaw_i);

    position_i(0) = state_i(0);
    position_i(1) = state_i(1);
    position_i(2) = state_i(2);
    velocity_i(0) = speed_i * c_p_i * c_y_i;
    velocity_i(1) = speed_i * c_p_i * s_y_i;
    velocity_i(2) = -speed_i * s_p_i;

    position_j(0) = other_information.m_ts_north;
    position_j(1) = other_information.m_ts_east;
    position_j(2) = other_information.m_ts_down;
    velocity_j(0) = other_information.m_ts_north_speed;
    velocity_j(1) = other_information.m_ts_east_speed;
    velocity_j(2) = other_information.m_ts_down_speed;

   


    // === 기존 CBF (h_collision) 계산 ===
    matrix::Vector<double,3> p_ij = position_i - position_j;
    matrix::Vector<double,3> v_ij = velocity_i - velocity_j;

    double norm_p_ij = p_ij.norm();
    double norm_pij_sq = p_ij.norm_squared();
    double dot_pv_ij = p_ij.dot(v_ij);

    m_h_x = (m_r_p / norm_p_ij) * dot_pv_ij + (norm_p_ij - m_critical_distance);

    m_gradient_p_ij_h = (m_r_p / norm_p_ij) * (v_ij - (dot_pv_ij / norm_pij_sq) * p_ij) + (p_ij / norm_p_ij);
    m_gradient_v_ij_h = m_r_p * p_ij / norm_p_ij;



    /*모델 동역학 기반 관련 내용*/
     calculate_grad_pij_x();
     calculate_grad_vij_x(speed_i, c_p_i, s_p_i, c_y_i, s_y_i);

     m_gradient_h_x_i=(m_gradient_p_ij_h.transpose()*m_grad_pij_x) + (m_gradient_v_ij_h.transpose()*m_grad_vij_x) ;



    
}