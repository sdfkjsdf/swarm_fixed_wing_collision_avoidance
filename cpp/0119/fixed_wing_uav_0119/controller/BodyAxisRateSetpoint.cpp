#include <controller/BodyAxisRateSetpoint.h>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h>
#include <cmath>
#include <algorithm>
#include <matrix/math.hpp>

BodyAxisRateSetpoint::BodyAxisRateSetpoint(const FixedwingSpec_t& spec)
    :m_spec(spec){}
matrix::Vector<double,3> BodyAxisRateSetpoint::calculate_bodyaxisrate(const matrix::Vector<double,11>& state,const matrix::Vector<double,3>& rollrate_pitchrate_yawrate )

{
   
    m_output_bodyaxisrate.setZero();

    const double n = state(0);
    const double e = state(1);
    const double d = state(2);

    const double roll = state(3);
    const double pitch = state(4);
    const double yaw = state(5);
    const double speed = state(6);

    const double p = state(7);
    const double q = state(8);
    const double r = state(9);
    const double at = state(10);

    const double c_r = std::cos(roll);
    const double s_r = std::sin(roll);
    const double c_p = std::cos(pitch);
    const double s_p = std::sin(pitch);
    const double c_y = std::cos(yaw);
    const double s_y = std::sin(yaw);
    const double tan_p = std::tan(pitch);

    const double p_max=m_spec.p_max;
    const double q_max=m_spec.q_max;
    const double r_max=m_spec.r_max;

    const double p_min=m_spec.p_min;
    const double q_min=m_spec.q_min;
    const double r_min=m_spec.r_min;
    

    matrix::Matrix<double,3,3> translation_matrix{};

        translation_matrix(0,0) = 1.0;
        translation_matrix(0,1) = 0.0;
        translation_matrix(0,2) =-s_p;

        translation_matrix(1,0) = 0.0;
        translation_matrix(1,1) = c_r;
        translation_matrix(1,2) = c_p*s_r ;

        translation_matrix(2,0) = 0.0 ;
        translation_matrix(2,1) = -s_r ;
        translation_matrix(2,2) = c_p*c_r ;


    matrix::Vector<double,3> pqr_setpoint_rar=translation_matrix*rollrate_pitchrate_yawrate ;

    m_output_bodyaxisrate(0)=std::clamp(pqr_setpoint_rar(0),p_min,p_max);
    m_output_bodyaxisrate(1)=std::clamp(pqr_setpoint_rar(1),q_min,q_max);
    m_output_bodyaxisrate(2)=std::clamp(pqr_setpoint_rar(2),r_min,r_max);

    return m_output_bodyaxisrate; 





}
