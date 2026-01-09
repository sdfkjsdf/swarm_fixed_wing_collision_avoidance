#include <algorithm>
#include <controller/ThrustSetpoint.h>
#include <cmath>

ThrustSetpoint::ThrustSetpoint(const FixedwingSpec_t& spec, const GainParameter_t& gain )
        :m_spec(spec),
         m_gain(gain)
        {}

double ThrustSetpoint:: calculate_at_sp(const matrix::Vector<double,11>& state,  
                                        const double& stall_speed, 
                                        const double& dot_ground_speed_setpoint)
{   
    /*일단 상태 받아오기 */
    const double roll = state(3);
    const double pitch = state(4);
    const double speed = state(6);
    const double q = state(8);
    const double r = state(9);
    const double c_p =std::cos(pitch);
    const double s_p = std::sin(pitch);
    const double c_r = std::cos(roll);
    const double s_r = std::sin(roll);

    /*모델 베이스로 지금 피치각속도 */

    const double current_pitchrate = (q*c_r)-(r*s_r);
    

    /*비행기 spec 가져오기*/
    const double max_speed=m_spec.max_speed;
    const double at_max=m_spec.at_max;
    const double at_min=m_spec.at_min;

    const double g = m_spec.g;
    const double k_acc = m_spec.k_acc;
    
    const double k_v =m_gain.k_v ;

    const double dot_speed_setpoint = (dot_ground_speed_setpoint + (speed*s_p*current_pitchrate))/c_p ;

    const double speed_setpoint_before_fev=( (1/k_v)*dot_speed_setpoint )+speed;
    const double speed_setpoint_after_fev=std::clamp(speed_setpoint_before_fev,stall_speed,max_speed);

    const double error_speed= speed_setpoint_after_fev-speed;
    const double dot_error_speed= k_v*error_speed;

    const double at_setpoint_rar= dot_error_speed +(g*s_p)+(k_acc * (speed*speed) );

    m_at_sp=std::clamp(at_setpoint_rar,at_min,at_max );

    return m_at_sp;
    








}
