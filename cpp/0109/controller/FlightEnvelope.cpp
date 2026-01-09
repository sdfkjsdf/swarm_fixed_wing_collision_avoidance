
/******************************************************** */
/* 작성자 : 이동혁    */
/* 작성날짜 : 2026/01/09일 */
/* 기능 세부 설명 :  "Investigation of Practical Flight Envelope Protection Systemsfor Small Aircraft 논문 기반으로 고정익 무인기의  Flight envelop을 구현하는 내용*/ 

/*  수정날짜 및 수정사항    */ 
/* 현재 남아 있는 내용은 지금 v_stall 이라는 실속속도를 구하는 과정에서 선회 상승 비행에서는 어떤식으로 해야 할 것인지를 잘 고민해보아야 함  */
/* 실속보호를 위한 각도가 지금 q_cmd에서 satruatiuon이 되는 것이 아닌 yawrate_sp 에서 saturation이 된느 구조임*/

/********************************************************* */

#include <matrix/math.hpp>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h>
#include <controller/FlightEnvelope.h>
#include <cmath>
#include <algorithm>

FlightEnvelope::FlightEnvelope(const FixedwingSpec_t& spec)
    :m_spec(spec)
    {}

 matrix::Vector<double,4> FlightEnvelope :: apply_FlightEnvelope(const matrix::Vector<double,11>& state,
                                                                 const double& roll_setpoint,
                                                                 const double& pitch_setpoint_before_fev 
                                                                 )
 {

    m_output_flightenvelope.setZero();

    /* 상태 값을 받아오기 */
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

    

     /*중력 및 항력 계수 받아오기*/

    const double g = m_spec.g;
    const double k_acc = m_spec.k_acc;

    /*각도 변환 함수 시작 */
    const double c_r = std::cos(roll);
    const double s_r = std::sin(roll);
    const double c_p = std::cos(pitch);
    const double s_p = std::sin(pitch);
    const double c_y = std::cos(yaw);
    const double s_y = std::sin(yaw);
    const double tan_p = std::tan(pitch);

    /*기체 spec 관련 내용을 받아오기 */

    const double max_speed=m_spec.max_speed;
    const double min_speed=m_spec.min_speed;

    const double max_pitch=m_spec.max_pitch;
    const double min_pitch=m_spec.min_pitch;

  

    /*이전의 set point 값을 받아오기 */

    const double roll_sp=roll_setpoint;
    const double pitch_sp_before_fev=pitch_setpoint_before_fev;
    




    /* 실속속도 연산 */
    /* 지금 해당 부분은 수평선회 비행에서의 실속을 가정한 것이지만 나중에  선회 상승비행시에도 고려를 해야 한다*/
    //const double roll_fff = std::max(std::abs(roll_sp), std::abs(roll));
    const double roll_fff=std::max(std::abs(roll),std::abs(roll_sp));
    const double stall_gain=std::max( c_p/(std::cos(roll_fff)) ,1.0 );
    const double stall_speed=min_speed*std::sqrt(stall_gain);
    const double speed_lowwer=std::clamp(stall_speed,min_speed,max_speed);
   

    

    /* 피치각 조절을 통한 속도 보호 */
    const double c1 = 0.0;
    const double c2 = 5; /* 지금  clamp 에러가 안 생길려면 최소 3이상으로 설정하는 것을 추천  */
    const double margin=0.0001;
   
    /*Stall 방지를 위한 피치각 상한 하한 조절 */
    const double pitch_upper=max_pitch * std::tanh(c1+(speed-speed_lowwer)/c2 );
    const double pitch_lowwer=min_pitch *std::tanh(c1+(max_speed-speed)/c2 );


    const double pitch_sp_after_fev = std::clamp(pitch_sp_before_fev,pitch_lowwer,pitch_upper);

    const double fev_pitch = ((pitch - ((pitch_upper + pitch_lowwer) / 2.0)) * (pitch - (pitch_upper + pitch_lowwer) / 2.0)) 
                        - ((pitch_upper - (pitch_upper + pitch_lowwer) / 2.0) * (pitch_upper - (pitch_upper + pitch_lowwer) / 2.0))+margin;                                  
    const double  gradient_envelope_pitch=2*(  pitch-( (pitch_upper+pitch_lowwer ) / 2 ) );

    m_output_flightenvelope(0)=stall_speed;
    m_output_flightenvelope(1)=pitch_sp_after_fev;
    m_output_flightenvelope(2)=fev_pitch;
    m_output_flightenvelope(3)=gradient_envelope_pitch;

    return  m_output_flightenvelope ;





 }
