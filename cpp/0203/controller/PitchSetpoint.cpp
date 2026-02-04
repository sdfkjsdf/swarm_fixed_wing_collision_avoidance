/******************************************************** */
/* 작성자 : 이동혁    */
/* 작성날짜 : 2026/01/09일 */
/* 기능 세부 설명 : pitch_setpoint을 생성"
해당 함수의  출력값은 RollratePitchrateSetpoint.cpp으로 들어간다

수정 사유
: Formation_control에서 실질적인 출력값이 rollsp,pitchrate,d(Ground_speed)/dt 이고 고도채널은 따로 조정함
: 따라서 기존의 코드 처럼 N,E,D,V를 가지고 CASCADE 구조로 내려 가는 것이 아님 

/********************************************************* */

#include <matrix/math.hpp>
#include "PitchSetpoint.h"
#include <Fixed_wing_aircraft_model/FixedwingSpec.h>
#include <controller/GainParameter.h>
#include <cmath>
#include <algorithm>

PitchSetPoint::PitchSetPoint(const FixedwingSpec_t& spec, const GainParameter_t& gain)
        :m_spec(spec),
        m_gain(gain)
{

}


 double PitchSetPoint::calculate_pitch_sp(const matrix::Vector<double, 11>& state, const matrix::Vector<double, 11>& setpoint)

 {

    m_pitch_setpoint = 0.0;

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


    /*각도 변환 함수 시작 */
    const double c_r = std::cos(roll);
    const double s_r = std::sin(roll);
    const double c_p = std::cos(pitch);
    const double s_p = std::sin(pitch);
    const double c_y = std::cos(yaw);
    const double s_y = std::sin(yaw);
    const double tan_p = std::tan(pitch);


    /*기체 spec 관련 내용을 받아오기 */
    const double max_pitch=m_spec.max_pitch;
    const double min_pitch=m_spec.min_pitch;
    const double g = m_spec.g;

    /*파라미터 값 받아오기*/

    const double k_d = m_gain.m_k_d;

    /* set point 받아오기 */    
    const double d_set=setpoint(2);

    /*pitch_sp 연산 시작  */

    const double error_d=d_set-d;
    const double dot_error_d=k_d*error_d;
    const double arg = std::clamp((dot_error_d/speed),-1.0,1.0); /*asin 연산전에 범위 맞추어 주기 */
    const double pitch_sp_rar=-std::asin(arg);

    const double pitch_sp=std::clamp(pitch_sp_rar,min_pitch,max_pitch);

    m_pitch_setpoint = pitch_sp; /* 아직 fev가 들어가기 전임 */

    /*pitch_sp 연산 종료*/

    return m_pitch_setpoint;

 }