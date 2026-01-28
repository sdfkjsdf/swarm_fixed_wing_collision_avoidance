/*

 작성자 : 이동혁    
작성날짜 : 2026/01/09일 
기능 세부 설명 : Rollrate_setpoint Pitchrate_setpoint을 생성 및 동시에 Flight envelope 을 적용

[입력 관련 주의사항]
해당 함수에서 roll_pitch_sp(0)은 롤각 setpoint,rollpitch_sp(1)은 피치각 setpoint가 들어가야 하며
현재 들어가는 피치각 setpoint인 경우 반드시  FlightEnvelope을 통해서 한번 필터링된 값이 들어가야 한다

[출력 관련 내용]
해당 함수의 출력값은 yawrate_sp 와 함께 (3,1)로 생성 후 -> BodyAxisRateSetpoint 의 입력으로 사용

[수정 사유]
:현재 "Investigation of Practical Flight Envelope Protection Systemsfor Small Aircraft 논문에 따르면 
지금  q_cmd에서 FEV을 위헤서 saturation 을 하는 것이아니라 현재 pitchrate_setpoint에서 바로 적용하는 것임 


*/


#include <controller/RollratePitchrateSetpoint.h>

RollratePitchrateSetpoint::RollratePitchrateSetpoint(const GainParameter_t& gain)
    :m_gain(gain){}

 matrix::Vector<double,2>RollratePitchrateSetpoint::calculate_rollrate_pitchrate(
                                                                                 const matrix::Vector<double,11>& state,
                                                                                 const matrix::Vector<double,2>& roll_pitch_sp,
                                                                                 const double& fev_pitch,
                                                                                 const double& gradient_fev_pitch)
 {
   /*출력값 초기화 */
   m_output_rate_of_rollpitch.setZero();

   /*상태관련 내용 받아오기*/

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
   /*Gain 파라미터 관련 내용 받아오기*/
    const double k_roll =m_gain.m_k_roll ;
    const double k_pitch =m_gain.m_k_pitch ;

   /*이전에 받아온 roll,pitch setpoint 받아오기*/
    const double roll_sp  = roll_pitch_sp(0);
    const double pitch_sp = roll_pitch_sp(1);

   /*FEV 관련 내용을 받아오기*/
   const double lyaponove_pitch = fev_pitch;
   const double gradient_lyaponove = gradient_fev_pitch;
   const double tol = 1e-6;

    
   /*FEV 적용 전의 값*/
    const double rollrate_sp=k_roll*(roll_sp-roll);
    const double pitchrate_sp_rar=k_pitch*(pitch_sp-pitch);



   m_output_rate_of_rollpitch(0)=rollrate_sp;


   /*Fev 적용 */

   bool islyaponove = (gradient_lyaponove<-tol);
   bool isgradient  = (gradient_lyaponove*pitchrate_sp_rar<tol) && (std::abs(gradient_lyaponove)<tol);

   bool pass_cond= islyaponove || isgradient ;


   if (pass_cond)
   {

      m_output_rate_of_rollpitch(1)=pitchrate_sp_rar;


   }
   else
   {

      m_output_rate_of_rollpitch(1)=0.0;

   }

   
    

    return m_output_rate_of_rollpitch;




 }
