#include "MainController.h"
#include <cmath>
#include <algorithm>

MainController::MainController(
    const FixedwingSpec_t& spec,
    const GainParameter_t& gain,
    const NdiParameter_t& ndi
) : 
     m_spec(spec),
    m_gain(gain),
    m_ndi(ndi),
    m_pitchsetpoint(spec, gain),
    m_rollratepitchratesetpoint(gain),        
    m_flightenvelope(spec),
    m_bodyaxisratesetpoint(spec),
    m_thrustsetpoint(spec,gain)
   
{}

matrix::Vector<double, 4> MainController::calculate_control_input(
    const matrix::Vector<double, 11>& state,
    const matrix::Vector<double, 11>& setpoint,
    const matrix::Vector<double,3>& guidance_setpoint)
    { 


         m_output_maincontroller.setZero();

        
         const double roll_setpoint=guidance_setpoint(0);
         const double yawrate_setpoint =guidance_setpoint(1);
         const double dot_ground_speed_setpoint=guidance_setpoint(2);



       m_pitchsetpoint.calculate_pitch_sp(state,setpoint );
       /*PitchSetpoint.cpp에 구현이 된 내용*/
       /*calculate_pitch_sp(const matrix::Vector<double, 11>& state, const matrix::Vector<double, 11>& setpoint)*/
            const double  pitch_setpoint_before_fev= m_pitchsetpoint.get_pitch_set();


    /* 비행영역 보호 시작 */

     m_flightenvelope.apply_FlightEnvelope(state,roll_setpoint,pitch_setpoint_before_fev);
     /*FlightEnvelope.cpp 에서 구현 */
     /*matrix::Vector<double,4> apply_FlightEnvelope(const matrix::Vector<double,11>& state,
                                                      const double& roll_setpoint,
                                                      const double& pitch_setpoint_before_fev);*/

        const double stall_speed =m_flightenvelope.get_stall_speed();
        const double pitch_setpoint_after_fev =m_flightenvelope.get_pitch_sp_after_fev();
        const double lyapunov_function_value_pitch_fev=m_flightenvelope.get_fev_pitch();
        const double gradient_lyapunov = m_flightenvelope.get_gradient_envelope_pitch();

     matrix::Vector<double,2> roll_pitch_setpoint_after_fev{};
        roll_pitch_setpoint_after_fev(0)=roll_setpoint;
        roll_pitch_setpoint_after_fev(1)=pitch_setpoint_after_fev;

    /*rollrate,pitchrate,yawrate 연산 및 결합*/

     m_rollratepitchratesetpoint.calculate_rollrate_pitchrate(state,roll_pitch_setpoint_after_fev,lyapunov_function_value_pitch_fev,gradient_lyapunov);
     /*RollratePitchrateSetpoint.cpp에서 구현*/
     /*matrix::Vector<double,2> calculate_rollrate_pitchrate(const matrix::Vector<double,11>& state, 
                                                             const matrix::Vector<double,2>& roll_pitch_sp ,
                                                             const double& fev_pitch,
                                                             const double& gradient_fev_pitch );*/


        const double rollrate_setpoint=m_rollratepitchratesetpoint.get_rollrate_sp();
        const double pitchrate_setpoint=m_rollratepitchratesetpoint.get_pitchrate_sp();

     matrix::Vector<double,3> setpoint_rate_of_roll_pitch_yaw{};

        setpoint_rate_of_roll_pitch_yaw(0)=rollrate_setpoint;
        setpoint_rate_of_roll_pitch_yaw(1)=pitchrate_setpoint;
        setpoint_rate_of_roll_pitch_yaw(2)=yawrate_setpoint;


    /*Body axis rate 연산*/
    
    m_bodyaxisratesetpoint.calculate_bodyaxisrate(state,setpoint_rate_of_roll_pitch_yaw);
    /*BodyAxisRateSetpoint.cpp에서 구현*/
    /*matrix::Vector<double,3> calculate_bodyaxisrate(const matrix::Vector<double,11>& state,const matrix::Vector<double,3>& rollrate_pitchrate_yawrate );*/


        const double p_setpoint=m_bodyaxisratesetpoint.get_p_setpoint();
              double q_setpoint=m_bodyaxisratesetpoint.get_q_setpoint(); /* 여기서 const 을 안한 이유는 나종에  lyapunov_function_value_pitch_fev,  gradient_lyapunov 이거에 따라서 saturattion 됨 */
        const double r_setpoint=m_bodyaxisratesetpoint.get_r_setpoint(); 

    
    
    /* thrust channel 연산 */
    m_thrustsetpoint.calculate_at_sp(state,stall_speed,dot_ground_speed_setpoint);
    /*ThrustSetpoin.cpp 에서 구현*/
    /*calculate_at_sp(const matrix::Vector<double,11>& state ,  const double& stall_speed , const double& dot_ground_speed_setpoint );*/
    
        const double at_setpoint=m_thrustsetpoint.get_at_sp();


    /* NDI 시작*/
        const double p = state(7);
        const double q = state(8);
        const double r = state(9);
        const double at = state(10);



        const double t_p =m_spec.timeconstant_p;
        const double t_q =m_spec.timeconstant_q;
        const double t_r =m_spec.timeconstant_r;
        const double t_a =m_spec.timeconstant_at;

        const double p_max=m_spec.p_max;
        const double q_max=m_spec.q_max;
        const double r_max=m_spec.r_max;
        const double at_max=m_spec.at_max;

        const double p_min=m_spec.p_min;
        const double q_min=m_spec.q_min;
        const double r_min=m_spec.r_min;
        const double at_min=m_spec.at_min;

        const double k_p_sp=m_ndi.m_k_p_sp;
        const double k_q_sp=m_ndi.m_k_q_sp;
        const double k_r_sp=m_ndi.m_k_r_sp;
        const double k_at_sp=m_ndi.m_k_at_sp;


        const double dot_p_sp=k_p_sp*(p_setpoint-p);
        const double dot_q_sp=k_q_sp*(q_setpoint-q);
        const double dot_r_sp=k_r_sp*(r_setpoint-r);
        const double dot_at_sp=k_at_sp*(at_setpoint-at);

        const double p_cmd_rar = p + (t_p*dot_p_sp);
        const double q_cmd_rar = q + (t_q*dot_q_sp);
        const double r_cmd_rar = r + (t_r*dot_r_sp);
        const double at_cmd_rar= at+ (t_a*dot_at_sp);

        const double p_cmd= std::clamp(p_cmd_rar,p_min,p_max);
        const double q_cmd= std::clamp(q_cmd_rar,q_min,q_max);
        const double r_cmd= std::clamp(r_cmd_rar,r_min,r_max);
        const double at_cmd =std::clamp(at_cmd_rar,at_min,at_max);

    m_output_maincontroller(0)=p_cmd;
    m_output_maincontroller(1)=q_cmd;
    m_output_maincontroller(2)=r_cmd;
    m_output_maincontroller(3)=at_cmd;


    return m_output_maincontroller;



    }

