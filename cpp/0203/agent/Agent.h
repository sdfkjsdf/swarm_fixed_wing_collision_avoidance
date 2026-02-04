#ifndef AGENT_H
#define AGENT_H

#include <Fixed_wing_aircraft_model/FixedwingDynamics.h>

#include <Fixed_wing_aircraft_model/CurrentState.h>
#include <controller/MainController.h>
#include <formation_guidance/MainFormationGuidance.h>
#include <propagation/DynamicsLinearization.h>


#include <collision_avoidance/APF/APFParameter.h>
#include <collision_avoidance/control_barrier_function/SafetyBarrierCertificate.h>

#include <ts_message/TsMessage.h>
#include <ts_message/TcMessage.h>
#include <make_csv/FixedWingLog.h>
#include<matrix/math.hpp>

class Agent {
public:
    
    Agent(int agent_id,
          const FixedwingSpec_t& g_FixedwingSpec,
          const Flockingparameter_t& g_Flockingparameter,
          const LeaderFollowerParameter_t& g_LeaderFollowerParameter,
          const GainParameter_t& g_GainParameter,
          const NdiParameter_t& g_NdiParameter,
          const APFParameter_t& g_APFParameter,
          const double dt);  

    ~Agent() = default;

    /*충돌회피 모드 설정*/
    void set_collision_avoidance_mode(bool mode){m_APF_mode=mode; }


/* 통신 관련 내용 시작*/


    /*시뮬레이션으로 부터 정보를 수신*/
    void receive_state_from_sim(const matrix::Vector<double, 11>& state)
    {m_agent_state=state;};



    /*지상국으로 부터 통신 수신*/
    void receive_tc_message(const TcMessage_t& tc_message) {
        m_tc_message = tc_message;
        m_time=tc_message.m_time;
        m_leader_follower_switch = tc_message.m_leader_follower_activate_bit;
        m_am_i_leader = tc_message.m_am_i_leader_bit;
    }
    /* 지상국으로 부터 포메이션 정보 수집*/
    void receive_wingman_setpoint(const matrix::Vector<double, 2>& desired_position )
    {
     m_desired_position=desired_position;

    }

    /*지상국에서 바로 보내주는 정보*/
    void receive_state_setpoint(const matrix::Vector<double, 11>& setpoint )
    {
     m_setpoint=setpoint;

    }



    /*다른 에이전트로부터 통신 수신*/

    void receive_ts_message(const TsMessage_t& other_information1,
                            const TsMessage_t& other_information2,
                            const TsMessage_t& other_information3,
                            const TsMessage_t& other_information4 ) 
    {
        m_received_other_information1=other_information1;
        m_received_other_information2=other_information2;
        m_received_other_information3=other_information3;
        m_received_other_information4=other_information4;

    }

    

   
   /*다른 에이전트에게 전송할 메시지를 생성*/
    void generate_ts_message(double time) {
        m_ts_message.generate_ts_message(
            m_agent_id, /* 이 메시지를 보내는 주체의 인덱스*/
            m_time,
            m_tc_message.m_leader_follower_activate_bit,  
            m_tc_message.m_am_i_leader_bit,               
            m_agent_state
        );
    }

    /* 다른 에이전트들에게 자신의 정보를 발신*/
    const TsMessage_t& broadcast_ts_message() const { 
    return m_ts_message; 
        }



/*통신 관련 내용 종료 */



    /*실질적인 제어 입력 연산 */
    void compute_control();
    /* 나중에 혹시나 Closed form backup cbf을 쓰는 경우 예측을 해야함으로*/
    void zoh_propagation_next_step();

    /*CBF 연산을 위한 파트임*/
    /*일단 지금은 제어 가능성을 위해서 확인만하는 단계임*/
    /*해당 함수를 돌리면 일단 현재 출력이 되는 것은 Lgh임*/
    void caluclate_safety_certificate(const matrix::Vector<double, 11>& state,const TsMessage_t& other_information1);



    /*적분기 및 상태 업데이트는 시뮬레이션 환경에서 이루어짐으로 자신의 정보를 시뮬레이션 환경에게 전달*/

    matrix::Vector<double, 4>send_control_input_to_sim()
    {   compute_control();
        return m_agent_control_input;
    };


    
    matrix::Vector<double,11>send_zoh_next_step()

    {
        zoh_propagation_next_step();
        return m_agent_zoh_next_state;



    };


    

        
private:
    /* 시뮬레이션으로 부터 받는 값*/
    matrix::Vector<double, 11> m_agent_state{};

    /*맴버 변수*/
    int m_agent_id; /* 해당 개체의 인덱스*/
    double m_dt;
    FixedwingDynamics m_dynamics;

    /*cbf 관련 출력값들 저장*/

     matrix::Matrix<double,1,4> m_lgh_i{};
     double m_lfh_i=0.0;






    /* 모듈 탑재 부분*/
    MainFormationGuidance m_guidance;
    MainController m_controller;
    DynamicsLinearization m_DynamicsLinearization; 
    SafetyBarrierCertificate m_SafetyBarrierCertificate;
    /* 지금 zoh 탑제 부분임 원래 적분기는 지금 시뮬레이션 상에 다는 것이 맞지만 나중에 backup cbf을 쓰는 경우 컴퓨터 내부에서 프로파게이션 해야 함으로  */
 
    

   
    /* 통신 관련 내용  */

    TsMessage_t m_ts_message{};
    TcMessage_t m_tc_message{};
    bool m_leader_follower_switch = false;   // Leader-Follower 활성화 여부
    bool m_am_i_leader = false;   // 내가 리더인지
    double m_time=0.0;
    TsMessage_t m_received_other_information1{};
    TsMessage_t m_received_other_information2{};
    TsMessage_t m_received_other_information3{};
    TsMessage_t m_received_other_information4{};

    /*제어관련 내용*/
    matrix::Vector<double, 2> m_desired_position{};
    matrix::Vector<double, 11> m_setpoint{};   

    /* 충돌회피 모드*/
    bool m_APF_mode=false;
   


    /*시뮬레에션에게 보낼 출력값*/
    matrix::Vector<double, 4> m_agent_control_input{};
    matrix::Vector<double,11> m_agent_zoh_next_state{};  /* 지금 zoh을 통해서 받은 것임*/





};

#endif