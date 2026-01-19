#ifndef AGENT_H
#define AGENT_H

#include <Fixed_wing_aircraft_model/FixedwingDynamics.h>
#include <Fixed_wing_aircraft_model/CurrentState.h>
#include <controller/MainController.h>
#include <formation_guidance/MainFormationGuidance.h>
#include <collision_avoidance/APF/APFParameter.h>
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
          const APFParameter_t& g_APFParameter);  

    ~Agent() = default;

    /*충돌회피 모드 설정*/
    void set_collision_avoidance_mode(bool mode){m_APF_mode=mode; }


    /*시뮬레이션으로 부터 정보를 수신*/
    void receive_state_from_sim(const matrix::Vector<double, 11>& state)
    {agent_state=state;};



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
            agent_state
        );
    }

    /* 다른 에이전트들에게 자신의 정보를 발신*/
    const TsMessage_t& broadcast_ts_message() const { 
    return m_ts_message; 
        }



    /*실질적인 제어 입력 연산 */
    void compute_control();

    matrix::Vector<double, 4>send_control_input_to_sim()
    {   compute_control();
        return m_agent_control_input;};


    



private:
    /* 시뮬레이션으로 부터 받는 값*/
    matrix::Vector<double, 11> agent_state{};

    /*맴버 변수*/
    int m_agent_id; /* 해당 개체의 인덱스*/
    FixedwingDynamics m_dynamics;
    MainFormationGuidance m_guidance;
    MainController m_controller;
    

   
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
    matrix::Vector<double, 2> m_desired_position{};
    matrix::Vector<double, 11> m_setpoint{};   

    /* 충돌회피 모드*/
    bool m_APF_mode=false;
   



    



    /*시뮬레에션에게 보낼 출력값*/
    matrix::Vector<double, 4> m_agent_control_input{};





};

#endif