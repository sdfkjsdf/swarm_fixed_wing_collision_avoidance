#ifndef TCMESSAGE_H
#define TCMESSAGE_H
#include <iostream>
#include <iomanip> // std::setprecision을 위해 반드시 필

struct TcMessage_t
{   
    int  m_receiver;
    double m_time;

    bool m_leader_follower_activate_bit;
    bool m_am_i_leader_bit;

    TcMessage_t()
        :m_time(0.0),
         m_receiver(0),
         m_leader_follower_activate_bit(false),
         m_am_i_leader_bit(false){}


   void broadcast_tc_message(int const receiver,double const time,bool const leader_follower_switch,bool const who_is_leader)
        {
            m_time=time;
            m_receiver=receiver;
            m_leader_follower_activate_bit=leader_follower_switch;
            m_am_i_leader_bit=who_is_leader;




        }
    bool get_leaderfollower_switch(){return m_am_i_leader_bit;};
    bool get_leader_index(){return m_am_i_leader_bit;};


     void print() const {
        std::cout << "================ [TsMessage Info] ================" << std::endl;
        std::cout << std::fixed << std::setprecision(4); // 소수점 4자리 고정
        
        // 변수명을 멤버 변수(m_...)와 일치시킴
        std::cout << " ID               : " << m_receiver << std::endl;
        std::cout << " Time             : " << m_time << " s" << std::endl;
        
        std::cout << " Leader/Follower  : " << (m_leader_follower_activate_bit ? "ON" : "OFF") << std::endl;
        std::cout << " Am I Leader?     : " << (m_am_i_leader_bit ? "YES" : "NO") << std::endl;
        
        std::cout << "==================================================" << std::endl;


     }







};







#endif