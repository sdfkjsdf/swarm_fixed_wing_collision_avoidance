#ifndef TSMESSAGE_H
#define TSMESSAGE_H
#include <cmath>
#include<matrix/math.hpp>
#include <iostream>
#include <iomanip> // std::setprecision을 위해 반드시 필요


struct TsMessage_t
{  double m_ts_time;         /*전송을 하는 시간을 의미*/
   double m_ts_north;
   double m_ts_east;
   double m_ts_north_speed;
   double m_ts_east_speed;
   double m_ts_yaw;
   double m_ts_current_yawrate; 

   int m_sender_index; /*전송자의 index을 의미*/

   bool m_leader_follower_activate_bit; /*leader-follower 자체가 켜졌는지 */
   bool m_am_i_leader_bit;     /*전송자가 리더인지 아닌지를 판별 */


/* 초기화 선언*/

 TsMessage_t()
   :m_ts_time(0.0),
    m_ts_north(0.0),
    m_ts_east(0.0),
    m_ts_north_speed(0.0),
    m_ts_east_speed(0.0),
    m_ts_yaw(0.0),
    m_ts_current_yawrate(0.0),
    m_sender_index(0),
    m_leader_follower_activate_bit(false),
    m_am_i_leader_bit(false){}


 void generate_ts_message(int const sender_index,
                           double const time,
                           bool const leader_follower_switch,
                           bool const who_is_leader,
                           matrix::Vector<double,11> const& state)
   
    {   
        const double roll = state(3);
        const double pitch = state(4);
        const double yaw = state(5);
        const double speed = state(6);
        const double q = state(8);
        const double r = state(9);
        const double c_r = std::cos(roll);
        const double s_r = std::sin(roll);
        const double c_p = std::cos(pitch);
        const double s_p = std::sin(pitch);
        const double c_y = std::cos(yaw);
        const double s_y = std::sin(yaw);

        m_ts_time = time ;
        m_sender_index= sender_index;
        m_leader_follower_activate_bit=leader_follower_switch;
        m_am_i_leader_bit=who_is_leader;
        m_ts_north = state(0);

        m_ts_east  = state(1);
        m_ts_north_speed = speed * c_p* c_y ;
        m_ts_east_speed  = speed * c_p * s_y;
        m_ts_yaw=state(5);
        m_ts_current_yawrate=((q*s_r)+(r*c_r))/c_p ; 
    }

     void print() const {
        std::cout << "================ [TsMessage Info] ================" << std::endl;
        std::cout << std::fixed << std::setprecision(4); // 소수점 4자리 고정
        
        // 변수명을 멤버 변수(m_...)와 일치시킴
        std::cout << " ID               : " << m_sender_index << std::endl;
        std::cout << " Time             : " << m_ts_time << " s" << std::endl;
        
        std::cout << " Position (N, E)  : (" << m_ts_north << ", " << m_ts_east << ") m" << std::endl;
        
        // ts_south_speed는 정의되어 있지 않으므로 m_ts_east_speed로 수정 (또는 논리에 맞게 수정)
        std::cout << " Velocity (N, E)  : (" << m_ts_north_speed << ", " << m_ts_east_speed << ") m/s" << std::endl;
        
        std::cout << " Yaw              : " << m_ts_yaw << " rad" << std::endl;
        std::cout << " Yaw Rate         : " << m_ts_current_yawrate << " rad/s" << std::endl;
        
        std::cout << " Leader/Follower  : " << (m_leader_follower_activate_bit ? "ON" : "OFF") << std::endl;
        std::cout << " Am I Leader?     : " << (m_am_i_leader_bit ? "YES" : "NO") << std::endl;
        
        std::cout << "==================================================" << std::endl;
    }











 };


#endif