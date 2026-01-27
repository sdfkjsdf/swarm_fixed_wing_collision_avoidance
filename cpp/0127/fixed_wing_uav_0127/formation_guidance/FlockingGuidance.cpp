#include <matrix/math.hpp>
#include <formation_guidance/FlockingGuidance.h>
#include <formation_guidance/FlockingParameter.h>
#include <fixed_wing_aircraft_model/FixedwingSpec.h>
#include <ts_message/TsMessage.h>
#include <ts_message/StateInformation.h>
#include<cmath>

FlockingGuidance::FlockingGuidance(const FixedwingSpec_t& spec ,const Flockingparameter_t& flocking_gain)
            :m_spec(spec),
            m_flocking_gain(flocking_gain) {}

 matrix::Vector<double,3> FlockingGuidance::caclulate_yawrate_ground_acc_ground_speed(const matrix::Vector<double,11>& state,
                                                                    const TsMessage_t other_information1,
                                                                    const TsMessage_t other_information2,
                                                                    const TsMessage_t other_information3,
                                                                    const TsMessage_t other_information4)
{
/*자신의 정보를 받아오기 */
matrix::Vector<double,2> self_position{};
matrix::Vector<double,2> self_velocity{};

    const double n = state(0);
    const double e = state(1);
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


    self_position(0)=n;
    self_position(1)=e;
    self_velocity(0)=speed * c_p* c_y ;
    self_velocity(1)=speed * c_p * s_y;



/*flocking 파라미터를 설정*/
   const double lammda=m_flocking_gain.m_lammda;
   const double beta =m_flocking_gain.m_beta;
   const double k_1=m_flocking_gain.m_k1;
   const double k_2=m_flocking_gain.m_k2;
   const double desired_distance=m_flocking_gain.m_desired_distance;

/* 기체 spec 불러오기*/
   const double g =m_spec.g;
   const double max_roll=m_spec.max_roll;

/* 자신의 좌표계로 변환*/
matrix::Vector<double,2> nose_direction{};
matrix::Vector<double,2> wing_direction{};
    nose_direction(0)=c_y;
    nose_direction(1)=s_y;
    wing_direction(0)=-s_y;
    wing_direction(1)=c_y;

/*일단 받은 신호을 묶기 */
TsMessage_t total_message[4]={other_information1,other_information2,other_information3,other_information4};

/* Ts 메시지 자료 구조*/

        // struct TsMessage_t
        // {  double m_ts_time;         /*전송을 하는 시간을 의미*/
        //    double m_ts_north;
        //    double m_ts_east;
        //    double m_ts_north_speed;
        //    double m_ts_east_speed;
        //    double m_ts_yaw;
        //    double m_ts_current_yawrate; 

        //    int m_sender_index; /*전송자의 index을 의미*/

        //    bool m_leader_follower_activate_bit; /*leader-follower 자체가 켜졌는지 */
        //    bool m_am_i_leader_bit;     /*전송자가 리더인지 아닌지를 판별 */




/*Flocking 제어입력 시작*/

matrix::Vector<double,2> NE_coordination_accleration{};



for(int i =0;i<4;i++)
{  
  matrix::Vector<double,2> other_position{};
  other_position(0)=total_message[i].m_ts_north;
  other_position(1)=total_message[i].m_ts_east;

  matrix::Vector<double,2> other_velocity{};
  other_velocity(0)=total_message[i].m_ts_north_speed;
  other_velocity(1)=total_message[i].m_ts_east_speed;


  

  matrix::Vector<double,2> relative_position= other_position -  self_position ;  
  matrix::Vector<double,2> relative_velocity= other_velocity -  self_velocity ;  
  
  double norm_position = relative_position.norm();
  double norm_position_squared = relative_position.norm_squared(); 
  double dot_pos_vel   = relative_position*relative_velocity;

  /*얼라이먼트 항 연산 */
  double alignment_gain =  lammda / std::pow(1.0 + norm_position * norm_position, beta);

  matrix::Vector<double,2> u1=alignment_gain*(relative_velocity); 

   
  /*cohesion and sepertaion*/
  double u2_scalar = (k_1 / (2.0 * norm_position_squared)) * dot_pos_vel;
  matrix::Vector<double,2> u2 = u2_scalar * (relative_position);

   double u3_scalar = (k_2 / (2.0 * norm_position)) * (norm_position - 2.0 * desired_distance);
   matrix::Vector<double,2> u3 = u3_scalar * (relative_position);

  NE_coordination_accleration+=(u1+u2+u3);

}
/* 총 에이전트 대수로 나누기 */
NE_coordination_accleration=NE_coordination_accleration/5.0;
/*
static matrix::Vector<double,6> get_state_other_state(const TsMessage_t& ts_message )
    
    { matrix::Vector<double,6> output{};
        output(0)=ts_message.m_ts_north;
        output(1)=ts_message.m_ts_east;
        output(2)=ts_message.m_ts_north_speed;
        output(3)=ts_message.m_ts_east_speed;
        output(4)=ts_message.m_ts_yaw;
        output(5)=ts_message.m_ts_current_yawrate;

        return output;
    };

*/


/*실질적인 flocking 제어입력으로 전환*/

double ground_speed = self_velocity.norm();

double ground_acclertion_setpoint=nose_direction*NE_coordination_accleration;


double yawrate_setpoint_rar=(wing_direction*NE_coordination_accleration)/ground_speed;








m_yawrate_ground_acc_ground_speed(0)=yawrate_setpoint_rar;
m_yawrate_ground_acc_ground_speed(1)=ground_acclertion_setpoint;
m_yawrate_ground_acc_ground_speed(2)=ground_speed;


return m_yawrate_ground_acc_ground_speed;










}

   








                                                                    





