#include <collision_avoidance/APF/APFCollisionAvoidance.h>

APFCollisionAvoidance::APFCollisionAvoidance(const APFParameter_t& APF_Parameter) 
    :m_APF_parameter(APF_Parameter){}


 matrix::Vector<double,3> APFCollisionAvoidance :: calculate_repulsive_effect(const matrix::Vector<double,11>& state,
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

    /*파라미터를 설정*/

        const double alpha =m_APF_parameter.m_alpha;
        const double a=m_APF_parameter.m_a;
        const double b=m_APF_parameter.m_b;

    
    /* 자신의 좌표계로 변환*/
        matrix::Vector<double,2> nose_direction{};
        matrix::Vector<double,2> wing_direction{};
            nose_direction(0)=c_y;
            nose_direction(1)=s_y;
            wing_direction(0)=-s_y;
            wing_direction(1)=c_y;

    /*일단 받은 신호을 묶기 */
    TsMessage_t total_message[4]={other_information1,other_information2,other_information3,other_information4};

    /*APF 시작 */

    matrix::Vector<double,2> NE_coordination_accleration{};

    for(int i =0;i<4;i++)
{ 

  matrix::Vector<double,2> relative_position= matrix::Vector<double,2>(StateInformation::get_state_other_state(total_message[i]).slice<2,1>(0,0))-  self_position ;  
  
  
  double norm_position = relative_position.norm();
 
 

  double APF_gain =  -alpha / ( 1.0 + std::pow( (norm_position/a), 2.0*b) );

  matrix::Vector<double,2> u1=APF_gain*(relative_position/norm_position); 

   

  NE_coordination_accleration+=(u1);

}

double ground_speed = self_velocity.norm();

double ground_acclertion_setpoint=nose_direction*NE_coordination_accleration;


double yawrate_setpoint_rar=(wing_direction*NE_coordination_accleration)/ground_speed;




 m_yawrate_ground_acc_ground_speed_APF(0)=yawrate_setpoint_rar;
 m_yawrate_ground_acc_ground_speed_APF(1)=ground_acclertion_setpoint;
 m_yawrate_ground_acc_ground_speed_APF(2)=ground_speed;


return m_yawrate_ground_acc_ground_speed_APF;





    }
