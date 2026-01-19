#include <matrix/math.hpp>
#include <formation_guidance/LeaderFollowerGuidance.h>
#include <formation_guidance/LeaderFollowerParameter.h>
#include <fixed_wing_aircraft_model/FixedwingSpec.h>
#include <ts_message/TsMessage.h>
#include <ts_message/StateInformation.h>
#include<cmath>

LeaderFollowerGuidance::LeaderFollowerGuidance(  const FixedwingSpec_t& spec, const LeaderFollowerParameter_t& leaderFollower_gain)
    : m_spec(spec),
      m_leaderfollower_gain(leaderFollower_gain) {}

 matrix::Vector<double,3> LeaderFollowerGuidance::caclulate_yawrate_ground_acc_ground_speed(
                                                                    bool const am_i_leader,
                                                                    const matrix::Vector<double,2>& desired_position,
                                                                    const matrix::Vector<double,11>& state,
                                                                    const TsMessage_t other_information1,
                                                                    const TsMessage_t other_information2,
                                                                    const TsMessage_t other_information3,
                                                                    const TsMessage_t other_information4 )
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

                    /* 자신의  ground speed 을 정의 */
                    const double ground_speed=self_velocity.norm();



    if (am_i_leader)
    
    
    {
        m_yawrate_ground_acc_ground_speed(0)=0.0;
        m_yawrate_ground_acc_ground_speed(1)=0.0;
        m_yawrate_ground_acc_ground_speed(2)=ground_speed;
        return  m_yawrate_ground_acc_ground_speed;



    }

    else
    {    
        /* leaderfollower   파라미터를 설정*/
                    const double  k_v=m_leaderfollower_gain.m_k_v;
                    const double  k_dy=m_leaderfollower_gain.m_k_dy;
                    const double  k_dot_dy = m_leaderfollower_gain.m_k_dot_dy;
                    const double  k_dx = m_leaderfollower_gain.m_k_dx;
                    const double  lookahead_distance=m_leaderfollower_gain.m_lookahead_distance;
                    const double  k_smooth=m_leaderfollower_gain.m_k_smooth;

        /*OFFSET을 즉각 정의*/
                    const double forward_offset=2.0*lookahead_distance;

        /* 기체 spec 불러오기*/
                    const double g =m_spec.g;
                    const double max_roll=m_spec.max_roll;


        /*일단 받은 신호을 묶기 */
            TsMessage_t total_message[4]={other_information1,other_information2,other_information3,other_information4};
            double leader_yaw=0.0;
            double leader_yawrate=0.0;
            matrix::Vector<double,2> leader_position{};
            matrix::Vector<double,2> leader_velocity{};


                /*누가 리더인지 찾기 */
                    for (int i=0;i<4;i++)
                    {
                        if (total_message[i].m_am_i_leader_bit)
                        {
                            leader_yaw=total_message[i].m_ts_yaw;
                            leader_yawrate=total_message[i].m_ts_current_yawrate;
                            leader_position(0)=total_message[i].m_ts_north;
                            leader_position(1)=total_message[i].m_ts_east;
                            leader_velocity(0)=total_message[i].m_ts_north_speed;
                            leader_velocity(1)=total_message[i].m_ts_east_speed;

                        }
                    }

                const double c_y_leader = std::cos(leader_yaw);
                const double s_y_leader = std::sin(leader_yaw);

                matrix::Matrix<double,2,2> rotation_matrix{};

                rotation_matrix(0,0) = c_y_leader;
                rotation_matrix(0,1) = s_y_leader;
                rotation_matrix(1,0) = -1.0*s_y_leader;
                rotation_matrix(1,1) = c_y_leader;

                  
                    
                /*error 구하기*/

                    matrix::Vector<double,2> error_position{};
                    matrix::Vector<double,2> error_velocity{};

                    error_position = self_position - leader_position;
                    error_velocity = self_velocity - leader_velocity;

                    matrix::Vector<double,2> Dx_Dy = desired_position - (rotation_matrix*error_position);

                    const double Dx= Dx_Dy(0);
                    const double Dy= Dx_Dy(1);

                    const double dot_Dy= leader_yawrate * ( matrix::Vector<double,2>(rotation_matrix.slice<1,2>(0,0)  ) *  error_position    ) 
                                          - ( matrix::Vector<double,2>(rotation_matrix.slice<1,2>(1,0)  ) *  error_velocity    );

                    const double target_follower_distance=Dx_Dy.norm();

                    const double weight = 0.5 *(1-std::tanh(k_smooth*(target_follower_distance-lookahead_distance)));

                    const double Dx_eff=Dx+(weight*forward_offset);

                    const double yaw_los_rar=std::atan2(k_dy*Dy+k_dot_dy*dot_Dy,Dx_eff);
                    const double yaw_los=std::atan2(std::sin(yaw_los_rar),std::cos(yaw_los_rar) );

                    const double error_yaw_leader_follower=std::atan2(std::sin(leader_yaw-yaw),std::cos(leader_yaw-yaw) );

                    const double yaw_relative_cmd_rar=std::atan2(std::sin(yaw_los+error_yaw_leader_follower),std::cos(yaw_los+error_yaw_leader_follower) );
                    const double yaw_relative_cmd=std::atan2(std::sin(yaw_relative_cmd_rar),std::cos(yaw_relative_cmd_rar) );


                    
                    const double yawrate_setpoint_rar=(2*ground_speed*std::sin(yaw_relative_cmd))/lookahead_distance;


                    /* roll_setpoint,yawrate_setpoint 구하기 */
            
                    /*ground_accleration 구하기*/

                    const double ground_speed_setpoint = leader_velocity.norm() + k_dx*Dx;
                    const double ground_accleration_setpoint=k_v*(ground_speed_setpoint-ground_speed);



                   
                     m_yawrate_ground_acc_ground_speed(0)=yawrate_setpoint_rar;
                     m_yawrate_ground_acc_ground_speed(1)=ground_accleration_setpoint;
                     m_yawrate_ground_acc_ground_speed(2)=ground_speed;








                    return m_yawrate_ground_acc_ground_speed;



    }





}

   








                                                                    





