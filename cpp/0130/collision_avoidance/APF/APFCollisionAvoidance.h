#ifndef APFCOLLISIONAVOIDANCE_H
#define APFCOLLISIONAVOIDANCE_H
#include<matrix/math.hpp>
#include<collision_avoidance/APF/APFParameter.h>
#include <ts_message/TsMessage.h>
#include <ts_message/StateInformation.h>


class APFCollisionAvoidance
{
    public:
    APFCollisionAvoidance(const APFParameter_t& APF_parameter);
    ~APFCollisionAvoidance()=default;

     matrix::Vector<double,3> calculate_repulsive_effect(
                                                        const matrix::Vector<double,11>& state,
                                                        const TsMessage_t other_information1,
                                                        const TsMessage_t other_information2,
                                                        const TsMessage_t other_information3,
                                                        const TsMessage_t other_information4);



    private:
    APFParameter_t m_APF_parameter;
    matrix::Vector<double,3> m_yawrate_ground_acc_ground_speed_APF{};


};


#endif