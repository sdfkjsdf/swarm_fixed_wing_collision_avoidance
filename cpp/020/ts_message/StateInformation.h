#ifndef STATEINFORMATION_H
#define STATEINFORMATION_H
#include <matrix/math.hpp>
#include <ts_message/TsMessage.h>

class StateInformation{
  public:
 

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




  
};



#endif