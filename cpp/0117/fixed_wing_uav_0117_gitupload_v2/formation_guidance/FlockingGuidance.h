#ifndef FLOCKINGGUIDANCE_H
#define FLOCKINGGUIDANCE_H

#include <formation_guidance/FlockingParameter.h>
#include <fixed_wing_aircraft_model/FixedwingSpec.h>
#include <ts_message/TsMessage.h>
#include <matrix/math.hpp>
#include <ts_message/StateInformation.h>

class FlockingGuidance
{

    public:
        FlockingGuidance(const FixedwingSpec_t& spec ,const Flockingparameter_t& flocking_gain );
        ~FlockingGuidance()=default;

        matrix::Vector<double,3> caclulate_roll_yawrate_ground_acc( const matrix::Vector<double,11>& state,
                                                                    const TsMessage_t other_information1,
                                                                    const TsMessage_t other_information2,
                                                                    const TsMessage_t other_information3,
                                                                    const TsMessage_t other_information4);

        double get_roll_setpoint(){return m_roll_yawrate_ground_acc(0);};
        double get_yawrate_setpoint(){return m_roll_yawrate_ground_acc(1);};
        double get_ground_acc_setpoint(){return m_roll_yawrate_ground_acc(2);};



        







    private:
        FixedwingSpec_t  m_spec;
        Flockingparameter_t m_flocking_gain;
        matrix::Vector<double,3> m_roll_yawrate_ground_acc{};





    
};


#endif