#ifndef BODYAXISRATESETPOINT_H
#define BODYAXISRATESETPOINT_H
#include<Fixed_wing_aircraft_model/FixedwingSpec.h>
#include<matrix/math.hpp>


class BodyAxisRateSetpoint
{ 
    public:

        BodyAxisRateSetpoint(const FixedwingSpec_t& spec );
        ~BodyAxisRateSetpoint()=default;
        matrix::Vector<double,3> calculate_bodyaxisrate(const matrix::Vector<double,11>& state,const matrix::Vector<double,3>& rollrate_pitchrate_yawrate );
        double get_p_setpoint(){return m_output_bodyaxisrate(0);};
        double get_q_setpoint(){return m_output_bodyaxisrate(1);};
        double get_r_setpoint(){return m_output_bodyaxisrate(2);};


    
    private:

        FixedwingSpec_t m_spec;
        matrix::Vector<double,3> m_output_bodyaxisrate{};



};

#endif