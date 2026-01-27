#ifndef THRUSTSTPOINT_H
#define THRUSTSTPOINT_H
#include<Fixed_wing_aircraft_model/FixedwingSpec.h>
#include<controller/GainParameter.h>
#include<matrix/math.hpp>



class ThrustSetpoint
{
    public :
        ThrustSetpoint(const FixedwingSpec_t& spec, const GainParameter_t& gain);
        ~ThrustSetpoint() = default;

        double calculate_at_sp(const matrix::Vector<double,11>& state ,  const double& stall_speed , const double& dot_ground_speed_setpoint );

        double get_at_sp(){return m_at_sp;};
        

        


    private:
        FixedwingSpec_t m_spec;
        GainParameter_t m_gain;
        double m_at_sp;



};










#endif