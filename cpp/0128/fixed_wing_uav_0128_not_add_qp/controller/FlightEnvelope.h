#ifndef FLIGHTENVELOPE_H
#define FLIGHTENVELOPE_H
#include <matrix/math.hpp>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h>


class FlightEnvelope
{
    public :
        FlightEnvelope(const FixedwingSpec_t& spec);
        ~FlightEnvelope()=default;

        matrix::Vector<double,4> apply_FlightEnvelope(const matrix::Vector<double,11>& state,
                                                      const double& roll_setpoint,
                                                      const double& pitch_setpoint_before_fev);

        double get_stall_speed(){return m_output_flightenvelope(0);};
        double get_pitch_sp_after_fev(){return m_output_flightenvelope(1);};
        double get_fev_pitch(){return m_output_flightenvelope(2);};
        double get_gradient_envelope_pitch(){return m_output_flightenvelope(3);};

        



    private :

        FixedwingSpec_t m_spec;
        matrix::Vector<double,4> m_output_flightenvelope{};

};


#endif