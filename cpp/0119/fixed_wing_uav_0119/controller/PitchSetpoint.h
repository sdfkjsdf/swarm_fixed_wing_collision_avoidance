#ifndef PITCHSETPOINT_H
#define PITCHSETPOINT_H

#include <matrix/math.hpp>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h>
#include <controller/GainParameter.h>

class PitchSetPoint
{
public:
    PitchSetPoint(const FixedwingSpec_t& spec, const GainParameter_t& gain);
    ~PitchSetPoint() = default;
    
    double calculate_pitch_sp(const matrix::Vector<double, 11>& state, const matrix::Vector<double, 11>& setpoint);
    
    double get_pitch_set() const { return m_pitch_setpoint; } 
    /*get_pitch_set() 의 출력값을 나중에 flight_envelope에 입력으로 주면 됨 */
    
  

private:

    FixedwingSpec_t m_spec;
    GainParameter_t m_gain;
    double m_pitch_setpoint = 0.0;
    
};

#endif 