#ifndef ROLLRATEPITCHRATESETPOINT_H
#define ROLLRATEPITCHRATESETPOINT_H

#include <controller/GainParameter.h>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h> 
#include <matrix/math.hpp>

class RollratePitchrateSetpoint
/* RollratePitchrateSetpoint */
{
    public:
       RollratePitchrateSetpoint(const GainParameter_t& gain);
       ~RollratePitchrateSetpoint()=default;
       matrix::Vector<double,2> calculate_rollrate_pitchrate(const matrix::Vector<double,11>& state, 
                                                             const matrix::Vector<double,2>& roll_pitch_sp ,
                                                             const double& fev_pitch,
                                                             const double& gradient_fev_pitch );

       double get_rollrate_sp(){return m_output_rate_of_rollpitch(0); };
       double get_pitchrate_sp(){return m_output_rate_of_rollpitch(1); };







    private:
    GainParameter_t m_gain ;
    matrix::Vector<double,2> m_output_rate_of_rollpitch{};







};








#endif