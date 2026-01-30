#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <matrix/math.hpp>
#include <Fixed_wing_aircraft_model/FixedwingSpec.h>
#include <controller/GainParameter.h>
#include <controller/NdiParameter.h>
#include <controller/PitchSetpoint.h>
#include <controller/RollratePitchrateSetpoint.h>
#include <controller/FlightEnvelope.h>
#include <controller/BodyAxisRateSetpoint.h>
#include <controller/ThrustSetpoint.h>



class MainController
{ 
    public:
        MainController(const FixedwingSpec_t& spec, const GainParameter_t& gain, const NdiParameter_t& ndi);
        ~MainController()=default;

        matrix::Vector<double, 4> calculate_control_input(
            const matrix::Vector<double, 11>& state,
            const matrix::Vector<double, 11>& setpoint,
            const matrix::Vector<double,3>& guidance_setpoint
            );

         matrix::Vector<double, 4> get_control_input(){return m_output_maincontroller;};

         matrix::Vector<double, 3> get_debug_rollpitchyawrate(){return m_cached_rollpitchyawratesetpoint;};
         matrix::Vector<double, 4> get_debug_flightenvelope(){return  m_cached_flightenvelope;};
         matrix::Vector<double ,3> get_debug_bodyaxisrate(){return m_cached_bodyaxisratesetpoint;};

         double get_stall_speed() { return m_flightenvelope.get_stall_speed(); }






       

    private:
        /*파라미터 내장*/
        FixedwingSpec_t m_spec;
        GainParameter_t m_gain;
        NdiParameter_t  m_ndi;

        /* 각종 서브 컨트롤러 내장*/
        PitchSetPoint m_pitchsetpoint;
        RollratePitchrateSetpoint m_rollratepitchratesetpoint;
        FlightEnvelope m_flightenvelope;
        BodyAxisRateSetpoint m_bodyaxisratesetpoint;
        ThrustSetpoint m_thrustsetpoint;

        /* 출력 내용 관련 선언*/
        matrix::Vector<double, 4> m_output_maincontroller;

        /*디버그용*/
        matrix::Vector<double, 3> m_cached_rollpitchyawratesetpoint{};
        matrix::Vector<double,4> m_cached_flightenvelope{};
        matrix::Vector<double ,3> m_cached_bodyaxisratesetpoint{};

    

    
    
    };







#endif