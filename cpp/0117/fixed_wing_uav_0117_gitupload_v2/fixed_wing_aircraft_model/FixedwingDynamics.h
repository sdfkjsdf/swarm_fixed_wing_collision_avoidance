#ifndef FIXEDWINGDYNAMICS_H 
#define FIXEDWINGDYNAMICS_H

#include <Fixed_wing_aircraft_model/FixedwingSpec.h> 
#include <matrix/math.hpp>




class FixedwingDynamics

{
    public :
        FixedwingDynamics(const FixedwingSpec_t& spec);
        ~FixedwingDynamics()=default;

         matrix::Vector<double, 11 > calcuate_dx_dt(matrix::Vector<double, 11 > state , matrix::Vector<double,4 > control_input );
         matrix::Vector<double, 11 > get_dx_dt() {return dx_dt ; }

    private:
        FixedwingSpec_t m_spec;
        matrix::Vector<double, 11 > dx_dt{} ;
        matrix::Vector<double, 11 > f_x{} ;
        matrix::Matrix<double,11, 4 > g_x{} ;
        


    


};

#endif //
