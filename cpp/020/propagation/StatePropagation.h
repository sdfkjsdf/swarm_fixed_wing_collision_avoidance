#ifndef PROPAGATION_H
#define PROPAGATION_H

#include <matrix/math.hpp>
#include <matrix/integration.hpp>
#include <Fixed_wing_aircraft_model/FixedwingDynamics.h>


class ProPagation 

{
    public:

        ProPagation()=default;
        ~ProPagation()=default;

        matrix::Vector<double, 11> step( FixedwingDynamics& model, const matrix::Vector<double, 11>& x, const matrix::Vector<double, 4>& u,double t, double dt) 
        {
            return matrix::integrate_rk4( [&model](double t, const matrix::Vector<double, 11>& state, const matrix::Vector<double, 4>& input) 
            {
                     return model.calcuate_dx_dt(state, input);
            },
        
        x, u, t, dt

        );
    }

};




#endif//