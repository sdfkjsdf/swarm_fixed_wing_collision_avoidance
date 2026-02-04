#ifndef WINGMANPOSITION_H
#define WINGMANPOSITION_H
#include <matrix/math.hpp>
#include <cmath>

class WingManPostion

{

  public:
        WingManPostion()=default;
        ~WingManPostion()=default;

        matrix::Vector<double,2> calculate_wingman_setposition(const double wingman_distance,
                                                               const matrix::Vector<double,2> wingman_parallel,
                                                               const double angle,
                                                               const double ratio)
        {  

            m_desired_position(0)=ratio * wingman_distance * std::cos(angle);
            m_desired_position(1)=ratio * wingman_distance * std::sin(angle);

            return  m_desired_position;



        };

        


 private:

        matrix::Vector<double,2> m_desired_position{};




};










#endif